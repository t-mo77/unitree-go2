"""マス目つきの PLANT（設計書 §6.4、R1・R3〜R8・R25、P4）：物の命題を含む式だけに使う。

マス目は anchor（合成を始めるときの Go2 の位置と向き）に合わせる。anchor がマス (0, 0) の中心で、
向き 0 が anchor の yaw。マス (i, j) の中心は (x0, y0) + R(yaw0)·(0.5 i, 0.5 j)、向き h は yaw0 + h·90°。
距離・角度はすべてマス目の座標（local）で測るので、anchor を回しても動かしても同じマスになる。
状態は (マス, 向き, 姿勢, 前の物, 後ろの物, 周回の進み)。前後の物のビットは立っているときだけ持つ（R1）。
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field, replace

from go2_nl_control.actions import ACTIONS, STANDING

from voice_ltl.automata import World
from voice_ltl.objects import (
    MAX_OBJECT_DISTANCE, MSG_TOO_SPREAD, Obj, Pose, msg_not_found, msg_too_far, split_prop, wrap_rad,
)
from voice_ltl.world import (
    ACTION_NAMES, ACTION_PROP, POSTURE_PROP, STEP_DISTANCE, UNCONTROLLABLE, UNSAFE_LABELS, PlantState,
    event_name, go2_sensor_events,
)

CELL = STEP_DISTANCE                     # 0.5 [m]
MAX_CELLS = 256                          # the rectangle's area [cells] (R8)
BLOCK_MARGIN = 0.45                      # blocked: d <= r + 0.45 (R5)
NEAR_MIN_MARGIN = 0.75                   # near: r + 0.75 <= d <= min(r + 1.5, 1.4), heading within 40° (R4)
NEAR_MAX_MARGIN = 1.5
NEAR_MAX_DISTANCE = 1.4
NEAR_MAX_ANGLE_DEG = 40.0
RING_MIN_MARGIN = 0.75                   # lap ring: r + 0.75 <= d <= r + 1.75 (R6)
RING_MAX_MARGIN = 1.75
LAP_DONE = 5                             # progress at which around_<id> holds (P4)
SAFETY_PROPS = ("obstacle_front", "obstacle_back", "moved_forward", "moved_backward")
HEADINGS = ((1, 0), (0, 1), (-1, 0), (0, -1))     # h = 0..3 in the grid frame, counter-clockwise
EPS = 1e-9                               # [m] on every distance test, so that two builds never differ on a boundary

_START = (0, 0)
_NEAR_ANGLE = math.radians(NEAR_MAX_ANGLE_DEG) + 1e-9
_MOVES = {"forward": 1, "backward": -1}
_TURNS = {"turn_left": 1, "turn_right": -1}


@dataclass(frozen=True)
class GridState:
    cell: tuple[int, int]
    heading: int
    posture: str
    front: bool = False
    back: bool = False
    laps: tuple[int, ...] = ()             # progress 0..LAP_DONE per GridWorld.lap_props


@dataclass(frozen=True)
class Snap:
    cell: tuple[int, int]
    heading: int
    offset: float                          # [m] from that cell's centre
    yaw_error: float                       # [rad] absolute, from that heading's yaw


@dataclass(frozen=True)
class Grid:
    anchor: Pose                           # odom pose of the centre of cell (0, 0), heading 0
    i_range: tuple[int, int]               # inclusive
    j_range: tuple[int, int]
    blocked: frozenset[tuple[int, int]]

    def inside(self, cell) -> bool:
        return (self.i_range[0] <= cell[0] <= self.i_range[1]
                and self.j_range[0] <= cell[1] <= self.j_range[1])

    def center(self, cell) -> tuple[float, float]:
        """マスの中心（odom）。"""
        x0, y0, yaw0 = self.anchor
        u, v = CELL * cell[0], CELL * cell[1]
        c, s = math.cos(yaw0), math.sin(yaw0)
        return x0 + c * u - s * v, y0 + s * u + c * v

    def heading_yaw(self, heading: int) -> float:
        """向き h の yaw（odom）、(−π, π] に折り返す。"""
        return wrap_rad(self.anchor[2] + heading * math.pi / 2)

    def local(self, x: float, y: float) -> tuple[float, float]:
        """odom の点をマス目の座標 [m] に：R(−yaw0)·((x, y) − (x0, y0))。"""
        x0, y0, yaw0 = self.anchor
        dx, dy = x - x0, y - y0
        c, s = math.cos(yaw0), math.sin(yaw0)
        return c * dx + s * dy, -s * dx + c * dy

    def snap(self, pose: Pose) -> Snap:
        """いちばん近いマスの中心と向き。yaw は折り返さなくてよい（DirectSimLink の yaw は 2π を超える）。"""
        lx, ly = self.local(pose[0], pose[1])
        cell = (math.floor(lx / CELL + 0.5), math.floor(ly / CELL + 0.5))
        heading = math.floor(wrap_rad(pose[2] - self.anchor[2]) / (math.pi / 2) + 0.5) % 4
        offset = math.hypot(lx - CELL * cell[0], ly - CELL * cell[1])
        yaw_error = abs(wrap_rad(pose[2] - self.heading_yaw(heading)))
        # rounded to 1e-9, so that the same pose with yaw + 2πk gives an equal Snap
        return Snap(cell, heading, round(offset, 9), round(yaw_error, 9))

    def to_json(self, robot: tuple[int, int, int] | None, goal: tuple[int, int, int] | None) -> dict:
        """Session.snapshot() の "grid"（R27）。"""
        x0, y0, yaw0 = self.anchor
        return {"cell": CELL, "origin": [float(x0), float(y0)], "yaw": float(yaw0),
                "i": list(self.i_range), "j": list(self.j_range),
                "blocked": [list(c) for c in sorted(self.blocked)],
                "robot": None if robot is None else list(robot),
                "goal": None if goal is None else list(goal)}


class GridError(Exception):
    """マス目を作れない（物が見つからない・遠すぎる・離れすぎている）。str(exc) が利用者への文言。"""


@dataclass
class GridWorld:
    world: World = field(init=False)       # the PLANT; grid_world sets it after the BFS, which needs step()
    grid: Grid
    objects: tuple[Obj, ...]               # the Scene's objects, odom
    atoms: frozenset[str]
    lap_props: tuple[str, ...]             # sorted around_*/around_cw_* atoms
    near: dict[str, frozenset[tuple[tuple[int, int], int]]]  # near prop -> {(cell, heading)}
    ring: dict[str, dict[tuple[int, int], int]] = field(default_factory=dict, repr=False)  # lap prop -> {cell: sector}
    start_covered: bool = False            # an object covers the start cell: it stays free, but no move enters it

    def __post_init__(self) -> None:
        self._actions = tuple(a for a in ACTION_NAMES if a != "backward" or "moved_backward" in self.atoms)
        self._laps_of = tuple((self.ring.get(p, {}), p.startswith("around_cw_")) for p in self.lap_props)

    def label(self, state: GridState, action: str | None) -> frozenset[str]:
        """R25：式に出る命題と安全に要る命題だけ。"""
        out = set()
        posture = POSTURE_PROP[state.posture]
        if posture in self.atoms:
            out.add(posture)
        prop = ACTION_PROP[action] if action is not None else None
        if prop and (prop in self.atoms or prop in SAFETY_PROPS):
            out.add(prop)
        if state.front:
            out.add("obstacle_front")
        if state.back:
            out.add("obstacle_back")
        key = (state.cell, state.heading)
        out.update(p for p, cells in self.near.items() if key in cells)
        out.update(p for p, progress in zip(self.lap_props, state.laps) if progress == LAP_DONE)
        return frozenset(out)

    def step(self, state: GridState, action: str) -> tuple[str, GridState] | None:
        """イベント名と行き先。ここでできない動作なら None。"""
        dst = self._dst(state, action)
        return None if dst is None else (self.event(dst, action), dst)

    def event(self, dst: GridState, action: str) -> str:
        return event_name(action, self.label(dst, action))

    def observe(self, src: GridState, action: str, cell: tuple[int, int], heading: int) -> GridState | None:
        """src から action をして、実際には (cell, heading) にいたときの状態（R10）。
        できない動作、入れないマス、0〜3 でない向きなら None。"""
        if heading not in range(4):
            return None
        dst = self._dst(src, action)
        if dst is None or not self._enterable(src.cell, cell):
            return None
        return GridState(cell, heading, dst.posture, dst.front, dst.back, self._laps(src, cell))

    # ---- 内部 ----

    def _dst(self, state: GridState, action: str) -> GridState | None:
        if action not in self._actions:
            return None
        posture = ACTIONS[action].transitions.get(state.posture)
        if posture is None or posture not in POSTURE_PROP:
            return None
        cell, heading, laps = state.cell, state.heading, state.laps
        if action in _MOVES:
            di, dj = HEADINGS[heading]
            sign = _MOVES[action]
            cell = (cell[0] + sign * di, cell[1] + sign * dj)
            if not self._enterable(state.cell, cell):
                return None
            laps = self._laps(state, cell)
        elif action in _TURNS:
            heading = (heading + _TURNS[action]) % 4
        keep = posture == STANDING and state.posture == STANDING     # R1
        return GridState(cell, heading, posture, keep and state.front, keep and state.back, laps)

    def _enterable(self, src_cell: tuple[int, int], cell: tuple[int, int]) -> bool:
        if not self.grid.inside(cell) or cell in self.grid.blocked:
            return False
        return not (self.start_covered and cell == _START and src_cell != _START)

    def _laps(self, src: GridState, cell: tuple[int, int]) -> tuple[int, ...]:
        return tuple(next_lap(p, ring.get(src.cell), ring.get(cell), cw)
                     for p, (ring, cw) in zip(src.laps, self._laps_of))


def sector(obj_local: tuple[float, float], cell_local: tuple[float, float]) -> int:
    """物の中心からマスの中心への向き：0 E [−45°, 45°)、1 N、2 W、3 S（R6）。

    角度は 1e-6 度に丸めてから分けるので、格子上の物の斜めのマス（ちょうど 45°）は
    いつも左回りで次の区画（N, W, S, E）に入る。
    """
    angle = round(math.degrees(math.atan2(cell_local[1] - obj_local[1], cell_local[0] - obj_local[0])), 6)
    if angle < -45.0:
        angle += 360.0
    return int((angle + 45.0) // 90.0)


def next_lap(progress: int, a: int | None, b: int | None, cw: bool = False) -> int:
    """1 回の移動での周回の進み。a, b は移動の前後のマスの区画（輪の外なら None）。"""
    if progress == LAP_DONE:
        return LAP_DONE
    if a is None or b is None:
        return 0
    if b == a:
        return progress
    if b == (a + 1) % 4:
        return 0 if cw else progress + 1
    if b == (a - 1) % 4:
        return progress + 1 if cw else 0
    return 0


def grid_world(posture: str, front: bool, back: bool, anchor: Pose,
               objects: Sequence[Obj], atoms: frozenset[str]) -> GridWorld:
    """式の物の命題に合わせたマス目と、その上の PLANT（到達できる状態だけ）。

    物が見つからない・6 m より遠い・長方形が MAX_CELLS を超えるときは GridError。
    同じ id の行が複数あれば、その物は最初の行（objects.parse_rows と同じ）。どの行もマスをふさぐ（R7）。
    """
    atoms = frozenset(atoms)
    objects = tuple(objects)
    by_id: dict[str, Obj] = {}
    for o in objects:
        by_id.setdefault(o.id, o)
    uses: dict[str, set[str]] = {}                       # referenced id -> {"near", "around", "around_cw"}
    for prop in sorted(atoms):
        parts = split_prop(prop)
        if parts is not None:
            uses.setdefault(parts[1], set()).add(parts[0])
    for obj_id in sorted(uses):
        if obj_id not in by_id:
            raise GridError(msg_not_found(obj_id.rsplit("_", 1)[0]))
    x0, y0 = anchor[0], anchor[1]
    for obj_id in sorted(uses):
        o = by_id[obj_id]
        if math.hypot(o.x - x0, o.y - y0) > MAX_OBJECT_DISTANCE + EPS:
            raise GridError(msg_too_far(o.label))

    probe = Grid(anchor, (0, 0), (0, 0), frozenset())
    local = [(o, probe.local(o.x, o.y)) for o in objects]   # every row blocks cells (R7)
    where = {obj_id: probe.local(o.x, o.y) for obj_id, o in by_id.items()}   # the same row as by_id
    i_lo = i_hi = j_lo = j_hi = 0                        # the robot's cell
    for obj_id, kinds in uses.items():
        r = by_id[obj_id].radius
        band = max(min(r + NEAR_MAX_MARGIN, NEAR_MAX_DISTANCE) if k == "near" else r + RING_MAX_MARGIN
                   for k in kinds)
        gx, gy = where[obj_id]
        i_lo, i_hi = min(i_lo, _ceil_cells(gx - band)), max(i_hi, _floor_cells(gx + band))
        j_lo, j_hi = min(j_lo, _ceil_cells(gy - band)), max(j_hi, _floor_cells(gy + band))
    i_range, j_range = (i_lo - 1, i_hi + 1), (j_lo - 1, j_hi + 1)
    if (i_range[1] - i_range[0] + 1) * (j_range[1] - j_range[0] + 1) > MAX_CELLS:
        raise GridError(MSG_TOO_SPREAD)

    cells = [(i, j) for i in range(i_range[0], i_range[1] + 1) for j in range(j_range[0], j_range[1] + 1)]
    covered = {c for c in cells if any(_dist(g, c) <= o.radius + BLOCK_MARGIN + EPS for o, g in local)}
    blocked = frozenset(covered - {_START})               # R7: the start cell is never blocked
    free = [c for c in cells if c not in covered]
    grid = Grid(anchor, i_range, j_range, blocked)

    near: dict[str, frozenset[tuple[tuple[int, int], int]]] = {}
    ring: dict[str, dict[tuple[int, int], int]] = {}
    for obj_id, kinds in sorted(uses.items()):
        o, g = by_id[obj_id], where[obj_id]
        r = o.radius
        if "near" in kinds:
            near[f"near_{obj_id}"] = frozenset(
                (c, h) for c in free
                if _in_band(_dist(g, c), r + NEAR_MIN_MARGIN, min(r + NEAR_MAX_MARGIN, NEAR_MAX_DISTANCE))
                for h in range(4) if abs(wrap_rad(_bearing(c, g) - h * math.pi / 2)) <= _NEAR_ANGLE)
        laps = [f"{kind}_{obj_id}" for kind in ("around", "around_cw") if kind in kinds]
        if laps:
            sectors = {c: sector(g, (CELL * c[0], CELL * c[1])) for c in free
                       if _in_band(_dist(g, c), r + RING_MIN_MARGIN, r + RING_MAX_MARGIN)}
            ring.update((p, sectors) for p in laps)
    lap_props = tuple(sorted(ring))

    gw = GridWorld(grid, objects, atoms, lap_props, near, ring, _START in covered)
    on_feet = posture == STANDING
    initial = GridState(_START, 0, posture, front and on_feet, back and on_feet, (0,) * len(lap_props))
    gw.world = _explore(gw, initial)
    return gw


# ---- 内部 ----

def _explore(gw: GridWorld, initial: GridState) -> World:
    """initial から到達できる状態だけを BFS で作る。状態は辞書で引く。"""
    states = [initial]
    index = {initial: 0}
    transitions = []
    event_label: dict[str, frozenset[str]] = {}
    i = 0
    while i < len(states):
        src = states[i]
        i += 1
        outgoing = []
        for action in gw._actions:
            dst = gw._dst(src, action)
            if dst is not None:
                label = gw.label(dst, action)
                ev = event_name(action, label)
                event_label[ev] = label
                outgoing.append((src, ev, dst, "c"))
        if src.posture == STANDING:                     # R1: the bits toggle as in go2_world
            for ev, ps in go2_sensor_events(PlantState(src.posture, src.front, src.back)):
                outgoing.append((src, ev, replace(src, front=ps.front, back=ps.back), "u"))
        else:                                           # lying or sitting: no bits, 4 self-loops
            outgoing += [(src, ev, src, "u") for ev in UNCONTROLLABLE]
        for _, _, dst, _ in outgoing:
            if dst not in index:
                index[dst] = len(states)
                states.append(dst)
        transitions += outgoing
    return World(
        states=states,
        initial=initial,
        transitions=transitions,
        marked=list(states),
        event_label=event_label,
        initial_label=gw.label(initial, None),
        propositions=tuple(sorted(gw.atoms | set(SAFETY_PROPS))),
        forbidden=frozenset(ev for ev, label in event_label.items()
                            if any(unsafe <= label for unsafe in UNSAFE_LABELS)),
    )


def _dist(g: tuple[float, float], cell: tuple[int, int]) -> float:
    return math.hypot(g[0] - CELL * cell[0], g[1] - CELL * cell[1])


def _bearing(cell: tuple[int, int], g: tuple[float, float]) -> float:
    """マスの中心から物の中心への向き（マス目の座標）。"""
    return math.atan2(g[1] - CELL * cell[1], g[0] - CELL * cell[0])


def _in_band(d: float, lo: float, hi: float) -> bool:
    return lo - EPS <= d <= hi + EPS


def _ceil_cells(v: float) -> int:
    return math.ceil(round(v / CELL, 9))


def _floor_cells(v: float) -> int:
    return math.floor(round(v / CELL, 9))
