"""grid_world and grid_shape: the grid PLANT for formulas with object propositions (design §6.4, R1, R3-R8, R25, P4),
with the blocked cells and near band B measured from the footprint (open-objects spec §10)."""

import dataclasses
import math
import random
import time
from collections import deque

import numpy as np
import pytest
from go2_nl_control.actions import LYING, STANDING

from voice_ltl import grid, objects, vocabulary
from voice_ltl.automata import World
from voice_ltl.grid import (
    BLOCK_MARGIN, CELL, LAP_DONE, LAP_TOO_BIG_RADIUS, NEAR_MAX_ANGLE_DEG, NEAR_MAX_CORE, NEAR_MAX_DISTANCE,
    NEAR_MAX_MARGIN, NEAR_MIN_EDGE, NEAR_MIN_MARGIN, NEAR_SMALL_MAX_CORE, NEAR_SMALL_MIN_CENTRE, NEAR_SMALL_SIDE, Grid,
    GridError, GridShape, GridState, GridWorld, core_limit, grid_shape, grid_world, is_small, next_lap, sector,
)
from voice_ltl.objects import (
    DEFAULT_LAYOUT, MSG_TOO_SPREAD, Obj, core_distance, edge_distance, footprint_angle, formula_atoms, msg_lap_too_big,
    msg_too_big_here, spine_angle, spine_point, winding_number, wrap_rad,
)

LAP = frozenset({"around_box_1"})
NEAR = frozenset({"near_chair_1"})
ANGLE = math.radians(40.0) + 1e-9


def gw_of(atoms, anchor=(0.0, 0.0, 0.0), objects=DEFAULT_LAYOUT, posture=LYING):
    return grid_world(posture, False, False, anchor, objects, frozenset(atoms))


def bfs_cells(world, prop):
    """BFS over controllable transitions (descending event names, as the planner breaks ties) to the
    first event whose label has `prop`. Returns the odom centres of the visited cells and the midpoints
    between them (the anchor of these worlds is the odom origin, so a centre is CELL * (i, j))."""
    out = {}
    for src, ev, dst, kind in world.transitions:
        if kind == "c":
            out.setdefault(src, []).append((ev, dst))
    prev = {world.initial: None}
    queue = deque([world.initial])
    while queue:
        src = queue.popleft()
        for ev, dst in sorted(out.get(src, ()), key=lambda t: t[0], reverse=True):
            if prop in world.event_label[ev]:
                states = [dst, src]
                while prev[states[-1]] is not None:
                    states.append(prev[states[-1]])
                cells = []
                for s in reversed(states):
                    if not cells or cells[-1] != s.cell:
                        cells.append(s.cell)
                points = []
                for a, b in zip(cells, cells[1:]):
                    points += [(CELL * a[0], CELL * a[1]), (CELL * (a[0] + b[0]) / 2, CELL * (a[1] + b[1]) / 2)]
                return points + [(CELL * cells[-1][0], CELL * cells[-1][1])]
            if dst not in prev:
                prev[dst] = src
                queue.append(dst)
    raise AssertionError(f"no path to {prop}")


def segment_distance(p, a, b):
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def segment_footprint_distance(a, b, o):
    """Least distance from the segment a-b to o's footprint, when they do not meet: two disjoint convex sets are
    closest at a corner of one of them (a move between free cells is 0.5 m long and both ends are > 0.45 m away)."""
    corners = [(o.x + sx * o.size[0] / 2, o.y + sy * o.size[1] / 2) for sx in (-1, 1) for sy in (-1, 1)]
    return min([edge_distance(*a, o), edge_distance(*b, o)] + [segment_distance(c, a, b) for c in corners])


def centre_angle(x, y, yaw, o):
    return abs(wrap_rad(math.atan2(o.y - y, o.x - x) - yaw))


def band_a(o, x, y, yaw):
    """Spec §10.4 band A (today's R4): r + 0.75 <= d_c <= min(r + 1.5, 1.3), the centre within 40°."""
    r, d_c = o.radius, math.hypot(x - o.x, y - o.y)
    return (r + NEAR_MIN_MARGIN - 1e-9 <= d_c <= min(r + NEAR_MAX_MARGIN, NEAR_MAX_DISTANCE) + 1e-9
            and centre_angle(x, y, yaw, o) <= ANGLE)


def band_b(o, x, y, yaw):
    """Spec §10.4 band B: d_e >= 0.65, d_k <= the core limit, small => d_c >= 1.05, the nearest spine point within 40°."""
    return (edge_distance(x, y, o) >= NEAR_MIN_EDGE - 1e-9 and core_distance(x, y, o) <= core_limit(o) + 1e-9
            and (not is_small(o) or math.hypot(x - o.x, y - o.y) >= NEAR_SMALL_MIN_CENTRE - 1e-9)
            and spine_angle(x, y, yaw, o) <= ANGLE)


def band_keys(g, o, cells):
    """The near keys of o by the spec's words, over any cells (not only the rectangle): free and A or B."""
    keys = set()
    for c in cells:
        x, y = g.center(c)
        if edge_distance(x, y, o) <= BLOCK_MARGIN + 1e-9:
            continue
        keys.update((c, h) for h in range(4)
                    if band_a(o, x, y, g.heading_yaw(h)) or band_b(o, x, y, g.heading_yaw(h)))
    return keys


# ---- the near-key sweep (spec §10.7, §10.10 items 4-7) ----
# The object about 3 m ahead at grid-local (3 + ox, oy), its footprint turned by deg against the grid: size is
# odom-aligned, so the anchor's yaw is -deg (design-near §4's convention). The lattice's 90° turns and mirrors map
# every other placement into offsets 0..0.5 m and 0..45°. The label only names the object (box_1, whatever its size).
SWEEP_OFFSETS = tuple(round(0.05 * k, 2) for k in range(10))                # 0 .. 0.45 m
SWEEP_DEGREES = tuple(3.0 * k for k in range(16)) + (41.0, 44.75)          # 0 .. 45° step 3, and the worst angles found
VOCABULARY_SIZES = tuple(sorted({e.size for e in vocabulary.VOCABULARY.entries}))
STOP_ERROR = 0.15                       # [m] the executor's stop error, in 16 directions (spec §10.8)
YAW_ERROR = math.radians(3.0)           # GRID_DONE_YAW


def placed(size, ox=0.0, oy=0.0, deg=0.0, x=3.0):
    """(anchor, obj): the object at grid-local (x + ox, oy), the anchor at the odom origin with yaw -deg."""
    t = -math.radians(deg)
    lx, ly = x + ox, oy
    return (0.0, 0.0, t), Obj("box_1", "box", lx * math.cos(t) - ly * math.sin(t), lx * math.sin(t) + ly * math.cos(t),
                              tuple(size))


def sweep(size):
    for deg in SWEEP_DEGREES:
        for ox in SWEEP_OFFSETS:
            for oy in SWEEP_OFFSETS:
                yield placed(size, ox, oy, deg)


def shape_near(anchor, o, others=()):
    return grid_shape(anchor, (o, *others), frozenset({f"near_{o.id}"}))


def size_id(size):
    return f"{size[0]}x{size[1]}"


def _wrap(a):
    """objects.wrap_rad on arrays: (−π, π]."""
    a = np.mod(a, 2 * np.pi)
    return np.where(a > np.pi, a - 2 * np.pi, a)


def stop_errors(keys, size):
    """keys: rows (x, y, yaw, object x, object y). For each key and each of the 16 stop errors of 0.15 m, the footprint
    distance d_e and the worst footprint angle (objects.footprint_angle) with the yaw off by -3° or +3°."""
    a, b = size[0] / 2, size[1] / 2
    phi = np.arange(16) * (np.pi / 8)
    qx = keys[:, 0:1] + STOP_ERROR * np.cos(phi) - keys[:, 3:4]        # relative to the object's centre, (K, 16)
    qy = keys[:, 1:2] + STOP_ERROR * np.sin(phi) - keys[:, 4:5]
    d_e = np.hypot(np.maximum(np.abs(qx) - a, 0.0), np.maximum(np.abs(qy) - b, 0.0))
    to_centre = np.arctan2(-qy, -qx)
    rel = np.stack([_wrap(np.arctan2(sy * b - qy, sx * a - qx) - to_centre) for sx in (-1, 1) for sy in (-1, 1)])
    lo, hi = rel.min(axis=0), rel.max(axis=0)
    worst = np.zeros_like(d_e)
    for dyaw in (-YAW_ERROR, YAW_ERROR):
        c = _wrap(to_centre - (keys[:, 2:3] + dyaw))
        low, high = c + lo, c + hi
        angle = np.where((low <= 0.0) & (0.0 <= high), 0.0, np.minimum(np.abs(_wrap(low)), np.abs(_wrap(high))))
        worst = np.maximum(worst, np.where(d_e == 0.0, 0.0, angle))
    return d_e, worst


# ---- the default layout (spec §10.7 (a)) ----
BOX_KEYS = {((3, 2), 0), ((3, 4), 0), ((4, 1), 1), ((4, 5), 3), ((6, 1), 1), ((6, 5), 3), ((7, 2), 2), ((7, 4), 2)}
CHAIR_KEYS = {((3, -4), 0), ((3, -2), 0), ((4, -5), 1), ((4, -1), 3), ((6, -5), 1), ((6, -1), 3), ((7, -4), 2), ((7, -2), 2)}
BOX_BLOCKED = {(5 + a, 3 + b) for a in (-1, 0, 1) for b in (-1, 0, 1)}
CHAIR_BLOCKED = {(5 + a, -3 + b) for a in (-1, 0, 1) for b in (-1, 0, 1)}
BOX_RING = {cell: s for s, cells in enumerate((        # around_box_1 by sector E, N, W, S (R6), 11 cells each
    ((7, 1), (7, 2), (7, 4), (8, 1), (8, 2), (8, 3), (8, 4), (8, 5), (9, 2), (9, 3), (9, 4)),
    ((3, 6), (4, 5), (4, 6), (4, 7), (5, 6), (5, 7), (6, 5), (6, 6), (6, 7), (7, 5), (7, 6)),
    ((1, 2), (1, 3), (1, 4), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5), (3, 2), (3, 4), (3, 5)),
    ((3, 0), (3, 1), (4, -1), (4, 0), (4, 1), (5, -1), (5, 0), (6, -1), (6, 0), (6, 1), (7, 0)))) for cell in cells}
# formula -> (i_range, j_range, states, transitions, events, forbidden events), captured from grid.py before band B
DEFAULT_WORLDS = {
    "F near_chair_1": ((-1, 8), (-6, 1), 1704, 15712, 80, 4),
    "F near_box_1": ((-1, 8), (-1, 6), 1704, 15712, 80, 4),
    "F around_box_1": ((-1, 10), (-2, 8), 9984, 92528, 80, 4),
    "F(around_box_1 & X F near_chair_1)": ((-1, 10), (-6, 8), 12000, 111200, 160, 8),
    "F(around_box_1 & X F around_cw_box_1)": ((-1, 10), (-2, 8), 28416, 263552, 160, 8),
}


def test_numbers():                            # plan Global Constraints, R4-R8, P4, spec §10.10 item 1
    assert (NEAR_MIN_EDGE, NEAR_SMALL_SIDE, NEAR_SMALL_MIN_CENTRE, NEAR_SMALL_MAX_CORE, NEAR_MAX_CORE) == \
        (0.65, 0.6, 1.05, 1.075, 1.25)
    assert (BLOCK_MARGIN, NEAR_MIN_MARGIN, NEAR_MAX_MARGIN, NEAR_MAX_DISTANCE, NEAR_MAX_ANGLE_DEG) == \
        (0.45, 0.75, 1.5, 1.3, 40.0)    # band A unchanged (ruling N1: 1.4 -> 1.3 m)
    assert CELL == 0.5 and grid.MAX_CELLS == 256 and LAP_DONE == 5 and LAP_TOO_BIG_RADIUS == 1.75
    assert (grid.RING_MIN_MARGIN, grid.RING_MAX_MARGIN) == (0.75, 1.75)


def test_small_is_both_sides_at_most_0_6():    # spec §10.2, O28
    for size in ((0.45, 0.45), (0.5, 0.5), (0.523, 0.529), (0.6, 0.6), (0.59, 0.55), (0.27, 0.38)):
        o = Obj("box_1", "box", 2.0, 0.0, size)
        assert is_small(o) and core_limit(o) == NEAR_SMALL_MAX_CORE, size
    for size in ((0.65, 0.65), (0.62, 0.52), (0.52, 0.62), (1.0, 0.3), (2.0, 0.9)):
        o = Obj("box_1", "box", 2.0, 0.0, size)
        assert not is_small(o) and core_limit(o) == NEAR_MAX_CORE, size


def test_default_layout_world_is_unchanged():             # spec §10.7 (a), §10.10 item 2
    """The five default formulas keep today's rectangle, blocked cells, near keys, ring and World."""
    gw = gw_of(frozenset({"near_box_1", "near_chair_1"}))
    assert gw.near["near_box_1"] == BOX_KEYS and gw.near["near_chair_1"] == CHAIR_KEYS
    for formula, (i_range, j_range, states, transitions, events, forbidden) in DEFAULT_WORLDS.items():
        gw = gw_of(formula_atoms(formula))
        w = gw.world
        assert (gw.grid.i_range, gw.grid.j_range) == (i_range, j_range), formula
        assert gw.grid.blocked == {c for c in BOX_BLOCKED | CHAIR_BLOCKED if gw.grid.inside(c)}, formula
        assert gw.near == {p: keys for p, keys in (("near_box_1", BOX_KEYS), ("near_chair_1", CHAIR_KEYS))
                           if p in w.propositions}, formula
        assert gw.ring == {p: BOX_RING for p in ("around_box_1", "around_cw_box_1") if p in w.propositions}, formula
        assert (len(w.states), len(w.transitions), len(w.event_label), len(w.forbidden)) == \
            (states, transitions, events, forbidden), formula
        assert gw.start_covered is False and w.initial == GridState((0, 0), 0, LYING, laps=(0,) * len(gw.ring))


def test_grid_world_is_grid_shape_and_the_bfs():
    shape = grid_shape((0.0, 0.0, 0.0), DEFAULT_LAYOUT, LAP | NEAR)
    gw = gw_of(LAP | NEAR)
    assert isinstance(shape, GridShape)
    assert (shape.grid, shape.near, shape.ring, shape.lap_props, shape.start_covered) == \
        (gw.grid, gw.near, gw.ring, gw.lap_props, gw.start_covered)


def test_measured_gazebo_box_keeps_near_cells():          # spec §10.10 item 3: today 0 (the part-4 regression)
    box = Obj("box_1", "box", 2.5, 1.5, (0.523, 0.529))
    assert len(grid_shape((0, 0, 0), (box,), frozenset({"near_box_1"})).near["near_box_1"]) == 8


LEAST_KEYS = (((1.6, 0.8), 8), ((2.0, 0.9), 8), ((1.8, 0.9), 8), ((0.9, 0.4), 8), ((1.8, 0.4), 8), ((1.0, 0.3), 8),
              ((0.65, 0.65), 8), ((3.0, 3.0), 8), ((0.45, 0.45), 4), ((0.5, 0.5), 4), ((0.523, 0.529), 5))


@pytest.mark.parametrize("size,least", LEAST_KEYS, ids=[size_id(s) for s, _ in LEAST_KEYS])
def test_big_objects_get_near_cells_at_every_offset_and_rotation(size, least):   # spec §10.10 item 4, the fast subset
    count, where = min(((len(shape_near(*p).near["near_box_1"]), p) for p in sweep(size)), key=lambda t: t[0])
    assert count >= least, where


@pytest.mark.parametrize("size,placement,rule_q", [((1.8, 0.4), (0.25, 0.25, 44.75), 0),
                                                   ((1.0, 0.3), (0.025, 0.025, 40.25), 2)])
def test_big_objects_get_near_cells_where_rule_q_had_none(monkeypatch, size, placement, rule_q):   # item 4
    anchor, o = placed(size, *placement)
    assert len(shape_near(anchor, o).near["near_box_1"]) >= 8
    monkeypatch.setattr(grid, "NEAR_MIN_EDGE", 0.575)        # rule Q (spec v1): every object on the small profile
    monkeypatch.setattr(grid, "NEAR_SMALL_SIDE", math.inf)
    assert len(shape_near(anchor, o).near["near_box_1"]) == rule_q


@pytest.mark.parametrize("size", VOCABULARY_SIZES, ids=[size_id(s) for s in VOCABULARY_SIZES])
def test_every_near_key_meets_the_edge_acceptance(size):  # spec §10.8, §10.10 item 5: the same sweep, every size
    rows = []                                             # (x, y, yaw, object x, object y) per key
    for anchor, o in sweep(size):
        shape = shape_near(anchor, o)
        g = shape.grid
        for c, h in shape.near["near_box_1"]:
            x, y = g.center(c)
            yaw = g.heading_yaw(h)
            assert g.inside(c) and c not in g.blocked, (anchor, o, c)
            d_e = edge_distance(x, y, o)
            assert d_e >= NEAR_MIN_EDGE - 1e-9 or band_a(o, x, y, yaw), (anchor, o, c, h)
            assert d_e <= NEAR_MAX_CORE + 1e-9, (anchor, o, c, h)
            assert min(spine_angle(x, y, yaw, o), centre_angle(x, y, yaw, o)) <= ANGLE, (anchor, o, c, h)
            rows.append((x, y, yaw, o.x, o.y))
    d_e, angle = stop_errors(np.array(rows), size)
    assert d_e.max() <= 1.40 + 1e-9                      # the 1.5 m line keeps 0.1 m
    if min(size) >= 0.22:                                 # thinner kinds may reach 45.3° (spec §10.8)
        assert np.degrees(angle.max()) <= 45.0 + 1e-9


def test_stop_errors_match_the_footprint_helpers():      # the vectorised check of item 5 is objects' definition
    rng = random.Random(7)
    for size in ((0.45, 0.45), (1.0, 0.3), (0.27, 0.38), (2.0, 0.9)):
        rows = [(rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-4, 4), rng.uniform(-1, 1), rng.uniform(-1, 1))
                for _ in range(200)]
        d_e, angle = stop_errors(np.array(rows), size)
        for k, (x, y, yaw, ox, oy) in enumerate(rows):
            o = Obj("box_1", "box", ox, oy, size)
            for n in range(16):
                qx, qy = x + STOP_ERROR * math.cos(n * math.pi / 8), y + STOP_ERROR * math.sin(n * math.pi / 8)
                assert d_e[k, n] == pytest.approx(edge_distance(qx, qy, o), abs=1e-12)
                want = max(footprint_angle(qx, qy, yaw + dyaw, o) for dyaw in (-YAW_ERROR, YAW_ERROR))
                assert angle[k, n] == pytest.approx(want, abs=1e-12)


@pytest.mark.parametrize("size,farthest", [((0.45, 0.45), 1.3), ((0.5, 0.5), 1.325)])
def test_chair_plan_stays_within_n1_at_every_placement(size, farthest):   # spec §10.10 item 6, ruling N1
    for anchor, o in sweep(size):
        shape = shape_near(anchor, o)
        for c, h in shape.near["near_box_1"]:
            x, y = shape.grid.center(c)
            assert math.hypot(x - o.x, y - o.y) <= farthest + 1e-9, (anchor, o, c)
            assert centre_angle(x, y, shape.grid.heading_yaw(h), o) <= ANGLE, (anchor, o, c, h)


@pytest.mark.parametrize("size", [(0.5, 0.5), (0.45, 0.45), (0.523, 0.529), (0.27, 0.38)], ids=size_id)
def test_small_profile_is_rule_q(monkeypatch, size):     # spec §10.10 item 7: 0.65 never binds when r <= 0.40
    placements = list(sweep(size))
    keys = [shape_near(*p).near["near_box_1"] for p in placements]
    monkeypatch.setattr(grid, "NEAR_MIN_EDGE", 0.575)    # rule Q's band B edge
    assert [shape_near(*p).near["near_box_1"] for p in placements] == keys


def test_blocked_is_footprint_plus_margin():              # spec §10.3, §10.10 item 8 (R5, R7)
    sofa = Obj("sofa_1", "sofa", 2.6, 1.9, (2.0, 0.9))
    shape = grid_shape((0.0, 0.0, 0.65), (sofa,), frozenset({"near_sofa_1"}))
    g = shape.grid
    cells = [(i, j) for i in range(g.i_range[0], g.i_range[1] + 1) for j in range(g.j_range[0], g.j_range[1] + 1)]
    assert g.blocked == {c for c in cells if edge_distance(*g.center(c), sofa) <= BLOCK_MARGIN + 1e-9} - {(0, 0)}
    circle = {c for c in cells if math.hypot(g.center(c)[0] - sofa.x, g.center(c)[1] - sofa.y)
              <= sofa.radius + BLOCK_MARGIN}                                      # today's rule
    assert g.blocked < circle
    aligned = Obj("sofa_1", "sofa", 2.5, 0.05, (2.0, 0.9))                       # its long side at y = 0.5
    g = grid_shape((0.0, 0.0, 0.0), (aligned,), frozenset({"near_sofa_1"})).grid
    x, y = g.center((5, 2))
    assert edge_distance(x, y, aligned) == pytest.approx(0.5) and (5, 2) not in g.blocked
    assert math.hypot(x - aligned.x, y - aligned.y) <= aligned.radius + BLOCK_MARGIN   # today's circle blocks it
    assert (5, 1) in g.blocked                                                     # on the long side


CLEARANCE_WORLDS = {
    "off-lattice box and chair": ((0.0, 0.0, 0.0), (Obj("box_1", "box", 2.3, 1.2),
                                                    Obj("chair_1", "chair", 2.6, -1.35, (0.45, 0.45))), LAP | NEAR),
    "sofa and table": ((0.0, 0.0, 0.3), (Obj("sofa_1", "sofa", 3.0, 0.9, (2.0, 0.9)),
                                         Obj("table_1", "table", 2.2, -1.6, (1.6, 0.8))),
                       frozenset({"near_sofa_1", "near_table_1"})),
    "lap around the sofa past the table": ((0.0, 0.0, 0.0), (Obj("sofa_1", "sofa", 3.2, 0.3, (1.8, 0.9)),
                                                             Obj("table_1", "table", 3.4, 3.4, (1.2, 0.75))),
                                           frozenset({"around_sofa_1"})),
}


@pytest.mark.parametrize("name", CLEARANCE_WORLDS)
def test_moves_keep_the_body_clear_of_every_footprint(name):   # spec §10.3, §10.10 item 9 (R5 general, R6)
    anchor, objs, atoms = CLEARANCE_WORLDS[name]
    gw = gw_of(atoms, anchor=anchor, objects=objs)
    least = math.inf
    for src, _, dst, kind in gw.world.transitions:
        if kind == "c" and src.cell != dst.cell:
            a, b = gw.grid.center(src.cell), gw.grid.center(dst.cell)
            least = min(least, *(segment_footprint_distance(a, b, o) for o in objs))
        for p, progress in zip(gw.lap_props, dst.laps):
            if 0 < progress < LAP_DONE:                                  # progress 1-4 only on the ring
                assert dst.cell in gw.ring[p], (p, dst)
    assert least >= 0.37                                                 # the bound is 0.374 (spec §10.3)


@pytest.mark.parametrize("size", [(0.5, 0.5), (0.8, 0.8), (3.0, 3.0)], ids=size_id)
def test_spine_facing_is_the_centre_rule_for_squares(size):   # spec §10.4, §10.10 item 10
    for deg in (0.0, 17.0, 44.75):
        for ox, oy in ((0.0, 0.0), (0.2, 0.35)):
            anchor, o = placed(size, ox, oy, deg)
            shape = shape_near(anchor, o)
            assert shape.near["near_box_1"]
            for c, h in shape.near["near_box_1"]:
                x, y = shape.grid.center(c)
                assert spine_point(x, y, o) == (o.x, o.y)
                assert centre_angle(x, y, shape.grid.heading_yaw(h), o) <= ANGLE


def test_large_profile_has_no_centre_floor(monkeypatch):  # spec §10.10 item 10
    """A lattice-aligned TV 1.0 x 0.3 has a key beside the middle of each long face, 0.65-1.0 m from it and closer
    to its centre than NEAR_SMALL_MIN_CENTRE, facing the face square-on."""
    tv = Obj("television_1", "television", 2.5, 0.0, (1.0, 0.3))
    atoms = frozenset({"near_television_1"})
    shape = grid_shape((0.0, 0.0, 0.0), (tv,), atoms)
    middle = {(c, h) for c, h in shape.near["near_television_1"] if c[0] == 5}
    assert middle == {((5, 2), 3), ((5, -2), 1)}                      # facing -y from above, +y from below
    for c, _ in middle:
        x, y = shape.grid.center(c)
        assert 0.65 <= edge_distance(x, y, tv) <= 1.0 and math.hypot(x - tv.x, y - tv.y) < NEAR_SMALL_MIN_CENTRE
    monkeypatch.setattr(grid, "NEAR_SMALL_SIDE", 1.0)                   # as if the TV were small: the floor drops them
    assert not {c for c, _ in grid_shape((0.0, 0.0, 0.0), (tv,), atoms).near["near_television_1"] if c[0] == 5}


@pytest.mark.parametrize("size", [(2.0, 0.9), (3.0, 3.0)], ids=size_id)
def test_near_rectangle_contains_every_near_key(size):    # spec §10.5 (R8), §10.10 item 11
    everywhere = [(i, j) for i in range(-16, 26) for j in range(-16, 17)]
    for deg in (0.0, 10.0, 27.0, 37.0, 45.0):
        for ox, oy in ((0.0, 0.0), (0.2, 0.35), (0.45, 0.1)):
            anchor, o = placed(size, ox, oy, deg)
            shape = shape_near(anchor, o)
            keys = shape.near["near_box_1"]
            assert keys and all(shape.grid.inside(c) for c, _ in keys)
            assert keys == band_keys(shape.grid, o, everywhere), (deg, ox, oy)


def test_lap_messages():                                   # spec §10.6, §10.10 item 12
    for anchor, o in sweep((2.5, 2.5)):                    # r = 1.77: by size alone, at every placement
        with pytest.raises(GridError) as exc:
            grid_shape(anchor, (o,), LAP)
        assert str(exc.value) == msg_lap_too_big("box")
    big = Obj("bed_1", "bed", 3.0, 0.0, (3.0, 1.8))        # r = 1.749
    g = grid_shape((0.0, 0.0, 0.0), (big,), frozenset({"around_bed_1"})).grid
    assert (g.i_range[1] - g.i_range[0] + 1) * (g.j_range[1] - g.j_range[0] + 1) == 225
    bed = Obj("bed_1", "bed", 5.5, 0.25, (2.0, 1.0))
    with pytest.raises(GridError) as exc:                  # 266 cells with the Go2's cell
        grid_shape((0.0, 0.0, 0.0), (bed,), frozenset({"around_bed_1"}))
    assert str(exc.value) == msg_too_big_here("bed")
    g = grid_shape((5.0, 0.0, 0.0), (bed,), frozenset({"around_bed_1"})).grid     # 10 cells closer: its own 182
    assert (g.i_range[1] - g.i_range[0] + 1) * (g.j_range[1] - g.j_range[0] + 1) == 182
    spread = (Obj("box_1", "box", 4.0, 4.0), Obj("chair_1", "chair", -4.0, -4.0))
    with pytest.raises(GridError) as exc:
        grid_shape((0.0, 0.0, 0.0), spread, LAP | NEAR)
    assert str(exc.value) == MSG_TOO_SPREAD


@pytest.mark.parametrize("anchor,obj,prop,area,states,keys", [      # one object, alone in the Scene
    ((0.0, 0.0, 0.0), Obj("sofa_1", "sofa", 3.0, 0.3, (1.8, 0.9)), "near_sofa_1", 117, 2328, 21),
    ((0.0, 0.0, 0.0), Obj("sofa_1", "sofa", 3.0, 0.3, (2.0, 0.9)), "near_sofa_1", 117, 2328, 25),
    ((0.0, 0.0, 0.3), Obj("table_1", "table", 2.6, 1.7, (1.6, 0.8)), "near_table_1", 117, 2400, 19),
    ((0.0, 0.0, 0.0), Obj("bed_1", "bed", 3.5, -0.4, (2.0, 1.0)), "near_bed_1", 126, 2544, 20),
    ((0.0, 0.0, 0.0), Obj("refrigerator_1", "refrigerator", 2.5, -1.5, (0.65, 0.65)), "near_refrigerator_1", 99, 2160, 16),
    ((0.0, 0.0, 0.0), Obj("sofa_1", "sofa", 3.0, 0.3, (1.8, 0.9)), "around_sofa_1", 182, 13344, None),
    ((0.0, 0.0, 0.0), Obj("bed_1", "bed", 3.5, -0.4, (2.0, 1.0)), "around_bed_1", 195, 14160, None),
], ids=["sofa 1.8 near", "sofa 2.0 near", "table near", "bed near", "refrigerator near", "sofa lap", "bed lap"])
def test_big_object_worlds_have_the_spec_sizes(anchor, obj, prop, area, states, keys):   # spec §10.7 (size and time)
    gw = gw_of({prop}, anchor=anchor, objects=(obj,))
    g = gw.grid
    assert (g.i_range[1] - g.i_range[0] + 1) * (g.j_range[1] - g.j_range[0] + 1) == area
    assert len(gw.world.states) == states
    assert keys is None or len(gw.near[prop]) == keys


def test_margin_widens_the_rectangle():                    # for Task 1.7 (O15)
    s1, s3 = (grid_shape((0, 0, 0), DEFAULT_LAYOUT, frozenset({"near_chair_1"}), m) for m in (1, 3))
    assert s3.grid.i_range == (s1.grid.i_range[0] - 2, s1.grid.i_range[1] + 2) and s3.near == s1.near
    assert s3.grid.j_range == (s1.grid.j_range[0] - 2, s1.grid.j_range[1] + 2)
    gw = grid_world(LYING, False, False, (0, 0, 0), DEFAULT_LAYOUT, frozenset({"near_chair_1"}), 3)
    assert gw.grid == s3.grid


FULL_SWEEP_LEAST = {     # spec §10.7 (*): large objects >= 8 keys; small ones as measured
    (0.45, 0.45): 4, (0.5, 0.5): 4, (0.523, 0.529): 5, (0.6, 0.6): 6, (0.59, 0.55): 6, (0.65, 0.65): 8,
    (0.75, 0.75): 8, (0.8, 0.35): 8, (0.9, 0.4): 8, (0.914, 0.914): 8, (1.0, 0.3): 8, (1.2, 0.4): 8, (1.2, 0.35): 8,
    (1.2, 0.75): 8, (1.5, 0.6): 8, (1.5, 0.4): 8, (1.5, 0.8): 8, (1.6, 0.8): 8, (1.8, 0.9): 8, (1.8, 1.04): 8,
    (1.8, 0.4): 8, (2.0, 0.9): 8, (2.0, 1.0): 8, (2.0, 2.0): 8, (3.0, 0.5): 8, (3.0, 3.0): 8,
}


@pytest.mark.slow
@pytest.mark.parametrize("size", FULL_SWEEP_LEAST, ids=size_id)
def test_near_keys_full_sweep(size):                       # O32: 0.025 m / 0.25° (72 400) plus 30 000 random
    rng = random.Random(32)
    fine = [(0.025 * a, 0.025 * b, 0.25 * k) for k in range(181) for a in range(20) for b in range(20)]
    rand = [(rng.uniform(0, 0.5), rng.uniform(0, 0.5), rng.uniform(0, 45)) for _ in range(30_000)]
    for ox, oy, deg in fine + rand:
        anchor, o = placed(size, ox, oy, deg)
        shape = shape_near(anchor, o)
        keys = shape.near["near_box_1"]
        assert len(keys) >= FULL_SWEEP_LEAST[size], (ox, oy, deg)
        for c, _ in keys:
            x, y = shape.grid.center(c)
            assert NEAR_MIN_EDGE - 1e-9 <= edge_distance(x, y, o) <= NEAR_MAX_CORE + 1e-9, (ox, oy, deg, c)


def test_rectangle_follows_the_bands():                  # R8
    g = gw_of(LAP).grid;          assert (g.i_range, g.j_range) == ((-1, 10), (-2, 8))   # 12 x 11
    g = gw_of(LAP | NEAR).grid;   assert (g.i_range, g.j_range) == ((-1, 10), (-6, 8))   # 12 x 15
    g = gw_of(NEAR).grid;         assert (g.i_range, g.j_range) == ((-1, 8), (-6, 1))


def test_every_object_blocks_and_no_move_enters_a_blocked_cell():
    gw = gw_of(LAP)
    assert {(5 + a, 3 + b) for a in (-1, 0, 1) for b in (-1, 0, 1)} <= gw.grid.blocked
    assert {(4, -2), (5, -2), (6, -2)} <= gw.grid.blocked            # chair_1 blocks though not in the formula
    assert (0, 0) not in gw.grid.blocked and (3, 3) not in gw.grid.blocked
    for _, _, dst, _ in gw.world.transitions:
        assert gw.grid.inside(dst.cell) and dst.cell not in gw.grid.blocked


@pytest.mark.parametrize("anchor,objs,prop", [
    ((0.0, 0.0, 0.0), DEFAULT_LAYOUT, "near_chair_1"),
    ((0.0, 0.0, 0.3), (Obj("sofa_1", "sofa", 3.0, 0.9, (2.0, 0.9)),), "near_sofa_1"),
    ((0.0, 0.0, -0.4), (Obj("television_1", "television", 2.4, -1.3, (1.0, 0.3)),), "near_television_1"),
])
def test_near_cells_face_the_object_within_the_band(anchor, objs, prop):    # R4, spec §10.4: band A or band B
    gw = gw_of({prop}, anchor=anchor, objects=objs)
    o = next(obj for obj in objs if prop == f"near_{obj.id}")
    cells = gw.near[prop]
    assert cells
    if prop == "near_chair_1":
        assert ((3, -2), 0) in cells and ((3, -2), 1) not in cells and ((3, -3), 0) not in cells
    for cell, h in cells:
        x, y = gw.grid.center(cell)
        assert band_a(o, x, y, gw.grid.heading_yaw(h)) or band_b(o, x, y, gw.grid.heading_yaw(h)), (cell, h)


def test_the_default_layout_keeps_near_cells_for_both_objects():   # R4 with ruling N1, spec §10.4
    """Band A (out to min(r + 1.5, 1.3)) and band B of a small object (centre >= 1.05, core <= 1.075) hold the
    same eight cells 0.5·√5 m (1.118 m) from each object of the default layout, one heading each; the four cells
    on the axes 1.0 m away and the cells 1.414 m away stay out."""
    for prop, keys in (("near_box_1", BOX_KEYS), ("near_chair_1", CHAIR_KEYS)):
        gw = gw_of({prop})
        o = next(obj for obj in DEFAULT_LAYOUT if prop == f"near_{obj.id}")
        assert gw.near[prop] == keys, prop
        assert {round(math.hypot(o.x - gw.grid.center(c)[0], o.y - gw.grid.center(c)[1]), 3) for c, _ in keys} == {1.118}


@pytest.mark.parametrize("p,a,b,cw,want", [(0, 0, 1, False, 1), (3, 3, 0, False, 4), (4, 3, 0, False, 5), (2, 1, 0, False, 0),
    (2, 1, 1, False, 2), (3, None, 1, False, 0), (2, 1, None, False, 0), (5, None, None, False, 5), (4, None, 1, False, 0),
    (0, 1, 0, True, 1), (2, 0, 1, True, 0), (1, 0, 2, False, 0)])
def test_next_lap(p, a, b, cw, want):
    assert next_lap(p, a, b, cw) == want


@pytest.mark.parametrize("prop,lo,hi", [("around_box_1", 0.95, 1.5), ("around_cw_box_1", -1.5, -0.95)])
def test_shortest_lap_winds_once_and_keeps_clear(prop, lo, hi):   # P4, R5
    box = DEFAULT_LAYOUT[0]
    path = bfs_cells(gw_of(frozenset({prop})).world, prop)
    assert lo <= winding_number(path, (box.x, box.y)) <= hi
    assert min(edge_distance(x, y, box) for x, y in path) >= 0.6


def test_backward_only_when_the_formula_moves_backward():
    assert not any(e.startswith("backward_to_") for e in gw_of(NEAR).world.event_label)
    assert any(e.startswith("backward_to_") for e in gw_of(NEAR | {"moved_backward"}).world.event_label)


def test_labels_are_projected_and_safety_kept():          # R25
    w = gw_of(NEAR).world
    assert set().union(*w.event_label.values()) <= {"near_chair_1", "obstacle_front", "obstacle_back", "moved_forward"}
    assert "stand_to_" in w.event_label
    assert w.forbidden == {e for e, l in w.event_label.items() if {"obstacle_front", "moved_forward"} <= l}


def test_sensor_events_are_self_loops_unless_standing():  # R1
    for src, ev, dst, kind in gw_of(NEAR).world.transitions:
        if kind == "u" and src.posture != STANDING:
            assert dst == src
        if ev.startswith("stand_to_"):
            assert (dst.front, dst.back) == (False, False)


def test_no_sitting_or_lying_down_while_an_obstacle_is_seen():   # R1: getting up would forget it
    down = [(src, ev) for src, ev, dst, kind in gw_of(NEAR | {"sitting"}, posture=STANDING).world.transitions
            if kind == "c" and src.posture == STANDING and dst.posture != STANDING]
    assert {ev.split("_to_")[0] for _, ev in down} == {"sit", "lie_down"}
    assert all((src.front, src.back) == (False, False) for src, _ in down)


def test_too_far_too_spread_and_not_found():             # spec §10.6: in this order
    far = (Obj("box_1", "box", 6.1, 0.0),)
    with pytest.raises(GridError, match="遠すぎます"):
        gw_of(LAP, objects=far)
    gw_of(LAP, objects=(Obj("box_1", "box", 5.9, 0.0),))
    spread = (Obj("box_1", "box", 4.0, 4.0), Obj("chair_1", "chair", -4.0, -4.0))
    with pytest.raises(GridError, match="離れすぎて"):
        gw_of(LAP | NEAR, objects=spread)
    with pytest.raises(GridError, match="見つかりません"):
        gw_of(frozenset({"near_box_2"}))
    huge = (Obj("box_1", "box", 3.0, 0.0, (2.5, 2.5)),)
    with pytest.raises(GridError) as exc:
        gw_of(LAP, objects=huge)
    assert str(exc.value) == msg_lap_too_big("box") == "「箱」は大きすぎて、周りを回る計画が作れません。"
    assert gw_of(frozenset({"near_box_1"}), objects=huge).near["near_box_1"]      # near is fine
    with pytest.raises(GridError, match="遠すぎます"):                            # too far comes first
        gw_of(LAP, objects=(Obj("box_1", "box", 6.1, 0.0, (2.5, 2.5)),))
    with pytest.raises(GridError, match="見つかりません"):                        # not found comes first
        gw_of(LAP | {"near_chair_2"}, objects=huge)
    bed = (Obj("bed_1", "bed", 5.5, 0.25, (2.0, 1.0)),)
    with pytest.raises(GridError) as exc:
        gw_of(frozenset({"around_bed_1"}), objects=bed)
    assert str(exc.value) == msg_too_big_here("bed") == \
        "「ベッド」は大きいので、ここからは計画できません。近づいてから、もう一度言ってください。"
    with pytest.raises(GridError, match="離れすぎて"):                            # two objects: too spread
        gw_of(frozenset({"around_bed_1", "near_chair_1"}), objects=bed + DEFAULT_LAYOUT[1:])


@pytest.mark.parametrize("t", [0.65, math.pi / 2, -1.2, 2.9])
def test_rotated_anchor_gives_the_same_cells(t):         # Review Focus 1, spec §10.10 (the rewritten test)
    """The near keys and the ring do not depend on the anchor's angle. The footprints stay odom-aligned, so the
    blocked cells and the World are the same only at t = π/2 with each size's x and y swapped."""
    x0, y0 = 1.0, -1.0
    turned = t == math.pi / 2

    def move(o):
        return dataclasses.replace(o, x=x0 + o.x * math.cos(t) - o.y * math.sin(t),
                                   y=y0 + o.x * math.sin(t) + o.y * math.cos(t), size=o.size[::-1] if turned else o.size)

    a, b = gw_of(LAP | NEAR), gw_of(LAP | NEAR, anchor=(x0, y0, t), objects=tuple(map(move, DEFAULT_LAYOUT)))
    assert a.near == b.near and a.ring == b.ring
    if turned:
        assert a.grid == dataclasses.replace(b.grid, anchor=a.grid.anchor) and len(a.world.states) == len(b.world.states)
        sofa = Obj("sofa_1", "sofa", 3.0, 0.9, (2.0, 0.9))
        s, r = (grid_shape(anchor, (o,), frozenset({"near_sofa_1"})) for anchor, o in
                (((0.0, 0.0, 0.0), sofa), ((x0, y0, t), move(sofa))))
        assert (s.grid.blocked, s.near) == (r.grid.blocked, r.near)
    elif t == 0.65:
        assert a.grid.blocked != b.grid.blocked


def test_start_cell_is_never_blocked():
    gw = gw_of(frozenset({"near_box_1"}), objects=(Obj("box_1", "box", 0.6, 0.0),))
    assert (0, 0) not in gw.grid.blocked and gw.world.states[0].cell == (0, 0)


def test_no_move_returns_into_a_start_cell_an_object_covers():   # R7: the exemption is for the initial state only
    gw = gw_of(frozenset({"near_box_1"}), objects=(Obj("box_1", "box", 0.6, 0.0),))
    moves = [(src, dst) for src, _, dst, kind in gw.world.transitions if kind == "c" and src.cell != dst.cell]
    assert any(src.cell == (0, 0) for src, _ in moves)                 # the Go2 can leave it
    assert not any(dst.cell == (0, 0) for _, dst in moves)
    left = next(dst for src, dst in moves if src.cell == (0, 0))
    assert gw.observe(left, "turn_left", (0, 0), 0) is None
    start = next(s for s in gw.world.states if s.cell == (0, 0) and s.posture == STANDING and s.heading == 0)
    assert gw.observe(start, "turn_left", (0, 0), 1) is not None      # turning where it stands is fine


def test_snap_frame_and_json():
    g = Grid((1.0, 2.0, math.pi / 2), (-1, 3), (-1, 3), frozenset({(2, 2)}))
    assert g.center((1, 0)) == pytest.approx((1.0, 2.5)) and g.heading_yaw(1) == pytest.approx(math.pi)
    s = g.snap((1.12, 2.61, math.pi / 2 + 0.2))
    assert (s.cell, s.heading) == ((1, 0), 0) and s.offset == pytest.approx(0.1628, abs=1e-3)
    assert s.yaw_error == pytest.approx(0.2)
    assert g.snap((1.12, 2.61, math.pi / 2 + 0.2 + 4 * math.pi)) == s     # DirectSimLink's yaw is unwrapped
    assert g.to_json((1, 0, 0), None) == {"cell": 0.5, "origin": [1.0, 2.0], "yaw": math.pi / 2, "i": [-1, 3],
        "j": [-1, 3], "blocked": [[2, 2]], "robot": [1, 0, 0], "goal": None}


def test_observe_lands_where_the_robot_is():
    gw = gw_of(LAP)
    src = next(s for s in gw.world.states if s.posture == STANDING and s.cell == (2, 1) and s.heading == 1)
    obs = gw.observe(src, "forward", (2, 2), 0)
    assert (obs.cell, obs.heading, obs.posture) == ((2, 2), 0, STANDING)
    assert gw.observe(src, "forward", (5, 3), 1) is None and gw.observe(src, "forward", (99, 99), 1) is None


def test_lap_and_chair_world_builds_quickly():
    t0 = time.monotonic()
    gw = gw_of(LAP | NEAR)
    assert time.monotonic() - t0 < 2.0 and len(gw.world.states) < 30_000


def test_grid_uses_the_shared_wrap():          # one (−π, π] wrap for the whole project (objects.wrap_rad)
    assert grid.wrap_rad is objects.wrap_rad and not hasattr(grid, "_wrap")


def test_a_duplicated_id_uses_its_first_row_everywhere():   # never one row's radius with another's position
    first = Obj("box_1", "box", 2.5, 1.5, (0.5, 0.5))
    later = Obj("box_1", "box", 2.5, -3.0, (1.0, 1.0))      # far below the first row's rectangle
    one, two = gw_of({"near_box_1"}, objects=(first,)), gw_of({"near_box_1"}, objects=(first, later))
    assert (two.grid.i_range, two.grid.j_range) == (one.grid.i_range, one.grid.j_range)
    assert two.near == one.near and two.grid.blocked == one.grid.blocked and two.near["near_box_1"]
    lap_one, lap_two = gw_of(LAP, objects=(first,)), gw_of(LAP, objects=(first, later))
    assert lap_two.ring == lap_one.ring


def test_observe_refuses_a_heading_outside_0_to_3():
    gw = gw_of(LAP)
    src = next(s for s in gw.world.states if s.posture == STANDING and s.cell == (2, 1) and s.heading == 1)
    assert gw.observe(src, "forward", (2, 2), 1) is not None
    for bad in (4, -1, 5):
        assert gw.observe(src, "forward", (2, 2), bad) is None


def test_observe_takes_only_an_int_heading():   # `True in range(4)` and `1.0 in range(4)` are true
    gw = gw_of(LAP)
    src = next(s for s in gw.world.states if s.posture == STANDING and s.cell == (2, 1) and s.heading == 1)
    assert type(gw.observe(src, "forward", (2, 2), 0).heading) is int
    for bad in (True, False, 1.0, 0.0, "1", None, (1,)):
        assert gw.observe(src, "forward", (2, 2), bad) is None, repr(bad)


def test_world_is_not_a_constructor_argument():
    world = next(f for f in dataclasses.fields(GridWorld) if f.name == "world")
    assert world.init is False
    gw = gw_of(NEAR)
    assert isinstance(gw.world, World) and gw.world.initial == GridState((0, 0), 0, LYING)
    assert gw.grid.inside((0, 0)) and gw.near["near_chair_1"] and gw.lap_props == () \
        and gw.start_covered is False                                  # the public names are unchanged


def test_sector_puts_the_diagonals_in_the_next_sector_counter_clockwise():   # R6, the 1e-6° rounding
    for obj in ((0.0, 0.0), (2.5, 1.5), (-1.0, 0.5)):                 # objects on the lattice
        cells = [(obj[0] + CELL * a, obj[1] + CELL * b) for a, b in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
        assert [sector(obj, c) for c in cells] == [1, 2, 3, 0]
        sides = [(obj[0] + CELL * a, obj[1] + CELL * b) for a, b in ((1, 0), (0, 1), (-1, 0), (0, -1))]
        assert [sector(obj, c) for c in sides] == [0, 1, 2, 3]


def test_observe_updates_the_lap_from_the_observed_cell():   # R10 with R6: box_1 is cell (5, 3)
    gw = gw_of(LAP)
    ring = gw.ring["around_box_1"]
    assert (ring[(9, 3)], ring[(8, 4)], ring[(8, 5)], ring[(7, 5)]) == (0, 0, 0, 1) and (10, 3) not in ring
    outside = GridState((10, 3), 2, STANDING, laps=(3,))              # hand-made: progress 3 off the ring
    assert gw.observe(outside, "forward", (9, 3), 2).laps == (0,)     # entering the ring starts at 0
    on_ring = GridState((8, 4), 1, STANDING, laps=(2,))
    assert gw.step(on_ring, "forward")[1].laps == (2,)                # planned: (8, 5), the same sector E
    assert gw.observe(on_ring, "forward", (8, 5), 1).laps == (2,)
    assert gw.observe(on_ring, "forward", (7, 5), 1).laps == (3,)     # observed: (7, 5) in N, one sector CCW
    assert gw.observe(on_ring, "forward", (9, 4), 1).laps == (2,)     # observed back in E: no change
    assert gw.observe(GridState((7, 5), 2, STANDING, laps=(3,)), "forward", (8, 5), 2).laps == (0,)   # clockwise
    done = GridState((8, 4), 1, STANDING, laps=(LAP_DONE,))
    assert gw.observe(done, "forward", (10, 5), 1).laps == (LAP_DONE,)   # a closed lap stays closed
