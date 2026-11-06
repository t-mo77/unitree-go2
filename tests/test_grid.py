"""grid_world: the grid PLANT for formulas with object propositions (design §6.4, R1, R3-R8, R25, P4)."""

import dataclasses
import math
import time
from collections import deque

import pytest
from go2_nl_control.actions import LYING, STANDING

from voice_ltl import grid, objects
from voice_ltl.automata import World
from voice_ltl.grid import CELL, LAP_DONE, Grid, GridError, GridState, GridWorld, grid_world, next_lap, sector
from voice_ltl.objects import DEFAULT_LAYOUT, Obj, edge_distance, winding_number

LAP = frozenset({"around_box_1"})
NEAR = frozenset({"near_chair_1"})


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


def test_near_cells_face_the_object_within_the_band():    # R4
    gw, chair = gw_of(NEAR), DEFAULT_LAYOUT[1]
    cells = gw.near["near_chair_1"]
    assert ((3, -2), 0) in cells and ((3, -2), 1) not in cells and ((3, -3), 0) not in cells
    for cell, h in cells:
        x, y = gw.grid.center(cell)
        assert chair.radius + 0.75 <= math.hypot(chair.x - x, chair.y - y) <= min(chair.radius + 1.5, 1.4)
        bearing = math.atan2(chair.y - y, chair.x - x)
        assert abs(math.remainder(bearing - gw.grid.heading_yaw(h), 2 * math.pi)) <= math.radians(40) + 1e-9


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


def test_too_far_too_spread_and_not_found():
    far = (Obj("box_1", "box", 6.1, 0.0),)
    with pytest.raises(GridError, match="遠すぎます"):
        gw_of(LAP, objects=far)
    gw_of(LAP, objects=(Obj("box_1", "box", 5.9, 0.0),))
    spread = (Obj("box_1", "box", 4.0, 4.0), Obj("chair_1", "chair", -4.0, -4.0))
    with pytest.raises(GridError, match="離れすぎて"):
        gw_of(LAP | NEAR, objects=spread)
    with pytest.raises(GridError, match="見つかりません"):
        gw_of(frozenset({"near_box_2"}))


def test_rotated_anchor_gives_the_same_cells():           # Review Focus 1
    x0, y0, t = 1.0, -1.0, 0.65
    moved = tuple(dataclasses.replace(o, x=x0 + o.x * math.cos(t) - o.y * math.sin(t),
                                      y=y0 + o.x * math.sin(t) + o.y * math.cos(t)) for o in DEFAULT_LAYOUT)
    a, b = gw_of(LAP | NEAR), gw_of(LAP | NEAR, anchor=(x0, y0, t), objects=moved)
    assert a.grid.blocked == b.grid.blocked and a.near == b.near and len(a.world.states) == len(b.world.states)


def test_moves_keep_the_body_clear_of_off_lattice_objects():   # R5 general invariant, R6 reachable only
    objs = (Obj("box_1", "box", 2.3, 1.2), Obj("chair_1", "chair", 2.6, -1.35, (0.45, 0.45)))
    gw = gw_of(LAP | NEAR, objects=objs)
    box = objs[0]
    for src, _, dst, kind in gw.world.transitions:
        if kind == "c" and src.cell != dst.cell:
            a, b = gw.grid.center(src.cell), gw.grid.center(dst.cell)
            for o in objs:
                assert segment_distance((o.x, o.y), a, b) >= o.radius + 0.35
        if 0 < dst.laps[0] < 5:                                      # progress 1-4 only on the ring
            x, y = gw.grid.center(dst.cell)
            assert box.radius + 0.75 - 1e-9 <= math.hypot(x - box.x, y - box.y) <= box.radius + 1.75 + 1e-9


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


def test_numbers():                            # plan Global Constraints, R4-R8, P4
    assert CELL == 0.5 and grid.MAX_CELLS == 256 and grid.BLOCK_MARGIN == 0.45 and LAP_DONE == 5
    assert (grid.NEAR_MIN_MARGIN, grid.NEAR_MAX_MARGIN, grid.NEAR_MAX_DISTANCE, grid.NEAR_MAX_ANGLE_DEG) == \
        (0.75, 1.5, 1.4, 40.0)
    assert (grid.RING_MIN_MARGIN, grid.RING_MAX_MARGIN) == (0.75, 1.75)


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
