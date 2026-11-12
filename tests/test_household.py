"""The household layout gazebo/layouts/household.json and its grid checks (open-objects spec §12.1–§12.5).

The layout keeps the default box_1 and chair_1 and adds five Fuel models. The checks build the grid and the PLANT
only (no synthesis):
- the acceptance runs from the spawn are unchanged by the new objects;
- every test formula of §12.4 has a plan from its test pose. "Test pose" means standing at (0, 0) facing 180° (B),
  +90° (L) or −90° (R). "Moves" means the shortest controllable path in the PLANT to a state with the target
  proposition.

Three §12.4 rows and three §12.3 gaps come out of the code differently from the spec. The spec computed them with
the sofa and the table at their model origins. The layout carries their footprint offsets, (0.02, 0) and
(0.002, 0), and every user takes the footprint centre from the layout. The pins below are the code's numbers, and
each such pin names the spec's number beside it. The spec also used the planned 0.6 x 0.4 bound for the suitcase;
H1 measured Suitcase1 in the Fuel image (0.291 x 0.468 in model axes, turned 90° so 0.468 x 0.291 in the world),
which moves one more row (the suitcase lap's PLANT) and the suitcase's, the sofa's and the table's gaps.
"""

import functools
import math
from collections import deque

import pytest
from go2_nl_control.actions import LYING, STANDING

from sim.sim_core import FakeCamera, SimGo2, in_camera_view, layout_objects
from voice_ltl import grid, layout, vocabulary
from voice_ltl.grid import grid_shape, grid_world
from voice_ltl.objects import DEFAULT_LAYOUT, Obj, formula_atoms, split_prop

NEW_IDS = ("trash_can_1", "suitcase_1", "sofa_1", "lamp_1", "table_1")
TEST_POSES = {"B": math.pi, "L": math.pi / 2, "R": -math.pi / 2}
# The four jittered starts of §12.4 (±0.1 m, ±5°), world axes, around the test pose. The yaw turns with the x shift.
# This set gives every "largest area" of the spec; the other four sign patterns give larger lamp grids (up to 135
# cells near, 176 for the lap).
JITTERS = ((0.1, 0.1, 5.0), (0.1, -0.1, 5.0), (-0.1, 0.1, -5.0), (-0.1, -0.1, -5.0))
# The acceptance starts of §12.4 (x, y, yaw in degrees).
ACCEPTANCE_STARTS = ((0.0, 0.0, 0.0), (0.06, -0.04, 3.0), (-0.05, 0.05, -3.0), (0.1, 0.1, 5.0), (-0.1, -0.1, -5.0))
ACCEPTANCE_FORMULAS = ("F around_box_1", "F near_chair_1", "F(near_chair_1 & X F hello)", "F near_box_1")


@functools.lru_cache(maxsize=None)
def household() -> layout.Layout:
    return layout.load("household")


def scene(sizes: dict[str, tuple[float, float]] | None = None) -> tuple[Obj, ...]:
    """The household objects (footprint centres, world sizes), with the sizes of some ids replaced."""
    sizes = sizes or {}
    return tuple(Obj(o.id, o.label, o.x, o.y, sizes.get(o.id, o.size)) for o in household().objs())


def target(formula: str) -> str:
    """The object proposition of a one-target test formula ("F near_sofa_1" -> "near_sofa_1")."""
    (prop,) = [a for a in formula_atoms(formula) if split_prop(a) is not None]
    return prop


def anchor(pose: str, jitter: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    dx, dy, dyaw = jitter
    return (dx, dy, TEST_POSES[pose] + math.radians(dyaw))


def build(formula: str, start: tuple[float, float, float], objs) -> grid.GridWorld:
    return grid_world(STANDING, False, False, start, objs, formula_atoms(formula))


def area(gw: grid.GridWorld) -> int:
    (i0, i1), (j0, j1) = gw.grid.i_range, gw.grid.j_range
    return (i1 - i0 + 1) * (j1 - j0 + 1)


def moves(gw: grid.GridWorld, prop: str) -> int | None:
    """The fewest controllable transitions from the initial state to a state labelled prop (BFS); None if none."""
    after: dict = {}
    for src, _, dst, kind in gw.world.transitions:
        if kind == "c":
            after.setdefault(src, []).append(dst)
    depth = {gw.world.initial: 0}
    queue = deque([gw.world.initial])
    while queue:
        state = queue.popleft()
        if prop in gw.label(state, None):
            return depth[state]
        for nxt in after.get(state, ()):
            if nxt not in depth:
                depth[nxt] = depth[state] + 1
                queue.append(nxt)
    return None


def footprint_gap(a: Obj, b: Obj) -> float:
    """The distance between two axis-aligned footprints (0 when they touch or overlap)."""
    dx = max(abs(a.x - b.x) - (a.size[0] + b.size[0]) / 2, 0.0)
    dy = max(abs(a.y - b.y) - (a.size[1] + b.size[1]) / 2, 0.0)
    return math.hypot(dx, dy)


# ---------------- the file ----------------
def test_household_layout_loads():
    lay = household()
    assert lay.name == "household" and "household" in layout.names()
    got = [(e.id, e.label, e.model, e.x, e.y, e.yaw_deg, e.offset, e.size, e.height, e.z, e.detectable, e.accept)
           for e in lay.entries]
    assert got == [
        ("box_1", "box", "primitive:box", 2.5, 1.5, 0, (0.0, 0.0), (0.5, 0.5), 0.5, 0.0, False, ("box",)),
        ("chair_1", "chair", "primitive:chair", 2.5, -1.5, 0, (0.0, 0.0), (0.45, 0.45), 0.9, 0.0, False, ("chair",)),
        # the room proposal after the H3 screening (R-H3b): Suitcase1 re-placed for Bf (yaw 90 kept, H1's measured
        # footprint), Construction Cone new (model_extent.py, collision: 0.50292 x 0.50292, height 1.088889,
        # z_min 0), the sofa 0.2 m back; TrashBin and LampAndStand left the room (R-H3c). detectable = Part B passed
        ("suitcase_1", "suitcase", "model://Suitcase1", -1.62, -1.14, 90, (0.0, 0.0), (0.291, 0.468), 0.752, 0.0,
         True, ("suitcase",)),
        ("traffic_cone_1", "traffic_cone", "model://construction_cone", -0.3, -4.45, 0, (0.0, 0.0), (0.503, 0.503),
         1.089, 0.0, True, ("traffic_cone",)),
        ("sofa_1", "sofa", "model://Sofa", -4.6, 0.0, 0, (0.02, 0.0), (1.044, 1.8), 0.943, 0.0, True, ("sofa",)),
        ("table_1", "table", "model://Table", -2.4, 4.4, 0, (0.002, 0.0), (1.5, 0.8), 1.015, 0.0, False,
         ("table", "desk")),
    ]
    world = {e.id: (e.centre(), e.world_size()) for e in lay.entries}
    assert world["sofa_1"][0] == pytest.approx((-4.58, 0.0)) and world["sofa_1"][1] == (1.044, 1.8)
    assert world["table_1"][0] == pytest.approx((-2.398, 4.4)) and world["table_1"][1] == (1.5, 0.8)
    assert world["suitcase_1"] == ((-1.62, -1.14), (0.468, 0.291))  # within the planned 0.6 x 0.4 (spec §12.3)
    for obj_id in ("box_1", "chair_1", "traffic_cone_1"):
        e = lay.find(obj_id)
        assert world[obj_id] == ((e.x, e.y), e.size)
    assert lay.objs()[:2] == DEFAULT_LAYOUT                      # the box and the chair as in the default layout
    assert lay.entries[:2] == layout.load("default").entries
    assert {e.folder for e in lay.entries[2:]} <= layout.model_folders()


def test_no_new_object_in_the_spawn_view():
    for obj_id in NEW_IDS:
        e = household().find(obj_id)
        (cx, cy), (sx, sy) = e.centre(), e.world_size()
        corners = [(cx + u * sx / 2, cy + v * sy / 2) for u in (-1, 1) for v in (-1, 1)]
        assert not in_camera_view((0.0, 0.0, 0.0), cx, cy), obj_id
        assert not any(in_camera_view((0.0, 0.0, 0.0), x, y) for x, y in corners), obj_id
    sim = SimGo2()                                               # the first Scene is today's: the box and the chair
    sim.place_objects(*layout_objects("household"))
    assert [o.id for o in FakeCamera().see(sim, 1.0)] == ["box_1", "chair_1"]


# ---------------- the acceptance runs from the spawn (§12.1, §12.4) ----------------
@pytest.mark.parametrize("formula", ACCEPTANCE_FORMULAS)
def test_acceptance_plants_are_unchanged_by_the_household_objects(formula):
    atoms = formula_atoms(formula)
    for sizes in ({}, {"box_1": (0.53, 0.52), "chair_1": (0.5, 0.5)}):      # design sizes, Gazebo-measured-like
        with_new = scene(sizes)
        without = tuple(o for o in with_new if o.id in ("box_1", "chair_1"))
        assert len(with_new) == 7 and len(without) == 2
        for x, y, yaw_deg in ACCEPTANCE_STARTS:
            start = (x, y, math.radians(yaw_deg))
            assert grid_shape(start, with_new, atoms) == grid_shape(start, without, atoms), (formula, sizes, start)
            a = grid_world(LYING, False, False, start, with_new, atoms).world
            b = grid_world(LYING, False, False, start, without, atoms).world
            assert (a.initial, a.states, a.transitions, a.event_label, a.forbidden, a.initial_label) == \
                (b.initial, b.states, b.transitions, b.event_label, b.forbidden, b.initial_label)


# ---------------- the test formulas (§12.4) ----------------
# formula, test pose, area [cells], moves, near keys (None for a lap), PLANT states, largest jittered area
TEST_FORMULAS = [
    ("F near_trash_can_1", "B", 64, 4, 12, 1416, 72),
    ("F around_trash_can_1", "B", 81, 23, None, 6912, 100),
    ("F near_suitcase_1", "B", 64, 4, 8, 1320, 72),
    ("F around_suitcase_1", "B", 121, 39, None, 8064, 110),  # spec 8 736 / 121 (the 0.6 x 0.4 bound; H1 measured)
    ("F near_sofa_1", "B", 165, 6, 18, 3144, 165),
    ("F around_sofa_1", "B", 221, 50, None, 14256, 238),    # spec 14 448 (the sofa at its origin, x -4.4)
    ("F near_lamp_1", "R", 112, 7, 15, 2328, 120),
    ("F around_lamp_1", "R", 160, 36, None, 10608, 160),
    ("F near_table_1", "L", 165, 10, 20, 3096, 168),         # spec 22 keys (the table at its origin, x -2.4)
    ("F around_table_1", "L", 204, 45, None, 12672, 221),    # spec 221 / 44 / 13 488 (the same)
]


@pytest.mark.parametrize("formula,pose,cells,steps,keys,states,jittered", TEST_FORMULAS,
                         ids=[row[0] for row in TEST_FORMULAS])
def test_every_test_formula_has_a_plan(formula, pose, cells, steps, keys, states, jittered):
    prop = target(formula)
    gw = build(formula, anchor(pose), scene())
    got = (area(gw), moves(gw, prop), len(gw.near[prop]) if keys is not None else None, len(gw.world.states))
    assert got == (cells, steps, keys, states)
    areas = []
    for jitter in JITTERS:
        g = build(formula, anchor(pose, jitter), scene())
        assert moves(g, prop) is not None, (formula, jitter)
        areas.append(area(g))
    assert max(areas) == jittered


def test_each_test_pose_sees_its_targets():                     # §12.3: so the turn puts them in the Scene
    targets = {"B": {"trash_can_1", "suitcase_1", "sofa_1"}, "R": {"lamp_1"}, "L": {"table_1"}}
    for pose, ids in targets.items():
        seen = {o.id for o in scene() if in_camera_view((0.0, 0.0, TEST_POSES[pose]), o.x, o.y)}
        assert ids <= seen, pose


def test_vocabulary_sizes_also_plan():
    """Detector mode before a measurement: the vocabulary [long, short] along world x (§12.4)."""
    sizes = {obj_id: vocabulary.size_of(obj_id.rsplit("_", 1)[0]) for obj_id in NEW_IDS}
    assert all(s is not None and s[0] >= s[1] for s in sizes.values())
    objs = scene(sizes)
    for formula, pose, *_ in TEST_FORMULAS:
        gw = build(formula, anchor(pose), objs)
        assert moves(gw, target(formula)) is not None, formula
    sofa = build("F near_sofa_1", anchor("B"), objs)
    table = build("F near_table_1", anchor("L"), objs)
    lamp = build("F near_lamp_1", anchor("R"), objs)
    assert (area(sofa), len(sofa.near["near_sofa_1"])) == (144, 17)
    assert (area(table), len(table.near["near_table_1"])) == (165, 18)
    assert grid.is_small(next(o for o in objs if o.id == "lamp_1")) and len(lamp.near["near_lamp_1"]) == 5


def test_inflated_sizes_lose_the_sofa_lap():
    """Sizes near the unmeasured [L, L] lose the sofa lap, so the test procedure waits for measured (§12.4)."""
    objs = scene({"sofa_1": (1.8, 1.8), "table_1": (1.5, 1.5), "suitcase_1": (0.7, 0.5), "lamp_1": (0.7, 0.7)})
    lap = build("F around_sofa_1", anchor("B"), objs)
    assert area(lap) == 255 and moves(lap, "around_sofa_1") is None
    for formula, pose, *_ in TEST_FORMULAS:
        if formula.startswith("F near_"):
            assert moves(build(formula, anchor(pose), objs), target(formula)) is not None, formula
    assert moves(build("F around_sofa_1", anchor("B"), scene()), "around_sofa_1") is not None   # measured: a plan


def test_nearest_footprint_gaps():
    objs = scene()
    nearest = {o.id: min(footprint_gap(o, p) for p in objs if p.id != o.id) for o in objs if o.id in NEW_IDS}
    # spec §12.3: 2.28, 2.12, 2.12, 2.68, 2.30 (the sofa at its origin, the suitcase at the 0.6 x 0.4 bound); the
    # sofa's 0.02 m offset brings it closer, the suitcase measured at H1 (0.468 x 0.291 in the world) moves away
    assert nearest == pytest.approx({"trash_can_1": 2.26, "suitcase_1": 2.17, "sofa_1": 2.17, "lamp_1": 2.68,
                                     "table_1": 2.35}, abs=0.01)
