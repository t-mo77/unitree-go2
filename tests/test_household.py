"""The household layout gazebo/layouts/household.json and its grid checks (open-objects spec §12.1–§12.5).

The layout keeps the default box_1 and chair_1 and adds four Fuel models: the room proposal after the H3 screening
(R-H3b). Suitcase1 stands about 1.5 m from Bf (1.457-1.556 m from the base), Construction Cone (traffic_cone_1) is
new and seen from R5, the sofa moved 0.2 m back, the table is unchanged; TrashBin and LampAndStand left the room
(R-H3c). The checks build the grid and the PLANT only (no synthesis):
- the acceptance runs from the spawn are unchanged by the new objects (spec §12.4, R-H3g);
- every target formula has a plan from its test pose. The test poses (R-H3a, R-H3b) are reached from the spawn:
  Bf = two left turns, then one F moved_forward (the sofa and the suitcase); R5 = one right turn, then five
  F moved_forward (the cone); L = one left turn (the table, logical only). Each is pinned twice: at its nominal
  pose (exact quarter turns; Bf one 0.5 m step, R5 five 0.55 m steps) and at the pose Gazebo measured (H3: the
  turns overshoot, about 96° and 191.5°, R-H3e; the R5 one is derived from H3's R, see MEASURED_POSES). "Moves"
  means the shortest controllable path in the PLANT to a state with the target proposition.
- the geometry facts the room's thin margins rest on, from the camera (R-H3b's ±35° is about the image centre, and
  the camera sits CAMERA_IN_BODY ahead of the base) and from the base.

The pins are the code's numbers. They moved with the room (R-H3g), as R-H1a's moved with the measured Suitcase1;
the spec's §12.3/§12.4 numbers are for the first room and Task 4.1 replaces them.
"""

import functools
import math
from collections import deque

import pytest
from go2_nl_control.actions import LYING, STANDING

from detector.camera import CameraModel
from detector.core import View
from detector.oracle import project_box
from sim.sim_core import FakeCamera, SimGo2, in_camera_view, layout_objects
from voice_ltl import grid, layout, vocabulary
from voice_ltl.grid import grid_shape, grid_world
from voice_ltl.objects import CAMERA_HFOV, CAMERA_IN_BODY, DEFAULT_LAYOUT, Obj, formula_atoms, split_prop

NEW_IDS = ("suitcase_1", "traffic_cone_1", "sofa_1", "table_1")
DETECTOR_TARGETS = {"Bf": {"sofa_1", "suitcase_1"}, "R5": {"traffic_cone_1"}}   # detector_check.PREVIEW_TARGETS
# The test poses (x, y, yaw in degrees). Nominal: exact quarter turns; Bf one 0.5 m step, R5 five 0.55 m steps (H3's
# measured step length). Measured: Gazebo's (H3, R-H3e): Bf after the two left turns and the step (H3 run 4), L as H3
# measured it. The R5 one is derived, not a run's: H3's R (0.027, 0.006, -95.8) plus five steps of 0.55 m along that
# heading (-0.2509, -2.7299), the room proposal's pose; the 4 real R5 runs are in MEASURED_RANGES.
TEST_POSES = {"Bf": (-0.5, 0.0, 180.0), "R5": (0.0, -2.75, -90.0), "L": (0.0, 0.0, 90.0)}
MEASURED_POSES = {"Bf": (-0.506, -0.062, -169.2), "R5": (-0.251, -2.730, -95.8), "L": (0.011, 0.037, 96.1)}
# Every pose the H3 screening measured at the end of the procedure lies in these boxes (x, y, heading; 6 Bf runs,
# 4 R5 runs), and so do H3b's Bf and R5: their 8 corners stand for them below. R-H3k (1) widened the R5 box to H3b's
# real R5 pose (H3B_R5: x 0.017 m and the heading 0.38° beyond the screening's -0.342 and -97.6°).
H3B_R5 = (-0.359, -2.709, -98.0)
MEASURED_RANGES = {"Bf": ((-0.560, -0.506), (-0.141, -0.054), (-172.4, -166.6)),
                   "R5": ((-0.359, -0.225), (-2.804, -2.654), (-98.0, -95.1))}
# The four jittered starts of §12.4 (±0.1 m, ±5°), world axes, around the test pose. The yaw turns with the x shift.
JITTERS = ((0.1, 0.1, 5.0), (0.1, -0.1, 5.0), (-0.1, 0.1, -5.0), (-0.1, -0.1, -5.0))
# record_frames' four view shifts (dx forward, dy left, dyaw), in the Go2's own frame (plan H3)
VIEW_SHIFTS = ((0.1, 0.1, 5.0), (0.1, -0.1, -5.0), (-0.1, 0.1, -5.0), (-0.1, -0.1, 5.0))
RENDER_FAR = 10.0             # the rendered camera's far clip (m): what YOLOE could see, occlusion aside
HALF_FOV = math.degrees(CAMERA_HFOV) / 2      # 59.99°: the bearing of the image's left and right edges
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


def anchor(pose: str, jitter: tuple[float, float, float] = (0.0, 0.0, 0.0),
           poses: dict = TEST_POSES) -> tuple[float, float, float]:
    """The start (x, y, yaw in rad) of a test pose, jittered in world axes."""
    (x, y, yaw_deg), (dx, dy, dyaw) = poses[pose], jitter
    return (x + dx, y + dy, math.radians(yaw_deg + dyaw))


def measured_corners(pose: str) -> list[tuple[float, float, float]]:
    """The 8 corners of the box of measured poses (x, y, yaw in rad)."""
    (x0, x1), (y0, y1), (h0, h1) = MEASURED_RANGES[pose]
    return [(x, y, math.radians(h)) for x in (x0, x1) for y in (y0, y1) for h in (h0, h1)]


def shifted(start: tuple[float, float, float], shift: tuple[float, float, float]) -> tuple[float, float, float]:
    """The Go2 at start moved by shift = (dx, dy, dyaw_deg) in its own frame (what record_frames --shift emulates)."""
    x, y, yaw = start
    dx, dy, dyaw = shift
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * dx - s * dy, y + s * dx + c * dy, yaw + math.radians(dyaw))


def bearings(start: tuple[float, float, float], o: Obj) -> list[tuple[float, float]]:
    """(bearing in degrees, left +; distance) from the front camera to the corners, edge midpoints and centre of o."""
    x, y, yaw = start
    c, s = math.cos(yaw), math.sin(yaw)
    cx, cy = x + c * CAMERA_IN_BODY[0] - s * CAMERA_IN_BODY[1], y + s * CAMERA_IN_BODY[0] + c * CAMERA_IN_BODY[1]
    out = []
    for u in (-1, 0, 1):
        for v in (-1, 0, 1):
            px, py = o.x + u * o.size[0] / 2, o.y + v * o.size[1] / 2
            b = math.atan2(py - cy, px - cx) - yaw
            out.append((math.degrees(math.atan2(math.sin(b), math.cos(b))), math.hypot(px - cx, py - cy)))
    return out


def rendered(start: tuple[float, float, float], o: Obj) -> bool:
    """Some point of o's footprint lies in the rendered camera's 120° within RENDER_FAR (no occlusion)."""
    return any(abs(b) <= HALF_FOV and d <= RENDER_FAR for b, d in bearings(start, o))


def sight(start: tuple[float, float, float], x: float, y: float, camera: bool = True) -> tuple[float, float]:
    """(bearing in degrees, left +; distance) to the point (x, y), from the front camera or from the base."""
    px, py, yaw = start
    if camera:
        c, s = math.cos(yaw), math.sin(yaw)
        px, py = px + c * CAMERA_IN_BODY[0] - s * CAMERA_IN_BODY[1], py + s * CAMERA_IN_BODY[0] + c * CAMERA_IN_BODY[1]
    b = math.atan2(y - py, x - px) - yaw
    return math.degrees(math.atan2(math.sin(b), math.cos(b))), math.hypot(x - px, y - py)


def truth_box(start: tuple[float, float, float], o: Obj) -> tuple[float, float, float, float] | None:
    """o's image box (x0, y0, x1, y1), clipped to the image, as detector_check.truth_boxes projects it: the footprint
    x the layout height through detector.oracle.project_box, with the camera model of the D7 ruling."""
    x, y, yaw = start
    view = View((x, y, 0.0), (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)), CameraModel.from_hfov(), 0.0)
    got = project_box(view, o.x, o.y, o.size, household().find(o.id).height, view.camera.width, view.camera.height)
    return None if got is None else got[0]


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
        assert len(with_new) == 6 and len(without) == 2
        for x, y, yaw_deg in ACCEPTANCE_STARTS:
            start = (x, y, math.radians(yaw_deg))
            assert grid_shape(start, with_new, atoms) == grid_shape(start, without, atoms), (formula, sizes, start)
            a = grid_world(LYING, False, False, start, with_new, atoms).world
            b = grid_world(LYING, False, False, start, without, atoms).world
            assert (a.initial, a.states, a.transitions, a.event_label, a.forbidden, a.initial_label) == \
                (b.initial, b.states, b.transitions, b.event_label, b.forbidden, b.initial_label)


def test_new_objects_keep_off_the_acceptance_cells():
    """The screening's line for §12.1 (room proposal), stricter than the PLANT test above, which is the rule (§12.4):
    every new footprint is at least 0.7 m from every cell centre of the acceptance grids (4 formulas x 5 starts).
    So the right edge of §12.1 is a footprint edge, about y <= -4.2 (the cone's is -4.1985): the proposal's y -4.5
    is the centre line of a 0.6 m object, not an edge (at y -4.0 the cone changed `F near_chair_1`)."""
    near = tuple(o for o in scene() if o.id in ("box_1", "chair_1"))
    centres = set()
    for formula in ACCEPTANCE_FORMULAS:
        for x, y, yaw_deg in ACCEPTANCE_STARTS:
            g = grid_world(LYING, False, False, (x, y, math.radians(yaw_deg)), near, formula_atoms(formula)).grid
            centres |= {g.center((i, j)) for i in range(g.i_range[0], g.i_range[1] + 1)
                        for j in range(g.j_range[0], g.j_range[1] + 1)}

    def gap(o: Obj, px: float, py: float) -> float:
        return math.hypot(max(abs(px - o.x) - o.size[0] / 2, 0.0), max(abs(py - o.y) - o.size[1] / 2, 0.0))

    gaps = {o.id: min(gap(o, px, py) for px, py in centres) for o in scene() if o.id in NEW_IDS}
    assert gaps == pytest.approx({"suitcase_1": 0.701, "traffic_cone_1": 0.768, "sofa_1": 3.376, "table_1": 1.002},
                                 abs=0.0005)
    assert min(gaps.values()) >= 0.7
    cone = household().find("traffic_cone_1")
    assert cone.centre()[1] + cone.world_size()[1] / 2 == pytest.approx(-4.1985)


# ---------------- the test formulas (§12.4) ----------------
# formula, test pose, area [cells], moves, near keys (None for a lap), PLANT states, largest jittered area; the room
# proposal's numbers (R-H3g), first at the nominal pose, then at the measured one
TEST_FORMULAS = [
    ("F near_sofa_1", "Bf", 154, 5, 18, 3072, 154),           # first room, from B: 165 / 6 / 18 / 3 144, 165
    ("F around_sofa_1", "Bf", 208, 49, None, 14208, 224),    # first room, from B: 221 / 50 / 14 256, 238
    ("F near_suitcase_1", "Bf", 49, 2, 8, 1032, 56),         # first room, from B: 64 / 4 / 8 / 1 320, 72
    ("F around_suitcase_1", "Bf", 100, 29, None, 8016, 100),  # first room, from B: 121 / 39 / 8 064, 110
    ("F near_traffic_cone_1", "R5", 72, 1, 6, 1584, 63),     # new kind
    ("F around_traffic_cone_1", "R5", 100, 29, None, 8256, 121),
    ("F near_table_1", "L", 165, 10, 20, 3408, 168),          # first room: 165 / 10 / 20 / 3 096, 168
    ("F around_table_1", "L", 204, 45, None, 13488, 221),     # first room: 204 / 45 / 12 672, 221
]
MEASURED_FORMULAS = [                                          # the same at MEASURED_POSES (the room proposal's table)
    ("F near_sofa_1", "Bf", 140, 5, 20, 2736, 154),
    ("F around_sofa_1", "Bf", 208, 52, None, 14016, 208),
    ("F near_suitcase_1", "Bf", 56, 2, 8, 1224, 56),
    ("F around_suitcase_1", "Bf", 100, 26, None, 7968, 110),
    ("F near_traffic_cone_1", "R5", 63, 1, 6, 1368, 63),
    ("F around_traffic_cone_1", "R5", 100, 29, None, 8448, 110),
    ("F near_table_1", "L", 150, 10, 20, 3048, 165),
    ("F around_table_1", "L", 221, 56, None, 14016, 204),
]


def check_plan(formula, pose, cells, steps, keys, states, jittered, poses):
    prop = target(formula)
    gw = build(formula, anchor(pose, poses=poses), scene())
    got = (area(gw), moves(gw, prop), len(gw.near[prop]) if keys is not None else None, len(gw.world.states))
    assert got == (cells, steps, keys, states)
    areas = []
    for jitter in JITTERS:
        g = build(formula, anchor(pose, jitter, poses), scene())
        assert moves(g, prop) is not None, (formula, jitter)
        areas.append(area(g))
    assert max(areas) == jittered


@pytest.mark.parametrize("formula,pose,cells,steps,keys,states,jittered", TEST_FORMULAS,
                         ids=[row[0] for row in TEST_FORMULAS])
def test_every_test_formula_has_a_plan(formula, pose, cells, steps, keys, states, jittered):
    check_plan(formula, pose, cells, steps, keys, states, jittered, TEST_POSES)


@pytest.mark.parametrize("formula,pose,cells,steps,keys,states,jittered", MEASURED_FORMULAS,
                         ids=[row[0] for row in MEASURED_FORMULAS])
def test_every_test_formula_has_a_plan_from_the_measured_pose(formula, pose, cells, steps, keys, states, jittered):
    check_plan(formula, pose, cells, steps, keys, states, jittered, MEASURED_POSES)


def test_every_measured_pose_plans():
    """The 8 corners of the boxes of measured poses plan near and lap of their targets (the H3 screening checked
    every measured pose with its 4 jitters too). The box is wider than the runs: one corner jittered by
    (+0.1, +0.1, +5°) loses the sofa lap, the thin margin of the sofa lap from Bf that H5 and H6 should know."""
    for pose, ids in DETECTOR_TARGETS.items():
        for start in measured_corners(pose):
            for obj_id in sorted(ids):
                for kind in ("near", "around"):
                    formula = f"F {kind}_{obj_id}"
                    assert moves(build(formula, start, scene()), target(formula)) is not None, (formula, start)
    edge = (-0.56 + 0.1, -0.141 + 0.1, math.radians(-172.4 + 5.0))
    assert moves(build("F around_sofa_1", edge, scene()), "around_sofa_1") is None


def test_h3b_r5_pose_plans_and_sees_only_the_cone():           # R-H3k (1): H3b's real R5, pinned as R-H3g
    """H3b's R5 pose lies in the widened box. From it, both cone formulas plan, from the pose and from its 4 jitters
    (H3b's table), and the pose view and its 4 shifts show the cone alone (R-H3c); table_1 stays 84.8° outside."""
    (x0, x1), (y0, y1), (h0, h1) = MEASURED_RANGES["R5"]
    x, y, heading = H3B_R5
    assert x0 <= x <= x1 and y0 <= y <= y1 and h0 <= heading <= h1
    start = (x, y, math.radians(heading))
    pins = {"F near_traffic_cone_1": ((72, 1, 7, 1584), [(63, 8), (56, 1), (63, 4), (56, 1)]),
            "F around_traffic_cone_1": ((100, 28, None, 8256), [(110, 31), (110, 31), (110, 30), (121, 35)])}
    for formula, (plan, jittered) in pins.items():
        prop = target(formula)
        gw = build(formula, start, scene())
        keys = len(gw.near[prop]) if formula.startswith("F near_") else None
        assert (area(gw), moves(gw, prop), keys, len(gw.world.states)) == plan, formula
        got = []
        for dx, dy, dyaw in JITTERS:
            g = build(formula, (x + dx, y + dy, math.radians(heading + dyaw)), scene())
            got.append((area(g), moves(g, prop)))
        assert got == jittered, formula
    views = [start] + [shifted(start, s) for s in VIEW_SHIFTS]
    assert [{o.id for o in scene() if rendered(v, o)} for v in views] == [{"traffic_cone_1"}] * 5
    cone, table = (next(o for o in scene() if o.id == i) for i in ("traffic_cone_1", "table_1"))
    (cam_b, cam_d), (base_b, base_d) = sight(start, cone.x, cone.y), sight(start, cone.x, cone.y, camera=False)
    assert (cam_b, base_b) == pytest.approx((12.2, 9.9), abs=0.05)         # H3b: well inside ±35° (R-H3b)
    assert (cam_d, base_d) == pytest.approx((1.421, 1.742), abs=0.0005)    # 1.5-2.5 m from the base
    assert min(min(abs(b) for b, _ in bearings(v, table)) for v in views) - HALF_FOV == pytest.approx(84.8, abs=0.05)


def test_each_test_pose_sees_its_targets():                     # §12.3: so the turn puts them in the Scene
    targets = {**DETECTOR_TARGETS, "L": {"table_1"}}
    for poses in (TEST_POSES, MEASURED_POSES):
        for pose, ids in targets.items():
            seen = {o.id for o in scene() if in_camera_view(anchor(pose, poses=poses), o.x, o.y)}
            assert ids <= seen, (pose, poses[pose])


def test_no_undetectable_model_in_a_detector_test_view():
    """R-H3c: at a detector test pose and its 4 view shifts the rendered camera (120°, 10 m, no occlusion) shows no
    model but the pose's targets, so every matched pair of §13.3 is a target. Checked at the measured poses (H3's and
    the corners of the box of every screening run's and H3b's, R-H3k (1)). At the nominal Bf, exactly 180°, table_1 enters the pose view (its corner
    (-3.148, 4.0) at 59.88° against the 59.99° edge) and the s2 and s3 views (both -5°). At the measured headings it
    stays at least 2.3° outside every view (the corner (-0.506, -0.054, -172.4°) with s3). R-H3e: the turns
    overshoot, so the room was placed with the measured yaws; re-run this view check if the turn controller changes.
    Also R-H3b: from Bf the suitcase never covers the sofa."""
    table = next(o for o in scene() if o.id == "table_1")
    nominal = anchor("Bf")
    assert [rendered(v, table) for v in [nominal] + [shifted(nominal, s) for s in VIEW_SHIFTS]] == \
        [True, False, True, True, False]                        # the pose view, s2 and s3
    margins = []
    for pose, ids in DETECTOR_TARGETS.items():
        for start in [anchor(pose, poses=MEASURED_POSES)] + measured_corners(pose):
            for view in [start] + [shifted(start, s) for s in VIEW_SHIFTS]:
                assert {o.id for o in scene() if rendered(view, o)} == ids, (pose, view)
                if pose == "Bf":
                    spans = {o.id: [b for b, _ in bearings(view, o)] for o in scene() if o.id in ids}
                    assert max(spans["sofa_1"]) < min(spans["suitcase_1"]), view    # the suitcase is left of it
                    margins.append(min(abs(b) for b, _ in bearings(view, table)) - HALF_FOV)
    assert min(margins) == pytest.approx(2.31, abs=0.005)       # table_1's least margin outside a Bf view


def test_the_suitcase_from_the_camera_and_from_the_base():
    """R-H3b's ±35° is about the image centre, so it is measured from the camera, which sits 0.327 m ahead of the base
    (CAMERA_IN_BODY); its 1.5-2.5 m is about the test pose, the base. The room proposal and R-H3h (4) gave base ->
    footprint centre (+29…+36°, 1.47-1.56 m over the screening's runs). From the camera, the suitcase's centre is
    +41.3° at H3's Bf and +35.8…+47.2° over the box of measured Bf poses (+28.5…+38.1° from the base): outside ±35°
    at every measured pose, not only in the -172.4° run (the least overshoot, 187.6° turned). It is 1.457-1.556 m
    from the base, under R-H3b's 1.5 m in part of the box, and 1.184-1.308 m from the camera. Its truth box stays in
    the image in the pose view and the s1, s3 and s4 views; in s2 (dyaw -5°) it runs past the left image edge at
    every measured pose, so a miss in Bf_s2 reads as clipping, not as a detector fault. The placement stands as
    R-H3h (4) accepted it (Part B passed on real renders; H3b is the gate). These pins make a change to the turn
    controller show up."""
    s = next(o for o in scene() if o.id == "suitcase_1")
    h3 = anchor("Bf", poses=MEASURED_POSES)
    poses = [h3] + measured_corners("Bf")

    def span(rows, k):
        return min(r[k] for r in rows), max(r[k] for r in rows)

    assert sight(h3, s.x, s.y)[0] == pytest.approx(41.3, abs=0.05)
    assert sight(h3, s.x, s.y, camera=False)[0] == pytest.approx(33.3, abs=0.05)
    assert sight(h3, s.x, s.y, camera=False)[1] == pytest.approx(1.550, abs=0.0005)
    cam = [sight(p, s.x, s.y) for p in poses]
    base = [sight(p, s.x, s.y, camera=False) for p in poses]
    assert span(cam, 0) == pytest.approx((35.8, 47.2), abs=0.05) and span(cam, 0)[0] > 35.0
    assert span(base, 0) == pytest.approx((28.5, 38.1), abs=0.05)
    assert span(base, 1) == pytest.approx((1.457, 1.556), abs=0.0005) and span(base, 1)[0] < 1.5
    assert span(cam, 1) == pytest.approx((1.184, 1.308), abs=0.0005)
    left = {}                                                   # per view: the footprint's leftmost bearing, box x0
    for p in poses:
        for k, view in enumerate([p] + [shifted(p, sh) for sh in VIEW_SHIFTS]):
            box = truth_box(view, s)
            far = max(b for b, _ in bearings(view, s))
            assert box is not None and (box[0] == 0.0) == (far > HALF_FOV) == (k == 2), (k, view)
            left.setdefault(k, []).append((far, box[0]))
    assert span(left[2], 0) == pytest.approx((61.2, 70.9), abs=0.05)     # s2: past the 60° edge, clipped at u = 0
    assert span(left[0], 0) == pytest.approx((48.9, 59.5), abs=0.05)     # the pose view: inside, by 0.5° at worst
    assert span(left[0], 1) == pytest.approx((6.1, 108.3), abs=0.05)     # ... its left edge at u 6.1-108.3 px


def test_vocabulary_sizes_also_plan():
    """Detector mode before a measurement: the vocabulary [long, short] along world x (§12.4)."""
    sizes = {obj_id: vocabulary.size_of(obj_id.rsplit("_", 1)[0]) for obj_id in NEW_IDS}
    assert all(s is not None and s[0] >= s[1] for s in sizes.values())
    objs = scene(sizes)
    for poses in (TEST_POSES, MEASURED_POSES):
        for formula, pose, *_ in TEST_FORMULAS:
            gw = build(formula, anchor(pose, poses=poses), objs)
            assert moves(gw, target(formula)) is not None, (formula, poses[pose])
    got = {}
    for poses in (TEST_POSES, MEASURED_POSES):
        sofa = build("F near_sofa_1", anchor("Bf", poses=poses), objs)
        table = build("F near_table_1", anchor("L", poses=poses), objs)
        cone = build("F near_traffic_cone_1", anchor("R5", poses=poses), objs)
        got[poses is MEASURED_POSES] = ((area(sofa), len(sofa.near["near_sofa_1"])),
                                        (area(table), len(table.near["near_table_1"])),
                                        len(cone.near["near_traffic_cone_1"]))
    # nominal, then measured (first room, nominal B / L: sofa (144, 17), table (165, 18), lamp 5 keys)
    assert got == {False: ((135, 17), (165, 18), 4), True: ((135, 20), (150, 21), 7)}
    assert grid.is_small(next(o for o in objs if o.id == "traffic_cone_1"))
    assert grid.is_small(next(o for o in objs if o.id == "suitcase_1"))


def test_inflated_sizes_lose_the_sofa_lap():
    """Sizes near the unmeasured [L, L] lose the sofa lap from the measured Bf, so the test procedure waits for
    measured (§12.4). From the nominal Bf (exactly 180°) the lap still has a plan: R-H3g moved this pin (first room,
    from B: 255 cells and no plan)."""
    objs = scene({"sofa_1": (1.8, 1.8), "table_1": (1.5, 1.5), "suitcase_1": (0.7, 0.5),
                  "traffic_cone_1": (0.6, 0.6)})
    lap = build("F around_sofa_1", anchor("Bf", poses=MEASURED_POSES), objs)
    assert area(lap) == 238 and moves(lap, "around_sofa_1") is None
    nominal = build("F around_sofa_1", anchor("Bf"), objs)
    assert (area(nominal), moves(nominal, "around_sofa_1")) == (255, 59)
    for poses in (TEST_POSES, MEASURED_POSES):
        for formula, pose, *_ in TEST_FORMULAS:
            if formula.startswith("F near_"):
                assert moves(build(formula, anchor(pose, poses=poses), objs), target(formula)) is not None, formula
    measured = build("F around_sofa_1", anchor("Bf", poses=MEASURED_POSES), scene())
    assert moves(measured, "around_sofa_1") is not None                                          # measured: a plan


def test_nearest_footprint_gaps():
    objs = scene()
    nearest = {o.id: min(footprint_gap(o, p) for p in objs if p.id != o.id) for o in objs if o.id in NEW_IDS}
    # the room proposal's 2.21, 3.03, 2.21, 3.23 (first room: suitcase 2.17, sofa 2.17, table 2.35)
    assert nearest == pytest.approx({"suitcase_1": 2.21, "traffic_cone_1": 3.03, "sofa_1": 2.21, "table_1": 3.23},
                                    abs=0.01)
