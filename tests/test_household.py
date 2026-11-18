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
# 4 R5 runs), and so do H3b's Bf and R5 and every pose H5 reached (H5_POSES). Their 8 corners are checked below, but
# they do not stand for the inside: the grid is anchored at the start (voice_ltl/grid.py, Grid.anchor), so whether
# a plan exists is not monotone in the start pose (test_bf_plans_with_the_rows_the_bridge_publishes).
# R-H3k (1) widened the R5 box to H3b's real R5 pose (H3B_R5: x 0.017 m and the heading 0.38° beyond the
# screening's -0.342 and -97.6°). R-H5a widened it again: Bf to H5's x -0.501..-0.453 (the old box ended at
# -0.506), y up to -0.032 (was -0.054) and heading down to -174.0° (was -172.4°), each with the old box; R5 to H5's
# x -0.413; and L got a box, H3's L and H5's four (it had none). R-H6d widened Bf and R5 to every pose H6 reached
# (H6_POSES), by the same rule (the old box plus the poses, no margin): Bf y -0.142..-0.024 (was -0.141..-0.032) and
# heading up to -165.5° (was -166.6°); R5 x -0.492..-0.122 (was -0.413..-0.225), y -2.877..-2.631 (was
# -2.804..-2.654) and heading -100.5..-93.5° (was -98.0..-95.1°). L is unchanged (H6 has no L run).
H3B_R5 = (-0.359, -2.709, -98.0)
MEASURED_RANGES = {"Bf": ((-0.560, -0.453), (-0.142, -0.024), (-174.0, -165.5)),
                   "R5": ((-0.492, -0.122), (-2.877, -2.631), (-100.5, -93.5)),
                   "L": ((0.011, 0.037), (-0.045, 0.037), (92.5, 96.6))}
R_H5A_BF = ((-0.560, -0.453), (-0.141, -0.032), (-174.0, -166.6))    # the Bf box before R-H6d (R-H5a's)
# H5's poses at the end of the procedure (H5's record, "着いた姿勢"; odometry read after resolve): run, then
# (x, y, heading). Each run walked the one formula of its name.
H5_POSES = {"Bf": (("sofa_near1", (-0.453, -0.112, -166.9)), ("sofa_near2", (-0.484, -0.060, -173.8)),
                   ("sofa_lap1", (-0.494, -0.046, -171.6)), ("sofa_lap2", (-0.501, -0.125, -170.7)),
                   ("suitcase_near1", (-0.453, -0.057, -170.6)), ("suitcase_near2", (-0.485, -0.141, -167.6)),
                   ("suitcase_lap1", (-0.466, -0.062, -174.0)), ("suitcase_lap2", (-0.471, -0.032, -169.1))),
            "R5": (("cone_near1", (-0.345, -2.757, -97.5)), ("cone_near2", (-0.310, -2.723, -96.8)),
                   ("cone_lap1", (-0.413, -2.718, -97.6)), ("cone_lap2", (-0.374, -2.755, -96.9))),
            "L": (("table_near1", (0.037, -0.045, 96.6)), ("table_near2", (0.036, 0.035, 92.5)),
                  ("table_lap1", (0.012, 0.024, 94.6)), ("table_lap2", (0.034, 0.019, 96.0)))}
# H6's poses at the end of the procedure (H6's record, "着いた姿勢"; odometry read after the resolves): run, then
# (x, y, heading). The M4 runs (sofa_*, suitcase_*, cone_*) and m0w_* (M0, GUI off) walked the formula of their
# name in detector mode, voice_* the formula Claude gave (the same; voice_sofa_lap stopped at its 8th step, R-H6b),
# h5d_* in logical mode (R-H5d's cone re-runs); m3_* are M3's accuracy poses (oracle, yoloe, and the run that looked
# for covers' cause) and walked no formula.
H6_POSES = {"Bf": (("m3_oracle_Bf", (-0.529, -0.029, -173.6)), ("m3_yoloe_Bf", (-0.522, -0.066, -168.8)),
                   ("m3_cover_Bf", (-0.500, -0.055, -168.4)),
                   ("sofa_near1", (-0.517, -0.103, -170.8)), ("sofa_near2", (-0.525, -0.049, -171.2)),
                   ("sofa_near3", (-0.521, -0.105, -171.3)), ("sofa_lap1", (-0.501, -0.030, -173.6)),
                   ("sofa_lap2", (-0.501, -0.130, -165.5)), ("sofa_lap2_rerun", (-0.530, -0.137, -168.6)),
                   ("sofa_lap3", (-0.521, -0.033, -171.4)),
                   ("suitcase_near1", (-0.524, -0.104, -171.0)), ("suitcase_near2", (-0.474, -0.070, -170.6)),
                   ("suitcase_near3", (-0.498, -0.142, -166.5)), ("suitcase_lap1", (-0.534, -0.113, -170.4)),
                   ("suitcase_lap2", (-0.512, -0.087, -168.2)), ("suitcase_lap3", (-0.533, -0.029, -172.7)),
                   ("m0w_sofa_near", (-0.556, -0.106, -171.0)), ("m0w_sofa_lap", (-0.557, -0.111, -169.8)),
                   ("voice_sofa_near", (-0.516, -0.080, -168.1)), ("voice_sofa_lap", (-0.528, -0.080, -167.8)),
                   ("voice_sofa_lap_rerun", (-0.515, -0.032, -172.2)),
                   ("voice_suitcase_near", (-0.490, -0.045, -170.3)),
                   ("voice_suitcase_lap", (-0.525, -0.024, -173.6))),
            "R5": (("m3_oracle_R5", (-0.435, -2.715, -98.6)), ("m3_yoloe_R5", (-0.382, -2.658, -97.5)),
                   ("m3_cover_R5", (-0.359, -2.694, -97.6)),
                   ("cone_near1", (-0.285, -2.736, -96.8)), ("cone_near2", (-0.210, -2.701, -95.4)),
                   ("cone_near3", (-0.489, -2.667, -100.5)), ("cone_lap1", (-0.393, -2.631, -98.2)),
                   ("cone_lap2", (-0.122, -2.877, -93.5)), ("cone_lap3", (-0.310, -2.662, -96.3)),
                   ("voice_cone_near", (-0.294, -2.710, -96.4)), ("voice_cone_lap", (-0.492, -2.713, -99.7)),
                   ("h5d_cone_near1", (-0.264, -2.663, -96.0)), ("h5d_cone_near2", (-0.384, -2.642, -98.8)),
                   ("h5d_cone_lap1", (-0.448, -2.694, -100.1)), ("h5d_cone_lap2", (-0.339, -2.665, -97.9)))}
POSE_TARGETS = {**DETECTOR_TARGETS, "L": {"table_1"}}          # the targets of each test pose (L: logical only)
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
    every measured pose with its 4 jitters too), L's table included since R-H5a gave L a box. The box is wider than
    the runs: the old Bf box's corner (-0.56, -0.141, -172.4°) jittered by (+0.1, +0.1, +5°) loses the sofa lap, and
    so does the widened box's corner (-0.453, -0.141, -166.6°) jittered by (+0.1, -0.1, +5°) (R-H5a; no other corner
    jitter loses it), as does H5's sofa_near1 pose (-0.453, -0.112, -166.9°) with the same jitter: the thin margin
    of the sofa lap from Bf that H5 and H6 should know. In R-H6d's box only the corner (-0.453, -0.024, -165.5°)
    with that jitter loses it (every jitter of every Bf and R5 corner plans every other formula). All four lie
    outside every real run. These are the layout's sizes; with the rows the bridge publishes the sofa lap is thinner
    still, inside the box too (test_bf_plans_with_the_rows_the_bridge_publishes)."""
    for pose, ids in POSE_TARGETS.items():
        for start in measured_corners(pose):
            for obj_id in sorted(ids):
                for kind in ("near", "around"):
                    formula = f"F {kind}_{obj_id}"
                    assert moves(build(formula, start, scene()), target(formula)) is not None, (formula, start)
    for edge in ((-0.56 + 0.1, -0.141 + 0.1, -172.4 + 5.0), (-0.453 + 0.1, -0.141 - 0.1, -166.6 + 5.0),
                 (-0.453 + 0.1, -0.112 - 0.1, -166.9 + 5.0), (-0.453 + 0.1, -0.024 - 0.1, -165.5 + 5.0)):
        x, y, heading = edge
        assert moves(build("F around_sofa_1", (x, y, math.radians(heading)), scene()), "around_sofa_1") is None


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


# R-H5a: the plan of the formula each H5 run walked, from its own pose with the layout's sizes: (area [cells], moves,
# near keys or None, PLANT states). The moves are H5's ("計画の手") in 14 of the 16 runs. The two table laps planned
# with the bridge's table row instead of the layout's 1.5 x 0.8 (table_lap1 measured 1.412 x 0.807, table_lap2 still
# on F2's seed 1.5 x 1.2, R-H5c); with that row they give the run's moves (H5_TABLE_ROWS).
H5_PLANS = {"sofa_near1": (154, 5, 20, 3072), "sofa_near2": (154, 5, 18, 3120),
            "sofa_lap1": (208, 49, None, 14160), "sofa_lap2": (224, 55, None, 14928),
            "suitcase_near1": (56, 2, 7, 1224), "suitcase_near2": (64, 6, 5, 1416),
            "suitcase_lap1": (100, 26, None, 7920), "suitcase_lap2": (100, 26, None, 8016),
            "cone_near1": (64, 1, 6, 1416), "cone_near2": (72, 1, 8, 1584),
            "cone_lap1": (100, 28, None, 8208), "cone_lap2": (100, 28, None, 8400),
            "table_near1": (150, 10, 19, 3024), "table_near2": (165, 10, 15, 3456),
            "table_lap1": (221, 54, None, 14256), "table_lap2": (221, 49, None, 14016)}
H5_TABLE_ROWS = {"table_lap1": ((1.412, 0.807), 56), "table_lap2": ((1.5, 1.2), 54)}   # run: (table size, moves)
H5_RUN_TARGETS = {"sofa": "sofa_1", "suitcase": "suitcase_1", "cone": "traffic_cone_1", "table": "table_1"}


def h5_formula(run: str) -> str:
    """The formula an H5 run walked ("cone_lap2" -> "F around_traffic_cone_1")."""
    obj, what = run.split("_")
    return f"F {'near' if what.startswith('near') else 'around'}_{H5_RUN_TARGETS[obj]}"


def test_h5_poses_plan_and_see_only_their_targets():           # R-H5a: H5's real poses, pinned as R-H3k (1)
    """Every pose H5 reached lies in its widened box. From each, every near and lap formula of the pose's targets
    plans, and the formula the run walked has its pinned plan (H5_PLANS). The Bf and R5 pose views and their 4 shifts
    show the pose's targets alone (R-H3c): table_1 stays at least 0.34° outside a Bf view (suitcase_lap1's pose,
    the most turned run, with s3), the thinnest margin of a real pose. From R5 the cone stays well inside ±35°
    (R-H3b); from L the logical camera sees the table (§12.3)."""
    for pose, runs in H5_POSES.items():
        (x0, x1), (y0, y1), (h0, h1) = MEASURED_RANGES[pose]
        for run, (x, y, heading) in runs:
            assert x0 <= x <= x1 and y0 <= y <= y1 and h0 <= heading <= h1, run
            start = (x, y, math.radians(heading))
            for obj_id in sorted(POSE_TARGETS[pose]):
                for kind in ("near", "around"):
                    formula = f"F {kind}_{obj_id}"
                    prop = target(formula)
                    gw = build(formula, start, scene())
                    assert moves(gw, prop) is not None, (run, formula)
                    if formula == h5_formula(run):
                        keys = len(gw.near[prop]) if kind == "near" else None
                        assert (area(gw), moves(gw, prop), keys, len(gw.world.states)) == H5_PLANS[run], run
    assert sorted(H5_PLANS) == sorted(run for runs in H5_POSES.values() for run, _ in runs)
    poses = {run: (x, y, math.radians(heading)) for runs in H5_POSES.values() for run, (x, y, heading) in runs}
    for run, (size, steps) in H5_TABLE_ROWS.items():
        assert moves(build(h5_formula(run), poses[run], scene({"table_1": size})), "around_table_1") == steps, run
    table, cone = (next(o for o in scene() if o.id == i) for i in ("table_1", "traffic_cone_1"))
    margins = []
    for pose, ids in DETECTOR_TARGETS.items():
        for run, _ in H5_POSES[pose]:
            views = [poses[run]] + [shifted(poses[run], s) for s in VIEW_SHIFTS]
            assert [{o.id for o in scene() if rendered(v, o)} for v in views] == [ids] * 5, run
            if pose == "Bf":
                margins.append((min(min(abs(b) for b, _ in bearings(v, table)) for v in views) - HALF_FOV, run))
    assert min(margins)[0] == pytest.approx(0.344, abs=0.001) and min(margins)[1] == "suitcase_lap1"
    r5 = [poses[run] for run, _ in H5_POSES["R5"]]
    cam, base = [sight(p, cone.x, cone.y) for p in r5], [sight(p, cone.x, cone.y, camera=False) for p in r5]
    assert (min(b for b, _ in cam), max(b for b, _ in cam)) == pytest.approx((8.8, 13.9), abs=0.05)
    assert (min(b for b, _ in base), max(b for b, _ in base)) == pytest.approx((7.1, 11.3), abs=0.05)
    assert (min(d for _, d in base), max(d for _, d in base)) == pytest.approx((1.694, 1.736), abs=0.0005)
    for run, _ in H5_POSES["L"]:
        assert in_camera_view(poses[run], table.x, table.y), run


# R-H6d: the plan of the formula each H6 run walked, from its own pose with the layout's sizes: (area [cells], moves,
# near keys or None, PLANT states), then the run's own plan length (H6's "手"; None for voice_sofa_lap, which
# stopped at its 8th step). The detector-mode runs planned with the detector's rows (the suitcase 0.52-0.59 x
# 0.35-0.60, the cone 0.54-0.68 x 0.46-0.61, the sofa 1.00-1.03 x 1.83-1.93), not the layout's sizes, so the moves
# are the run's in 21 of the 31 runs with a known length; the 4 logical cone runs (h5d_*) are among them.
H6_PLANS = {"sofa_near1": ((140, 5, 22, 2736), 5), "sofa_near2": ((140, 5, 21, 2736), 5),
            "sofa_near3": ((140, 5, 22, 2736), 5), "sofa_lap1": ((208, 52, None, 14208), 52),
            "sofa_lap2": ((208, 48, None, 14304), 55), "sofa_lap2_rerun": ((208, 52, None, 14256), 50),
            "sofa_lap3": ((208, 49, None, 14016), 49),
            "suitcase_near1": ((56, 2, 8, 1248), 2), "suitcase_near2": ((56, 2, 6, 1224), 2),
            "suitcase_near3": ((64, 6, 4, 1416), 3), "suitcase_lap1": ((100, 26, None, 7920), 26),
            "suitcase_lap2": ((100, 26, None, 8064), 26), "suitcase_lap3": ((100, 26, None, 7920), 28),
            "m0w_sofa_near": ((140, 5, 21, 2760), 5), "m0w_sofa_lap": ((208, 50, None, 14016), 52),
            "voice_sofa_near": ((154, 5, 18, 3096), 5), "voice_sofa_lap": ((208, 52, None, 14256), None),
            "voice_sofa_lap_rerun": ((208, 49, None, 14208), 52),
            "voice_suitcase_near": ((56, 2, 7, 1200), 2), "voice_suitcase_lap": ((110, 26, None, 8496), 28),
            "cone_near1": ((72, 1, 5, 1584), 1), "cone_near2": ((63, 1, 10, 1368), 1),
            "cone_near3": ((63, 1, 8, 1368), 1), "cone_lap1": ((100, 28, None, 8400), 28),
            "cone_lap2": ((121, 39, None, 8880), 37), "cone_lap3": ((100, 30, None, 8256), 29),
            "voice_cone_near": ((72, 1, 6, 1608), 1), "voice_cone_lap": ((110, 30, None, 8880), 35),
            "h5d_cone_near1": ((63, 1, 5, 1344), 1), "h5d_cone_near2": ((63, 1, 6, 1368), 1),
            "h5d_cone_lap1": ((110, 30, None, 8880), 30), "h5d_cone_lap2": ((100, 28, None, 8016), 28)}
H6_RUN_TARGETS = {"sofa": "sofa_1", "suitcase": "suitcase_1", "cone": "traffic_cone_1"}


def h6_formula(run: str) -> str | None:
    """The formula an H6 run walked ("voice_cone_lap" -> "F around_traffic_cone_1"); None for M3's poses."""
    if run.startswith("m3_"):
        return None
    words = run.split("_")
    (obj,) = [w for w in words if w in H6_RUN_TARGETS]
    (what,) = [w for w in words if w.rstrip("0123456789") in ("near", "lap")]
    return f"F {'near' if what.startswith('near') else 'around'}_{H6_RUN_TARGETS[obj]}"


def test_h6_poses_plan_and_see_only_their_targets():           # R-H6d: H6's real poses, pinned as R-H5a did
    """Every pose H6 reached lies in its widened box, and 14 of the 38 lie outside the boxes before R-H6d (Bf: y up
    to 0.008 m past, heading 1.1° past; R5: x up to 0.103 m, y 0.073 m, heading 2.5° past). From each, every near and
    lap formula of the pose's targets plans, and the formula the run walked has its pinned plan (H6_PLANS). The pose
    views and their 4 shifts show the pose's targets alone (R-H3c): table_1 stays at least 0.91° outside a Bf view
    (sofa_lap1's pose with s3; H5's least was 0.34°). From R5 the cone stays inside ±35° of the image centre (R-H3b):
    -3.7…+20.2° from the camera, 1.58-1.82 m from the base."""
    old = {"Bf": R_H5A_BF, "R5": ((-0.413, -0.225), (-2.804, -2.654), (-98.0, -95.1))}

    def inside(box, x, y, heading):
        (x0, x1), (y0, y1), (h0, h1) = box
        return x0 <= x <= x1 and y0 <= y <= y1 and h0 <= heading <= h1

    outside, matches = [], 0
    for pose, runs in H6_POSES.items():
        for run, (x, y, heading) in runs:
            assert inside(MEASURED_RANGES[pose], x, y, heading), run
            if not inside(old[pose], x, y, heading):
                outside.append(run)
            start = (x, y, math.radians(heading))
            for obj_id in sorted(POSE_TARGETS[pose]):
                for kind in ("near", "around"):
                    formula = f"F {kind}_{obj_id}"
                    prop = target(formula)
                    gw = build(formula, start, scene())
                    assert moves(gw, prop) is not None, (run, formula)
                    if formula == h6_formula(run):
                        keys = len(gw.near[prop]) if kind == "near" else None
                        plan, steps = H6_PLANS[run]
                        assert (area(gw), moves(gw, prop), keys, len(gw.world.states)) == plan, run
                        matches += plan[1] == steps
    assert sorted(H6_PLANS) == sorted(run for runs in H6_POSES.values() for run, _ in runs if h6_formula(run))
    assert sum(steps is not None for _, steps in H6_PLANS.values()) == 31 and matches == 21
    assert sorted(outside) == sorted(["m3_oracle_Bf", "sofa_lap1", "sofa_lap2", "suitcase_near3", "suitcase_lap3",
                                      "voice_suitcase_lap", "m3_oracle_R5", "cone_near2", "cone_near3", "cone_lap1",
                                      "cone_lap2", "voice_cone_lap", "h5d_cone_near2", "h5d_cone_lap1"])
    poses = {run: (x, y, math.radians(heading)) for runs in H6_POSES.values() for run, (x, y, heading) in runs}
    table, cone = (next(o for o in scene() if o.id == i) for i in ("table_1", "traffic_cone_1"))
    margins = []
    for pose, ids in DETECTOR_TARGETS.items():
        for run, _ in H6_POSES[pose]:
            views = [poses[run]] + [shifted(poses[run], s) for s in VIEW_SHIFTS]
            assert [{o.id for o in scene() if rendered(v, o)} for v in views] == [ids] * 5, run
            if pose == "Bf":
                margins.append((min(min(abs(b) for b, _ in bearings(v, table)) for v in views) - HALF_FOV, run))
    assert min(margins)[0] == pytest.approx(0.912, abs=0.001) and min(margins)[1] == "sofa_lap1"
    r5 = [poses[run] for run, _ in H6_POSES["R5"]]
    cam, base = [sight(p, cone.x, cone.y) for p in r5], [sight(p, cone.x, cone.y, camera=False) for p in r5]
    assert (min(b for b, _ in cam), max(b for b, _ in cam)) == pytest.approx((-3.7, 20.2), abs=0.05)
    assert (min(d for _, d in base), max(d for _, d in base)) == pytest.approx((1.583, 1.821), abs=0.0005)


# The rows the bridge publishes from Bf (H5's record, "測った足跡の大きさ"): the sofa measured 1.052-1.054 x 1.782-1.830
# and the suitcase 0.495-0.506 x 0.305-0.312 (world axes). R-H5d floors each axis at the layout's size, so the sofa
# row is 1.052-1.054 x 1.8-1.830 and the suitcase's stays as measured (above the layout's 0.468 x 0.291). Each range
# is checked at both ends.
H5_BF_ROWS = {"suitcase_1": ((0.495, 0.305), (0.506, 0.312)), "sofa_1": ((1.052, 1.8), (1.054, 1.83))}
# The sofa lap's moves from each H5 Bf pose with those rows: (with sofa 1.052 x 1.8, with sofa 1.054 x 1.83), the
# same with either suitcase row. The first number is also the layout sizes' at every pose.
H5_BF_SOFA_LAP = {"sofa_near1": (54, 50), "sofa_near2": (50, 50), "sofa_lap1": (49, 49), "sofa_lap2": (55, 55),
                  "suitcase_near1": (56, 56), "suitcase_near2": (53, 51), "suitcase_lap1": (50, 50),
                  "suitcase_lap2": (52, 52)}
# Spots in the part of the Bf box that R-H5a added (no run was there) where the sofa lap has no plan: (x, y,
# heading), then its moves with the layout sizes, with the smallest rows (suitcase 0.495 x 0.305, sofa 1.052 x 1.8)
# and with the largest (0.506 x 0.312, 1.054 x 1.83); None: no plan.
BF_LAP_HOLES = (((-0.453, -0.0974, -168.08), (52, None, None)),   # the review's two: the measured suitcase alone
                ((-0.4744, -0.032, -166.6), (54, None, None)),
                ((-0.485, -0.141, -167.1), (50, 50, None)),        # 0.5° from H5's suitcase_near2 pose
                ((-0.458, -0.032, -167.35), (None, None, None)))   # the layout sizes too


def test_bf_plans_with_the_rows_the_bridge_publishes():         # R-H5a with R-H5d's rows (fix round 2)
    """The plan checks above use the layout's sizes; a run plans with the rows the bridge publishes, and from Bf the
    suitcase's measured row is wider than the layout's along world x (0.495-0.506 against 0.468). From every H5 Bf
    pose all four Bf formulas still plan with those rows (the sofa lap with each of the 4 row sets, the others with
    the smallest and the largest), and the sofa lap's moves are pinned (H5_BF_SOFA_LAP).

    The thin margin: the sofa lap's ring passes the suitcase, and where the grid (anchored at the start,
    Grid.anchor) puts a ring cell within BLOCK_MARGIN of the suitcase's footprint, the ring breaks and the lap has no
    plan. A wider suitcase row breaks it at more starts, so the box holds holes between starts that plan. On a
    6 x 6 x 6 grid over R-H5a's Bf box (R_H5A_BF) the lap has no plan at 6 of 216 points with the smallest rows,
    11 with the largest, 0 with the layout sizes. A finer scan (0.005 m, 0.25°, 6 y values) of x -0.503..-0.453,
    heading -170.1..-166.6° finds 97 of 990 with the smallest rows, 234 with the largest, and 16 with the layout sizes too.
    Every hole found lies at x -0.493..-0.453 and heading -169.1..-166.6°, in the part R-H5a added; none is at
    H5's poses. BF_LAP_HOLES pins four: the review's two (the
    first is 0.015 m and 1.2° from H5's sofa_near1 pose), one 0.5° from H5's suitcase_near2 pose with the largest
    rows, and one with the layout sizes. At the review's two the suitcase's x width does it alone (0.495 x 0.291
    empties the lap, 0.468 x 0.305 keeps the layout's moves). The box's corner (-0.453, -0.032, -166.6°) loses the
    lap with the largest sofa row. So an H6 sofa lap from Bf that ends its procedure in that part can be refused
    with no plan."""
    sets = [{"suitcase_1": s, "sofa_1": f} for s in H5_BF_ROWS["suitcase_1"] for f in H5_BF_ROWS["sofa_1"]]
    ends = (sets[0], sets[-1])                                  # the smallest and the largest rows
    for run, (x, y, heading) in H5_POSES["Bf"]:
        start = (x, y, math.radians(heading))
        for sizes in sets:
            got = moves(build("F around_sofa_1", start, scene(sizes)), "around_sofa_1")
            assert got == H5_BF_SOFA_LAP[run][H5_BF_ROWS["sofa_1"].index(sizes["sofa_1"])], (run, sizes)
        for sizes in ends:
            for formula in ("F near_sofa_1", "F near_suitcase_1", "F around_suitcase_1"):
                assert moves(build(formula, start, scene(sizes)), target(formula)) is not None, (run, formula, sizes)
        assert moves(build("F around_sofa_1", start, scene()), "around_sofa_1") == H5_BF_SOFA_LAP[run][0], run
    (x0, x1), (y0, y1), (h0, h1) = MEASURED_RANGES["Bf"]
    for (x, y, heading), pinned in BF_LAP_HOLES:
        assert x0 <= x <= x1 and y0 <= y <= y1 and h0 <= heading <= h1
        assert not (-0.560 <= x <= -0.506 and -0.141 <= y <= -0.054 and -172.4 <= heading <= -166.6)   # old box
        start = (x, y, math.radians(heading))
        got = tuple(moves(build("F around_sofa_1", start, scene(sizes)), "around_sofa_1") for sizes in ({}, *ends))
        assert got == pinned, start
    for (x, y, heading), (steps, _, _) in BF_LAP_HOLES[:2]:   # the suitcase's x width does it alone, not its y
        start = (x, y, math.radians(heading))
        for sizes, steps_got in (({"suitcase_1": (0.468, 0.305)}, steps), ({"suitcase_1": (0.495, 0.291)}, None),
                                 ({"suitcase_1": (0.495, 0.305)}, None)):
            assert moves(build("F around_sofa_1", start, scene(sizes)), "around_sofa_1") == steps_got, (start, sizes)
    (_, cx), (_, cy), (_, ch) = R_H5A_BF
    corner = (cx, cy, math.radians(ch))                         # R-H5a's box corner (-0.453, -0.032, -166.6°)
    for sizes, steps_got in (({}, 54), (sets[0], 54), ({"sofa_1": (1.054, 1.83)}, None), (sets[1], None)):
        assert moves(build("F around_sofa_1", corner, scene(sizes)), "around_sofa_1") == steps_got, sizes


# R-H6d: the Scene two H6 sofa laps planned with (`objects_check.py objects` just before the formula; detector rows:
# id, x, y, size in world axes). The sofa row stands about 0.075 m nearer than the layout's centre and is about
# 1.0 x 1.86 (its far side short, R-H6a); the suitcase row is 0.526-0.555 along world x against the layout's 0.468.
H6_BF_ROWS = {"sofa_lap1": (("chair_1", 2.496, -1.5, (0.555, 0.544)), ("sofa_1", -4.504, -0.006, (1.0, 1.858)),
                            ("suitcase_1", -1.602, -1.115, (0.555, 0.373))),
              "sofa_lap2": (("chair_1", 2.496, -1.506, (0.556, 0.551)), ("sofa_1", -4.505, 0.008, (1.002, 1.861)),
                            ("suitcase_1", -1.593, -1.119, (0.526, 0.378)))}
# Spots in the part of the Bf box that R-H6d added (no run was there) where the sofa lap has no plan with the layout
# sizes, nor with either row set above: (x, y, heading)
H6_BF_LAP_HOLES = ((-0.4744, -0.024, -166.2), (-0.453, -0.024, -167.2))


def test_bf_sofa_lap_with_the_rows_h6_planned_with():           # R-H6d with H6's detector rows
    """In detector mode the sofa lap plans with the detector's rows, and its thin margin from Bf (the ring passes
    the suitcase; test_bf_plans_with_the_rows_the_bridge_publishes) is thinner at the least turned headings. H6's
    sofa_lap2 ended its procedure at -165.5° and planned with its own rows (55 moves, as the run). With sofa_lap1's
    rows the same pose has no plan, and so has suitcase_near3's (-166.5°): the suitcase row's width along world x
    does it (sofa_lap2's 0.526 and 0.53 plan, 0.54 and wider do not). With the other five row sets of H6's sofa laps
    (not pinned here) sofa_lap2's pose has no plan either, and suitcase_near3's plans with 2 of the 7; every other H6
    Bf pose plans with all 7. On a scan of R-H6d's Bf box (a 6 x 6 x 6 grid plus the added edges, 360 points) the
    lap has no plan at 4 points with the layout sizes, all in the part R-H6d added (H6_BF_LAP_HOLES pins two), and
    at 48-125 points with each of those 7 row sets (4-21 of them inside R-H5a's box). So in H6b a sofa lap from Bf
    whose procedure ends turned less than about 167° can be refused with no plan; that is a refusal, not a fault."""
    def rows(run, suitcase_x=None):
        out = []
        for obj_id, x, y, size in H6_BF_ROWS[run]:
            if obj_id == "suitcase_1" and suitcase_x is not None:
                size = (suitcase_x, size[1])
            out.append(Obj(obj_id, obj_id.rsplit("_", 1)[0], x, y, size))
        return tuple(out)

    def lap(start, objs):
        return moves(build("F around_sofa_1", start, objs), "around_sofa_1")

    poses = {run: (x, y, math.radians(heading)) for run, (x, y, heading) in H6_POSES["Bf"]}
    assert [lap(poses["sofa_lap2"], objs) for objs in (rows("sofa_lap2"), rows("sofa_lap1"), scene())] == \
        [55, None, 48]
    assert [lap(poses["suitcase_near3"], objs) for objs in (rows("sofa_lap2"), rows("sofa_lap1"), scene())] == \
        [55, None, 50]
    assert [lap(poses["sofa_lap1"], objs) for objs in (rows("sofa_lap2"), rows("sofa_lap1"), scene())] == \
        [53, 53, 52]
    assert [lap(poses["sofa_lap2"], rows("sofa_lap2", w)) for w in (0.53, 0.54, 0.555)] == [55, None, None]
    (x0, x1), (y0, y1), (h0, h1) = MEASURED_RANGES["Bf"]
    (ox0, ox1), (oy0, oy1), (oh0, oh1) = R_H5A_BF
    for x, y, heading in H6_BF_LAP_HOLES:
        assert x0 <= x <= x1 and y0 <= y <= y1 and h0 <= heading <= h1
        assert not (ox0 <= x <= ox1 and oy0 <= y <= oy1 and oh0 <= heading <= oh1)     # R-H5a's box
        start = (x, y, math.radians(heading))
        assert [lap(start, objs) for objs in (scene(), rows("sofa_lap2"), rows("sofa_lap1"))] == [None] * 3


def test_each_test_pose_sees_its_targets():                     # §12.3: so the turn puts them in the Scene
    targets = {**DETECTOR_TARGETS, "L": {"table_1"}}
    for poses in (TEST_POSES, MEASURED_POSES):
        for pose, ids in targets.items():
            seen = {o.id for o in scene() if in_camera_view(anchor(pose, poses=poses), o.x, o.y)}
            assert ids <= seen, (pose, poses[pose])


def test_no_undetectable_model_in_a_detector_test_view():
    """R-H3c: at a detector test pose and its 4 view shifts the rendered camera (120°, 10 m, no occlusion) shows no
    model but the pose's targets, so every matched pair of §13.3 is a target. Checked at the measured poses (H3's and
    the corners of the box of every screening run's, H3b's and H5's, R-H3k (1) and R-H5a; H5's own poses in
    test_h5_poses_plan_and_see_only_their_targets). At the nominal Bf, exactly 180°, table_1 enters the pose view
    (its corner (-3.148, 4.0) at 59.88° against the 59.99° edge) and the s2 and s3 views (both -5°). At the
    measured headings it stays outside every view, but only just at the widened box's extremes: 0.03° at the
    corner (-0.453, -0.032, -174.0°) with s3 (2.3° at the corner (-0.506, -0.054, -172.4°) of the box before
    R-H5a; 0.34° at H5's most turned real pose). R-H6d's box breaks it at one corner, where no run was: at
    (-0.453, -0.024, -174.0°), which joins H6's y with H5's heading and x, table_1's corner comes 0.02° inside the
    s3 view. Every other view of the corners keeps it at least 0.69° outside (the corner (-0.453, -0.142, -174.0°)
    with s3), and H6's own poses at least 0.91° (test_h6_poses_plan_and_see_only_their_targets). R-H3e: the turns
    overshoot, so the room was placed with the measured yaws; re-run this view check if the turn controller changes.
    Also R-H3b: from Bf the suitcase never covers the sofa."""
    table = next(o for o in scene() if o.id == "table_1")
    nominal = anchor("Bf")
    assert [rendered(v, table) for v in [nominal] + [shifted(nominal, s) for s in VIEW_SHIFTS]] == \
        [True, False, True, True, False]                        # the pose view, s2 and s3
    margins, more = [], []
    for pose, ids in DETECTOR_TARGETS.items():
        for start in [anchor(pose, poses=MEASURED_POSES)] + measured_corners(pose):
            for k, view in enumerate([start] + [shifted(start, s) for s in VIEW_SHIFTS]):
                seen = {o.id for o in scene() if rendered(view, o)}
                assert ids <= seen, (pose, view)
                if seen != ids:                                 # a model that is not a target is in view
                    x, y, yaw = start
                    more.append((pose, (round(x, 3), round(y, 3), round(math.degrees(yaw), 1)), k, seen - ids))
                if pose == "Bf":
                    spans = {o.id: [b for b, _ in bearings(view, o)] for o in scene() if o.id in ids}
                    assert max(spans["sofa_1"]) < min(spans["suitcase_1"]), view    # the suitcase is left of it
                    margins.append((min(abs(b) for b, _ in bearings(view, table)) - HALF_FOV, seen == ids))
    assert more == [("Bf", (-0.453, -0.024, -174.0), 3, {"table_1"})]              # R-H6d's corner, s3
    assert [m for m, alone in margins if not alone] == pytest.approx([-0.020], abs=0.0005)
    least = min(m for m, alone in margins if alone)                                 # table_1's least margin, Bf
    assert least == pytest.approx(0.690, abs=0.0005) and least > 0


def test_the_suitcase_from_the_camera_and_from_the_base():
    """R-H3b's ±35° is about the image centre, so it is measured from the camera, which sits 0.327 m ahead of the base
    (CAMERA_IN_BODY); its 1.5-2.5 m is about the test pose, the base. The room proposal and R-H3h (4) gave base ->
    footprint centre (+29…+36°, 1.47-1.56 m over the screening's runs). From the camera, the suitcase's centre is
    +41.3° at H3's Bf, +35.3…+46.0° at H5's eight Bf poses, +34.7…+48.1° at H6's 23 and +32.6…+49.8° over the box
    of measured Bf poses (+26.0…+40.5° from the base): outside ±35° at every real pose but H6's sofa_lap2 (34.7°,
    the least turned run, -165.5°), and inside it only near the box's corner (-0.453, -0.142, -165.5°), 32.6°
    (R-H5a's box: near (-0.453, -0.141, -166.6°), 34.0°). It is 1.456-1.615 m from the base, under R-H3b's 1.5 m in
    part of the box, and 1.180-1.371 m from the camera. At H3's and H5's poses its truth box stays in the image in the
    pose view and the s1, s3 and s4 views (the pose view's far edge 2.0° inside at worst); in s2 (dyaw -5°) it runs
    past the left image edge at H3's pose, at 7 of H5's 8 (sofa_near1's keeps it 0.2° inside) and at 22 of H6's 23
    (sofa_lap2's keeps it inside), so a miss in Bf_s2 reads as clipping, not as a detector fault. H6's M3 oracle pose
    (-0.529, -0.029, -173.6°) also clips the pose view (far edge 60.05° against the 59.99° edge). Three corners of
    the box leave the s2 pattern, where no run was: (-0.453, -0.142, -165.5°) and (-0.453, -0.024, -165.5°) clip no
    view, and (-0.56, -0.024, -174.0°) clips the pose view and s3 too (far edges 61.8° and 61.5°). The placement
    stands as R-H3h (4) accepted it (Part B passed on real renders; H3b is the gate). These pins make a change to the
    turn controller show up."""
    s = next(o for o in scene() if o.id == "suitcase_1")
    h3 = anchor("Bf", poses=MEASURED_POSES)
    real = [h3] + [(x, y, math.radians(heading)) for _, (x, y, heading) in H5_POSES["Bf"]]
    h6_runs = [run for run, _ in H6_POSES["Bf"]]
    h6 = [(x, y, math.radians(heading)) for _, (x, y, heading) in H6_POSES["Bf"]]
    poses = real + h6 + measured_corners("Bf")
    n, m = len(real), len(real) + len(h6)                     # poses[:n] H3's and H5's, poses[n:m] H6's

    def span(rows, k):
        return min(r[k] for r in rows), max(r[k] for r in rows)

    assert sight(h3, s.x, s.y)[0] == pytest.approx(41.3, abs=0.05)
    assert sight(h3, s.x, s.y, camera=False)[0] == pytest.approx(33.3, abs=0.05)
    assert sight(h3, s.x, s.y, camera=False)[1] == pytest.approx(1.550, abs=0.0005)
    cam = [sight(p, s.x, s.y) for p in poses]
    base = [sight(p, s.x, s.y, camera=False) for p in poses]
    assert span(cam, 0) == pytest.approx((32.6, 49.8), abs=0.05) and span(cam, 0)[0] < 35.0
    assert span(cam[1:n], 0) == pytest.approx((35.25, 45.99), abs=0.005) and span(cam[:n], 0)[0] > 35.0
    assert span(cam[n:m], 0) == pytest.approx((34.66, 48.12), abs=0.005)
    assert [run for run, (b, _) in zip(h6_runs, cam[n:m]) if b < 35.0] == ["sofa_lap2"]
    assert span(base, 0) == pytest.approx((26.0, 40.5), abs=0.05)
    assert span(base, 1) == pytest.approx((1.456, 1.615), abs=0.0005) and span(base, 1)[0] < 1.5
    assert span(cam, 1) == pytest.approx((1.180, 1.371), abs=0.0005)
    left, clipped = {}, []                       # per view: the footprint's leftmost bearing, box x0; clipped views
    for p in poses:
        cut = set()
        for k, view in enumerate([p] + [shifted(p, sh) for sh in VIEW_SHIFTS]):
            box = truth_box(view, s)
            far = max(b for b, _ in bearings(view, s))
            assert box is not None and (box[0] == 0.0) == (far > HALF_FOV), (k, view)
            if box[0] == 0.0:
                cut.add(k)
            left.setdefault(k, []).append((far, box[0]))
        clipped.append(cut)
    assert clipped[:n] == [{2}, set()] + [{2}] * 7                   # H3's, then H5's (sofa_near1 clips none)
    assert {run: cut for run, cut in zip(h6_runs, clipped[n:m]) if cut != {2}} == {"m3_oracle_Bf": {0, 2},
                                                                                    "sofa_lap2": set()}
    assert clipped[m:] == [{2}, {2}, {0, 2, 3}, {2}, {2}, set(), {2}, set()]         # measured_corners' order
    assert span(left[2], 0) == pytest.approx((57.69, 72.87), abs=0.005)  # s2: past the 60° edge, clipped at u = 0
    assert span(left[2][:n], 0) == pytest.approx((59.76, 69.23), abs=0.005)
    assert span(left[0][:n], 0) == pytest.approx((47.70, 57.945), abs=0.005)  # the pose view: inside
    assert span(left[0][:n], 1) == pytest.approx((24.825, 116.85), abs=0.005)  # its left edge at u (px)
    assert span(left[0][n:m], 0)[1] == pytest.approx(60.045, abs=0.005)  # past the edge at H6's m3_oracle_Bf
    assert span(left[0], 0)[1] == pytest.approx(61.81, abs=0.005)       # ... and at the box corner


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
