"""gazebo/tools/objects_check.py の合否の計算（summarize ほか）。ROS 2 も Gazebo も使わない。"""

import math
from dataclasses import replace
from types import SimpleNamespace

import pytest

from gazebo.tools.objects_check import (THRESHOLDS, accuracy, accuracy_check, covers, first_action_required,
                                        last_source, layout_mismatch, layout_name, odometry_sample, pair_rows,
                                        pick_target, resolve, rtf_stats, settled, summarize, valid_message)
from voice_ltl import layout
from voice_ltl.objects import DEFAULT_LAYOUT, Obj, format_message

BOX = (2.5, 1.5)        # DEFAULT_LAYOUT の box_1
CHAIR = (2.5, -1.5)     # DEFAULT_LAYOUT の chair_1
STAND_Z = 0.33
LYING_Z = 0.08
LOG = ["計画：立つ → 前進", "開始：立つ", "完了：立つ", "開始：前進"]


def keys(result: dict) -> list[str]:
    """failures の各行の頭（THRESHOLDS の名前）。"""
    return [f.split(":")[0] for f in result["failures"]]


def arc(turns: float, clockwise: bool = False, radius: float = 1.2, centre=BOX, n: int = 72,
        t0: float = 10.0) -> list[tuple]:
    """centre の周りの円弧を turns 周、立った高さで。東の点から始め、向きは進む方向（接線）。"""
    sign = -1.0 if clockwise else 1.0
    steps = round(n * turns)
    out = []
    for k in range(steps + 1):
        a = sign * 2 * math.pi * turns * k / steps
        yaw = math.atan2(math.sin(a + sign * math.pi / 2), math.cos(a + sign * math.pi / 2))
        out.append((t0 + 0.5 * k, centre[0] + radius * math.cos(a), centre[1] + radius * math.sin(a),
                    yaw, STAND_Z, 0.05))
    return out


def walk_to(x: float, y: float, final_yaw: float = 0.0, n: int = 20) -> list[tuple]:
    """原点で伏せ → 立つ → (x, y) へまっすぐ歩き、最後に final_yaw を向く。"""
    heading = math.atan2(y, x)
    out = [(0.0, 0.0, 0.0, 0.0, LYING_Z, 0.0), (4.0, 0.0, 0.0, 0.0, STAND_Z, 0.04)]
    for k in range(1, n + 1):
        out.append((4.0 + k, x * k / n, y * k / n, heading, STAND_Z, 0.04))
    out.append((5.0 + n, x, y, final_yaw, STAND_Z, 0.03))
    return out


def test_thresholds_are_the_plan_values():
    assert THRESHOLDS == {"lap": {"winding": (0.9, 1.5), "min_edge_box_1": 0.6},
                          "near": {"final_dist_chair_1": 1.5, "final_angle_chair_1_deg": 45.0,
                                   "min_edge_chair_1": 0.45},
                          "all": {"max_tilt": 1.0, "first_action": "立つ", "exit_code": 0},
                          # open-objects spec §10.8 (edge-based lines), O16 (the box's centre line), §13.3
                          "target_near": {"max_edge": 1.5, "max_angle_deg": 45.0, "min_clearance": 0.30},
                          "target_lap": {"winding": (0.9, 1.5), "min_edge": 0.6},
                          "box_centre": {"final_dist_box_1": 1.5, "final_angle_box_1_deg": 45.0},
                          "accuracy": {"max_pos_err_target": 0.15, "max_pos_err": 0.35, "median_pos_err": 0.25,
                                       "max_size_err": 0.4, "min_label_acc": 0.8, "min_recall": 0.6,
                                       "max_false_pos": 1}}


def test_counter_clockwise_circle_around_the_box_passes_the_lap():
    r = summarize(arc(1.0), "lap", 0, ["開始：立つ"])
    assert r["passed"] is True and r["failures"] == []
    assert r["expect"] == "lap" and r["exit_code"] == 0 and r["first_action"] == "立つ"
    assert r["winding_box_1"] == pytest.approx(1.0, abs=1e-3)
    assert r["min_edge_box_1"] == pytest.approx(1.2 - 0.25 * math.sqrt(2), abs=1e-3)   # at a diagonal
    assert r["max_tilt"] == pytest.approx(0.05)
    assert set(r) == {"expect", "exit_code", "first_action", "winding_box_1", "min_edge_box_1",
                      "min_edge_chair_1", "final_dist_chair_1", "final_angle_chair_1_deg",
                      "max_tilt", "passed", "failures"}


def test_clockwise_circle_fails_on_winding():
    r = summarize(arc(1.0, clockwise=True), "lap", 0, ["開始：立つ"])
    assert r["passed"] is False
    assert keys(r) == ["winding"]
    assert r["winding_box_1"] == pytest.approx(-1.0, abs=1e-3)


@pytest.mark.parametrize("turns, passed", [(0.88, False), (0.92, True), (1.45, True), (1.55, False)])
def test_winding_must_be_in_the_half_open_band(turns, passed):
    r = summarize(arc(turns), "lap", 0, ["開始：立つ"])
    assert r["winding_box_1"] == pytest.approx(turns, abs=1e-3)
    assert r["passed"] is passed
    assert keys(r) == ([] if passed else ["winding"])


def test_a_lap_too_close_to_the_box_fails_on_its_edge():
    r = summarize(arc(1.0, radius=0.8), "lap", 0, ["開始：立つ"])
    assert r["min_edge_box_1"] == pytest.approx(0.8 - 0.25 * math.sqrt(2), abs=1e-3)   # 0.446
    assert keys(r) == ["min_edge_box_1"]


def test_samples_before_standing_count_only_for_tilt():
    # lying 0.25 m from the box edge, then standing up on the circle: the lying samples would
    # break the edge and the winding if they counted
    lying = [(0.0, 2.0, 1.5, 0.0, LYING_Z, 0.0), (1.0, 2.0, 1.5, 0.0, LYING_Z, 0.3)]
    r = summarize(lying + arc(1.0), "lap", 0, LOG)
    assert r["passed"] is True, r["failures"]
    assert r["min_edge_box_1"] == pytest.approx(1.2 - 0.25 * math.sqrt(2), abs=1e-3)
    assert r["winding_box_1"] == pytest.approx(1.0, abs=1e-3)
    assert r["max_tilt"] == pytest.approx(0.3)


def test_straight_walk_that_stops_near_the_chair_passes():
    r = summarize(walk_to(1.5, -1.0), "near", 0, LOG)
    assert r["passed"] is True and r["failures"] == []
    assert r["final_dist_chair_1"] == pytest.approx(1.118, abs=1e-3)
    assert r["final_angle_chair_1_deg"] == pytest.approx(26.6, abs=0.05)
    assert r["min_edge_chair_1"] >= 0.45


@pytest.mark.parametrize("end, yaw, failed", [
    ((0.5, -1.0), 0.0, "final_dist_chair_1"),             # 2.06 m from the chair
    ((1.5, -1.0), math.pi / 2, "final_angle_chair_1_deg"),  # facing left: 116.6° off
    ((2.0, -1.5), 0.0, "min_edge_chair_1"),               # 0.275 m from its edge
])
def test_near_fails_when_far_turned_away_or_touching(end, yaw, failed):
    r = summarize(walk_to(*end, final_yaw=yaw), "near", 0, LOG)
    assert r["passed"] is False
    assert keys(r) == [failed]


def test_tilt_over_the_limit_fails():
    samples = arc(1.0)
    t, x, y, yaw, z, _ = samples[30]
    samples[30] = (t, x, y, yaw, z, 1.2)
    r = summarize(samples, "lap", 0, ["開始：立つ"])
    assert r["max_tilt"] == pytest.approx(1.2)
    assert keys(r) == ["max_tilt"]


def test_a_failed_run_fails():
    r = summarize(arc(1.0), "lap", 1, ["開始：立つ"])
    assert r["exit_code"] == 1
    assert keys(r) == ["exit_code"]


@pytest.mark.parametrize("log, first", [
    (["計画：前進", "開始：前進", "開始：立つ"], "前進"),
    (["式が文法か単語帳に合いません：F x"], None),
    ([], None),
])
def test_the_first_action_must_be_stand(log, first):
    r = summarize(arc(1.0), "lap", 0, log)
    assert r["first_action"] == first
    assert keys(r) == ["first_action"]


def test_no_odometry_fails_every_measured_check():
    r = summarize([], "near", 0, ["開始：立つ"])
    assert r["passed"] is False
    assert r["final_dist_chair_1"] is None and r["max_tilt"] is None
    assert keys(r) == ["final_dist_chair_1", "final_angle_chair_1_deg", "min_edge_chair_1", "max_tilt"]


def test_never_standing_fails_the_lap():
    lying = [(float(k), 0.0, 0.0, 0.0, LYING_Z, 0.0) for k in range(5)]
    r = summarize(lying, "lap", 0, ["開始：立つ"])
    assert r["winding_box_1"] is None and r["min_edge_box_1"] is None
    assert keys(r) == ["winding", "min_edge_box_1"]


def test_unknown_expect_is_refused():
    with pytest.raises(ValueError):
        summarize(arc(1.0), "sideways", 0, ["開始：立つ"])


def test_the_layout_decides_where_the_box_is():
    moved = tuple(o if o.id != "box_1" else Obj("box_1", "box", 0.0, 5.0) for o in DEFAULT_LAYOUT)
    r = summarize(arc(1.0, centre=(0.0, 5.0)), "lap", 0, ["開始：立つ"], layout=moved)
    assert r["passed"] is True, r["failures"]


def test_odometry_sample_reads_pose_and_tilt():
    yaw, pitch = math.pi / 2, 0.2      # quaternion of yaw then pitch (ZYX), roll 0
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    q = SimpleNamespace(w=cy * cp, x=-sy * sp, y=cy * sp, z=sy * cp)
    msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=500_000_000)),
                          pose=SimpleNamespace(pose=SimpleNamespace(
                              position=SimpleNamespace(x=1.0, y=-2.0, z=0.31), orientation=q)))
    t, x, y, yw, z, tilt = odometry_sample(msg)
    assert (t, x, y, z) == pytest.approx((12.5, 1.0, -2.0, 0.31))
    assert yw == pytest.approx(yaw)
    assert tilt == pytest.approx(pitch)


def _message(rows, frame="odom"):
    return {"frame": frame, "stamp": 1.0, "objects": rows}


BOX_ROW = {"id": "box_1", "label": "box", "x": 2.5, "y": 1.5, "size": [0.5, 0.5], "seen": 1.0,
           "visible": True}
CHAIR_ROW = {"id": "chair_1", "label": "chair", "x": 2.5, "y": -1.5, "size": [0.45, 0.45],
             "seen": 1.0, "visible": True}


@pytest.mark.parametrize("payload, ok", [
    (_message([BOX_ROW, CHAIR_ROW]), True),
    (_message([BOX_ROW, CHAIR_ROW, {"id": "ground_plane", "label": "ground", "x": 0.0, "y": 0.0,
                                    "size": [100.0, 100.0], "seen": 1.0, "visible": True}]), False),
    (_message([BOX_ROW], frame="map"), False),
    (_message([]), False),
    ({"frame": "odom"}, False),
    ("not a dict", False),
])
def test_a_valid_message_keeps_every_row(payload, ok):
    assert valid_message(payload) is ok


# ---- Task I part 1: the review's fixes (wrap_rad, the RTF window, the child stopped in finally) ----

def test_the_final_angle_uses_objects_wrap_rad(monkeypatch):
    import gazebo.tools.objects_check as oc
    from voice_ltl import objects

    calls = []

    def spy(a):
        calls.append(a)
        return objects.wrap_rad(a)

    monkeypatch.setattr(oc, "wrap_rad", spy)
    assert not hasattr(oc, "_wrap")
    # bearing to the chair from (1.5, -1.0) is -26.6°; facing +x after one extra turn (yaw 2π)
    r = summarize(walk_to(1.5, -1.0, final_yaw=2 * math.pi), "near", 0, LOG)
    assert len(calls) == 1
    assert r["final_angle_chair_1_deg"] == pytest.approx(26.6, abs=0.05) and r["passed"] is True


def test_the_final_angle_folds_across_minus_pi():
    # bearing -26.6°, yaw +3π/2 (= -90°): the angle is 63.4°, not 296.6°
    r = summarize(walk_to(1.5, -1.0, final_yaw=1.5 * math.pi), "near", 0, LOG)
    assert r["final_angle_chair_1_deg"] == pytest.approx(63.4, abs=0.05)


def test_rtf_is_sim_time_over_the_wall_window():
    from gazebo.tools.objects_check import real_time_factor

    ticks = [(100.0 + 0.1 * k, 50.0 + 0.08 * k) for k in range(201)]   # RTF 0.8 for 20 s
    assert real_time_factor(ticks, 100.0, 120.0) == pytest.approx(0.8)


def test_rtf_counts_a_clock_stall_at_the_end_of_the_window():
    """/clock stops for the last 5 s of a 20 s window: those seconds count, with no sim time."""
    from gazebo.tools.objects_check import real_time_factor

    ticks = [(100.0 + 0.1 * k, 50.0 + 0.08 * k) for k in range(151)]   # until 115.0, then nothing
    assert real_time_factor(ticks, 100.0, 120.0) == pytest.approx(0.6)   # 12 sim s / 20 wall s


def test_rtf_starts_from_the_last_tick_before_the_window():
    from gazebo.tools.objects_check import real_time_factor

    ticks = [(99.95, 49.0), (100.5, 50.0), (110.0, 58.0)]
    assert real_time_factor(ticks, 100.0, 110.0) == pytest.approx(0.9)   # 58 - 49 over 10 s
    assert real_time_factor([], 100.0, 110.0) is None
    assert real_time_factor(ticks, 110.0, 110.0) is None


class FakeChild:
    """subprocess.Popen as _stop_child and _watch use it: running until it gets SIGINT."""

    def __init__(self, exited: bool = False):
        self.signals: list[int] = []
        self.code = 0 if exited else None
        self.killed = False

    def poll(self):
        return self.code

    def send_signal(self, sig):
        self.signals.append(sig)
        self.code = 1

    def wait(self, timeout=None):
        return self.code

    def kill(self):
        self.killed = True
        self.code = -9


def test_the_child_is_stopped_when_recording_fails():
    import signal

    from gazebo.tools.objects_check import _watch

    child = FakeChild()

    def broken_spin(done, timeout):
        raise RuntimeError("rclpy failed")

    with pytest.raises(RuntimeError):
        _watch(child, broken_spin, 300.0)
    assert child.signals == [signal.SIGINT]   # run_formula gets SIGINT: StopMove x3


def test_the_child_is_stopped_on_ctrl_c_and_the_run_goes_on():
    import signal

    from gazebo.tools.objects_check import _watch

    child = FakeChild()

    def interrupted(done, timeout):
        raise KeyboardInterrupt

    _watch(child, interrupted, 300.0)          # no exception: the JSON line is still printed
    assert child.signals == [signal.SIGINT]


def test_a_child_that_has_exited_is_left_alone():
    from gazebo.tools.objects_check import TAIL_S, _watch

    child = FakeChild(exited=True)
    waits = []

    def spin(done, timeout):
        waits.append(timeout)
        return done()

    _watch(child, spin, 300.0)
    assert child.signals == [] and not child.killed
    assert waits[-1] == TAIL_S                 # recorded TAIL_S more after the child exited


def test_a_tip_over_two_seconds_after_the_end_counts_in_max_tilt():
    """The tail records 3 s after run_formula exits, so a Go2 that falls over shortly after the
    end fails the run (A N3 and B L1 fell 1.2-2 s after it; a 1 s tail passed over both)."""
    from gazebo.tools.objects_check import TAIL_S, _watch

    assert TAIL_S == 3.0
    samples = arc(1.0)
    end = samples[-1][0]
    _, x, y, yaw, z, _ = samples[-1]
    late = [(end + dt, x, y, yaw, z, tilt) for dt, tilt in
            ((0.5, 0.3), (1.0, 0.6), (1.5, 0.9), (2.0, 1.4), (2.5, 3.1))]

    def spin(done, timeout):
        """The recorder: odometry keeps coming for `timeout` seconds after the child exits."""
        if done():
            return True
        samples.extend(s for s in late if s[0] <= end + timeout)
        return False

    _watch(FakeChild(exited=True), spin, 300.0)
    r = summarize(samples, "lap", 0, ["開始：立つ"])
    assert r["max_tilt"] == pytest.approx(3.1)
    assert keys(r) == ["max_tilt"]


def test_a_child_past_its_time_is_stopped_once():
    import signal

    from gazebo.tools.objects_check import CHILD_MARGIN_S, _watch

    child = FakeChild()
    waits = []

    def spin(done, timeout):
        waits.append(timeout)
        return False                            # the child never exits by itself

    _watch(child, spin, 10.0)
    assert child.signals == [signal.SIGINT]
    assert waits[0] == 10.0 + CHILD_MARGIN_S


# ---- Task 2.4: target runs (spec §10.8, C1, O16), watch, accuracy (§11.11, §13.3), resolve (M10), RTF ----

SOFA = Obj("sofa_1", "sofa", -4.4, 0.0, (1.044, 1.8))     # the household sofa: behind the start, long side on y
SOFA_EDGE_X = -4.4 + 1.044 / 2                              # its +x edge, the side the start faces
BOX_OBJ, CHAIR_OBJ = DEFAULT_LAYOUT
TRASH = Obj("trash_can_1", "trash_can", -1.5, -1.5, (0.27, 0.38))
TURN_LOG = ["計画：右を向く → 前進", "開始：右を向く", "完了：右を向く", "開始：前進"]


def before_sofa(edge: float, final_yaw: float = math.pi) -> list[tuple]:
    """Lying at the origin, stand, walk along -x and stop `edge` m from the sofa's +x edge."""
    return walk_to(SOFA_EDGE_X + edge, 0.0, final_yaw=final_yaw)


def standing_start(path: list[tuple], z: float = 0.30) -> list[tuple]:
    """The same run, but the first sample is already standing (z), as after the turns of spec §12.4."""
    t, x, y, yaw, _, tilt = path[0]
    return [(t, x, y, yaw, z, tilt)] + path[1:]


def around(obj: Obj, d: float, n: int = 12, t0: float = 10.0) -> list[tuple]:
    """Once counter-clockwise around obj's footprint at edge distance d: a quarter circle at each corner
    (the straight sides join them), standing. Every sample is exactly d from the footprint."""
    a, b = obj.size[0] / 2, obj.size[1] / 2
    pts = [(a + d, 0.0)]
    for (cx, cy), start in zip(((a, b), (-a, b), (-a, -b), (a, -b)), (0, 90, 180, 270)):
        for k in range(n + 1):
            ang = math.radians(start + 90 * k / n)
            pts.append((cx + d * math.cos(ang), cy + d * math.sin(ang)))
    pts.append((a + d, 0.0))
    return [(t0 + 0.5 * k, obj.x + px, obj.y + py, 0.0, STAND_Z, 0.05) for k, (px, py) in enumerate(pts)]


def test_target_near_lines_on_synthetic_paths():
    ok = summarize(before_sofa(1.5), "near", 0, LOG, target=SOFA, others=(TRASH,))
    assert ok["passed"] is True, ok["failures"]
    assert ok["target"] == "sofa_1" and ok["expect"] == "near"
    assert ok["final_edge"] == pytest.approx(1.5, abs=1e-3)
    assert ok["final_footprint_angle_deg"] == 0.0
    assert ok["min_clearance"] == pytest.approx(1.31, abs=1e-3)      # the trash can beside the path, 1.5 - 0.19
    assert ok["min_clearance_id"] == "trash_can_1"
    assert "final_dist_chair_1" not in ok and "winding_box_1" not in ok   # §2 keys only for chair_1 / box_1
    # the target among the others counts once
    assert summarize(before_sofa(1.5), "near", 0, LOG, target=SOFA, others=(SOFA, TRASH)) == ok

    far = summarize(before_sofa(1.51), "near", 0, LOG, target=SOFA, others=(TRASH,))
    assert far["final_edge"] == pytest.approx(1.51, abs=1e-3)
    assert keys(far) == ["final_edge"]

    hull = math.atan2(0.9, 1.5)      # the sofa's near corners seen from the stop, either side of facing -x
    turned = summarize(before_sofa(1.5, final_yaw=math.pi - hull - math.radians(46)), "near", 0, LOG,
                       target=SOFA, others=(TRASH,))
    assert turned["final_footprint_angle_deg"] == pytest.approx(46.0, abs=0.05)
    assert keys(turned) == ["final_footprint_angle_deg"]
    inside = summarize(before_sofa(1.5, final_yaw=math.pi - hull - math.radians(44.9)), "near", 0, LOG,
                       target=SOFA, others=(TRASH,))
    assert inside["passed"] is True, inside["failures"]

    close = replace(TRASH, y=-0.48)            # its edge 0.29 m beside the straight path along y = 0
    brushed = summarize(before_sofa(1.5), "near", 0, LOG, target=SOFA, others=(close,))
    assert brushed["min_clearance"] == pytest.approx(0.29, abs=1e-3)
    assert brushed["min_clearance_id"] == "trash_can_1"
    assert keys(brushed) == ["min_clearance"]


def test_target_lap_lines():
    ok = summarize(around(SOFA, 0.6), "lap", 0, ["開始：立つ"], target=SOFA, others=(TRASH,))
    assert ok["passed"] is True, ok["failures"]
    assert ok["winding"] == pytest.approx(1.0, abs=1e-3)
    assert ok["min_edge"] == pytest.approx(0.6, abs=1e-6)
    tight = summarize(around(SOFA, 0.59), "lap", 0, ["開始：立つ"], target=SOFA, others=(TRASH,))
    assert tight["min_edge"] == pytest.approx(0.59, abs=1e-6)
    assert keys(tight) == ["min_edge"]
    backwards = summarize(around(SOFA, 0.6)[::-1], "lap", 0, ["開始：立つ"], target=SOFA)
    assert backwards["winding"] == pytest.approx(-1.0, abs=1e-3) and keys(backwards) == ["winding"]


def test_runs_starting_standing_need_no_stand():
    """C1: after the turns of spec §12.4 the Go2 already stands, so the run starts with a turn or a walk."""
    standing = standing_start(before_sofa(1.2), z=0.30)
    assert first_action_required(standing) is None
    r = summarize(standing, "near", 0, TURN_LOG, target=SOFA)
    assert r["passed"] is True, r["failures"]
    assert r["first_action"] == "右を向く" and r["first_action_required"] is None

    lying = standing_start(before_sofa(1.2), z=0.10)
    assert first_action_required(lying) == "立つ"
    r = summarize(lying, "near", 0, TURN_LOG, target=SOFA)
    assert r["first_action_required"] == "立つ"
    assert keys(r) == ["first_action"]
    assert summarize(lying, "near", 0, LOG, target=SOFA)["passed"] is True
    assert first_action_required([]) == "立つ"           # nothing recorded: the strict rule


def test_box_near_centre_line_only_in_logical_mode():
    """O16: 「箱の前まで」 keeps its 1.5 m / 45° from the box's centre, measured only in logical mode (O28)."""
    far_from_centre = walk_to(0.85, 1.5)     # 1.4 m from the box's edge, 1.65 m from its centre
    r = summarize(far_from_centre, "near", 0, LOG, target=BOX_OBJ, others=(CHAIR_OBJ,), source="logical")
    assert r["final_edge"] == pytest.approx(1.4, abs=1e-3)
    assert r["final_dist_box_1"] == pytest.approx(1.65, abs=1e-3)
    assert keys(r) == ["final_dist_box_1"]
    for source in ("detector", "fake", None):
        r = summarize(far_from_centre, "near", 0, LOG, target=BOX_OBJ, others=(CHAIR_OBJ,), source=source)
        assert r["passed"] is True, (source, r["failures"])
        assert "final_dist_box_1" not in r and "final_angle_box_1_deg" not in r
        assert r["source"] == source

    # 50° off the centre, but the box's near edge is within 45° of the final yaw
    askew = walk_to(1.25, 1.5, final_yaw=math.radians(50))
    r = summarize(askew, "near", 0, LOG, target=BOX_OBJ, others=(CHAIR_OBJ,), source="logical")
    assert r["final_angle_box_1_deg"] == pytest.approx(50.0, abs=0.05)
    assert r["final_footprint_angle_deg"] == pytest.approx(50.0 - math.degrees(math.atan2(0.25, 1.0)), abs=0.05)
    assert keys(r) == ["final_angle_box_1_deg"]
    assert summarize(askew, "near", 0, LOG, target=BOX_OBJ, source="detector")["passed"] is True


def test_target_chair_and_box_keep_the_section2_lines():
    r = summarize(walk_to(1.5, -1.0), "near", 0, LOG, target=CHAIR_OBJ, others=(BOX_OBJ,))
    assert r["passed"] is True, r["failures"]
    assert r["final_dist_chair_1"] == pytest.approx(1.118, abs=1e-3)
    assert r["final_angle_chair_1_deg"] == pytest.approx(26.6, abs=0.05)
    assert "final_dist_box_1" not in r
    touch = summarize(walk_to(2.0, -1.5), "near", 0, LOG, target=CHAIR_OBJ, others=(BOX_OBJ,))
    assert keys(touch) == ["min_clearance", "min_edge_chair_1"]          # 0.275 m from the chair's edge
    assert any("final_edge" in n for n in touch["notes"])               # recorded below 0.50 (spec §10.8)

    lap = summarize(arc(1.0), "lap", 0, ["開始：立つ"], target=BOX_OBJ, others=(CHAIR_OBJ,))
    assert lap["passed"] is True, lap["failures"]
    assert lap["winding_box_1"] == lap["winding"] and lap["min_edge_box_1"] == lap["min_edge"]
    tight = summarize(arc(1.0, radius=0.8), "lap", 0, ["開始：立つ"], target=BOX_OBJ)
    assert keys(tight) == ["min_edge"]          # the §2 box line is the same line, reported once


def test_section2_keys_unchanged_without_target():
    old = {"expect", "exit_code", "first_action", "winding_box_1", "min_edge_box_1", "min_edge_chair_1",
           "final_dist_chair_1", "final_angle_chair_1_deg", "max_tilt", "passed", "failures"}
    assert set(summarize(arc(1.0), "lap", 0, ["開始：立つ"])) == old
    assert set(summarize(walk_to(1.5, -1.0), "near", 0, LOG, source="logical")) == old   # no centre line
    with_rtf = summarize(arc(1.0), "lap", 0, ["開始：立つ"], rtf={"median": 0.8})
    assert set(with_rtf) == old | {"rtf"} and with_rtf["rtf"] == {"median": 0.8}
    # without a target, 「立つ」 stays the first action even for a run that starts standing (C1 is target mode)
    assert keys(summarize(arc(1.0), "lap", 0, ["開始：右を向く"])) == ["first_action"]


def test_watch_skips_the_child_checks():
    standing = standing_start(before_sofa(1.5))
    r = summarize(standing, "near", None, None, target=SOFA)
    assert r["passed"] is True, r["failures"]
    assert r["exit_code"] is None and r["first_action"] is None
    assert summarize(before_sofa(1.5), "near", None, None, target=SOFA)["passed"] is True   # even from lying
    far = summarize(standing_start(before_sofa(1.51)), "near", None, None, target=SOFA)
    assert keys(far) == ["final_edge"]


def test_settled():
    def at(t, x, yaw=0.0):
        return (t, x, 0.0, yaw, STAND_Z, 0.02)

    assert settled([at(0.5 * k, 0.0) for k in range(41)]) is False                 # still from the start
    walk = [at(0.5 * k, 0.1 * k) for k in range(11)]                               # 1 m in 5 s, then stops
    assert settled(walk + [at(5.0 + 0.5 * k, 1.0) for k in range(1, 21)]) is True   # 10 s still
    assert settled(walk + [at(5.0 + 0.5 * k, 1.0) for k in range(1, 19)]) is False  # 9 s still
    turning = walk + [at(5.0 + 0.5 * k, 1.0, math.radians(k)) for k in range(1, 21)]
    assert settled(turning) is False                                               # turning on the spot
    assert settled([]) is False
    short = [at(0.5 * k, 0.02 * k) for k in range(11)] + [at(5.0 + 0.5 * k, 0.2) for k in range(1, 21)]
    assert settled(short) is False                                                 # moved 0.2 < 0.3 m
    assert settled(short, moved_m=0.15) is True


def test_last_source():
    def msg(**extra):
        return {"frame": "odom", "stamp": 1.0, "objects": [], **extra}

    assert last_source([msg(source="detector"), msg(source="logical")]) == "logical"
    assert last_source([]) is None
    assert last_source([msg(source="detector"), msg()]) is None
    assert last_source([msg(source="logical"), None]) is None              # a message that was not JSON
    assert last_source([msg(source="camera")]) is None                     # not one of objects.SOURCES
    r = summarize(walk_to(0.85, 1.5), "near", 0, LOG, target=BOX_OBJ,
                  source=last_source([msg(source="logical"), msg()]))
    assert "final_dist_box_1" not in r and r["passed"] is True


def test_rtf_stats_split_walking_and_still():
    ticks = []
    for k in range(601):                          # 60 wall s: RTF 0.8 for 30 s, then 0.6
        sim = 0.08 * k if k <= 300 else 24.0 + 0.06 * (k - 300)
        ticks.append((100.0 + k / 10, sim))
    samples = [(k / 10, min(k / 10, 23.0), 0.0, 0.0, STAND_Z, 0.02) for k in range(421)]   # walks until sim 23 s
    r = rtf_stats(ticks, samples)
    assert r["windows"] == 6 and r["walking_windows"] == 3 and r["still_windows"] == 3
    assert r["walking"] == pytest.approx(0.8, abs=1e-3)
    assert r["still"] == pytest.approx(0.6, abs=1e-3)
    assert r["median"] == pytest.approx(0.7, abs=1e-3)
    assert r["window_s"] == 10.0
    empty = rtf_stats([], [])
    assert empty["windows"] == 0 and empty["median"] is None and empty["walking"] is None
    assert rtf_stats(ticks[:50], samples)["windows"] == 0                  # shorter than one window


def test_layout_name_and_targets():
    assert layout_name(None, {}) == "household"
    assert layout_name(None, {"GO2_LAYOUT": "default"}) == "default"
    assert layout_name(None, {"GO2_LAYOUT": ""}) == "household"
    assert layout_name("default", {"GO2_LAYOUT": "household"}) == "default"
    target, others = pick_target(HOUSE, "sofa_1")
    assert target == SOFA_TRUTH and "sofa_1" not in {o.id for o in others} and len(others) == 6
    with pytest.raises(ValueError, match="table_1"):           # the message lists the layout's ids
        pick_target(HOUSE, "piano_1")


# ---- accuracy (spec §11.11, §13.3; design-detector §16) ----

def household_like() -> layout.Layout:
    """The household room of spec §12.3 (sizes as the layout's world footprints), parsed like the file."""
    def entry(obj_id, model, x, y, size, detectable, accept, height=0.8):
        return {"id": obj_id, "model": model, "x": x, "y": y, "yaw_deg": 0,
                "footprint": {"offset": [0.0, 0.0], "size": list(size)}, "height": height,
                "detectable": detectable, "accept": accept}

    data = {"version": 1, "objects": [
        entry("box_1", "primitive:box", 2.5, 1.5, (0.5, 0.5), False, ["box"], 0.5),
        entry("chair_1", "primitive:chair", 2.5, -1.5, (0.45, 0.45), False, ["chair"], 0.9),
        entry("trash_can_1", "model://TrashBin", -1.5, -1.5, (0.27, 0.38), True, ["trash_can", "bucket"]),
        entry("suitcase_1", "model://Suitcase1", -1.5, 1.5, (0.6, 0.4), True, ["suitcase"]),
        entry("sofa_1", "model://Sofa", -4.4, 0.0, (1.044, 1.8), True, ["sofa"]),
        entry("lamp_1", "model://LampAndStand", 0.8, -4.4, (0.62, 0.52), True, ["lamp"]),
        entry("table_1", "model://Table", -2.4, 4.4, (1.5, 0.8), True, ["table", "desk"]),
    ]}
    folders = frozenset({"TrashBin", "Suitcase1", "Sofa", "LampAndStand", "Table"})
    return layout.parse(data, "household", folders=folders)


HOUSE = household_like()
SOFA_TRUTH = HOUSE.find("sofa_1").obj()
IN_VIEW = frozenset({"trash_can_1", "suitcase_1", "sofa_1"})    # pose B: at the start, turned to face -x


def truth_rows(visible=IN_VIEW) -> tuple[Obj, ...]:
    return tuple(replace(o, visible=o.id in visible, seen=1.0) for o in HOUSE.objs())


PERFECT = tuple(o for o in truth_rows() if o.visible)


def acc(det, truth=None, button_boxes=(), robot_xy=(0.0, 0.0), targets=(), **kw) -> dict:
    return accuracy(det, truth_rows() if truth is None else truth, HOUSE, button_boxes, robot_xy, targets, **kw)


def changed(obj_id: str, dx: float = 0.0, dy: float = 0.0, **fields) -> tuple[Obj, ...]:
    return tuple(replace(o, x=o.x + dx, y=o.y + dy, **fields) if o.id == obj_id else o for o in PERFECT)


def pair_of(result: dict, truth_id: str) -> dict:
    return next(p for p in result["pairs"] if p["truth"] == truth_id)


def det_message(rows, source="detector") -> dict:
    return format_message(rows, 1.0, source=source)


def truth_message(rows=None, name="household") -> dict:
    return {**format_message(truth_rows() if rows is None else rows, 1.0, source="logical"), "layout": name}


def test_pair_rows_is_greedy_label_agnostic_and_bounded():
    a = Obj("bench_1", "bench", -4.3, 0.1, (1.0, 1.7))
    b = Obj("trash_can_2", "trash_can", -1.45, -1.52, (0.3, 0.4))
    c = Obj("trash_can_3", "trash_can", -1.6, -1.4, (0.3, 0.4))      # farther from the trash can than b
    pairs = pair_rows([c, a, b], [SOFA_TRUTH, HOUSE.find("trash_can_1").obj()])
    assert {(d.id, t.id) for d, t in pairs} == {("bench_1", "sofa_1"), ("trash_can_2", "trash_can_1")}
    # within max(0.75, truth radius): the sofa's radius is 1.04 m, a trash can's is 0.75 m
    assert pair_rows([replace(a, x=-4.4, y=1.0)], [SOFA_TRUTH]) != []
    assert pair_rows([replace(a, x=-4.4, y=1.1)], [SOFA_TRUTH]) == []
    assert pair_rows([replace(b, x=-1.5, y=-0.8)], [HOUSE.find("trash_can_1").obj()]) != []    # 0.7 m
    assert pair_rows([replace(b, x=-1.5, y=-0.7)], [HOUSE.find("trash_can_1").obj()]) == []    # 0.8 m
    assert pair_rows([], [SOFA_TRUTH]) == [] and pair_rows([a], []) == []


def test_accuracy_perfect_rows_pass():
    r = acc(PERFECT)
    assert r["passed"] is True and r["failures"] == []
    assert r["recall"] == 1.0 and r["label_acc"] == 1.0 and r["false_pos"] == 0
    assert r["max_pos_err"] == 0.0 and r["median_pos_err"] == 0.0 and r["max_size_err"] == 0.0
    assert r["covers_all"] is True and r["button_boxes_seen"] == [] and r["missed"] == []
    assert sorted(p["truth"] for p in r["pairs"]) == sorted(IN_VIEW)
    assert r["in_scope"] == ["sofa_1", "suitcase_1", "trash_can_1"]   # the lamp and the table are not visible


def test_accuracy_a_large_position_error_fails_max_pos_err():
    r = acc(changed("suitcase_1", dx=0.4))
    assert r["max_pos_err"] == pytest.approx(0.4, abs=1e-3)
    assert pair_of(r, "suitcase_1")["pos_err"] == pytest.approx(0.4, abs=1e-3)
    assert keys(r) == ["max_pos_err"]


def test_accuracy_a_target_off_by_0_2_fails_the_target_line():
    det = changed("trash_can_1", dy=0.2)
    assert acc(det)["passed"] is True                       # 0.2 m is within the line for any object
    r = acc(det, targets=("trash_can_1",))
    assert keys(r) == ["max_pos_err_target"]
    assert pair_of(r, "trash_can_1")["target"] is True
    assert acc(PERFECT, targets=("trash_can_1", "sofa_1"))["passed"] is True
    unpaired = acc(tuple(o for o in PERFECT if o.id != "sofa_1"), targets=("sofa_1",))
    assert keys(unpaired) == ["max_pos_err_target"]          # a target nobody found fails its line


def test_accuracy_labels_count_the_accept_alternatives():
    bucket = tuple(replace(o, id="bucket_1", label="bucket") if o.id == "trash_can_1" else o for o in PERFECT)
    r = acc(bucket)
    assert r["label_acc"] == 1.0 and r["passed"] is True     # the layout accepts bucket for the trash can
    wrong = tuple(replace(o, id="box_7", label="box") if o.id == "suitcase_1" else o for o in PERFECT)
    r = acc(wrong)
    assert r["label_acc"] == pytest.approx(0.667, abs=1e-3)
    assert pair_of(r, "suitcase_1")["label_ok"] is False and pair_of(r, "suitcase_1")["id"] == "box_7"
    assert keys(r) == ["min_label_acc"]


def test_accuracy_a_missing_object_lowers_recall():
    r = acc(tuple(o for o in PERFECT if o.id != "sofa_1"))
    assert r["recall"] == pytest.approx(0.667, abs=1e-3) and r["missed"] == ["sofa_1"]
    assert r["passed"] is True
    r = acc(tuple(o for o in PERFECT if o.id == "trash_can_1"))
    assert r["recall"] == pytest.approx(0.333, abs=1e-3)
    assert keys(r) == ["min_recall"]
    nothing = acc((), truth_rows(visible=frozenset()))           # no detectable truth in range: cannot pass
    assert nothing["recall"] is None and keys(nothing) == ["min_recall"]


def test_accuracy_an_extra_row_is_a_false_positive():
    ghost = Obj("ball_1", "ball", 1.0, 3.0, (0.22, 0.22))
    r = acc(PERFECT + (ghost,))
    assert r["false_pos"] == 1 and r["false_pos_ids"] == ["ball_1"] and r["passed"] is True
    r = acc(PERFECT + (ghost, Obj("ball_2", "ball", 3.0, 0.0, (0.22, 0.22))))
    assert r["false_pos"] == 2
    assert keys(r) == ["max_false_pos"]


def test_accuracy_a_row_on_a_button_box_is_seen_not_false():
    on_box = Obj("box_2", "box", 1.02, 0.01, (0.32, 0.3))
    r = acc(PERFECT + (on_box,), button_boxes=[(1.0, 0.0, 0.3)])
    assert r["button_boxes_seen"] == ["box_2"] and r["false_pos"] == 0 and r["passed"] is True
    r = acc(PERFECT + (on_box,))            # without the box from /sim/world it is a false positive
    assert r["false_pos"] == 1 and r["button_boxes_seen"] == []


def test_accuracy_non_detectable_truth_is_out_of_recall_but_still_matches():
    truth = truth_rows(IN_VIEW | {"box_1", "table_1"})   # box_1 is not detectable; table_1 is 5.01 m away
    found = PERFECT + (HOUSE.find("box_1").obj(), HOUSE.find("table_1").obj())
    r = acc(found, truth)
    assert r["false_pos"] == 0 and r["recall"] == 1.0 and r["passed"] is True
    assert r["in_scope"] == ["sofa_1", "suitcase_1", "trash_can_1"]
    assert pair_of(r, "box_1")["in_scope"] is False and pair_of(r, "table_1")["in_scope"] is False
    assert acc(PERFECT, truth)["recall"] == 1.0              # not finding them does not lower recall
    assert acc(PERFECT, truth, range_m=6.0)["missed"] == ["table_1"]   # in range at 6 m


def test_accuracy_an_undersized_footprint_does_not_cover():
    r = acc(changed("sofa_1", size=(0.8, 1.5)))
    sofa = pair_of(r, "sofa_1")
    assert sofa["covers"] is False and sofa["fill_short_m"] == 0.0
    assert sofa["size_err"] == pytest.approx(0.3, abs=1e-3)
    assert r["covers_all"] is False
    assert r["passed"] is True                  # covers is reported (O12, C2); the size line holds
    assert covers(replace(SOFA_TRUTH, size=(0.8, 1.5)), SOFA_TRUTH, False, (0.0, 0.0)) == (False, 0.0)
    assert covers(replace(SOFA_TRUTH, size=(1.0435, 1.7995)), SOFA_TRUTH, False, (0.0, 0.0)) == (True, 0.0)


def test_accuracy_a_filled_sofa_short_on_its_far_side_covers():
    """C2: seen face-on from the start, the near (+x) side and both y sides are observed; the depth was filled
    from the vocabulary and ends 0.14 m before the real far (-x) side."""
    near, far = SOFA_EDGE_X + 0.01, -4.4 - 0.522 + 0.14
    det = Obj("sofa_1", "sofa", (near + far) / 2, 0.0, (near - far, 1.82))
    ok, short = covers(det, SOFA_TRUTH, True, (0.0, 0.0))
    assert ok is True and short == pytest.approx(0.14, abs=1e-6)
    assert covers(det, SOFA_TRUTH, False, (0.0, 0.0)) == (False, 0.0)
    shallow = replace(det, x=det.x - 0.05)                  # the observed near side misses 0.04 m
    assert covers(shallow, SOFA_TRUTH, True, (0.0, 0.0))[0] is False
    # the depth axis is y for an object to the side; seen from +y, its far side is the low one
    side = Obj("trash_can_1", "trash_can", 0.3, -3.0, (0.4, 0.4))
    ok, short = covers(Obj("trash_can_2", "trash_can", 0.3, -2.945, (0.42, 0.31)), side, True, (0.0, 0.0))
    assert ok is True and short == pytest.approx(0.1, abs=1e-6)
    r = acc(tuple(det if o.id == "sofa_1" else o for o in PERFECT), filled=frozenset({"sofa_1"}))
    pair = pair_of(r, "sofa_1")
    assert pair["covers"] is True and pair["filled"] is True and pair["fill_short_m"] == pytest.approx(0.14)
    assert r["passed"] is True and r["covers_all"] is True


def test_accuracy_a_wrong_source_is_no_data():
    for source in ("logical", "fake", None):
        code, out = accuracy_check(det_message(PERFECT, source), truth_message(), HOUSE, robot_xy=(0.0, 0.0))
        assert code == 2 and "source" in out["error"], source
    code, out = accuracy_check(det_message(PERFECT), truth_message(), HOUSE, robot_xy=(0.0, 0.0))
    assert code == 0 and out["passed"] is True
    assert accuracy_check(None, truth_message(), HOUSE, robot_xy=(0.0, 0.0))[0] == 2
    assert accuracy_check(det_message(PERFECT), None, HOUSE, robot_xy=(0.0, 0.0))[0] == 2
    assert accuracy_check(det_message(PERFECT), truth_message(), HOUSE, robot_xy=None)[0] == 2
    code, out = accuracy_check(det_message(PERFECT[:1]), truth_message(), HOUSE, robot_xy=(0.0, 0.0))
    assert code == 1 and "min_recall" in keys(out)


def test_accuracy_check_reads_world_and_status():
    on_box = Obj("box_2", "box", 1.02, 0.01, (0.32, 0.3))
    status = {"ok": True, "detector": "oracle", "still_inferences": 3, "filled": ["sofa_1"], "ids_retired": 3,
              "notes": ["box: ids used up"]}
    code, out = accuracy_check(det_message(PERFECT + (on_box,)), truth_message(), HOUSE,
                               world={"boxes": [[1.0, 0.0, 0.3]], "collided": False}, robot_xy=(0.0, 0.0),
                               targets=("sofa_1",), status=status, first_status={"ids_retired": 1})
    assert code == 0, out["failures"]
    assert out["button_boxes_seen"] == ["box_2"] and pair_of(out, "sofa_1")["filled"] is True
    assert out["ids_retired"] == 2 and out["detector"] == "oracle" and out["notes"] == ["box: ids used up"]
    assert out["still_inferences"] == 3
    code, out = accuracy_check(det_message(PERFECT), truth_message(), HOUSE, robot_xy=(0.0, 0.0))
    assert code == 0 and out["ids_retired"] is None and out["detector"] is None


def test_median_position_line():                                # pre-final review F12 (spec §13.3)
    # two of three pairs 0.3 m off: the max line (0.3 <= 0.35) holds, the median line (0.3 > 0.25) fails
    det = tuple(replace(o, x=o.x + 0.3) if o.id in ("suitcase_1", "sofa_1") else o for o in PERFECT)
    r = acc(det)
    assert r["max_pos_err"] == pytest.approx(0.3, abs=0.002) and r["median_pos_err"] == pytest.approx(0.3, abs=0.002)
    assert keys(r) == ["median_pos_err"] and r["passed"] is False


def test_resolve_pairs_the_detector_row():
    truth = (SOFA_TRUTH, HOUSE.find("trash_can_1").obj())
    det = (Obj("bench_1", "bench", -4.3, 0.1, (1.0, 1.7)), Obj("trash_can_2", "trash_can", -1.45, -1.52, (0.3, 0.4)))
    assert resolve(det, truth, "sofa_1") == "bench_1"           # label-agnostic, as accuracy pairs
    assert resolve(det, truth, "trash_can_1") == "trash_can_2"
    assert resolve(det, truth, "lamp_1") is None                # not in the truth (not seen yet)
    assert resolve((), truth, "sofa_1") is None
    assert resolve((replace(det[0], y=1.2),), truth, "sofa_1") is None   # 1.2 m off: beyond the sofa's radius
    # in logical mode the bridge's ids are the truth ids
    assert resolve(truth, truth, "sofa_1") == "sofa_1"


def test_layout_mismatch_is_no_data():
    assert layout_mismatch(truth_message(), "household") is None
    other = truth_message(name="default")
    error = layout_mismatch(other, "household")
    assert "default" in error and "household" in error and "--layout default" in error
    # the app service passes no GO2_LAYOUT, so the hint names compose run's -e (pre-final review F7)
    assert "or set GO2_LAYOUT" not in error and "-e GO2_LAYOUT=default" in error
    unnamed = {k: v for k, v in truth_message().items() if k != "layout"}
    assert layout_mismatch(unnamed, "household") is not None
    assert layout_mismatch(None, "household") is not None
    code, out = accuracy_check(det_message(PERFECT), other, HOUSE, robot_xy=(0.0, 0.0))
    assert code == 2 and "default" in out["error"]


def test_cli_arguments():
    from gazebo.tools.objects_check import _parse

    a = _parse(["run", "--expect", "near", "--target", "sofa_1", "--layout", "household", "F near_sofa_3"])
    assert (a.expect, a.target, a.layout, a.formula, a.timeout) == ("near", "sofa_1", "household",
                                                                    "F near_sofa_3", 300.0)
    a = _parse(["run", "--expect", "lap", "F around_box_1"])
    assert a.target is None and a.layout is None
    a = _parse(["watch", "--expect", "lap", "--target", "sofa_1"])
    assert (a.seconds, a.layout) == (300.0, None)
    a = _parse(["accuracy", "--target", "sofa_1", "trash_can_1"])
    assert (a.seconds, a.range, a.target) == (15.0, 5.0, ["sofa_1", "trash_can_1"])
    assert _parse(["accuracy"]).target == []
    a = _parse(["resolve", "--target", "sofa_1"])
    assert (a.target, a.timeout) == ("sofa_1", 30.0)
    for bad in (["watch", "--expect", "near"], ["resolve"], ["watch", "--expect", "near", "--target", "x",
                                                            "--seconds", "0"]):
        with pytest.raises(SystemExit):
            _parse(bad)


def odom(t, x, speed):
    """A nav_msgs/Odometry stand-in: sim stamp t, at (x, 0), moving at speed along x."""
    sec = int(t)
    v = SimpleNamespace(x=speed, y=0.0, z=0.0)
    w = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=int((t - sec) * 1e9))),
                           pose=SimpleNamespace(pose=SimpleNamespace(
                               position=SimpleNamespace(x=x, y=0.0, z=0.33),
                               orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0))),
                           twist=SimpleNamespace(twist=SimpleNamespace(linear=v, angular=w)))


def test_feed_tracks_the_still_gate():
    """accuracy and resolve wait on bridge_core.StillGate fed from odometry (the bridge's own gate)."""
    from gazebo.bridge_core import StillGate
    from gazebo.tools.objects_check import _Feed

    feed = _Feed(StillGate())
    assert feed.still() is False and feed.robot_xy() is None
    feed.on_odom(odom(1.0, 0.0, 0.3))
    assert feed.still() is False and feed.still_wall is None
    feed.on_odom(odom(1.1, 0.03, 0.0))                  # stops: the gate opens 0.3 s later (sim time)
    assert feed.still_wall is not None and feed.still() is False
    opened = feed.still_wall
    feed.on_odom(odom(1.45, 0.03, 0.0))
    assert feed.still() is True and feed.still_wall == opened and feed.robot_xy() == (0.03, 0.0)
    feed.on_odom(odom(1.5, 0.05, 0.2))                  # walks again
    assert feed.still() is False and feed.still_wall is None
    assert len(feed.samples) == 4


class Wall:
    """A settable wall clock for _Feed and ListedWait."""

    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t


def test_feed_does_not_date_a_stop_it_did_not_see():      # pre-final review F11
    from gazebo.bridge_core import StillGate
    from gazebo.tools.objects_check import _Feed

    wall = Wall(100.0)
    feed = _Feed(StillGate(), clock=wall)
    feed.on_odom(odom(5.0, 1.0, 0.0))                    # the Go2 already stands still at the first odometry
    wall.t = 101.0
    feed.on_odom(odom(5.5, 1.0, 0.0))
    assert feed.still() is True and feed.still_wall is None      # the stop came before this tool: no time for it
    wall.t = 110.0
    feed.on_odom(odom(6.0, 1.1, 0.3))                    # walks
    assert feed.still() is False and feed.still_wall is None
    wall.t = 112.0
    feed.on_odom(odom(6.5, 1.2, 0.0))                    # stops: this stop was seen
    assert feed.still_wall == 112.0


def test_resolve_times_the_listing_from_the_stop_it_saw():   # pre-final review F11, spec §13.5
    from gazebo.bridge_core import StillGate
    from gazebo.tools.objects_check import ListedWait, _Feed

    truth = truth_message()
    sofa = replace(SOFA_TRUTH, id="sofa_3", measured=True)
    unlisted, unmeasured, listed = det_message(()), det_message((replace(sofa, measured=False),)), det_message((sofa,))

    def run(odoms, messages):
        """odoms: (wall, sim t, speed); messages: (wall, payload), interleaved by wall time."""
        wall = Wall()
        feed = _Feed(StillGate(), clock=wall)
        wait = ListedWait("sofa_1", HOUSE)
        events = sorted([(w, 0, (t, v)) for w, t, v in odoms] + [(w, 1, m) for w, m in messages],
                        key=lambda e: (e[0], e[1]))
        for w, kind, item in events:
            wall.t = w
            if kind == 0:
                feed.on_odom(odom(item[0], 0.0, item[1]))
            elif wait.message(item, truth, feed.still_wall, w):
                return wait
        return wait

    walk_then_stop = [(10.0, 1.0, 0.3), (12.0, 3.0, 0.0), (13.0, 4.0, 0.0)]
    # the Go2 walks, stops at wall 12.0, the row is listed at 16.5: 4.5 s
    w = run(walk_then_stop, [(11.0, unlisted), (14.0, unmeasured), (16.5, listed)])
    assert (w.id, w.measured, w.after, w.note) == ("sofa_3", True, 4.5, None)
    # the tool started with the Go2 already still (the review's example: stopped at T, resolve's odometry at
    # T+4, the row at T+6): no number, though the row was unlisted in an earlier message
    w = run([(4.0, 3.0, 0.0), (5.0, 4.0, 0.0)], [(4.5, unlisted), (6.0, listed)])
    assert w.id == "sofa_3" and w.measured is True and w.after is None and "still" in w.note
    # the row was listed in the first message the tool got: listed before it started, no number
    w = run(walk_then_stop, [(13.5, listed)])
    assert w.id == "sofa_3" and w.after is None and "first" in w.note
    # listed while the Go2 walks (no stop to measure from)
    w = run([(10.0, 1.0, 0.3), (11.0, 2.0, 0.3)], [(10.5, unlisted), (11.5, listed)])
    assert w.after is None and w.note is not None
    # not measured yet: not done
    w = run(walk_then_stop, [(11.0, unlisted), (14.0, unmeasured)])
    assert w.id == "sofa_3" and w.measured is False and w.after is None
    # a truth message for another layout ends the wait with an error
    other = ListedWait("sofa_1", HOUSE)
    assert other.message(listed, truth_message(name="default"), 1.0, 2.0) is True and "default" in other.error


def test_resolve_waits_for_a_row_with_an_accepted_label():   # pre-final review round 2 (F11, M10)
    """The Bf procedure stops at B before F moved_forward, and at B the sofa was listed as chair (H3: 'chair
    0.734'). The tracker retires that id when it relabels the track at Bf. So resolve ends only on a paired row
    whose label the target's layout entry accepts, notes a row with another label, and times the listing from
    the stop at Bf."""
    from gazebo.bridge_core import StillGate
    from gazebo.tools.objects_check import ListedWait, _Feed

    truth = truth_message()
    as_chair = replace(SOFA_TRUTH, id="chair_1", label="chair", x=SOFA_TRUTH.x + 0.2, size=(0.6, 0.6), measured=True)
    as_sofa = replace(SOFA_TRUTH, id="sofa_1", size=(1.0, 1.9), measured=True)

    def run(events):
        wall = Wall()
        feed = _Feed(StillGate(), clock=wall)
        wait = ListedWait("sofa_1", HOUSE)
        for w, kind, item in events:
            wall.t = w
            if kind == "odom":
                feed.on_odom(odom(item[0], 0.0, item[1]))
            elif wait.message(item, truth, feed.still_wall, w):
                return w, wait
        return None, wait

    to_b = [(0.0, "odom", (1.0, 0.0)), (1.0, "msg", det_message(())),            # lying still, then unlisted
            (2.0, "odom", (2.0, 0.3)), (20.0, "odom", (20.0, 0.3)),              # stand and two left turns
            (21.0, "odom", (21.0, 0.0)),                                          # the stop at B
            (24.5, "msg", det_message((as_chair,)))]                              # the sofa listed as chair
    to_bf = [(27.0, "odom", (27.0, 0.3)), (33.0, "odom", (33.0, 0.0)),           # F moved_forward, the stop at Bf
             (36.0, "msg", det_message((as_sofa,)))]                              # relabelled: sofa_1
    end, wait = run(to_b + to_bf)
    assert end == 36.0 and (wait.id, wait.measured, wait.after, wait.note) == ("sofa_1", True, 3.0, None)
    assert len(wait.notes) == 1 and "chair_1" in wait.notes[0] and "'chair'" in wait.notes[0]
    # only the chair row: no id (never the retired chair_1), and the wait goes on
    end, wait = run(to_b)
    assert end is None and (wait.id, wait.measured, wait.after) == (None, None, None) and len(wait.notes) == 1
    # a label the entry accepts besides its own ends the wait (table_1 accepts desk)
    table = HOUSE.find("table_1").obj()
    desk = replace(table, id="desk_2", label="desk", measured=True)
    wait = ListedWait("table_1", HOUSE)
    assert wait.message(det_message((desk,)), truth_message(), None, 1.0) is True and wait.id == "desk_2"
    with pytest.raises(ValueError):
        ListedWait("lamp_9", HOUSE)
