"""gazebo/tools/objects_check.py の合否の計算（summarize ほか）。ROS 2 も Gazebo も使わない。"""

import math
from types import SimpleNamespace

import pytest

from gazebo.tools.objects_check import THRESHOLDS, odometry_sample, summarize, valid_message
from voice_ltl.objects import DEFAULT_LAYOUT, Obj

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
                          "all": {"max_tilt": 1.0, "first_action": "立つ", "exit_code": 0}}


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
