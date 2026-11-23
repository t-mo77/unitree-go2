import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock, call

import numpy as np
import pytest
from fastapi.testclient import TestClient
from rclpy.executors import SingleThreadedExecutor

from go2_nl_control.actions import STANDING
from go2_nl_control.executor import STOP
from go2_nl_control.supervisor import CLEAR
from voice_ltl import follow

PAGE = "http://localhost:8810"


@pytest.fixture
def track(monkeypatch):
    executor = Mock(spec=SingleThreadedExecutor)
    monkeypatch.setattr(follow, "SingleThreadedExecutor", lambda: executor)
    node = Mock()
    node.create_publisher.return_value.get_subscription_count.return_value = 1
    node.count_publishers.return_value = 1
    monkeypatch.setattr(follow.rclpy, "create_node", lambda _: node)
    link = Mock()
    link.reading.return_value = SimpleNamespace(front=CLEAR)
    link.posture.return_value = STANDING
    track = follow.UTrack(link, Mock())
    track._call = Mock()
    return track


def test_follows_only_after_initial_checks_and_stops_on_loss(track):
    track._call.side_effect = ['{"enable":0}', '{"enable":false}', "", '{"enable":1}', '{"enable":0}', ""]
    owner = threading.Event()
    track.start(owner)
    track.avoidance.verify_enabled.assert_called_once()
    assert track.status(owner) == "UWB リモコンを追尾中"
    with pytest.raises(RuntimeError, match="途切れました"):
        track.status(owner)
    assert not track.stop(object())
    assert track.stop(owner)
    assert track._call.call_args_list == [
        call(follow.SWITCH_GET, {}), call(follow.IS_TRACKING, {}),
        call(follow.SWITCH_SET, {"enable": 1}), call(follow.IS_TRACKING, {}),
        call(follow.IS_TRACKING, {}), call(follow.SWITCH_SET, {"enable": 0}),
    ]


def test_lost_enable_reply_requires_cleanup_and_blocks_another_start(track):
    track._call.side_effect = ['{"enable":0}', '{"enable":0}', RuntimeError("timeout"),
                               RuntimeError("timeout"), RuntimeError("timeout"), ""]
    owner = threading.Event()
    with pytest.raises(RuntimeError, match="timeout"):
        track.start(owner)
    with pytest.raises(RuntimeError, match="解除を確認できません"):
        track.stop(owner)
    track.link.send.assert_called_once_with(STOP)
    with pytest.raises(RuntimeError, match="前回の解除"):
        track.start(object())
    assert track.stop()


def test_browser_disconnect_during_preflight_prevents_enable(track):
    owner = threading.Event()
    track.avoidance.verify_enabled.side_effect = owner.set
    track._call.side_effect = ['{"enable":0}', '{"enable":0}']
    with pytest.raises(RuntimeError, match="ブラウザ"):
        track.start(owner)
    assert track._call.call_args_list == [call(follow.SWITCH_GET, {}), call(follow.IS_TRACKING, {})]
    assert not track.stop()


def test_browser_timeout_releases_tracking(monkeypatch):
    monkeypatch.setattr(follow, "HEARTBEAT_TIMEOUT", 0.2)
    track = Mock()
    track.status.return_value = "UWB リモコンを追尾中"
    app = follow.create_follow_app(track, 8810, Mock())
    client = TestClient(app, base_url=PAGE, headers={"origin": PAGE})
    assert client.get("/").status_code == 200
    assert client.post("/api/text", json={"text": "歩いて"}).status_code == 404
    with client.websocket_connect("ws://localhost:8810/ws/follow") as socket:
        socket.send_text("start")
        assert socket.receive_json()["active"] is True
        assert socket.receive_json()["message"] == "ブラウザとの通信が途切れました"
    track.stop.assert_called_once_with(track.start.call_args.args[0])


def test_positions_are_available_before_start_and_expire(monkeypatch):
    clock = SimpleNamespace(now=5.0)
    monkeypatch.setattr(follow, "time", SimpleNamespace(monotonic=lambda: clock.now))
    positions = follow.FollowPositions.__new__(follow.FollowPositions)
    pose = SimpleNamespace(position=[2, 3], imu_state=SimpleNamespace(rpy=[0, 0, math.pi / 2]))
    remote = SimpleNamespace(distance_est=2, orientation_est=-math.pi / 2, channel=0, error_state=0)
    positions.on_pose(pose)
    positions.on_uwb(remote)

    # Project the measurement using the pose at receipt, even if the Go2 turns before the poll.
    pose.imu_state.rpy[2] = 0
    positions.on_pose(pose)
    track = Mock(spec=follow.UTrack)
    app = follow.create_follow_app(track, 8810, positions)
    client = TestClient(app, base_url=PAGE)
    response = client.get("/api/position")
    assert response.headers["cache-control"] == "no-store"
    data = response.json()
    assert data["go2"] == [2, 3, 0]
    assert data["uwb"]["position"] == pytest.approx([4, 3])
    assert data["uwb"]["channel"] == 0
    track.start.assert_not_called()

    remote.error_state = 5
    positions.on_uwb(remote)
    assert positions.snapshot()["uwb"] == {
        "channel": 0, "error": 5, "distance": None, "bearing": None, "position": None,
    }
    clock.now += 1.1
    assert positions.snapshot() == {"go2": None, "uwb": None}


# ---------- このプログラムによる追尾（FOLLOW_CONTROL=own） ----------

def test_follow_control_defaults_and_refuses_utrack_in_simulation():
    assert follow.follow_control({}) == "utrack"
    assert follow.follow_control({"SIM": "1"}) == "own"
    assert follow.follow_control({"FOLLOW_CONTROL": "own"}) == "own"
    with pytest.raises(ValueError, match="UTrack"):
        follow.follow_control({"SIM": "1", "FOLLOW_CONTROL": "utrack"})
    with pytest.raises(ValueError, match="utrack か own"):
        follow.follow_control({"FOLLOW_CONTROL": "walk"})


def test_walk_velocity_turns_first_walks_then_stops_with_hysteresis():
    cfg = follow.WalkConfig()
    # 横にいる：その場で回るだけ（左が正）
    forward, turn, walking = follow.walk_velocity(3.0, math.radians(90), False, False, cfg)
    assert forward == 0 and turn == pytest.approx(cfg.max_turn) and walking
    # 正面の遠く：最高速で前進
    forward, turn, walking = follow.walk_velocity(3.0, 0.0, False, False, cfg)
    assert forward == pytest.approx(cfg.max_speed) and turn == 0 and walking
    # 前に障害物：前進しない
    assert follow.walk_velocity(3.0, 0.0, True, True, cfg)[0] == 0
    # 近づくほど遅く、止まる距離で止まる
    slow = follow.walk_velocity(1.2, 0.0, False, True, cfg)[0]
    assert 0 < slow < cfg.max_speed
    assert follow.walk_velocity(0.9, 0.0, False, True, cfg)[:3] == (0.0, 0.0, False)
    # 止まった後は、行き違いの幅を越えるまで歩き出さない
    assert follow.walk_velocity(1.2, 0.0, False, False, cfg)[2] is False
    assert follow.walk_velocity(1.4, 0.0, False, False, cfg)[2] is True
    # 止まっていても、大きくずれたら向きは直す
    assert follow.walk_velocity(0.8, math.radians(-40), False, False, cfg)[1] < 0


def test_ramp_limits_speeding_up_only():
    assert follow._ramp_up(0.0, 0.4, 0.06) == pytest.approx(0.06)
    assert follow._ramp_up(0.3, 0.0, 0.06) == 0.0
    assert follow._ramp_up(0.5, -0.5, 0.2) == -0.5
    assert follow._ramp_up(-0.1, -0.5, 0.2) == pytest.approx(-0.3)


@pytest.fixture
def walker():
    link = Mock()
    link.reading.return_value = SimpleNamespace(front=CLEAR, front_blocked=False)
    link.posture.return_value = STANDING
    link.pose.return_value = (0.0, 0.0, 0.0)
    positions = Mock()
    positions.measurement.return_value = (3.0, 0.0)
    positions.obstacles.return_value = None
    return follow.WalkToUwb(link, positions)


def test_walker_sends_ramped_moves_and_stops_when_uwb_is_lost(walker):
    owner = threading.Event()
    walker.start(owner)
    walker.halt.set()            # 自動の周期を止め、tick を手で回す
    walker.thread.join()
    walker._tick(0.1)
    move = walker.link.send.call_args.args[0]
    assert move.api_id == follow.MOVE and move.parameter == {"x": 0.06, "y": 0.0, "z": 0.0}
    assert "歩いています" in walker.status(owner)
    walker.positions.measurement.return_value = None
    walker._tick(0.1)
    assert walker.link.send.call_args.args[0] == STOP
    assert "見失いました" in walker.status(owner)
    walker.link.send.reset_mock()
    walker._tick(0.1)            # 止まったままなら送り直さない
    walker.link.send.assert_not_called()
    assert walker.stop(owner)
    walker.link.send.assert_called_once_with(STOP)
    assert walker.status(owner) is None


def test_walker_refuses_without_uwb_and_reports_faults(walker):
    walker.positions.measurement.return_value = None
    with pytest.raises(RuntimeError, match="測位"):
        walker.start(threading.Event())
    walker.positions.measurement.return_value = (3.0, 0.0)
    owner = threading.Event()
    walker.start(owner)
    walker.link.posture.return_value = "LYING"
    walker.thread.join(timeout=2)
    with pytest.raises(RuntimeError, match="立たせて"):
        walker.status(owner)
    walker.link.send.assert_called_with(STOP)
    assert walker.stop(owner)


def test_fake_uwb_measures_relative_to_go2_and_rejects_far_places():
    from sim.fake_uwb import UwbTag
    tag = UwbTag(noise_distance=0, noise_bearing=0)
    assert tag.measure((0, 0, 0)) is None
    tag.place_ahead((1, 1, math.pi / 2))
    assert tag.position == pytest.approx((1, 3))
    tag.place_ahead((0, 0, 0))   # すでに置いてあれば動かさない
    distance, bearing = tag.measure((1, 1, 0))
    assert distance == pytest.approx(2) and bearing == pytest.approx(math.pi / 2)
    with pytest.raises(ValueError):
        tag.place(100, 0)
    with pytest.raises(ValueError):
        tag.place(float("nan"), 0)


def test_positions_measurement_drops_stale_and_errors(monkeypatch):
    clock = SimpleNamespace(now=5.0)
    monkeypatch.setattr(follow, "time", SimpleNamespace(monotonic=lambda: clock.now))
    positions = follow.FollowPositions.__new__(follow.FollowPositions)
    positions.pose = None
    positions.on_uwb(SimpleNamespace(distance_est=2, orientation_est=0.5, channel=0, error_state=0))
    assert positions.measurement() == pytest.approx((2, 0.5))
    clock.now += follow.UWB_FRESH_S + 0.1
    assert positions.measurement() is None
    positions.on_uwb(SimpleNamespace(distance_est=2, orientation_est=0.5, channel=0, error_state=3))
    assert positions.measurement() is None


def test_simulation_endpoints():
    sim = Mock()
    sim.place.return_value = (1.0, 2.0)
    app = follow.create_follow_app(Mock(), 8810, Mock(), "own", sim)
    client = TestClient(app, base_url=PAGE, headers={"origin": PAGE})
    assert client.get("/api/config").json() == {"control": "own", "sim": True, "camera": False}
    assert client.post("/api/sim/uwb", json={"x": 1, "y": 2}).json() == {"position": [1.0, 2.0]}
    sim.place.side_effect = ValueError("遠すぎます")
    assert client.post("/api/sim/uwb", json={"x": 99, "y": 0}).status_code == 400
    assert client.post("/api/sim/stand").status_code == 200
    real = TestClient(follow.create_follow_app(Mock(), 8810, Mock()), base_url=PAGE, headers={"origin": PAGE})
    assert real.get("/api/config").json() == {"control": "utrack", "sim": False, "camera": False}
    assert real.get("/api/camera").status_code == 204
    assert real.post("/api/sim/uwb", json={"x": 1, "y": 2}).status_code in (404, 405)


def test_spinning_link_keeps_receiving_and_reports_background_errors():
    link = Mock()
    received = threading.Event()
    link.spin_once.side_effect = lambda: received.set()
    spinning = follow.SpinningLink(link, period=0.01)
    try:
        assert received.wait(1)               # 誰も呼ばなくても受信を処理する
        assert spinning.posture is link.posture
        link.spin_once.side_effect = RuntimeError("別の送り手")
        for _ in range(100):
            if spinning._error is not None:
                break
            threading.Event().wait(0.01)
        link.spin_once.side_effect = None
        with pytest.raises(RuntimeError, match="別の送り手"):
            spinning.spin_once()
    finally:
        spinning.close()
    spinning.spin_once()                       # 伝えた異常は消える


def test_walker_detours_around_obstacles_and_stops_when_boxed_in(walker):
    from tests.test_avoid import square
    walker.positions.obstacles.return_value = square(1.5, 0.1)
    walker.walking = True
    forward, turn = walker._steer(3.0, 0.0)
    assert turn < 0 and walker.side == -1 and "右によけて" in walker.message
    walker.avoid = follow.AvoidConfig(max_detour=math.radians(60))   # 三方を壁に囲まれた
    walker.positions.obstacles.return_value = np.array(
        [(0.6, y) for y in np.arange(-3, 3, 0.05)]
        + [(x, side) for x in np.arange(-3, 0.6, 0.05) for side in (0.5, -0.5)])
    assert walker._steer(3.0, 0.0) == (0.0, 0.0)
    assert "道が見つかりません" in walker.message
    walker.positions.obstacles.return_value = None   # 点群が無ければ（Go2 本体の回避に任せ）まっすぐ
    forward, turn = walker._steer(3.0, 0.0)
    assert forward > 0 and turn == 0 and walker.side == 0


def test_obstacles_on_the_map_in_odometry_with_nearest_and_front_free(monkeypatch):
    from tests.test_avoid import square
    clock = SimpleNamespace(now=5.0)
    monkeypatch.setattr(follow, "time", SimpleNamespace(monotonic=lambda: clock.now))
    positions = follow.FollowPositions.__new__(follow.FollowPositions)
    positions.pose = (clock.now, (1.0, 2.0, math.pi / 2))   # 北を向いている
    positions.uwb = None
    positions.perception = object()
    positions.cloud = (clock.now, square(2.0, 0.0), positions._recent_pose(clock.now))
    obstacles = positions.snapshot()["obstacles"]
    assert obstacles["front_free"] == pytest.approx(2.0 - 0.225 - 0.35)
    assert obstacles["nearest"] == pytest.approx(1.775, abs=0.01)
    assert obstacles["nearest_bearing"] == pytest.approx(0, abs=0.03)
    xs, ys = zip(*obstacles["points"])
    # 機体の前 2 m の椅子は、オドメトリでは (1, 4) のまわり
    assert min(ys) == pytest.approx(3.8, abs=0.06) and max(ys) == pytest.approx(4.2, abs=0.06)
    assert min(xs) == pytest.approx(0.8, abs=0.06) and max(xs) == pytest.approx(1.2, abs=0.06)
    clock.now += 1
    assert positions.snapshot()["obstacles"] is None


def test_walker_reports_its_heading_for_the_map(walker):
    from tests.test_avoid import square
    walker.link.pose.return_value = (0.0, 0.0, 1.0)
    walker.positions.obstacles.return_value = square(1.5, 0.1)
    walker.walking = True
    assert walker.plan() is None        # 追尾していなければ出さない
    walker.owner = threading.Event()
    walker._steer(3.0, 0.0)
    assert walker.plan() < 1.0          # 右へよける向き
    walker._steer(0.5, 0.0)             # 着いたら消える
    assert walker.plan() is None
