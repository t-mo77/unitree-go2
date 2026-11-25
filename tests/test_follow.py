import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from fastapi.testclient import TestClient

from go2_nl_control.actions import BALANCE_STAND, LYING, STAND_UP, STANDING
from go2_nl_control.executor import STOP
from go2_nl_control.supervisor import CLEAR
from voice_ltl import follow

PAGE = "http://localhost:8810"


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
    pose = SimpleNamespace(position=[2, 3], imu_state=SimpleNamespace(rpy=[0, 0, math.pi / 2]),
                           velocity=[0, 0, 0], yaw_speed=0)
    remote = SimpleNamespace(distance_est=2, orientation_est=-math.pi / 2, channel=0, error_state=0)
    positions.on_pose(pose)
    positions.on_uwb(remote)

    # Project the measurement using the pose at receipt, even if the Go2 turns before the poll.
    pose.imu_state.rpy[2] = 0
    positions.on_pose(pose)
    track = Mock(spec=follow.WalkToUwb)
    track.plan.return_value = None
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
    # 到着後は方位がずれても旋回しない
    assert follow.walk_velocity(0.8, math.radians(-40), False, False, cfg) == (0.0, 0.0, False)


def test_ramp_limits_speeding_up_only():
    assert follow._ramp_up(0.0, 0.4, 0.06) == pytest.approx(0.06)
    assert follow._ramp_up(0.3, 0.0, 0.06) == 0.0
    assert follow._ramp_up(0.5, -0.5, 0.2) == -0.5
    assert follow._ramp_up(-0.1, -0.5, 0.2) == pytest.approx(-0.3)


@pytest.fixture
def walker():
    link = Mock()
    link.obstacle_authority = "pc"
    link.reading.return_value = SimpleNamespace(front=CLEAR, front_blocked=False)
    link.posture.return_value = STANDING
    link.pose.return_value = (0.0, 0.0, 0.0)
    positions = Mock()
    positions.measurement.return_value = (3.0, 0.0)
    positions.obstacles.return_value = np.empty((0, 2))
    return follow.WalkToUwb(link, positions)


def test_walker_sends_ramped_moves_and_stops_when_uwb_is_lost(walker):
    owner = threading.Event()
    walker.start(owner)
    walker.halt.set()            # 自動の周期を止め、tick を手で回す
    walker.thread.join()
    walker.halt.clear()
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
    assert [args.args[0] for args in walker.link.send.call_args_list] == [STOP] * 3
    assert walker.status(owner) is None


def test_walker_requires_fresh_lidar_to_start_and_keep_moving(walker):
    owner = threading.Event()
    walker.positions.obstacles.return_value = None
    with pytest.raises(RuntimeError, match="LiDAR"):
        walker.start(owner)
    walker.link.send.assert_not_called()

    walker.positions.obstacles.return_value = np.empty((0, 2))
    walker.start(owner)
    walker.halt.set()
    walker.thread.join()
    walker.halt.clear()
    walker._tick(0.1)
    assert walker.link.send.call_args.args[0].api_id == follow.MOVE

    walker.positions.obstacles.return_value = None
    walker._tick(0.1)
    assert walker.link.send.call_args.args[0] == STOP
    assert "LiDAR" in walker.status(owner)
    assert walker.stop(owner)


def test_onboard_follow_leaves_obstacle_decisions_to_go2_and_stops_on_uwb_loss(walker):
    link, positions = walker.link, walker.positions
    link.obstacle_authority = "go2"
    positions.obstacles.return_value = None
    link.reading.return_value = SimpleNamespace(front=follow.SENSOR_LOST, front_blocked=True)
    track = follow.WalkToUwb(link, positions)
    owner = threading.Event()
    track.start(owner)
    track.halt.set()
    track.thread.join()
    track.halt.clear()

    track._tick(0.1)
    move = link.send.call_args.args[0]
    assert move.api_id == follow.MOVE and move.parameter["x"] > 0
    positions.obstacles.assert_not_called()
    link.reading.assert_not_called()

    positions.measurement.return_value = None
    track._tick(0.1)
    assert link.send.call_args.args[0] == STOP
    assert track.stop(owner)


def test_real_follow_refuses_to_start_when_onboard_avoidance_is_unavailable(monkeypatch):
    monkeypatch.setenv("SIM", "0")
    link = Mock()
    link.prepare_onboard_avoidance.side_effect = RuntimeError("本体の障害物回避が無効です")
    ros_link = Mock(return_value=link)
    run = Mock()
    monkeypatch.setattr(follow, "RosLink", ros_link)
    monkeypatch.setattr(follow.uvicorn, "run", run)

    with pytest.raises(SystemExit, match="障害物回避が無効"):
        follow.main()
    assert ros_link.call_args.kwargs["obstacle_authority"] == "go2"
    link.prepare_onboard_avoidance.assert_called_once()
    link.close.assert_called_once_with(stop_times=0)
    run.assert_not_called()


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


def test_stop_prevents_a_pending_move(walker):
    computing = threading.Event()
    release = threading.Event()
    sent_stop = threading.Event()

    def measurement():
        computing.set()
        release.wait(1)
        return 3.0, 0.0

    walker.positions.measurement.side_effect = [(3.0, 0.0), None]
    walker.start(threading.Event())
    walker.positions.measurement.side_effect = measurement
    walker.link.send.side_effect = lambda cmd: sent_stop.set() if cmd == STOP else None
    stopping = threading.Thread(target=walker.stop)
    try:
        assert computing.wait(1)
        stopping.start()
        assert sent_stop.wait(1)
    finally:
        release.set()
        if stopping.ident is not None:
            stopping.join(timeout=2)
        else:
            walker.stop()
    assert not stopping.is_alive()
    assert [args.args[0] for args in walker.link.send.call_args_list] == [STOP] * 3


def test_stop_rejects_start_from_an_older_connection(walker):
    stop_epoch = walker.stop_epoch
    walker.stop()
    with pytest.raises(RuntimeError, match="停止しています"):
        walker.start(threading.Event(), stop_epoch)
    assert walker.owner is None


def test_fails_when_rotating_without_progress(walker):
    walker.positions.measurement.return_value = (3.0, math.pi / 2)
    walker.turn_budget_used = follow.MAX_TURN_WITHOUT_PROGRESS
    walker.progress_distance = 3.0
    with pytest.raises(RuntimeError, match="旋回"):
        walker._tick(0.1)
    walker.link.send.assert_not_called()


def test_stand_uses_stand_up_then_balance_stand(walker, monkeypatch):
    monkeypatch.setitem(follow.ACTIONS, "stand",
                        SimpleNamespace(sequence=((STAND_UP, 0), (BALANCE_STAND, 0))))
    walker.link.posture.side_effect = [LYING, STANDING]
    assert walker.stand() == "Go2 が立ちました"
    assert [args.args[0].api_id for args in walker.link.send.call_args_list] == [STAND_UP, BALANCE_STAND]


def test_motion_telemetry_must_report_fresh_stationary_samples(monkeypatch):
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(follow, "time", SimpleNamespace(monotonic=lambda: clock.now))
    positions = follow.FollowPositions.__new__(follow.FollowPositions)
    pose = SimpleNamespace(position=[0, 0], imu_state=SimpleNamespace(rpy=[0, 0, 0]),
                           velocity=[0, 0, 0], yaw_speed=0.4)
    positions.on_pose(pose)
    assert not positions.stationary_after(0)
    pose.yaw_speed = 0
    clock.now = 0.1
    positions.on_pose(pose)
    clock.now = 0.45
    positions.on_pose(pose)
    assert positions.stationary_after(0)
    clock.now += follow.STATE_TIMEOUT + 0.1
    assert not positions.stationary_after(0)


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
    track = Mock()
    track.stand.return_value = "Go2 が立ちました"
    app = follow.create_follow_app(track, 8810, Mock(), sim=sim)
    client = TestClient(app, base_url=PAGE, headers={"origin": PAGE})
    assert client.get("/api/config").json() == {"control": "own", "sim": True, "camera": False}
    assert client.post("/api/sim/uwb", json={"x": 1, "y": 2}).json() == {"position": [1.0, 2.0]}
    sim.place.side_effect = ValueError("遠すぎます")
    assert client.post("/api/sim/uwb", json={"x": 99, "y": 0}).status_code == 400
    assert client.post("/api/sim/stand").status_code == 200
    real = TestClient(follow.create_follow_app(track, 8810, Mock()), base_url=PAGE, headers={"origin": PAGE})
    assert real.get("/api/config").json() == {"control": "own", "sim": False, "camera": False}
    assert real.get("/api/camera").status_code == 204
    assert real.post("/api/stand").json() == {"message": "Go2 が立ちました"}
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
    forward, turn = walker._steer(3.0, 0.0, walker.positions.obstacles())
    assert turn < 0 and walker.side == -1 and "右によけて" in walker.message
    walker.avoid = follow.AvoidConfig(max_detour=math.radians(60))   # 三方を壁に囲まれた
    walker.positions.obstacles.return_value = np.array(
        [(0.6, y) for y in np.arange(-3, 3, 0.05)]
        + [(x, side) for x in np.arange(-3, 0.6, 0.05) for side in (0.5, -0.5)])
    assert walker._steer(3.0, 0.0, walker.positions.obstacles()) == (0.0, 0.0)
    assert "道が見つかりません" in walker.message
    walker.positions.obstacles.return_value = np.empty((0, 2))   # 点群から障害物が検出されなければまっすぐ
    forward, turn = walker._steer(3.0, 0.0, walker.positions.obstacles())
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
    walker._steer(3.0, 0.0, walker.positions.obstacles())
    assert walker.plan() < 1.0          # 右へよける向き
    walker._steer(0.5, 0.0, walker.positions.obstacles())             # 着いたら消える
    assert walker.plan() is None
