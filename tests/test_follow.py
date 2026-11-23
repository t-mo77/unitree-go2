import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock, call

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
    track = Mock()
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
