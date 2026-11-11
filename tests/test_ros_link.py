"""RosLink's safety mechanisms, without starting ROS 2 (TD-5).

rclpy's node, publisher and executor are replaced by fakes, so nothing goes on the network. What
these fakes cannot show (real DDS discovery of other senders, StopMove reaching the Go2) is checked
by hand on the robot (docs/real_robot.md).
"""

import fcntl
import json
import signal
import time

import numpy as np
import pytest
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from unitree_go.msg import SportModeState

from go2_nl_control.actions import MOVE, STANDING, STOP_MOVE, UNKNOWN
from go2_nl_control.executor import Command
from go2_nl_control.perception import PerceptionConfig
from go2_nl_control.supervisor import BLOCKED, CLEAR, SENSOR_LOST

from voice_ltl import ros_link
from voice_ltl.ros_link import REQUEST_TOPIC, STATE_TIMEOUT, RosLink


class FakePub:
    def __init__(self):
        self.msgs, self.sigint_while_sending, self.subscribers, self.fail = [], [], 1, False

    def publish(self, msg):
        if self.fail:
            raise RuntimeError("publisher is gone")
        self.sigint_while_sending.append(signal.getsignal(signal.SIGINT))
        self.msgs.append(msg)

    def get_subscription_count(self):
        return self.subscribers


class FakeNode:
    def __init__(self, name):
        self.name, self.pub, self.publishers, self.destroyed = name, FakePub(), {}, False
        self.subscriptions = []

    def create_publisher(self, msg_type, topic, qos):
        self.publishers[topic] = 1   # this program itself
        return self.pub

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions.append(topic)

    def count_publishers(self, topic):
        return self.publishers.get(topic, 0)

    def destroy_node(self):
        self.destroyed = True


class FakeExecutor:
    def __init__(self):
        self.shut = False

    def add_node(self, node):
        pass

    def spin_once(self, timeout_sec=0.0):
        pass

    def shutdown(self):
        self.shut = True


class FakeRclpy:
    def __init__(self, ok=True):
        self._ok, self.init_kwargs, self.nodes, self.shutdowns = ok, [], [], 0

    def ok(self):
        return self._ok

    def init(self, **kwargs):
        self.init_kwargs.append(kwargs)
        self._ok = True

    def create_node(self, name):
        self.nodes.append(FakeNode(name))
        return self.nodes[-1]

    def try_shutdown(self):
        self.shutdowns += 1


SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)


@pytest.fixture
def ros(monkeypatch, tmp_path):
    """Fake rclpy pieces, a lock file of this test's own, and the signal handlers put back after."""
    fake = FakeRclpy()
    monkeypatch.setattr(ros_link, "rclpy", fake)
    monkeypatch.setattr(ros_link, "SingleThreadedExecutor", FakeExecutor)
    monkeypatch.setattr(ros_link, "LOCK_FILE", str(tmp_path / "go2_voice_ltl.lock"))
    saved = {s: signal.getsignal(s) for s in SIGNALS}
    links = []

    def make(perception=None, **kw):
        links.append(RosLink(perception or PerceptionConfig(), **kw))
        return links[-1]
    yield fake, make
    for link in links:
        link.close(0)   # unregisters its atexit close; sends nothing
    for s, handler in saved.items():
        signal.signal(s, handler)


def _lock_is_free(path) -> bool:
    with open(path, "w") as f:
        try:
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False
        fcntl.flock(f, fcntl.LOCK_UN)
        return True


def _state(error_code=1013, x=0.0):
    msg = SportModeState()
    msg.error_code = error_code
    msg.position[0] = x
    return msg


def test_second_instance_is_refused_by_the_lock_before_ros_starts(ros):
    fake, make = ros
    first = make()
    with pytest.raises(RuntimeError, match="すでに動いています"):
        make()
    assert len(fake.nodes) == 1   # the second never became a node, so it is no sender either
    first.close(0)
    assert _lock_is_free(ros_link.LOCK_FILE)
    make()                        # and once the first has closed, a new one starts


def test_lock_is_released_when_ros_setup_fails(ros, monkeypatch):
    fake, make = ros

    def broken(name):
        raise RuntimeError("no ROS")
    monkeypatch.setattr(fake, "create_node", broken)
    with pytest.raises(RuntimeError, match="no ROS"):
        make()
    assert _lock_is_free(ros_link.LOCK_FILE)


def test_ros_signal_handling_is_off_and_owned_rclpy_is_shut_down(ros):
    fake, make = ros
    fake._ok = False                      # nobody started rclpy: RosLink starts and stops it
    link = make()
    assert fake.init_kwargs == [{"signal_handler_options": SignalHandlerOptions.NO}]
    link.close(0)
    assert fake.shutdowns == 1


def test_sigterm_and_sighup_become_keyboard_interrupt(ros):
    _, make = ros
    make()
    for sig in (signal.SIGTERM, signal.SIGHUP):
        # checked first, so a missing handler fails here instead of killing pytest
        assert signal.getsignal(sig) is ros_link._raise_keyboard_interrupt
        with pytest.raises(KeyboardInterrupt):
            signal.raise_signal(sig)


def test_wait_ready_refuses_when_another_program_sends(ros, monkeypatch):
    _, make = ros
    link = make()
    monkeypatch.setattr(link, "_spin_for", lambda sec: None)
    link._on_state(_state())
    node = ros_link.rclpy.nodes[-1]
    assert link.other_senders() == 0
    link.wait_ready(timeout_s=0.1)        # only this program: fine
    node.publishers[REQUEST_TOPIC] = 2    # one more sender on /api/sport/request
    assert link.other_senders() == 1
    with pytest.raises(RuntimeError, match="別のプログラム"):
        link.wait_ready(timeout_s=0.1)
    assert node.pub.msgs == []            # refusing sends nothing


def test_wait_ready_says_go2_not_found_without_a_subscriber_or_state(ros, monkeypatch):
    _, make = ros
    link = make()
    monkeypatch.setattr(link, "_spin_for", lambda sec: None)
    ros_link.rclpy.nodes[-1].pub.subscribers = 0
    with pytest.raises(RuntimeError, match="見つかりません"):
        link.wait_ready(timeout_s=0.05)


def test_close_sends_stopmove_three_times_ignoring_ctrl_c_then_cleans_up_once(ros):
    _, make = ros
    link = make()
    node = ros_link.rclpy.nodes[-1]
    before = signal.getsignal(signal.SIGINT)
    link.close()                                           # the default: 3 times
    assert [m.header.identity.api_id for m in node.pub.msgs] == [STOP_MOVE] * 3
    assert [m.header.identity.id for m in node.pub.msgs] == [1, 2, 3]
    assert node.pub.sigint_while_sending == [signal.SIG_IGN] * 3
    assert signal.getsignal(signal.SIGINT) is before       # put back afterwards
    assert node.destroyed and link._executor.shut and _lock_is_free(ros_link.LOCK_FILE)
    link.close()                                           # second time: nothing
    assert len(node.pub.msgs) == 3


def test_close_zero_sends_nothing(ros):
    _, make = ros
    link = make()
    link.close(0)
    assert ros_link.rclpy.nodes[-1].pub.msgs == []


def test_close_still_cleans_up_when_sending_fails(ros, capsys):
    _, make = ros
    link = make()
    node = ros_link.rclpy.nodes[-1]
    node.pub.fail = True
    link.close()
    assert "StopMove を送れませんでした" in capsys.readouterr().out
    assert node.destroyed and _lock_is_free(ros_link.LOCK_FILE)


def test_close_runs_at_exit_until_called(ros, monkeypatch):
    calls = []
    monkeypatch.setattr(ros_link.atexit, "register", lambda f: calls.append(("register", f)))
    monkeypatch.setattr(ros_link.atexit, "unregister", lambda f: calls.append(("unregister", f)))
    _, make = ros
    link = make()
    link.close(0)
    assert calls == [("register", link.close), ("unregister", link.close)]


def test_send_numbers_requests_and_puts_the_parameter_as_json(ros):
    _, make = ros
    link = make()
    link.send(Command(MOVE, {"x": 0.3, "y": 0.0, "z": 0.0}))
    msg = ros_link.rclpy.nodes[-1].pub.msgs[-1]
    assert msg.header.identity.api_id == MOVE and msg.header.identity.id == 1
    assert json.loads(msg.parameter) == {"x": 0.3, "y": 0.0, "z": 0.0}


def test_reading_is_sensor_lost_until_sportmodestate_arrives_and_when_it_goes_stale(ros):
    _, make = ros
    link = make(PerceptionConfig(lidar_x=0.28, lidar_z=0.10, lidar_pitch=0.0))
    now = time.time()
    # a box 0.4 m ahead of the body centre, 0.1 m up, in that mount's LiDAR frame
    box = np.column_stack([np.full(20, 0.4 - 0.28), np.linspace(-0.1, 0.1, 20), np.zeros(20)])
    link._sensing.update(box, now)                 # a box 0.4 m ahead, as RosLink's _on_cloud does
    r = link.reading(now)
    assert (r.front, r.back) == (SENSOR_LOST, SENSOR_LOST) and r.front_dist == pytest.approx(0.4)
    link._on_state(_state())
    assert link._state_time == pytest.approx(time.time(), abs=1.0)
    link._state_time = now
    r = link.reading(now + STATE_TIMEOUT - 0.1)
    assert (r.front, r.back) == (BLOCKED, CLEAR)
    late = now + STATE_TIMEOUT + 0.1
    link._sensing.update(box, late)                # the LiDAR still comes ...
    assert link._sensing.current(late).front == BLOCKED
    r = link.reading(late)                         # ... but /sportmodestate stopped 0.6 s ago
    assert (r.front, r.back) == (SENSOR_LOST, SENSOR_LOST) and r.front_dist == pytest.approx(0.4)


def test_posture_and_pose_come_from_sportmodestate(ros):
    _, make = ros
    link = make()
    assert (link.posture(), link.pose(), link.error_code()) == (UNKNOWN, None, None)
    link._on_state(_state(error_code=1013, x=1.5))
    assert link.posture() == STANDING and link.error_code() == 1013 and link.pose()[0] == 1.5
    link._on_state(_state(error_code=1002))        # StandUp's locked stand: not walkable yet
    assert link.posture() == UNKNOWN


def test_objects_come_from_go2_objects_and_go_stale(ros, monkeypatch):
    _, make = ros
    link = make()
    assert ros_link.OBJECTS_TOPIC in link.node.subscriptions and link.objects() == []
    monkeypatch.setattr(ros_link.time, "time", lambda: 100.0)
    msg = {"frame": "odom", "stamp": 1.0, "objects": [
        {"id": "box_1", "label": "box", "x": 2.5, "y": 1.5, "size": [0.5, 0.5], "seen": 1.0, "visible": True},
        {"id": "ground_plane", "label": "box", "x": 0, "y": 0}]}
    link._on_objects(String(data=json.dumps(msg)))
    link._on_objects(String(data="not json"))
    link._on_objects(String(data="[" * 100000))     # too deeply nested for json: ignored, not raised
    link._on_objects(String(data="[1, 2]"))         # JSON, but not a dict
    assert [r["id"] for r in link.objects()] == ["box_1"]
    monkeypatch.setattr(ros_link.time, "time", lambda: 102.1)
    assert link.objects() == []
