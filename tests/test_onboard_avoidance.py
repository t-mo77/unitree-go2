import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from unitree_api.msg import Response

from voice_ltl import onboard_avoidance as oa


class Node:
    def __init__(self):
        self.messages = []
        self.client = None
        self.enabled = True
        self.reject = None
        self.pub = SimpleNamespace(publish=self.publish, get_subscription_count=lambda: 2)

    def publish(self, message):
        self.messages.append(message)
        if not message.header.policy.noreply:
            response = Response()
            response.header.identity.id = message.header.identity.id
            response.header.identity.api_id = message.header.identity.api_id
            response.header.status.code = 1 if message.header.identity.api_id == self.reject else 0
            response.data = json.dumps({"enable": self.enabled})
            self.client._on_response(response)

    def create_publisher(self, *args):
        return self.pub

    def create_subscription(self, *args):
        pass

    def get_publishers_info_by_topic(self, topic):
        return [SimpleNamespace(node_name="go2_onboard_avoidance", endpoint_gid=bytes([1]) * 24)]

    def destroy_node(self):
        pass


@pytest.fixture
def client(monkeypatch):
    monkeypatch.setattr(oa, "SingleThreadedExecutor", Mock)
    node = Node()
    instance = oa.OnboardAvoidance(node=node, start_thread=False)
    node.client = instance
    yield instance, node
    instance.close()


def test_verify_enabled_then_acquire_api_and_send_correct_avoidance_velocity(client):
    instance, node = client
    instance.prepare()
    instance.move({"x": .2, "y": -.1, "z": .3})
    assert [m.header.identity.api_id for m in node.messages] == [oa.SWITCH_GET, oa.USE_API, oa.MOVE]
    assert json.loads(node.messages[-1].parameter) == {"x": .2, "y": -.1, "yaw": .3, "mode": 0}
    assert node.messages[-1].header.policy.noreply
    instance.stop()
    assert json.loads(node.messages[-1].parameter) == {"x": 0., "y": 0., "yaw": 0., "mode": 0}


def test_disabled_onboard_avoidance_never_acquires_or_moves(client):
    instance, node = client
    node.enabled = False
    with pytest.raises(RuntimeError, match="無効"):
        instance.prepare()
    with pytest.raises(RuntimeError):
        instance.move({"x": .2, "y": 0., "z": 0.})
    assert [m.header.identity.api_id for m in node.messages] == [oa.SWITCH_GET]


def test_rejection_does_not_fall_back_to_sport_move(client):
    instance, node = client
    node.reject = oa.USE_API
    with pytest.raises(RuntimeError, match="拒否"):
        instance.prepare()
    with pytest.raises(RuntimeError):
        instance.move({"x": .2, "y": 0., "z": 0.})
    assert oa.MOVE not in [m.header.identity.api_id for m in node.messages]
    node.reject = None


def test_close_sends_zero_velocity_and_releases_api_ownership(client):
    instance, node = client
    instance.prepare()
    instance.close()
    assert [m.header.identity.api_id for m in node.messages[-2:]] == [oa.MOVE, oa.USE_API]
    assert json.loads(node.messages[-1].parameter) == {"is_remote_commands_from_api": False}


def test_robot_status_poll_does_not_block_our_next_move(client):
    from unitree_api.msg import Request
    instance, node = client
    instance.prepare()
    other = Request()
    other.header.identity.api_id = oa.SWITCH_GET
    instance.guard.on_request(other)
    instance.move({"x": .1, "y": 0., "z": 0.})
    assert node.messages[-1].header.identity.api_id == oa.MOVE
    # 同じ相手の移動指令は照会とは区別して拒否する。
    other.header.identity.api_id = oa.MOVE
    instance.guard.on_request(other)
    with pytest.raises(RuntimeError):
        instance.move({"x": .1, "y": 0., "z": 0.})
