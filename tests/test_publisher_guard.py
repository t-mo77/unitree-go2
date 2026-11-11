from types import SimpleNamespace

from voice_ltl.publisher_guard import BARE_DDS, QUIET_SECONDS, PublisherGuard


def endpoint(index, name=BARE_DDS):
    return SimpleNamespace(node_name=name, endpoint_gid=bytes([index]) * 24)


class Node:
    def __init__(self, endpoints):
        self.endpoints = endpoints

    def get_publishers_info_by_topic(self, topic):
        return self.endpoints


def test_idle_native_publishers_do_not_block_but_ros_publisher_does():
    node = Node([endpoint(i) for i in range(10)])
    guard = PublisherGuard()
    guard.freeze(node)
    assert guard.conflicts(node) == 0
    node.endpoints.append(endpoint(12, "keyboard_velocity"))
    assert guard.conflicts(node) == 1


def test_actual_native_request_blocks_until_quiet_then_allows_new_commands():
    node = Node([endpoint(1)])
    now = [0.0]
    guard = PublisherGuard(clock=lambda: now[0])
    guard.freeze(node)
    guard.on_request(None, SimpleNamespace(publisher_gid=bytes([1]) * 24))
    assert guard.conflicts(node) == 1
    guard.freeze(node)  # freezeで活動した送り手を再登録しない
    assert guard.conflicts(node) == 1
    now[0] += QUIET_SECONDS + 0.1
    assert guard.conflicts(node) == 0
    # 再度送信し始めれば、すぐに競合へ戻る。
    guard.on_request(None, SimpleNamespace(publisher_gid=bytes([1]) * 24))
    assert guard.conflicts(node) == 1


def test_repeated_foreign_commands_do_not_expire_while_active():
    node = Node([endpoint(1)])
    now = [0.0]
    guard = PublisherGuard(clock=lambda: now[0])
    guard.freeze(node)
    for _ in range(10):
        guard.on_request(None, SimpleNamespace(publisher_gid=bytes([1]) * 24))
        now[0] += QUIET_SECONDS / 2
        assert guard.conflicts(node) == 1


def test_active_request_before_discovery_is_not_ignored():
    guard = PublisherGuard()
    guard.on_request(None, SimpleNamespace(publisher_gid=bytes([1]) * 24))
    guard.freeze(Node([endpoint(1)]))
    assert guard.conflicts(Node([endpoint(1)])) == 1


def test_new_native_publisher_is_not_automatically_allowed():
    node = Node([endpoint(1)])
    guard = PublisherGuard()
    guard.freeze(node)
    node.endpoints.append(endpoint(2))
    assert guard.conflicts(node) == 1


def test_own_publisher_and_messages_are_not_conflicts():
    node = Node([endpoint(1), endpoint(2, "go2_voice_ltl")])
    guard = PublisherGuard(own=1)
    guard.freeze(node)
    guard.on_request(None, SimpleNamespace(publisher_gid=bytes([2]) * 24))
    assert guard.conflicts(node) == 0


def request(number, api=1003):
    return SimpleNamespace(header=SimpleNamespace(identity=SimpleNamespace(id=number, api_id=api)),
                           parameter="")


def test_humble_single_argument_callback_accepts_own_but_blocks_foreign_requests():
    now = [0.0]
    guard = PublisherGuard(own=1, clock=lambda: now[0])
    node = Node([endpoint(1), endpoint(2, "go2_voice_ltl")])
    guard.freeze(node)
    own = request(1000000)
    guard.remember_own(own)
    guard.on_request(own)  # 古いHumbleと同じ、infoがない呼び出し
    assert guard.conflicts(node) == 0
    guard.on_request(request(0))
    assert guard.conflicts(node) > 0
    now[0] += QUIET_SECONDS + 0.1
    assert guard.conflicts(node) == 0


def test_matching_api_without_matching_request_id_is_not_our_command():
    guard = PublisherGuard()
    guard.remember_own(request(1234))
    guard.on_request(request(5678))
    assert guard.unknown_active()


def test_onboard_switch_get_polling_is_not_a_competing_command():
    now = [0.0]
    guard = PublisherGuard(topic="/api/obstacles_avoid/request", read_only_apis=(1002,),
                           activity_only=True, clock=lambda: now[0])
    node = Node([endpoint(1)])
    guard.freeze(node)
    for _ in range(64):
        guard.on_request(request(0, api=1002))
        now[0] += .08
        assert guard.conflicts(node) == 0
    # 状態照会用のDDS/ROS送り手が増えても操作がなければ競合にしない。
    node.endpoints.extend([endpoint(2), endpoint(3, "avoidance_status_reader")])
    assert guard.conflicts(node) == 0


def test_onboard_foreign_moves_and_setting_changes_still_stop():
    node = Node([endpoint(1)])
    for api in (1001, 1003, 1004):
        now = [0.0]
        guard = PublisherGuard(read_only_apis=(1002,), activity_only=True, clock=lambda: now[0])
        guard.freeze(node)
        guard.on_request(request(0, api=api))
        assert guard.conflicts(node) == 1
        assert guard.last_foreign_api == api
        now[0] += QUIET_SECONDS + .1
        assert guard.conflicts(node) == 0


def test_switch_get_exception_is_not_applied_to_sport_balance_stand():
    node = Node([endpoint(1)])
    guard = PublisherGuard()  # Sport APIの1002はBalanceStandであり、照会ではない
    guard.freeze(node)
    guard.on_request(request(0, api=1002))
    assert guard.conflicts(node) == 1
