"""voice_ltl.robot_check: the wired real-Go2 check (README section 6). ROS is never started here."""

import pytest

from voice_ltl import ros_link
from voice_ltl.robot_check import (BAD, OK, WARN, Check, IfaceInfo, LIDAR_TOPIC, REQUEST_TOPIC,
                                   STATE_TOPIC, check_iface, check_settings, check_topics,
                                   config_from_args, exit_code, format_check, read_iface)

IFACE = "enp3s0"
GOOD_ENV = {
    "SIM": "0",
    "GO2_IFACE": IFACE,
    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
    "CYCLONEDDS_URI": '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp3s0" '
                      'priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>',
}
GOOD_IFACE = IfaceInfo(up=True, running=True, multicast=True,
                       address="192.168.123.99", netmask="255.255.255.0")


def _levels(checks):
    return [c.level for c in checks]


def _text(checks):
    return "\n".join(c.text for c in checks)


def test_topic_names_are_the_apps():
    assert (STATE_TOPIC, LIDAR_TOPIC, REQUEST_TOPIC) == \
        (ros_link.STATE_TOPIC, ros_link.LIDAR_TOPIC, ros_link.REQUEST_TOPIC)


def test_good_settings_pass_and_name_domain_zero():
    checks = check_settings(GOOD_ENV)
    assert set(_levels(checks)) == {OK}
    assert "ROS_DOMAIN_ID 0" in _text(checks)
    # an explicit 0 is the same as unset
    assert set(_levels(check_settings({**GOOD_ENV, "ROS_DOMAIN_ID": " 0 "}))) == {OK}


def test_simulation_compose_is_refused():
    """SIM=1 means compose.yaml (bridge network): the Go2's LAN port is not in the container."""
    checks = check_settings({**GOOD_ENV, "SIM": "1"})
    assert BAD in _levels(checks)
    assert "compose.robot.yaml" in _text(checks)


@pytest.mark.parametrize("value", ["", "   "])
def test_missing_iface_is_bad_and_says_where_to_write_it(value):
    checks = check_settings({**GOOD_ENV, "GO2_IFACE": value})
    assert BAD in _levels(checks)
    assert "env.txt" in _text(checks) and "GO2_IFACE" in _text(checks)


def test_other_domain_id_is_bad():
    checks = check_settings({**GOOD_ENV, "ROS_DOMAIN_ID": "78"})
    assert BAD in _levels(checks)
    assert "78" in _text(checks) and "0" in _text(checks)


def test_other_rmw_is_bad():
    checks = check_settings({**GOOD_ENV, "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"})
    assert BAD in _levels(checks)
    assert "rmw_cyclonedds_cpp" in _text(checks)


def test_uri_must_name_the_iface_and_is_never_printed():
    env = {**GOOD_ENV, "CYCLONEDDS_URI": GOOD_ENV["CYCLONEDDS_URI"].replace("enp3s0", "eth9")}
    checks = check_settings(env)
    assert BAD in _levels(checks)
    assert "eth9" not in _text(checks) and "<CycloneDDS>" not in _text(checks)
    assert BAD in _levels(check_settings({k: v for k, v in GOOD_ENV.items() if k != "CYCLONEDDS_URI"}))


def test_good_iface():
    checks = check_iface(IFACE, GOOD_IFACE)
    assert _levels(checks) == [OK]
    assert "192.168.123.99/24" in _text(checks)


@pytest.mark.parametrize("info, words", [
    (None, ["見つかりません", "compose.robot.yaml"]),
    (IfaceInfo(False, False, True, "192.168.123.99", "255.255.255.0"), ["オフ"]),
    (IfaceInfo(True, False, True, "192.168.123.99", "255.255.255.0"), ["ケーブル", "電源"]),
    (IfaceInfo(True, True, False, "192.168.123.99", "255.255.255.0"), ["マルチキャスト"]),
    (IfaceInfo(True, True, True, None, None), ["IPv4", "192.168.123.99/24"]),
    (IfaceInfo(True, True, True, "192.168.1.20", "255.255.255.0"), ["192.168.123.0/24"]),
    (IfaceInfo(True, True, True, "192.168.123.161", "255.255.255.0"), ["192.168.123.161", "Go2 本体"]),
])
def test_bad_iface(info, words):
    checks = check_iface(IFACE, info)
    assert BAD in _levels(checks)
    for w in words:
        assert w in _text(checks)


def test_wider_mask_is_only_a_warning():
    checks = check_iface(IFACE, IfaceInfo(True, True, True, "192.168.123.99", "255.255.0.0"))
    assert BAD not in _levels(checks) and WARN in _levels(checks)


def test_everything_arrives():
    checks = check_topics(5.0, n_state=2500, n_cloud=75, error_code=1001, request_subs=1, request_pubs=0)
    assert set(_levels(checks)) == {OK}
    text = _text(checks)
    assert "500 Hz" in text and "15 Hz" in text and "1001（伏せ）" in text


def test_no_state_is_bad_and_points_at_troubleshooting():
    checks = check_topics(5.0, n_state=0, n_cloud=0, error_code=None, request_subs=0, request_pubs=0)
    assert _levels(checks).count(BAD) == 3      # state, cloud, no receiver
    assert STATE_TOPIC in _text(checks) and "困ったとき" in _text(checks)


def test_unknown_posture_is_a_warning():
    checks = check_topics(5.0, n_state=10, n_cloud=10, error_code=1002, request_subs=1, request_pubs=0)
    assert BAD not in _levels(checks) and WARN in _levels(checks)
    assert "1002（不明）" in _text(checks)


def test_missing_lidar_is_bad():
    checks = check_topics(5.0, n_state=10, n_cloud=0, error_code=1001, request_subs=1, request_pubs=0)
    assert _levels(checks).count(BAD) == 1
    assert LIDAR_TOPIC in _text(checks)


def test_no_receiver_names_the_motion_service():
    checks = check_topics(5.0, n_state=10, n_cloud=10, error_code=1001, request_subs=0, request_pubs=0)
    assert _levels(checks).count(BAD) == 1
    assert REQUEST_TOPIC in _text(checks) and "mcf" in _text(checks)


def test_other_sender_is_bad_because_the_server_would_refuse():
    checks = check_topics(5.0, n_state=10, n_cloud=10, error_code=1001, request_subs=1, request_pubs=2)
    assert _levels(checks).count(BAD) == 1
    assert "2 つ" in _text(checks) and "断ります" in _text(checks)


def test_format_and_exit_code():
    assert format_check(Check(OK, "a")) == "［OK］ a"
    assert format_check(Check(BAD, "b")) == "［だめ］ b"
    assert format_check(Check(WARN, "c")) == "［注意］ c"
    assert exit_code([Check(OK, "a"), Check(WARN, "c")]) == 0
    assert exit_code([Check(OK, "a"), Check(BAD, "b")]) == 1


def test_seconds_option():
    assert config_from_args([]) == 5.0
    assert config_from_args(["--seconds", "2.5"]) == 2.5
    with pytest.raises(SystemExit):
        config_from_args(["--seconds", "0"])


def test_main_stops_before_ros_when_settings_are_bad(monkeypatch):
    import voice_ltl.robot_check as rc

    def no_ros(_seconds):
        raise AssertionError("ROS 2 must not start")

    monkeypatch.setattr(rc, "listen", no_ros)
    lines: list[str] = []
    assert rc.main([], env={**GOOD_ENV, "GO2_IFACE": ""}, out=lines.append) == 1
    assert any(line.startswith("［だめ］") and "GO2_IFACE" in line for line in lines)
    assert "ROS 2 はまだ起動していません" in lines[-1]


def test_read_iface_on_loopback_and_on_a_missing_name():
    """Linux only (the container): lo is up and has 127.0.0.1/8; a made-up name is None."""
    lo = read_iface("lo")
    assert lo is not None and lo.up and lo.address == "127.0.0.1" and lo.netmask == "255.0.0.0"
    assert read_iface("go2-no-such-if") is None
