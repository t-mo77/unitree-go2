from xml.etree import ElementTree

from voice_ltl import local, server


def test_fake_is_local_and_isolated_from_robot(monkeypatch, tmp_path):
    monkeypatch.setattr(local, "ROOT", tmp_path)
    (tmp_path / "env.txt").write_text("LLM_BACKEND=codex\nGO2_IFACE=enp3s0\n")
    env = local.environment("fake", {"PATH": "/usr/bin", "ROS_DOMAIN_ID": "0"})
    assert env["SIM"] == "1"
    assert env["GO2_OBSTACLE_AUTHORITY"] == "pc"
    assert env["ROS_DOMAIN_ID"] == "73"
    config = ElementTree.fromstring(env["CYCLONEDDS_URI"])
    assert config.find(".//NetworkInterface").get("name") == "lo"
    assert config.find(".//AllowMulticast").text == "false"
    assert config.find(".//Peer").get("Address") == "127.0.0.1"
    assert config.find(".//ParticipantIndex").text == "auto"
    assert server.listen_address(env) == ("127.0.0.1", 8810)


def test_robot_uses_selected_interface_and_domain_zero(monkeypatch, tmp_path):
    monkeypatch.setattr(local, "ROOT", tmp_path)
    (tmp_path / "env.txt").write_text("LLM_BACKEND=codex\nGO2_IFACE=enp3s0\nGO2_UI_PORT=9123\n")
    env = local.environment("robot", {"PATH": "/usr/bin", "ROS_DOMAIN_ID": "73"})
    assert env["SIM"] == "0"
    assert env["GO2_OBSTACLE_AUTHORITY"] == "go2"
    assert env["ROS_DOMAIN_ID"] == "0"
    config = ElementTree.fromstring(env["CYCLONEDDS_URI"])
    assert config.find(".//NetworkInterface").get("name") == "enp3s0"
    assert server.listen_address(env) == ("127.0.0.1", 9123)
