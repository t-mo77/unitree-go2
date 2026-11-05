"""Gazebo の物（logical camera・box_1 と chair_1・/go2/objects）の、ROS を使わない部品（Task D）。"""

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from gazebo.bridge_core import (OBJECT_SPAWN_Z, ObjectMemory, compose, footprint_size,
                                lidar_points_world, object_label, object_sdf, quat_to_matrix)
from voice_ltl.objects import (CAMERA_FAR, CAMERA_HFOV, CAMERA_HZ, CAMERA_IN_BODY, CAMERA_NEAR,
                               DEFAULT_LAYOUT, DEFAULT_SIZE, parse)

ROOT = Path(__file__).resolve().parent.parent
XACRO = ROOT / "gazebo" / "go2_sensors.urdf.xacro"
LAUNCH = ROOT / "gazebo" / "launch" / "go2_gazebo.launch.py"


def quat_yaw(t):
    return (math.cos(t / 2), 0.0, 0.0, math.sin(t / 2))


def test_compose_puts_the_camera_model_in_the_world():    # map-gazebo §3, measured in Gazebo
    world = compose((-0.687, 0.097, 0.373), quat_yaw(0.3), (2.0612, -0.7388, -0.1730))
    assert world == pytest.approx((1.5, 0.0, 0.2), abs=2e-3)
    assert quat_to_matrix(quat_yaw(math.pi / 2)) @ np.array([1, 0, 0]) == pytest.approx([0, 1, 0], abs=1e-9)


@pytest.mark.parametrize("name,label", [("box_1", "box"), ("chair_12", "chair"), ("ground_plane", None),
                                        ("robot", None), ("sim_box_3", None), ("box_0", None),
                                        ("Box_1", None), ("box_100", None)])
def test_only_listed_names_are_objects(name, label):
    assert object_label(name) == label


def test_lidar_points_go_to_the_world():
    pts = lidar_points_world(np.array([[1.0, 0.0, 0.0]]), (2.0, 3.0, 0.33), quat_yaw(math.pi / 2))
    assert pts.tolist()[0] == pytest.approx([2.0, 4.28, 0.43])


def test_footprint_size_of_a_box_and_too_few_points():
    side = np.arange(-0.25, 0.2501, 0.05)
    ring = ([(3 + u, 1 + v) for u in side for v in (-0.25, 0.25)]
            + [(3 + v, 1 + u) for u in side for v in (-0.25, 0.25)])
    pts = np.array([(x, y, z) for x, y in ring for z in (0.1, 0.3)] + [(9, 9, 0.2), (3, 1, 0.01)])
    assert footprint_size(pts, 3.0, 1.0) == pytest.approx((0.5, 0.5), abs=0.03)
    assert footprint_size(pts[:9], 3.0, 1.0) is None


def test_memory_remembers_flags_visibility_and_keeps_the_largest_size():
    m = ObjectMemory()
    m.update([("box_1", "box", 2.5, 1.5)], 10.0)
    (o,) = m.objects()
    assert (o.visible, o.seen, o.size) == (True, 10.0, DEFAULT_SIZE)
    m.update([], 11.0)
    (o,) = m.objects()
    assert (o.visible, o.seen) == (False, 10.0)
    m.set_size("box_1", (0.4, 0.6))
    m.set_size("box_1", (0.45, 0.5))
    m.set_size("nope_1", (1, 1))
    assert m.objects()[0].size == (0.45, 0.6)
    assert parse(m.message(12.0)) == tuple(m.objects())


def test_object_sdf_is_static_xml_of_the_right_size():
    box, chair = (ET.fromstring(object_sdf(o.id, o.label)) for o in DEFAULT_LAYOUT)
    assert box.find("model").get("name") == "box_1" and box.find("model/static").text == "true"
    assert box.find(".//collision/geometry/box/size").text == "0.5 0.5 0.5"
    assert {c.get("name") for c in chair.iter("collision")} >= {"seat", "leg_fl", "leg_fr", "leg_rl", "leg_rr", "back"}
    for model in (box, chair):   # sdformat refuses a link whose collisions and visuals share a name
        names = [e.get("name") for e in model.find("model/link") if e.tag in ("collision", "visual")]
        assert len(names) == len(set(names))
        assert ([c.find("geometry/box/size").text for c in model.iter("collision")]
                == [v.find("geometry/box/size").text for v in model.iter("visual")])   # visuals equal collisions


def test_objects_stand_on_the_floor_with_the_layout_footprint():   # R23
    for o in DEFAULT_LAYOUT:
        lo, hi = [], []
        for c in ET.fromstring(object_sdf(o.id, o.label)).iter("collision"):
            centre = np.array([float(v) for v in c.find("pose").text.split()[:3]])
            half = np.array([float(v) for v in c.find("geometry/box/size").text.split()]) / 2
            lo.append(centre - half)
            hi.append(centre + half)
        spawn = np.array([0.0, 0.0, OBJECT_SPAWN_Z[o.label]])
        lo, hi = np.min(lo, axis=0) + spawn, np.max(hi, axis=0) + spawn
        assert lo[2] == pytest.approx(0.0), o.id                       # neither floating nor sunk
        assert tuple(hi[:2] - lo[:2]) == pytest.approx(o.size), o.id   # the footprint the fake Go2 uses
        assert hi[2] == pytest.approx({"box": 0.5, "chair": 0.9}[o.label]), o.id   # chair back up to 0.9 m


def test_xacro_mounts_the_logical_camera_like_the_real_front_camera():
    text = XACRO.read_text(encoding="utf-8")
    joint = re.search(r'<joint name="logical_camera_joint".*?<origin xyz="([^"]+)" rpy="([^"]+)"', text, re.S)
    assert tuple(float(v) for v in joint.group(1).split()) == CAMERA_IN_BODY
    assert (CAMERA_NEAR, CAMERA_FAR, CAMERA_HFOV, CAMERA_HZ) == (0.1, 8.0, 2.094, 5.0)   # the values below
    for tag, value in (("near", "0.1"), ("far", "8.0"), ("horizontal_fov", "2.094"),
                       ("aspect_ratio", "1.778"), ("update_rate", "5"), ("topic", "gazebo/logical_camera")):
        assert f"<{tag}>{value}</{tag}>" in text, tag
    assert 'filename="gz-sim-logical-camera-system"' in text


def test_launch_bridges_the_logical_camera():
    assert ('"/gazebo/logical_camera@ros_gz_interfaces/msg/LogicalCameraImage[gz.msgs.LogicalCameraImage"'
            in LAUNCH.read_text(encoding="utf-8"))
