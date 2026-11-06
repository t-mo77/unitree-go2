"""Gazebo の物（logical camera・box_1 と chair_1・/go2/objects）の、ROS を使わない部品（Task D）。"""

import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from gazebo.bridge_core import (OBJECT_SPAWN_Z, SIZE_BOX_GRACE_S, SIZE_BOX_MARGIN, BoxWorld,
                                ObjectMemory, compose, drop_box_points, footprint_size,
                                lidar_points_world, object_label, object_sdf, quat_to_matrix)
from sim.sim_core import BOX_HEIGHT, Box
from voice_ltl.grid import NEAR_MAX_DISTANCE, NEAR_MAX_MARGIN, NEAR_MIN_MARGIN
from voice_ltl.objects import (CAMERA_FAR, CAMERA_HFOV, CAMERA_HZ, CAMERA_IN_BODY, CAMERA_NEAR,
                               DEFAULT_LAYOUT, DEFAULT_SIZE, parse)

ROOT = Path(__file__).resolve().parent.parent
XACRO = ROOT / "gazebo" / "go2_sensors.urdf.xacro"
LAUNCH = ROOT / "gazebo" / "launch" / "go2_gazebo.launch.py"
README = ROOT / "README.md"
DESIGN = ROOT / "docs" / "superpowers" / "specs" / "2026-11-05-go2-objects-design.md"


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


def side_points(cx, cy, sx, sy, top, bottom=0.1, step=0.05):
    """The points a LiDAR can hit on the four sides of an axis-aligned box, every 0.1 m from bottom to top."""
    us, vs = np.arange(-sx / 2, sx / 2 + 1e-9, step), np.arange(-sy / 2, sy / 2 + 1e-9, step)
    ring = ([(cx + u, cy + v) for u in us for v in (-sy / 2, sy / 2)]
            + [(cx + u, cy + v) for u in (-sx / 2, sx / 2) for v in vs])
    return np.array([(x, y, z) for x, y in ring for z in np.arange(bottom, top + 1e-9, 0.1)])


def test_drop_box_points_keeps_what_is_outside_a_sim_box():
    box = Box(1.0, 2.0)                      # 0.3 m, 0.4 m high, axis-aligned like box_sdf
    h = box.size / 2 + SIZE_BOX_MARGIN
    pts = np.array([[1.0, 2.0, 0.2], [1.0 + h - 0.01, 2.0, 0.1], [1.0, 2.0 - h + 0.01, BOX_HEIGHT],
                    [1.0 + h + 0.01, 2.0, 0.2], [1.0, 2.0 + h + 0.01, 0.2],
                    [1.0, 2.0, BOX_HEIGHT + SIZE_BOX_MARGIN + 0.01]])   # above the box: a taller object's points
    assert drop_box_points(pts, [box]).tolist() == pts[3:].tolist()
    assert drop_box_points(pts, []).tolist() == pts.tolist()
    assert drop_box_points(np.empty((0, 3)), [box]).shape == (0, 3)


def test_a_sim_box_beside_box_1_leaves_its_size_unchanged():
    m = ObjectMemory()
    m.update([("box_1", "box", 2.5, 1.5)], 10.0)
    box_1 = side_points(2.5, 1.5, 0.5, 0.5, top=0.5)
    m.set_size("box_1", footprint_size(box_1, 2.5, 1.5))
    assert m.objects()[0].size == pytest.approx((0.5, 0.5), abs=0.03)
    w = BoxWorld()
    _, sim = w.place("front", x=3.0, y=0.3, yaw=math.pi / 2)   # 「前に箱を置く」: 0.1 m from box_1's side
    assert (sim.x, sim.y) == pytest.approx((3.0, 1.5))
    frame = np.vstack([box_1, side_points(sim.x, sim.y, sim.size, sim.size, top=BOX_HEIGHT)])
    assert footprint_size(frame, 2.5, 1.5)[0] > 1.0                # unfiltered, box_1 would grow for good
    m.set_size("box_1", footprint_size(drop_box_points(frame, w.sizing_boxes(11.0)), 2.5, 1.5))
    assert m.objects()[0].size == pytest.approx((0.5, 0.5), abs=0.03)


def test_a_box_put_in_front_at_the_chair_keeps_its_near_band():   # Review Focus 4
    chair = DEFAULT_LAYOUT[1]
    m = ObjectMemory()
    m.update([(chair.id, chair.label, chair.x, chair.y)], 10.0)
    points = np.vstack([side_points(chair.x + dx, chair.y + dy, 0.04, 0.04, top=0.4)        # legs
                        for dx in (-0.2, 0.2) for dy in (-0.2, 0.2)]
                       + [side_points(chair.x, chair.y, 0.45, 0.45, top=0.43, bottom=0.43),  # seat
                          side_points(chair.x - 0.205, chair.y, 0.04, 0.45, top=0.9, bottom=0.5)])  # back
    m.set_size(chair.id, footprint_size(points, chair.x, chair.y))
    before = m.objects()[0].size
    assert before == pytest.approx((0.45, 0.45), abs=0.03)
    w = BoxWorld()
    # 「椅子まで歩いて」 ended in the near band (1.07-1.3 m, heading within 40°; the executor stops up to
    # ~0.15 m off the cell centre, so up to 1.4 m), then 「前に箱を置く」
    for dist, yaw in ((1.4, 0.0), (1.1, 0.0), (1.2, math.radians(30)), (1.3, math.radians(-25))):
        _, sim = w.place("front", x=chair.x - dist, y=chair.y, yaw=yaw)
        frame = np.vstack([points, side_points(sim.x, sim.y, sim.size, sim.size, top=BOX_HEIGHT)])
        grown = footprint_size(frame, chair.x, chair.y)
        assert max(grown) > max(before) + 0.04, (dist, yaw)          # unfiltered, the chair would grow
        m.set_size(chair.id, footprint_size(drop_box_points(frame, w.sizing_boxes(11.0)), chair.x, chair.y))
        assert m.objects()[0].size == pytest.approx(before), (dist, yaw)
    r = math.hypot(*m.objects()[0].size) / 2
    # the near band stays open: the cells 0.5·√5 m (1.118 m) from the chair stay inside it (R4, ruling N1)
    assert r + NEAR_MIN_MARGIN < 0.5 * math.sqrt(5) <= min(r + NEAR_MAX_MARGIN, NEAR_MAX_DISTANCE)


def test_sim_boxes_stay_out_of_sizing_until_they_are_gone_from_the_cloud():
    w = BoxWorld()
    n1, b1 = w.place("front", x=0.0, y=0.0, yaw=0.0)   # asked for: Gazebo may draw it before the answer
    n2, b2 = w.place("back", x=0.0, y=0.0, yaw=0.0)
    assert w.sizing_boxes(0.0) == [b1, b2]
    assert w.world(5.0, 5.0)["boxes"] == []            # /sim/world still shows only the placed boxes
    w.add(n1, b1)
    w.failed(n2)
    assert w.sizing_boxes(1.0) == [b1]
    assert w.world(5.0, 5.0)["boxes"] == [[1.2, 0.0, 0.3]]
    assert w.clear(now=10.0) == [n1] and w.world(5.0, 5.0)["boxes"] == []
    assert w.sizing_boxes(10.0 + SIZE_BOX_GRACE_S - 0.1) == [b1]   # removal is asynchronous; clouds in flight
    assert w.sizing_boxes(10.0 + SIZE_BOX_GRACE_S + 0.1) == []


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


def test_the_readme_echo_shows_the_whole_objects_message():
    """README §3: `ros2 topic echo` cuts strings at 128 characters unless --full-length is given
    (Humble ros2topic/verb/echo.py). The default /go2/objects message is longer, so chair_1 would not show."""
    from ros2topic.verb.echo import DEFAULT_TRUNCATE_LENGTH
    from rosidl_runtime_py import message_to_yaml
    from std_msgs.msg import String
    m = ObjectMemory()
    m.update([(o.id, o.label, o.x, o.y) for o in DEFAULT_LAYOUT], 1_762_400_000.123)   # before any LiDAR size
    data = json.dumps(m.message(1_762_400_000.5))                     # as gazebo_bridge._publish_objects
    assert len(data) > DEFAULT_TRUNCATE_LENGTH
    assert "chair_1" not in message_to_yaml(String(data=data), truncate_length=DEFAULT_TRUNCATE_LENGTH)
    assert "chair_1" in message_to_yaml(String(data=data), truncate_length=None)    # what --full-length prints
    commands = [line.split() for line in README.read_text(encoding="utf-8").splitlines()
                if "ros2 topic echo" in line and "/go2/objects" in line]
    assert commands
    for words in commands:
        assert "--full-length" in words or "-f" in words, " ".join(words)


def test_the_docs_state_the_gazebo_lap_result():
    """Design §2 is still not met in Gazebo: on the final code (a2fe140: M1, corners, the turn ended
    before the walk and the slower speeds of C2, the near band to 1.3 m of ruling N1) the lap passed
    2 of 5 and the chair 3 of 3 (docs/reference/gazebo_objects_e2e.md, part 4 above the older parts
    and conclusions). README §4 and §8 and design §12 say so, name the Gazebo controller's sway as
    the cause and a controller patch as the open route, and that the rest is undecided; README
    does not read as if the grid executor were checked in Gazebo."""
    readme = README.read_text(encoding="utf-8")
    assert "偽の Go2 と Gazebo でしか確かめていません" not in readme
    assert "確かめた範囲では転んでいません。" not in readme.replace("物を使わない指示で確かめた範囲では", "")
    objects = readme[readme.index("### 物を使う指示（箱と椅子）"):readme.index("## 5.")]
    lap_row = next(line for line in objects.splitlines() if line.startswith("| 箱の周りを一周して |"))
    assert "Gazebo ではまだ毎回は通らない" in lap_row and "5 回のうち 2 回" in lap_row and "§8" in lap_row
    assert "Gazebo では、設計書（物）§2 の合格の条件をまだ満たしていません" in objects
    near_row = next(line for line in objects.splitlines() if line.startswith("| `near_<id>` |"))
    assert "1.3 m" in near_row and "1.4 m" not in near_row
    unknown = readme[readme.index("## 8. わかっていないこと"):]
    gazebo = unknown[unknown.index("- **Gazebo での物を使う指示"):unknown.index("- **物を使う指示の実機での動き")]
    for words in ("まだ満たせていません", "5 回のうち 2 回", "椅子までは 3/3", "31 回のうち 1 回", "unitree_guide",
                  "パッチ", "まだ決めていません", "gazebo_objects_e2e.md", "第 4 部", "椅子まで 1.28 m"):
        assert words in gazebo, words
    design = DESIGN.read_text(encoding="utf-8")
    results = design[design.index("**確かめた結果**"):]
    for words in ("§2 の合格の条件は、Gazebo ではまだ満たせていない", "一周 2/5", "椅子まで 3/3", "25 回", "終えたのは 1 回",
                  "10 回のうち 7 回合格", "unitree_guide", "まだ決めていないこと（利用者の判断を待つ）"):
        assert words in results, words
    assert "min(物の半径 + 1.5 m, 1.3 m)" in design and "min(物の半径 + 1.5 m, 1.4 m)" not in design
    record = (ROOT / "docs" / "reference" / "gazebo_objects_e2e.md").read_text(encoding="utf-8")
    assert record.index("## 第 4 部") < record.index("## 第 3 部") < record.index("## 結論")
    assert "**最後のコード（`a2fe140`）で一周 2/5・椅子 3/3・挨拶つき 0/1、操作画面の一周 1/3 とそこからの椅子まで 1/1。**" in record
    assert "終わって 1 秒（`TAIL_S`）しか見ない" not in record                  # TAIL_S is 3 s since 6bf599c
