"""Gazebo の物（logical camera・配置ファイルの物・/go2/objects と /gazebo/objects_truth）の、ROS を使わない部品
（Task D、open-objects Task 2.3）。"""

import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

from gazebo import bridge_core
from gazebo.bridge_core import (CARDBOARD_MESH_SIDE, OBJECT_SPAWN_Z, OBJECTS_SOURCES, SIZE_BOX_GRACE_S,
                                SIZE_BOX_MARGIN, SIZE_RADIUS, TRUTH_TOPIC, BoxWorld, ObjectMemory, box_sdf,
                                bridge_settings, compose, drop_box_points, footprint_size, layout_name, lidar_points_world,
                                logical_rows, missing_models, object_label, object_sdf, objects_publisher_note,
                                objects_source, objects_topics, quat_to_matrix, spawn_fields)
from sim.sim_core import BOX_HEIGHT, Box
from voice_ltl import layout
from voice_ltl.grid import NEAR_MAX_DISTANCE, NEAR_MAX_MARGIN, NEAR_MIN_MARGIN
from voice_ltl.objects import (CAMERA_FAR, CAMERA_HFOV, CAMERA_HZ, CAMERA_IN_BODY, CAMERA_NEAR,
                               DEFAULT_LAYOUT, DEFAULT_SIZE, OBJECTS_TOPIC, measure_radius, parse)

ROOT = Path(__file__).resolve().parent.parent
XACRO = ROOT / "gazebo" / "go2_sensors.urdf.xacro"
LAUNCH = ROOT / "gazebo" / "launch" / "go2_gazebo.launch.py"
README = ROOT / "README.md"
# README keeps the commands only; the details moved to these docs
SIMULATION = ROOT / "docs" / "simulation.md"
VOCABULARY = ROOT / "docs" / "vocabulary.md"
UNKNOWNS = ROOT / "docs" / "unknowns.md"
USER_DOCS = (README, ROOT / "docs" / "overview.md", ROOT / "docs" / "setup.md", SIMULATION, VOCABULARY,
             ROOT / "docs" / "real_robot.md", UNKNOWNS)
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
    assert (o.visible, o.seen, o.size, o.measured) == (True, 10.0, DEFAULT_SIZE, False)   # box: [0.5, 0.5]
    m.update([], 11.0)
    (o,) = m.objects()
    assert (o.visible, o.seen) == (False, 10.0)
    m.set_size("box_1", (0.4, 0.6))
    m.set_size("box_1", (0.45, 0.5))
    m.set_size("nope_1", (1, 1))
    assert m.objects()[0].size == (0.45, 0.6) and m.objects()[0].measured is True
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
    for e in layout.load("default").entries:
        sdf, x, y, z, yaw = spawn_fields(e)
        assert (x, y, yaw) == (e.x, e.y, 0.0), e.id
        lo, hi = [], []
        for c in ET.fromstring(sdf).iter("collision"):
            centre = np.array([float(v) for v in c.find("pose").text.split()[:3]])
            half = np.array([float(v) for v in c.find("geometry/box/size").text.split()]) / 2
            lo.append(centre - half)
            hi.append(centre + half)
        spawn = np.array([0.0, 0.0, z])
        lo, hi = np.min(lo, axis=0) + spawn, np.max(hi, axis=0) + spawn
        assert lo[2] == pytest.approx(0.0), e.id                              # neither floating nor sunk
        assert tuple(hi[:2] - lo[:2]) == pytest.approx(e.world_size()), e.id  # the footprint the fake Go2 uses
        assert tuple((hi[:2] + lo[:2]) / 2) == pytest.approx(e.world_offset()), e.id   # centred on the footprint
        assert hi[2] == pytest.approx(e.height), e.id                         # chair back up to 0.9 m
        assert e.height == {"box": 0.5, "chair": 0.9}[e.label]


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
    from gazebo.launch_config import GZ_BRIDGE_ARGS       # the main bridge's arguments (open-objects Task 2.9)
    assert ("/gazebo/logical_camera@ros_gz_interfaces/msg/LogicalCameraImage[gz.msgs.LogicalCameraImage"
            in GZ_BRIDGE_ARGS)
    assert "arguments=list(GZ_BRIDGE_ARGS)," in LAUNCH.read_text(encoding="utf-8")


def test_the_readme_echo_shows_the_whole_objects_message():
    """README and docs/simulation.md: `ros2 topic echo` cuts strings at 128 characters unless --full-length is given
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
    commands = [line.split() for doc in (README, SIMULATION) for line in doc.read_text(encoding="utf-8").splitlines()
                if "ros2 topic echo" in line and "/go2/objects" in line]
    assert commands
    for words in commands:
        assert "--full-length" in words or "-f" in words, " ".join(words)


def test_the_docs_state_the_gazebo_lap_result():
    """Design §2 is met in Gazebo since the controller patches 0003-0004 (image
    go2-voice-ltl-gazebo:patch-0004, code a2fe140 unchanged), in two sets (docs/reference/
    gazebo_objects_e2e.md, part 5 above parts 4, 3 and the conclusions). Set 1: lap 5/5, chair 3/3,
    hello 1/1; its RTF (0.87) and L1-L3 were on battery in Low Power Mode, AC from 07:08:34 in L4.
    Set 2, all on AC (RTF 0.99): lap 4/5 (L2-L5 in a row; L1 fell right after two Gazebo stalls),
    chair 3/3, hello 1/1. Laps 9/10, 371 lap steps with 1 fall against C2's 336 with 6. The docs
    say the fall and its likely cause (the gait clock counts wall time), not "5 of 5". Patch 0005
    (part 6, image go2-voice-ltl-gazebo:patch-0005) counts the gait in simulation time: laps with
    Gazebo paused on purpose went 0/3 on patch-0004 and 3/3 on patch-0005; its acceptance set (on
    battery, RTF 0.89) was lap 5/5, chair 3/3, hello 1/1, with larger walk-start tilts in the chair
    runs left unexplained. Before the patches the lap passed 2 of 5 (part 4). docs/vocabulary.md and docs/unknowns.md
    (README sections 4 and 8 before the README kept only the commands) and design §2 and §12 say so, name what is
    still unchecked, and the user docs do not read as if the grid executor were checked on the real robot."""
    readme = "\n".join(doc.read_text(encoding="utf-8") for doc in USER_DOCS)
    assert "偽の Go2 と Gazebo でしか確かめていません" not in readme
    assert "確かめた範囲では転んでいません。" not in readme.replace("物を使わない指示で確かめた範囲では", "")
    vocabulary = VOCABULARY.read_text(encoding="utf-8")
    objects = vocabulary[vocabulary.index("### 物を使う指示（箱と椅子）"):vocabulary.index("## 単語帳と状態遷移図の足し方")]
    lap_row = next(line for line in objects.splitlines() if line.startswith("| 箱の周りを一周して |"))
    assert "controller のパッチ" in lap_row and "10 回のうち 9 回" in lap_row and "[わかっていないこと](unknowns.md)" in lap_row
    assert "5 回のうち 5 回" not in lap_row and "Gazebo が止まった後に転んだ" in lap_row
    assert "まだ毎回は通らない" not in lap_row
    assert "0005 の後は 5 回とも通った" in lap_row and "わざと止めながらでも 3 回とも" in lap_row
    assert "Gazebo でも、controller のパッチ（0003・0004）の後に、設計書（物）§2 の合格の条件を満たしました" in objects
    assert "§2 の合格の条件をまだ満たしていません" not in objects
    assert "0005 を足した後の組でも満たしました（一周 5/5・椅子 3/3・挨拶つき 1/1。[わかっていないこと](unknowns.md)）" in objects
    near_row = next(line for line in objects.splitlines() if line.startswith("| `near_<id>` |"))
    assert "1.3 m" in near_row and "1.4 m" not in near_row
    unknown = UNKNOWNS.read_text(encoding="utf-8")
    gazebo = unknown[unknown.index("- **Gazebo での物を使う指示"):unknown.index("- **物を使う指示の実機での動き")]
    for words in ("満たしました", "一周 5/5", "一周 4/5", "RTF 0.99", "椅子 3/3", "低電力モード", "patch-0004",
                  "L4 の途中から電源", "静かな Mac", "371 回、転んだのは 1 回", "336 回で 6 回", "5 回のうち 2 回",
                  "2 回続けて止まった", "31 回のうち 1 回", "unitree_guide", "gazebo_objects_e2e.md", "第 5 部",
                  "椅子まで 1.28 m", "patch-0005", "第 6 部", "3 回とも止まりの直後に転び", "0005 は 3 回とも一周",
                  "一周 5/5（一周の動作 192 回で転び 0）", "RTF 0.89", "確かめていません", "0005 の組は電源につないで流していません"):
        assert words in gazebo, words
    assert "シミュレーションの時間で数えるパッチは、まだ作っていません" not in gazebo
    assert "ただし Mac が低電力モードのときに測りました" not in gazebo           # only R1 and L1-L3 were
    assert "GO2_GAZEBO_IMAGE=go2-voice-ltl-gazebo:patch-0002" in readme and "GO2_GAZEBO_IMAGE=go2-voice-ltl-gazebo:dev" not in readme
    design = DESIGN.read_text(encoding="utf-8")
    goal = design[design.index("## 2. ゴール"):design.index("## 3. やらないこと")]
    assert "**Gazebo での状態（2026-11-11、§12）：満たした。**" in goal and "patch-0004" in goal
    assert "一周 4/5" in goal and "10 回のうち 1 回" in goal
    assert "patch-0005" in goal and "3/3（0005 の前は 0/3）" in goal and "RTF 0.89" in goal
    results = design[design.index("**確かめた結果**"):]
    for words in ("§2 の合格の条件は、Gazebo で満たした", "一周 5/5", "一周 4/5", "一周は 2 組で 9/10", "椅子まで 3/3",
                  "低電力モード", "patch-0004", "07:08:34", "RTF 0.99", "動作 371 回で転び 1", "一周 2/5", "25 回", "終えたのは 1 回", "10 回のうち 7 回合格", "unitree_guide",
                  "決めたこと（利用者の判断、C4）"):
        assert words in results, words
    assert "Gazebo ではまだ満たせていない" not in results
    assert "**0005 の後も §2 を満たした（第 6 部、C5）。**" in results and "0004 が 0/3、0005 が 3/3" in results
    assert "最大 0.117" in results and "確かめていない" in results
    assert "シミュレーションの時間で数えるパッチは作っていない" not in results
    assert "**C5 歩き方の周期をシミュレーションの時間で数える（2026-11-11、利用者の判断）：**" in design
    assert "まだ決めていないこと（利用者の判断を待つ）" not in results
    assert "min(物の半径 + 1.5 m, 1.3 m)" in design and "min(物の半径 + 1.5 m, 1.4 m)" not in design
    record = (ROOT / "docs" / "reference" / "gazebo_objects_e2e.md").read_text(encoding="utf-8")
    assert (record.index("## 第 6 部") < record.index("## 第 5 部") < record.index("## 第 4 部")
            < record.index("## 第 3 部") < record.index("## 結論"))
    part6 = record[record.index("## 第 6 部"):record.index("## 第 5 部")]
    for words in ("**0004 は一周 0/3、0005 は 3/3。**", "| R1 | 11:02:52 | — | — | RTF **0.89** | — |",
                  "一周の動作（立つは数えない）は 192 回で転び 0", "0.333 → 0.516 m", "#### 歩き出しの傾き（椅子・挨拶つき）",
                  "`lowpowermode` は 0", "0005 のせいかどうかは確かめていない", "1.131 m・23.5°", "id `39fd601dc2a3`",
                  "GO2_GAZEBO_IMAGE=go2-voice-ltl-gazebo:patch-0004 docker compose --profile gazebo up", "`INVALID`"):
        assert words in part6, words
    part5 = record[record.index("## 第 5 部"):record.index("## 第 4 部")]
    assert "§2 の合格の条件（一周 3 回・椅子 3 回、転ばない）を Gazebo で満たした。" in part5
    assert "一周 5/5" in part5 and "低電力モード" in part5 and "RTF は R1 で 1 回だけ測った（0.87" in part5
    assert "07:08:34" in part5 and "loadavg は、回の前と走っている間（30 秒ごと）で 3.9〜23" in part5
    assert "### 電源での 2 組目（2026-11-11 08:03〜08:22）" in part5 and "#### L1 の転び方" in part5
    assert "| **9/10**（どちらの組も 3 回以上続けて転ばない） |" in part5 and "371 / 1" in part5
    assert "| R1 | 08:03:44 | — | — | RTF **0.99** | — |" in part5
    assert "測る間はつながらなかった" not in record and "loadavg は 5〜9" not in part5
    assert "一周の動作（立つは数えない）192 回で転び 0" in part5 and "336 回で 6 回転んだ" in part5
    assert "roll −0.323" in part5 and "一時停止：前進" in part5 and "約 0.14 秒遅れ" in part5
    assert "go2-voice-ltl-gazebo:patch-0002" in part5
    assert "## 結論（第 1 部・第 2 部）" in record
    summary = record[record.index("## まとめ：一周・椅子・RTF の全部"):record.index("## 気づいたこと")]
    assert "第 4 部の `a2fe140`" in summary and "第 5 部の 1 組目、`go2-voice-ltl-gazebo:patch-0004`" in summary
    assert "第 5 部の 2 組目、電源" in summary and "10 回で 9 回" in summary
    assert "第 6 部の受け入れの組、`patch-0005`、バッテリー" in summary and "| 第 6 部、`patch-0005`" in summary
    assert "**最後のコード（`a2fe140`）で一周 2/5・椅子 3/3・挨拶つき 0/1、操作画面の一周 1/3 とそこからの椅子まで 1/1。**" in record
    assert "終わって 1 秒（`TAIL_S`）しか見ない" not in record                  # TAIL_S is 3 s since 6bf599c


def test_the_gazebo_image_carries_the_controller_patches():
    """Patches 0003-0004 (gazebo_objects_e2e.md part 5) and 0005 (part 6) are applied by
    Dockerfile.gazebo with 0001-0002 (go2-voice-ltl-gazebo:patch-0005, built first); compose builds and runs
    the thin image go2-voice-ltl-gazebo:patch-0005-fuel2 on top of it (Dockerfile.gazebo-fuel: the pinned
    Fuel models, open-objects spec §12.6, R-H3g); the older images (kept as go2-voice-ltl-gazebo:patch-0005-fuel1,
    :patch-0004 and :patch-0002) are reached through GO2_GAZEBO_IMAGE, with a warning never to build with it set (compose
    would build the thin Fuel image under that tag). 0003 drops the three lines that wrote the low-pass velocity back into the Kalman state and
    keeps feeding the filters; 0004 makes StateTrotting::enter() start from STANCE_ALL; 0005 makes the
    wave generator count the controller's update time (the simulation time) instead of the wall clock."""
    patches = sorted(p.name for p in (ROOT / "docker" / "patches").glob("*.patch"))
    assert patches == ["0001-estimator-world-frame-leg-odometry.patch",
                       "0002-feet-end-calc-body-frame-stance-angle.patch",
                       "0003-estimator-velocity-low-pass-out-of-state.patch",
                       "0004-trotting-enter-from-stance-all.patch",
                       "0005-wave-generator-controller-time.patch"]
    p3 = (ROOT / "docker" / "patches" / patches[2]).read_text(encoding="utf-8")
    removed = [line for line in p3.splitlines() if line.startswith("-") and not line.startswith("---")]
    assert removed == [f"-    x_hat_({i + 3}) = low_pass_filters_[{i}]->getValue();" for i in range(3)]
    assert "     low_pass_filters_[0]->addValue(x_hat_(3));" in p3              # context: still fed
    p4 = (ROOT / "docker" / "patches" / patches[3]).read_text(encoding="utf-8")
    added = [line for line in p4.splitlines() if line.startswith("+") and not line.startswith("+++")]
    assert added[-1] == "+    wave_generator_->status_ = WaveStatus::STANCE_ALL;"
    assert "@@ -41,6 +41,12 @@ void StateTrotting::enter() {" in p4
    p5 = (ROOT / "docker" / "patches" / patches[4]).read_text(encoding="utf-8")
    removed = [line for line in p5.splitlines() if line.startswith("-") and not line.startswith("---")]
    assert removed == ["-    start_t_ = getSystemTime();",
                       "-            const double past_t = static_cast<double>(getSystemTime() - start_t_) * 1e-6;"]
    added = [line for line in p5.splitlines() if line.startswith("+") and not line.startswith("+++")]
    assert "+        ctrl_component_.wave_generator_->setTime(time.seconds());" in added
    assert "+            const double past_t = static_cast<double>(now_us_ - start_t_) * 1e-6;" in added
    lines = p5.splitlines()
    i = lines.index("+        ctrl_component_.wave_generator_->setTime(time.seconds());")
    assert lines[i + 1] == "         ctrl_component_.wave_generator_->update();"   # the time is set before update()
    dockerfile = (ROOT / "docker" / "Dockerfile.gazebo").read_text(encoding="utf-8")
    assert "for p in /tmp/quadruped_ros2_control_patches/*.patch; do git apply --verbose" in dockerfile
    compose = (ROOT / "compose.yaml").read_text(encoding="utf-8")
    gazebo = compose[compose.index("\n  gazebo:"):compose.index("\nvolumes:")]
    assert "    image: ${GO2_GAZEBO_IMAGE:-go2-voice-ltl-gazebo:patch-0005-fuel2}" in gazebo.splitlines()
    assert "GO2_GAZEBO_IMAGE=go2-voice-ltl-gazebo:patch-0004 docker compose --profile gazebo up" in gazebo
    assert "GO2_GAZEBO_IMAGE=go2-voice-ltl-gazebo:patch-0002 docker compose --profile gazebo up" in gazebo
    assert "Never run" in gazebo and "this tree cannot rebuild it" in gazebo
    assert "GO2_GAZEBO_IMAGE=go2-voice-ltl-gazebo:dev" not in gazebo


# ---- open-objects Task 2.3: the layout, the truth topic, the source switch and the sizing ----

def household_like_layout() -> layout.Layout:
    """The spec §12.5 household rows (gazebo/layouts/household.json itself belongs to Task 2.5), with the model
    folders of that time, so that a later room change (R-H3b: TrashBin and LampAndStand left docker/fuel_models.json)
    does not move these bridge tests."""
    def row(obj_id, model, x, y, size, height, offset=(0.0, 0.0), detectable=False, accept=None):
        return {"id": obj_id, "model": model, "x": x, "y": y, "yaw_deg": 0,
                "footprint": {"offset": list(offset), "size": list(size)}, "height": height,
                "detectable": detectable, "accept": accept or [obj_id.rsplit("_", 1)[0]]}
    return layout.parse({"version": 1, "objects": [
        row("box_1", "primitive:box", 2.5, 1.5, (0.5, 0.5), 0.5),
        row("chair_1", "primitive:chair", 2.5, -1.5, (0.45, 0.45), 0.9),
        row("trash_can_1", "model://TrashBin", -1.5, -1.5, (0.266, 0.375), 0.446),
        row("suitcase_1", "model://Suitcase1", -1.5, 1.5, (0.6, 0.4), 0.8),
        row("sofa_1", "model://Sofa", -4.4, 0.0, (1.044, 1.8), 0.943, offset=(0.02, 0.0), detectable=True),
        row("lamp_1", "model://LampAndStand", 0.8, -4.4, (0.622, 0.525), 1.021, detectable=True),
        row("table_1", "model://Table", -2.4, 4.4, (1.5, 0.8), 1.015, offset=(0.002, 0.0),
            accept=["table", "desk"]),
    ]}, "household_like", folders=frozenset({"Sofa", "TrashBin", "LampAndStand", "Table", "Suitcase1"}))


def one_entry(**changes) -> layout.Entry:
    base = {"id": "sofa_1", "model": "model://Sofa", "x": -4.4, "y": 0.0, "yaw_deg": 0,
            "footprint": {"offset": [0.02, 0.0], "size": [1.044, 1.8]}, "height": 0.943,
            "detectable": True, "accept": ["sofa"]}
    return layout.parse({"version": 1, "objects": [{**base, **changes}]}, "one").entries[0]


def test_objects_topics_and_source():
    assert OBJECTS_SOURCES == ("logical", "detector") and TRUTH_TOPIC == "/gazebo/objects_truth"
    assert objects_topics("logical") == ("/go2/objects", "/gazebo/objects_truth")
    assert objects_topics("detector") == ("/gazebo/objects_truth",)
    for bad in ("Logical", "", "detecter"):
        with pytest.raises(ValueError):
            objects_topics(bad)
    assert objects_source({}) == "logical" and objects_source({"OBJECTS_SOURCE": "detector"}) == "detector"
    assert objects_source({"OBJECTS_SOURCE": "logical"}) == "logical"
    for bad in ("Logical", "", "fake", "detecter"):          # any other value: the bridge does not start
        with pytest.raises(ValueError, match="OBJECTS_SOURCE") as e:
            objects_source({"OBJECTS_SOURCE": bad})
        assert "logical" in str(e.value) and "detector" in str(e.value) and "\n" not in str(e.value)
    assert layout_name({}) == "household" and layout_name({"GO2_LAYOUT": "default"}) == "default"


def test_publisher_note_names_the_missing_detector():    # Review Focus 3
    assert objects_publisher_note("logical", 1) is None and objects_publisher_note("detector", 1) is None
    assert "--profile detector" in objects_publisher_note("detector", 0)
    assert "OBJECTS_SOURCE" in objects_publisher_note("logical", 2)
    assert "OBJECTS_SOURCE" in objects_publisher_note("detector", 2) and "3" in objects_publisher_note("logical", 3)
    assert objects_publisher_note("logical", 0) is None       # the bridge is the publisher; not yet in the graph
    for source, count in (("detector", 0), ("logical", 2)):
        note = objects_publisher_note(source, count)
        assert OBJECTS_TOPIC in note and "\n" not in note
    with pytest.raises(ValueError):
        objects_publisher_note("fake", 1)


def test_missing_model_folder_stops_the_bridge_before_spawning(tmp_path):   # Review Focus 5
    (tmp_path / "Sofa").mkdir()
    assert missing_models(household_like_layout(), str(tmp_path)) == ["LampAndStand", "Suitcase1", "Table", "TrashBin"]
    assert missing_models(layout.load("default"), "") == []
    other = tmp_path / "other"
    for folder in ("LampAndStand", "Suitcase1", "Table"):
        (other / folder).mkdir(parents=True)
    (other / "TrashBin").write_text("not a folder", encoding="utf-8")
    path = f"{tmp_path / 'none'}::{tmp_path}:{other}"          # any entry may hold a folder; empty entries skipped
    assert missing_models(household_like_layout(), path) == ["TrashBin"]
    assert missing_models(household_like_layout(), "") == ["LampAndStand", "Sofa", "Suitcase1", "Table", "TrashBin"]


def test_bridge_settings_stop_before_spawning_with_one_line(tmp_path, monkeypatch):   # Review Focus 5
    assert bridge_settings({"GO2_LAYOUT": "default"}) == ("logical", layout.load("default"))
    assert bridge_settings({"OBJECTS_SOURCE": "detector", "GO2_LAYOUT": "default"})[0] == "detector"
    with pytest.raises(ValueError, match="OBJECTS_SOURCE"):
        bridge_settings({"OBJECTS_SOURCE": "detecter", "GO2_LAYOUT": "default"})
    with pytest.raises(ValueError) as e:
        bridge_settings({"GO2_LAYOUT": "nope"})
    assert all(w in str(e.value) for w in ("GO2_LAYOUT", "nope", "default")) and "\n" not in str(e.value)
    monkeypatch.setattr(bridge_core, "load_layout", lambda name: household_like_layout())
    (tmp_path / "Sofa").mkdir()
    with pytest.raises(ValueError) as e:
        bridge_settings({"GZ_SIM_RESOURCE_PATH": str(tmp_path)})
    note = str(e.value)
    assert all(f in note for f in ("LampAndStand", "Suitcase1", "Table", "TrashBin", "GZ_SIM_RESOURCE_PATH", "household"))
    assert "Sofa" not in note and str(tmp_path) not in note and "\n" not in note   # names the variable, not its value
    assert "go2-voice-ltl-gazebo:patch-0005-fuel2" in note and "fuel1" not in note    # the image with the room
    (tmp_path / "sub").mkdir()
    for folder in ("LampAndStand", "Suitcase1", "Table", "TrashBin"):
        (tmp_path / "sub" / folder).mkdir()
    assert bridge_settings({"GZ_SIM_RESOURCE_PATH": f"{tmp_path}:{tmp_path / 'sub'}"})[1].name == "household_like"


def test_logical_rows_use_the_footprint_centre():        # C3
    rows = logical_rows([("box_1", 2.5, 1.5), ("chair_1", 2.5, -1.5), ("sim_box_1", 1.0, 0.0), ("ground_plane", 0, 0)],
                        layout.load("default"))
    assert rows == [("box_1", "box", 2.5, 1.5), ("chair_1", "chair", 2.5, -1.5)]   # today's positions
    # an entry with offset (0.19, 0) at yaw 90 is published 0.19 m along +y of its origin
    # (the alternative Bookshelf's offset, spec §12.3; any folder of docker/fuel_models.json passes the loader)
    lay = layout.parse({"version": 1, "objects": [
        {"id": "bookshelf_1", "model": "model://Sofa", "x": 1.0, "y": 2.0, "yaw_deg": 90,
         "footprint": {"offset": [0.19, 0.0], "size": [0.9, 0.3]}, "height": 1.8,
         "detectable": False, "accept": ["bookshelf"]}]}, "turned")
    ((obj_id, label, x, y),) = logical_rows([("bookshelf_1", 1.0, 2.0), ("box_1", 2.5, 1.5)], lay)
    assert (obj_id, label) == ("bookshelf_1", "bookshelf") and (x, y) == pytest.approx((1.0, 2.19))
    household = household_like_layout()
    rows = logical_rows([("sofa_1", -4.4, 0.0), ("table_1", -2.4, 4.4)], household)
    assert rows == [("sofa_1", "sofa", pytest.approx(-4.38), 0.0), ("table_1", "table", pytest.approx(-2.398), 4.4)]


def test_truth_rows_come_from_the_layout():
    lay = household_like_layout()
    truth = ObjectMemory(sizes={e.id: e.world_size() for e in lay.entries})
    truth.update(logical_rows([("sofa_1", -4.4, 0.0), ("box_1", 2.5, 1.5), ("sim_box_1", 1.0, 0.0)], lay), 10.0)
    truth.update(logical_rows([("sofa_1", -4.4, 0.0)], lay), 11.0)
    truth.set_size("sofa_1", (0.5, 0.5))                       # the truth keeps the layout's sizes
    rows = {o.id: o for o in truth.objects()}
    assert list(rows) == ["box_1", "sofa_1"]                   # no button boxes; only what the camera saw
    sofa, box = rows["sofa_1"], rows["box_1"]
    assert (sofa.x, sofa.y) == pytest.approx((-4.38, 0.0))
    assert sofa.size == (1.044, 1.8) and box.size == (0.5, 0.5)   # world axes of the layout footprint
    assert (sofa.visible, sofa.seen, box.visible, box.seen) == (True, 11.0, False, 10.0)
    assert sofa.measured is None and box.measured is None
    msg = truth.message(12.0, layout=lay.name)
    assert (msg["frame"], msg["source"], msg["layout"]) == ("odom", "logical", "household_like")
    assert all("measured" not in r and "confidence" not in r for r in msg["objects"])
    parsed = parse(json.loads(json.dumps(msg)))
    assert [(o.id, o.label, o.size, o.visible, o.seen) for o in parsed] == [
        (o.id, o.label, o.size, o.visible, o.seen) for o in truth.objects()]
    assert [(o.x, o.y) for o in parsed] == pytest.approx([(2.5, 1.5), (-4.38, 0.0)])
    assert "layout" not in ObjectMemory().message(12.0)        # /go2/objects carries no layout


def test_memory_starts_at_the_vocabulary_square_and_marks_measured():
    m = ObjectMemory()
    m.update([("sofa_1", "sofa", -4.38, 0.0), ("chair_1", "chair", 2.5, -1.5)], 10.0)
    chair, sofa = m.objects()
    assert (chair.size, chair.measured) == ((0.45, 0.45), False)   # the vocabulary square [L, L]
    assert (sofa.size, sofa.measured) == ((1.8, 1.8), False)
    m.set_size("chair_1", (0.5, 0.48))
    m.update([("chair_1", "chair", 2.5, -1.5)], 11.0)               # kept across frames
    chair = m.objects()[0]
    assert (chair.size, chair.measured) == ((0.5, 0.48), True)     # the first LiDAR size replaces the square
    m.set_size("chair_1", (0.4, 0.55))
    assert m.objects()[0].size == (0.5, 0.55)                       # then the larger per axis (today)
    rows = {r["id"]: r for r in m.message(12.0)["objects"]}
    assert rows["chair_1"]["measured"] is True and rows["sofa_1"]["measured"] is False
    assert m.message(12.0)["source"] == "logical"
    m.update([("box_10", "box", 0.0, 0.0), ("box_2", "box", 1.0, 0.0)], 13.0)
    assert [o.id for o in m.objects()] == ["box_2", "box_10", "chair_1", "sofa_1"]   # _id_key order


def test_measuring_radius_follows_the_kind():
    sofa = side_points(-4.38, 0.0, 1.8, 1.04, top=0.9)
    assert footprint_size(sofa, -4.38, 0.0, radius=measure_radius("sofa")) == pytest.approx((1.8, 1.04), abs=0.05)
    assert max(footprint_size(sofa, -4.38, 0.0, radius=SIZE_RADIUS)) <= 1.2      # today's 0.6 m cut it
    assert measure_radius("box") == pytest.approx(0.654, abs=1e-3)
    assert measure_radius("chair") == pytest.approx(0.618, abs=1e-3)
    for label, (sx, sy, top) in {"box": (0.5, 0.5, 0.5), "chair": (0.45, 0.45, 0.9)}.items():
        pts = side_points(2.5, 1.5, sx, sy, top=top)       # nothing else near: the same size as with 0.6 m
        assert footprint_size(pts, 2.5, 1.5, radius=measure_radius(label)) == footprint_size(pts, 2.5, 1.5)


def test_spawn_fields_for_primitives_and_fuel_models():
    box, chair = layout.load("default").entries
    assert spawn_fields(box) == (object_sdf("box_1", "box"), 2.5, 1.5, 0.25, 0.0)
    assert spawn_fields(chair) == (object_sdf("chair_1", "chair"), 2.5, -1.5, 0.45, 0.0)
    sdf, x, y, z, yaw = spawn_fields(one_entry(yaw_deg=90))
    inc = ET.fromstring(sdf).find("include")
    assert (inc.find("uri").text, inc.find("name").text, inc.find("static").text) == ("model://Sofa", "sofa_1", "true")
    assert (x, y, z) == (-4.4, 0.0, 0.0)                     # the model origin, not the footprint centre
    assert yaw == pytest.approx(math.pi / 2)
    assert spawn_fields(one_entry(z=0.15))[3] == 0.15 and spawn_fields(one_entry(yaw_deg=-90))[4] == pytest.approx(-math.pi / 2)
    for e in household_like_layout().entries:
        sdf, x, y, z, yaw = spawn_fields(e)
        assert ET.fromstring(sdf).find(".//static").text == "true" and (x, y) == (e.x, e.y), e.id


def test_box_sdf_cardboard_only_in_detector_mode():
    b = Box(1.0, 2.0)
    plain, card = (ET.fromstring(box_sdf("sim_box_1", b, cardboard=c)) for c in (False, True))
    assert ET.tostring(ET.fromstring(box_sdf("sim_box_1", b))) == ET.tostring(plain)      # the default is plain
    assert ([ET.tostring(c) for c in plain.iter("collision")] == [ET.tostring(c) for c in card.iter("collision")])
    assert plain.find(".//collision/geometry/box/size").text == f"{b.size} {b.size} {BOX_HEIGHT}"
    assert "cardboard_box" not in box_sdf("sim_box_1", b) and "cardboard_box" not in ET.tostring(plain, "unicode")
    (visual,) = card.iter("visual")
    assert "cardboard_box" in visual.find("geometry/mesh/uri").text
    scale = np.array([float(v) for v in visual.find("geometry/mesh/scale").text.split()])
    assert scale * CARDBOARD_MESH_SIDE == pytest.approx([b.size, b.size, BOX_HEIGHT], abs=1e-4)   # same size
    assert [float(v) for v in visual.find("pose").text.split()] == pytest.approx([0, 0, -BOX_HEIGHT / 2, 0, 0, 0])
    assert card.find("model").get("name") == "sim_box_1" and card.find("model/static").text == "true"


def test_compose_gazebo_service():
    text = (ROOT / "compose.yaml").read_text(encoding="utf-8")
    gazebo = yaml.safe_load(text)["services"]["gazebo"]
    assert gazebo["image"] == "${GO2_GAZEBO_IMAGE:-go2-voice-ltl-gazebo:patch-0005-fuel2}"
    assert gazebo["build"] == {"context": "./docker", "dockerfile": "Dockerfile.gazebo-fuel"}
    assert gazebo["environment"] == {"OBJECTS_SOURCE": "${OBJECTS_SOURCE:-logical}",
                                     "GO2_LAYOUT": "${GO2_LAYOUT:-household}"}
    assert gazebo["ports"] == ["127.0.0.1:${GO2_GAZEBO_PORT:-6080}:6080"]
    lines = text[text.index("\n  gazebo:"):text.index("\nvolumes:")].splitlines()
    assert ("    # go2-voice-ltl-gazebo:patch-0005 (patches 0001-0005, docker/patches) is built first with "
            "docker/Dockerfile.gazebo (docs/simulation.md); build here makes the thin image patch-0005-fuel2 on top "
            "of it (docker/Dockerfile.gazebo-fuel: the pinned Fuel models). The older images are kept on this Mac only:"
            in lines)
    older = lines[lines.index("    # go2-voice-ltl-gazebo:patch-0005 (patches 0001-0005, docker/patches) is built first with "
                              "docker/Dockerfile.gazebo (docs/simulation.md); build here makes the thin image "
                              "patch-0005-fuel2 on top of it (docker/Dockerfile.gazebo-fuel: the pinned Fuel models). "
                              "The older images are kept on this Mac only:") + 1]
    assert older.startswith("    #   go2-voice-ltl-gazebo:patch-0005-fuel1 (")    # the first Fuel room, kept (R-H3g)
    assert "    # the thin Fuel image (Dockerfile.gazebo-fuel, on top of patch-0005) and put it under the given tag." in lines
    assert not any("# patches 0001-0005 (docker/patches)." in line for line in lines)
    assert "0001-0005 from this tree" not in "\n".join(lines)
