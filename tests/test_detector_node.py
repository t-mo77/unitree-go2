"""detector/node.py: the rclpy node of the detector container (Task 2.10; spec §11.1 start-up order, §11.3 topics,
§11.9 failures; design-detector.md §3.5; review M8, M9).

The pure parts (the refusal line, the message conversions, the camera constants against the xacro) need no ROS.
The rest runs rclpy in this process and is skipped without it. These tests use the ROS_DOMAIN_ID of the environment
(T passes the task's id) and keep DDS on this container's loopback (ROS_LOCALHOST_ONLY), so nothing reaches the
user's stack. A probe node publishes the sensors and listens on /go2/objects and /go2/detector_status.
"""

import json
import logging
import math
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from detector import node as node_module
from detector.camera import CAMERA_CLIP, CAMERA_H, CAMERA_HFOV, CAMERA_RATE_HZ, CAMERA_W, CameraModel
from detector.node import (CAMERA_INFO_TOPIC, CLOUD_TOPIC, IMAGE_TOPIC, ODOM_TOPIC, STATUS_TOPIC, image_from_msg,
                           points_from_cloud, stamp_of, startup_refusal)
from detector.oracle import OracleDetector
from detector.protocol import vocabulary_prompts
from detector.scripted import ScriptedDetector
from gazebo import launch_config
from gazebo.bridge_core import TRUTH_TOPIC
from voice_ltl import objects
from voice_ltl.objects import OBJECTS_TOPIC

ROOT = Path(__file__).resolve().parent.parent
XACRO = ROOT / "gazebo" / "go2_sensors.urdf.xacro"
SENSOR_TOPICS = {ODOM_TOPIC, CLOUD_TOPIC, IMAGE_TOPIC, CAMERA_INFO_TOPIC}

# PointField datatypes (sensor_msgs/PointField)
INT8, UINT8, INT16, UINT16, INT32, UINT32, FLOAT32, FLOAT64 = range(1, 9)


# --- the refusal (review M9) -------------------------------------------------------------------------------------

def test_startup_refusal():
    for environ in ({}, {"OBJECTS_SOURCE": "logical"}):
        line = startup_refusal(environ)
        assert isinstance(line, str) and "OBJECTS_SOURCE" in line and "logical" in line
        assert "\n" not in line                                   # one line
        assert "--profile detector" in line                       # how to start it on purpose
    for bad in ("lidar", "", "DETECTOR"):
        line = startup_refusal({"OBJECTS_SOURCE": bad})
        assert isinstance(line, str) and "OBJECTS_SOURCE" in line and repr(bad) in line and "\n" not in line
    assert startup_refusal({"OBJECTS_SOURCE": "detector"}) is None
    assert startup_refusal({"OBJECTS_SOURCE": "detector", "DETECTOR": "oracle"}) is None


class FakeRclpy:
    def __init__(self):
        self.inits = 0

    def init(self, *args, **kwargs):
        self.inits += 1
        raise AssertionError("rclpy.init must not run on a refusal")


@pytest.mark.parametrize("source", ["logical", None])
def test_main_returns_0_on_refusal(monkeypatch, capsys, source):
    if source is None:
        monkeypatch.delenv("OBJECTS_SOURCE", raising=False)
    else:
        monkeypatch.setenv("OBJECTS_SOURCE", source)
    fake = FakeRclpy()
    monkeypatch.setattr(node_module, "rclpy", fake)
    assert node_module.main() == 0                                # compose's restart: on-failure:3 leaves it stopped
    out = capsys.readouterr().out
    assert out == startup_refusal({} if source is None else {"OBJECTS_SOURCE": source}) + "\n"
    assert fake.inits == 0


# --- message conversions -----------------------------------------------------------------------------------------

def _stamp(t):
    sec = int(math.floor(t))
    return SimpleNamespace(sec=sec, nanosec=int(round((t - sec) * 1e9)))


def _fake_image(img, encoding="rgb8", pad=0, t=12.5):
    h, w = img.shape[:2]
    rows = np.zeros((h, 3 * w + pad), dtype=np.uint8)
    rows[:, :3 * w] = img.reshape(h, 3 * w)
    return SimpleNamespace(header=SimpleNamespace(stamp=_stamp(t)), height=h, width=w, encoding=encoding,
                           is_bigendian=0, step=3 * w + pad, data=rows.tobytes())


def _ros_image(img, encoding="rgb8", pad=0, t=12.5):
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import Image
    fake = _fake_image(img, encoding, pad, t)
    msg = Image(height=fake.height, width=fake.width, encoding=encoding, step=fake.step, data=fake.data)
    msg.header.stamp = Time(sec=fake.header.stamp.sec, nanosec=fake.header.stamp.nanosec)
    return msg


def _fake_cloud(points, fields=(("x", 0, FLOAT32), ("y", 4, FLOAT32), ("z", 8, FLOAT32)), point_step=12,
                bigendian=False, height=1, row_pad=0, t=12.5):
    """A PointCloud2-shaped object holding points (N x 3) in the given layout (extra bytes left zero)."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    n = len(pts)
    width = n // height
    kinds = {FLOAT32: "f4", FLOAT64: "f8"}
    order = ">" if bigendian else "<"
    dtype = np.dtype({"names": [f[0] for f in fields], "formats": [order + kinds[f[2]] for f in fields],
                      "offsets": [f[1] for f in fields], "itemsize": point_step})
    arr = np.zeros(n, dtype=dtype)
    for i, name in enumerate("xyz"):
        if name in dtype.names:
            arr[name] = pts[:, i]
    raw = arr.tobytes()
    row = width * point_step
    data = b"".join(raw[r * row:(r + 1) * row] + bytes(row_pad) for r in range(height))
    return SimpleNamespace(header=SimpleNamespace(stamp=_stamp(t)), height=height, width=width,
                           fields=[SimpleNamespace(name=f[0], offset=f[1], datatype=f[2], count=1) for f in fields],
                           is_bigendian=bigendian, point_step=point_step, row_step=row + row_pad, data=data,
                           is_dense=True)


def _ros_cloud(points, extra_intensity=False, t=12.5):
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    header = Header(frame_id="utlidar_lidar", stamp=Time(sec=int(t), nanosec=int(round((t % 1) * 1e9))))
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    if not extra_intensity:
        return point_cloud2.create_cloud_xyz32(header, pts)        # what the bridge publishes on /utlidar/cloud
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(("x", "y", "z", "intensity"))]
    rows = [(float(x), float(y), float(z), 7.0) for x, y, z in pts]
    return point_cloud2.create_cloud(header, fields, rows)


@pytest.fixture(params=["fake", "ros"])
def kind(request):
    if request.param == "ros":
        pytest.importorskip("sensor_msgs_py")
    return request.param


def test_image_and_cloud_conversion(kind):
    make_image = _ros_image if kind == "ros" else _fake_image
    rng = np.random.default_rng(7)
    img = rng.integers(0, 256, size=(CAMERA_H, CAMERA_W, 3), dtype=np.uint8)
    for msg in (make_image(img), make_image(img, pad=8)):     # rows may be padded (step > 3 W)
        got = image_from_msg(msg)
        assert got.dtype == np.uint8 and got.shape == (CAMERA_H, CAMERA_W, 3)
        assert np.array_equal(got, img)
    small = rng.integers(0, 256, size=(3, 5, 3), dtype=np.uint8)
    assert np.array_equal(image_from_msg(make_image(small[..., ::-1], encoding="bgr8")), small)   # BGR -> RGB
    with pytest.raises(ValueError, match="mono8"):
        image_from_msg(make_image(small[..., 0:1].repeat(3, axis=2), encoding="mono8"))
    short = _fake_image(small)
    short.data = short.data[:-1]
    with pytest.raises(ValueError):
        image_from_msg(short)
    assert stamp_of(make_image(small, t=12.5)) == pytest.approx(12.5, abs=1e-9)

    pts = rng.uniform(-5.0, 5.0, size=(40, 3))
    want = pts.astype(np.float32).astype(np.float64)
    if kind == "ros":
        clouds = [_ros_cloud(pts), _ros_cloud(pts, extra_intensity=True)]
    else:
        clouds = [
            _fake_cloud(pts),                                                       # the bridge's layout
            _fake_cloud(pts, fields=(("x", 0, FLOAT32), ("y", 4, FLOAT32), ("z", 8, FLOAT32),
                                     ("intensity", 12, FLOAT32)), point_step=16),
            _fake_cloud(pts, bigendian=True),
            _fake_cloud(pts, height=4, row_pad=6),                                  # rows padded
        ]
    for msg in clouds:
        got = points_from_cloud(msg)
        assert got.dtype == np.float64 and got.shape == (40, 3)
        assert np.array_equal(got, want)
    if kind == "fake":
        doubles = _fake_cloud(pts, fields=(("z", 0, FLOAT64), ("x", 8, FLOAT64), ("y", 16, FLOAT64)),
                              point_step=24)
        assert np.array_equal(points_from_cloud(doubles), pts)                  # any order, float64 kept
        holes = pts.copy()
        holes[3, 1] = np.nan
        holes[9, 2] = np.inf
        got = points_from_cloud(_fake_cloud(holes))
        assert got.shape == (38, 3) and np.isfinite(got).all()                   # non-finite rows dropped
        assert points_from_cloud(_fake_cloud(np.zeros((0, 3)))).shape == (0, 3)
        no_z = _fake_cloud(pts, fields=(("x", 0, FLOAT32), ("y", 4, FLOAT32), ("w", 8, FLOAT32)))
        with pytest.raises(ValueError, match="z"):
            points_from_cloud(no_z)
        cut = _fake_cloud(pts)
        cut.data = cut.data[:-4]
        with pytest.raises(ValueError):
            points_from_cloud(cut)
    assert stamp_of(clouds[0]) == pytest.approx(12.5, abs=1e-9)


# --- the camera (D7) and the topics ------------------------------------------------------------------------------

def _front_camera() -> ET.Element:
    sensors = [s for s in ET.parse(XACRO).getroot().iter("sensor") if s.get("name") == "front_camera"]
    assert len(sensors) == 1
    return sensors[0]


def test_camera_constants_match_the_xacro():
    sensor = _front_camera()
    assert sensor.get("type") == "camera"
    cam = sensor.find("camera")
    assert float(cam.findtext("horizontal_fov")) == CAMERA_HFOV
    assert (int(cam.findtext("image/width")), int(cam.findtext("image/height"))) == (CAMERA_W, CAMERA_H)
    assert cam.findtext("image/format") == "R8G8B8"                              # rgb8 on the ROS side
    assert (float(cam.findtext("clip/near")), float(cam.findtext("clip/far"))) == CAMERA_CLIP
    assert float(sensor.findtext("update_rate")) == CAMERA_RATE_HZ
    assert "/" + sensor.findtext("topic") == IMAGE_TOPIC
    assert "/" + cam.findtext("camera_info_topic") == CAMERA_INFO_TOPIC
    k = CameraModel.from_hfov()                                                  # until CameraInfo arrives
    assert (round(k.fx, 3), round(k.fy, 3), k.cx, k.cy) == (184.836, 184.836, 320.0, 240.0)


def test_topics_match_their_other_ends():
    def ros_side(args):
        return {a.split("@")[0] for a in args}
    assert {IMAGE_TOPIC, CAMERA_INFO_TOPIC} == ros_side(launch_config.CAMERA_BRIDGE_ARGS)   # front_camera_bridge
    assert ODOM_TOPIC in ros_side(launch_config.GZ_BRIDGE_ARGS)                             # Gazebo's truth
    bridge = (ROOT / "gazebo" / "gazebo_bridge.py").read_text(encoding="utf-8")
    assert f'LIDAR_TOPIC = "{CLOUD_TOPIC}"' in bridge                            # the bridge's /utlidar/cloud
    from gazebo.tools import objects_check
    assert objects_check.STATUS_TOPIC == STATUS_TOPIC
    assert not any(t.startswith("/sim/") for t in SENSOR_TOPICS | {STATUS_TOPIC, TRUTH_TOPIC})   # D9


# --- the node (rclpy) --------------------------------------------------------------------------------------------

def wait_for(predicate, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return bool(predicate())


def _subscribed(node):
    return {s.topic_name for s in node.subscriptions}


class Ros:
    """One executor thread for the nodes under test and a probe node of the test's own."""

    def __init__(self, rclpy):
        from rclpy.executors import SingleThreadedExecutor
        self.executor = SingleThreadedExecutor()
        self.probe = rclpy.create_node("test_detector_node_probe")
        self.executor.add_node(self.probe)
        self.nodes = []
        self.thread = threading.Thread(target=self._spin, name="test-spin", daemon=True)
        self.thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:                          # the context shut down under it
            pass

    def add(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)
        return node

    def listen(self, topic):
        from std_msgs.msg import String
        got = []
        self.probe.create_subscription(String, topic, lambda m: got.append((time.monotonic(), json.loads(m.data))),
                                       10)
        return got

    def close(self):
        self.executor.shutdown(timeout_sec=5.0)
        self.thread.join(5.0)
        for n in self.nodes:
            n.destroy_node()
        self.probe.destroy_node()


@pytest.fixture
def rclpy_here(monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")        # DDS stays on this container's loopback
    return rclpy


@pytest.fixture
def ros(rclpy_here):
    from rclpy.signals import SignalHandlerOptions
    rclpy_here.init(signal_handler_options=SignalHandlerOptions.NO)
    harness = Ros(rclpy_here)
    try:
        yield harness
    finally:
        harness.close()
        rclpy_here.try_shutdown()


def test_main_returns_1_when_the_detector_fails_to_load(rclpy_here, monkeypatch, caplog):
    monkeypatch.setenv("OBJECTS_SOURCE", "detector")
    monkeypatch.setenv("DETECTOR", "nope")
    caplog.set_level(logging.INFO, logger="detector.node")
    start = time.monotonic()
    assert node_module.main() == 1                         # compose restarts it up to 3 times (spec §11.9)
    assert time.monotonic() - start < 10.0
    errors = [r for r in caplog.records if r.name == "detector.node" and r.levelno >= logging.ERROR]
    assert errors and "'nope'" in errors[0].getMessage() and "DETECTOR" in errors[0].getMessage()
    assert not rclpy_here.ok()                             # main() shut rclpy down again


def test_node_publishes_an_empty_detector_list_at_once(ros):
    release = threading.Event()
    made = []

    def slow():                                            # a detector whose model takes long to load
        release.wait(30.0)
        made.append(ScriptedDetector([]))
        return made[-1]

    listed = ros.listen(OBJECTS_TOPIC)
    status = ros.listen(STATUS_TOPIC)
    start = time.monotonic()
    node = ros.add(node_module.DetectorNode(make_detector=slow))
    try:
        assert wait_for(lambda: listed, 3.0)
        arrived, first = listed[0]
        assert arrived - start <= 1.0                      # within a second of start-up (spec §11.1)
        assert first["frame"] == "odom" and first["source"] == "detector" and first["objects"] == []
        assert objects.parse_source(first) == "detector" and objects.parse(first) == ()
        assert wait_for(lambda: status, 3.0)
        assert status[0][1] == {"ok": False, "reason": "loading"}
        assert wait_for(lambda: len(listed) >= 2, 3.0)    # the 2 Hz timer runs while the detector loads ...
        assert all(m["source"] == "detector" and m["objects"] == [] for _, m in listed)
        assert not made and node.core is None and not node.ready.is_set()
        assert _subscribed(node) == set()                  # ... and no sensor is subscribed before it has loaded
    finally:
        release.set()
    assert wait_for(node.ready.is_set, 5.0)
    assert made and made[0].vocabulary == vocabulary_prompts()   # every kind's detect, and the ignore list
    assert _subscribed(node) == SENSOR_TOPICS               # no truth without the oracle, never /sim/* (D9)
    seen = len(status)
    assert wait_for(lambda: len(status) > seen and "detector" in status[-1][1], 3.0)
    assert status[-1][1]["detector"] == "scripted" and status[-1][1]["reason"] == "loading"   # sensors on their way


def _odometry(t, moving):
    from nav_msgs.msg import Odometry
    msg = Odometry()
    msg.header.frame_id = "odom"
    msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int(round((t % 1) * 1e9)) % 1_000_000_000
    msg.pose.pose.position.z = 0.33
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = 0.3 if moving else 0.0
    return msg


def _feed(pubs, t0, seconds, moving, until=lambda: False):
    """Odometry at 50 Hz, scans at 10 Hz and images at 2 Hz, stamped with sim time = 100 s + the wall time since t0."""
    odom_pub, cloud_pub, image_pub = pubs
    from sensor_msgs.msg import Image
    image = Image(height=CAMERA_H, width=CAMERA_W, encoding="rgb8", step=3 * CAMERA_W,
                  data=bytes(3 * CAMERA_W * CAMERA_H))
    end = time.monotonic() + seconds
    tick = 0
    while time.monotonic() < end and not until():
        t = 100.0 + time.monotonic() - t0
        odom_pub.publish(_odometry(t, moving))
        if tick % 5 == 0:
            cloud_pub.publish(_ros_cloud([[2.0, 0.0, 0.2], [2.0, 0.1, 0.3], [2.1, -0.1, 0.4]], t=t))
        if tick % 25 == 0:
            image.header.stamp.sec, image.header.stamp.nanosec = int(t), int(round((t % 1) * 1e9)) % 1_000_000_000
            image_pub.publish(image)
        tick += 1
        time.sleep(0.02)


def test_node_runs_the_detector_only_while_still(ros):
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, PointCloud2
    det = ScriptedDetector([])
    node = ros.add(node_module.DetectorNode(make_detector=lambda: det))
    assert wait_for(node.ready.is_set, 5.0)
    pubs = (ros.probe.create_publisher(Odometry, ODOM_TOPIC, qos_profile_sensor_data),
            ros.probe.create_publisher(PointCloud2, CLOUD_TOPIC, qos_profile_sensor_data),
            ros.probe.create_publisher(Image, IMAGE_TOPIC, qos_profile_sensor_data))
    t0 = time.monotonic()
    _feed(pubs, t0, 2.0, moving=True)                 # walking: the images arrive and are dropped
    health = node.core.health()
    assert det.calls == 0 and health["frames_dropped"] >= 2 and health["inferences"] == 0
    _feed(pubs, t0, 5.0, moving=False, until=lambda: det.calls >= 1)
    assert det.calls >= 1                                  # still for 0.3 s: the image and a scan are paired
    assert wait_for(lambda: node.core.health()["inferences"] >= 1, 2.0)


def test_with_the_oracle_the_node_feeds_it_the_truth(ros):
    from std_msgs.msg import String

    class RecordingOracle(OracleDetector):
        def __init__(self):
            super().__init__()
            self.truths = []

        def set_truth(self, rows, heights=None):
            self.truths.append((tuple(rows), heights))
            super().set_truth(rows, heights)

    oracle = RecordingOracle()
    node = ros.add(node_module.DetectorNode(make_detector=lambda: oracle))
    assert wait_for(node.ready.is_set, 5.0)
    assert _subscribed(node) == SENSOR_TOPICS | {TRUTH_TOPIC}
    truth = ros.probe.create_publisher(String, TRUTH_TOPIC, 10)
    row = {"id": "chair_1", "label": "chair", "x": 2.0, "y": 0.5, "size": [0.45, 0.45], "seen": time.time(),
           "visible": True}
    payload = json.dumps({"frame": "odom", "stamp": time.time(), "source": "logical", "layout": "default",
                          "objects": [row]})
    assert wait_for(lambda: truth.publish(String(data=payload)) or oracle.truths, 3.0)
    rows, heights = oracle.truths[-1]
    assert [r.id for r in rows] == ["chair_1"] and rows[0].size == (0.45, 0.45)
    assert heights is None                                 # the oracle uses the vocabulary height (review M8)


def test_a_second_objects_publisher_is_an_error_naming_objects_source(ros, caplog):
    from std_msgs.msg import String
    node = ros.add(node_module.DetectorNode(make_detector=lambda: ScriptedDetector([])))
    assert wait_for(node.ready.is_set, 5.0)
    caplog.set_level(logging.INFO, logger="detector.node")
    node._check_publishers()                               # the 10 s timer: alone on /go2/objects, nothing to say
    assert not [r for r in caplog.records if r.levelno >= logging.ERROR]
    ros.probe.create_publisher(String, OBJECTS_TOPIC, 10)  # a second publisher (a logical bridge, a fake Go2)
    assert wait_for(lambda: node.count_publishers(OBJECTS_TOPIC) == 2, 3.0)
    node._check_publishers()
    errors = [r.getMessage() for r in caplog.records if r.levelno >= logging.ERROR]
    assert len(errors) == 1 and "OBJECTS_SOURCE=detector" in errors[0] and OBJECTS_TOPIC in errors[0]
