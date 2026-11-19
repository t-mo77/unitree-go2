"""The detector container's ROS 2 node (spec §11.1 start-up order, §11.3 topics, §11.5, §11.9; design-detector.md
§3.5; review M8, M9). The detector image runs it as `python3 -m detector.node` (compose service `detector`).

Start-up, in this order:
1. OBJECTS_SOURCE is read once (bridge_core.objects_source). Anything but detector: one line on stdout and exit 0
   before rclpy starts, so compose's `restart: on-failure:3` leaves the container stopped (review M9).
2. The node, its publishers and its steady wall-clock timers: /go2/objects at 2 Hz and /go2/detector_status at 1 Hz.
   Both publish at once: {"frame": "odom", "stamp": ..., "source": "detector", "objects": []} and {"ok": false,
   "reason": "loading"}, so the app sees `source: "detector"` within a second even while the model loads for 15 s.
3. A loader thread: the detector (make_detector(); by default protocol.load_detector, which reads env DETECTOR), its
   set_vocabulary with every kind's detect prompts and the detect_ignore list, a DetectorCore, mark_loaded(). On any
   failure (an unknown DETECTOR, a model or weights the adapter cannot load) the node logs it and main() returns 1,
   so compose restarts it up to 3 times (spec §11.9). The vocabulary file itself is read and checked when this
   module is imported (voice_ltl.vocabulary fails closed at import), so a broken file ends the process with exit
   code 1 before main(), the traceback ending in the loader's one-line message (ruling (c) of the Task 1.1 review).
4. Then, in the executor thread, the subscriptions: odometry, /utlidar/cloud, the camera image (depth 1) and its
   CameraInfo, all qos_profile_sensor_data. A detector with set_truth (the oracle, DETECTOR=oracle) also gets
   /gazebo/objects_truth, whose rows go to set_truth without heights (the oracle uses the vocabulary height, review
   M8). Then a worker thread calls the core's next_job() and, for every job it gets, run(job) at once (one inference
   in flight; the core decides when, only while the Go2 stands still).

The node never subscribes to /sim/* (D9). Every 10 s (wall) it logs the error of bridge_core.objects_publisher_note
when /go2/objects has another publisher. It logs through Python logging as detector.node (the core logs as
detector.core).

The message conversions (image_from_msg, points_from_cloud, stamp_of) use numpy only (np.frombuffer; no cv_bridge,
no sensor_msgs_py). rclpy, std_msgs, sensor_msgs and nav_msgs are imported only when present, so this module and its
pure functions import without ROS 2 (main() then refuses to start).
"""

from __future__ import annotations

import json
import logging
import os
import sys
import threading
import time
from collections.abc import Callable, Mapping

import numpy as np

from detector.core import DetectorCore
from detector.protocol import Detector, load_detector, vocabulary_prompts
from gazebo.bridge_core import PUBLISHER_NOTE_S, TRUTH_TOPIC, objects_publisher_note, objects_source
from voice_ltl import objects, vocabulary
from voice_ltl.objects import OBJECTS_HZ, OBJECTS_TOPIC

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from std_msgs.msg import String
except ImportError:          # no ROS 2 here: the pure functions still work, main() refuses to start
    rclpy = None
    Node = object

STATUS_TOPIC = "/go2/detector_status"; IMAGE_TOPIC = "/gazebo/front_camera/image"
CAMERA_INFO_TOPIC = "/gazebo/front_camera/camera_info"; CLOUD_TOPIC = "/utlidar/cloud"
ODOM_TOPIC = "/model/robot/odometry"
NODE_NAME = "go2_detector"
STATUS_HZ = 1.0
START_POLL_S = 0.1           # wall: how often the executor looks whether the detector has loaded
WORKER_IDLE_S = 0.05         # wall: the worker's wait when there is no job
WORKER_JOIN_S = 5.0          # wall: destroy_node waits this long for an inference in flight
START_HINT = "OBJECTS_SOURCE=detector SIM_KIND=gazebo docker compose --profile gazebo --profile detector up -d"
_POINT_TYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}   # sensor_msgs/PointField

_LOG = logging.getLogger("detector.node")     # not __name__: `python3 -m detector.node` runs as __main__


def startup_refusal(environ: Mapping[str, str]) -> str | None:
    """One line when OBJECTS_SOURCE is not "detector" (unset means logical), else None. main() prints it and returns
    0 before rclpy starts (review M9)."""
    try:
        source = objects_source(environ)
    except ValueError as exc:                     # names the value and the valid ones
        return f"detector not started: {exc}. It runs only with {START_HINT}"
    if source == "detector":
        return None
    given = f"OBJECTS_SOURCE={source}" if "OBJECTS_SOURCE" in environ else "OBJECTS_SOURCE is not set (logical)"
    return (f"detector not started: {given}, so the Gazebo bridge publishes {OBJECTS_TOPIC} from its logical camera. "
            f"The detector runs only with {START_HINT}")


# --- message conversions (numpy only) ----------------------------------------------------------------------------

def stamp_of(msg) -> float:
    """header.stamp in seconds (sim time for the sensors)."""
    stamp = msg.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def image_from_msg(msg) -> np.ndarray:
    """sensor_msgs/Image -> H x W x 3 uint8 RGB (np.frombuffer, no cv_bridge). rgb8 as the camera sends it; bgr8 is
    turned into RGB. Rows may be padded (step > 3 W). ValueError for another encoding or too little data."""
    encoding = str(msg.encoding)
    if encoding.lower() not in ("rgb8", "bgr8"):
        raise ValueError(f"image encoding {encoding!r}: the detector takes rgb8 (or bgr8)")
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    if h < 0 or w < 0 or step < 3 * w:
        raise ValueError(f"image {w} x {h} with step {step}: an rgb8 row needs {3 * w} bytes")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < step * h:
        raise ValueError(f"image {w} x {h} with step {step}: {raw.size} bytes, {step * h} expected")
    image = raw[:step * h].reshape(h, step)[:, :3 * w].reshape(h, w, 3)
    if encoding.lower() == "bgr8":
        image = image[..., ::-1]
    return np.ascontiguousarray(image)


def points_from_cloud(msg) -> np.ndarray:
    """sensor_msgs/PointCloud2 -> N x 3 float64 of its x, y and z fields (any offsets, point step, row padding, byte
    order and numeric PointField type). Rows with a non-finite coordinate are dropped (/utlidar/cloud has none).
    ValueError when x, y or z is missing, the layout is impossible or the data is shorter than the header says."""
    fields = {f.name: f for f in msg.fields}
    missing = [c for c in "xyz" if c not in fields]
    if missing:
        raise ValueError(f"point cloud without the field(s) {', '.join(missing)} "
                         f"(it has {', '.join(fields) or 'none'})")
    order = ">" if msg.is_bigendian else "<"
    formats = []
    for c in "xyz":
        kind = _POINT_TYPES.get(int(fields[c].datatype))
        if kind is None:
            raise ValueError(f"point cloud field {c}: PointField datatype {fields[c].datatype} is not a number type")
        formats.append(order + kind)
    offsets = [int(fields[c].offset) for c in "xyz"]
    width, height, step, row_step = int(msg.width), int(msg.height), int(msg.point_step), int(msg.row_step)
    try:
        dtype = np.dtype({"names": ["x", "y", "z"], "formats": formats, "offsets": offsets, "itemsize": step})
    except (TypeError, ValueError) as exc:
        raise ValueError(f"point cloud layout (offsets {offsets}, point step {step}): {exc}") from None
    if width <= 0 or height <= 0:
        return np.zeros((0, 3))
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    row = width * step
    if row_step < row or raw.size < row_step * (height - 1) + row:
        raise ValueError(f"point cloud {width} x {height} (point step {step}, row step {row_step}): "
                         f"{raw.size} bytes of data")
    if row_step == row:
        points = np.frombuffer(raw, dtype=dtype, count=width * height)
    else:
        points = np.concatenate([np.frombuffer(raw, dtype=dtype, count=width, offset=r * row_step)
                                 for r in range(height)])
    xyz = np.column_stack([points["x"], points["y"], points["z"]]).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


# --- the node ----------------------------------------------------------------------------------------------------

class DetectorNode(Node):
    """See the module docstring. core is None while the detector loads; ready is set once the sensors are subscribed
    and the worker runs; exit_code becomes 1 when loading failed (main() then ends the process)."""

    def __init__(self, make_detector: Callable[[], Detector] = load_detector) -> None:
        super().__init__(NODE_NAME)
        self.core: DetectorCore | None = None
        self.detector: Detector | None = None
        self.ready = threading.Event()
        self.exit_code: int | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._worker: threading.Thread | None = None
        self._warned: set[str] = set()
        # 1. publishers and timers first, publishing at once (spec §11.1)
        self._objects_pub = self.create_publisher(String, OBJECTS_TOPIC, 10)
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        wall = Clock(clock_type=ClockType.STEADY_TIME)      # runs on while Gazebo is paused or gone
        self.create_timer(1.0 / OBJECTS_HZ, self._publish_objects, clock=wall)
        self.create_timer(1.0 / STATUS_HZ, self._publish_status, clock=wall)
        self.create_timer(PUBLISHER_NOTE_S, self._check_publishers, clock=wall)
        self._start_timer = self.create_timer(START_POLL_S, self._start, clock=wall)
        self._publish_objects()
        self._publish_status()
        # 2. the detector, in its own thread (the YOLOE text encoder can take 15 s)
        self._loader = threading.Thread(target=self._load, args=(make_detector,), name="detector-load", daemon=True)
        self._loader.start()

    # --- start-up ------------------------------------------------------------------------------------------------

    def _load(self, make_detector: Callable[[], Detector]) -> None:
        detector = None
        try:
            detector = make_detector()
            prompts, ignore = vocabulary_prompts(vocabulary.VOCABULARY)
            detector.set_vocabulary(prompts, ignore)
            core = DetectorCore(detector, vocabulary.VOCABULARY)
            core.mark_loaded()
        except Exception as exc:
            line = (f"the detector failed to load, so the node stops (exit 1; compose restarts it up to 3 times): "
                    f"{type(exc).__name__}: {exc}")
            _LOG.error(line, exc_info=not isinstance(exc, ValueError))   # a ValueError names the bad value itself
            if detector is not None:
                _close(detector)
            self.exit_code = 1
            return
        with self._lock:
            stopped = self._stop.is_set()                    # destroyed while it loaded
            if not stopped:
                self.detector, self.core = detector, core
        if stopped:
            _close(detector)

    def _start(self) -> None:
        """The start-up timer: once the detector has loaded, the subscriptions and the worker (spec §11.1)."""
        with self._lock:
            core, detector = self.core, self.detector
        if core is None or self.ready.is_set():
            return
        self._start_timer.cancel()
        sensor = qos_profile_sensor_data
        image_qos = QoSProfile(history=sensor.history, depth=1, reliability=sensor.reliability,
                               durability=sensor.durability)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, sensor)
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self._on_cloud, sensor)
        self.create_subscription(Image, IMAGE_TOPIC, self._on_image, image_qos)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._on_camera_info, sensor)
        topics = [ODOM_TOPIC, CLOUD_TOPIC, IMAGE_TOPIC, CAMERA_INFO_TOPIC]
        if callable(getattr(detector, "set_truth", None)):  # the oracle (measurement only)
            self.create_subscription(String, TRUTH_TOPIC, self._on_truth, 10)
            topics.append(TRUTH_TOPIC)
        self._worker = threading.Thread(target=self._work, args=(core,), name="detector-worker", daemon=True)
        self._worker.start()
        self.ready.set()
        _LOG.info("detector %s loaded; listening on %s", getattr(detector, "name", "?"), ", ".join(topics))

    def _work(self, core: DetectorCore) -> None:
        """One inference at a time: next_job(), then at once run(job) (the core's contract), until destroyed."""
        while not self._stop.is_set():
            try:
                job = core.next_job()
                if job is not None:
                    core.run(job)
                    continue
            except Exception:
                _LOG.exception("the detector worker failed; it goes on")
            self._stop.wait(WORKER_IDLE_S)

    # --- sensors (only after loading, so the core exists) ---------------------------------------------------------

    def _on_odom(self, msg) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular       # the base frame, as the bridge's StillGate gets
        self.core.on_odom(stamp_of(msg), (p.x, p.y, p.z), (q.w, q.x, q.y, q.z), (v.x, v.y, v.z), (w.x, w.y, w.z))

    def _on_cloud(self, msg) -> None:
        try:
            points = points_from_cloud(msg)
        except ValueError as exc:
            self._warn_once(f"ignoring a {CLOUD_TOPIC} message: {exc}")
            return
        self.core.on_cloud(stamp_of(msg), points)

    def _on_image(self, msg) -> None:
        try:
            image = image_from_msg(msg)
        except ValueError as exc:
            self._warn_once(f"ignoring a {IMAGE_TOPIC} message: {exc}")
            return
        self.core.on_image(stamp_of(msg), image)

    def _on_camera_info(self, msg) -> None:
        self.core.on_camera_info(tuple(msg.k), int(msg.width), int(msg.height))

    def _on_truth(self, msg) -> None:
        try:
            payload = json.loads(msg.data)
        except ValueError as exc:
            self._warn_once(f"ignoring a {TRUTH_TOPIC} message that is not JSON: {exc}")
            return
        self.detector.set_truth(objects.parse(payload))   # no heights: the vocabulary's (review M8)

    def _warn_once(self, line: str) -> None:
        if line not in self._warned:
            self._warned.add(line)
            _LOG.warning(line)

    # --- output --------------------------------------------------------------------------------------------------

    def _publish_objects(self) -> None:
        core = self.core
        msg = core.message() if core is not None else objects.format_message((), time.time(), source="detector")
        self._objects_pub.publish(String(data=json.dumps(msg)))

    def _publish_status(self) -> None:
        core = self.core
        status = core.health() if core is not None else {"ok": False, "reason": "loading"}
        self._status_pub.publish(String(data=json.dumps(status)))

    def _check_publishers(self) -> None:
        """Every PUBLISHER_NOTE_S: an error when /go2/objects has another publisher (it does not stop; spec §11.9)."""
        note = objects_publisher_note("detector", self.count_publishers(OBJECTS_TOPIC))
        if note is not None:
            _LOG.error(note)

    def destroy_node(self) -> bool:
        self._stop.set()
        worker = self._worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(WORKER_JOIN_S)
        with self._lock:
            detector, self.detector = self.detector, None
        if detector is not None:
            _close(detector)
        return super().destroy_node()


def _close(detector: Detector) -> None:
    try:
        detector.close()
    except Exception:
        _LOG.exception("closing the detector failed")


def main() -> int:
    """python3 -m detector.node. 0 on a refusal (before rclpy starts) or a normal stop; 1 when the detector failed
    to load (spec §11.9)."""
    refusal = startup_refusal(os.environ)
    if refusal is not None:
        print(refusal, flush=True)
        return 0
    if rclpy is None:
        print("detector not started: rclpy, std_msgs, sensor_msgs or nav_msgs is missing (it runs in the detector "
              "image)", file=sys.stderr, flush=True)
        return 1
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] [%(asctime)s] [%(name)s]: %(message)s")
    rclpy.init()
    node = None
    executor = SingleThreadedExecutor()
    code = 0
    try:
        node = DetectorNode()
        executor.add_node(node)
        _LOG.info("detector node up: %s and %s publish while the detector loads", OBJECTS_TOPIC, STATUS_TOPIC)
        while rclpy.ok() and node.exit_code is None:
            executor.spin_once(timeout_sec=0.1)
        code = node.exit_code or 0
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
