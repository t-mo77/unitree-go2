#!/usr/bin/env python3
"""Record one still camera image and LiDAR scan pair in Gazebo, for detector_check.py (spec §11.11, §12.8;
design-detector §12.2).

It runs inside the gazebo container (ROS 2 Jazzy rclpy, numpy 1.26, the gz CLI). Deviation from spec §11.11, which
puts it in the detector image: --shift needs `gz service`, which only the gazebo image has; the gazebo image has
no OpenCV and no verified PIL, so the PNG is written with gazebo/tools/png_rgb.py (plan review I8).

    python3 gazebo/tools/record_frames.py <out_dir> --view <name> [--shift dx,dy,dyaw_deg] [--layout household]

It waits until the Go2 has stood still for the bridge's StillGate (bridge_core.StillGate: each angular rate
<= 0.1 rad/s and speed <= 0.05 m/s for >= 0.3 s, sim time), then pairs the newest image and the newest scan of that
still interval (odometry reaching both stamps, the poses at the two stamps within 0.02 m and 1 deg, like
detector/core.py) and the first /gazebo/objects_truth that arrived at least TRUTH_AFTER_S after the image, and saves
into <out_dir>:
- <view>.png: /gazebo/front_camera/image as RGB (png_rgb.write_png);
- <view>_scan.npy: /utlidar/cloud, N x 3 float32 in the LiDAR frame (body and ground removed by the bridge);
- <view>.json: pair_record(): both stamps and the base poses at them (odom), the still interval's start, the
  CameraInfo (K, width, height), the truth message, the last /sim/world, the last /go2/detector_status (null without
  a detector), the layout name and the shift.
It prints one JSON line ({"view", "files", ...} or {"error"}) and exits 0 (saved), 1 (nothing paired within
--timeout, a set_pose that failed, or the room not moved back) or 2 (bad arguments or layout).

--shift dx,dy,dyaw_deg emulates the Go2 standing at its pose moved by (dx, dy) m and dyaw degrees in its own frame
(dx forward, dy left, dyaw counter-clockwise; plan H3's four shifts are (±0.1, ±0.1, ±5)). The Go2 does not move:
every model of the layout is moved by the inverse of that shift about the Go2 (shifted_pose) with
`gz service -s /world/empty/set_pose`, the way the camera spike moved its room. The pair counts only once the move
has settled (SHIFT_SETTLE_S of sim time) and the truth shows every visible moved model where it was put
(SHIFT_CHECK_M), so a set_pose that Gazebo ignored fails instead of recording the unmoved room. Afterwards every
model goes back to its layout pose, also after an error or Ctrl-C. A model turned by dyaw keeps its layout footprint
in the truth (the bridge's axis-aligned size), a small error at 5 deg.

rclpy and the message types are imported inside main() only; the module imports the standard library, numpy and
this repository, so its pure parts (shifted_pose, pair_record, image_rgb, Recorder) are tested in the app image.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from collections import deque
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]          # this repository (/work in the gazebo container)
for _path in (ROOT / "nl_control", ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from detector.core import PAIR_MAX_SHIFT_DEG, PAIR_MAX_SHIFT_M, PoseBuffer  # noqa: E402
from gazebo.bridge_core import TRUTH_TOPIC, StillGate, spawn_fields  # noqa: E402
from gazebo.tools.png_rgb import write_png  # noqa: E402
from voice_ltl import layout as layouts  # noqa: E402
from voice_ltl.objects import parse  # noqa: E402

RECORD_VERSION = 1
IMAGE_TOPIC = "/gazebo/front_camera/image"
CAMERA_INFO_TOPIC = "/gazebo/front_camera/camera_info"
CLOUD_TOPIC = "/utlidar/cloud"
ODOM_TOPIC = "/model/robot/odometry"
WORLD_TOPIC = "/sim/world"
STATUS_TOPIC = "/go2/detector_status"
WORLD_NAME = "empty"                       # gazebo_bridge.WORLD_NAME
SET_POSE_SERVICE = f"/world/{WORLD_NAME}/set_pose"
DEFAULT_LAYOUT_NAME = "household"          # GO2_LAYOUT unset: the Gazebo default (spec §12.5)
NODE_NAME = "go2_record_frames"
WAIT_S = 60.0                              # --timeout default (wall)
TRUTH_AFTER_S = 0.5                        # wall: the truth (2 Hz) must arrive this long after the image
SHIFT_SETTLE_S = 1.0                       # sim: after the models moved, images and scans count from this on
SHIFT_CHECK_M = 0.05                       # a visible moved model's truth centre must be this close to its place
GZ_TIMEOUT_MS = 3000
GZ_WAIT_S = 10.0                           # wall, for one `gz service` call
SPIN_S = 0.05
VIEW_SHAPE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}")

Pose = tuple[tuple[float, float, float], tuple[float, float, float, float]]   # base position, quaternion (w, x, y, z)


# --- pure helpers ----------------------------------------------------------------------------------------------------

def parse_shift(text: str) -> tuple[float, float, float]:
    """"dx,dy,dyaw_deg" -> three finite numbers (ValueError otherwise; argparse reports it)."""
    parts = str(text).split(",")
    if len(parts) != 3:
        raise ValueError(f"--shift is dx,dy,dyaw_deg (three numbers): {text!r}")
    values = tuple(float(p) for p in parts)
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"--shift needs finite numbers: {text!r}")
    return values


def shifted_pose(x: float, y: float, yaw_deg: float, shift: Sequence[float],
                 robot: Sequence[float] = (0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    """Where to put a model at (x, y, yaw_deg) (odom) so that the Go2 at robot = (x, y, yaw_deg) sees it as it would
    see the unmoved model from its pose moved by shift = (dx, dy, dyaw_deg) in its own frame:
    M' = R · S⁻¹ · R⁻¹ · M (planar poses). The yaw is returned in (-180, 180]."""
    rx, ry, ryaw = (float(v) for v in robot)
    dx, dy, dyaw = (float(v) for v in shift)
    # the model in the robot's frame
    c, s = math.cos(math.radians(ryaw)), math.sin(math.radians(ryaw))
    px, py = float(x) - rx, float(y) - ry
    lx, ly, lyaw = c * px + s * py, -s * px + c * py, float(yaw_deg) - ryaw
    # ... seen from the shifted robot: S⁻¹ · L
    c2, s2 = math.cos(math.radians(dyaw)), math.sin(math.radians(dyaw))
    qx, qy = lx - dx, ly - dy
    mx, my, myaw = c2 * qx + s2 * qy, -s2 * qx + c2 * qy, lyaw - dyaw
    # ... back in odom: R · (S⁻¹ · L)
    return rx + c * mx - s * my, ry + s * mx + c * my, _wrap_deg(myaw + ryaw)


def set_pose_command(name: str, x: float, y: float, z: float, yaw_deg: float, world: str = WORLD_NAME,
                     timeout_ms: int = GZ_TIMEOUT_MS) -> list[str]:
    """The `gz service` call that puts model name at (x, y, z) turned by yaw_deg about z."""
    half = math.radians(float(yaw_deg)) / 2
    req = (f'name: "{name}" position: {{x: {float(x):.6f} y: {float(y):.6f} z: {float(z):.6f}}} '
           f"orientation: {{w: {math.cos(half):.9f} x: 0 y: 0 z: {math.sin(half):.9f}}}")
    return ["gz", "service", "-s", f"/world/{world}/set_pose", "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean", "--timeout", str(int(timeout_ms)), "--req", req]


def image_rgb(width: int, height: int, encoding: str, step: int, data: bytes) -> np.ndarray:
    """sensor_msgs/Image fields -> H x W x 3 uint8 RGB (a copy). rgb8, bgr8, rgba8 and bgra8; ValueError otherwise."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
    w, h, step = int(width), int(height), int(step)
    if channels is None:
        raise ValueError(f"image encoding {encoding!r}: only rgb8, bgr8, rgba8 and bgra8 are read")
    if w < 1 or h < 1 or step < w * channels or len(data) < step * h:
        raise ValueError(f"image {w} x {h} {encoding}: step {step} and {len(data)} bytes do not fit")
    a = np.frombuffer(bytes(data[:step * h]), dtype=np.uint8).reshape(h, step)[:, :w * channels]
    a = a.reshape(h, w, channels)[:, :, :3]
    return np.ascontiguousarray(a[:, :, ::-1] if encoding.startswith("bgr") else a)


def pair_record(view: str, image_t: float, image_pose: Pose, cloud_t: float, cloud_pose: Pose,
                camera_info: Mapping | None, truth: object, world: object, status: object, *,
                still_since: float | None = None, layout: str | None = None,
                shift: Sequence[float] | None = None, robot: Sequence[float] | None = None,
                moved: Mapping[str, Sequence[float]] | None = None) -> dict:
    """The <view>.json of one pair (plain JSON). image_pose / cloud_pose: the base pose (position, quaternion
    (w, x, y, z), odom) at each stamp; camera_info: {"k": 9 numbers, "width", "height"} or None; truth, world, status:
    the last /gazebo/objects_truth, /sim/world and /go2/detector_status JSON (or None); shift, robot (x, y, yaw_deg)
    and moved ({id: [x, y, yaw_deg]}) only for --shift. ValueError for a view that is not a file stem
    (VIEW_SHAPE) or a non-finite stamp or pose."""
    if not (isinstance(view, str) and VIEW_SHAPE.fullmatch(view)):
        raise ValueError(f"a view name is 1-64 letters, digits, _ or - (not starting with _ or -): {view!r}")
    info = None
    if camera_info is not None:
        info = {"k": _numbers(camera_info["k"], 9, "camera_info k"),
                "width": int(camera_info["width"]), "height": int(camera_info["height"])}
    return {"version": RECORD_VERSION, "view": view, "image": f"{view}.png", "scan": f"{view}_scan.npy",
            "image_t": _number(image_t, "image_t"), "cloud_t": _number(cloud_t, "cloud_t"),
            "still_since": None if still_since is None else _number(still_since, "still_since"),
            "image_pose": _pose(image_pose, "image_pose"), "cloud_pose": _pose(cloud_pose, "cloud_pose"),
            "camera_info": info, "truth": truth, "world": world, "status": status, "layout": layout,
            "shift": None if shift is None else _numbers(shift, 3, "shift"),
            "robot": None if robot is None else _numbers(robot, 3, "robot"),
            "moved": {k: _numbers(v, 3, f"moved {k}") for k, v in sorted(dict(moved or {}).items())}}


def save_pair(out_dir: str | os.PathLike, record: Mapping, image: np.ndarray, scan: np.ndarray) -> list[Path]:
    """Write <view>.png, <view>_scan.npy (float32 N x 3) and, last, <view>.json into out_dir (made if missing)."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    png, npy, js = out / record["image"], out / record["scan"], out / f"{record['view']}.json"
    write_png(png, image)
    np.save(npy, np.asarray(scan, dtype=np.float32).reshape(-1, 3))
    js.write_text(json.dumps(record, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    return [png, npy, js]


def shift_mismatch(truth: object, expect: Mapping[str, tuple[float, float]], tol: float = SHIFT_CHECK_M) -> list[str]:
    """The ids of the truth message's visible rows that expect places elsewhere (farther than tol), sorted."""
    return sorted(o.id for o in parse(truth) if o.visible and o.id in expect
                  and math.hypot(o.x - expect[o.id][0], o.y - expect[o.id][1]) > tol)


def layout_moves(lay: layouts.Layout, shift: Sequence[float], robot: Sequence[float]
                 ) -> list[tuple[str, tuple[float, float, float, float], tuple[float, float, float, float],
                                 tuple[float, float]]]:
    """(id, the layout pose (x, y, z, yaw_deg), the shifted pose, the truth centre the bridge then reports) of every
    layout model; z stays the spawn height (bridge_core.spawn_fields)."""
    out = []
    for e in lay.entries:
        _sdf, x, y, z, _yaw = spawn_fields(e)
        mx, my, myaw = shifted_pose(x, y, e.yaw_deg, shift, robot)
        dx, dy = e.world_offset()                     # bridge_core.logical_rows: origin + the layout's offset
        out.append((e.id, (x, y, z, float(e.yaw_deg)), (mx, my, z, myaw), (mx + dx, my + dy)))
    return out


@dataclass(frozen=True, eq=False)
class Pair:
    image_t: float
    image: np.ndarray
    image_pose: Pose
    cloud_t: float
    cloud: np.ndarray
    cloud_pose: Pose
    still_since: float
    truth: object


@dataclass(frozen=True, eq=False)
class _Kept:
    t: float
    data: np.ndarray
    still_since: float
    wall: float


class Recorder:
    """The waiting of record_frames without ROS: feed it the messages, ask pair(). Set not_before (sim) and expect
    ({id: truth centre}) for a shifted view."""

    def __init__(self) -> None:
        self.gate = StillGate()
        self.poses = PoseBuffer()
        self.not_before: float | None = None
        self.expect: dict[str, tuple[float, float]] = {}
        self.mismatch: list[str] = []
        self.camera_info: dict | None = None
        self.world: object = None
        self.status: object = None
        self._image: _Kept | None = None
        self._cloud: _Kept | None = None
        self._truth: deque[tuple[float, object]] = deque(maxlen=32)
        self.latest_t: float | None = None

    def on_odom(self, t: float, pos, quat, linear, angular) -> None:
        self.gate.observe(float(t), tuple(linear), tuple(angular))
        self.poses.add(t, pos, quat)
        self.latest_t = self.poses.latest()

    def _keeps(self, t: float) -> bool:
        return self.gate.allows(t) and (self.not_before is None or t >= self.not_before - 1e-9)

    def on_image(self, t: float, rgb: np.ndarray, wall: float) -> None:
        t = float(t)
        if self._keeps(t):
            self._image = _Kept(t, rgb, self.gate.still_since, float(wall))

    def on_cloud(self, t: float, points: np.ndarray) -> None:
        t = float(t)
        if self._keeps(t):
            self._cloud = _Kept(t, np.asarray(points), self.gate.still_since, 0.0)

    def on_camera_info(self, info: Mapping) -> None:
        self.camera_info = {"k": [float(v) for v in info["k"]], "width": int(info["width"]),
                            "height": int(info["height"])}

    def on_truth(self, payload: object, wall: float) -> None:
        self._truth.append((float(wall), payload))

    def on_world(self, payload: object) -> None:
        self.world = payload

    def on_status(self, payload: object) -> None:
        self.status = payload

    def robot(self) -> tuple[float, float, float] | None:
        """The latest base pose (x, y, yaw_deg), or None before odometry."""
        if self.latest_t is None:
            return None
        (x, y, _z), (w, qx, qy, qz) = self.poses.at(self.latest_t)
        return x, y, math.degrees(math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))

    def still(self) -> bool:
        return self.latest_t is not None and self.gate.allows(self.latest_t)

    def pair(self) -> Pair | None:
        """The pair to save, or None while something is missing (missing() says what)."""
        img, cloud = self._image, self._cloud
        since = self.gate.still_since
        if img is None or cloud is None or since is None or img.still_since != since or cloud.still_since != since:
            return None
        pi, pc = self.poses.at(img.t), self.poses.at(cloud.t)
        if pi is None or pc is None:
            return None
        if math.dist(pi[0], pc[0]) > PAIR_MAX_SHIFT_M + 1e-9 or _turn_deg(pi[1], pc[1]) > PAIR_MAX_SHIFT_DEG + 1e-9:
            self._image = None                            # the Go2 moved between the stamps: take the next image
            return None
        if self.camera_info is None:
            return None
        truth = None
        for wall, payload in self._truth:                  # the first truth late enough (and showing the shift)
            if wall < img.wall + TRUTH_AFTER_S - 1e-9:
                continue
            self.mismatch = shift_mismatch(payload, self.expect) if self.expect else []
            if not self.mismatch:
                truth = payload
                break
        if truth is None:
            return None
        return Pair(img.t, img.data, pi, cloud.t, cloud.data, pc, since, truth)

    def missing(self) -> list[str]:
        """What pair() still waits for, in words (for the timeout's error line)."""
        out = []
        if self.latest_t is None:
            out.append(f"odometry ({ODOM_TOPIC})")
        elif not self.still():
            out.append("a still Go2 (the still gate is closed)")
        since = self.gate.still_since
        if self._image is None or self._image.still_since != since:
            out.append(f"an image in the open still gate ({IMAGE_TOPIC}"
                       + (", after the shift settled)" if self.not_before is not None else ")"))
        if self._cloud is None or self._cloud.still_since != since:
            out.append(f"a scan in the open still gate ({CLOUD_TOPIC})")
        if self.camera_info is None:
            out.append(f"CameraInfo ({CAMERA_INFO_TOPIC})")
        if self.mismatch:
            out.append(f"{TRUTH_TOPIC} showing {', '.join(self.mismatch)} moved by the shift (set_pose ignored?)")
        elif self._image is not None and not any(w >= self._image.wall + TRUTH_AFTER_S - 1e-9 for w, _ in self._truth):
            out.append(f"{TRUTH_TOPIC} after the image")
        return out


def _wrap_deg(a: float) -> float:
    a = math.fmod(float(a), 360.0)
    if a <= -180.0:
        a += 360.0
    elif a > 180.0:
        a -= 360.0
    return a


def _turn_deg(q0: Sequence[float], q1: Sequence[float]) -> float:
    dot = abs(sum(float(a) * float(b) for a, b in zip(q0, q1)))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def _number(v: object, what: str) -> float:
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise ValueError(f"{what} must be a number: {v!r}") from None
    if not math.isfinite(f):
        raise ValueError(f"{what} must be finite: {v!r}")
    return f


def _numbers(v: object, n: int, what: str) -> list[float]:
    try:
        values = list(v)
    except TypeError:
        raise ValueError(f"{what} must be {n} numbers: {v!r}") from None
    if len(values) != n:
        raise ValueError(f"{what} must be {n} numbers: {v!r}")
    return [_number(x, what) for x in values]


def _pose(p: object, what: str) -> dict:
    try:
        pos, quat = p
    except (TypeError, ValueError):
        raise ValueError(f"{what} is (position, quaternion): {p!r}") from None
    return {"pos": _numbers(pos, 3, f"{what} position"), "quat": _numbers(quat, 4, f"{what} quaternion")}


# --- the command line (ROS) ----------------------------------------------------------------------------------------

def _parse(argv: list[str] | None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Record one still image and scan pair in Gazebo (spec §11.11).")
    p.add_argument("out_dir", help="the set's folder (made if missing)")
    p.add_argument("--view", required=True, help="the view name: the files are <view>.png, <view>_scan.npy, "
                                                 "<view>.json (plan H3: spawn, spawn_lying, B, B_s1, ...)")
    p.add_argument("--shift", type=_shift_arg, default=None, metavar="dx,dy,dyaw_deg",
                   help="emulate the Go2 moved by (dx, dy) m and dyaw deg in its own frame by moving the layout")
    p.add_argument("--layout", default=None, help="the layout (default: GO2_LAYOUT, else household)")
    p.add_argument("--timeout", type=float, default=WAIT_S, help=f"seconds to wait (wall, default {WAIT_S:g})")
    args = p.parse_args(argv)
    if not VIEW_SHAPE.fullmatch(args.view):
        p.error(f"--view must be 1-64 letters, digits, _ or - (not starting with _ or -): {args.view!r}")
    return args


def _shift_arg(text: str) -> tuple[float, float, float]:
    try:
        return parse_shift(text)
    except ValueError as e:
        raise argparse.ArgumentTypeError(str(e)) from None


def _print(payload: dict) -> None:
    print(json.dumps(payload, ensure_ascii=False), flush=True)


def _say(text: str) -> None:
    print(f"[record_frames] {text}", file=sys.stderr, flush=True)


def _gz_set_pose(name: str, pose: Sequence[float]) -> bool:
    try:
        done = subprocess.run(set_pose_command(name, *pose), capture_output=True, text=True, timeout=GZ_WAIT_S)
    except (OSError, subprocess.TimeoutExpired) as e:
        _say(f"gz service set_pose {name}: {e}")
        return False
    if done.returncode != 0 or "data: true" not in done.stdout:
        _say(f"gz service set_pose {name}: exit {done.returncode}: {(done.stdout + done.stderr).strip()[:200]}")
        return False
    return True


def _stamp(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _json(text: str) -> object:
    try:
        return json.loads(text)
    except ValueError:
        return None


def main(argv: list[str] | None = None) -> int:
    args = _parse(argv)
    name = args.layout or os.environ.get("GO2_LAYOUT") or DEFAULT_LAYOUT_NAME
    try:
        lay = layouts.load(name)
    except layouts.LayoutError as e:
        _print({"error": str(e)})
        return 2
    if args.shift is not None and shutil.which("gz") is None:
        _print({"error": "--shift needs the gz CLI (run record_frames in the gazebo container)"})
        return 2

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node(NODE_NAME)
    rec = Recorder()

    def on_odom(msg) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        rec.on_odom(_stamp(msg.header), (p.x, p.y, p.z), (q.w, q.x, q.y, q.z), (v.x, v.y, v.z), (w.x, w.y, w.z))

    def on_image(msg) -> None:
        try:
            rgb = image_rgb(msg.width, msg.height, msg.encoding, msg.step, msg.data)
        except ValueError as e:
            _say(str(e))
            return
        rec.on_image(_stamp(msg.header), rgb, time.monotonic())

    def on_cloud(msg) -> None:
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        rec.on_cloud(_stamp(msg.header), np.asarray(pts, dtype=np.float32).reshape(-1, 3))

    image_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Odometry, ODOM_TOPIC, on_odom, qos_profile_sensor_data)
    node.create_subscription(Image, IMAGE_TOPIC, on_image, image_qos)
    node.create_subscription(CameraInfo, CAMERA_INFO_TOPIC,
                             lambda m: rec.on_camera_info({"k": list(m.k), "width": m.width, "height": m.height}),
                             qos_profile_sensor_data)
    node.create_subscription(PointCloud2, CLOUD_TOPIC, on_cloud, qos_profile_sensor_data)
    node.create_subscription(String, TRUTH_TOPIC, lambda m: rec.on_truth(_json(m.data), time.monotonic()), 10)
    node.create_subscription(String, WORLD_TOPIC, lambda m: rec.on_world(_json(m.data)), 10)
    node.create_subscription(String, STATUS_TOPIC, lambda m: rec.on_status(_json(m.data)), 10)

    deadline = time.monotonic() + args.timeout
    restore: list[tuple[str, tuple[float, float, float, float]]] = []
    result: dict = {}
    code = 1
    robot = None
    moved: dict[str, tuple[float, float, float]] = {}

    def spin_until(done) -> bool:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=SPIN_S)
            if done():
                return True
        return False

    try:
        if args.shift is not None:
            if not spin_until(rec.still):
                raise _Failure(f"no still Go2 within {args.timeout:g} s to shift about "
                               f"(missing: {'; '.join(rec.missing())})")
            robot = rec.robot()
            for obj_id, home, pose, centre in layout_moves(lay, args.shift, robot):
                restore.append((obj_id, home))
                if not _gz_set_pose(obj_id, pose):
                    raise _Failure(f"gz service {SET_POSE_SERVICE} failed for {obj_id}")
                moved[obj_id] = (pose[0], pose[1], pose[3])
                rec.expect[obj_id] = centre
            rclpy.spin_once(node, timeout_sec=SPIN_S)
            rec.not_before = (rec.latest_t or 0.0) + SHIFT_SETTLE_S
        if not spin_until(lambda: rec.pair() is not None):
            raise _Failure(f"no still pair within {args.timeout:g} s (missing: {'; '.join(rec.missing())})")
        pair = rec.pair()
        record = pair_record(args.view, pair.image_t, pair.image_pose, pair.cloud_t, pair.cloud_pose,
                             rec.camera_info, pair.truth, rec.world, rec.status, still_since=pair.still_since,
                             layout=lay.name, shift=args.shift, robot=robot, moved=moved)
        files = save_pair(args.out_dir, record, pair.image, pair.cloud)
        visible = sorted(o.id for o in parse(pair.truth) if o.visible)
        result = {"view": args.view, "files": [str(f) for f in files], "image_t": pair.image_t,
                  "cloud_t": pair.cloud_t, "points": int(len(pair.cloud)), "visible": visible,
                  "shift": record["shift"]}
        code = 0
    except _Failure as e:
        result = {"view": args.view, "error": str(e)}
    except KeyboardInterrupt:
        result = {"view": args.view, "error": "interrupted"}
    finally:
        back = [obj_id for obj_id, home in restore if not _gz_set_pose(obj_id, home)]
        if back:
            result["restore_error"] = (f"not moved back to the layout: {', '.join(back)} "
                                       "(restart Gazebo before the next recording)")
            code = 1
        node.destroy_node()
        rclpy.try_shutdown()
    _print(result)
    return code


class _Failure(Exception):
    pass


if __name__ == "__main__":
    sys.exit(main())
