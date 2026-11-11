"""The detector's core: when to detect, pairing, rate limits, failures and the status (spec §11.5, §11.9;
design-detector.md §6 and §10). The node (detector/node.py) feeds it the sensors and runs a worker thread that
calls next_job() and run(); the 2 Hz timer publishes message(), the 1 Hz timer health(). Every method is
thread-safe; run() holds no lock while the detector works.

When (§11.5):
- on_odom feeds bridge_core.StillGate (the bridge's own) and the PoseBuffer. An image or a scan is kept, with the
  gate's still_since at its arrival, only if the gate allows its stamp (still for >= 0.3 s before it) and odometry
  is fresh; otherwise it is dropped (images are counted in frames_dropped).
- next_job pairs the newest kept image with the newest kept scan of the same still interval, while that interval
  is still going on, once odometry has reached both stamps (waiting at most PAIR_WAIT_S, wall), and only if the
  poses at the two stamps are within PAIR_MAX_SHIFT_M and PAIR_MAX_SHIFT_DEG. A newer image replaces an older one;
  one replaced in the same interval without any scan of that interval is dropped with the status no_cloud.
- Rate limits (wall clock, steady): one job in flight; from the end of one inference to the start of the next
  max(DET_MIN_GAP_S, the p50 of the last 10 latencies); after DET_SETTLE_FRAMES inferences in one still interval,
  one every idle period (start to start) until the interval ends. The idle period is DET_IDLE_PERIOD_S, doubled
  after each inference while that p50 exceeds SLOW_LATENCY_S (status slow), at most DET_IDLE_MAX_S, and back to
  DET_IDLE_PERIOD_S once the p50 is at or below it.

Geometry and tracking: run() calls the detector's prepare(view) when it has one, then detect(image), and turns
each usable detection (a label of the vocabulary allowlist, not an ignored prompt, confidence >= the detector's
sighting threshold, a box inside the image) into a tracker Observation through geometry.observe and the 1:1
mapping of Task 2.1: the image-stamp pose for the camera, the scan-stamp pose for the LiDAR points
(bridge_core.lidar_points_world), so a result that lands after the Go2 walked off is still placed right. A
sighting without a footprint makes no Observation and counts in health()["shapeless"]. The tracker's in_view is
make_in_view(): within 6 m, inside the camera's horizontal view and not occluded by the LiDAR returns.

Forgetting: on every kept still scan (any direction, at most VACANCY_MAX_HZ, wall) the vacancy test of
geometry.vacancy_counts runs for the tracker's LiDAR-supported confirmed tracks, also while the camera is gone.

Clocks (M3): timers and rate limits use wall (steady); the gate, the pose lookups and every "0.5 s apart" use the
sensor stamps; the rows' seen is clock() (time.time()) at the image's arrival. Any odometry, image or scan stamp
more than TIME_BACK_S before the latest one seen calls reset("sim time went backwards") (Gazebo restarted, M4):
gate, poses, kept frames and tracks are cleared, and the ids restart at 1.

Additions to the plan's constants (no number of the spec changes): ODOM_TIMEOUT_S (odometry older than this, wall,
closes the gate and reads no_odometry), CAMERA_WARN_S (no_camera warns once a minute, spec §11.9), and the in_view
constants OCCLUDED_SHORT_M / OCCLUDED_SHARE (design-detector §8.6) and SEEN_MARGIN.

numpy, the standard library and this repository only (no ML, no rclpy).
"""

from __future__ import annotations

import bisect
import logging
import math
import statistics
import threading
import time
from collections import deque
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass

import numpy as np

from detector.camera import CAMERA_H, CAMERA_W, PROJ_NEAR, CameraModel, camera_pose, world_to_camera
from detector.geometry import LIDAR_STEP, VACANCY_MIN_HEIGHT, cut, footprint, is_vacant, observe, vacancy_counts
from detector.protocol import Detection, Detector, check_detections
from detector.tracker import INVIEW_RANGE, Observation, Tracker
from gazebo.bridge_core import StillGate, lidar_points_world
from voice_ltl import objects, vocabulary
from voice_ltl.vocabulary import Vocabulary

POSE_BUFFER_S = 2.0; PAIR_WAIT_S = 0.1; PAIR_MAX_SHIFT_M = 0.02; PAIR_MAX_SHIFT_DEG = 1.0
DET_MIN_GAP_S = 1.0; DET_SETTLE_FRAMES = 3; DET_IDLE_PERIOD_S = 3.0; DET_IDLE_MAX_S = 12.0; SLOW_LATENCY_S = 1.5
CAMERA_TIMEOUT_S = 10.0; ERROR_LIMIT = 3; ERROR_BACKOFF_S = 30.0; VACANCY_MAX_HZ = 2.0; TIME_BACK_S = 1.0
ODOM_TIMEOUT_S = 1.0          # wall: odometry older than this closes the gate (status no_odometry)
CAMERA_WARN_S = 60.0          # no_camera warns once a minute
LATENCY_WINDOW = 10           # p50 / p90 over the last 10 inferences
OCCLUDED_SHORT_M = 0.3        # in view: a return this far short of the object's near side blocks its ray ...
OCCLUDED_SHARE = 0.5          # ... and more than this share of the returns in its bearing window blocked hides it
SEEN_MARGIN = 0.1             # in view: within a footprint of this image's sightings (+0.1 m) is seen

_EPS = 1e-6
_LOG = logging.getLogger("detector.core")


@dataclass(frozen=True)
class View:
    """Where the camera was for one image: the base pose at the image stamp and the intrinsics in use."""

    base_pos: tuple[float, float, float]
    base_quat: tuple[float, float, float, float]       # (w, x, y, z)
    camera: CameraModel
    stamp: float                                       # the image stamp (sim)


@dataclass(frozen=True, eq=False)
class Job:
    """One image and the scan it is paired with (one still interval)."""

    image: np.ndarray            # H x W x 3 uint8 RGB
    image_t: float               # sim stamps
    cloud: np.ndarray            # N x 3, the LiDAR frame
    cloud_t: float
    still_since: float           # the gate's still_since when both arrived
    arrived: float | None = None  # clock() when the image arrived (the rows' seen); None: clock() at run


class PoseBuffer:
    """The last POSE_BUFFER_S (sim) of odometry poses, looked up at any stamp between two samples: position
    interpolated linearly, orientation by normalised lerp. Not thread-safe (the core locks around it)."""

    def __init__(self, span_s: float = POSE_BUFFER_S) -> None:
        self._span = float(span_s)
        self._t: list[float] = []
        self._pose: list[tuple[tuple[float, float, float], tuple[float, float, float, float]]] = []

    def add(self, t: float, pos, quat) -> None:
        """One sample (sim time t, base position, quaternion (w, x, y, z)); a non-finite one is ignored."""
        try:
            t = float(t)
            p = tuple(float(v) for v in pos)[:3]
            q = np.asarray([float(v) for v in quat][:4], dtype=np.float64)
        except (TypeError, ValueError):
            return
        n = float(np.linalg.norm(q)) if len(q) == 4 else 0.0
        if len(p) != 3 or not (math.isfinite(t) and all(math.isfinite(v) for v in p) and math.isfinite(n)) or n < 1e-9:
            return
        pose = (p, tuple(float(v) for v in q / n))
        i = bisect.bisect_left(self._t, t)
        if i < len(self._t) and self._t[i] == t:
            self._pose[i] = pose
        else:
            self._t.insert(i, t)
            self._pose.insert(i, pose)
        cut_at = bisect.bisect_left(self._t, self._t[-1] - self._span - 1e-9)
        if cut_at:
            del self._t[:cut_at]
            del self._pose[:cut_at]

    def at(self, t: float) -> tuple[tuple[float, float, float], tuple[float, float, float, float]] | None:
        """The pose at sim time t, interpolated; None outside the samples held."""
        if not self._t:
            return None
        t = float(t)
        if not (self._t[0] - 1e-9 <= t <= self._t[-1] + 1e-9):
            return None
        i = bisect.bisect_left(self._t, t - 1e-9)
        if i >= len(self._t) or abs(self._t[i] - t) <= 1e-9:
            return self._pose[min(i, len(self._t) - 1)]
        t0, t1 = self._t[i - 1], self._t[i]
        a = (t - t0) / (t1 - t0)
        (p0, q0), (p1, q1) = self._pose[i - 1], self._pose[i]
        pos = tuple(float(u + a * (v - u)) for u, v in zip(p0, p1))
        q0a, q1a = np.asarray(q0), np.asarray(q1)
        if float(q0a @ q1a) < 0.0:
            q1a = -q1a
        q = (1.0 - a) * q0a + a * q1a
        return pos, tuple(float(v) for v in q / np.linalg.norm(q))

    def latest(self) -> float | None:
        return self._t[-1] if self._t else None

    def clear(self) -> None:
        self._t.clear()
        self._pose.clear()


@dataclass(frozen=True, eq=False)
class _Kept:
    t: float                     # sim stamp
    data: np.ndarray
    still_since: float
    wall: float                  # arrival, wall
    clock: float                 # arrival, clock()


class DetectorCore:
    """Everything of the detector node except ROS (spec §11.5, §11.9). See the module docstring."""

    def __init__(self, detector: Detector, vocab: Vocabulary = vocabulary.VOCABULARY,
                 wall: Callable[[], float] = time.monotonic, clock: Callable[[], float] = time.time) -> None:
        self._detector = detector
        self._wall = wall
        self._clock = clock
        self._labels = frozenset(vocab.labels) & vocabulary.LABELS     # observe() needs the loaded vocabulary's
        self._ignore = frozenset(vocab.detect_ignore)
        self._heights = {e.label: float(e.height) for e in vocab.entries}
        self.tracker = Tracker(detector.thresholds, {e.label: tuple(e.size) for e in vocab.entries}, footprint)
        self._lock = threading.RLock()
        self._camera = CameraModel.from_hfov()
        self._loaded = False
        self._loaded_wall: float | None = None
        self._seen = {"odom": False, "cloud": False, "image": False}
        self._last_odom_wall: float | None = None
        self._last_image_wall: float | None = None
        self._last_cloud_wall: float | None = None
        self._in_flight = False
        self._flight_gen = 0
        self._last_start: float | None = None
        self._last_end: float | None = None
        self._latencies: deque[float] = deque(maxlen=LATENCY_WINDOW)
        self._idle = DET_IDLE_PERIOD_S
        self._errors_in_row = 0
        self._error = False
        self._paused_until: float | None = None
        self._inferences = 0
        self._frames_dropped = 0
        self._shapeless = 0
        self._bad_camera = False
        self._bad_logged: set[tuple[int, ...]] = set()
        self._k_warned = False
        self._camera_warned: float | None = None
        self._gen = 0
        self._clear_sensors()

    # --- state ---------------------------------------------------------------------------------------------------

    def _clear_sensors(self) -> None:
        self._gate = StillGate()
        self._poses = PoseBuffer()
        self._image: _Kept | None = None
        self._cloud: _Kept | None = None
        self._scan_intervals: deque[float] = deque(maxlen=16)   # still intervals with a kept scan
        self._runs_since: float | None = None                   # the interval the inference count belongs to
        self._runs = 0
        self._pending_vacancy: tuple[float, np.ndarray, float] | None = None
        self._last_vacancy_wall: float | None = None
        self._latest: float | None = None                       # the latest sensor stamp seen
        self._no_cloud = False

    def mark_loaded(self) -> None:
        """The detector is ready (vocabulary set, model and embeddings loaded). The status leaves loading once the
        first odometry, scan and image have arrived too, or CAMERA_TIMEOUT_S (wall) after this call at the latest:
        a sensor still missing then reads no_odometry, no_camera or no_cloud."""
        with self._lock:
            if not self._loaded:
                self._loaded_wall = self._wall()
            self._loaded = True

    def reset(self, reason: str) -> None:
        """Clear the gate, the pose buffer, the kept frames and the tracks; the ids restart at 1. A job in flight
        finishes, but its result is dropped."""
        with self._lock:
            _LOG.warning("detector core reset: %s (gate, poses, kept frames and tracks cleared; ids restart at 1)",
                         reason)
            self._gen += 1
            self.tracker.reset()
            self._clear_sensors()

    @property
    def idle_period_s(self) -> float:
        """The current idle period (DET_IDLE_PERIOD_S, doubled while slow, at most DET_IDLE_MAX_S)."""
        with self._lock:
            return self._idle

    @property
    def gap_s(self) -> float:
        """The current gap from one inference's end to the next start: max(DET_MIN_GAP_S, p50)."""
        with self._lock:
            return max(DET_MIN_GAP_S, statistics.median(self._latencies)) if self._latencies else DET_MIN_GAP_S

    # --- sensors -------------------------------------------------------------------------------------------------

    def on_odom(self, t: float, pos, quat, linear, angular) -> None:
        """One odometry message: sim stamp t, base pose (odom), twist (linear m/s, angular rad/s)."""
        with self._lock:
            if not self._stamp(t):
                return
            self._seen["odom"] = True
            self._last_odom_wall = self._wall()
            self._gate.observe(float(t), tuple(linear), tuple(angular))
            self._poses.add(t, pos, quat)
            self._vacancy()

    def on_image(self, t: float, image_rgb: np.ndarray) -> None:
        """One camera image (H x W x 3 uint8 RGB) stamped t (sim). The core keeps a reference; it never changes it."""
        with self._lock:
            if not self._stamp(t):
                return
            t = float(t)
            now = self._wall()
            self._seen["image"] = True
            self._last_image_wall = now
            image = np.asarray(image_rgb)
            if self._refuse_size(image):
                return
            if not self._allows(t, now):
                self._frames_dropped += 1
                return
            since = self._gate.still_since
            old = self._image
            if old is not None and (old.still_since != since or old.still_since not in self._scan_intervals):
                self._frames_dropped += 1                        # never paired: its interval ended, or had no scan
                if old.still_since == since:
                    self._no_cloud = True
            self._image = _Kept(t, image, since, now, float(self._clock()))

    def on_cloud(self, t: float, points_lidar: np.ndarray) -> None:
        """One scan of /utlidar/cloud (N x 3 in the LiDAR frame, body and ground removed) stamped t (sim)."""
        with self._lock:
            if not self._stamp(t):
                return
            t = float(t)
            now = self._wall()
            self._seen["cloud"] = True
            self._last_cloud_wall = now
            if not self._allows(t, now):
                return
            p = np.asarray(points_lidar, dtype=np.float64)
            p = p.reshape(-1, p.shape[-1])[:, :3] if p.size else np.zeros((0, 3))
            since = self._gate.still_since
            self._cloud = _Kept(t, p, since, now, float(self._clock()))
            if since not in self._scan_intervals:
                self._scan_intervals.append(since)
            self._no_cloud = False
            if self._pending_vacancy is None and (self._last_vacancy_wall is None
                                                  or now - self._last_vacancy_wall >= 1.0 / VACANCY_MAX_HZ - _EPS):
                self._pending_vacancy = (t, p, since)
            self._vacancy()

    def on_camera_info(self, k: Sequence[float], width: int, height: int) -> None:
        """CameraInfo: its K replaces the intrinsics from the field of view. A K more than 1 % away from
        (W/2) / tan(hfov/2), W/2, H/2 is used but warned about once."""
        with self._lock:
            try:
                cam = CameraModel.from_k(k, width, height)
            except (TypeError, ValueError) as exc:
                if not self._k_warned:
                    self._k_warned = True
                    _LOG.warning("ignoring a CameraInfo the camera model cannot use: %s", exc)
                return
            if cam == self._camera:
                return
            if not cam.agrees(CameraModel.from_hfov(cam.width, cam.height)) and not self._k_warned:
                self._k_warned = True
                _LOG.warning("CameraInfo K %s for %d x %d is more than 1 %% from the field of view's "
                             "(fx = fy = %.3f); using CameraInfo", list(k), cam.width, cam.height,
                             CameraModel.from_hfov(cam.width, cam.height).fx)
            self._camera = cam

    # --- jobs ----------------------------------------------------------------------------------------------------

    def next_job(self) -> Job | None:
        """The next image and scan to run, or None (rate limits, nothing paired yet, or a pair dropped). The job is
        in flight until run() returns."""
        with self._lock:
            now = self._wall()
            if not self._loaded or self._in_flight:
                return None
            if self._paused_until is not None:
                if now < self._paused_until - _EPS:
                    return None
                self._paused_until = None
            if self._last_end is not None and now - self._last_end < self.gap_s - _EPS:
                return None
            img = self._image
            if img is None:
                return None
            if img.still_since != self._gate.still_since or not self._odom_fresh(now):
                self._drop_image()                               # its still interval ended
                return None
            if (self._runs_in(img.still_since) >= DET_SETTLE_FRAMES and self._last_start is not None
                    and now - self._last_start < self._idle - _EPS):
                return None
            scan = self._cloud
            if scan is None or scan.still_since != img.still_since:
                return None                                      # wait for a scan of this interval
            latest = self._poses.latest()
            if latest is None or latest < max(img.t, scan.t) - 1e-9:
                if now - max(img.wall, scan.wall) > PAIR_WAIT_S + _EPS:
                    self._drop_image()                           # odometry did not reach the stamps in time
                return None
            pi, pc = self._poses.at(img.t), self._poses.at(scan.t)
            if pi is None or pc is None or _shift_m(pi, pc) > PAIR_MAX_SHIFT_M + 1e-9 \
                    or _shift_deg(pi, pc) > PAIR_MAX_SHIFT_DEG + 1e-9:
                self._drop_image()
                return None
            self._image = None
            self._in_flight = True
            self._flight_gen = self._gen
            return Job(img.data, img.t, scan.data, scan.t, img.still_since, img.clock)

    def run(self, job: Job) -> None:
        """prepare(view) when the detector has it, detect, observe, track. An exception is logged with its
        traceback and counted; ERROR_LIMIT in a row pause inference for ERROR_BACKOFF_S."""
        with self._lock:
            gen = self._flight_gen if self._in_flight else self._gen
            pi, pc = self._poses.at(job.image_t), self._poses.at(job.cloud_t)
            camera = self._camera
            if gen != self._gen or pi is None or pc is None:
                self._frames_dropped += 1                        # reset meanwhile, or poses no longer held
                self._in_flight = False
                return
            start = self._wall()
            self._last_start = start
        view = View(pi[0], pi[1], camera, float(job.image_t))
        try:
            prepare = getattr(self._detector, "prepare", None)
            if callable(prepare):
                prepare(view)
            dets = self._detector.detect(job.image)
            latency = self._wall() - start
            if not isinstance(dets, list):
                raise ValueError(f"detect() must return a list, not {type(dets).__name__}")
            observations, shapeless, world, lidar = self._observations(dets, job, view, pc)
        except Exception:
            _LOG.exception("detector %s failed on the image of %.3f", getattr(self._detector, "name", "?"),
                           job.image_t)
            with self._lock:
                self._failed()
            return
        with self._lock:
            self._succeeded(latency, job.still_since)
            if gen != self._gen:
                return                                           # sim time went back meanwhile: a stale result
            self._shapeless += shapeless
            sizes = {(round(r.x, 3), round(r.y, 3)): r.size for r in self.tracker.rows(self._clock(), self._idle)}
            in_view = make_in_view(view.camera, view.base_pos, view.base_quat, world, lidar,
                                   [(o.x, o.y, o.size) for o in observations], sizes)
            arrived = float(job.arrived) if job.arrived is not None else float(self._clock())
            self.tracker.update(observations, float(job.image_t), arrived, in_view)

    # --- output --------------------------------------------------------------------------------------------------

    def message(self) -> dict:
        """The /go2/objects message: objects.format_message(rows, clock(), source="detector")."""
        with self._lock:
            now = float(self._clock())
            return objects.format_message(self.tracker.rows(now, self._idle), now, source="detector")

    def health(self) -> dict:
        """The /go2/detector_status JSON: the spec §11.9 keys, plus filled (ids whose depth is the vocabulary's,
        C2), shapeless (sightings without a footprint) and notes (the tracker's, C6). It also warns once a
        minute while there is no camera."""
        with self._lock:
            now = self._wall()
            reason = self._reason(now)
            if reason == "no_camera" and (self._camera_warned is None or now - self._camera_warned >= CAMERA_WARN_S):
                self._camera_warned = now
                if self._last_image_wall is None:
                    _LOG.warning("no camera image since the detector loaded %.0f s ago (is front_camera_bridge "
                                 "running?): tracks kept, vacancy runs on the scans", now - self._loaded_wall)
                else:
                    _LOG.warning("no camera image for %.0f s: tracks kept, vacancy runs on the scans",
                                 now - self._last_image_wall)
            stats = self.tracker.stats()
            lat = sorted(self._latencies)
            p50 = statistics.median(lat) if lat else None
            p90 = lat[max(0, math.ceil(0.9 * len(lat)) - 1)] if lat else None
            return {
                "ok": reason == "ok",
                "reason": reason,
                "detector": str(getattr(self._detector, "name", "")),
                "frame_age_s": _age(now, self._last_image_wall),
                "cloud_age_s": _age(now, self._last_cloud_wall),
                "still": self._still(now),
                "latency_ms_p50": None if p50 is None else round(1000.0 * p50, 1),
                "latency_ms_p90": None if p90 is None else round(1000.0 * p90, 1),
                "inferences": self._inferences,
                "still_inferences": self._runs_in(self._gate.still_since) if self._still(now) else 0,
                "frames_dropped": self._frames_dropped,
                "tracks_tentative": stats["tracks_tentative"],
                "tracks_confirmed": stats["tracks_confirmed"],
                "ids_issued": dict(stats["ids_issued"]),
                "ids_retired": stats["ids_retired"],
                "filled": list(stats["filled"]),
                "shapeless": self._shapeless,
                "notes": list(stats["notes"]),
            }

    # --- helpers (called with the lock held) ---------------------------------------------------------------------

    def _stamp(self, t: float) -> bool:
        """Note a sensor stamp; a stamp more than TIME_BACK_S before the latest resets the core. False for a
        stamp that is not a finite number (the message is ignored)."""
        try:
            t = float(t)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(t):
            return False
        if self._latest is not None and t < self._latest - TIME_BACK_S - 1e-9:
            self.reset(f"sim time went backwards ({self._latest:.3f} -> {t:.3f} s)")
        self._latest = t if self._latest is None else max(self._latest, t)
        return True

    def _odom_fresh(self, now: float) -> bool:
        return self._last_odom_wall is not None and now - self._last_odom_wall <= ODOM_TIMEOUT_S + _EPS

    def _allows(self, t: float, now: float) -> bool:
        return self._odom_fresh(now) and self._gate.allows(t)

    def _still(self, now: float) -> bool:
        latest = self._poses.latest()
        return bool(self._odom_fresh(now) and latest is not None and self._gate.allows(latest))

    def _runs_in(self, since: float | None) -> int:
        return self._runs if since is not None and since == self._runs_since else 0

    def _drop_image(self) -> None:
        self._image = None
        self._frames_dropped += 1

    def _refuse_size(self, image: np.ndarray) -> bool:
        """An image whose size is not the constants' and the CameraInfo's is refused (bad_camera, logged once
        per size)."""
        shape = tuple(int(v) for v in image.shape)
        h, w = (shape[0], shape[1]) if len(shape) >= 2 else (0, 0)
        ok = (len(shape) == 3 and shape[2] == 3 and (w, h) == (CAMERA_W, CAMERA_H)
              and (w, h) == (self._camera.width, self._camera.height))
        if ok:
            self._bad_camera = False
            return False
        self._frames_dropped += 1
        self._bad_camera = True
        if shape not in self._bad_logged:
            self._bad_logged.add(shape)
            size = f"{w} x {h}" + ("" if len(shape) == 3 and shape[2] == 3 else f" (shape {shape})")
            _LOG.warning("refusing a %s image: the camera is %d x %d, CameraInfo %d x %d", size, CAMERA_W, CAMERA_H,
                         self._camera.width, self._camera.height)
        return True

    def _reason(self, now: float) -> str:
        if not self._loaded:
            return "loading"
        if not all(self._seen.values()) and now - self._loaded_wall <= CAMERA_TIMEOUT_S + _EPS:
            return "loading"                                 # the first odometry, scan or image is on its way
        if not self._odom_fresh(now):
            return "no_odometry"                             # stale, or never came
        if self._last_image_wall is None or now - self._last_image_wall > CAMERA_TIMEOUT_S + _EPS:
            return "no_camera"
        if self._bad_camera:
            return "bad_camera"
        if self._no_cloud or not self._seen["cloud"]:
            return "no_cloud"
        if self._error or (self._paused_until is not None and now < self._paused_until - _EPS):
            return "detector_error"
        if self._latencies and statistics.median(self._latencies) > SLOW_LATENCY_S:
            return "slow"
        return "ok"

    def _failed(self) -> None:
        now = self._wall()
        self._in_flight = False
        self._last_end = now
        self._error = True
        self._errors_in_row += 1
        if self._errors_in_row >= ERROR_LIMIT:
            self._errors_in_row = 0
            self._paused_until = now + ERROR_BACKOFF_S
            _LOG.error("%d detector errors in a row: no inference for %.0f s", ERROR_LIMIT, ERROR_BACKOFF_S)

    def _succeeded(self, latency: float, since: float) -> None:
        self._in_flight = False
        self._last_end = self._wall()
        self._error = False
        self._errors_in_row = 0
        self._inferences += 1
        if since != self._runs_since:
            self._runs_since, self._runs = since, 0
        self._runs += 1
        self._latencies.append(max(0.0, float(latency)))
        if statistics.median(self._latencies) > SLOW_LATENCY_S:
            self._idle = min(2.0 * self._idle, DET_IDLE_MAX_S)
        else:
            self._idle = DET_IDLE_PERIOD_S

    def _height(self, label: str) -> float:
        h = self._heights.get(label) or vocabulary.height_of(label)
        return float(h) if h else VACANCY_MIN_HEIGHT

    def _vacancy(self) -> None:
        """Run the vacancy test on the pending still scan once odometry has reached its stamp; drop it when the
        Go2 moved since, or when its stamp has left the pose buffer (an odometry gap), so the next scan can go."""
        pending = self._pending_vacancy
        if pending is None:
            return
        t, points, since = pending
        if since != self._gate.still_since:
            self._pending_vacancy = None                         # the Go2 moved since: not a still scan
            return
        pose = self._poses.at(t)
        if pose is None:
            latest = self._poses.latest()
            if latest is not None and latest >= t - 1e-9:
                self._pending_vacancy = None                     # its poses are gone: it can never be placed
            return                                               # otherwise wait for odometry
        self._pending_vacancy = None
        self._last_vacancy_wall = self._wall()
        targets = self.tracker.vacancy_targets()
        if not targets:
            return
        world = lidar_points_world(points, *pose)
        lidar = lidar_points_world(np.zeros((1, 3)), *pose)[0]
        for key, x, y, size, label in targets:
            n_in, n_through = vacancy_counts(world, lidar, (x, y), size, self._height(label))
            self.tracker.mark_vacant(key, is_vacant(n_in, n_through), t)

    # --- geometry (no lock: job data only) -----------------------------------------------------------------------

    def _usable(self, d: object, shape: tuple[int, int, int], sighting: float) -> bool:
        if not isinstance(d, Detection) or d.label not in self._labels or d.label in self._ignore:
            return False
        try:
            check_detections([d], shape, self._labels)
        except ValueError:
            return False
        return float(d.confidence) >= sighting - 1e-12

    def _observations(self, dets: list, job: Job, view: View, cloud_pose) -> tuple[list[Observation], int,
                                                                                    np.ndarray, np.ndarray]:
        world = lidar_points_world(job.cloud, *cloud_pose)
        lidar = lidar_points_world(np.zeros((1, 3)), *cloud_pose)[0]
        shape = tuple(job.image.shape)
        sighting = float(self._detector.thresholds.sighting)
        out: list[Observation] = []
        shapeless = 0
        for d in dets:
            if not self._usable(d, shape, sighting):
                continue
            s = observe(d.box, d.mask, world, view.base_pos, view.base_quat, view.camera, d.label)
            fp = s.footprint
            if fp is None:
                shapeless += 1                                   # nothing to associate it with
                continue
            out.append(Observation(d.label, float(d.confidence), tuple(float(v) for v in d.box), fp.x, fp.y,
                                   fp.size, fp.filled, s.points_xy, s.supported, s.fallback, s.cam_xy))
        return out, shapeless, world, lidar


def make_in_view(camera: CameraModel, base_pos: Sequence[float], base_quat: Sequence[float],
                 points_world: np.ndarray, lidar_pos: Sequence[float],
                 seen: Sequence[tuple[float, float, tuple[float, float]]] = (),
                 sizes: Mapping[tuple[float, float], tuple[float, float]] | None = None
                 ) -> Callable[[float, float], bool]:
    """The tracker's in_view(x, y) for one image (tracker.Tracker.update; design-detector §8.3, §8.6):
    - False beyond INVIEW_RANGE (6 m) of the camera (at the image stamp);
    - True inside the footprint (+ SEEN_MARGIN) of one of this image's sightings (seen: (x, y, size));
    - False when the point at the camera's height projects behind PROJ_NEAR or outside the image's width;
    - False when the LiDAR returns (points_world, from lidar_pos) in its bearing window stop more than
      OCCLUDED_SHORT_M short of its near side for more than OCCLUDED_SHARE of them. The near side is r = half the
      diagonal of its published size (sizes, keyed by (x, y) rounded to mm) or of objects.DEFAULT_SIZE, so a big
      object's own front face never hides it. No returns in the window: not occluded."""
    cam_pos, _ = camera_pose(base_pos, base_quat)
    cx, cy, cz = (float(v) for v in cam_pos)
    lid = np.asarray(lidar_pos, dtype=np.float64).reshape(-1)[:2]
    p = cut(points_world)
    d = p[:, :2] - lid
    ranges = np.hypot(d[:, 0], d[:, 1])
    bearings = np.arctan2(d[:, 1], d[:, 0])
    boxes = [(float(x), float(y), float(s[0]) / 2 + SEEN_MARGIN, float(s[1]) / 2 + SEEN_MARGIN) for x, y, s in seen]
    sizes = dict(sizes or {})

    def in_view(x: float, y: float) -> bool:
        x, y = float(x), float(y)
        if not (math.isfinite(x) and math.isfinite(y)) or math.hypot(x - cx, y - cy) > INVIEW_RANGE + 1e-9:
            return False
        if any(abs(x - bx) <= hx and abs(y - by) <= hy for bx, by, hx, hy in boxes):
            return True
        q = world_to_camera(np.array([[x, y, cz]]), base_pos, base_quat)[0]
        if q[0] <= PROJ_NEAR or not 0.0 <= camera.cx - camera.fx * q[1] / q[0] < camera.width:
            return False
        r = math.hypot(*sizes.get((round(x, 3), round(y, 3)), objects.DEFAULT_SIZE)) / 2
        dist = math.hypot(x - lid[0], y - lid[1])
        if dist <= r or len(ranges) == 0:
            return True
        half = max(math.atan2(r, dist), LIDAR_STEP)
        window = np.abs((bearings - math.atan2(y - lid[1], x - lid[0]) + math.pi) % (2 * math.pi) - math.pi) <= half
        n = int(window.sum())
        if n == 0:
            return True
        short = int((ranges[window] < dist - r - OCCLUDED_SHORT_M).sum())
        return short <= OCCLUDED_SHARE * n

    return in_view


def _shift_m(a, b) -> float:
    return math.dist(a[0], b[0])


def _shift_deg(a, b) -> float:
    dot = abs(float(np.dot(a[1], b[1])))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def _age(now: float, then: float | None) -> float | None:
    return None if then is None else round(max(0.0, now - then), 3)
