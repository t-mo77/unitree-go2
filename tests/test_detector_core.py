"""detector/core.py: still gating, pairing, rate limits, failures and health (Task 2.7; spec §11.5, §11.9;
design-detector.md §6, §10 and §16 "Core"), with the ScriptedDetector.

A Rig feeds a DetectorCore the way the node does (detector/node.py, Task 2.10): odometry at 50 Hz, scans at 10 Hz
and images at 2 Hz in sim time. The scans come from the ray caster of tests/test_detector_geometry.py, put into
the LiDAR frame as /utlidar/cloud carries them (no floor returns: the bridge removes the ground). The fake steady
wall clock and the fake time.time() advance with sim time (RTF 1), unless a test's detector advances them itself
(its latency). With worker=True every step also runs the node's worker loop once (next_job, then run).
"""

import json
import logging
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from detector import core as core_module
from detector.camera import CAMERA_H, CAMERA_W, CameraModel
from detector.core import (
    CAMERA_TIMEOUT_S, DET_IDLE_MAX_S, DET_IDLE_PERIOD_S, DET_MIN_GAP_S, DET_SETTLE_FRAMES, ERROR_BACKOFF_S,
    ERROR_LIMIT, PAIR_MAX_SHIFT_DEG, PAIR_MAX_SHIFT_M, PAIR_WAIT_S, POSE_BUFFER_S, SLOW_LATENCY_S, TIME_BACK_S,
    VACANCY_MAX_HZ, DetectorCore, Job, PoseBuffer, View, make_in_view,
)
from detector.protocol import YOLOE_THRESHOLDS, Detection
from detector.scripted import ScriptedDetector
from detector.tracker import MAX_ID, MIN_HIT_GAP_S, Observation
from gazebo import bridge_core
from gazebo.bridge_core import LIDAR_IN_BODY, quat_to_matrix
from test_detector_geometry import BASE, Q0, bbox, scan, solid, yaw_q
from voice_ltl import objects, vocabulary

ROOT = Path(__file__).resolve().parent.parent
ZERO = (0.0, 0.0, 0.0)
ODOM_DT, SCAN_DT, IMAGE_DT = 0.02, 0.1, 0.5
IMAGE = np.zeros((CAMERA_H, CAMERA_W, 3), dtype=np.uint8)
CHAIR = solid(2.5, 0.0, 0.45, 0.45, height=0.9)
BOX = solid(2.5, 0.0, 0.5, 0.5, height=0.5)
WALL = solid(5.05, 0.0, 0.1, 8.0, height=2.0)
SCENE = (CHAIR, WALL)
CHAIR_BOX = bbox(CHAIR)
CHAIR_SEEN = Detection("chair", 0.8, CHAIR_BOX)
SPEC_KEYS = {"ok", "reason", "detector", "frame_age_s", "cloud_age_s", "still", "latency_ms_p50", "latency_ms_p90",
             "inferences", "still_inferences", "frames_dropped", "tracks_tentative", "tracks_confirmed",
             "ids_issued", "ids_retired"}


def to_lidar(points_world, base=BASE, q=Q0):
    """World points -> the LiDAR frame (the inverse of bridge_core.lidar_points_world)."""
    return (np.asarray(points_world, dtype=np.float64) - np.asarray(base)) @ quat_to_matrix(q) \
        - np.asarray(LIDAR_IN_BODY)


def on_grid(t, dt):
    return abs(t / dt - round(t / dt)) < 1e-6


class Clocks:
    def __init__(self):
        self.w = 500.0              # the steady wall clock (time.monotonic)
        self.c = 1.79e9             # time.time()

    def wall(self):
        return self.w

    def clock(self):
        return self.c

    def advance(self, dt):
        self.w += dt
        self.c += dt


class Rig:
    """One DetectorCore and the sensors around it. detector: a Detector, or a function of the rig that makes one
    (so it can use the rig's clocks); by default a ScriptedDetector that sees the chair in every image."""

    def __init__(self, script=None, detector=None, scene=SCENE, vocab=vocabulary.VOCABULARY, loaded=True):
        self.clocks = Clocks()
        if detector is None:
            detector = ScriptedDetector([[CHAIR_SEEN]] if script is None else script)
        elif callable(detector) and not hasattr(detector, "detect"):
            detector = detector(self)
        self.detector = detector
        self.core = DetectorCore(detector, vocab, wall=self.clocks.wall, clock=self.clocks.clock)
        if loaded:
            self.core.mark_loaded()
        self.scene = tuple(scene)
        self.base, self.q = BASE, Q0
        self.image = IMAGE
        self.t = None
        self.pending = None
        self.starts = []            # the wall clock at each detect() call (record_starts)
        self._clouds = {}

    def __getattr__(self, name):
        return getattr(self.core, name)

    # the worker loop of the node
    def next_job(self):
        job = self.core.next_job()
        if job is not None:
            self.pending = job
        return job

    def run(self, job):
        if job is self.pending:
            self.pending = None
        self.core.run(job)

    def work(self):
        if self.pending is not None:
            self.run(self.pending)
        job = self.next_job()
        if job is not None:
            self.run(job)

    def record_starts(self):
        detect = self.detector.detect

        def recording(image):
            self.starts.append(self.clocks.w)
            return detect(image)
        self.detector.detect = recording
        return self.starts

    # sensors
    def cloud(self):
        key = (self.base, self.q, self.scene)
        if key not in self._clouds:
            self._clouds[key] = to_lidar(scan(list(self.scene), self.base, self.q), self.base, self.q)
        return self._clouds[key]

    def feed(self, t0, t1, *, velocity=0.0, trot=False, odom=True, clouds=True, images=True, worker=True):
        """Steps of ODOM_DT from t0 to t1 (inclusive; from t0 + ODOM_DT when t0 is the last step fed): odometry,
        then a scan every 0.1 s and an image every 0.5 s (sim), then the worker. velocity (m/s along x) moves
        the base and shows in the twist; trot shows a 0.3 m/s twist without moving the base."""
        if self.t is not None and abs(t0 - self.t) < 1e-9:
            t0 += ODOM_DT
        for k in range(int(round((t1 - t0) / ODOM_DT)) + 1):
            t = round(t0 + k * ODOM_DT, 6)
            if self.t is not None:
                self.clocks.advance(ODOM_DT)
            if velocity:
                self.base = (round(self.base[0] + velocity * ODOM_DT, 9), self.base[1], self.base[2])
            if odom:
                self.core.on_odom(t, self.base, self.q, (velocity or (0.3 if trot else 0.0), 0.0, 0.0), ZERO)
            if clouds and on_grid(t, SCAN_DT):
                self.core.on_cloud(t, self.cloud())
            if images and on_grid(t, IMAGE_DT):
                self.core.on_image(t, self.image)
            if worker:
                self.work()
            self.t = t

    def ids(self):
        return [r["id"] for r in self.core.message()["objects"]]


def confirmed_chair_core():
    """chair_1 confirmed by two jobs (images at sim 119.5 and 120.5), the Go2 still since 119.0."""
    rig = Rig()
    rig.feed(119.0, 121.0)
    assert rig.ids() == ["chair_1"]
    return rig


def feed_still(rig, t0):
    """Half a second of a still Go2 from t0, without the worker: the gate opens, an image and a scan are kept."""
    rig.feed(t0, t0 + 0.5, worker=False)


def confirm_chair(rig, t0):
    """Two more seconds with the worker (and the job a test took from next_job, if any)."""
    rig.feed(t0, t0 + 2.0)


def slow_detector(latencies, seen=(CHAIR_SEEN,)):
    """A ScriptedDetector whose call k takes latencies[k] (the last repeats) on the rig's fake wall clock."""
    def make(rig):
        def script(k, image):
            rig.clocks.advance(latencies[min(k, len(latencies) - 1)])
            return list(seen)
        return ScriptedDetector(script)
    return make


# --- constants and the pose buffer -----------------------------------------------------------------------------

def test_constants_are_the_plan_values():
    assert (POSE_BUFFER_S, PAIR_WAIT_S, PAIR_MAX_SHIFT_M, PAIR_MAX_SHIFT_DEG) == (2.0, 0.1, 0.02, 1.0)
    assert (DET_MIN_GAP_S, DET_SETTLE_FRAMES, DET_IDLE_PERIOD_S, DET_IDLE_MAX_S, SLOW_LATENCY_S) == \
        (1.0, 3, 3.0, 12.0, 1.5)
    assert (CAMERA_TIMEOUT_S, ERROR_LIMIT, ERROR_BACKOFF_S, VACANCY_MAX_HZ, TIME_BACK_S) == (10.0, 3, 30.0, 2.0, 1.0)
    # the gate is the bridge's own (spec §11.3): each rate <= 0.1 rad/s, speed <= 0.05 m/s, for >= 0.3 s
    assert (bridge_core.SIZE_STILL_RATE, bridge_core.SIZE_STILL_SPEED, bridge_core.SIZE_STILL_S) == (0.1, 0.05, 0.3)
    assert core_module.StillGate is bridge_core.StillGate


def test_pose_buffer_interpolates_between_samples():
    buf = PoseBuffer()
    assert buf.latest() is None and buf.at(1.0) is None
    buf.add(1.0, (0.0, 0.0, 0.3), (1.0, 0.0, 0.0, 0.0))
    buf.add(1.1, (0.1, 0.2, 0.3), yaw_q(10.0))
    pos, quat = buf.at(1.05)
    assert pos == pytest.approx((0.05, 0.1, 0.3))
    assert 2 * math.degrees(math.atan2(quat[3], quat[0])) == pytest.approx(5.0, abs=1e-6)
    assert buf.at(1.0) == ((0.0, 0.0, 0.3), (1.0, 0.0, 0.0, 0.0))
    assert buf.at(0.99) is None and buf.at(1.11) is None          # outside the samples: not known
    assert buf.latest() == 1.1
    buf.add(1.08, (0.08, 0.0, 0.3), Q0)                             # a late sample goes in its place
    assert buf.at(1.08)[0] == pytest.approx((0.08, 0.0, 0.3)) and buf.latest() == 1.1
    buf.clear()
    assert buf.latest() is None and buf.at(1.05) is None


def test_pose_buffer_keeps_the_last_two_seconds():
    buf = PoseBuffer()
    for k in range(201):                                            # 4 s at 50 Hz
        buf.add(10.0 + k * 0.02, (k * 0.01, 0.0, 0.3), Q0)
    assert buf.latest() == pytest.approx(14.0)
    assert buf.at(11.9) is None                                     # older than POSE_BUFFER_S
    assert buf.at(12.1)[0][0] == pytest.approx(1.05)
    buf.add(13.0, (float("nan"), 0.0, 0.3), Q0)                     # a bad sample is ignored
    assert buf.at(13.0)[0][0] == pytest.approx(1.5)


# --- start-up and the status ----------------------------------------------------------------------------------

def test_start_up_gives_empty_lists_with_loading_then_ok_after_the_first_sensors():
    rig = Rig(loaded=False)
    assert rig.message() == {"frame": "odom", "stamp": rig.clocks.c, "source": "detector", "objects": []}
    assert (rig.health()["ok"], rig.health()["reason"]) == (False, "loading")
    rig.feed(10.0, 11.0)                                  # sensors while the model loads: no job
    assert rig.detector.calls == 0 and rig.health()["reason"] == "loading"
    assert rig.message()["objects"] == []

    rig = Rig()                                           # loaded, then the first odometry, scan and image
    core = rig.core
    assert core.health()["reason"] == "loading"
    core.on_odom(10.0, BASE, Q0, ZERO, ZERO)
    assert core.health()["reason"] == "loading"
    core.on_cloud(10.0, rig.cloud())
    assert core.health()["reason"] == "loading"
    core.on_image(10.0, IMAGE)
    assert (core.health()["ok"], core.health()["reason"]) == (True, "ok")


def test_a_camera_missing_from_the_start_says_no_camera_after_10_s(caplog):
    rig = Rig()                                           # loaded; odometry and scans, the camera never comes
    with caplog.at_level(logging.WARNING, logger="detector.core"):
        rig.feed(10.0, 10.0 + CAMERA_TIMEOUT_S - 0.1, images=False)
        assert rig.health()["reason"] == "loading"       # the first image may still be on its way
        rig.feed(rig.t, 10.0 + CAMERA_TIMEOUT_S + 0.5, images=False)
        h = rig.health()
        assert (h["ok"], h["reason"], h["frame_age_s"]) == (False, "no_camera", None)
        rig.health()
    warnings = [r.getMessage() for r in caplog.records if "no camera" in r.getMessage()]
    assert len(warnings) == 1 and "front_camera_bridge" in warnings[0]                 # once a minute


@pytest.mark.parametrize("missing, reason", [
    pytest.param({"odom": False}, "no_odometry", id="odometry"),
    pytest.param({"clouds": False, "trot": True}, "no_cloud", id="lidar"),   # stepping: no image left unpaired
])
def test_a_sensor_missing_from_the_start_ends_loading_after_10_s(missing, reason):
    rig = Rig()
    rig.feed(10.0, 10.0 + CAMERA_TIMEOUT_S - 0.1, **missing)
    assert rig.health()["reason"] == "loading"
    rig.feed(rig.t, 10.0 + CAMERA_TIMEOUT_S + 0.5, **missing)
    h = rig.health()
    assert (h["ok"], h["reason"], h["still"]) == (False, reason, False)
    assert rig.detector.calls == 0


def test_health_has_every_spec_key_plus_filled_shapeless_and_notes():
    rig = confirmed_chair_core()
    h = rig.health()
    assert set(h) == SPEC_KEYS | {"filled", "shapeless", "notes"}
    assert (h["ok"], h["reason"], h["detector"], h["still"]) == (True, "ok", "scripted", True)
    assert (h["inferences"], h["still_inferences"], h["tracks_tentative"], h["tracks_confirmed"]) == (2, 2, 0, 1)
    assert (h["ids_issued"], h["ids_retired"], h["filled"], h["shapeless"], h["notes"]) == \
        ({"chair": 1}, 0, ["chair_1"], 0, [])                 # seen face-on: the depth is the vocabulary's
    assert (h["latency_ms_p50"], h["latency_ms_p90"]) == (0.0, 0.0)
    assert h["frame_age_s"] == pytest.approx(0.0, abs=1e-6) and h["cloud_age_s"] == pytest.approx(0.0, abs=1e-6)
    assert h["frames_dropped"] == 1                           # the image of 119.0 (the gate closed)
    json.dumps(h)                                             # it goes into /go2/detector_status


def test_message_round_trips_through_objects_parse():
    rig = confirmed_chair_core()
    payload = json.loads(json.dumps(rig.message()))
    assert objects.parse_source(payload) == "detector" and payload["stamp"] == rig.clocks.c
    [obj] = objects.parse(payload)
    assert (obj.id, obj.label, obj.visible, obj.measured) == ("chair_1", "chair", True, True)
    assert (obj.x, obj.y) == pytest.approx((2.5, 0.0), abs=0.05)
    assert obj.confidence == 0.8 and obj.seen == pytest.approx(rig.clocks.c - 0.5, abs=1e-3)   # 120.5's image


# --- when to detect (spec §11.5) -----------------------------------------------------------------------------

def test_no_job_before_the_gate_opens():
    rig = Rig()
    rig.feed(10.0, 10.28, worker=False)                   # still for 0.28 s: scans and the image of 10.0 dropped
    rig.core.on_image(10.28, IMAGE)
    assert rig.core.next_job() is None
    rig = Rig()
    rig.feed(10.0, 13.0, velocity=0.3)                    # walking: images arrive and are dropped
    assert rig.detector.calls == 0 and rig.health()["still"] is False
    assert rig.health()["frames_dropped"] == 7


def test_an_image_stamped_before_the_gate_opened_is_refused_although_still_now():
    rig = Rig()
    rig.feed(10.0, 10.6, images=False, worker=False)      # still since 10.0: the gate is open now
    rig.core.on_image(10.2, IMAGE)                        # taken 0.2 s into the stop, arriving late
    assert rig.core.next_job() is None and rig.health()["frames_dropped"] == 1
    rig.core.on_image(10.6, IMAGE)
    job = rig.core.next_job()
    assert job is not None and (job.image_t, job.still_since) == (10.6, 10.0)


def test_image_and_scan_from_different_still_intervals_never_pair():
    rig = Rig()
    rig.feed(10.0, 10.4, images=False, worker=False)      # scans of the first stop (10.3, 10.4)
    rig.feed(10.42, 10.6, trot=True, images=False, clouds=False, worker=False)   # moving in place: same pose
    rig.feed(10.62, 11.0, clouds=False, worker=False)     # a second stop from 10.62: its image of 11.0, no scan
    assert rig.core.next_job() is None
    rig.core.on_cloud(11.0, rig.cloud())
    job = rig.core.next_job()
    assert job is not None and (job.image_t, job.cloud_t) == (11.0, 11.0)
    assert job.still_since == pytest.approx(10.62)


def test_a_frame_kept_while_still_is_not_run_once_the_go2_walks():   # pre-final review F13 (D5)
    rig = Rig()
    feed_still(rig, t0=10.0)                              # still: an image and a scan are kept, no worker
    rig.feed(rig.t, rig.t + 0.3, trot=True, images=False, clouds=False, worker=False)   # walking again
    dropped = rig.health()["frames_dropped"]
    assert rig.core.next_job() is None                    # no inference while the Go2 walks
    assert rig.health()["frames_dropped"] == dropped + 1  # the kept image is dropped, not kept for later


def test_a_job_waits_for_odometry_to_reach_both_stamps():
    rig = Rig()
    core = rig.core
    rig.feed(10.0, 10.48, worker=False)
    core.on_cloud(10.5, rig.cloud())                      # both ahead of the odometry (10.48)
    core.on_image(10.5, IMAGE)
    assert core.next_job() is None
    rig.clocks.advance(PAIR_WAIT_S / 2)
    assert core.next_job() is None
    core.on_odom(10.5, BASE, Q0, ZERO, ZERO)
    job = core.next_job()
    assert job is not None and (job.image_t, job.cloud_t) == (10.5, 10.5)

    rig = Rig()                                           # odometry at the image stamp, short of the scan's
    core = rig.core
    rig.feed(10.0, 10.5, clouds=False, images=False, worker=False)
    core.on_image(10.5, IMAGE)
    core.on_cloud(10.52, rig.cloud())
    dropped = core.health()["frames_dropped"]
    assert core.next_job() is None
    rig.clocks.advance(PAIR_WAIT_S / 2)
    assert core.next_job() is None and core.health()["frames_dropped"] == dropped    # it waits, not drops
    core.on_odom(10.52, BASE, Q0, ZERO, ZERO)
    job = core.next_job()
    assert job is not None and (job.image_t, job.cloud_t) == (10.5, 10.52)

    rig = Rig()                                           # odometry later than PAIR_WAIT_S: the image is dropped
    core = rig.core
    rig.feed(10.0, 10.48, worker=False)
    core.on_cloud(10.5, rig.cloud())
    core.on_image(10.5, IMAGE)
    rig.clocks.advance(PAIR_WAIT_S + 0.05)
    assert core.next_job() is None and core.health()["frames_dropped"] == 2     # with the image of 10.0
    core.on_odom(10.5, BASE, Q0, ZERO, ZERO)
    assert core.next_job() is None                        # it is gone


@pytest.mark.parametrize("shift, turn, kept", [
    pytest.param(0.03, 0.0, False, id="0.03-m"),
    pytest.param(0.0, 1.5, False, id="1.5-deg"),
    pytest.param(0.01, 0.5, True, id="0.01-m-0.5-deg"),
])
def test_poses_apart_at_the_two_stamps_drop_the_job(shift, turn, kept):
    rig = Rig()
    core = rig.core
    rig.feed(10.0, 10.5, worker=False)                    # the image of 10.5 kept
    drifted = (BASE[0] + shift, BASE[1], BASE[2])
    core.on_odom(10.52, drifted, yaw_q(turn), ZERO, ZERO)  # still by its twist, but the pose moved
    core.on_odom(10.6, drifted, yaw_q(turn), ZERO, ZERO)
    core.on_cloud(10.6, rig.cloud())
    job = core.next_job()
    assert (job is not None) is kept
    assert core.health()["frames_dropped"] == (1 if kept else 2)   # the image of 10.0, and the dropped job


def test_one_job_in_flight():
    rig = Rig()
    rig.feed(10.0, 10.5, worker=False)
    job = rig.core.next_job()
    assert job is not None
    rig.feed(10.52, 12.0, worker=False)                   # newer images, the gap long past
    assert rig.core.next_job() is None
    rig.core.run(job)
    assert rig.core.next_job() is None                    # the gap starts when the inference ends
    rig.feed(12.02, 13.0, worker=False)
    assert rig.core.next_job() is not None


def test_the_gap_is_max_of_1_s_and_the_p50_latency():
    rig = Rig()
    starts = rig.record_starts()
    rig.feed(10.0, 13.0)
    assert len(starts) >= 2 and 1.0 - 1e-6 <= starts[1] - starts[0] <= 1.0 + IMAGE_DT + 1e-6
    rig = Rig(detector=slow_detector([1.25]))             # still below SLOW_LATENCY_S
    starts = rig.record_starts()
    rig.feed(10.0, 16.0)
    ends = [s + 1.25 for s in starts]
    assert len(starts) >= DET_SETTLE_FRAMES
    for end, start in list(zip(ends, starts[1:]))[:DET_SETTLE_FRAMES - 1]:
        assert 1.25 - 1e-6 <= start - end <= 1.25 + IMAGE_DT + 1e-6
    assert rig.health()["reason"] == "ok"


def test_idle_period_after_three_frames_and_motion_resets_it():
    rig = Rig()
    starts = rig.record_starts()
    rig.feed(10.0, 20.0)
    gaps = np.diff(starts)
    assert len(starts) == 5
    assert all(DET_MIN_GAP_S - 1e-6 <= g <= DET_MIN_GAP_S + IMAGE_DT + 1e-6 for g in gaps[:DET_SETTLE_FRAMES - 1])
    assert all(DET_IDLE_PERIOD_S - 1e-6 <= g <= DET_IDLE_PERIOD_S + IMAGE_DT + 1e-6
               for g in gaps[DET_SETTLE_FRAMES - 1:])
    assert rig.health()["still_inferences"] == 5
    rig.feed(20.02, 20.3, velocity=0.3)                   # a step
    assert rig.health()["still_inferences"] == 0
    n = len(starts)
    rig.feed(20.32, 21.0)                                 # still from 20.32: the first image kept is 21.0's
    assert len(starts) == n + 1 and starts[n] == pytest.approx(rig.clocks.w)   # 2.5 s after the last: no idle wait
    rig.feed(21.02, 24.0)
    again = np.diff(starts[n:])
    assert len(again) >= 2 and all(g <= DET_MIN_GAP_S + IMAGE_DT + 1e-6 for g in again[:DET_SETTLE_FRAMES - 1])


def test_a_result_applied_after_the_go2_moved_uses_the_stamp_poses(monkeypatch):
    rig = Rig()
    rig.feed(10.0, 10.5)                                  # one hit
    rig.feed(10.52, 11.5, worker=False)
    job = rig.core.next_job()
    assert job is not None and job.image_t == 11.5
    rig.feed(11.52, 12.5, velocity=0.5, worker=False)     # the Go2 walks 0.5 m before the result lands
    seen = []
    update = rig.core.tracker.update
    monkeypatch.setattr(rig.core.tracker, "update", lambda obs, *a, **k: (seen.extend(obs), update(obs, *a, **k)))
    rig.core.run(job)
    [o] = seen
    assert o.cam_xy == pytest.approx((0.32715, -0.00003))  # the camera where the image was taken
    assert (o.x, o.y) == pytest.approx((2.5, 0.0), abs=0.05)
    [row] = rig.message()["objects"]
    assert row["id"] == "chair_1"
    assert (row["x"], row["y"]) == pytest.approx((2.5, 0.0), abs=0.05)


def test_a_slow_p50_doubles_the_idle_period_until_it_recovers():
    rig = Rig(detector=slow_detector([2.0, 2.0, 2.0, 0.0]))
    starts = rig.record_starts()
    assert rig.core.idle_period_s == DET_IDLE_PERIOD_S
    rig.feed(10.0, 14.0)
    assert len(starts) == 2
    assert rig.health()["reason"] == "slow" and not rig.health()["ok"]
    assert rig.core.idle_period_s == 4 * DET_IDLE_PERIOD_S      # doubled after each slow inference
    assert starts[1] - (starts[0] + 2.0) >= 2.0 - 1e-6        # the gap is the p50 (2.0 s)
    rig.feed(14.02, 35.0)                                      # calls at 14.5 (slow) and 24.5 (fast)
    assert len(starts) == 4 and rig.core.idle_period_s == DET_IDLE_MAX_S   # at most 12 s
    assert starts[3] - starts[2] >= DET_IDLE_MAX_S - 1e-6
    rows = rig.message()["objects"]
    assert rows and rows[0]["visible"]                         # 10.5 s after the last hit: held 2 x 12 s
    rig.feed(35.02, 120.0)                                     # fast again: the p50 of the last 10 recovers
    assert rig.health()["reason"] == "ok" and rig.core.idle_period_s == DET_IDLE_PERIOD_S


# --- what becomes a track -------------------------------------------------------------------------------------

def test_labels_outside_the_vocabulary_and_ignored_prompts_never_become_tracks():
    dets = [Detection("unicorn", 0.9, CHAIR_BOX), Detection("person", 0.9, CHAIR_BOX),
            Detection("dog", 0.9, (10.0, 10.0, 60.0, 60.0)), CHAIR_SEEN,
            Detection("box", 0.2, (400.0, 200.0, 460.0, 260.0))]   # below the sighting threshold
    rig = Rig(script=[dets])
    rig.feed(10.0, 12.0)
    h = rig.health()
    assert rig.ids() == ["chair_1"] and h["tracks_tentative"] == 0 and h["reason"] == "ok"

    stool = Detection("stool", 0.8, CHAIR_BOX)            # the core's vocabulary decides (box and chair must stay)
    rig = Rig(script=[[stool]])
    rig.feed(10.0, 12.0)
    assert rig.ids() == ["stool_1"]
    data = json.loads(vocabulary.VOCABULARY_PATH.read_text(encoding="utf-8"))
    data["labels"] = [k for k in data["labels"] if k["label"] != "stool"]
    rig = Rig(script=[[stool]], vocab=vocabulary.parse(data, "test vocabulary"))
    rig.feed(10.0, 12.0)
    assert rig.ids() == [] and rig.health()["tracks_tentative"] == 0 and rig.health()["reason"] == "ok"


def test_shapeless_sighting_makes_no_observation(monkeypatch):
    # a trash can over a 2 m wall (the seed is implausible) and a box at the image's bottom edge without points
    near_wall = solid(3.05, 0.0, 0.1, 6.0, height=2.0)
    dets = [Detection("trash_can", 0.9, (150.0, 150.0, 490.0, 300.0)),
            Detection("box", 0.9, (20.0, 300.0, 60.0, 479.0))]
    rig = Rig(script=[dets], scene=(near_wall,))
    seen = []
    update = rig.core.tracker.update
    monkeypatch.setattr(rig.core.tracker, "update", lambda obs, *a, **k: (seen.append(list(obs)), update(obs, *a, **k)))
    rig.feed(10.0, 10.5)
    assert seen == [[]]
    h = rig.health()
    assert (h["shapeless"], h["tracks_tentative"], h["reason"]) == (2, 0, "ok")


def test_used_up_ids_show_in_the_status():
    rig = Rig(script=[[Detection("box", 0.8, bbox(BOX))]], scene=(BOX, WALL))
    tracker = rig.core.tracker
    far = Observation("box", 0.8, (0.0, 0.0, 1.0, 1.0), -3.0, -3.0, (0.5, 0.5), False,
                      np.array([[-3.2, -3.2], [-2.8, -2.8]]), True, False, (0.0, 0.0))
    t = 0.0
    for n in range(1, MAX_ID + 1):                        # box_1 ... box_99 issued and retired
        tracker.update([far], t, 1000.0 + t, lambda x, y: True)
        tracker.update([far], t + 1.0, 1001.0 + t, lambda x, y: True)
        key = f"box_{n}"
        tracker.mark_vacant(key, True, t + 1.0)
        tracker.mark_vacant(key, True, t + 1.0 + MIN_HIT_GAP_S)
        t += 2.0
    assert tracker.stats()["ids_issued"] == {"box": 99} and tracker.stats()["notes"] == []
    rig.feed(10.0, 12.0)                                  # the next box is confirmed: no id left
    h = rig.health()
    assert h["notes"] == ["box: ids 1-99 used, new box tracks ignored"]
    assert rig.message()["objects"] == []
    h["notes"].append("x")
    assert rig.health()["notes"] == ["box: ids 1-99 used, new box tracks ignored"]   # a copy


# --- failures (spec §11.9) ------------------------------------------------------------------------------------

def test_no_camera_for_10_s_keeps_tracks_and_vacancy_still_runs(caplog):
    rig = confirmed_chair_core()
    with caplog.at_level(logging.WARNING, logger="detector.core"):
        rig.feed(121.0, 121.0 + CAMERA_TIMEOUT_S - 0.1, images=False)
        assert rig.health()["reason"] == "ok"
        rig.feed(rig.t, 121.0 + CAMERA_TIMEOUT_S + 0.5, images=False)
        h = rig.health()
        assert (h["ok"], h["reason"]) == (False, "no_camera")
        rows = rig.message()["objects"]
        assert [r["id"] for r in rows] == ["chair_1"] and rows[0]["visible"] is False   # kept, not seen
        rig.health()
    assert sum("no camera" in r.getMessage() for r in caplog.records) == 1      # once a minute
    rig.scene = (WALL,)                                   # the chair is taken away
    rig.feed(rig.t, rig.t + 1.5, images=False)
    h = rig.health()
    assert rig.message()["objects"] == [] and (h["ids_retired"], h["reason"]) == (1, "no_camera")


def test_vacancy_runs_at_most_twice_a_second(monkeypatch):
    rig = confirmed_chair_core()
    stamps = []
    mark = rig.core.tracker.mark_vacant
    monkeypatch.setattr(rig.core.tracker, "mark_vacant",
                        lambda key, vacant, t: (stamps.append((key, vacant, t)), mark(key, vacant, t)))
    rig.feed(121.0, 124.0)
    assert 5 <= len(stamps) <= 7 and all(key == "chair_1" and not vacant for key, vacant, _t in stamps)
    assert all(b - a >= 1.0 / VACANCY_MAX_HZ - 1e-6 for (_k, _v, a), (_k2, _v2, b) in zip(stamps, stamps[1:]))


def test_a_vacancy_scan_ahead_of_the_odometry_waits_for_it(monkeypatch):
    rig = confirmed_chair_core()
    rig.feed(121.0, 122.0, images=False)
    stamps = []
    mark = rig.core.tracker.mark_vacant
    monkeypatch.setattr(rig.core.tracker, "mark_vacant",
                        lambda key, vacant, t: (stamps.append(t), mark(key, vacant, t)))
    rig.clocks.advance(1.0 / VACANCY_MAX_HZ)
    rig.core.on_cloud(122.05, rig.cloud())                # ahead of the odometry (122.0)
    rig.core.on_odom(122.04, BASE, Q0, ZERO, ZERO)
    assert stamps == []
    rig.core.on_odom(122.06, BASE, Q0, ZERO, ZERO)
    assert stamps == [122.05]


def test_a_pending_vacancy_scan_whose_poses_are_gone_is_dropped():
    rig = confirmed_chair_core()
    rig.feed(121.0, 122.0, images=False)
    rig.clocks.advance(1.0 / VACANCY_MAX_HZ)              # the vacancy rate limit lets the next scan in
    rig.core.on_cloud(122.05, rig.cloud())                # a still scan ahead of the odometry (122.0): pending
    rig.clocks.advance(3.0)                               # an odometry gap of 3 s: 122.05 leaves the pose buffer
    rig.scene = (WALL,)                                   # the chair is taken away; the Go2 still stands
    rig.feed(125.06, 130.0, images=False)
    assert rig.ids() == [] and rig.health()["ids_retired"] == 1


def test_no_scan_in_the_still_interval_drops_the_image_and_says_no_cloud():
    rig = Rig()
    rig.feed(10.0, 11.0)
    rig.feed(11.02, 11.5, velocity=0.3)                   # a step
    dropped = rig.health()["frames_dropped"]
    calls = rig.detector.calls
    rig.feed(11.52, 13.0, clouds=False)                   # still again, the LiDAR silent: images 12.0, 12.5, 13.0
    h = rig.health()
    assert (h["ok"], h["reason"]) == (False, "no_cloud")
    assert h["frames_dropped"] == dropped + 2 and rig.detector.calls == calls
    assert rig.next_job() is None
    rig.feed(13.02, 13.2)                                 # a scan again: the newest image pairs with it
    assert rig.health()["reason"] == "ok" and rig.detector.calls == calls + 1


def test_a_lidar_lost_during_a_stop_says_no_cloud_once_its_last_scan_left_the_pose_buffer():
    rig = confirmed_chair_core()                          # still since 119.0, the last scan at 121.0
    rig.feed(121.02, 122.0, clouds=False)                 # the LiDAR stops; the Go2 keeps standing
    calls = rig.detector.calls                            # (the image at 121.5 may still pair with 121.0)
    rig.feed(122.02, 125.0, clouds=False)                 # 121.0 leaves the 2 s pose buffer
    h = rig.health()
    assert (h["ok"], h["reason"], h["still"]) == (False, "no_cloud", True)
    assert rig.detector.calls == calls and h["cloud_age_s"] > POSE_BUFFER_S
    rig.feed(125.02, 125.5)                               # scans again, still the same stop
    assert rig.health()["reason"] == "ok" and rig.detector.calls == calls + 1


def test_no_odometry_means_no_job_and_no_odometry():
    rig = Rig()
    rig.feed(10.0, 11.0)
    assert rig.health()["reason"] == "ok"
    calls = rig.detector.calls
    rig.feed(11.02, 13.0, odom=False)                     # scans and images, no odometry
    h = rig.health()
    assert (h["ok"], h["reason"], h["still"]) == (False, "no_odometry", False)
    assert rig.next_job() is None and rig.detector.calls == calls


def test_an_image_of_another_size_is_refused_once_with_bad_camera(caplog):
    rig = Rig()
    rig.feed(10.0, 10.5)
    dropped, calls = rig.health()["frames_dropped"], rig.detector.calls
    rig.image = np.zeros((240, 320, 3), dtype=np.uint8)   # not the constants
    with caplog.at_level(logging.WARNING, logger="detector.core"):
        rig.feed(10.52, 13.0)
    h = rig.health()
    assert (h["ok"], h["reason"], h["frames_dropped"]) == (False, "bad_camera", dropped + 5)
    assert rig.detector.calls == calls
    assert sum("320 x 240" in r.getMessage() for r in caplog.records) == 1        # logged once
    rig.image = IMAGE
    rig.feed(13.02, 14.0)
    assert rig.health()["reason"] == "ok" and rig.detector.calls > calls

    rig = Rig()                                           # a CameraInfo of another size: both sizes refused
    rig.core.on_camera_info([92.418, 0, 160, 0, 92.418, 120, 0, 0, 1], 320, 240)
    rig.image = np.zeros((240, 320, 3), dtype=np.uint8)
    rig.feed(10.0, 11.0)
    assert rig.health()["reason"] == "bad_camera" and rig.detector.calls == 0
    rig.image = IMAGE
    rig.feed(11.02, 12.0)
    assert rig.health()["reason"] == "bad_camera" and rig.detector.calls == 0


def test_camera_info_replaces_the_intrinsics_and_warns_once_when_they_disagree(caplog):
    rig = Rig()
    k = [184.836, 0, 320, 0, 184.836, 240, 0, 0, 1]
    with caplog.at_level(logging.WARNING, logger="detector.core"):
        rig.core.on_camera_info(k, 640, 480)
        assert not caplog.records
        odd = [200.0, 0, 320, 0, 200.0, 240, 0, 0, 1]
        rig.core.on_camera_info(odd, 640, 480)
        rig.core.on_camera_info(odd, 640, 480)
    assert sum("CameraInfo" in r.getMessage() for r in caplog.records) == 1
    views = []
    rig.detector.prepare = views.append
    rig.feed(10.0, 10.5)
    assert views and views[0].camera == CameraModel.from_k(odd, 640, 480)
    assert views[0].stamp == 10.5 and views[0].base_pos == BASE and views[0].base_quat == Q0


def test_three_exceptions_pause_for_30_s_then_recover(caplog):
    rig = Rig(detector=ScriptedDetector([[CHAIR_SEEN]], fail_on=frozenset({2, 3, 4})))
    with caplog.at_level(logging.ERROR, logger="detector.core"):
        rig.feed(10.0, 15.0)                              # calls at 10.5 and 11.5 confirm chair_1; 3 fail
    assert rig.detector.calls == 2 + ERROR_LIMIT
    assert sum(r.exc_info is not None for r in caplog.records) == ERROR_LIMIT     # each with its traceback
    h = rig.health()
    assert (h["ok"], h["reason"]) == (False, "detector_error")
    rig.feed(15.02, 14.5 + ERROR_BACKOFF_S - 0.1)
    assert rig.detector.calls == 2 + ERROR_LIMIT          # paused
    assert rig.ids() == ["chair_1"]                       # tracks and publishing go on
    rig.feed(rig.t, 14.5 + ERROR_BACKOFF_S + 1.0)
    assert rig.detector.calls == 3 + ERROR_LIMIT and rig.health()["reason"] == "ok"
    assert rig.health()["inferences"] == 3


def test_sim_time_going_backwards_resets_gate_buffer_and_tracks():   # Review Focus 2
    core = confirmed_chair_core()                         # chair_1 confirmed at sim t ~ 120
    feed_still(core, t0=0.5)                              # Gazebo restarted: stamps from 0.5 s
    assert core.health()["tracks_confirmed"] == 0 and core.next_job() is not None   # the gate works again
    confirm_chair(core, t0=1.0)
    assert [r["id"] for r in core.message()["objects"]] == ["chair_1"]              # numbering restarted


def test_a_small_step_back_is_no_restart(caplog):
    rig = confirmed_chair_core()
    with caplog.at_level(logging.WARNING, logger="detector.core"):
        rig.core.on_image(121.0 - TIME_BACK_S + 0.1, IMAGE)   # late, but within TIME_BACK_S
        rig.core.on_cloud(120.5, rig.cloud())
    assert rig.ids() == ["chair_1"] and not any("backwards" in r.getMessage() for r in caplog.records)
    with caplog.at_level(logging.WARNING, logger="detector.core"):
        rig.core.on_cloud(121.0 - TIME_BACK_S - 0.1, rig.cloud())
    assert rig.ids() == [] and any("sim time went backwards" in r.getMessage() for r in caplog.records)


def test_a_result_from_before_a_reset_is_discarded():
    def make(rig):
        def script(k, image):
            if k == 0:
                rig.core.reset("test")                    # the callback thread resets while detect() runs
            return [CHAIR_SEEN]
        return ScriptedDetector(script)

    rig = Rig(detector=make)
    rig.feed(10.0, 10.5)
    assert rig.detector.calls == 1 and rig.health()["tracks_tentative"] == 0
    rig.feed(10.52, 12.0)                                 # the worker is free again
    h = rig.health()
    assert rig.detector.calls == 2 and h["tracks_tentative"] + h["tracks_confirmed"] == 1


# --- in view (the tracker's in_view; design-detector §8.3, §8.6) -----------------------------------------------

def test_in_view_applies_range_frustum_and_occlusion():
    cam = CameraModel.from_hfov()
    crate = solid(1.5, 0.0, 0.3, 1.0, height=1.0)          # in front of anything at y = 0 beyond it
    world = scan([crate, WALL], BASE, Q0)
    lidar = bridge_core.lidar_points_world(np.zeros((1, 3)), BASE, Q0)[0]
    in_view = make_in_view(cam, BASE, Q0, world, lidar, seen=[], sizes={})
    assert in_view(3.0, 2.0)                               # a clear line
    assert not in_view(3.0, 0.0)                           # behind the crate
    assert not in_view(6.5, 1.0)                           # beyond 6 m
    assert not in_view(-1.0, 0.0) and not in_view(0.5, 3.0)   # behind the camera, outside the 120° view
    # seen in this image: in view whatever the LiDAR says
    seen = make_in_view(cam, BASE, Q0, world, lidar, seen=[(3.0, 0.0, (0.5, 0.5))], sizes={})
    assert seen(3.0, 0.0) and seen(3.2, 0.1)
    # a big object's own front face is not an occluder: the size of the published row decides
    big = solid(3.5, 0.0, 1.6, 2.0, height=1.0)           # its front face 0.8 m before its centre
    world = scan([big, WALL], BASE, Q0)
    assert make_in_view(cam, BASE, Q0, world, lidar, seen=[], sizes={(3.5, 0.0): (1.6, 2.0)})(3.5, 0.0)
    assert not make_in_view(cam, BASE, Q0, world, lidar, seen=[], sizes={})(3.5, 0.0)   # taken as 0.5 x 0.5


# --- the core imports no ML -----------------------------------------------------------------------------------

def test_the_core_and_the_oracle_import_no_ml():
    code = ("import sys, detector.core, detector.oracle\n"
            "bad = sorted(m for m in ('ultralytics', 'torch', 'torchvision', 'cv2', 'rclpy') if m in sys.modules)\n"
            "assert not bad, bad\n")
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(ROOT), str(ROOT / "nl_control")]))
    done = subprocess.run([sys.executable, "-c", code], cwd=ROOT, env=env, capture_output=True, text=True,
                          timeout=60)
    assert done.returncode == 0, done.stderr[-2000:]


def test_job_and_view_fields():
    view = View(BASE, Q0, CameraModel.from_hfov(), 1.0)
    assert (view.base_pos, view.base_quat, view.stamp) == (BASE, Q0, 1.0)
    job = Job(IMAGE, 1.0, np.zeros((0, 3)), 1.1, 0.5)
    assert (job.image_t, job.cloud_t, job.still_since) == (1.0, 1.1, 0.5)
    assert YOLOE_THRESHOLDS.sighting == 0.25
