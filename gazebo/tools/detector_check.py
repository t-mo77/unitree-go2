#!/usr/bin/env python3
"""Replay recorded still pairs through DetectorCore and score them against the truth, without Gazebo (spec §11.11,
§12.8, §13.3; design-detector §12.2, §12.3).

    python3 gazebo/tools/detector_check.py <set_dir> [--detector yoloe|oracle|yoloe_pf] [--layout household]

The detector image runs it for yoloe (and yoloe_pf), the app image for the oracle. <set_dir> holds the pairs of
gazebo/tools/record_frames.py (<view>.json, <view>.png, <view>_scan.npy). The detector is load_detector(--detector)
(default: DETECTOR, else yoloe) with set_vocabulary(the vocabulary's detect / detect_ignore prompts); the oracle gets
set_truth(the frame's truth rows) without heights, so its box uses the vocabulary height as in the node (review M8).

Replay (one fresh DetectorCore per frame, so every frame is scored on its own and the ids restart at 1): the core
gets the CameraInfo, then REPLAY_MAX_S of a still Go2 at the image's pose on fake clocks that advance with sim time
(odometry at 50 Hz, the scan at 10 Hz, the image at 2 Hz, the worker after every step) until its status shows
still_inferences >= 3, the point objects_check.py accuracy scores at. The scan is put into the LiDAR frame of the
image's pose first, so every LiDAR point keeps its odom position. A still interval of Gazebo gives (almost) the same
image every time (spec §11.5), so the detector runs once per frame and the core's later detect() calls on the frame
get the same detections; the latency is that one call's.

Output: one JSON line per frame, the accuracy result of objects_check.accuracy_check (detector rows vs the frame's
/gazebo/objects_truth, the layout's detectable / accept, /sim/world's button boxes, the base position for the robot,
the status for filled and still_inferences), plus "view", "code", "reason" (the status), "latency_ms", "hits" (the
detections at or above the detector's sighting threshold) and "set_failures" (set_failures(): the failures that count
for the set). Its targets (the 0.15 m line) are the PREVIEW_TARGETS of the frame's pose ("Bf_s2" is pose Bf; a view
of no test pose, such as B on the way to Bf, has none). Then one line for the set: the frames scored (with a
detectable truth in range, so with a recall line), the no_recall ones (no detectable truth in range: accuracy's
recall line has nothing to count there and is n/a, but their position, size, label and false-positive lines still
count), passed and failed (a frame fails on any of its set_failures), data errors, confusion() and preview(). Exit 0 when no frame fails and one at least is scored, 1
otherwise, 2 for data errors (no set, an unreadable record, a frame without truth for the layout, a detector that
cannot load).

confusion(frames): which label hit which truth object, with confidences (the tuning table of spec §12.8 step 4).
preview(frames, targets, confirm): the detection preview's rule (spec §12.8 step 3, plan H3, review M4 and M8).

The standard library, numpy and this repository only; the detector's own module is imported by load_detector.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import statistics
import sys
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]          # this repository (/work in the containers)
for _path in (ROOT / "nl_control", ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from detector.camera import CameraModel  # noqa: E402
from detector.core import DetectorCore, View  # noqa: E402
from detector.oracle import project_box  # noqa: E402
from detector.protocol import Detection, Detector, check_detections, load_detector, vocabulary_prompts  # noqa: E402
from gazebo.bridge_core import LIDAR_IN_BODY, lidar_points_world, quat_to_matrix  # noqa: E402
from gazebo.tools import objects_check  # noqa: E402
from gazebo.tools.png_rgb import read_png  # noqa: E402
from voice_ltl import layout as layouts  # noqa: E402
from voice_ltl import objects, vocabulary  # noqa: E402
from voice_ltl.objects import Obj  # noqa: E402

RECORD_VERSION = 1                         # == record_frames.RECORD_VERSION (not imported: no ROS names here)
REPLAY_T0 = 100.0                          # sim time of the first odometry of a replay
REPLAY_MAX_S = 10.0                        # sim seconds of still Go2 at most per frame
ODOM_DT, SCAN_EVERY, IMAGE_EVERY = 0.02, 5, 25   # 50 Hz odometry; a scan every 0.1 s, an image every 0.5 s
WALL0, CLOCK0 = 1000.0, 1.8e9              # the fake steady clock and time.time() at a replay's start
# the test pose of each detector target (§12.8 step 3), the room proposal after the H3 screening (R-H3a, R-H3b):
# Bf = F standing, F(turned_left & X F turned_left), F moved_forward; R5 = F standing, F turned_right, five
# F moved_forward in one formula. Each pose has the views <pose> and <pose>_s1 ... <pose>_s4 (record_frames --view)
PREVIEW_TARGETS: Mapping[str, str] = {"sofa_1": "Bf", "suitcase_1": "Bf", "traffic_cone_1": "R5"}
PREVIEW_IOU = 0.5                          # a hit is on the truth box at IoU >= 0.5
PREVIEW_SHIFT_HITS = 2                     # ... in the pose view and in >= 2 of its (4) shifted views
PREVIEW_MIN_KINDS = 3                      # new kinds needed (spec §12.8 step 5, §13.4)
SPAWN_VIEW = "spawn"                       # the standing spawn view: only box and chair may confirm there
SPAWN_LABELS = frozenset({"box", "chair"})
IGNORED_VIEWS = frozenset({"spawn_lying"})  # replayed and printed; preview never looks at it (review M4)
CONFUSION_IOU = 0.3                        # confusion(): a hit belongs to the truth box it overlaps most, from 0.3
NO_TRUTH = "-"                             # confusion()'s key for hits on no truth object
NO_RECALL = "min_recall: no detectable truth in range"   # objects_check.accuracy's failure for an empty scope
_SHIFTED = re.compile(r"(?P<pose>.+)_s(?P<k>[1-9][0-9]*)")
_ZERO = (0.0, 0.0, 0.0)


class SetError(ValueError):
    """A set this tool cannot read: one line naming the file and why."""


@dataclass(frozen=True, eq=False)
class Frame:
    """One recorded pair: its <view>.json, the image (H x W x 3 uint8 RGB) and the scan (N x 3, LiDAR frame)."""

    view: str
    record: dict
    image: np.ndarray
    scan: np.ndarray


@dataclass(frozen=True, eq=False)
class Replayed:
    """One frame after the replay."""

    view: str
    base_pos: tuple[float, float, float]           # the base pose at the image stamp
    base_quat: tuple[float, float, float, float]
    camera: CameraModel
    image_t: float
    truth: tuple[Obj, ...]                         # the frame's /gazebo/objects_truth rows
    layout: layouts.Layout
    hits: tuple[Detection, ...]                    # the detector's detections at or above its sighting threshold
    message: dict                                  # the core's /go2/objects
    health: dict                                   # the core's /go2/detector_status
    code: int                                      # objects_check.accuracy_check: 0 pass, 1 fail, 2 no data
    accuracy: dict
    latency_ms: float | None                       # the detector's one detect() call on this image


def pose_of(view: str) -> str:
    """The test pose of a view name: "Bf_s3" -> "Bf", "R5" -> "R5", "spawn_lying" -> "spawn_lying"."""
    m = _SHIFTED.fullmatch(view)
    return m.group("pose") if m else view


def load_frames(set_dir: str | os.PathLike) -> list[Frame]:
    """Every <view>.json of set_dir with its image and scan, sorted by view name. SetError for a missing folder, no
    records, or a record, image or scan that cannot be read."""
    folder = Path(set_dir)
    if not folder.is_dir():
        raise SetError(f"{folder}: no such folder")
    paths = sorted(folder.glob("*.json"))
    if not paths:
        raise SetError(f"{folder}: no recorded pairs (<view>.json)")
    return [_load_frame(p) for p in paths]


def replay(frames: Sequence[Frame], detector: Detector, lay: layouts.Layout,
           targets: Mapping[str, str] = PREVIEW_TARGETS) -> list[Replayed]:
    """Replay every frame through its own DetectorCore (see the module docstring) and score it."""
    once = _Once(detector)
    return [_replay(f, once, lay, targets) for f in frames]


def set_failures(accuracy: Mapping) -> list[str]:
    """The failures of one frame's accuracy result that count for the set: every one but NO_RECALL, the recall line
    of a frame without a detectable truth in range, which is n/a there (its other lines still hold or fail)."""
    return [f for f in accuracy.get("failures", ()) if f != NO_RECALL]


def frame_line(r: Replayed) -> dict:
    """The JSON line of one frame."""
    return {"view": r.view, "code": r.code, **r.accuracy, "reason": r.health.get("reason"),
            "latency_ms": r.latency_ms,
            "hits": [{"label": d.label, "confidence": round(float(d.confidence), 3),
                      "box": [round(float(v), 1) for v in d.box]} for d in r.hits],
            "set_failures": set_failures(r.accuracy)}


def truth_boxes(frame: Replayed) -> dict[str, tuple[float, float, float, float]]:
    """The image box of every truth row of the frame: its footprint x the layout height (the layout is the truth
    here; review M8), the vocabulary height for an id the layout lacks, projected at the image pose. Rows out of
    view have none."""
    view = View(frame.base_pos, frame.base_quat, frame.camera, frame.image_t)
    out = {}
    for row in frame.truth:
        entry = frame.layout.find(row.id)
        height = entry.height if entry is not None else vocabulary.height_of(row.label)
        if not height:
            continue
        got = project_box(view, row.x, row.y, row.size, height, frame.camera.width, frame.camera.height)
        if got is not None:
            out[row.id] = got[0]
    return out


def iou(a: Sequence[float], b: Sequence[float]) -> float:
    """Intersection over union of two image boxes (x0, y0, x1, y1)."""
    inter = max(0.0, min(a[2], b[2]) - max(a[0], b[0])) * max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def confusion(frames: Sequence[Replayed]) -> dict:
    """Which label hit which truth object, with which confidence (spec §11.11, §12.8 step 4), over every frame:
    {"by_label": {label: {truth id: [hit, ...]}}, "by_truth": {truth id: {label: [hit, ...]}}} with
    hit = {"view", "confidence", "iou"}. A hit belongs to the truth box (truth_boxes) it overlaps most when that IoU
    is at least CONFUSION_IOU, else to NO_TRUTH (its iou is then the best overlap). Hits are the detections at or
    above the detector's sighting threshold. Keys are sorted; hits keep the frame order."""
    by_label: dict[str, dict[str, list[dict]]] = {}
    by_truth: dict[str, dict[str, list[dict]]] = {}
    for f in frames:
        boxes = truth_boxes(f)
        for d in f.hits:
            best, best_iou = NO_TRUTH, 0.0
            for obj_id in sorted(boxes):
                v = iou(d.box, boxes[obj_id])
                if v > best_iou:
                    best, best_iou = obj_id, v
            if best_iou < CONFUSION_IOU:
                best = NO_TRUTH
            hit = {"view": f.view, "confidence": round(float(d.confidence), 3), "iou": round(best_iou, 3)}
            by_label.setdefault(d.label, {}).setdefault(best, []).append(hit)
            by_truth.setdefault(best, {}).setdefault(d.label, []).append(hit)
    return {"by_label": _sorted(by_label), "by_truth": _sorted(by_truth)}


def preview(frames: Sequence[Replayed], targets: Mapping[str, str], confirm: float = 0.35) -> dict:
    """The detection preview (spec §12.8 step 3). targets: truth id -> its test pose (PREVIEW_TARGETS).

    A target passes when a hit with one of its layout accept labels at confidence >= confirm lies on its truth box
    (IoU >= PREVIEW_IOU; truth_boxes: the layout height) in the pose view and in >= PREVIEW_SHIFT_HITS of the pose's
    shifted views (<pose>_s1 ... <pose>_s4), and no other target's label at >= confirm lies on it in any of those
    views. The standing spawn view SPAWN_VIEW passes when nothing confirms (>= confirm) with a label other than
    box and chair. IGNORED_VIEWS (spawn_lying) count for nothing.
    {"targets": {id: {"pose", "passed", "pose_hit" (None: no pose frame), "shift_hits", "shift_views", "hits",
    "wrong"}}, "spawn": {"passed" (None: no spawn frame), "labels"}, "detectable": [ids passed],
    "enough": >= PREVIEW_MIN_KINDS passed, "confirm"}"""
    by_view = {f.view: f for f in frames if f.view not in IGNORED_VIEWS}
    accepts = {t: _accept(frames, t) for t in targets}
    result: dict = {}
    for target, pose in sorted(targets.items()):
        own = accepts[target]
        others = set().union(*(accepts[o] for o in targets if o != target)) - own if len(targets) > 1 else set()
        shifted = sorted((v for v in by_view if pose_of(v) == pose and v != pose),
                         key=lambda v: int(_SHIFTED.fullmatch(v).group("k")))
        hits, wrong, hit_views = [], [], set()
        for view in ([pose] if pose in by_view else []) + shifted:
            frame = by_view[view]
            box = truth_boxes(frame).get(target)
            if box is None:
                continue
            for d in frame.hits:
                if float(d.confidence) < confirm - 1e-12:
                    continue
                overlap = iou(d.box, box)
                if overlap < PREVIEW_IOU - 1e-12:
                    continue
                entry = {"view": view, "label": d.label, "confidence": round(float(d.confidence), 3),
                         "iou": round(overlap, 3)}
                if d.label in own:
                    hits.append(entry)
                    hit_views.add(view)
                elif d.label in others:
                    wrong.append(entry)
        pose_hit = (pose in hit_views) if pose in by_view else None
        shift_hits = sum(v in hit_views for v in shifted)
        result[target] = {"pose": pose, "passed": bool(pose_hit) and shift_hits >= PREVIEW_SHIFT_HITS and not wrong,
                          "pose_hit": pose_hit, "shift_hits": shift_hits, "shift_views": shifted,
                          "hits": hits, "wrong": wrong}
    spawn = by_view.get(SPAWN_VIEW)
    labels = [] if spawn is None else [{"label": d.label, "confidence": round(float(d.confidence), 3)}
                                       for d in spawn.hits
                                       if d.label not in SPAWN_LABELS and float(d.confidence) >= confirm - 1e-12]
    detectable = sorted(t for t, r in result.items() if r["passed"])
    return {"targets": result, "spawn": {"passed": None if spawn is None else not labels, "labels": labels},
            "detectable": detectable, "enough": len(detectable) >= PREVIEW_MIN_KINDS, "confirm": float(confirm)}


def set_line(frames: Sequence[Replayed], set_name: str, detector_name: str, lay: layouts.Layout,
             targets: Mapping[str, str] = PREVIEW_TARGETS, confirm: float = 0.35) -> dict:
    """The set's JSON line (see the module docstring)."""
    errors = sorted(f.view for f in frames if f.code == 2)
    judged = [f for f in frames if f.code != 2]
    scored = sorted(f.view for f in judged if f.accuracy.get("in_scope"))
    no_recall = sorted(f.view for f in judged if not f.accuracy.get("in_scope"))
    failed = sorted(f.view for f in judged if set_failures(f.accuracy))
    latencies = [f.latency_ms for f in frames if f.latency_ms is not None]
    return {"set": set_name, "detector": detector_name, "layout": lay.name, "frames": len(frames),
            "scored": scored, "no_recall": no_recall,
            "passed_frames": sorted(f.view for f in judged if f.view not in failed), "failed": failed,
            "errors": errors, "passed": bool(scored) and not failed and not errors,
            "latency_ms_p50": round(statistics.median(latencies), 1) if latencies else None,
            "latency_ms_max": round(max(latencies), 1) if latencies else None,
            "confusion": confusion(frames), "preview": preview(frames, targets, confirm)}


# --- internals -----------------------------------------------------------------------------------------------------

class _Once:
    """The detector as the replay's cores see it: the first detect() of a frame runs the detector (timed); the
    core's later calls on that frame get the same detections (new_frame() starts the next frame)."""

    def __init__(self, detector: Detector) -> None:
        self.inner = detector
        self.name = str(getattr(detector, "name", ""))
        self.thresholds = detector.thresholds
        self.result: list | None = None
        self.latency_s: float | None = None

    def new_frame(self) -> None:
        self.result, self.latency_s = None, None

    def set_vocabulary(self, prompts, ignore) -> None:
        self.inner.set_vocabulary(prompts, ignore)

    def prepare(self, view) -> None:
        prepare = getattr(self.inner, "prepare", None)
        if callable(prepare):
            prepare(view)

    def detect(self, image_rgb: np.ndarray) -> list:
        if self.result is None:
            start = time.perf_counter()
            dets = self.inner.detect(image_rgb)
            self.latency_s = time.perf_counter() - start
            if not isinstance(dets, list):
                raise ValueError(f"detect() must return a list, not {type(dets).__name__}")
            self.result = dets
        return list(self.result)

    def close(self) -> None:
        pass                                               # main() closes the detector itself


class _Clocks:
    def __init__(self) -> None:
        self.w, self.c = WALL0, CLOCK0

    def wall(self) -> float:
        return self.w

    def clock(self) -> float:
        return self.c


def _replay(frame: Frame, once: _Once, lay: layouts.Layout, targets: Mapping[str, str]) -> Replayed:
    rec = frame.record
    pos, quat = _pose(rec["image_pose"])
    cloud_pos, cloud_quat = _pose(rec["cloud_pose"])
    info = rec.get("camera_info")
    camera = (CameraModel.from_k(info["k"], info["width"], info["height"]) if isinstance(info, dict)
              else CameraModel.from_hfov())
    truth = objects.parse(rec.get("truth"))
    set_truth = getattr(once.inner, "set_truth", None)
    if callable(set_truth):
        set_truth(truth)                                   # no heights: the vocabulary's (review M8)
    once.new_frame()
    clocks = _Clocks()
    core = DetectorCore(once, wall=clocks.wall, clock=clocks.clock)
    core.mark_loaded()
    first = core.health()
    if isinstance(info, dict):
        core.on_camera_info(info["k"], info["width"], info["height"])
    # the scan in the LiDAR frame of the image's pose: the same odom points under one odometry pose
    world = lidar_points_world(frame.scan, cloud_pos, cloud_quat)
    scan = (world - np.asarray(pos)) @ quat_to_matrix(quat) - np.asarray(LIDAR_IN_BODY)
    for k in range(int(round(REPLAY_MAX_S / ODOM_DT)) + 1):
        t = round(REPLAY_T0 + k * ODOM_DT, 6)
        if k:
            clocks.w += ODOM_DT
            clocks.c += ODOM_DT
        core.on_odom(t, pos, quat, _ZERO, _ZERO)
        if k % SCAN_EVERY == 0:
            core.on_cloud(t, scan)
        if k % IMAGE_EVERY == 0:
            core.on_image(t, frame.image)
        job = core.next_job()
        if job is not None:
            core.run(job)
            if core.health()["still_inferences"] >= objects_check.STILL_INFERENCES:
                break
    message, health = core.message(), core.health()
    mine = sorted(t for t, p in targets.items() if p == pose_of(frame.view))
    code, result = objects_check.accuracy_check(message, rec.get("truth"), lay, rec.get("world"), (pos[0], pos[1]),
                                                mine, objects_check.ACCURACY_RANGE_M, health, first)
    sighting = float(once.thresholds.sighting)
    hits = tuple(d for d in (once.result or ()) if _hit(d, frame.image.shape, sighting))
    latency = None if once.latency_s is None else round(1000.0 * once.latency_s, 1)
    return Replayed(frame.view, pos, quat, camera, float(rec["image_t"]), truth, lay, hits, message, health, code,
                    result, latency)


def _hit(d: object, shape: tuple[int, ...], sighting: float) -> bool:
    if not isinstance(d, Detection):
        return False
    try:
        check_detections([d], shape, vocabulary.LABELS)
    except ValueError:
        return False
    return float(d.confidence) >= sighting - 1e-12


def _accept(frames: Sequence[Replayed], target: str) -> set[str]:
    """The target's layout accept labels (its own label when the layout lacks it)."""
    for f in frames:
        entry = f.layout.find(target)
        if entry is not None:
            return set(entry.accept)
    m = re.fullmatch(r"(.+)_[1-9][0-9]?", target)
    return {m.group(1)} if m else set()


def _sorted(table: dict) -> dict:
    return {k: dict(sorted(v.items())) for k, v in sorted(table.items())}


def _pose(p: object) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    pos, quat = tuple(float(v) for v in p["pos"]), tuple(float(v) for v in p["quat"])
    if len(pos) != 3 or len(quat) != 4 or not all(math.isfinite(v) for v in pos + quat):
        raise ValueError(f"a pose is 3 + 4 finite numbers: {p!r}")
    return pos, quat


def _load_frame(path: Path) -> Frame:
    where = path.name
    try:
        record = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError) as e:
        raise SetError(f"{where}: cannot read the record ({type(e).__name__})") from None
    except ValueError as e:
        raise SetError(f"{where}: not valid JSON ({e})") from None
    if not isinstance(record, dict) or record.get("version") != RECORD_VERSION:
        raise SetError(f"{where}: not a record_frames pair of version {RECORD_VERSION}")
    if record.get("view") != path.stem:
        raise SetError(f"{where}: its view is {record.get('view')!r}, not {path.stem!r}")
    try:
        _pose(record["image_pose"])
        _pose(record["cloud_pose"])
        float(record["image_t"])
        info = record.get("camera_info")
        if info is not None:
            CameraModel.from_k(info["k"], info["width"], info["height"])
    except (KeyError, TypeError, ValueError) as e:
        raise SetError(f"{where}: a bad pose, stamp or camera_info ({e})") from None
    for key in ("image", "scan"):
        if record.get(key) != {"image": f"{path.stem}.png", "scan": f"{path.stem}_scan.npy"}[key]:
            raise SetError(f"{where}: {key} must be the file next to it, not {record.get(key)!r}")
    try:
        image = read_png(path.parent / record["image"])
    except OSError as e:
        raise SetError(f"{record['image']}: cannot read the image ({type(e).__name__})") from None
    except ValueError as e:
        raise SetError(str(e)) from None
    try:
        scan = np.load(path.parent / record["scan"], allow_pickle=False)
    except (OSError, ValueError) as e:
        raise SetError(f"{record['scan']}: cannot read the scan ({e})") from None
    scan = np.asarray(scan, dtype=np.float64)
    if scan.ndim != 2 or scan.shape[1] != 3 or not np.isfinite(scan).all():
        raise SetError(f"{record['scan']}: a scan is N x 3 finite numbers, not {scan.shape}")
    return Frame(path.stem, record, image, scan)


def _parse(argv: list[str] | None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Replay recorded still pairs through the detector core (spec §11.11).")
    p.add_argument("set_dir", help="the folder record_frames.py wrote")
    p.add_argument("--detector", choices=("yoloe", "oracle", "yoloe_pf"), default=None,
                   help="the detector (default: DETECTOR, else yoloe)")
    p.add_argument("--layout", default=None, help="the layout (default: GO2_LAYOUT, else household)")
    return p.parse_args(argv)


def _print(payload: dict) -> None:
    print(json.dumps(payload, ensure_ascii=False), flush=True)


def main(argv: list[str] | None = None) -> int:
    args = _parse(argv)
    try:
        lay = layouts.load(objects_check.layout_name(args.layout))
        frames = load_frames(args.set_dir)
    except (layouts.LayoutError, SetError) as e:
        _print({"error": str(e)})
        return 2
    try:
        detector = load_detector(args.detector)
        detector.set_vocabulary(*vocabulary_prompts())
    except Exception as e:                                 # an import or model failure: no data
        _print({"error": f"detector {args.detector or os.environ.get('DETECTOR') or 'yoloe'} did not load: "
                         f"{type(e).__name__}: {e}"})
        return 2
    try:
        replayed = replay(frames, detector, lay, PREVIEW_TARGETS)
    finally:
        detector.close()
    for r in replayed:
        _print(frame_line(r))
    line = set_line(replayed, Path(args.set_dir).name, str(getattr(detector, "name", "")), lay, PREVIEW_TARGETS,
                    float(detector.thresholds.confirm))
    _print(line)
    if line["errors"]:
        return 2
    return 0 if line["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
