"""The oracle detector: the truth projected into the image (measurement only, never the default; spec §11.10,
§11.11; design-detector §11.1).

For every truth row (a vocabulary label) it returns the image box of the truth footprint x a height, from the
camera model at the image-stamp pose that DetectorCore passes to prepare(view), with confidence 1.0 and no mask.
The height is the vocabulary's (spec §11.10: truth footprint x vocabulary height) unless set_truth gets heights
by id. Running the core with it isolates the geometry and tracking error from the detector's.

It has no occlusion model (use viewpoints without occlusion). Like every detector it applies class-agnostic NMS
(IoU 0.7), keeping the nearer of two boxes that overlap that much. A box is the projection of the part of the
truth box in front of the camera's near plane (PROJ_NEAR), clipped to the image; truth entirely behind it, beyond
the far clip or outside the image gives no box.

The standard library, numpy and this repository only.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

import numpy as np

from detector.camera import CAMERA_CLIP, PROJ_NEAR, world_to_camera
from detector.protocol import YOLOE_THRESHOLDS, Detection, Thresholds
from voice_ltl import vocabulary
from voice_ltl.objects import Obj

NMS_IOU = 0.7
MIN_BOX_PX = 1.0                     # a box narrower or lower than this (after clipping) is no box
# the 12 edges of a box whose corner i has x, y, z on its bits 1, 2, 4
_EDGES = tuple((i, i | bit) for i in range(8) for bit in (1, 2, 4) if not i & bit)


class OracleDetector:
    name = "oracle"

    def __init__(self, thresholds: Thresholds = YOLOE_THRESHOLDS) -> None:
        self.thresholds = thresholds
        self._labels: frozenset[str] | None = None       # set_vocabulary's labels; None: every vocabulary label
        self._truth: tuple[tuple[str, float, float, float, float, float], ...] = ()
        self._view = None

    def set_vocabulary(self, prompts: Mapping[str, Sequence[str]], ignore: Sequence[str]) -> None:
        """Only the labels given (vocabulary labels) are returned; ignore has nothing to drop here."""
        self._labels = frozenset(prompts) & vocabulary.LABELS

    def set_truth(self, rows: Sequence[Obj], heights: Mapping[str, float] | None = None) -> None:
        """The truth rows (/gazebo/objects_truth, odom). heights: by id; for an id without one, and with None,
        vocabulary.height_of(row.label). Rows whose label is not a vocabulary label are left out."""
        truth = []
        for row in rows:
            if not isinstance(row, Obj) or row.label not in vocabulary.LABELS:
                continue
            h = None if heights is None else heights.get(row.id)
            if h is None:
                h = vocabulary.height_of(row.label)
            try:
                values = tuple(float(v) for v in (row.x, row.y, row.size[0], row.size[1], h))
            except (TypeError, ValueError, IndexError):
                continue
            if all(math.isfinite(v) for v in values) and min(values[2:]) > 0:
                truth.append((row.label, *values))
        self._truth = tuple(truth)                        # one assignment: safe beside a running detect()

    def prepare(self, view) -> None:
        """The camera pose and model of the next image (detector.core.View)."""
        self._view = view

    def detect(self, image_rgb: np.ndarray) -> list[Detection]:
        view, truth, labels = self._view, self._truth, self._labels
        if view is None:
            return []
        height_px, width_px = (int(v) for v in np.shape(image_rgb)[:2])
        found = []
        for label, x, y, sx, sy, h in truth:
            if labels is not None and label not in labels:
                continue
            got = project_box(view, x, y, (sx, sy), h, width_px, height_px)
            if got is not None:
                found.append((got[1], label, got[0]))
        found.sort(key=lambda f: f[0])                    # nearest first
        kept: list[Detection] = []
        for _depth, label, box in found:
            if all(_iou(box, k.box) < NMS_IOU for k in kept):
                kept.append(Detection(label, 1.0, box))
        return kept

    def close(self) -> None:
        self._truth = ()
        self._view = None


def project_box(view, x: float, y: float, size: tuple[float, float], height: float, width_px: int,
                height_px: int) -> tuple[tuple[float, float, float, float], float] | None:
    """The image box (x0, y0, x1, y1) of the odom box centred at (x, y) with size (sx, sy), from the floor to
    height, at view's pose and camera, and its nearest depth (camera x); None when no part is in view."""
    sx, sy = size
    corners = np.array([[x + (sx if i & 1 else -sx) / 2, y + (sy if i & 2 else -sy) / 2, height if i & 4 else 0.0]
                        for i in range(8)])
    p = world_to_camera(corners, view.base_pos, view.base_quat)
    ahead = p[:, 0] > PROJ_NEAR
    if not ahead.any() or (p[:, 0] > CAMERA_CLIP[1]).all():
        return None
    parts = [p[ahead]]
    for i, j in _EDGES:                                   # where the near plane cuts an edge
        if ahead[i] != ahead[j]:
            a = (PROJ_NEAR - p[i, 0]) / (p[j, 0] - p[i, 0])
            q = p[i] + a * (p[j] - p[i])
            q[0] = PROJ_NEAR
            parts.append(q[None, :])
    q = np.vstack(parts)
    cam = view.camera
    u = cam.cx - cam.fx * q[:, 1] / q[:, 0]
    v = cam.cy - cam.fy * q[:, 2] / q[:, 0]
    x0, x1 = (float(np.clip(w, 0.0, width_px)) for w in (u.min(), u.max()))
    y0, y1 = (float(np.clip(w, 0.0, height_px)) for w in (v.min(), v.max()))
    if x1 - x0 < MIN_BOX_PX or y1 - y0 < MIN_BOX_PX:
        return None
    return (x0, y0, x1, y1), float(q[:, 0].min())


def _iou(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
    inter = max(0.0, min(a[2], b[2]) - max(a[0], b[0])) * max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0
