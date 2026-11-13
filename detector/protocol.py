"""検出器の取り替え口（open-objects spec §11.10、design-detector §11）。

検出器は絵（H × W × 3 の uint8 RGB）を受け、単語帳の label の検出の一覧を返す。確からしさのしきい値は
検出器のもの（`Thresholds`。検出器ごとに確からしさの付け方が違う）。`sighting` は core がかけ、
`confirm`・`confirm_unsupported` は tracker が使う。

検出器の約束（`check_detections` が確かめる）：
- 一覧（list）を返す。label は単語帳の label（検出器の名前は取り替え口の中で写す）。
- confidence は [0, 1] の有限の数。box は渡した絵の画素で 0 ≤ x0 < x1 ≤ W、0 ≤ y0 < y1 ≤ H。
- mask は None か、絵と同じ H × W の真偽。
- 絵を変えない・持たない。種類を区別しない NMS をする。自分の下限まで低い確からしさを返してよい。

`load_detector` は名前（既定は環境の DETECTOR、なければ yoloe）で選び、そのときだけ import する。
このモジュールは標準ライブラリ・numpy・このリポジトリだけを import する（ultralytics・torch は
detector/yoloe*.py だけ。D5）。
"""

from __future__ import annotations

import importlib
import math
import numbers
import os
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

import numpy as np

from voice_ltl import vocabulary

DEFAULT_DETECTOR = "yoloe"
# name -> (module, class); imported only when that detector is made. The prompt-free YOLOE (spec §11.10,
# detector/yoloe_pf.py) is not built in v1 (O24: text-prompt YOLOE stays), so it is not listed until it exists.
DETECTORS: Mapping[str, tuple[str, str]] = {
    "yoloe": ("detector.yoloe", "YoloeDetector"),
    "oracle": ("detector.oracle", "OracleDetector"),
}


@dataclass(frozen=True)
class Detection:
    label: str                                   # a vocabulary label (the adapter maps its classes / prompts)
    confidence: float                            # in [0, 1]
    box: tuple[float, float, float, float]       # x0, y0, x1, y1 in pixels of the image given; x0 < x1, y0 < y1
    mask: np.ndarray | None = field(default=None, compare=False)   # bool H x W, or None; not compared


@dataclass(frozen=True)
class Thresholds:
    """One detector's score calibration (spec §11.7): sighting <= confirm <= confirm_unsupported."""

    sighting: float                              # below: ignored
    confirm: float                               # a new id needs one hit at or above (of the label confirmed)
    confirm_unsupported: float                   # every hit of a track without LiDAR support
    by_label: Mapping[str, float] = field(default_factory=dict)   # per-label confirm overrides (vocabulary labels)

    def confirm_for(self, label: str) -> float:
        """The confirm threshold of label: its override, else confirm."""
        return float(self.by_label.get(label, self.confirm))


@runtime_checkable
class Detector(Protocol):
    name: str                                    # logs and status only
    thresholds: Thresholds

    def set_vocabulary(self, prompts: Mapping[str, Sequence[str]], ignore: Sequence[str]) -> None:
        """label -> English prompts (the vocabulary's "detect"); ignore: prompts detected but never returned."""

    def detect(self, image_rgb: np.ndarray) -> list[Detection]:
        """H x W x 3 uint8 RGB; must not keep or modify it; class-agnostic NMS; may go down to its own floor."""

    def close(self) -> None:
        """Free the model."""

    # optional: def prepare(self, view) -> None -- the core calls it before detect() when present
    # (view: detector.core.View)


YOLOE_THRESHOLDS = Thresholds(0.25, 0.35, 0.5)


def vocabulary_prompts(vocab: vocabulary.Vocabulary = vocabulary.VOCABULARY
                       ) -> tuple[dict[str, tuple[str, ...]], tuple[str, ...]]:
    """The arguments of set_vocabulary: every kind's "detect" prompts, and the "detect_ignore" prompts."""
    return {e.label: tuple(e.detect) for e in vocab.entries}, tuple(vocab.detect_ignore)


def _real(v: object) -> bool:
    """A finite real number (numpy's too), not a bool."""
    if isinstance(v, (bool, np.bool_)) or not isinstance(v, numbers.Real):
        return False
    try:
        return math.isfinite(float(v))
    except (OverflowError, ValueError):
        return False


def check_detections(dets: Sequence[Detection], image_shape: tuple[int, ...], labels: frozenset[str]) -> None:
    """The detector contract (spec §11.10). Raises ValueError naming the first breach."""
    if not isinstance(dets, list):
        raise ValueError(f"detect() must return a list, not {type(dets).__name__}")
    if len(image_shape) < 2:
        raise ValueError(f"image shape {image_shape!r} has no height and width")
    h, w = int(image_shape[0]), int(image_shape[1])
    for i, d in enumerate(dets):
        where = f"detection {i}"
        if not isinstance(d, Detection):
            raise ValueError(f"{where}: not a Detection: {d!r}")
        if d.label not in labels:
            raise ValueError(f"{where}: label {d.label!r} is not a vocabulary label")
        if not (_real(d.confidence) and 0.0 <= float(d.confidence) <= 1.0):
            raise ValueError(f"{where} ({d.label}): confidence {d.confidence!r} is not a number in [0, 1]")
        box = d.box
        if not (isinstance(box, (tuple, list)) and len(box) == 4 and all(_real(v) for v in box)):
            raise ValueError(f"{where} ({d.label}): box {box!r} is not 4 finite numbers")
        x0, y0, x1, y1 = (float(v) for v in box)
        if not (0.0 <= x0 < x1 <= w and 0.0 <= y0 < y1 <= h):
            raise ValueError(f"{where} ({d.label}): box {box!r} is not inside the {w} x {h} image "
                             "with x0 < x1 and y0 < y1")
        if d.mask is not None:
            if not isinstance(d.mask, np.ndarray) or d.mask.dtype != np.bool_ or d.mask.shape != (h, w):
                shape = getattr(d.mask, "shape", None)
                dtype = getattr(d.mask, "dtype", None)
                raise ValueError(f"{where} ({d.label}): mask {shape} {dtype} is not a bool {h} x {w} array")


def check_thresholds(thresholds: Thresholds, labels: frozenset[str]) -> None:
    """sighting <= confirm <= confirm_unsupported, all in [0, 1]; overrides only for labels, in [sighting, 1].
    Raises ValueError."""
    values = (thresholds.sighting, thresholds.confirm, thresholds.confirm_unsupported)
    if not all(_real(v) and 0.0 <= float(v) <= 1.0 for v in values):
        raise ValueError(f"thresholds {values!r} must be numbers in [0, 1]")
    if not values[0] <= values[1] <= values[2]:
        raise ValueError(f"thresholds {values!r} must be sighting <= confirm <= confirm_unsupported")
    for label, v in dict(thresholds.by_label).items():
        if label not in labels:
            raise ValueError(f"confirm override for {label!r}: not a vocabulary label")
        if not (_real(v) and values[0] <= float(v) <= 1.0):
            raise ValueError(f"confirm override for {label!r}: {v!r} is not in [sighting, 1]")


def load_detector(name: str | None = None) -> Detector:
    """The detector called name (default: env DETECTOR, else "yoloe"); its module is imported only now.
    An unknown name raises ValueError before anything is imported."""
    if name is None:
        name = os.environ.get("DETECTOR") or DEFAULT_DETECTOR
    if name not in DETECTORS:
        raise ValueError(f"DETECTOR must be one of {', '.join(DETECTORS)}: {name!r}")
    module_name, class_name = DETECTORS[name]
    return getattr(_import(module_name), class_name)()


def _import(module_name: str):
    """The one lazy import (a seam for the tests)."""
    return importlib.import_module(module_name)
