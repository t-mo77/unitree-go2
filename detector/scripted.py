"""決めた検出を返す検出器（テスト用。Task 2.2・2.7・2.10・2.12 のテストが使う。review M7）。

標準ライブラリと numpy だけ。`Detector` の約束を満たす。
"""

from __future__ import annotations

import time
from collections.abc import Callable, Mapping, Sequence

import numpy as np

from detector.protocol import YOLOE_THRESHOLDS, Detection, Thresholds


class ScriptedDetector:
    """detect() call k (from 0) returns script[k] (the last one repeats; [] for an empty script), or
    script(k, image). It sleeps delay_s first, and raises RuntimeError on the calls in fail_on."""

    name = "scripted"

    def __init__(self, script: Sequence[list[Detection]] | Callable[[int, np.ndarray], list[Detection]],
                 thresholds: Thresholds = YOLOE_THRESHOLDS, delay_s: float = 0.0,
                 fail_on: frozenset[int] = frozenset()) -> None:
        self._script = script
        self.thresholds = thresholds
        self.delay_s = float(delay_s)
        self.fail_on = frozenset(fail_on)
        self.calls = 0
        self.vocabulary: tuple[Mapping[str, Sequence[str]], Sequence[str]] | None = None
        self.closed = False

    def set_vocabulary(self, prompts: Mapping[str, Sequence[str]], ignore: Sequence[str]) -> None:
        self.vocabulary = ({label: tuple(p) for label, p in prompts.items()}, tuple(ignore))

    def detect(self, image_rgb: np.ndarray) -> list[Detection]:
        k = self.calls
        self.calls += 1
        if self.delay_s > 0:
            time.sleep(self.delay_s)
        if k in self.fail_on:
            raise RuntimeError(f"scripted detector: call {k} fails (fail_on)")
        if callable(self._script):
            return list(self._script(k, image_rgb))
        if len(self._script) == 0:
            return []
        return list(self._script[min(k, len(self._script) - 1)])

    def close(self) -> None:
        self.closed = True
