"""detector/protocol.py：検出器の取り替え口の約束（spec §11.10、design-detector §11・§16 "Protocol contract"）。

約束のテストは、ここで使える実装すべてに流す（ScriptedDetector、Task 2.7 の OracleDetector、
Task 2.11 の YoloeDetector。YoloeDetector は `detector` の印つきで、ultralytics のない所では飛ばす）。
実装ごとの factory は、set_vocabulary の前の用意まで済ませた検出器を返す。
"""

import importlib.util
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import pytest

from voice_ltl import vocabulary
from detector import protocol
from detector.protocol import (
    YOLOE_THRESHOLDS, Detection, Detector, Thresholds, check_detections, check_thresholds, load_detector,
    vocabulary_prompts,
)
from detector.scripted import ScriptedDetector
from voice_ltl.objects import Obj

ROOT = Path(__file__).resolve().parent.parent
IMAGE_SHAPE = (480, 640, 3)


def make_image():
    rng = np.random.default_rng(7)
    return rng.integers(0, 256, size=IMAGE_SHAPE, dtype=np.uint8)


def make_scripted():
    def script(k, image):
        h, w = image.shape[:2]
        mask = np.zeros((h, w), dtype=bool)
        mask[200:300, 100:220] = True
        return [Detection("chair", 0.72, (100.0, 200.0, 220.0, 300.0), mask),
                Detection("box", 0.31, (400.0, 250.0, 520.0, 330.0)),
                Detection("sofa", 0.12, (0.0, 0.0, float(w), float(h)))]
    return ScriptedDetector(script)


def make_oracle():
    """The oracle with a truth in view, one partly outside the image and one behind the camera, at a pose."""
    from detector.camera import CameraModel
    from detector.core import View
    from detector.oracle import OracleDetector

    det = OracleDetector()
    det.set_truth([Obj("chair_1", "chair", 2.5, -0.5, (0.45, 0.45)), Obj("sofa_1", "sofa", 4.0, 1.0, (0.9, 1.8)),
                   Obj("table_1", "table", 1.0, 1.4, (1.2, 0.75)), Obj("box_1", "box", -2.0, 0.0, (0.5, 0.5))])
    det.prepare(View((0.0, 0.0, 0.35), (1.0, 0.0, 0.0, 0.0), CameraModel.from_hfov(), 0.0))
    return det


def make_yoloe():
    """The YOLOE-11s adapter with its embedding cache in a throwaway folder (the detector image only)."""
    from detector.yoloe import YoloeDetector

    return YoloeDetector(cache_dir=os.path.join(tempfile.gettempdir(), "go2-test-yoloe-cache"))


IMPLEMENTATIONS = [
    pytest.param(make_scripted, id="scripted"),
    pytest.param(make_oracle, id="oracle"),
    pytest.param(make_yoloe, id="yoloe", marks=[
        pytest.mark.detector,
        pytest.mark.skipif(importlib.util.find_spec("ultralytics") is None,
                           reason="needs the detector image (ultralytics)")]),
]


@pytest.fixture(params=IMPLEMENTATIONS)
def detector(request):
    det = request.param()
    det.set_vocabulary(*vocabulary_prompts())
    yield det
    det.close()


@pytest.fixture
def image():
    return make_image()


# --- the contract, for every implementation --------------------------------------------------------------------

def test_satisfies_the_protocol(detector):
    assert isinstance(detector, Detector)
    assert isinstance(detector.name, str) and detector.name
    assert isinstance(detector.thresholds, Thresholds)


def test_detect_returns_a_list(detector, image):
    dets = detector.detect(image)
    assert isinstance(dets, list)
    assert all(isinstance(d, Detection) for d in dets)


def test_labels_are_in_the_vocabulary(detector, image):
    assert all(d.label in vocabulary.LABELS for d in detector.detect(image))


def test_confidences_are_in_0_1(detector, image):
    for d in detector.detect(image):
        assert not isinstance(d.confidence, bool)
        assert 0.0 <= d.confidence <= 1.0


def test_boxes_are_inside_the_image(detector, image):
    h, w = image.shape[:2]
    for d in detector.detect(image):
        x0, y0, x1, y1 = d.box
        assert 0 <= x0 < x1 <= w and 0 <= y0 < y1 <= h


def test_masks_have_the_image_shape(detector, image):
    for d in detector.detect(image):
        if d.mask is not None:
            assert d.mask.shape == image.shape[:2] and d.mask.dtype == np.bool_


def test_the_input_image_is_unchanged(detector, image):
    before = image.copy()
    detector.detect(image)
    detector.detect(image)
    assert np.array_equal(image, before)


def test_the_output_passes_check_detections(detector, image):
    check_detections(detector.detect(image), image.shape, vocabulary.LABELS)


def test_thresholds_are_ordered(detector):
    t = detector.thresholds
    assert 0.0 <= t.sighting <= t.confirm <= t.confirm_unsupported <= 1.0
    check_thresholds(t, vocabulary.LABELS)


def test_overrides_only_for_vocabulary_labels(detector):
    assert set(detector.thresholds.by_label) <= vocabulary.LABELS


# --- the protocol module ---------------------------------------------------------------------------------------

def test_yoloe_thresholds_are_the_spec_values():
    assert (YOLOE_THRESHOLDS.sighting, YOLOE_THRESHOLDS.confirm, YOLOE_THRESHOLDS.confirm_unsupported) == \
        (0.25, 0.35, 0.5)
    assert dict(YOLOE_THRESHOLDS.by_label) == {}


def test_confirm_for_uses_the_override():
    t = Thresholds(0.25, 0.35, 0.5, by_label={"chair": 0.6})
    assert (t.confirm_for("chair"), t.confirm_for("box")) == (0.6, 0.35)


def ok_mask():
    return np.zeros(IMAGE_SHAPE[:2], dtype=bool)


@pytest.mark.parametrize("dets", [
    pytest.param((Detection("chair", 0.5, (1.0, 1.0, 2.0, 2.0)),), id="a-tuple"),
    pytest.param(["chair"], id="not-a-detection"),
    pytest.param([Detection("unicorn", 0.5, (1.0, 1.0, 2.0, 2.0))], id="label-outside"),
    pytest.param([Detection("person", 0.5, (1.0, 1.0, 2.0, 2.0))], id="ignored-prompt"),
    pytest.param([Detection("chair", 1.2, (1.0, 1.0, 2.0, 2.0))], id="confidence-above-1"),
    pytest.param([Detection("chair", -0.1, (1.0, 1.0, 2.0, 2.0))], id="confidence-below-0"),
    pytest.param([Detection("chair", True, (1.0, 1.0, 2.0, 2.0))], id="confidence-bool"),
    pytest.param([Detection("chair", float("nan"), (1.0, 1.0, 2.0, 2.0))], id="confidence-nan"),
    pytest.param([Detection("chair", 0.5, (2.0, 1.0, 2.0, 2.0))], id="x0-not-below-x1"),
    pytest.param([Detection("chair", 0.5, (1.0, 3.0, 2.0, 2.0))], id="y0-above-y1"),
    pytest.param([Detection("chair", 0.5, (-1.0, 1.0, 2.0, 2.0))], id="left-of-the-image"),
    pytest.param([Detection("chair", 0.5, (1.0, 1.0, 641.0, 2.0))], id="right-of-the-image"),
    pytest.param([Detection("chair", 0.5, (1.0, 1.0, 2.0, 481.0))], id="below-the-image"),
    pytest.param([Detection("chair", 0.5, (1.0, 1.0, 2.0))], id="three-numbers"),
    pytest.param([Detection("chair", 0.5, (1.0, 1.0, 2.0, float("inf")))], id="infinite"),
    pytest.param([Detection("chair", 0.5, (1.0, 1.0, 2.0, 2.0), np.zeros((240, 320), dtype=bool))],
                 id="mask-shape"),
    pytest.param([Detection("chair", 0.5, (1.0, 1.0, 2.0, 2.0), np.zeros(IMAGE_SHAPE[:2], dtype=np.uint8))],
                 id="mask-not-bool"),
])
def test_check_detections_rejects_each_breach(dets):
    with pytest.raises(ValueError):
        check_detections(dets, IMAGE_SHAPE, vocabulary.LABELS)


def test_check_detections_accepts_numpy_numbers_and_edges():
    dets = [Detection("chair", np.float32(0.5), (np.float32(0.0), 0, 640, 480), ok_mask()),
            Detection("box", 0.0, (10.0, 10.0, 10.5, 10.5)), Detection("sofa", 1.0, (0.0, 0.0, 1.0, 1.0))]
    check_detections(dets, IMAGE_SHAPE, vocabulary.LABELS)
    check_detections([], IMAGE_SHAPE, vocabulary.LABELS)


@pytest.mark.parametrize("thresholds", [
    pytest.param(Thresholds(0.4, 0.35, 0.5), id="sighting-above-confirm"),
    pytest.param(Thresholds(0.25, 0.6, 0.5), id="confirm-above-unsupported"),
    pytest.param(Thresholds(-0.1, 0.35, 0.5), id="below-0"),
    pytest.param(Thresholds(0.25, 0.35, 1.5), id="above-1"),
    pytest.param(Thresholds(0.25, 0.35, float("nan")), id="nan"),
    pytest.param(Thresholds(0.25, 0.35, 0.5, by_label={"unicorn": 0.5}), id="override-outside"),
    pytest.param(Thresholds(0.25, 0.35, 0.5, by_label={"chair": 0.2}), id="override-below-sighting"),
    pytest.param(Thresholds(0.25, 0.35, 0.5, by_label={"chair": 1.1}), id="override-above-1"),
])
def test_check_thresholds_rejects_disorder_and_unknown_labels(thresholds):
    with pytest.raises(ValueError):
        check_thresholds(thresholds, vocabulary.LABELS)


def test_vocabulary_prompts_are_the_detect_prompts_and_the_ignore_list():
    prompts, ignore = vocabulary_prompts()
    assert set(prompts) == vocabulary.LABELS
    assert prompts["chair"] == ("chair", "office chair")
    assert sum(len(v) for v in prompts.values()) == len(vocabulary.DETECT) == 87
    assert ignore == ("person", "dog", "cat")


def test_load_detector_rejects_unknown_names(monkeypatch):
    imported = []
    monkeypatch.setattr(protocol, "_import", lambda name: imported.append(name))
    with pytest.raises(ValueError, match="nope"):
        load_detector("nope")
    monkeypatch.setenv("DETECTOR", "nope")
    with pytest.raises(ValueError, match="yoloe"):
        load_detector()
    assert imported == []                               # nothing is imported for a bad name


@pytest.mark.parametrize("name, module", [("yoloe", "detector.yoloe"), ("oracle", "detector.oracle"),
                                          ("yoloe_pf", "detector.yoloe_pf")])
def test_load_detector_imports_lazily(monkeypatch, name, module):
    class Fake:
        def __getattr__(self, attr):
            return lambda: "made"

    imported = []
    monkeypatch.setattr(protocol, "_import", lambda m: imported.append(m) or Fake())
    assert load_detector(name) == "made"
    assert imported == [module]


def test_load_detector_defaults_to_yoloe(monkeypatch):
    imported = []

    class Fake:
        YoloeDetector = staticmethod(lambda: "yoloe")

    monkeypatch.setattr(protocol, "_import", lambda m: imported.append(m) or Fake())
    monkeypatch.delenv("DETECTOR", raising=False)
    assert load_detector() == "yoloe"
    monkeypatch.setenv("DETECTOR", "")
    assert load_detector() == "yoloe"
    assert imported == ["detector.yoloe", "detector.yoloe"]


def test_importing_the_protocol_imports_no_ml():
    code = ("import sys, detector.protocol, detector.scripted, detector.tracker\n"
            "bad = sorted(m for m in ('ultralytics', 'torch', 'torchvision', 'cv2') if m in sys.modules)\n"
            "assert not bad, bad\n")
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(ROOT), str(ROOT / "nl_control")]))
    done = subprocess.run([sys.executable, "-c", code], cwd=ROOT, env=env, capture_output=True, text=True,
                          timeout=60)
    assert done.returncode == 0, done.stderr[-2000:]


# --- the scripted detector (the shared test detector of Tasks 2.2, 2.7, 2.10 and 2.12) -----------------------

def test_scripted_returns_its_script_and_repeats_the_last():
    a = [Detection("chair", 0.6, (1.0, 1.0, 5.0, 5.0))]
    b = [Detection("box", 0.4, (2.0, 2.0, 6.0, 6.0))]
    det = ScriptedDetector([a, b])
    img = make_image()
    assert [det.detect(img), det.detect(img), det.detect(img)] == [a, b, b]
    assert det.calls == 3
    out = det.detect(img)
    out.append("x")
    assert det.detect(img) == b                         # a fresh list each call
    assert ScriptedDetector([]).detect(img) == []
    assert det.name == "scripted" and det.thresholds == YOLOE_THRESHOLDS


def test_scripted_calls_a_function_with_the_call_index():
    seen = []
    det = ScriptedDetector(lambda k, image: seen.append((k, image.shape)) or [])
    det.detect(make_image())
    det.detect(make_image())
    assert seen == [(0, IMAGE_SHAPE), (1, IMAGE_SHAPE)]


def test_scripted_fails_on_the_given_calls():
    det = ScriptedDetector([[]], fail_on=frozenset({1, 2}))
    img = make_image()
    assert det.detect(img) == []
    for _ in range(2):
        with pytest.raises(RuntimeError):
            det.detect(img)
    assert det.detect(img) == []
    assert det.calls == 4


def test_scripted_sleeps_first(monkeypatch):
    slept = []
    monkeypatch.setattr("detector.scripted.time.sleep", slept.append)
    det = ScriptedDetector([[]], delay_s=0.25, fail_on=frozenset({0}))
    with pytest.raises(RuntimeError):
        det.detect(make_image())
    assert slept == [0.25]
    ScriptedDetector([[]]).detect(make_image())
    assert slept == [0.25]


def test_scripted_records_the_vocabulary_and_thresholds():
    det = ScriptedDetector([[]], thresholds=Thresholds(0.2, 0.3, 0.4, by_label={"box": 0.5}))
    assert det.vocabulary is None
    det.set_vocabulary({"box": ["box", "cardboard box"]}, ["person"])
    assert det.vocabulary == ({"box": ("box", "cardboard box")}, ("person",))
    assert det.thresholds.confirm_for("box") == 0.5
    det.close()


def test_detections_with_masks_compare_without_the_mask():
    m = np.ones((2, 2), dtype=bool)
    assert Detection("box", 0.5, (0, 0, 1, 1), m) == Detection("box", 0.5, (0, 0, 1, 1), m.copy())
