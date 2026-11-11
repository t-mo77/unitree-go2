"""detector/yoloe.py: the YOLOE-11s text-prompt adapter behind the detector protocol (open-objects spec §11.1,
§11.10; design-detector §3.3, §3.4, §11.1, §16 "YOLOE adapter"; plan Task 2.11).

The unmarked tests run in the app image (no ultralytics, torch or cv2): the class list, the cache key, the
pure post-processing and the import. The `detector`-marked tests need the detector image (plan H2) and skip
elsewhere. This file never imports torch, ultralytics or cv2 (test_detector_image.test_licence_isolation): the
marked tests read the thread counts through threads() and the embedding path through embedding_source.
"""

import hashlib
import importlib.util
import itertools
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from voice_ltl import vocabulary
from detector import yoloe
from detector.protocol import YOLOE_THRESHOLDS, Thresholds, check_thresholds, vocabulary_prompts
from detector.yoloe import class_list, embedding_key, masks_to_image, to_detections

ROOT = Path(__file__).resolve().parent.parent
HAS_ULTRALYTICS = importlib.util.find_spec("ultralytics") is not None
SHA = "8e439445c87338b79d9ce21dec109f4621e26df67e94d26ea1a98c1e64dce3e3"     # yoloe-11s-seg.pt (spec §11.1)


# --- the class list (unmarked) ---------------------------------------------------------------------------------

def test_class_list_puts_ignore_last_and_maps_labels():
    names, labels = class_list({"chair": ["chair", "office chair"], "box": ("cardboard box",)}, ["person", "dog"])
    assert names == ["chair", "office chair", "cardboard box", "person", "dog"]
    assert labels == ["chair", "chair", "box", None, None]

    prompts, ignore = vocabulary_prompts()                   # the real vocabulary: 87 prompts + 3 ignored
    names, labels = class_list(prompts, ignore)
    assert len(names) == len(labels) == len(vocabulary.DETECT) + 3 == 90
    assert names[-3:] == ["person", "dog", "cat"] and labels[-3:] == [None, None, None]
    assert None not in labels[:-3] and set(labels[:-3]) == vocabulary.LABELS
    assert [n for n, lab in zip(names, labels) if lab == "chair"] == ["chair", "office chair"]
    assert names[:-3] == [p for label in prompts for p in prompts[label]]     # the vocabulary's order
    assert len(set(names)) == len(names)


@pytest.mark.parametrize("prompts, ignore", [
    pytest.param({"chair": []}, [], id="a-label-without-prompts"),
    pytest.param({"chair": "chair"}, [], id="a-string-not-a-list"),
    pytest.param({"chair": ["chair"]}, "person", id="ignore-a-string"),
    pytest.param({"chair": ["chair", "chair"]}, [], id="a-prompt-twice-in-a-label"),
    pytest.param({"chair": ["seat"], "sofa": ["seat"]}, [], id="a-prompt-of-two-labels"),
    pytest.param({"chair": ["person"]}, ["person"], id="a-prompt-also-ignored"),
    pytest.param({"chair": ["chair"]}, ["dog", "dog"], id="an-ignore-prompt-twice"),
    pytest.param({"chair": [""]}, [], id="an-empty-prompt"),
    pytest.param({"chair": [" "]}, [], id="a-blank-prompt"),
    pytest.param({"chair": [3]}, [], id="not-text"),
    pytest.param({}, [], id="nothing"),
])
def test_class_list_rejects_what_the_model_cannot_take(prompts, ignore):
    with pytest.raises(ValueError):
        class_list(prompts, ignore)


def test_embedding_key_changes_with_the_list_and_order():
    names = ["chair", "office chair", "box", "person"]
    key = embedding_key(SHA, names)
    assert key == hashlib.sha256(json.dumps([SHA, names]).encode("utf-8")).hexdigest()
    assert len(key) == 64 and int(key, 16) >= 0
    assert embedding_key(SHA, tuple(names)) == key                              # any sequence, same key
    assert embedding_key(SHA, ["chair", "office chair", "box"]) != key          # ignore prompts count
    assert embedding_key(SHA, ["office chair", "chair", "box", "person"]) != key    # the order counts
    assert embedding_key(SHA, names + ["sofa"]) != key
    assert embedding_key("0" * 64, names) != key                                # another model file
    every = {embedding_key(SHA, list(p)) for p in itertools.permutations(names)}
    assert len(every) == 24


# --- the post-processing (unmarked: plain arrays, as the adapter takes them out of ultralytics' result) --------

LABELS = ["chair", "chair", "box", None]               # class_list({"chair": [2 prompts], "box": [1]}, [1 ignore])


def test_to_detections_merges_prompts_and_drops_ignore_after_nms():
    cls = np.array([1.0, 3.0, 0.0, 2.0], dtype=np.float32)     # "office chair", "person", "chair", "box"
    conf = np.array([0.91, 0.8, 0.4, 0.12], dtype=np.float32)
    xyxy = np.array([[10, 20, 110, 220], [300, 0, 400, 470], [500, 100, 600, 200], [0, 0, 50, 40]],
                    dtype=np.float32)
    dets = to_detections(cls, conf, xyxy, None, LABELS, (480, 640))
    assert [d.label for d in dets] == ["chair", "chair", "box"]                 # model order, person gone
    assert [round(d.confidence, 6) for d in dets] == [0.91, 0.4, 0.12]
    assert dets[0].box == (10.0, 20.0, 110.0, 220.0)
    for d in dets:
        assert type(d.confidence) is float and all(type(v) is float for v in d.box) and d.mask is None
    assert to_detections(np.zeros(0), np.zeros(0), np.zeros((0, 4)), None, LABELS, (480, 640)) == []


def test_to_detections_clips_boxes_and_drops_what_is_left_of_nothing():
    cls = np.array([0, 0, 2, 2, 7, 0])
    conf = np.array([1.2, 0.5, 0.5, float("nan"), 0.5, -0.1])
    xyxy = np.array([[-5, -3, 700, 500],                        # clipped to the image
                     [640, 10, 660, 20],                        # nothing left inside
                     [10, 10, 10, 30],                          # zero width
                     [10, 10, 20, 30],                          # nan confidence
                     [10, 10, 20, 30],                          # a class index outside the list
                     [10, 10, 20, 30]], dtype=np.float64)      # confidence below 0: clipped to 0
    dets = to_detections(cls, conf, xyxy, None, LABELS, (480, 640))
    assert [(d.label, d.confidence, d.box) for d in dets] == [
        ("chair", 1.0, (0.0, 0.0, 640.0, 480.0)), ("chair", 0.0, (10.0, 10.0, 20.0, 30.0))]


def test_to_detections_masks_have_the_image_shape():
    masks = np.zeros((2, 480, 640), dtype=np.uint8)            # ultralytics' 0/1 masks at the letterboxed size
    masks[0, 100:200, 50:150] = 1
    dets = to_detections(np.array([0, 3]), np.array([0.6, 0.9]), np.array([[50, 100, 150, 200], [0, 0, 9, 9]]),
                         masks, LABELS, (480, 640))
    assert len(dets) == 1
    m = dets[0].mask
    assert m.dtype == np.bool_ and m.shape == (480, 640) and int(m.sum()) == 100 * 100
    assert m[150, 100] and not m[50, 100]
    # a mask stack that does not match the boxes is left out rather than misassigned
    wrong = to_detections(np.array([0]), np.array([0.6]), np.array([[50, 100, 150, 200]]), masks, LABELS, (480, 640))
    assert wrong[0].mask is None


def test_masks_follow_the_letterbox_back_to_the_image():
    # a 300 x 500 (H x W) image at imgsz 640: gain 1.28, content 384 x 640, padded to 384 (a stride multiple)
    same = np.zeros((1, 384, 640), dtype=np.uint8)
    same[0, 128:256, 320:640] = 1                              # image rows 100-200, the right half
    out = masks_to_image(same, (300, 500))
    assert out.shape == (1, 300, 500) and out.dtype == np.bool_
    assert out[0, 150, 400] and not out[0, 150, 100] and not out[0, 50, 400]
    assert abs(int(out.sum()) - 100 * 250) <= 2 * (100 + 250)

    # a 480 x 640 image letterboxed into 640 x 640 (padding 80 rows above and below)
    padded = np.zeros((1, 640, 640), dtype=np.uint8)
    padded[0, 80:560, 0:320] = 1                                # the left half of the content
    out = masks_to_image(padded, (480, 640))
    assert out.shape == (1, 480, 640) and bool(out[0, :, :320].all()) and not out[0, :, 320:].any()

    exact = np.ones((2, 480, 640), dtype=np.float32)
    assert masks_to_image(exact, (480, 640)).all()
    assert masks_to_image(np.zeros((0, 480, 640), dtype=np.uint8), (480, 640)).shape == (0, 480, 640)


# --- the module (unmarked) -------------------------------------------------------------------------------------

def test_module_imports_without_ultralytics():
    code = ("import sys, detector.yoloe, detector.protocol\n"
            "bad = sorted(m for m in ('ultralytics', 'torch', 'torchvision', 'cv2') if m in sys.modules)\n"
            "assert not bad, bad\n"
            "print(detector.protocol.DETECTORS['yoloe'])\n")
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(ROOT), str(ROOT / "nl_control")]))
    done = subprocess.run([sys.executable, "-c", code], cwd=ROOT, env=env, capture_output=True, text=True,
                          timeout=60)
    assert done.returncode == 0, done.stderr[-2000:]
    assert done.stdout.strip() == "('detector.yoloe', 'YoloeDetector')"

    assert yoloe.MODEL_PATH == "/opt/yoloe/yoloe-11s-seg.pt" and yoloe.CACHE_DIR == "/cache"
    assert yoloe.DET_TORCH_THREADS == 1
    assert yoloe.PREDICT == {"conf": 0.10, "iou": 0.7, "agnostic_nms": True, "imgsz": 640}
    assert yoloe.BY_LABEL == {}
    assert yoloe.YoloeDetector.name == "yoloe-11s-seg"
    t = yoloe.YoloeDetector.thresholds
    assert t == Thresholds(0.25, 0.35, 0.5, {}) == YOLOE_THRESHOLDS and dict(t.by_label) == yoloe.BY_LABEL
    check_thresholds(t, vocabulary.LABELS)                      # H3's overrides must stay vocabulary labels


@pytest.mark.skipif(HAS_ULTRALYTICS, reason="the app image only")
def test_making_it_without_ultralytics_names_the_detector_image():
    with pytest.raises(ImportError, match="detector image"):
        yoloe.YoloeDetector()


# --- the real model (`detector`: the detector image, plan H2) --------------------------------------------------

def sample(name: str) -> np.ndarray:
    """One of ultralytics' bundled sample pictures as H x W x 3 uint8 RGB (bus.jpg: 4 people and a bus; zidane.jpg:
    2 people). Read with PIL (ultralytics needs it), so this file imports neither ultralytics nor cv2."""
    from PIL import Image

    folder = importlib.util.find_spec("ultralytics").submodule_search_locations[0]
    with Image.open(os.path.join(folder, "assets", name)) as im:
        return np.asarray(im.convert("RGB"), dtype=np.uint8).copy()


def iou(a, b) -> float:
    w = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    h = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = w * h
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def raw_rows(det, image):
    """(class index, confidence, box) for every class-agnostic NMS survivor, ignored classes included."""
    cls, conf, xyxy, _masks = det._predict(image)
    return [(int(c), float(s), tuple(float(v) for v in b)) for c, s, b in zip(cls, conf, xyxy)]


@pytest.fixture
def cache(tmp_path):
    return str(tmp_path / "cache")


@pytest.mark.detector
@pytest.mark.skipif(not HAS_ULTRALYTICS, reason="needs the detector image (ultralytics)")
def test_prompts_of_one_label_merge(cache):
    det = yoloe.YoloeDetector(cache_dir=cache)
    try:
        det.set_vocabulary({"chair": ("person", "man"), "table": ("table",)}, ())   # people answer to "chair" here
        image = sample("zidane.jpg")
        raw = raw_rows(det, image)
        dets = det.detect(image)
        assert any(c in (0, 1) and s >= 0.5 for c, s, _ in raw)                  # the people are seen
        assert [d.label for d in dets] == [("chair" if c in (0, 1) else "table") for c, _, _ in raw]
        chairs = [d for d in dets if d.label == "chair"]
        assert chairs and all(iou(a.box, b.box) <= 0.75 for a, b in itertools.combinations(chairs, 2))
    finally:
        det.close()


@pytest.mark.detector
@pytest.mark.skipif(not HAS_ULTRALYTICS, reason="needs the detector image (ultralytics)")
def test_ignore_classes_are_dropped_and_still_suppress(cache):
    det = yoloe.YoloeDetector(cache_dir=cache)
    try:
        prompts, ignore = vocabulary_prompts()
        det.set_vocabulary(prompts, ignore)
        n = len(class_list(prompts, ignore)[0])
        ignored = set(range(n - len(ignore), n))
        people = 0
        for name in ("zidane.jpg", "bus.jpg"):
            image = sample(name)
            raw = raw_rows(det, image)
            dets = det.detect(image)
            person_boxes = [b for c, s, b in raw if c in ignored and s >= 0.25]
            people += len(person_boxes)
            assert len(dets) == sum(c not in ignored for c, _, _ in raw)         # exactly the ignored rows go
            assert all(d.label in vocabulary.LABELS for d in dets)
            for d in dets:                                                      # nothing kept on a person
                assert all(iou(d.box, b) <= 0.75 for b in person_boxes), (d, person_boxes)
        assert people >= 2
    finally:
        det.close()


@pytest.mark.detector
@pytest.mark.skipif(not HAS_ULTRALYTICS, reason="needs the detector image (ultralytics)")
def test_agnostic_nms_is_on(cache):
    det = yoloe.YoloeDetector(cache_dir=cache)
    try:
        det.set_vocabulary(*vocabulary_prompts())
        seen = []
        predict = det._model.predict

        def spy(*args, **kwargs):
            seen.append(kwargs)
            return predict(*args, **kwargs)

        det._model.predict = spy
        for name in ("zidane.jpg", "bus.jpg"):
            raw = raw_rows(det, sample(name))
            # every survivor, of any class (ignored ones too), overlaps every other by at most the NMS IoU
            # (+ slack for the clipping to the image after NMS)
            assert all(iou(a[2], b[2]) <= 0.75 for a, b in itertools.combinations(raw, 2))
        assert seen and all(k.get("agnostic_nms") is True and k.get("iou") == 0.7 and k.get("conf") == 0.10
                            and k.get("imgsz") == 640 for k in seen)
    finally:
        det.close()


@pytest.mark.detector
@pytest.mark.skipif(not HAS_ULTRALYTICS, reason="needs the detector image (ultralytics)")
def test_threads_are_1_after_warm_up(cache):
    det = yoloe.YoloeDetector(cache_dir=cache)
    try:
        det.set_vocabulary(*vocabulary_prompts())                  # the warm-up predict runs here
        assert det.threads() == {"torch": 1, "interop": 1, "cv2": 1}
        det.detect(sample("bus.jpg"))
        det.detect(np.zeros((480, 640, 3), dtype=np.uint8))
        assert det.threads() == {"torch": 1, "interop": 1, "cv2": 1}
    finally:
        det.close()


def spy_encoder(det) -> list:
    """Record every text-encoder use of det's model (both get_text_pe levels; the inner one builds
    mobileclip_blt.ts), still running it."""
    calls = []
    for owner in (det._model, det._model.model):
        original = owner.get_text_pe

        def spy(*args, _original=original, _owner=type(owner).__name__, **kwargs):
            calls.append(_owner)
            return _original(*args, **kwargs)

        owner.get_text_pe = spy
    return calls


@pytest.mark.detector
@pytest.mark.skipif(not HAS_ULTRALYTICS, reason="needs the detector image (ultralytics)")
def test_embedding_cache_skips_the_encoder(tmp_path):
    cache = tmp_path / "cache"
    prompts, ignore = vocabulary_prompts()
    names = class_list(prompts, ignore)[0]

    first = yoloe.YoloeDetector(cache_dir=str(cache))
    assert first.embedding_source is None
    first.set_vocabulary(prompts, ignore)
    assert first.embedding_source == "encoder"
    image = sample("bus.jpg")
    expected = [(d.label, round(d.confidence, 4), tuple(round(v, 1) for v in d.box)) for d in first.detect(image)]
    first.close()
    files = sorted(p.name for p in cache.glob("yoloe-pe-*.npz"))
    assert len(files) == 1 and files[0].startswith("yoloe-pe-") and len(files[0]) == len("yoloe-pe-.npz") + 64

    second = yoloe.YoloeDetector(cache_dir=str(cache))
    calls = spy_encoder(second)
    second.set_vocabulary(prompts, ignore)
    assert second.embedding_source == "cache" and calls == []           # mobileclip_blt.ts never loaded
    got = [(d.label, round(d.confidence, 4), tuple(round(v, 1) for v in d.box)) for d in second.detect(image)]
    assert got == expected                                              # the same embeddings
    second.close()
    assert sorted(p.name for p in cache.glob("yoloe-pe-*.npz")) == files

    changed = dict(reversed(list(prompts.items())))                     # the same prompts, another order
    assert class_list(changed, ignore)[0] != names
    third = yoloe.YoloeDetector(cache_dir=str(cache))
    calls = spy_encoder(third)
    third.set_vocabulary(changed, ignore)
    assert third.embedding_source == "encoder" and calls
    third.close()
    assert len(list(cache.glob("yoloe-pe-*.npz"))) == 2
