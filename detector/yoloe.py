"""The YOLOE-11s text-prompt adapter behind the detector protocol (open-objects spec §11.1, §11.10; design-detector
§3.3, §3.4, §11.1, §11.2).

This file (with any later detector/yoloe*.py) is the only one that imports ultralytics, torch or cv2 (D5). They
live in the detector image only (go2-voice-ltl-detector:yoloe-11s): ultralytics, the YOLOE weights and the CLIP
tokenizer fork are AGPL-3.0, and the text encoder mobileclip_blt.ts is Apple's MobileCLIP under the Apple ML
Research Model TOU (Research Purposes only; spec §11.2). Every such import sits inside a function, so the module
imports anywhere (the app image's tests, load_detector's table) and needs them only when a YoloeDetector is made.

- Classes: every prompt of every label, then every ignore prompt (person, dog, cat), as one class list; class
  index -> label, None for an ignore prompt. An open-vocabulary model puts its nearest listed name on a thing left
  out of the list, so a person stays in it, and its box is dropped only after the class-agnostic NMS: it still
  suppresses a "chair" on the person. Prompts of one label merge into that label.
- Text embeddings are computed once per model file and class list and kept in <cache_dir>/yoloe-pe-<key>.npz
  (the named volume detector-cache:/cache), key = embedding_key(sha256 of the model file, class list). A start
  with the same list never loads the 600 MB text encoder. ultralytics' save/load_prompt_embeddings write and read
  the file; if they are missing or fail, the same three arrays are written and read here and the tensor passed
  as set_classes(names, embeddings). A changed vocabulary "detect" changes the key: the next start computes them
  again (no image rebuild). embedding_source says which path made them ("cache" or "encoder").
- Threads: interop 1 before any parallel work (once per process); ultralytics sets torch to min(8, cpus - 1)
  threads when it builds its predictor, on the first predict after set_classes, so set_vocabulary runs one
  warm-up predict on a black camera-size frame and then sets torch to DET_TORCH_THREADS; cv2 1.
- detect: RGB -> BGR (ultralytics reads numpy images as BGR) into a copy, predict(**PREDICT), boxes clipped to the
  image, masks from the letterboxed input size back to the image size, ignore classes dropped.
"""

from __future__ import annotations

import hashlib
import json
import logging
import math
import os
import time
from collections.abc import Mapping, Sequence
from pathlib import Path

import numpy as np

from detector.camera import CAMERA_H, CAMERA_W
from detector.protocol import YOLOE_THRESHOLDS, Detection, Thresholds
from voice_ltl import vocabulary

log = logging.getLogger(__name__)

MODEL_PATH = "/opt/yoloe/yoloe-11s-seg.pt"
CACHE_DIR = "/cache"
DET_TORCH_THREADS = 1
PREDICT = {"conf": 0.10, "iou": 0.7, "agnostic_nms": True, "imgsz": 640}
BY_LABEL: dict[str, float] = {}                       # per-label confirm overrides, set by H3 only
CACHE_PREFIX = "yoloe-pe-"
DETECTOR_IMAGE = "go2-voice-ltl-detector:yoloe-11s"


# --- pure parts (numpy and the standard library; tested in the app image) ----------------------------------------

def class_list(prompts: Mapping[str, Sequence[str]], ignore: Sequence[str]) -> tuple[list[str], list[str | None]]:
    """(names, labels): every prompt (labels in order, each label's prompts in order), then every ignore prompt;
    labels[i] is the label of names[i], None for an ignore prompt. Raises ValueError for what the model cannot take:
    a label without prompts, a string where a list belongs, a blank or non-text prompt, a prompt given twice
    (within a label, across labels or also ignored), or no prompt at all."""
    names: list[str] = []
    labels: list[str | None] = []
    for label, texts in prompts.items():
        if not isinstance(label, str) or not label:
            raise ValueError(f"label {label!r} is not a name")
        if isinstance(texts, (str, bytes)):
            raise ValueError(f"prompts of {label}: a list of prompts, not the string {texts!r}")
        texts = list(texts)
        if not texts:
            raise ValueError(f"prompts of {label}: none given")
        for text in texts:
            names.append(_prompt(text, label))
            labels.append(label)
    if isinstance(ignore, (str, bytes)):
        raise ValueError(f"ignore: a list of prompts, not the string {ignore!r}")
    for text in ignore:
        names.append(_prompt(text, "ignore"))
        labels.append(None)
    if not names:
        raise ValueError("no prompts: the model needs at least one class")
    owner: dict[str, str] = {}
    for name, label in zip(names, labels):
        who = label if label is not None else "ignore"
        if name in owner:
            raise ValueError(f"prompt {name!r} is given twice ({owner[name]} and {who})")
        owner[name] = who
    return names, labels


def _prompt(text: object, owner: str) -> str:
    if not isinstance(text, str) or not text.strip():
        raise ValueError(f"prompts of {owner}: {text!r} is not a prompt (non-blank text)")
    return text


def embedding_key(model_sha256: str, names: Sequence[str]) -> str:
    """sha256 (hex) of json([model file sha256, names in order]); the ignore prompts are part of names."""
    return hashlib.sha256(json.dumps([model_sha256, list(names)]).encode("utf-8")).hexdigest()


def file_sha256(path: str | os.PathLike) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def masks_to_image(masks: np.ndarray, image_shape: Sequence[int]) -> np.ndarray:
    """N x h x w masks at the model's letterboxed input size -> N x H x W bool at the image size: the letterbox
    padding is cut off (ultralytics' centred padding, as its scale_masks), then nearest-neighbour resampling."""
    m = np.asarray(masks)
    if m.ndim != 3:
        raise ValueError(f"masks of shape {m.shape} are not N x h x w")
    height, width = int(image_shape[0]), int(image_shape[1])
    h, w = m.shape[1:]
    if (h, w) != (height, width):
        gain = min(h / height, w / width)
        ch, cw = max(round(height * gain), 1), max(round(width * gain), 1)      # the content inside the padding
        top, left = max(round((h - ch) / 2 - 0.1), 0), max(round((w - cw) / 2 - 0.1), 0)
        m = m[:, top:top + ch, left:left + cw]
        ys = np.minimum(((np.arange(height) + 0.5) * (m.shape[1] / height)).astype(np.intp), m.shape[1] - 1)
        xs = np.minimum(((np.arange(width) + 0.5) * (m.shape[2] / width)).astype(np.intp), m.shape[2] - 1)
        m = m[:, ys][:, :, xs]
    return m > 0.5


def to_detections(cls: np.ndarray, conf: np.ndarray, xyxy: np.ndarray, masks: np.ndarray | None,
                  labels: Sequence[str | None], image_shape: Sequence[int]) -> list[Detection]:
    """The detections of one result (NMS already done, every class in it): in the model's order, each class index
    mapped to its label, ignore classes (label None) and unknown indices dropped, confidences clipped to [0, 1]
    (a non-finite one drops the row), boxes clipped to the image (one with nothing left drops), masks at the image
    size (none when the mask stack does not match the rows)."""
    cls = np.asarray(cls, dtype=np.float64).reshape(-1)
    conf = np.asarray(conf, dtype=np.float64).reshape(-1)
    xyxy = np.asarray(xyxy, dtype=np.float64).reshape(-1, 4)
    n = len(cls)
    if len(conf) != n or len(xyxy) != n:
        raise ValueError(f"{n} classes, {len(conf)} confidences and {len(xyxy)} boxes do not match")
    if masks is not None:
        masks = np.asarray(masks)
        if masks.ndim != 3 or len(masks) != n:
            log.warning("a mask stack of shape %s for %d boxes is left out", masks.shape, n)
            masks = None
    height, width = int(image_shape[0]), int(image_shape[1])
    out = []
    for i in range(n):
        c = cls[i]
        if not (math.isfinite(c) and c == int(c) and 0 <= int(c) < len(labels)):
            continue
        label = labels[int(c)]
        if label is None:                                   # an ignore prompt: it took part in the NMS, now it goes
            continue
        score = float(conf[i])
        if not math.isfinite(score):
            continue
        x0, y0, x1, y1 = (float(v) for v in xyxy[i])
        if not all(math.isfinite(v) for v in (x0, y0, x1, y1)):
            continue
        x0, x1 = (min(max(v, 0.0), float(width)) for v in (x0, x1))
        y0, y1 = (min(max(v, 0.0), float(height)) for v in (y0, y1))
        if not (x0 < x1 and y0 < y1):
            continue
        mask = None if masks is None else masks_to_image(masks[i:i + 1], (height, width))[0]
        out.append(Detection(label, min(max(score, 0.0), 1.0), (x0, y0, x1, y1), mask))
    return out


def _numpy(value) -> np.ndarray:
    """A torch tensor (or anything array-like) as a numpy array."""
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        return value.numpy()
    return np.asarray(value)


# --- the ML side (the detector image) -----------------------------------------------------------------------------

def _load_ml():
    """torch, cv2 and ultralytics' YOLOE, imported only here (D5). torch's interop threads are set to 1 right
    after torch is imported, before anything can start parallel work."""
    try:
        import torch
    except ImportError as e:
        raise ImportError(f"YoloeDetector needs the detector image {DETECTOR_IMAGE} (torch, ultralytics, cv2): {e}"
                          ) from e
    try:
        torch.set_num_interop_threads(1)
    except RuntimeError as e:                          # allowed once per process, before any parallel work
        if torch.get_num_interop_threads() != 1:
            log.warning("torch interop threads stay at %d: %s", torch.get_num_interop_threads(), e)
    try:
        import cv2
        from ultralytics import YOLOE
    except ImportError as e:
        raise ImportError(f"YoloeDetector needs the detector image {DETECTOR_IMAGE} (torch, ultralytics, cv2): {e}"
                          ) from e
    cv2.setNumThreads(1)
    return torch, cv2, YOLOE


class YoloeDetector:
    """YOLOE-11s-seg with text prompts, on the CPU (spec §11.10)."""

    name = "yoloe-11s-seg"
    thresholds = Thresholds(YOLOE_THRESHOLDS.sighting, YOLOE_THRESHOLDS.confirm,
                            YOLOE_THRESHOLDS.confirm_unsupported, dict(BY_LABEL))

    def __init__(self, model_path: str = MODEL_PATH, cache_dir: str = CACHE_DIR) -> None:
        self._torch, self._cv2, yoloe_class = _load_ml()
        path = Path(model_path)
        if not path.is_file():
            raise FileNotFoundError(f"YOLOE weights {model_path} not found (the detector image has them in "
                                    f"{Path(MODEL_PATH).parent})")
        self.model_path = str(path)
        self.cache_dir = Path(cache_dir)
        self.embedding_source: str | None = None
        self._labels: list[str | None] | None = None
        self._model_sha256 = file_sha256(path)
        started = time.monotonic()
        self._model = yoloe_class(self.model_path)
        log.info("YOLOE model %s loaded in %.2f s", path.name, time.monotonic() - started)

    # -- the protocol --

    def set_vocabulary(self, prompts: Mapping[str, Sequence[str]], ignore: Sequence[str]) -> None:
        """One class per prompt (ignore prompts last); the text embeddings from the cache, else from the encoder
        (then cached); one warm-up predict; then the thread counts. Labels must be vocabulary labels."""
        self._require()
        names, labels = class_list(prompts, ignore)
        outside = sorted({label for label in labels if label is not None} - vocabulary.LABELS)
        if outside:
            raise ValueError(f"not vocabulary labels: {', '.join(outside)}")
        path = self.cache_dir / f"{CACHE_PREFIX}{embedding_key(self._model_sha256, names)}.npz"
        started = time.monotonic()
        if path.is_file() and self._load_embeddings(path, names):
            source = "cache"
        else:
            self._encode(names)
            source = "encoder"
            self._save_embeddings(path, names)
        log.info("text embeddings of %d classes (%d ignored) from the %s in %.2f s (%s)", len(names),
                 labels.count(None), source, time.monotonic() - started, path.name)
        self._labels = labels
        self.embedding_source = source
        self._warm_up()

    def detect(self, image_rgb: np.ndarray) -> list[Detection]:
        if self._labels is None:
            raise RuntimeError("YoloeDetector.detect before set_vocabulary")
        cls, conf, xyxy, masks = self._predict(image_rgb)
        return to_detections(cls, conf, xyxy, masks, self._labels, image_rgb.shape[:2])

    def close(self) -> None:
        self._model = None
        self._labels = None

    # -- status --

    def threads(self) -> dict[str, int]:
        return {"torch": int(self._torch.get_num_threads()), "interop": int(self._torch.get_num_interop_threads()),
                "cv2": int(self._cv2.getNumThreads())}

    # -- inside --

    def _require(self):
        if self._model is None:
            raise RuntimeError("YoloeDetector is closed")
        return self._model

    def _model_names(self) -> list[str]:
        names = self._require().names
        return [str(names[i]) for i in sorted(names)] if isinstance(names, dict) else [str(n) for n in names]

    def _load_embeddings(self, path: Path, names: list[str]) -> bool:
        """The cached embeddings: ultralytics' loader first, then the same file read here. False (logged) when
        neither gives exactly these classes; the caller then computes them."""
        model = self._require()
        loaders = []
        if hasattr(model, "load_prompt_embeddings"):
            loaders.append(("ultralytics", lambda: model.load_prompt_embeddings(str(path))))
        loaders.append(("npz", lambda: self._load_npz(path, names)))
        for how, load in loaders:
            try:
                load()
                if self._model_names() != names:
                    raise ValueError("its class names differ from the list")
                return True
            except Exception as e:                       # a broken or foreign file: try the next way, then compute
                log.warning("text embedding cache %s not loaded (%s): %s", path.name, how, e)
        return False

    def _load_npz(self, path: Path, names: list[str]) -> None:
        with np.load(path, allow_pickle=False) as data:
            embeddings, cached = data["embeddings"], [str(n) for n in data["names"]]
        if cached != names:
            raise ValueError("its class names differ from the list")
        if embeddings.dtype != np.float32 or embeddings.ndim != 3 or embeddings.shape[:2] != (1, len(names)):
            raise ValueError(f"embeddings {embeddings.dtype} {embeddings.shape} are not float32 1 x {len(names)} x D")
        self._require().set_classes(names, self._torch.from_numpy(np.ascontiguousarray(embeddings)))

    def _encode(self, names: list[str]) -> None:
        """The text encoder (mobileclip_blt.ts, about 600 MB; seconds, about 1.3 GB at its peak)."""
        model = self._require()
        model.set_classes(names, model.get_text_pe(names))

    def _save_embeddings(self, path: Path, names: list[str]) -> None:
        """Write the embeddings now set (ultralytics' writer, else the same arrays here) to a temporary file and
        move it into place. A failure is logged: the next start computes them again."""
        model = self._require()
        tmp = path.with_name(f"{path.stem}.{os.getpid()}.tmp.npz")
        writers = []
        if hasattr(model, "save_prompt_embeddings"):
            writers.append(("ultralytics", lambda: model.save_prompt_embeddings(str(tmp))))
        writers.append(("npz", lambda: self._save_npz(tmp, names)))
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
        except OSError as e:
            log.warning("text embeddings not cached: %s (%s)", path.parent, e)
            return
        for how, write in writers:
            try:
                write()
                os.replace(tmp, path)
                return
            except Exception as e:
                log.warning("text embeddings not cached in %s (%s): %s", path.name, how, e)
            finally:
                try:
                    tmp.unlink()
                except OSError:
                    pass

    def _save_npz(self, tmp: Path, names: list[str]) -> None:
        embeddings = _numpy(self._require().model.pe).astype(np.float32)
        model_id = Path(self.model_path).stem
        np.savez_compressed(tmp, embeddings=embeddings, names=np.asarray(names, dtype=np.str_),
                            model=np.asarray(model_id[:-4] if model_id.endswith("-seg") else model_id,
                                             dtype=np.str_))

    def _warm_up(self) -> None:
        """One predict on a black camera-size frame builds ultralytics' predictor (which sets torch's threads),
        then torch and cv2 go to their thread counts."""
        self._predict(np.zeros((CAMERA_H, CAMERA_W, 3), dtype=np.uint8))
        self._torch.set_num_threads(DET_TORCH_THREADS)
        self._cv2.setNumThreads(1)

    def _predict(self, image_rgb: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray | None]:
        """(class indices, confidences, boxes x0 y0 x1 y1 in image pixels, masks or None) of every class-agnostic
        NMS survivor, ignore classes included."""
        if not (isinstance(image_rgb, np.ndarray) and image_rgb.ndim == 3 and image_rgb.shape[2] == 3
                and image_rgb.dtype == np.uint8):
            raise ValueError(f"an image is H x W x 3 uint8 RGB, not {getattr(image_rgb, 'shape', None)} "
                             f"{getattr(image_rgb, 'dtype', type(image_rgb).__name__)}")
        bgr = np.ascontiguousarray(image_rgb[:, :, ::-1])          # a copy: the caller's image is never touched
        result = self._require().predict(bgr, verbose=False, **PREDICT)[0]
        if self._torch.get_num_threads() != DET_TORCH_THREADS:
            self._torch.set_num_threads(DET_TORCH_THREADS)          # building the predictor resets it
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return np.zeros(0), np.zeros(0), np.zeros((0, 4)), None
        masks = None if result.masks is None else _numpy(result.masks.data)
        return _numpy(boxes.cls), _numpy(boxes.conf), _numpy(boxes.xyxy), masks
