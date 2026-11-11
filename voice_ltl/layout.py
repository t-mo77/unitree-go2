"""物の置き方のファイル gazebo/layouts/<名前>.json を読み、確かめる（open-objects spec §12.5）。

Gazebo の橋渡し・偽の Go2・objects_check.py・oracle・言い方の試験が同じものを使う。標準ライブラリだけを使う
（Jazzy の Gazebo ブリッジと検出器も読み込む）。足跡はモデルの軸（モデルを測った値のまま）で書き、
世界の軸の大きさ・ずれ・中心は Entry の world_size / world_offset / centre の 1 か所で出す。
確かめに 1 つでも外れたら LayoutError（「<ファイル>: <id か ->: <値>: <わけ>」の 1 行。JSON の誤りは行と桁も）。
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

# Relative, so every absolute import here stays a standard-library one (test_layout checks those).
from . import objects, vocabulary

ROOT = Path(__file__).resolve().parent.parent
LAYOUTS_DIR = ROOT / "gazebo" / "layouts"
FUEL_MODELS = ROOT / "docker" / "fuel_models.json"
PRIMITIVES = {"primitive:box": "box", "primitive:chair": "chair"}
MODEL_PREFIX = "model://"
VERSION = 1
MAX_SIZE = objects.MAX_SIZE                     # sides and heights in (0, MAX_SIZE] m

_TOP_KEYS = ("version", "objects")
_KEYS = ("id", "model", "x", "y", "yaw_deg", "footprint", "height", "z", "detectable", "accept")
_REQUIRED = ("id", "model", "x", "y", "yaw_deg", "footprint", "height", "detectable", "accept")
_FOOTPRINT_KEYS = ("offset", "size")
_ID_SHAPE = re.compile(r"(.+)_([1-9][0-9]?)")    # the label is checked against the vocabulary
_FOLDER_SHAPE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*")
_QUARTER = ((1, 0), (0, 1), (-1, 0), (0, -1))    # (cos, sin) of 0, 90, 180, 270 deg, exactly


class LayoutError(ValueError):
    """str(e) == "<file>: <id or '-'>: <value>: <reason>" on one line."""

    def __init__(self, source: str, obj_id: str, value: object, reason: str):
        self.source, self.obj_id, self.value, self.reason = source, obj_id, value, reason
        super().__init__(f"{source}: {vocabulary._show(obj_id)}: {vocabulary._show(value)}: {reason}")


@dataclass(frozen=True)
class Entry:
    id: str
    label: str
    model: str                                  # "primitive:box", "primitive:chair" or "model://<folder>"
    x: float                                    # the model origin in the world (odom) [m]
    y: float
    yaw_deg: int                                # a multiple of 90
    offset: tuple[float, float]                 # model axes: origin -> footprint centre [m]
    size: tuple[float, float]                   # model axes (as measured on the model) [m]
    height: float
    z: float                                    # the spawn height of a model:// origin (0.0 for primitives)
    detectable: bool                            # counted in the detector's recall (spec §12.8)
    accept: tuple[str, ...]                     # labels taken as right, its own label included

    def world_size(self) -> tuple[float, float]:
        """The footprint along world x and y: the model's sides swapped at 90 and 270 deg."""
        return (self.size[1], self.size[0]) if self.yaw_deg % 180 else self.size

    def world_offset(self) -> tuple[float, float]:
        """The offset turned by yaw_deg (exact: only quarter turns)."""
        c, s = _QUARTER[(self.yaw_deg // 90) % 4]
        dx, dy = self.offset
        return (c * dx - s * dy, s * dx + c * dy)

    def centre(self) -> tuple[float, float]:
        """The footprint centre in the world: the origin + world_offset()."""
        dx, dy = self.world_offset()
        return (self.x + dx, self.y + dy)

    def obj(self) -> objects.Obj:
        return objects.Obj(self.id, self.label, *self.centre(), self.world_size())

    @property
    def folder(self) -> str | None:
        """The model folder ("Sofa" for model://Sofa); None for a primitive."""
        return self.model[len(MODEL_PREFIX):] if self.model.startswith(MODEL_PREFIX) else None


@dataclass(frozen=True)
class Layout:
    name: str
    entries: tuple[Entry, ...]

    def objs(self) -> tuple[objects.Obj, ...]:
        return tuple(e.obj() for e in self.entries)

    def find(self, obj_id: str) -> Entry | None:
        return next((e for e in self.entries if e.id == obj_id), None)


def names() -> tuple[str, ...]:
    """The layout files' stems, sorted (gazebo/layouts/*.json)."""
    try:
        return tuple(sorted(p.stem for p in LAYOUTS_DIR.glob("*.json") if p.is_file()))
    except OSError:
        return ()


def model_folders() -> frozenset[str]:
    """The model folders listed in docker/fuel_models.json (the models the Gazebo image carries)."""
    source = _display(FUEL_MODELS)
    data = _read_json(FUEL_MODELS, source)
    models = data.get("models") if isinstance(data, dict) else None
    if not isinstance(models, list):
        raise LayoutError(source, "-", "models", "the file needs a list \"models\"")
    folders = []
    for m in models:
        folder = m.get("folder") if isinstance(m, dict) else None
        if not (isinstance(folder, str) and _FOLDER_SHAPE.fullmatch(folder)):
            raise LayoutError(source, "-", folder if folder is not None else m, "each model needs a folder name")
        folders.append(folder)
    return frozenset(folders)


def parse(data: object, name: str, folders: frozenset[str] | None = None) -> Layout:
    """Check a layout file's data (spec §12.5) and build it. folders: the model:// folders allowed
    (None: model_folders(), read only when a model:// entry needs it)."""
    source = _display(LAYOUTS_DIR / f"{name}.json")
    if not isinstance(data, dict):
        raise LayoutError(source, "-", type(data).__name__, "a layout file holds one JSON object")
    for key in data:
        if key not in _TOP_KEYS:
            raise LayoutError(source, "-", key, f"unknown key (the keys are {', '.join(_TOP_KEYS)})")
    version = data.get("version")
    if not (isinstance(version, int) and not isinstance(version, bool) and version == VERSION):
        raise LayoutError(source, "-", f"version {vocabulary._show(version)}", f"version must be {VERSION}")
    rows = data.get("objects")
    if not isinstance(rows, list):
        raise LayoutError(source, "-", "objects", "\"objects\" must be a list")
    checker = _Checker(source, folders)
    entries: list[Entry] = []
    for row in rows:
        entry = checker.entry(row)
        if any(e.id == entry.id for e in entries):
            raise LayoutError(source, entry.id, entry.id, "the id appears twice")
        entries.append(entry)
    return Layout(name, tuple(entries))


def load(name: str) -> Layout:
    """Read gazebo/layouts/<name>.json. A name that is not one of names() raises with the valid names."""
    known = names()
    if not isinstance(name, str) or name not in known:
        shown = name if isinstance(name, str) else repr(name)
        raise LayoutError(f"{_display(LAYOUTS_DIR)}/{shown}.json", "-", shown,
                          f"no such layout (the layouts are {', '.join(known) or 'none'})")
    path = LAYOUTS_DIR / f"{name}.json"
    return parse(_read_json(path, _display(path)), name)


# ---- 内部 ----

class _Checker:
    """One entry's checks, in key order; the first failure raises."""

    def __init__(self, source: str, folders: frozenset[str] | None):
        self.source = source
        self.folders = folders

    def fail(self, obj_id: str, value: object, reason: str) -> None:
        raise LayoutError(self.source, obj_id, value, reason)

    def entry(self, row: object) -> Entry:
        if not isinstance(row, dict):
            self.fail("-", row, "each object is a JSON object")
        obj_id = row.get("id")
        label = self.label_of(obj_id)
        for key in row:
            if key not in _KEYS:
                self.fail(obj_id, key, f"unknown key (the keys are {', '.join(_KEYS)})")
        for key in _REQUIRED:
            if key not in row:
                self.fail(obj_id, key, "a required key is missing")
        model = self.model(obj_id, label, row["model"])
        x, y = self.number(obj_id, "x", row["x"]), self.number(obj_id, "y", row["y"])
        yaw_deg = self.yaw(obj_id, row["yaw_deg"])
        offset, size = self.footprint(obj_id, row["footprint"])
        height = self.side(obj_id, "height", row["height"])
        if "z" in row and not model.startswith(MODEL_PREFIX):
            self.fail(obj_id, f"z {vocabulary._show(row['z'])}", "z is for model:// entries only")
        z = self.number(obj_id, "z", row.get("z", 0.0))
        detectable = row["detectable"]
        if not isinstance(detectable, bool):
            self.fail(obj_id, f"detectable {vocabulary._show(detectable)}", "detectable must be true or false")
        accept = self.accept(obj_id, label, row["accept"])
        return Entry(obj_id, label, model, x, y, yaw_deg, offset, size, height, z, detectable, accept)

    def label_of(self, obj_id: object) -> str:
        m = _ID_SHAPE.fullmatch(obj_id) if isinstance(obj_id, str) else None
        if m is None:
            self.fail("-", obj_id, "an id is <label>_<n> with n in 1..99")
        if m.group(1) not in vocabulary.LABELS:
            self.fail(obj_id, m.group(1), "the id's label is not a kind in voice_ltl/vocabulary.json")
        if objects.ID_RE.fullmatch(obj_id) is None:         # cannot happen for a vocabulary label; kept as a guard
            self.fail(obj_id, obj_id, "an id is <label>_<n> with n in 1..99")
        return m.group(1)

    def model(self, obj_id: str, label: str, model: object) -> str:
        if isinstance(model, str) and model in PRIMITIVES:
            if PRIMITIVES[model] != label:
                self.fail(obj_id, model, f"{model} is for the label {PRIMITIVES[model]} only")
            return model
        if isinstance(model, str) and model.startswith(MODEL_PREFIX):
            folder = model[len(MODEL_PREFIX):]
            if not _FOLDER_SHAPE.fullmatch(folder):
                self.fail(obj_id, model, "model://<folder> needs one folder name")
            if self.folders is None:
                self.folders = model_folders()
            if folder not in self.folders:
                self.fail(obj_id, model, f"the folder is not in {_display(FUEL_MODELS)} "
                                         f"(the folders are {', '.join(sorted(self.folders)) or 'none'})")
            return model
        self.fail(obj_id, model, f"model is {', '.join(PRIMITIVES)} or model://<folder>")

    def number(self, obj_id: str, key: str, v: object) -> float:
        if not vocabulary._number(v):
            self.fail(obj_id, f"{key} {vocabulary._show(v)}", f"{key} must be a finite number")
        return float(v)

    def side(self, obj_id: str, key: str, v: object) -> float:
        if not (vocabulary._number(v) and 0 < v <= MAX_SIZE):
            self.fail(obj_id, f"{key} {vocabulary._show(v)}", f"{key} must be in (0, {MAX_SIZE:g}] m")
        return float(v)

    def yaw(self, obj_id: str, v: object) -> int:
        if not (vocabulary._number(v) and float(v).is_integer() and int(v) % 90 == 0):
            self.fail(obj_id, f"yaw_deg {vocabulary._show(v)}", "yaw_deg must be a multiple of 90")
        return int(v)

    def pair(self, obj_id: str, key: str, v: object) -> list:
        if not (isinstance(v, list) and len(v) == 2):
            self.fail(obj_id, f"{key} {vocabulary._show(v)}", f"{key} must be [x, y] in model axes")
        return v

    def footprint(self, obj_id: str, fp: object) -> tuple[tuple[float, float], tuple[float, float]]:
        if not isinstance(fp, dict):
            self.fail(obj_id, f"footprint {vocabulary._show(fp)}", "footprint is {\"offset\": [dx, dy], \"size\": [sx, sy]}")
        for key in fp:
            if key not in _FOOTPRINT_KEYS:
                self.fail(obj_id, f"footprint {key}", "unknown key (the keys are offset, size)")
        for key in _FOOTPRINT_KEYS:
            if key not in fp:
                self.fail(obj_id, f"footprint {key}", "a required key is missing")
        dx, dy = (self.number(obj_id, "offset", v) for v in self.pair(obj_id, "offset", fp["offset"]))
        sx, sy = (self.side(obj_id, "size", v) for v in self.pair(obj_id, "size", fp["size"]))
        return (dx, dy), (sx, sy)

    def accept(self, obj_id: str, label: str, v: object) -> tuple[str, ...]:
        if not (isinstance(v, list) and v):
            self.fail(obj_id, f"accept {vocabulary._show(v)}", "accept is a list of vocabulary labels")
        for a in v:
            if not (isinstance(a, str) and a in vocabulary.LABELS):
                self.fail(obj_id, f"accept {vocabulary._show(a)}", "not a kind in voice_ltl/vocabulary.json")
        if len(set(v)) != len(v):
            self.fail(obj_id, f"accept {vocabulary._show(v)}", "a label appears twice")
        if label not in v:
            self.fail(obj_id, f"accept {vocabulary._show(v)}", f"accept must include the id's own label {label}")
        return tuple(v)


def _read_json(path: Path, source: str) -> object:
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as e:
        raise LayoutError(source, "-", path.name, f"cannot read the file ({type(e).__name__})") from None
    try:
        return json.loads(text, object_pairs_hook=vocabulary._object_without_repeats)
    except json.JSONDecodeError as e:
        raise LayoutError(source, "-", f"line {e.lineno} column {e.colno}", f"not valid JSON ({e.msg})") from None
    except vocabulary._RepeatedKey as e:
        raise LayoutError(source, "-", e.key, "a key appears twice in one object") from None


def _display(path: Path) -> str:
    """The file as errors name it: relative to the repository when inside it."""
    try:
        return path.resolve().relative_to(ROOT).as_posix()
    except (OSError, ValueError):
        return str(path)
