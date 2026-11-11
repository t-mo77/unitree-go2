#!/usr/bin/env python3
"""Gazebo のモデルの当たり判定（collision）の形から、モデルの軸での足跡・ずれ・高さを出す（open-objects spec §12.2, §12.5）。

extent(<モデルのフォルダ>) は model.sdf の collision をすべて、link と collision の <pose> を重ねてモデルの軸に写し、
その箱（軸に沿った外枠）を {"size": [sx, sy], "offset": [dx, dy], "height": h, "z_min": z} で返す（m、6 桁で丸める）。
size と offset は配置ファイルの footprint（モデルの原点から足跡の中心まで、と大きさ）にそのまま写せる。
height は z の幅、z_min はいちばん下（0 でなければ、床に立てる置く高さは -z_min）。
- 形：box、cylinder、sphere、mesh（OBJ・DAE・STL の頂点に <scale> を掛ける）。ほかの形やメッシュの形式は ExtentError。
- mesh の uri は model://<フォルダ>/...（モデルのフォルダの隣のフォルダ）か、model.sdf からの相対パス。
- DAE は Gazebo の読み方に合わせて、<unit meter> と、シーンの node の matrix・translate・rotate・scale を掛け、
  シーンに置かれた形だけを数える。<up_axis> が Z_UP 以外（Y_UP・X_UP）なら、軸を決められないので ExtentError。
- モデル自身の <pose> は数えない（置くときの位置と向きが代わりになる）。visual は数えない。
標準ライブラリだけを使う（plan H1 は Gazebo のイメージの中で `python3 -I gazebo/tools/model_extent.py <フォルダ>` と動かす）。

使い方:
    python3 gazebo/tools/model_extent.py /opt/go2_models/Sofa
"""

from __future__ import annotations

import itertools
import json
import math
import re
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

DIGITS = 6
MAX_NODE_DEPTH = 64

_IDENTITY = (((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), (0.0, 0.0, 0.0))


class ExtentError(ValueError):
    """A model whose collision extent this tool cannot measure (one line naming the file and why)."""


# --- affine maps (A, t): p -> A p + t ---------------------------------------------------------------------------

def _compose(a, b):
    """a after b."""
    (A, s), (B, t) = a, b
    AB = tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)) for i in range(3))
    At = tuple(sum(A[i][k] * t[k] for k in range(3)) + s[i] for i in range(3))
    return AB, At


def _apply(tf, p):
    A, t = tf
    return tuple(A[i][0] * p[0] + A[i][1] * p[1] + A[i][2] * p[2] + t[i] for i in range(3))


def _rpy(roll: float, pitch: float, yaw: float):
    """SDF's fixed-axis roll, pitch, yaw: Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def _axis_angle(x: float, y: float, z: float, angle: float, where: str):
    n = math.sqrt(x * x + y * y + z * z)
    if n == 0.0:
        raise ExtentError(f"{where}: a rotation about the zero axis")
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return ((c + x * x * C, x * y * C - z * s, x * z * C + y * s),
            (y * x * C + z * s, c + y * y * C, y * z * C - x * s),
            (z * x * C - y * s, z * y * C + x * s, c + z * z * C))


def _quat(x: float, y: float, z: float, w: float, where: str):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        raise ExtentError(f"{where}: a zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


# --- reading -----------------------------------------------------------------------------------------------------

def _floats(text: str | None, where: str) -> list[float]:
    try:
        values = [float(v) for v in (text or "").split()]
    except ValueError:
        raise ExtentError(f"{where}: {text!r} is not a list of numbers") from None
    if not all(math.isfinite(v) for v in values):
        raise ExtentError(f"{where}: {text!r} has a number that is not finite")
    return values


def _xml(path: Path, where: str) -> ET.Element:
    try:
        root = ET.parse(path).getroot()
    except OSError as e:
        raise ExtentError(f"{path}: cannot read ({e.strerror or e})") from None
    except ET.ParseError as e:
        raise ExtentError(f"{where}: {path.name} is not XML ({e})") from None
    for el in root.iter():                                           # COLLADA files carry a namespace
        if isinstance(el.tag, str) and "}" in el.tag:
            el.tag = el.tag.split("}", 1)[1]
    return root


def _pose(owner: ET.Element, allowed_relative_to: tuple[str, ...], where: str):
    el = owner.find("pose")
    if el is None or not (el.text or "").strip():
        return _IDENTITY
    relative_to = (el.get("relative_to") or "").strip()
    if relative_to and relative_to not in allowed_relative_to:
        raise ExtentError(f"{where}: <pose relative_to=\"{relative_to}\"> is not supported")
    values = _floats(el.text, f"{where} <pose>")
    rotation_format = (el.get("rotation_format") or "euler_rpy").strip()
    if rotation_format == "euler_rpy" and len(values) == 6:
        x, y, z, roll, pitch, yaw = values
        if (el.get("degrees") or "").strip().lower() in ("true", "1"):
            roll, pitch, yaw = math.radians(roll), math.radians(pitch), math.radians(yaw)
        return _rpy(roll, pitch, yaw), (x, y, z)
    if rotation_format == "quat_xyzw" and len(values) == 7:
        x, y, z, qx, qy, qz, qw = values
        return _quat(qx, qy, qz, qw, where), (x, y, z)
    raise ExtentError(f"{where}: <pose rotation_format=\"{rotation_format}\"> with {len(values)} numbers "
                      f"is not supported")


def _vector(shape: ET.Element, tag: str, n: int, default: tuple[float, ...], where: str) -> tuple[float, ...]:
    text = shape.findtext(tag)
    if text is None:
        return default                                               # SDF's default
    values = _floats(text, f"{where} <{tag}>")
    if len(values) != n:
        raise ExtentError(f"{where}: <{tag}> needs {n} numbers, not {text.strip()!r}")
    return tuple(values)


class _Box:
    """The axis-aligned box around everything added (model axes)."""

    def __init__(self):
        self.lo = [math.inf] * 3
        self.hi = [-math.inf] * 3

    def add(self, p, where: str) -> None:
        if not all(math.isfinite(v) for v in p):
            raise ExtentError(f"{where}: a vertex that is not finite")
        for i in range(3):
            self.lo[i] = min(self.lo[i], p[i])
            self.hi[i] = max(self.hi[i], p[i])

    def add_centred(self, centre, half, where: str) -> None:
        self.add([c - h for c, h in zip(centre, half)], where)
        self.add([c + h for c, h in zip(centre, half)], where)

    def result(self) -> dict:
        def r(v: float) -> float:
            return round(v, DIGITS) + 0.0                            # + 0.0 turns -0.0 into 0.0
        lo, hi = self.lo, self.hi
        return {"size": [r(hi[0] - lo[0]), r(hi[1] - lo[1])],
                "offset": [r((lo[0] + hi[0]) / 2), r((lo[1] + hi[1]) / 2)],
                "height": r(hi[2] - lo[2]), "z_min": r(lo[2])}


# --- meshes ------------------------------------------------------------------------------------------------------

def _mesh_path(uri: str | None, sdf_path: Path, where: str) -> Path:
    uri = (uri or "").strip()
    if uri.startswith("model://"):
        name, _, rest = uri[len("model://"):].partition("/")
        parts = [p for p in rest.split("/") if p]
        if not name or name in (".", "..") or not parts or ".." in parts:
            raise ExtentError(f"{where}: the mesh URI {uri!r} is not model://<folder>/<file>")
        path = sdf_path.parent.parent / name / Path(*parts)
    elif uri and "://" not in uri and not uri.startswith("/"):
        path = sdf_path.parent / uri
    else:
        raise ExtentError(f"{where}: the mesh URI {uri!r} is not supported (model://<folder>/... or a relative path)")
    if not path.is_file():
        raise ExtentError(f"{where}: the mesh {uri} is not a file ({path})")
    return path


def _obj_points(path: Path, where: str) -> list:
    points = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for number, line in enumerate(f, 1):
            if line.startswith(("v ", "v\t")):
                values = _floats(" ".join(line.split()[1:4]), f"{where}: {path.name} line {number}")
                if len(values) != 3:
                    raise ExtentError(f"{where}: {path.name} line {number}: a vertex needs x y z")
                points.append(tuple(values))
    return points


def _stl_points(path: Path, where: str) -> list:
    data = path.read_bytes()
    if len(data) >= 84:
        (count,) = struct.unpack_from("<I", data, 80)
        if len(data) == 84 + 50 * count:                             # binary STL
            points = []
            for k in range(count):
                values = struct.unpack_from("<12f", data, 84 + 50 * k)
                points += [values[3:6], values[6:9], values[9:12]]
            return points
    text = data.decode("ascii", "replace")
    if not text.lstrip().startswith("solid"):
        raise ExtentError(f"{where}: {path.name} is neither a binary nor an ASCII STL file")
    return [tuple(_floats(" ".join(m.groups()), f"{where}: {path.name}"))
            for m in re.finditer(r"vertex\s+(\S+)\s+(\S+)\s+(\S+)", text)]


def _ref(ids: dict, url: str | None, tag: str, where: str) -> ET.Element:
    if not (url or "").startswith("#"):
        raise ExtentError(f"{where}: the reference {url!r} is not a #id in the same file")
    el = ids.get(url[1:])
    if el is None or el.tag != tag:
        raise ExtentError(f"{where}: {url} is not a <{tag}> in the file")
    return el


def _dae_positions(geometry: ET.Element, ids: dict, where: str) -> list:
    mesh = geometry.find("mesh")
    if mesh is None:
        raise ExtentError(f"{where}: geometry {geometry.get('id')} has no <mesh>")
    inputs = [i for i in mesh.findall("vertices/input") if i.get("semantic") == "POSITION"]
    if len(inputs) != 1:
        raise ExtentError(f"{where}: geometry {geometry.get('id')} needs one POSITION input")
    source = _ref(ids, inputs[0].get("source"), "source", where)
    values = _floats(source.findtext("float_array"), f"{where}: source {source.get('id')}")
    accessor = source.find("technique_common/accessor")
    try:
        stride = int(accessor.get("stride", "1")) if accessor is not None else 3
        offset = int(accessor.get("offset", "0")) if accessor is not None else 0
        count = int(accessor.get("count")) if accessor is not None else len(values) // 3
    except (TypeError, ValueError):
        raise ExtentError(f"{where}: source {source.get('id')} has a bad accessor") from None
    if stride < 3 or offset < 0 or count < 0 or (count and offset + (count - 1) * stride + 3 > len(values)):
        raise ExtentError(f"{where}: source {source.get('id')}: {count} points of stride {stride} do not fit "
                          f"{len(values)} numbers")
    return [tuple(values[offset + k * stride: offset + k * stride + 3]) for k in range(count)]


def _dae_local(node: ET.Element, where: str):
    """The node's own transform: its transform elements multiplied in document order."""
    local = _IDENTITY
    for child in node:
        tag = child.tag
        if tag in ("matrix", "translate", "rotate", "scale"):
            values = _floats(child.text, f"{where}: <{tag}>")
            need = {"matrix": 16, "translate": 3, "rotate": 4, "scale": 3}[tag]
            if len(values) != need:
                raise ExtentError(f"{where}: <{tag}> needs {need} numbers")
            if tag == "matrix":
                if values[12:16] != [0.0, 0.0, 0.0, 1.0]:
                    raise ExtentError(f"{where}: a <matrix> whose last row is not 0 0 0 1")
                step = (tuple(tuple(values[4 * i: 4 * i + 3]) for i in range(3)),
                        (values[3], values[7], values[11]))
            elif tag == "translate":
                step = (_IDENTITY[0], tuple(values))
            elif tag == "rotate":
                step = (_axis_angle(*values[:3], math.radians(values[3]), where), (0.0, 0.0, 0.0))
            else:
                sx, sy, sz = values
                step = (((sx, 0.0, 0.0), (0.0, sy, 0.0), (0.0, 0.0, sz)), (0.0, 0.0, 0.0))
            local = _compose(local, step)
        elif tag in ("lookat", "skew"):
            raise ExtentError(f"{where}: a node with <{tag}> is not supported")
    return local


def _dae_points(path: Path, where: str) -> list:
    root = _xml(path, where)
    where = f"{where}: {path.name}"
    meter, up_axis = 1.0, None
    asset = root.find("asset")
    if asset is not None:
        unit = asset.find("unit")
        if unit is not None and unit.get("meter") is not None:
            values = _floats(unit.get("meter"), f"{where}: <unit meter>")
            if len(values) != 1 or values[0] <= 0.0:
                raise ExtentError(f"{where}: <unit meter={unit.get('meter')!r}> is not one positive number")
            meter = values[0]
        up_axis = (asset.findtext("up_axis") or "").strip() or None
    if up_axis not in (None, "Z_UP"):
        raise ExtentError(f"{where}: up_axis {up_axis} is not supported (Z_UP only; measure this model by rendering)")
    ids = {el.get("id"): el for el in root.iter() if el.get("id")}
    scene = root.find("scene/instance_visual_scene")
    if scene is None:
        raise ExtentError(f"{where}: no <scene><instance_visual_scene>")
    visual_scene = _ref(ids, scene.get("url"), "visual_scene", where)
    points = []

    def walk(node: ET.Element, tf, depth: int) -> None:
        if depth > MAX_NODE_DEPTH:
            raise ExtentError(f"{where}: the nodes nest deeper than {MAX_NODE_DEPTH}")
        tf = _compose(tf, _dae_local(node, where))
        for child in node:
            if child.tag == "instance_geometry":
                geometry = _ref(ids, child.get("url"), "geometry", where)
                points.extend(_apply(tf, p) for p in _dae_positions(geometry, ids, where))
            elif child.tag == "instance_node":
                walk(_ref(ids, child.get("url"), "node", where), tf, depth + 1)
            elif child.tag == "node":
                walk(child, tf, depth + 1)
            elif child.tag == "instance_controller":
                raise ExtentError(f"{where}: <instance_controller> (a skinned mesh) is not supported")

    for node in visual_scene.findall("node"):
        walk(node, _IDENTITY, 0)
    return [(x * meter, y * meter, z * meter) for x, y, z in points]


_READERS = {".obj": _obj_points, ".dae": _dae_points, ".stl": _stl_points}


def _mesh_points(path: Path, where: str) -> list:
    reader = _READERS.get(path.suffix.lower())
    if reader is None:
        raise ExtentError(f"{where}: the mesh format {path.suffix or '(none)'} of {path.name} is not supported "
                          f"(OBJ, DAE, STL)")
    try:
        points = reader(path, where)
    except OSError as e:
        raise ExtentError(f"{where}: cannot read {path} ({e.strerror or e})") from None
    if not points:
        raise ExtentError(f"{where}: {path.name} has no vertices")
    return points


# --- the model ---------------------------------------------------------------------------------------------------

def _add_geometry(box: _Box, geometry: ET.Element, tf, sdf_path: Path, where: str) -> None:
    shapes = [c for c in geometry if isinstance(c.tag, str)]
    if len(shapes) != 1:
        raise ExtentError(f"{where}: <geometry> needs exactly one shape")
    shape = shapes[0]
    A, centre = tf
    if shape.tag == "box":
        sx, sy, sz = _vector(shape, "size", 3, (1.0, 1.0, 1.0), where)
        for cx, cy, cz in itertools.product((-0.5, 0.5), repeat=3):
            box.add(_apply(tf, (cx * sx, cy * sy, cz * sz)), where)
    elif shape.tag == "cylinder":                                    # axis along the shape's z
        (radius,) = _vector(shape, "radius", 1, (1.0,), where)
        (length,) = _vector(shape, "length", 1, (1.0,), where)
        half = [0.5 * length * abs(A[i][2]) + radius * math.hypot(A[i][0], A[i][1]) for i in range(3)]
        box.add_centred(centre, half, where)
    elif shape.tag == "sphere":
        (radius,) = _vector(shape, "radius", 1, (1.0,), where)
        box.add_centred(centre, [radius] * 3, where)
    elif shape.tag == "mesh":
        if shape.find("submesh") is not None:
            raise ExtentError(f"{where}: a mesh with <submesh> is not supported")
        scale = _vector(shape, "scale", 3, (1.0, 1.0, 1.0), where)
        path = _mesh_path(shape.findtext("uri"), sdf_path, where)
        for p in _mesh_points(path, where):
            box.add(_apply(tf, (p[0] * scale[0], p[1] * scale[1], p[2] * scale[2])), where)
    else:
        raise ExtentError(f"{where}: the geometry {shape.tag} is not supported (box, cylinder, sphere, mesh)")


def _the_model(sdf_path: Path) -> ET.Element:
    root = _xml(sdf_path, sdf_path.name)
    models = root.findall("model") if root.tag == "sdf" else []
    if root.tag != "sdf" or len(models) != 1 or len(root) != 1:
        raise ExtentError(f"{sdf_path}: the file needs <sdf> with exactly one <model> in it")
    model = models[0]
    for tag in ("model", "include"):
        if model.find(tag) is not None:
            raise ExtentError(f"{sdf_path}: a <{tag}> inside the model is not supported")
    return model


def extent(model_dir: Path) -> dict:
    """The model's collision extent in model axes: {"size": [sx, sy], "offset": [dx, dy], "height": h, "z_min": z}."""
    sdf_path = Path(model_dir) / "model.sdf"
    model = _the_model(sdf_path)
    box, collisions = _Box(), 0
    for link in model.findall("link"):
        link_name = link.get("name", "")
        link_tf = _pose(link, ("__model__",), f"{sdf_path.name}: link {link_name}")
        for collision in link.findall("collision"):
            where = f"{sdf_path.name}: link {link_name} collision {collision.get('name', '')}"
            geometry = collision.find("geometry")
            if geometry is None:
                raise ExtentError(f"{where}: no <geometry>")
            tf = _compose(link_tf, _pose(collision, (link_name,), where))
            _add_geometry(box, geometry, tf, sdf_path, where)
            collisions += 1
    if collisions == 0:
        raise ExtentError(f"{sdf_path}: no collision geometry in any link")
    return box.result()


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1:
        print("usage: model_extent.py <model folder>", file=sys.stderr)
        return 2
    try:
        print(json.dumps(extent(Path(args[0]))))
    except ExtentError as e:
        print(f"model_extent: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
