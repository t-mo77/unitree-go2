"""gazebo/tools/png_rgb.py, gazebo/tools/record_frames.py and gazebo/tools/detector_check.py (Task 2.12; spec
§11.11, §12.8; design-detector §12.2, §12.3; plan review I8, M4, M8).

The fixture set tests/fixtures/detector/household/ holds synthetic recorded pairs of the household room in the
format record_frames writes (<view>.json from pair_record, <view>_scan.npy in the LiDAR frame) and no images
(spec §11.11: renders of CC BY models stay out of the repository). make_frame() below made it, and
test_fixtures_are_the_synthetic_frames checks that it still does:
- the room is SCENE_LAYOUT, gazebo/layouts/household.json as Task 2.12 found it, frozen here so that a later edit of
  the layout file (the detection preview H3 sets "detectable"; the room fix after it, R-H3b) does not change the
  fixtures or these tests; FIXTURE_TARGETS are that room's detector targets and test poses (spec §12.3 then), frozen
  with it, where detector_check.PREVIEW_TARGETS now holds the room proposal's (R-H3a, R-H3b);
- the scan is the ray caster of tests/test_detector_geometry.py (the Gazebo LiDAR's pattern, faces only, no floor:
  /utlidar/cloud has the ground removed) at the scan's pose, which differs from the image's by CLOUD_DELTA (within
  the core's 0.02 m / 1° pairing limit);
- the truth message is the bridge's (bridge_core.ObjectMemory with the layout sizes, bridge_core.logical_rows), with
  visible inside the logical camera's frustum (120°, 8 m);
- a shifted view moves every model with record_frames.shifted_pose, as record_frames --shift does in Gazebo.
The tests write the images with write_png (black, with a marker pixel: the oracle and the ScriptedDetector never look
at the picture).
"""

import json
import math
import os
import shutil
import struct
import subprocess
import sys
import zlib
from pathlib import Path

import numpy as np
import pytest

from detector.camera import CAMERA_H, CAMERA_HFOV, CAMERA_W, CameraModel, world_to_camera
from detector.core import View
from detector.oracle import OracleDetector, project_box
from detector.protocol import YOLOE_THRESHOLDS, Detection, vocabulary_prompts
from detector.scripted import ScriptedDetector
from gazebo.bridge_core import ObjectMemory, logical_rows
from gazebo.tools import detector_check, objects_check, record_frames
from gazebo.tools.png_rgb import read_png, write_png
from test_detector_core import to_lidar
from test_detector_geometry import scan, solid, yaw_q
from voice_ltl import layout as layouts
from voice_ltl import objects

ROOT = Path(__file__).resolve().parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "detector" / "household"
CAM = CameraModel.from_hfov()
CAMERA_INFO = {"k": [round(CAM.fx, 3), 0.0, 320.0, 0.0, round(CAM.fy, 3), 240.0, 0.0, 0.0, 1.0],
               "width": CAMERA_W, "height": CAMERA_H}
WORLD = {"boxes": [], "collided": False}
FOLDERS = frozenset({"TrashBin", "Suitcase1", "Sofa", "LampAndStand", "Table"})
SCENE_LAYOUT = {"version": 1, "objects": [
    {"id": "box_1", "model": "primitive:box", "x": 2.5, "y": 1.5, "yaw_deg": 0,
     "footprint": {"offset": [0.0, 0.0], "size": [0.5, 0.5]}, "height": 0.5, "detectable": False, "accept": ["box"]},
    {"id": "chair_1", "model": "primitive:chair", "x": 2.5, "y": -1.5, "yaw_deg": 0,
     "footprint": {"offset": [0.0, 0.0], "size": [0.45, 0.45]}, "height": 0.9, "detectable": False,
     "accept": ["chair"]},
    {"id": "trash_can_1", "model": "model://TrashBin", "x": -1.5, "y": -1.5, "yaw_deg": 0,
     "footprint": {"offset": [0.0, 0.0], "size": [0.266, 0.375]}, "height": 0.446, "detectable": False,
     "accept": ["trash_can"]},
    {"id": "suitcase_1", "model": "model://Suitcase1", "x": -1.5, "y": 1.5, "yaw_deg": 0,
     "footprint": {"offset": [0.0, 0.0], "size": [0.6, 0.4]}, "height": 0.8, "detectable": False,
     "accept": ["suitcase"]},
    {"id": "sofa_1", "model": "model://Sofa", "x": -4.4, "y": 0.0, "yaw_deg": 0,
     "footprint": {"offset": [0.02, 0.0], "size": [1.044, 1.8]}, "height": 0.943, "detectable": True,
     "accept": ["sofa"]},
    {"id": "lamp_1", "model": "model://LampAndStand", "x": 0.8, "y": -4.4, "yaw_deg": 0,
     "footprint": {"offset": [0.0, 0.0], "size": [0.622, 0.525]}, "height": 1.021, "detectable": True,
     "accept": ["lamp"]},
    {"id": "table_1", "model": "model://Table", "x": -2.4, "y": 4.4, "yaw_deg": 0,
     "footprint": {"offset": [0.002, 0.0], "size": [1.5, 0.8]}, "height": 1.015, "detectable": False,
     "accept": ["table", "desk"]}]}
LAYOUT = layouts.parse(SCENE_LAYOUT, "household", FOLDERS)
FIXTURE_TARGETS = {"trash_can_1": "B", "suitcase_1": "B", "sofa_1": "B", "lamp_1": "R"}   # SCENE_LAYOUT's test poses
IMAGE_T, CLOUD_T, STILL_SINCE = 50.5, 50.4, 49.8
CLOUD_DELTA = (0.004, -0.003, 0.2)            # the scan's pose - the image's (m, m, deg)
SEEN = 1.8e9                                  # the bridge's time.time() of the truth rows
LOGICAL_VFOV = 2 * math.atan(math.tan(CAMERA_HFOV / 2) / 1.778)   # the xacro's logical camera aspect ratio
SHIFTS = {"s1": (0.1, 0.1, 5.0), "s2": (0.1, -0.1, -5.0), "s3": (-0.1, 0.1, -5.0), "s4": (-0.1, -0.1, 5.0)}
STAND, LYING = 0.35, 0.11                     # the base height (m)
SPAWN = ((0.0, 0.0, STAND), 0.0)
POSE_B = ((0.03, -0.02, STAND), 178.5)        # after the turns of plan H3 (a little off the axes)
POSE_L = ((0.01, 0.02, STAND), 89.0)
POSE_R = ((-0.02, 0.04, STAND), -91.0)
FIXTURE_VIEWS = (("spawn", SPAWN, None), ("spawn_lying", ((0.0, 0.0, LYING), 0.0), None), ("B", POSE_B, None),
                 ("B_s1", POSE_B, SHIFTS["s1"]), ("L", POSE_L, None), ("R", POSE_R, None))
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
# The oracle's known failures on the fixture set, until the controller rules on Task 2.12's concern 1 (the oracle
# draws every truth row, and the core's ground-ray fallback places a box cut by the image edge short; Tasks 2.7 and
# 2.1): at pose L box_1 (bearing -58 deg, image columns 603-640) pairs 0.725 m from its truth. view -> the lines
# that fail there (detector_check.set_failures), in order.
KNOWN_ORACLE_FAILURES = {"L": ("max_pos_err",)}


# --- the synthetic frames ------------------------------------------------------------------------------------------

def _logical_sees(x, y, base, q):
    """The logical camera lists a model whose origin is inside its frustum (120°, 0.1-8 m)."""
    p = world_to_camera(np.array([[x, y, 0.0]]), base, q)[0]
    return bool(0.1 < p[0] <= 8.0 and abs(math.atan2(p[1], p[0])) <= CAMERA_HFOV / 2
                and abs(math.atan2(p[2], p[0])) <= LOGICAL_VFOV / 2)


def placed(shift, robot):
    """(entry, model origin x, y, yaw_deg) of every layout model, moved the way record_frames --shift moves them."""
    out = []
    for e in LAYOUT.entries:
        if shift is None:
            out.append((e, e.x, e.y, float(e.yaw_deg)))
        else:
            out.append((e, *record_frames.shifted_pose(e.x, e.y, e.yaw_deg, shift, robot)))
    return out


def make_frame(view, pose, shift=None, with_scan=True):
    """(record, scan) of one synthetic still pair (see the module docstring)."""
    base, yaw_deg = pose
    q = yaw_q(yaw_deg)
    robot = (base[0], base[1], yaw_deg)
    cbase = (base[0] + CLOUD_DELTA[0], base[1] + CLOUD_DELTA[1], base[2])
    cq = yaw_q(yaw_deg + CLOUD_DELTA[2])
    models = placed(shift, robot)
    solids = []
    for e, x, y, yaw in models:
        t = math.radians(yaw)
        dx, dy = e.offset
        solids.append(solid(x + math.cos(t) * dx - math.sin(t) * dy, y + math.sin(t) * dx + math.cos(t) * dy,
                            e.size[0], e.size[1], yaw_deg=yaw, height=e.height))
    if with_scan:
        points = to_lidar(scan(solids, cbase, cq), cbase, cq).astype(np.float32)
    else:
        points = np.zeros((0, 3), dtype=np.float32)
    rows = logical_rows([(e.id, x, y) for e, x, y, _ in models], LAYOUT)
    seen = [r for r, (e, x, y, _) in zip(rows, models) if _logical_sees(x, y, base, q)]
    memory = ObjectMemory(sizes={e.id: e.world_size() for e in LAYOUT.entries})
    memory.update(rows, SEEN)
    memory.update(seen, SEEN + 1.0)
    truth = memory.message(SEEN + 1.0, layout="household")
    moved = None if shift is None else {e.id: (x, y, yaw) for e, x, y, yaw in models}
    record = record_frames.pair_record(view, IMAGE_T, (base, q), CLOUD_T, (cbase, cq), CAMERA_INFO, truth, WORLD,
                                       None, still_since=STILL_SINCE, layout="household", shift=shift,
                                       robot=None if shift is None else robot, moved=moved)
    return record, points


def fixture_frames():
    return [make_frame(view, pose, shift) for view, pose, shift in FIXTURE_VIEWS]


def write_fixture_set(directory):
    """Write the fixture set (JSON and scans, no images) into directory: how FIXTURES was made."""
    directory.mkdir(parents=True, exist_ok=True)
    for record, points in fixture_frames():
        (directory / f"{record['view']}.json").write_text(
            json.dumps(record, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
        np.save(directory / record["scan"], points)


def marker_image(k):
    image = np.zeros((CAMERA_H, CAMERA_W, 3), dtype=np.uint8)
    image[0, 0, 0] = k
    return image


def fixture_set(tmp_path):
    """The committed fixture set with its images written (black, marker pixel = 1, 2, ... in view order)."""
    out = tmp_path / "household"
    shutil.copytree(FIXTURES, out)
    for k, path in enumerate(sorted(out.glob("*.json")), 1):
        write_png(out / f"{path.stem}.png", marker_image(k))
    return out


def built_set(tmp_path, views):
    """A set of make_frame(view, pose, shift, with_scan=False) frames saved with record_frames.save_pair;
    returns (directory, {marker: view})."""
    out = tmp_path / "set"
    markers = {}
    for k, (view, pose, shift) in enumerate(views, 1):
        record, points = make_frame(view, pose, shift, with_scan=False)
        record_frames.save_pair(out, record, marker_image(k), points)
        markers[k] = view
    return out, markers


def pose_views(pose_name, pose):
    """The pose view and its four shifted views (plan H3)."""
    return [(pose_name, pose, None)] + [(f"{pose_name}_{k}", pose, s) for k, s in SHIFTS.items()]


def truth_box(record, obj_id, lay=LAYOUT, scale=None):
    """The image box of a truth row of record, projected with the layout height (detector_check's preview rule)."""
    row = next(r for r in objects.parse(record["truth"]) if r.id == obj_id)
    pose = record["image_pose"]
    view = View(tuple(pose["pos"]), tuple(pose["quat"]), CAM, record["image_t"])
    box = project_box(view, row.x, row.y, row.size, lay.find(obj_id).height, CAMERA_W, CAMERA_H)[0]
    if scale is None:
        return box
    x0, y0, x1, y1 = box
    return (x0, y0, x0 + (x1 - x0) * scale[0], y0 + (y1 - y0) * scale[1])


def scripted_by_view(directory, markers, hits):
    """A ScriptedDetector that returns, for the image of view v, Detection(label, confidence, box) for each
    (label, confidence, truth id or a box, box scale) in hits[v]."""
    records = {r.view: r.record for r in detector_check.load_frames(directory)}

    def detections(view):
        out = []
        for label, confidence, where, *scale in hits.get(view, ()):
            box = where if isinstance(where, tuple) else truth_box(records[view], where, scale=scale[0] if scale
                                                                   else None)
            out.append(Detection(label, confidence, tuple(float(v) for v in box)))
        return out

    return ScriptedDetector(lambda k, image: detections(markers[int(image[0, 0, 0])]))


# --- png_rgb -------------------------------------------------------------------------------------------------------

def _chunks(data):
    pos = 8
    while pos < len(data):
        length, kind = struct.unpack(">I4s", data[pos:pos + 8])
        yield kind, data[pos + 8:pos + 8 + length]
        pos += 12 + length


def _paeth(a, b, c):
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    return a if pa <= pb and pa <= pc else (b if pb <= pc else c)


def _filtered(img, filters):
    """The PNG scanlines of img (H x W x C uint8), row i filtered with filters[i] (encoded by hand)."""
    h, bpp = img.shape[0], img.shape[2]
    rows = img.reshape(h, -1).astype(int).tolist()
    prior = [0] * len(rows[0])
    out = b""
    for cur, f in zip(rows, filters):
        enc = []
        for x, value in enumerate(cur):
            a = cur[x - bpp] if x >= bpp else 0
            b = prior[x]
            c = prior[x - bpp] if x >= bpp else 0
            pred = (0, a, b, (a + b) // 2, _paeth(a, b, c))[f] if f <= 4 else 0
            enc.append((value - pred) % 256)
        out += bytes([f] + enc)
        prior = cur
    return out


def _chunk(kind, body):
    return struct.pack(">I", len(body)) + kind + body + struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF)


def png_bytes(img, filters, colour=2, depth=8, interlace=0, idat_parts=1, raw=None):
    h, w = img.shape[:2]
    data = zlib.compress(_filtered(img, filters) if raw is None else raw)
    cut = [len(data) * i // idat_parts for i in range(idat_parts + 1)]
    return (PNG_SIGNATURE + _chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, depth, colour, 0, 0, interlace))
            + b"".join(_chunk(b"IDAT", data[a:b]) for a, b in zip(cut, cut[1:])) + _chunk(b"IEND", b""))


@pytest.mark.parametrize("shape", [(1, 1, 3), (5, 7, 3), (CAMERA_H, CAMERA_W, 3)])
def test_png_round_trip(tmp_path, shape):
    img = np.random.default_rng(3).integers(0, 256, shape, dtype=np.uint8)
    path = tmp_path / "a.png"
    write_png(path, img)
    data = path.read_bytes()
    assert data.startswith(PNG_SIGNATURE)
    chunks = list(_chunks(data))
    assert [kind for kind, _ in chunks] == [b"IHDR", b"IDAT", b"IEND"]
    assert struct.unpack(">IIBBBBB", chunks[0][1]) == (shape[1], shape[0], 8, 2, 0, 0, 0)
    rows = np.frombuffer(zlib.decompress(chunks[1][1]), dtype=np.uint8).reshape(shape[0], 1 + 3 * shape[1])
    assert not rows[:, 0].any()                                      # filter 0 on every row
    back = read_png(path)
    assert back.dtype == np.uint8 and back.shape == shape and np.array_equal(back, img)

    for bad in (img.astype(np.float32), img[:, :, 0], np.zeros((2, 2, 4), np.uint8), np.zeros((0, 3, 3), np.uint8)):
        with pytest.raises(ValueError):
            write_png(tmp_path / "bad.png", bad)


def test_read_png_undoes_every_filter(tmp_path):
    rng = np.random.default_rng(7)
    img = rng.integers(0, 256, (4, 3, 3), dtype=np.uint8)           # 3 x 4: one row per filter 1-4
    img[0, :, :] = (250, 251, 252)                                   # sums past 255 wrap
    path = tmp_path / "f.png"
    path.write_bytes(png_bytes(img, [1, 2, 3, 4]))
    assert np.array_equal(read_png(path), img)

    path.write_bytes(png_bytes(img, [4, 3, 0, 2], idat_parts=3))     # IDAT split in three
    assert np.array_equal(read_png(path), img)

    rgba = rng.integers(0, 256, (4, 3, 4), dtype=np.uint8)
    path.write_bytes(png_bytes(rgba, [2, 1, 4, 3], colour=6))        # RGBA: the alpha is dropped
    back = read_png(path)
    assert back.shape == (4, 3, 3) and np.array_equal(back, rgba[:, :, :3])


@pytest.mark.parametrize("make", [
    pytest.param(lambda img: png_bytes(img, [0] * 4, interlace=1), id="interlaced"),
    pytest.param(lambda img: png_bytes(img, [0] * 4, depth=16), id="16-bit"),
    pytest.param(lambda img: png_bytes(img, [0] * 4, colour=3), id="palette"),
    pytest.param(lambda img: png_bytes(img, [0] * 4, colour=0), id="grey"),
    pytest.param(lambda img: png_bytes(img, [0, 5, 0, 0]), id="filter-5"),
    pytest.param(lambda img: png_bytes(img, [0] * 4)[:-20], id="truncated"),
    pytest.param(lambda img: png_bytes(img, [0] * 4)[:-12], id="no-IEND"),
    pytest.param(lambda img: png_bytes(img, [0] * 4, raw=b"\x00" * 10), id="short-data"),
    pytest.param(lambda img: b"GIF89a" + png_bytes(img, [0] * 4)[8:], id="no-signature"),
    pytest.param(lambda img: png_bytes(img, [0] * 4).replace(b"IDAT", b"IDAt", 1), id="bad-crc"),
])
def test_read_png_refuses_what_it_cannot_read(tmp_path, make):
    path = tmp_path / "bad.png"
    path.write_bytes(make(np.zeros((4, 3, 3), dtype=np.uint8)))
    with pytest.raises(ValueError):
        read_png(path)


# --- record_frames -------------------------------------------------------------------------------------------------

def _se2(x, y, yaw_deg):
    t = math.radians(yaw_deg)
    return np.array([[math.cos(t), -math.sin(t), x], [math.sin(t), math.cos(t), y], [0.0, 0.0, 1.0]])


def _pose(m):
    return m[0, 2], m[1, 2], math.degrees(math.atan2(m[1, 0], m[0, 0]))


@pytest.mark.parametrize("robot", [(0.0, 0.0, 0.0), POSE_B[0][:2] + (POSE_B[1],), (1.0, 2.0, -91.0)])
@pytest.mark.parametrize("shift", list(SHIFTS.values()))
@pytest.mark.parametrize("model", [(-4.4, 0.0, 0.0), (0.8, -4.4, 90.0)])
def test_shifted_pose_inverts_the_view(robot, shift, model):
    moved = record_frames.shifted_pose(*model, shift, robot)
    # seen from the robot, the moved model sits where the model sits seen from the robot moved by the shift
    seen_moved = np.linalg.inv(_se2(*robot)) @ _se2(*moved)
    seen_shifted = np.linalg.inv(_se2(*robot) @ _se2(*shift)) @ _se2(*model)
    assert _pose(seen_moved) == pytest.approx(_pose(seen_shifted), abs=1e-9)
    assert -180.0 < moved[2] <= 180.0
    # so the camera at the robot sees the moved origin at the pixel the shifted camera sees the model at
    base, q = (robot[0], robot[1], STAND), yaw_q(robot[2])
    sx, sy, syaw = _pose(_se2(*robot) @ _se2(*shift))
    a = world_to_camera(np.array([[moved[0], moved[1], 0.3]]), base, q)
    b = world_to_camera(np.array([[model[0], model[1], 0.3]]), (sx, sy, STAND), yaw_q(syaw))
    assert a == pytest.approx(b, abs=1e-9)


def test_shifted_pose_examples_and_the_shift_argument():
    # stepping 0.1 m forward brings what is ahead 0.1 m closer; turning left by 90° puts it on the right
    assert record_frames.shifted_pose(2.0, 0.0, 0.0, (0.1, 0.0, 0.0)) == pytest.approx((1.9, 0.0, 0.0))
    assert record_frames.shifted_pose(2.0, 0.0, 0.0, (0.0, 0.0, 90.0)) == pytest.approx((0.0, -2.0, -90.0))
    assert record_frames.shifted_pose(2.0, 1.0, 30.0, (0.0, 0.0, 0.0), (5.0, 5.0, 45.0)) == \
        pytest.approx((2.0, 1.0, 30.0))
    assert record_frames.parse_shift("0.1,-0.1,5") == (0.1, -0.1, 5.0)
    assert record_frames.parse_shift(" -0.1 , 0.1 , -5 ") == (-0.1, 0.1, -5.0)
    for bad in ("0.1,0.1", "0.1,0.1,5,1", "a,b,c", "nan,0,0", "0,inf,0"):
        with pytest.raises(ValueError):
            record_frames.parse_shift(bad)
    cmd = record_frames.set_pose_command("sofa_1", -4.3, 0.2, 0.0, 5.0)
    assert cmd[:4] == ["gz", "service", "-s", "/world/empty/set_pose"]
    assert cmd[cmd.index("--reqtype") + 1] == "gz.msgs.Pose" and cmd[cmd.index("--reptype") + 1] == "gz.msgs.Boolean"
    req = cmd[cmd.index("--req") + 1]
    assert 'name: "sofa_1"' in req and "x: -4.300000" in req and f"z: {math.sin(math.radians(2.5)):.9f}" in req


def test_pair_record_shape():
    base_q = ((0.03, -0.02, STAND), yaw_q(178.5))
    truth = {"frame": "odom", "stamp": SEEN, "source": "logical", "layout": "household", "objects": []}
    record = record_frames.pair_record("B_s1", IMAGE_T, base_q, CLOUD_T, base_q, CAMERA_INFO, truth, WORLD,
                                       {"ok": True, "reason": "ok"}, still_since=STILL_SINCE, layout="household",
                                       shift=(0.1, 0.1, 5.0), robot=(0.03, -0.02, 178.5),
                                       moved={"sofa_1": (-4.3, 0.2, 5.0), "box_1": (2.4, 1.6, 5.0)})
    assert set(record) == {"version", "view", "image", "scan", "image_t", "cloud_t", "still_since", "image_pose",
                           "cloud_pose", "camera_info", "truth", "world", "status", "layout", "shift", "robot",
                           "moved"}
    assert record["version"] == record_frames.RECORD_VERSION == 1
    assert (record["view"], record["image"], record["scan"]) == ("B_s1", "B_s1.png", "B_s1_scan.npy")
    assert (record["image_t"], record["cloud_t"], record["still_since"]) == (IMAGE_T, CLOUD_T, STILL_SINCE)
    assert record["image_pose"] == {"pos": [0.03, -0.02, STAND], "quat": list(yaw_q(178.5))}
    assert record["camera_info"] == CAMERA_INFO
    assert (record["truth"], record["world"], record["status"]) == (truth, WORLD, {"ok": True, "reason": "ok"})
    assert record["shift"] == [0.1, 0.1, 5.0] and record["robot"] == [0.03, -0.02, 178.5]
    assert list(record["moved"]) == ["box_1", "sofa_1"] and record["moved"]["sofa_1"] == [-4.3, 0.2, 5.0]
    assert json.loads(json.dumps(record)) == record                   # plain JSON
    plain = record_frames.pair_record("spawn", IMAGE_T, base_q, CLOUD_T, base_q, None, None, None, None)
    assert (plain["camera_info"], plain["shift"], plain["robot"], plain["moved"], plain["still_since"]) == \
        (None, None, None, {}, None)
    for view in ("", "../B", "B/1", "B s1", "-B"):
        with pytest.raises(ValueError):
            record_frames.pair_record(view, IMAGE_T, base_q, CLOUD_T, base_q, None, None, None, None)
    with pytest.raises(ValueError):
        record_frames.pair_record("B", IMAGE_T, ((0.0, 0.0), (1.0, 0.0, 0.0, 0.0)), CLOUD_T, base_q, None, None,
                                  None, None)
    with pytest.raises(ValueError):
        record_frames.pair_record("B", float("nan"), base_q, CLOUD_T, base_q, None, None, None, None)


def test_image_rgb_reads_the_camera_encodings():
    rgb = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    padded = np.zeros((2, 12), np.uint8)
    padded[:, :9] = rgb.reshape(2, 9)                                # a row step longer than 3 W
    assert np.array_equal(record_frames.image_rgb(3, 2, "rgb8", 12, padded.tobytes()), rgb)
    assert np.array_equal(record_frames.image_rgb(3, 2, "bgr8", 9, rgb[:, :, ::-1].tobytes()), rgb)
    rgba = np.concatenate([rgb, np.full((2, 3, 1), 255, np.uint8)], axis=2)
    assert np.array_equal(record_frames.image_rgb(3, 2, "rgba8", 12, rgba.tobytes()), rgb)
    for args in ((3, 2, "mono8", 3, bytes(6)), (3, 2, "rgb8", 9, bytes(10)), (3, 2, "rgb8", 8, bytes(16))):
        with pytest.raises(ValueError):
            record_frames.image_rgb(*args)


def _feed_still(rec, t0, t1, pos=(0.0, 0.0, STAND), quat=(1.0, 0.0, 0.0, 0.0), speed=0.0, wall=None):
    for k in range(int(round((t1 - t0) / 0.02)) + 1):
        rec.on_odom(round(t0 + 0.02 * k, 6), pos, quat, (speed, 0.0, 0.0), (0.0, 0.0, 0.0), wall=wall)


def test_recorder_pairs_a_still_image_and_scan():
    rec = record_frames.Recorder()
    image = marker_image(1)
    cloud = np.ones((5, 3), np.float32)
    rec.on_camera_info(CAMERA_INFO)
    _feed_still(rec, 10.0, 10.4, speed=0.3)                          # walking: nothing is kept
    rec.on_image(10.4, image, wall=1.0)
    rec.on_cloud(10.4, cloud)
    _feed_still(rec, 10.42, 10.6)                                    # still since 10.42, open from 10.72
    rec.on_image(10.6, image, wall=2.0)
    rec.on_cloud(10.6, cloud)
    assert rec.pair() is None and any("an image in the open still gate" in m for m in rec.missing())
    _feed_still(rec, 10.62, 11.2)
    rec.on_image(11.0, image, wall=3.0)
    rec.on_cloud(11.1, cloud)
    truth = {"frame": "odom", "stamp": SEEN, "source": "logical", "layout": "household", "objects": []}
    rec.on_truth(truth, wall=3.2)                                    # too soon after the image
    assert rec.pair() is None and "/gazebo/objects_truth" in " ".join(rec.missing())
    rec.on_truth(truth, wall=3.6)
    pair = rec.pair()
    assert (pair.image_t, pair.cloud_t, pair.still_since) == (11.0, 11.1, 10.42)
    assert pair.image is image and pair.cloud is cloud and pair.truth == truth
    assert pair.image_pose == ((0.0, 0.0, STAND), (1.0, 0.0, 0.0, 0.0)) and pair.cloud_pose == pair.image_pose
    assert rec.robot() == pytest.approx((0.0, 0.0, 0.0))


def test_recorder_waits_for_the_shift_to_settle_and_show_in_the_truth():
    rec = record_frames.Recorder()
    rec.on_camera_info(CAMERA_INFO)
    _feed_still(rec, 20.0, 21.0)
    rec.not_before = 21.0
    rec.expect = {"sofa_1": (-4.3, 0.2)}
    image, cloud = marker_image(1), np.ones((5, 3), np.float32)
    rec.on_image(20.5, image, wall=1.0)                              # before the move settled
    rec.on_cloud(20.5, cloud)
    _feed_still(rec, 21.02, 21.6)
    assert rec.pair() is None
    rec.on_image(21.5, image, wall=2.0)
    rec.on_cloud(21.5, cloud)
    stale = {"frame": "odom", "stamp": SEEN, "source": "logical", "layout": "household",
             "objects": [{"id": "sofa_1", "label": "sofa", "x": -4.38, "y": 0.0, "size": [1.044, 1.8],
                          "seen": SEEN, "visible": True}]}
    rec.on_truth(stale, wall=2.6)
    assert rec.pair() is None and rec.mismatch == ["sofa_1"]
    fresh = json.loads(json.dumps(stale))
    fresh["objects"][0].update(x=-4.28, y=0.2)
    rec.on_truth(fresh, wall=3.1)
    pair = rec.pair()
    assert pair is not None and pair.truth == fresh and pair.image_t == 21.5 and rec.mismatch == []


def sofa_truth(x, y):
    """A truth message with sofa_1 visible at (x, y)."""
    return {"frame": "odom", "stamp": SEEN, "source": "logical", "layout": "household",
            "objects": [{"id": "sofa_1", "label": "sofa", "x": x, "y": y, "size": [1.044, 1.8], "seen": SEEN,
                         "visible": True}]}


def test_recorder_never_pairs_an_image_or_scan_kept_before_not_before():
    # rclpy may run an image callback that was ready before the moves only after them: what was kept while
    # not_before was still None must not pair once not_before is set (fix round 1)
    rec = record_frames.Recorder()
    rec.on_camera_info(CAMERA_INFO)
    _feed_still(rec, 20.0, 21.0)
    old, new, cloud = marker_image(1), marker_image(2), np.ones((5, 3), np.float32)
    rec.on_image(20.9, old, wall=1.0)                                # kept while not_before is None
    rec.on_cloud(20.9, cloud)
    rec.not_before = 22.0
    rec.expect = {"sofa_1": (-4.28, 0.2)}
    _feed_still(rec, 21.02, 22.2)
    rec.on_truth(sofa_truth(-4.28, 0.2), wall=2.0)                   # a truth that shows the move
    assert rec.pair() is None
    missing = " ".join(rec.missing())
    assert "an image in the open still gate" in missing and "after the shift settled" in missing
    assert "a scan in the open still gate" in missing
    rec.on_cloud(22.1, cloud)                                        # a scan after not_before alone is not enough
    assert rec.pair() is None and "a scan" not in " ".join(rec.missing())
    rec.on_image(22.0, new, wall=2.5)
    rec.on_truth(sofa_truth(-4.28, 0.2), wall=3.0)
    pair = rec.pair()
    assert pair is not None and (pair.image_t, pair.cloud_t) == (22.0, 22.1) and pair.image is new


def test_recorder_holds_the_shift_until_odometry_after_the_moves():
    # main(): hold() before the first set_pose, moved() after the last; settle() opens not_before only once
    # SHIFT_FRESH_ODOM odometry messages arrived after the moves (more than DDS may still hold from before them)
    n = record_frames.SHIFT_FRESH_ODOM
    assert n > 5                                                     # the sensor-data QoS depth
    rec = record_frames.Recorder()
    rec.on_camera_info(CAMERA_INFO)
    old, new, cloud = marker_image(1), marker_image(2), np.ones((5, 3), np.float32)
    _feed_still(rec, 20.0, 21.0, wall=1.0)
    rec.on_image(20.9, old, wall=1.0)                                # kept before the shift
    rec.on_cloud(20.9, cloud)
    rec.on_truth(sofa_truth(-4.38, 0.0), wall=1.0)
    assert rec.still()
    rec.hold()
    assert rec.not_before == math.inf and rec.pair() is None and not rec.settle()
    rec.expect = {"sofa_1": (-4.28, 0.2)}
    rec.moved(wall=2.0)
    # what DDS still held from before the moves arrives now: n - 1 odometry messages, an image and a scan
    stamps = [round(21.02 + 0.02 * k, 6) for k in range(n + 100)]
    for t in stamps[:n - 1]:
        rec.on_odom(t, (0.0, 0.0, STAND), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), wall=2.01)
    rec.on_image(21.1, old, wall=2.01)
    rec.on_cloud(21.1, cloud)
    assert not rec.settle() and rec.not_before == math.inf and rec.pair() is None
    assert any("odometry after the shift" in m for m in rec.missing())
    rec.on_odom(stamps[n - 1], (0.0, 0.0, STAND), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), wall=2.05)
    assert rec.settle() and rec.settle()                             # once open, it stays open
    settle_at = stamps[n - 1] + record_frames.SHIFT_SETTLE_S
    assert rec.not_before == pytest.approx(settle_at)
    assert not any("odometry after the shift" in m for m in rec.missing())
    for t in stamps[n:]:
        rec.on_odom(t, (0.0, 0.0, STAND), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), wall=3.0)
    rec.on_image(settle_at - 0.1, old, wall=3.0)                     # before the move settled: not kept
    rec.on_cloud(settle_at - 0.1, cloud)
    rec.on_truth(sofa_truth(-4.28, 0.2), wall=3.6)
    assert rec.pair() is None                                        # nothing from before the moves pairs
    rec.on_image(settle_at + 0.2, new, wall=3.7)
    rec.on_cloud(settle_at + 0.3, cloud)
    rec.on_truth(sofa_truth(-4.28, 0.2), wall=4.2)
    pair = rec.pair()
    assert pair is not None and pair.image is new
    assert (pair.image_t, pair.cloud_t) == pytest.approx((settle_at + 0.2, settle_at + 0.3))


def test_recorder_keeps_the_first_image_while_the_truth_is_due():
    # the camera (2 Hz sim) against the truth (2 Hz wall) at an RTF of 1: a truth lands 0.45 s after each image,
    # never TRUTH_AFTER_S after the newest one, but it does after the interval's first image
    rec = record_frames.Recorder()
    rec.on_camera_info(CAMERA_INFO)
    cloud = np.ones((5, 3), np.float32)
    _feed_still(rec, 30.0, 33.0)
    truth = sofa_truth(-4.38, 0.0)
    for k, t in enumerate((31.0, 31.5, 32.0, 32.5)):
        rec.on_image(t, marker_image(k + 1), wall=1.0 + 0.5 * k)
        rec.on_cloud(t + 0.05, cloud)
        rec.on_truth(truth, wall=1.45 + 0.5 * k)
    pair = rec.pair()
    assert pair is not None and pair.image_t == 31.0 and int(pair.image[0, 0, 0]) == 1 and pair.cloud_t == 32.55
    # a new still interval keeps a new image
    _feed_still(rec, 33.02, 33.2, speed=0.3)
    _feed_still(rec, 33.22, 34.0)
    rec.on_image(33.8, marker_image(9), wall=4.0)
    assert rec.pair() is None                                        # the scan is of the last interval
    rec.on_cloud(33.85, cloud)
    rec.on_image(33.9, marker_image(10), wall=4.3)
    rec.on_truth(truth, wall=4.5)
    pair = rec.pair()
    assert pair is not None and pair.image_t == 33.8 and int(pair.image[0, 0, 0]) == 9


def test_record_frames_imports_without_rclpy():
    code = ("import sys, gazebo.tools.record_frames, gazebo.tools.detector_check, gazebo.tools.png_rgb; "
            "bad = sorted(m for m in ('rclpy', 'cv2', 'PIL', 'torch', 'ultralytics', 'sensor_msgs') "
            "if m in sys.modules); assert not bad, bad")
    paths = [str(ROOT), str(ROOT / "nl_control")] + [p for p in os.environ.get("PYTHONPATH", "").split(os.pathsep) if p]
    env = {**os.environ, "PYTHONPATH": os.pathsep.join(paths)}
    done = subprocess.run([sys.executable, "-c", code], cwd=ROOT, env=env, capture_output=True, text=True)
    assert done.returncode == 0, done.stderr


# --- detector_check ------------------------------------------------------------------------------------------------

def test_fixtures_are_the_synthetic_frames(tmp_path):
    made = tmp_path / "made"
    write_fixture_set(made)
    assert sorted(p.name for p in FIXTURES.iterdir()) == sorted(p.name for p in made.iterdir())
    assert not list(FIXTURES.glob("*.png"))                          # no images in the repository (spec §11.11)
    for path in sorted(made.glob("*.json")):
        assert json.loads((FIXTURES / path.name).read_text(encoding="utf-8")) == \
            json.loads(path.read_text(encoding="utf-8")), path.name
        a, b = np.load(FIXTURES / f"{path.stem}_scan.npy"), np.load(made / f"{path.stem}_scan.npy")
        assert a.dtype == b.dtype == np.float32 and a.shape == b.shape and np.allclose(a, b, atol=1e-5)


def oracle():
    det = OracleDetector()
    det.set_vocabulary(*vocabulary_prompts())
    return det


def test_replay_with_the_oracle_passes_accuracy(tmp_path):
    frames = detector_check.load_frames(fixture_set(tmp_path))
    assert [f.view for f in frames] == ["B", "B_s1", "L", "R", "spawn", "spawn_lying"]
    out = {r.view: r for r in detector_check.replay(frames, oracle(), LAYOUT, FIXTURE_TARGETS)}
    scored = sorted(v for v, r in out.items() if r.accuracy.get("in_scope"))
    assert scored == ["B", "B_s1", "R"]                              # a detectable truth within 5 m
    for view in scored:
        acc = out[view].accuracy
        assert out[view].code == 0 and acc["passed"] and acc["covers_all"], (view, acc)
        # within the spec's line of 1: the oracle draws every truth row, so a model cut by the image edge or hidden
        # behind another one (table_1 behind suitcase_1 in B) can confirm on the wrong or too few LiDAR points
        assert acc["false_pos"] <= 1 and acc["label_acc"] == 1.0
    assert out["B"].accuracy["targets"] == ["sofa_1", "suitcase_1", "trash_can_1"]   # spec §12.3: pose B
    assert out["R"].accuracy["targets"] == ["lamp_1"]
    assert {p["truth"] for p in out["B"].accuracy["pairs"]} == {"sofa_1", "suitcase_1", "trash_can_1"}
    assert {p["truth"] for p in out["spawn"].accuracy["pairs"]} == {"box_1", "chair_1"}
    for r in out.values():
        assert r.health["still_inferences"] >= 3 and r.health["reason"] == "ok"
        assert all(h.confidence == 1.0 for h in r.hits)
    # the shifted view sees the moved room: its rows sit at the moved truth
    moved = {p["truth"]: p["pos_err"] for p in out["B_s1"].accuracy["pairs"]}
    assert moved and max(moved.values()) <= 0.15

    # a frame without a detectable truth in range has no recall line, but its other lines still count (fix round 1)
    assert sorted(v for v, r in out.items() if not r.accuracy.get("in_scope")) == ["L", "spawn", "spawn_lying"]
    for view in ("spawn", "spawn_lying"):
        assert out[view].accuracy["failures"] == [detector_check.NO_RECALL]
        assert detector_check.frame_line(out[view])["set_failures"] == []
    for view, known in KNOWN_ORACLE_FAILURES.items():
        got = detector_check.frame_line(out[view])["set_failures"]
        assert [f.split(":")[0] for f in got] == list(known), (view, got)

    line = detector_check.set_line(list(out.values()), "household", "oracle", LAYOUT, FIXTURE_TARGETS)
    assert line["scored"] == scored and line["no_recall"] == ["L", "spawn", "spawn_lying"] and line["errors"] == []
    assert line["failed"] == sorted(KNOWN_ORACLE_FAILURES) and line["passed"] is False
    assert line["passed_frames"] == ["B", "B_s1", "R", "spawn", "spawn_lying"]
    assert line["frames"] == 6 and set(line["confusion"]) == {"by_label", "by_truth"}
    rest = [r for v, r in out.items() if v not in KNOWN_ORACLE_FAILURES]
    assert detector_check.set_line(rest, "household", "oracle", LAYOUT, FIXTURE_TARGETS)["passed"] is True


def test_preview_targets_are_the_rooms_detector_targets():
    # R-H3a, R-H3b (the room proposal after the H3 screening): the sofa and the suitcase from Bf (B plus one
    # F moved_forward), the traffic cone from R5 (R plus five); B, L and R are no longer detector test poses
    assert detector_check.PREVIEW_TARGETS == {"sofa_1": "Bf", "suitcase_1": "Bf", "traffic_cone_1": "R5"}
    house = layouts.load("household")
    assert set(detector_check.PREVIEW_TARGETS) == {e.id for e in house.entries if e.detectable}   # Part B passed
    assert all(house.find(t).accept == (t.rsplit("_", 1)[0],) for t in detector_check.PREVIEW_TARGETS)
    views = ("Bf", "Bf_s1", "Bf_s4", "R5", "R5_s2", "spawn", "spawn_lying", "B")
    assert [detector_check.pose_of(v) for v in views] == ["Bf", "Bf", "Bf", "R5", "R5", "spawn", "spawn_lying", "B"]
    for view in views + ("R5_s3", "Bf_s2"):                                # names record_frames --view takes
        assert record_frames._parse(["frames", "--view", view]).view == view


def test_set_failures_drop_only_the_missing_recall_line():
    assert objects_check.accuracy([], [], LAYOUT, [], (0.0, 0.0), [])["failures"] == [detector_check.NO_RECALL]
    assert detector_check.set_failures({"failures": [detector_check.NO_RECALL]}) == []
    assert detector_check.set_failures({"failures": ["max_pos_err: 0.725 > 0.35", detector_check.NO_RECALL]}) == \
        ["max_pos_err: 0.725 > 0.35"]
    assert detector_check.set_failures({"failures": ["min_recall: 0.500 < 0.6", "max_false_pos: 2 > 1"]}) == \
        ["min_recall: 0.500 < 0.6", "max_false_pos: 2 > 1"]
    assert detector_check.set_failures({"error": "no /go2/objects"}) == []


def test_detector_check_command_line_with_the_oracle(tmp_path, capsys):
    set_dir = fixture_set(tmp_path)
    assert detector_check.main([str(set_dir), "--detector", "oracle", "--layout", "household"]) == 1
    lines = [json.loads(s) for s in capsys.readouterr().out.splitlines()]
    assert [line.get("view") for line in lines[:-1]] == ["B", "B_s1", "L", "R", "spawn", "spawn_lying"]
    assert all({"code", "passed", "pairs", "hits", "latency_ms", "set_failures"} <= set(line) for line in lines[:-1])
    assert lines[-1]["set"] == "household" and lines[-1]["detector"] == "oracle" and lines[-1]["passed"] is False
    assert lines[-1]["failed"] == sorted(KNOWN_ORACLE_FAILURES)
    assert {"confusion", "preview"} <= set(lines[-1])
    for view in KNOWN_ORACLE_FAILURES:                               # without them the set passes: exit 0
        for name in (f"{view}.json", f"{view}.png", f"{view}_scan.npy"):
            (set_dir / name).unlink()
    assert detector_check.main([str(set_dir), "--detector", "oracle", "--layout", "household"]) == 0
    last = json.loads(capsys.readouterr().out.splitlines()[-1])
    assert last["passed"] is True and last["failed"] == [] and last["no_recall"] == ["spawn", "spawn_lying"]

    assert detector_check.main([str(tmp_path / "missing"), "--detector", "oracle"]) == 2
    assert "error" in json.loads(capsys.readouterr().out.splitlines()[-1])
    (set_dir / "B.json").write_text("{", encoding="utf-8")
    assert detector_check.main([str(set_dir), "--detector", "oracle", "--layout", "household"]) == 2
    error = json.loads(capsys.readouterr().out.splitlines()[-1])["error"]
    assert "B.json" in error


def test_replay_gives_the_oracle_the_truth_without_heights(tmp_path):
    frames = detector_check.load_frames(fixture_set(tmp_path))
    det = oracle()
    calls = []
    real = det.set_truth
    det.set_truth = lambda rows, heights=None: (calls.append((rows, heights)), real(rows, heights))
    detector_check.replay([f for f in frames if f.view == "B"], det, LAYOUT)
    (rows, heights), = calls
    assert heights is None and {r.id for r in rows} == {e.id for e in LAYOUT.entries}   # review M8


def test_confusion_counts_prompts_per_truth_object(tmp_path):
    directory, markers = built_set(tmp_path, [("B", POSE_B, None), ("R", POSE_R, None)])
    hits = {"B": [("sofa", 0.7, "sofa_1"), ("bench", 0.4, "sofa_1"), ("box", 0.3, "trash_can_1"),
                  ("chair", 0.5, (300.0, 10.0, 340.0, 40.0)),          # on no truth object
                  ("suitcase", 0.2, "suitcase_1")],                    # below sighting (0.25): not a hit
            "R": [("lamp", 0.6, "lamp_1"), ("desk", 0.45, "lamp_1", (1.0, 0.5))]}   # the lamp's upper half
    frames = detector_check.replay(detector_check.load_frames(directory), scripted_by_view(directory, markers, hits),
                                   LAYOUT)
    table = detector_check.confusion(frames)
    by_label, by_truth = table["by_label"], table["by_truth"]
    assert sorted(by_label) == ["bench", "box", "chair", "desk", "lamp", "sofa"]
    assert by_label["sofa"] == {"sofa_1": [{"view": "B", "confidence": 0.7, "iou": 1.0}]}
    assert by_label["bench"] == {"sofa_1": [{"view": "B", "confidence": 0.4, "iou": 1.0}]}
    assert by_label["box"] == {"trash_can_1": [{"view": "B", "confidence": 0.3, "iou": 1.0}]}
    assert list(by_label["chair"]) == [detector_check.NO_TRUTH]
    assert by_label["chair"][detector_check.NO_TRUTH][0]["confidence"] == 0.5
    assert list(by_label["desk"]) == ["lamp_1"] and by_label["desk"]["lamp_1"][0]["iou"] == pytest.approx(0.5, abs=0.01)
    assert sorted(by_truth) == sorted(["sofa_1", "trash_can_1", "lamp_1", detector_check.NO_TRUTH])
    assert sorted(by_truth["sofa_1"]) == ["bench", "sofa"] and sorted(by_truth["lamp_1"]) == ["desk", "lamp"]
    assert by_truth[detector_check.NO_TRUTH] == {"chair": by_label["chair"][detector_check.NO_TRUTH]}
    assert by_truth["sofa_1"]["bench"] == by_label["bench"]["sofa_1"]


def test_preview_rule(tmp_path):
    views = ([("spawn", SPAWN, None), ("spawn_lying", ((0.0, 0.0, LYING), 0.0), None)]
             + pose_views("B", POSE_B) + pose_views("R", POSE_R))
    directory, markers = built_set(tmp_path, views)
    frames = detector_check.load_frames(directory)
    targets = {"sofa_1": "B", "lamp_1": "R"}

    def run(hits):
        replayed = detector_check.replay(frames, scripted_by_view(directory, markers, hits), LAYOUT, targets)
        return detector_check.preview(replayed, targets, confirm=YOLOE_THRESHOLDS.confirm)

    spawn_ok = [("box", 0.8, "box_1"), ("chair", 0.7, "chair_1")]
    base = {"B": [("sofa", 0.6, "sofa_1")], "B_s1": [("sofa", 0.5, "sofa_1")], "B_s2": [("sofa", 0.4, "sofa_1")],
            "B_s3": [("sofa", 0.3, "sofa_1")],                         # below confirm: does not count
            "R": [("lamp", 0.7, "lamp_1")], "R_s1": [("lamp", 0.6, "lamp_1")],
            "R_s2": [("lamp", 0.6, "lamp_1", (0.5, 0.5))],              # IoU 0.25 with the truth box: no hit
            "spawn": spawn_ok,
            "spawn_lying": [("sofa", 0.6, "box_1")]}                      # ignored by preview
    got = run(base)
    sofa, lamp = got["targets"]["sofa_1"], got["targets"]["lamp_1"]
    assert sofa["passed"] is True and sofa["pose_hit"] is True and sofa["shift_hits"] == 2   # pose + 2 of 4
    assert sofa["shift_views"] == ["B_s1", "B_s2", "B_s3", "B_s4"] and sofa["wrong"] == []
    assert lamp["passed"] is False and lamp["pose_hit"] is True and lamp["shift_hits"] == 1   # 1 of 4 fails
    assert got["spawn"]["passed"] is True and got["spawn"]["labels"] == []
    assert got["detectable"] == ["sofa_1"] and got["enough"] is False

    worse = {**base, "B_s4": [("lamp", 0.5, "sofa_1")],                # another target's label on the sofa
             "spawn": spawn_ok + [("sofa", 0.6, "box_1")]}             # a new label in the standing spawn view
    got = run(worse)
    sofa = got["targets"]["sofa_1"]
    assert sofa["passed"] is False and sofa["shift_hits"] == 2
    assert sofa["wrong"] == [{"view": "B_s4", "label": "lamp", "confidence": 0.5, "iou": 1.0}]
    assert got["spawn"]["passed"] is False and got["spawn"]["labels"] == [{"label": "sofa", "confidence": 0.6}]
    assert got["detectable"] == []

    shy = {**base, "B": [("sofa", 0.34, "sofa_1")]}                    # the pose view itself below confirm
    assert run(shy)["targets"]["sofa_1"]["pose_hit"] is False
    missing = [f for f in frames if f.view != "R"]
    replayed = detector_check.replay(missing, scripted_by_view(directory, markers, base), LAYOUT, targets)
    lamp = detector_check.preview(replayed, targets)["targets"]["lamp_1"]
    assert lamp["passed"] is False and lamp["pose_hit"] is None        # no frame R
