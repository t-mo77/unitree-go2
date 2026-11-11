"""The detector's camera model and LiDAR footprint geometry (Task 2.1; spec §11.4, §11.6;
design-detector.md §7 and §16 "Camera and extrinsics", "Selection, clusters, growth, footprint").

The synthetic clouds come from a small ray caster with the Gazebo LiDAR's pattern
(gazebo/go2_sensors.urdf.xacro: 360 columns of 1°, 16 rings of 4° from −30° to +30°, 0.05-10 m) hitting solid
boxes, so the geometry sees what gpu_lidar sees: returns of the visible faces only, one per ray.
"""

import math
import os
import subprocess
import sys
import warnings
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from detector import geometry as g
from detector.camera import (CAMERA_CLIP, CAMERA_H, CAMERA_HFOV, CAMERA_RATE_HZ, CAMERA_W, PROJ_NEAR,
                             CameraModel, camera_pose, t_cam_lidar, world_to_camera)
from gazebo import bridge_core
from gazebo.bridge_core import LIDAR_IN_BODY, lidar_points_world, quat_to_matrix
from voice_ltl import objects, vocabulary

ROOT = Path(__file__).resolve().parent.parent
XACRO = ROOT / "gazebo" / "go2_sensors.urdf.xacro"

BASE = (0.0, 0.0, 0.35)              # standing: the LiDAR about 0.45 m and the camera about 0.39 m above the floor
Q0 = (1.0, 0.0, 0.0, 0.0)
CAM = CameraModel.from_hfov()
AZ = np.radians(np.arange(-180.0, 180.0, 1.0))
EL = np.radians(-30.0 + 4.0 * np.arange(16))


def yaw_q(deg):
    t = math.radians(deg)
    return (math.cos(t / 2), 0.0, 0.0, math.sin(t / 2))


def pitch_q(deg):                    # positive: nose down (x turns toward -z)
    t = math.radians(deg)
    return (math.cos(t / 2), 0.0, math.sin(t / 2), 0.0)


def solid(cx, cy, sx, sy, yaw_deg=0.0, height=0.5, z0=0.0):
    """A solid box: centre (cx, cy), sides (sx along its yaw, sy across), from z0 to z0 + height."""
    return (float(cx), float(cy), float(sx), float(sy), math.radians(yaw_deg), float(z0), float(z0 + height))


def corners_xy(s):
    cx, cy, sx, sy, yaw = s[:5]
    a = np.array([math.cos(yaw), math.sin(yaw)])
    b = np.array([-math.sin(yaw), math.cos(yaw)])
    return np.array([[cx, cy] + i * sx / 2 * a + j * sy / 2 * b for i in (-1, 1) for j in (-1, 1)])


def _hit(origin, d, s):
    cx, cy, sx, sy, yaw, z0, z1 = s
    c, sn = math.cos(yaw), math.sin(yaw)
    ox, oy = origin[0] - cx, origin[1] - cy
    o = np.array([c * ox + sn * oy, -sn * ox + c * oy, origin[2]])
    dd = np.stack([c * d[:, 0] + sn * d[:, 1], -sn * d[:, 0] + c * d[:, 1], d[:, 2]], axis=1)
    dd = np.where(np.abs(dd) < 1e-12, 1e-12, dd)
    lo, hi = np.array([-sx / 2, -sy / 2, z0]), np.array([sx / 2, sy / 2, z1])
    t1, t2 = (lo - o) / dd, (hi - o) / dd
    t_in, t_out = np.minimum(t1, t2).max(axis=1), np.maximum(t1, t2).min(axis=1)
    return np.where((t_in <= t_out) & (t_in > 0), t_in, np.inf)


def scan(solids, base=BASE, q=Q0, floor=False, which=False):
    """World points of one LiDAR scan: the nearest solid (or the floor) along each ray, 0.05-10 m."""
    origin = lidar_points_world(np.zeros((1, 3)), base, q)[0]
    az, el = np.meshgrid(AZ, EL)
    d = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=-1).reshape(-1, 3)
    d = d @ quat_to_matrix(q).T
    hits = np.array([_hit(origin, d, s) for s in solids] or np.full((1, len(d)), np.inf))
    if floor:
        with np.errstate(divide="ignore"):
            hits = np.vstack([hits, np.where(d[:, 2] < 0, -origin[2] / d[:, 2], np.inf)])
    t, k = hits.min(axis=0), hits.argmin(axis=0)
    ok = (t >= 0.05) & (t <= 10.0)
    pts = origin + d[ok] * t[ok, None]
    return (pts, k[ok]) if which else pts


def bbox(s, base=BASE, q=Q0, cam=CAM):
    """The detection box (x0, y0, x1, y1) of a solid: its projected corners, clipped to the image."""
    cs = np.array([[x, y, z] for x, y in corners_xy(s) for z in (s[5], s[6])])
    p = world_to_camera(cs, base, q)
    uv = np.column_stack([cam.cx - cam.fx * p[:, 1] / p[:, 0], cam.cy - cam.fy * p[:, 2] / p[:, 0]])
    x0, y0 = np.clip(uv.min(axis=0), 0, [cam.width, cam.height])
    x1, y1 = np.clip(uv.max(axis=0), 0, [cam.width, cam.height])
    return (float(x0), float(y0), float(x1), float(y1))


def rect_mask(box, cam=CAM):
    m = np.zeros((cam.height, cam.width), dtype=bool)
    x0, y0, x1, y1 = (int(round(v)) for v in box)
    m[y0:y1, x0:x1] = True
    return m


def covers(fp, s, tol=1e-9):
    c = corners_xy(s)
    return bool((np.abs(c[:, 0] - fp.x) <= fp.size[0] / 2 + tol).all()
                and (np.abs(c[:, 1] - fp.y) <= fp.size[1] / 2 + tol).all())


def cam_xy(base=BASE, q=Q0):
    pos, _ = camera_pose(base, q)
    return (float(pos[0]), float(pos[1]))


# ---- camera and extrinsics ----
def test_fx_from_hfov_matches_the_spike():
    small = CameraModel.from_hfov(320, 240)
    assert small.fx == pytest.approx(92.418, abs=1e-3) and small.fy == small.fx
    assert (small.cx, small.cy) == (160, 120)
    assert CAM.fx == pytest.approx(184.836, abs=1e-3) and CAM.fy == CAM.fx
    assert (CAM.width, CAM.height, CAM.cx, CAM.cy) == (640, 480, 320, 240)
    assert (CAMERA_W, CAMERA_H, CAMERA_RATE_HZ, CAMERA_HFOV, CAMERA_CLIP, PROJ_NEAR) == (
        640, 480, 2.0, 2.094, (0.1, 10.0), 0.1)
    assert CAMERA_HFOV == objects.CAMERA_HFOV
    # CameraInfo.k of the spike (v1_spawn.json, 640 x 480) agrees with the hfov model; 1 % apart does not
    spike = CameraModel.from_k([184.8364, 0.0, 320.0, 0.0, 184.8364, 240.0, 0.0, 0.0, 1.0], 640, 480)
    assert spike == CameraModel(640, 480, 184.8364, 184.8364, 320.0, 240.0)
    assert spike.agrees(CAM) and CAM.agrees(spike)
    assert not CAM.agrees(CameraModel(640, 480, CAM.fx * 1.011, CAM.fy, 320.0, 240.0))
    assert not CAM.agrees(CameraModel(640, 480, CAM.fx, CAM.fy, 320.0, 245.0))
    assert not CAM.agrees(small)
    with pytest.raises(ValueError):
        CameraModel.from_k([184.8, 0.0, 320.0], 640, 480)


@pytest.mark.parametrize("p_cam, pixel", [
    ((2.172, 1.5, -0.147), (192.4, 252.5)),      # box_1 in the spike's v1_spawn
    ((2.173, -1.5, 0.052), (447.6, 235.6)),      # chair_1 in the same image
])
def test_spike_box_projects_to_its_recorded_pixel(p_cam, pixel):
    uv, ok = CAM.project(np.array([p_cam]))
    assert ok.tolist() == [True]
    assert uv[0] == pytest.approx(pixel, abs=0.1)
    ray = CAM.ray(*uv[0])                            # and back: the pixel's ray points at the object
    assert np.linalg.norm(ray) == pytest.approx(1.0)
    assert ray == pytest.approx(np.array(p_cam) / np.linalg.norm(p_cam), abs=1e-9)


def _joint_origin(name):
    joint = next(j for j in ET.parse(XACRO).getroot().iter("joint") if j.get("name") == name)
    origin = joint.find("origin")
    return (tuple(float(v) for v in origin.get("xyz").split()),
            tuple(float(v) for v in origin.get("rpy").split()))


def test_t_cam_lidar_matches_the_xacro():
    lidar, lidar_rpy = _joint_origin("utlidar_joint")
    camera, camera_rpy = _joint_origin("logical_camera_joint")
    assert lidar_rpy == camera_rpy == (0.0, 0.0, 0.0)         # both on base, no rotation: translation only
    assert lidar == pytest.approx(LIDAR_IN_BODY) and camera == pytest.approx(objects.CAMERA_IN_BODY)
    t = t_cam_lidar()
    assert t.shape == (4, 4)
    assert t[:3, :3] == pytest.approx(np.eye(3)) and t[3] == pytest.approx([0, 0, 0, 1])
    assert t[:3, 3] == pytest.approx(np.subtract(lidar, camera), abs=1e-12)
    assert t[:3, 3] == pytest.approx((-0.04715, 0.00003, 0.05703), abs=1e-12)
    # a LiDAR point reaches the camera frame by the transform, whatever the base pose (here lying and turned)
    rng = np.random.default_rng(3)
    p_lidar = rng.uniform(-3, 3, (20, 3))
    pos, q = (1.0, -2.0, 0.12), (0.9, 0.05, 0.3, 0.2)
    q = tuple(np.array(q) / np.linalg.norm(q))
    via_world = world_to_camera(lidar_points_world(p_lidar, pos, q), pos, q)
    assert via_world == pytest.approx((np.column_stack([p_lidar, np.ones(20)]) @ t.T)[:, :3], abs=1e-9)


def test_camera_pose_and_world_to_camera_follow_the_base():
    pos, r = camera_pose((1.0, 2.0, 0.35), yaw_q(90))
    assert pos == pytest.approx([1.0 + 0.00003, 2.0 + 0.32715, 0.35 + 0.04297])
    assert r == pytest.approx(quat_to_matrix(yaw_q(90)))
    # a point 2 m ahead of the camera of a Go2 facing +y is straight ahead in the camera frame (x forward)
    p = world_to_camera(np.array([[pos[0], pos[1] + 2.0, pos[2] + 0.5]]), (1.0, 2.0, 0.35), yaw_q(90))
    assert p[0] == pytest.approx([2.0, 0.0, 0.5])


def test_points_behind_or_outside_are_dropped():
    p = np.array([
        [2.0, 0.0, 0.0],       # ahead, image centre
        [0.05, 0.0, 0.0],      # nearer than PROJ_NEAR
        [0.1, 0.0, 0.0],       # at PROJ_NEAR: not beyond it
        [0.0, 0.3, 0.0],       # in the camera plane
        [-1.0, 0.0, 0.0],      # behind
        [1.0, 2.0, 0.0],       # left of the image (u < 0)
        [1.0, 0.0, 1.5],       # above the image (v < 0)
        [1.0, 1.7, 0.0],       # just inside the left edge
        [1.0, -1.7, -1.29],    # just inside the right and bottom edges
    ])
    with warnings.catch_warnings():
        warnings.simplefilter("error")                # no division warnings for x = 0 or behind
        uv, ok = CAM.project(p)
    assert uv.shape == (9, 2) and ok.dtype == bool
    assert ok.tolist() == [True, False, False, False, False, False, False, True, True]
    assert np.isnan(uv[1:5]).all()                    # not projected at all
    assert uv[0] == pytest.approx([320, 240])
    assert CAM.project(np.zeros((0, 3)))[1].shape == (0,)


# ---- selection, clusters, growth ----
def test_constants_are_the_spec_values():
    assert (g.GROUND_Z, g.HEIGHT_CAP, g.MASK_ERODE_PX, g.BOX_SHRINK, g.SEED_EPS) == (0.05, 2.5, 2, (0.10, 0.05), 0.25)
    assert (g.MIN_CLUSTER_POINTS, g.MIN_CLUSTER_SHARE, g.EDGE_PAD, g.SIZE_MIN, g.DET_RANGE) == (5, 0.25, 0.05, 0.1, 8.0)
    assert (g.FALLBACK_BOTTOM_PX, g.VACANCY_GROW, g.VACANCY_MIN_HEIGHT) == (3, 0.1, 0.3)
    assert g.GROUND_Z == bridge_core.GROUND_Z and g.DET_RANGE == objects.CAMERA_FAR
    assert g.MAX_SIZE == objects.MAX_SIZE == 3.0


def test_cut_with_a_lying_pitch():
    # lying, nose down 12°: in the LiDAR frame the floor ahead is not at one height, in the world it is
    base, q = (0.0, 0.0, 0.12), pitch_q(12.0)
    world = np.array([[1.5, 0.0, 0.0], [3.0, 0.5, 0.02],               # floor
                      [2.0, 0.0, 0.3], [2.0, 0.2, 0.06], [2.5, -0.1, 2.4],   # kept
                      [2.0, 0.0, 2.6], [np.nan, 0.0, 1.0]])            # above HEIGHT_CAP, a missing return
    r = quat_to_matrix(q)
    p_lidar = (world - np.array(base)) @ r - np.array(LIDAR_IN_BODY)
    back = lidar_points_world(p_lidar, base, q)
    assert back[:6] == pytest.approx(world[:6])
    kept = g.cut(back)
    assert kept == pytest.approx(world[2:5])
    assert not np.allclose(p_lidar[0, 2], p_lidar[1, 2], atol=0.05)   # a LiDAR-frame cut would be wrong


def test_mask_seed_drops_edge_points():
    pts = np.arange(30, dtype=float).reshape(10, 3)
    uv = np.array([[150.5, 150.5],     # 0 the middle
                   [100.5, 150.5],     # 1 the mask's edge column
                   [102.5, 150.5],     # 2 kept by the eroded mask, outside the shrunk box
                   [197.5, 150.5],     # 3 the same on the right
                   [198.5, 150.5],     # 4 eroded away
                   [90.0, 150.0],      # 5 outside the mask and the box
                   [150.5, 101.5],     # 6 eroded row
                   [112.0, 150.0],     # 7 inside both
                   [150.0, 194.0],     # 8 inside both
                   [150.5, 150.5]])    # 9 not in the image
    in_image = np.array([True] * 9 + [False])
    box = (100.0, 100.0, 200.0, 200.0)
    mask = rect_mask(box)                                   # pixels 100..199 in both directions
    seed = g.seed_points(pts, uv, in_image, box, mask)
    # eroded by 2 px: columns / rows 100, 101, 198, 199 go; outside the mask and outside the image never count
    assert seed.tolist() == pts[[0, 2, 3, 7, 8]].tolist()
    # without a mask: the box shrunk by 10 % of its width and 5 % of its height per side (110..190, 105..195)
    seed = g.seed_points(pts, uv, in_image, box, None)
    assert seed.tolist() == pts[[0, 7, 8]].tolist()
    # a mask removes what the box holds besides the object (the floor and the wall behind a table)
    small = np.zeros_like(mask)
    small[140:160, 140:160] = True
    assert g.seed_points(pts, uv, in_image, box, small).tolist() == pts[[0]].tolist()
    assert g.erode(mask, 2).sum() == 96 * 96
    assert g.seed_points(np.zeros((0, 3)), np.zeros((0, 2)), np.zeros(0, bool), box, mask).shape == (0, 3)


def test_seed_clusters():
    two = np.array([[0.0, 0.0], [0.1, 0.0], [0.5, 0.0], [0.6, 0.05]])      # 0.4 m apart
    assert [gr.tolist() for gr in g.clusters(two)] == [[0, 1], [2, 3]]
    chain = np.array([[0.2 * k, 0.0, 0.5] for k in range(12)])            # gaps of 0.2 m, XYZ accepted
    assert [gr.tolist() for gr in g.clusters(chain)] == [list(range(12))]
    shuffled = chain[[5, 0, 11, 3, 8, 1, 10, 2, 9, 4, 7, 6]]
    assert len(g.clusters(shuffled)) == 1
    assert g.clusters(np.zeros((0, 2))) == []
    assert len(g.clusters(two, eps=0.45)) == 1


def _row(x, y0, y1, n, z=0.4):
    return np.array([[x, y, z] for y in np.linspace(y0, y1, n)])


def test_pick_nearest_substantial_cluster():
    cam = (0.33, 0.0)
    face, wall = _row(2.0, -0.25, 0.25, 20), _row(4.0, -1.0, 1.0, 60)
    sliver, occluder = _row(1.2, 0.0, 0.05, 2), _row(1.0, -0.4, 0.4, 30)

    def pick(*parts):
        pts = np.vstack(parts)
        return g.pick_cluster(pts, g.clusters(pts[:, :2]), cam)

    assert pick(face, wall).tolist() == face.tolist()             # the background loses on distance
    assert pick(sliver, face, wall).tolist() == face.tolist()     # a 2-point sliver in front loses on size
    # documented limit: a big occluder in front of the object wins (the tracker sees a wrong footprint)
    assert pick(occluder, face).tolist() == occluder.tolist()
    # substantial = at least max(5, 25 % of the largest cluster)
    assert pick(_row(2.0, 0.0, 0.1, 4)) is None
    assert pick(_row(2.0, 0.0, 0.1, 5)).shape == (5, 3)
    assert pick(_row(1.5, 0.0, 0.1, 7), _row(3.0, -1.0, 1.0, 30)).shape == (30, 3)   # 7 < 7.5
    assert g.pick_cluster(np.zeros((0, 3)), [], cam) is None


def _extent_along(points_xy, deg):
    a = np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg))])
    s = points_xy @ a
    return s.max() - s.min()


def test_growth():
    L, S = vocabulary.size_of("table")
    z_max = vocabulary.height_of("table") + 0.3
    radius = objects.measure_radius("table")
    cxy = cam_xy()
    # a table cut by the left image edge: half its points are outside the image; the growth recovers them
    bearing = math.radians(CAMERA_HFOV / 2 * 180 / math.pi)
    centre = np.array(cxy) + 2.5 * np.array([math.cos(bearing), math.sin(bearing)])
    table = solid(*centre, L, S, yaw_deg=math.degrees(bearing) + 90.0, height=0.72)
    pts = g.cut(scan([table]))
    _, in_image = CAM.project(world_to_camera(pts, BASE, Q0))
    seed = pts[in_image]
    assert 0.3 * len(pts) < len(seed) < 0.7 * len(pts)
    grown = g.grow(seed, pts, radius, z_max, cam_xy=cxy)
    long_axis = math.degrees(bearing) + 90.0
    assert _extent_along(seed[:, :2], long_axis) < 0.75
    assert _extent_along(grown[:, :2], long_axis) >= L - 0.1
    assert len(grown) == len(pts)
    assert {tuple(p) for p in seed.tolist()} <= {tuple(p) for p in grown.tolist()}

    # a chair 0.3 m beside the table is not joined (0.3 m > the 0.15 m growth step at this range)
    table = solid(2.375, 0.0, 0.75, 1.2, height=0.72)                  # x 2.0-2.75, y -0.6..0.6
    chair = solid(2.225, 1.125, 0.45, 0.45, height=0.9)                 # y 0.9-1.35
    pts, k = scan([table, chair], which=True)
    keep = (pts[:, 2] >= g.GROUND_Z) & (pts[:, 2] <= g.HEIGHT_CAP)
    pts, k = pts[keep], k[keep]
    grown = g.grow(pts[(k == 0) & (pts[:, 1] < 0.4)], pts, radius, z_max, cam_xy=cxy)
    assert len(grown) == (k == 0).sum() and (grown[:, 1] <= 0.6 + 1e-9).all()

    # the growth step follows the range: max(0.15, 1.5 x the 1° point spacing)
    assert g.grow_eps(1.0) == 0.15 and g.grow_eps(5.7) == 0.15
    assert g.grow_eps(8.0) == pytest.approx(1.5 * 8.0 * math.radians(1.0))
    pair = lambda x: np.array([[x, 0.0, 0.4], [x, 0.18, 0.4]])
    assert len(g.grow(pair(8.0)[:1], pair(8.0), 0.6, 1.0, cam_xy=(0.0, 0.0))) == 2
    assert len(g.grow(pair(2.0)[:1], pair(2.0), 0.6, 1.0, cam_xy=(0.0, 0.0))) == 1
    # bounded by the measuring radius around the seed's median and by height + 0.3 m
    line = np.array([[2.0, k / 10, 0.4] for k in range(20)])
    assert len(g.grow(line[:1], line, 0.6, 1.0, cam_xy=cxy)) == 7          # y 0.0 .. 0.6
    tall = np.array([[2.0, 0.0, 0.4], [2.0, 0.1, 0.8], [2.0, 0.2, 1.3]])
    assert len(g.grow(tall[:1], tall, 0.6, 1.0, cam_xy=cxy)) == 2


def test_implausible_growth_falls_back_to_the_seed():
    # a trash can between two cabinets with flush fronts: the growth runs along the fronts to the measuring
    # radius (0.6 m each way), wider than 2L + 0.5 = 1.1 m, so the seed (the can's own face) is used
    prior = vocabulary.size_of("trash_can")
    can = solid(2.15, 0.0, 0.3, 0.3, height=0.45)
    left, right = solid(2.225, 0.6, 0.45, 0.9, height=1.2), solid(2.225, -0.6, 0.45, 0.9, height=1.2)
    box = bbox(can)
    s = g.observe(box, rect_mask(box), scan([can, left, right], floor=True), BASE, Q0, CAM, "trash_can")
    assert s.supported and not s.fallback
    assert s.footprint == g.footprint(s.points_xy, s.cam_xy, prior)
    assert g.plausible(s.footprint, prior) and covers(s.footprint, can)
    assert (np.abs(s.points_xy[:, 1]) <= 0.15 + 1e-9).all()               # the seed alone
    pts = g.cut(scan([can, left, right]))
    grown = g.grow(pts[np.abs(pts[:, 1]) < 0.1], pts, objects.measure_radius("trash_can"), 0.75,
                   cam_xy=s.cam_xy)
    assert not g.plausible(g.footprint(grown[:, :2], s.cam_xy, prior), prior)

    # documented limit (reported): with the growth bounded by the measuring radius and the sides clamped to
    # MAX_SIZE, a kind with 2L + 0.5 >= 3.0 m (L >= 1.25: sofa, bed, ...) is never implausible
    sofa = vocabulary.size_of("sofa")
    assert g.plausible(g.Footprint(3.0, 0.0, (3.0, 3.0), False, (3.0, 0.0, 3.0, 3.0)), sofa)
    assert 2 * objects.measure_radius("sofa") + 2 * g.EDGE_PAD < 2 * sofa[0] + 0.5


# ---- footprint from one view ----
def test_footprint_from_one_view():
    cxy = cam_xy()
    # face-on 0.5 m box: depth completed from the prior
    box = solid(2.25, 0.0, 0.5, 0.5)
    pts = g.cut(scan([box]))
    fp = g.footprint(pts[:, :2], cxy, (0.5, 0.5))
    assert (fp.x, fp.y) == pytest.approx((2.25, 0.0), abs=0.03)
    assert all(0.5 <= v <= 0.62 for v in fp.size)
    assert fp.filled and covers(fp, box)
    ox, oy, osx, osy = fp.observed                                      # the observed face, no depth fill
    assert osx == pytest.approx(2 * g.EDGE_PAD, abs=0.01) and ox == pytest.approx(2.0, abs=0.01)
    assert (oy, osy) == pytest.approx((fp.y, fp.size[1]), abs=1e-4)

    # corner view: both faces seen, the depth is the observed one
    diamond = solid(2.5, 0.0, 0.5, 0.5, yaw_deg=45.0)
    fp = g.footprint(g.cut(scan([diamond]))[:, :2], cxy, (0.5, 0.5))
    assert not fp.filled and fp.observed == (fp.x, fp.y, *fp.size)
    assert covers(fp, diamond) and all(0.707 <= v <= 0.95 for v in fp.size)     # the AABB 0.707 + the pads

    # a 1.8 x 0.9 sofa seen from its short side: the hidden depth is L
    sofa = solid(2.9, 0.0, 1.8, 0.9, height=0.85)
    fp = g.footprint(g.cut(scan([sofa]))[:, :2], cxy, (1.8, 0.9))
    assert fp.filled and covers(fp, sofa)
    assert fp.size[0] == pytest.approx(1.8 + 2 * g.EDGE_PAD, abs=0.01) and fp.size[1] <= 0.9 + 0.1
    # ... and from its long side: the hidden depth is S
    fp = g.footprint(g.cut(scan([solid(2.45, 0.0, 1.8, 0.9, yaw_deg=90.0, height=0.85)]))[:, :2], cxy, (1.8, 0.9))
    assert fp.filled and fp.size[0] == pytest.approx(0.9 + 2 * g.EDGE_PAD, abs=0.01)

    # a sofa turned 30°: the footprint is the axis-aligned box of the rectangle (from the front and face-on)
    for sofa in (solid(3.0, 0.0, 1.8, 0.9, yaw_deg=30.0, height=0.85),
                 solid(*(np.array(cxy) + 3.0 * np.array([math.cos(math.radians(30)), math.sin(math.radians(30))])),
                       1.8, 0.9, yaw_deg=120.0, height=0.85)):
        c = corners_xy(sofa)
        true_size = c.max(axis=0) - c.min(axis=0)
        fp = g.footprint(g.cut(scan([sofa]))[:, :2], cxy, (1.8, 0.9))
        assert covers(fp, sofa)
        assert (fp.x, fp.y) == pytest.approx((sofa[0], sofa[1]), abs=0.08)
        assert np.all(np.array(fp.size) <= true_size + 0.3)         # the edge pads, turned 30°

    # a stray point is trimmed
    pts = g.cut(scan([box]))[:, :2]
    clean = g.footprint(pts, cxy, (0.5, 0.5))
    assert g.footprint(np.vstack([pts, [[2.0, 1.5]]]), cxy, (0.5, 0.5)) == clean
    assert g.footprint(np.vstack([pts, [[3.6, 0.0]]]), cxy, (0.5, 0.5)) == clean

    # sides clamped to [SIZE_MIN, MAX_SIZE]; nothing to measure -> None
    fp = g.footprint(np.array([[2.0, 0.0]]), cxy, (0.05, 0.05))
    assert min(fp.size) >= g.SIZE_MIN and fp.filled
    wide = np.array([[2.0, y] for y in np.linspace(-2.0, 2.0, 80)])
    assert max(g.footprint(wide, cxy, (2.0, 1.0)).size) == g.MAX_SIZE
    assert g.footprint(np.zeros((0, 2)), cxy, (0.5, 0.5)) is None
    assert g.footprint(np.array([[np.nan, 1.0]]), cxy, (0.5, 0.5)) is None


COVER_SIZES = [(0.3, 0.3, 0.45), (0.5, 0.5, 0.5), (1.0, 1.0, 0.8), (2.0, 2.0, 0.8), (0.9, 0.45, 1.2),
               (1.2, 0.4, 0.45), (1.8, 0.9, 0.85), (2.0, 1.0, 0.5), (0.4, 0.3, 0.7)]


def _relative_yaw(yaw, bearing):
    return abs((yaw - bearing + 90) % 180 - 90)          # the long axis against the ray, 0-90°


# The cases where the cover check is allowed to fail, by rule (reported to the controller):
COVER_EXCEPTIONS = {
    # a near-square kind (S > 0.7 L, like the 0.4 x 0.3 air purifier) with its short face toward the camera
    # (the long axis within 30° of the ray): the design's completion rule reads w_obs >= 0.7 L as "the long
    # side faces us" and completes the depth with S, so up to L - S of the far side is outside
    "near-square short face": lambda L, S, h, yaw, rng, bearing: (
        0.7 * L < S < L and _relative_yaw(yaw, bearing) <= 30),
}


def _cover_cases():
    for L, S, h in COVER_SIZES:
        for yaw in range(0, 91, 15):
            for rng in (1.0, 2.0, 3.0, 4.0, 5.0):
                for bearing in (0, 30):
                    yield L, S, h, yaw, rng, bearing


def _cover_case(L, S, h, yaw, rng, bearing):
    rel = math.radians(yaw - bearing)
    centre = rng + L / 2 * abs(math.cos(rel)) + S / 2 * abs(math.sin(rel))   # near side `rng` from the LiDAR
    lidar_xy = np.array(LIDAR_IN_BODY[:2]) + BASE[:2]
    b = math.radians(bearing)
    s = solid(*(lidar_xy + centre * np.array([math.cos(b), math.sin(b)])), L, S, yaw_deg=yaw, height=h)
    pts = g.cut(scan([s]))
    return s, g.footprint(pts[:, :2], cam_xy(), (L, S))


def test_published_footprint_covers_the_object():
    """O12: the published axis-aligned footprint contains the true footprint, for boxes of 0.3-2.0 m, yaws
    0-90° in 15° steps, near sides 1-5 m away, straight ahead and 30° to the left, from the returns of the
    visible faces (the prior is the true size)."""
    failed, excused = [], {reason: 0 for reason in COVER_EXCEPTIONS}
    n = 0
    for case in _cover_cases():
        s, fp = _cover_case(*case)
        n += 1
        assert fp is not None and not math.isnan(fp.x)
        if covers(fp, s):
            continue
        reasons = [reason for reason, rule in COVER_EXCEPTIONS.items() if rule(*case)]
        if reasons:
            excused[reasons[0]] += 1
        else:
            failed.append(case)
    assert n == len(COVER_SIZES) * 7 * 5 * 2 == 630
    assert failed == []
    assert 0 < excused["near-square short face"] <= 8      # the documented limit is real (5 of 630 today)


# ---- fallback ----
def test_ground_ray_fallback():
    h, cy, fy = 0.39, CAM.cy, CAM.fy
    cam_pos, level = np.array([0.358, 0.0, h]), np.eye(3)
    sofa = vocabulary.size_of("sofa")
    # flat floor, level camera: the bottom-centre pixel is h fy / (v - cy) ahead; then S/2 deeper; size [L, L]
    fp = g.ground_ray((300.0, 200.0, 340.0, 300.0), CAM, cam_pos, level, sofa)
    d = h * fy / (300.0 - cy)
    assert (fp.x, fp.y) == pytest.approx((0.358 + d + sofa[1] / 2, 0.0), abs=1e-9)
    assert fp.size == (sofa[0], sofa[0]) and fp.filled
    assert fp.observed[:2] == pytest.approx((0.358 + d, 0.0)) and fp.observed[2:] == (g.SIZE_MIN, g.SIZE_MIN)
    # off-centre pixel: along its ray
    fp = g.ground_ray((100.0, 200.0, 140.0, 300.0), CAM, cam_pos, level, (0.5, 0.5))
    assert math.atan2(fp.y, fp.x - 0.358) == pytest.approx(math.atan2(320.0 - 120.0, fy), abs=1e-9)
    # the box bottom must be >= 3 px above the image's bottom edge (the object's base in view)
    assert g.ground_ray((300.0, 400.0, 340.0, 477.0), CAM, cam_pos, level, (0.5, 0.5)) is not None
    assert g.ground_ray((300.0, 400.0, 340.0, 477.5), CAM, cam_pos, level, (0.5, 0.5)) is None
    assert g.ground_ray((300.0, 400.0, 340.0, 480.0), CAM, cam_pos, level, (0.5, 0.5)) is None
    # at or above the horizon, or beyond DET_RANGE (8 m): nothing
    assert g.ground_ray((300.0, 100.0, 340.0, 240.0), CAM, cam_pos, level, (0.5, 0.5)) is None
    assert g.ground_ray((300.0, 100.0, 340.0, 230.0), CAM, cam_pos, level, (0.5, 0.5)) is None
    assert h * fy / (249.0 - cy) > g.DET_RANGE
    assert g.ground_ray((300.0, 100.0, 340.0, 249.0), CAM, cam_pos, level, (0.5, 0.5)) is None
    # a tilted camera (10° nose down): the optical axis meets the floor h / tan(10°) ahead
    tilted = quat_to_matrix(pitch_q(10.0))
    fp = g.ground_ray((300.0, 200.0, 340.0, 240.0), CAM, cam_pos, tilted, (0.5, 0.5))
    assert fp.observed[0] == pytest.approx(0.358 + h / math.tan(math.radians(10.0)), abs=1e-9)


# ---- vacancy ----
def test_vacancy():
    centre, size, height = (3.0, 0.0), (0.5, 0.5), 0.5
    box, wall = solid(3.0, 0.0, 0.5, 0.5), solid(5.05, 0.0, 0.1, 6.0, height=2.0)
    lidar = lidar_points_world(np.zeros((1, 3)), BASE, Q0)[0]

    def counts(solids, c=centre, sz=size, h=height, drop=None):
        pts = scan(solids, floor=True)
        if drop is not None:
            pts = pts[~drop(pts)]
        return g.vacancy_counts(pts, lidar, c, sz, h)

    n_in, n_through = counts([box, wall])                              # present
    assert n_in >= 3 and not g.is_vacant(n_in, n_through)
    n_in, n_through = counts([wall])                                   # removed, a wall behind
    assert n_in == 0 and n_through >= 5 and g.is_vacant(n_in, n_through)
    # removed behind the Go2 (outside the camera view): the scan is 360°
    back, back_wall = solid(-3.0, 0.0, 0.5, 0.5), solid(-5.05, 0.0, 0.1, 6.0, height=2.0)
    assert not g.is_vacant(*counts([back, back_wall], c=(-3.0, 0.0)))
    assert g.is_vacant(*counts([back_wall], c=(-3.0, 0.0)))
    # an occluder in front: the rays end before the footprint
    occluder = solid(2.0, 0.0, 0.2, 1.2, height=1.5)
    n_in, n_through = counts([occluder, wall])
    assert n_in == 0 and n_through < 5 and not g.is_vacant(n_in, n_through)
    # behind the Go2's own body: those returns are removed (drop_self_points), so no ray passes through
    bearing = lambda p: np.abs(np.arctan2(p[:, 1] - lidar[1], p[:, 0] - lidar[0])) < math.radians(10)
    assert not g.is_vacant(*counts([wall], drop=bearing))
    # rays passing over a low object are not counted: a 0.3 m object about 4 m away, where the -2° ring is
    # above it and the -6° ring already below GROUND_Z (it meets the floor inside the footprint)
    low, low_wall = (4.35, 0.0), solid(6.05, 0.0, 0.1, 6.0, height=2.0)
    n_in, n_through = counts([low_wall], c=low, sz=(0.3, 0.3), h=0.3)
    assert n_through == 0 and not g.is_vacant(n_in, n_through)
    n_in, n_through = counts([low_wall], c=low, sz=(0.3, 0.3), h=1.0)  # the same rays do pass a 1 m one
    assert n_through >= 5
    # the box height is at least VACANCY_MIN_HEIGHT; the footprint is grown by VACANCY_GROW
    near_edge = np.array([[3.0 + 0.25 + 0.09, 0.0, 0.2], [3.0, 0.25 + 0.09, 0.2], [3.0, 0.0, 0.29]])
    assert g.vacancy_counts(near_edge, lidar, centre, size, 0.1)[0] == 3
    assert g.vacancy_counts(near_edge + [0.02, 0.0, 0.0], lidar, centre, size, 0.1)[0] == 2
    assert g.is_vacant(2, 5) and not g.is_vacant(3, 5) and not g.is_vacant(0, 4)
    assert g.vacancy_counts(np.zeros((0, 3)), lidar, centre, size, height) == (0, 0)
    # standing inside the (grown) footprint: nothing passes through it
    assert g.vacancy_counts(scan([wall], floor=True), lidar, (0.3, 0.0), (0.5, 0.5), height)[1] == 0


# ---- observe ----
def test_observe_gives_what_the_tracker_needs():
    prior = vocabulary.size_of("box")
    box = solid(2.25, 0.0, 0.5, 0.5)
    pts = scan([box, solid(5.05, 0.0, 0.1, 6.0, height=2.0)], floor=True)
    px = bbox(box)
    for mask in (rect_mask(px), None):
        s = g.observe(px, mask, pts, BASE, Q0, CAM, "box")
        assert s.supported and not s.fallback
        assert s.cam_xy == cam_xy() and isinstance(s.cam_xy[0], float)
        assert s.points_xy.ndim == 2 and s.points_xy.shape[1] == 2 and len(s.points_xy) >= 10
        assert (np.abs(s.points_xy[:, 0] - 2.25) <= 0.25 + 0.1).all()
        assert (np.abs(s.points_xy[:, 1]) <= 0.25 + 0.1).all()
        assert g.footprint(s.points_xy, s.cam_xy, prior) == s.footprint
        assert (s.footprint.x, s.footprint.y) == pytest.approx((2.25, 0.0), abs=0.05)
        assert covers(s.footprint, box)

    # 2 LiDAR points and a visible bottom edge: the ground-ray fallback, never supported, size [L, L]
    far = solid(3.25, 0.0, 0.5, 0.5)
    on_box = scan([far])
    s = g.observe(bbox(far), None, on_box[:2], BASE, Q0, CAM, "sofa")
    sofa = vocabulary.size_of("sofa")
    assert s.fallback and not s.supported and s.points_xy is None
    assert s.footprint.size == (sofa[0], sofa[0]) and s.footprint.filled
    assert s.footprint.x == pytest.approx(3.0 + sofa[1] / 2, abs=0.15) and s.footprint.y == pytest.approx(0.0, abs=0.05)
    assert s.cam_xy == cam_xy()
    # ... and with the bottom cut by the image edge: no geometry at all
    s = g.observe((300.0, 300.0, 340.0, 479.0), None, on_box[:2], BASE, Q0, CAM, "box")
    assert s.footprint is None and not s.supported and not s.fallback and s.points_xy is None

    # an implausible seed: a wide box over a 2 m wall named trash_can -> no geometry, not supported
    wall = solid(3.05, 0.0, 0.1, 6.0, height=2.0)
    pts = scan([wall], floor=True)
    s = g.observe((150.0, 150.0, 490.0, 300.0), None, pts, BASE, Q0, CAM, "trash_can")
    assert s.footprint is None and not s.supported and not s.fallback and s.points_xy is None

    # a turned base: cam_xy and the footprint follow the pose at the image stamp
    q = yaw_q(90.0)
    turned = solid(0.0, 2.25, 0.5, 0.5)
    s = g.observe(bbox(turned, q=q), None, scan([turned], q=q, floor=True), BASE, q, CAM, "box")
    assert s.supported and s.cam_xy == pytest.approx((0.00003, 0.32715))
    assert (s.footprint.x, s.footprint.y) == pytest.approx((0.0, 2.25), abs=0.05)

    with pytest.raises(ValueError):
        g.observe(px, None, pts, BASE, Q0, CAM, "unicorn")


def test_geometry_imports_neither_cv2_nor_torch():
    code = "import detector.geometry, sys; assert 'cv2' not in sys.modules and 'torch' not in sys.modules"
    env = {**os.environ, "PYTHONPATH": os.pathsep.join([str(ROOT), str(ROOT / "nl_control")])}
    assert subprocess.run([sys.executable, "-c", code], cwd=ROOT, env=env).returncode == 0
