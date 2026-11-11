"""Where a detection is and how big: LiDAR points under the image box (spec §11.6; design-detector.md §7.2-§7.5
and §8.6's vacancy test). numpy only: it runs in the app image (numpy 1.26, no cv2) and the detector image (2.x).

All coordinates are odom (the Gazebo world): x forward at the start, y left, z up. The world points are one LiDAR
scan put into odom with the pose at the scan stamp (bridge_core.lidar_points_world, which handles a lying tilt);
the camera pose is the one at the image stamp.

observe() is the whole pipeline for one detection, and its Sighting maps 1:1 onto the tracker's Observation:
  1. cut the world points to GROUND_Z <= z <= HEIGHT_CAP;
  2. project them with the image-stamp pose and keep those in the image beyond PROJ_NEAR;
  3. seed: the points inside the mask eroded by MASK_ERODE_PX, or without a mask inside the box shrunk by
     BOX_SHRINK of its width / height per side;
  4. single-linkage clusters of the seed in odom XY at SEED_EPS;
  5. the nearest substantial cluster (>= max(MIN_CLUSTER_POINTS, MIN_CLUSTER_SHARE x the largest));
  6. grow it over all cut points (360°) at grow_eps(range), within objects.measure_radius(label) of the seed's
     median and up to the vocabulary height + 0.3 m;
  7. if the grown footprint is implausible (a side > 2 L + 0.5), the seed cluster alone; if that is implausible
     too, no geometry;
  8. no substantial cluster: the ground-ray fallback ([L, L], never LiDAR-supported).

The one-view footprint (footprint()) deviates from design-detector.md §7.3 in four places, all so that the
published box covers the object (O12; test_published_footprint_covers_the_object simulates 630 views):
  - the rectangle is fitted along the object's faces (the L-shape closeness criterion of Zhang et al. 2017,
    ties broken by least squares; the ray frame when it scores as well), not along the ray: a corner view then
    includes the hidden far corner, and a face seen at an angle is completed along its normal;
  - the MAD trim never drops a point within the prior's diagonal hypot(L, S) of the median (per axis): the
    returns of an elongated object seen at an angle are lopsided, and 3 MAD + 0.1 m cut its far end;
  - an observed depth is kept (filled False) in two cases: in a corner view (the depth axis is the face axis
    nearer the ray), when the face along the depth axis is seen to its far end (a face lies on the far side,
    the end is seen at <= 60° incidence, or the returns there are as dense as that, which covers a union of
    views) and d_obs >= min(0.6 S, 0.1 m); or, in any view, when d_obs >= 0.6 S also reaches the prior's
    depth for that view (within the far side's pad). Otherwise the prior's depth is used and filled is True.
    A face seen at a grazing angle is sampled only part-way (the last 1° wedge can hide most of a sofa's
    side), so its last return says little about its end. An object smaller than its prior, seen to its far
    end, keeps its own depth (spec §11.6: a box seen from two sides gets its observed depth);
  - the margin is EDGE_PAD on the near side and on sides along which a face lies, and one point spacing along
    the face (range x 1° / cos(incidence), up to 60°) where a face ends; beyond about 2.9 m the spacing exceeds
    EDGE_PAD.
The depth rule is otherwise the design's, along the face axis that is thinner (one face seen) or nearer the ray
(a corner): L if the observed width is < 0.7 L, else S. Its limit: a near-square kind (S > 0.7 L) seen on its
short face gets S as its depth (listed in the test).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from detector.camera import CameraModel, camera_pose, world_to_camera
from voice_ltl import objects, vocabulary

GROUND_Z = 0.05               # m: lower points are the floor (bridge_core.GROUND_Z, R23)
HEIGHT_CAP = 2.5              # m: higher points are not objects of the vocabulary
MASK_ERODE_PX = 2
BOX_SHRINK = (0.10, 0.05)     # of the box width / height, per side
SEED_EPS = 0.25               # m
MIN_CLUSTER_POINTS = 5
MIN_CLUSTER_SHARE = 0.25
EDGE_PAD = 0.05               # m per side: the outermost returns can sit one point spacing inside the true edge
SIZE_MIN = 0.1                # m (the bridge's)
DET_RANGE = 8.0               # m (objects.CAMERA_FAR)
FALLBACK_BOTTOM_PX = 3        # the box bottom at least this far above the image's bottom edge
VACANCY_GROW = 0.1            # m
VACANCY_MIN_HEIGHT = 0.3      # m

MAX_SIZE = objects.MAX_SIZE
LIDAR_STEP = math.radians(1.0)               # the LiDAR's horizontal spacing (1°; gazebo/go2_sensors.urdf.xacro)
GROW_EPS_MIN = 0.15                          # m
GROW_EPS_SPACING = 1.5 * LIDAR_STEP          # 1.5 x the horizontal point spacing, per metre of range
PAD_SPACINGS = 1.1                           # where a face ends: this many point spacings along the face (a bound)
PAD_GAIN_MAX = 2.0                           # ... with the incidence counted up to 60°
FLAT_TOL, FLAT_SPREAD = 0.02, 0.1            # m: points within FLAT_TOL of a side spread FLAT_SPREAD along it = a face
SAME_POINT = 1e-3                            # m: returns closer than this are one spot (the rings on a vertical face)
GROW_Z_MARGIN = 0.3                          # m above the vocabulary height
PLAUSIBLE_GAIN, PLAUSIBLE_MARGIN = 2.0, 0.5  # a side > 2 L + 0.5 m merged with something else
TRIM_MAD, TRIM_MARGIN = 3.0, 0.1             # drop points beyond 3 MAD + 0.1 m of the median (and beyond the diagonal)
DEPTH_SEEN = 0.6                             # an observed depth >= 0.6 S is taken as it is
SHORT_FACE = 0.7                             # an observed width < 0.7 L: the short side faces us, so the depth is L
FACE_STEP_DEG, FACE_FINE_DEG = 1.0, 0.05     # orientation search for the face frame
FACE_D0 = 0.01                               # m: the closeness criterion's floor distance
FACE_TOL = 0.01                              # the ray frame wins when its closeness is within 1 % of the best
VACANT_MAX_IN, VACANT_MIN_THROUGH = 3, 5     # vacant iff n_in < 3 and n_through >= 5
_BLOCK = 512


@dataclass(frozen=True)
class Footprint:
    """An axis-aligned footprint in odom: centre (x, y) and size (sx, sy), each side in [SIZE_MIN, MAX_SIZE]."""

    x: float
    y: float
    size: tuple[float, float]
    filled: bool                                       # the depth came from the vocabulary prior (C2)
    observed: tuple[float, float, float, float]        # (cx, cy, sx, sy) of the observed rectangle, no depth fill


@dataclass(frozen=True, eq=False)
class Sighting:
    """What the tracker needs from one detection (Task 2.2's Observation, with the detector's label and
    confidence). When supported, footprint == footprint(points_xy, cam_xy, vocabulary size)."""

    footprint: Footprint | None    # None: no usable geometry (the seed alone implausible, spec §11.6 step 7)
    supported: bool                # LiDAR-supported; False for the ground-ray fallback and for None
    fallback: bool                 # footprint from ground_ray
    points_xy: np.ndarray | None   # N x 2 odom XY of the picked cluster (grown, or the seed alone when the growth
                                   # was implausible), before the MAD trim; None unless supported
    cam_xy: tuple[float, float]    # camera position in odom at the image stamp


# ---- points ----
def _points(points: np.ndarray, dims: int = 3) -> np.ndarray:
    p = np.asarray(points, dtype=np.float64)
    if p.size == 0:
        return np.zeros((0, dims))
    p = p.reshape(-1, p.shape[-1])[:, :dims]
    return p[np.isfinite(p).all(axis=1)]


def cut(points_world: np.ndarray) -> np.ndarray:
    """The world points (N x 3) with GROUND_Z <= z <= HEIGHT_CAP (missing returns dropped)."""
    p = _points(points_world)
    return p[(p[:, 2] >= GROUND_Z) & (p[:, 2] <= HEIGHT_CAP)]


def erode(mask: np.ndarray, r: int = MASK_ERODE_PX) -> np.ndarray:
    """A boolean image eroded by a (2r + 1)-pixel square (outside the image counts as background)."""
    m = np.asarray(mask, dtype=bool)
    for axis in (0, 1):
        n = m.shape[axis]
        pad = [(0, 0), (0, 0)]
        pad[axis] = (r, r)
        padded = np.pad(m, pad, constant_values=False)
        out = np.ones_like(m)
        for k in range(2 * r + 1):
            window = [slice(None), slice(None)]
            window[axis] = slice(k, k + n)
            out &= padded[tuple(window)]
        m = out
    return m


def seed_points(points_world: np.ndarray, uv: np.ndarray, in_image: np.ndarray, box: Sequence[float],
                mask: np.ndarray | None) -> np.ndarray:
    """The seed of one detection (M x 3): the points (in the image) whose pixel is inside the mask eroded by
    MASK_ERODE_PX, or, without a mask, inside the box (x0, y0, x1, y1) shrunk by BOX_SHRINK per side."""
    p = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    ok = np.asarray(in_image, dtype=bool).reshape(-1)
    if mask is None:
        x0, y0, x1, y1 = (float(v) for v in box)
        dx, dy = BOX_SHRINK[0] * (x1 - x0), BOX_SHRINK[1] * (y1 - y0)
        with np.errstate(invalid="ignore"):
            ok = ok & (uv[:, 0] >= x0 + dx) & (uv[:, 0] <= x1 - dx) & (uv[:, 1] >= y0 + dy) & (uv[:, 1] <= y1 - dy)
        return p[ok]
    m = erode(mask, MASK_ERODE_PX)
    h, w = m.shape
    with np.errstate(invalid="ignore"):
        col, row = np.floor(uv[:, 0]), np.floor(uv[:, 1])
        ok = ok & (col >= 0) & (col < w) & (row >= 0) & (row < h)
    hit = np.zeros(len(p), dtype=bool)
    hit[ok] = m[row[ok].astype(int), col[ok].astype(int)]
    return p[hit]


def _components(xy: np.ndarray, eps: np.ndarray, links: tuple[np.ndarray, np.ndarray] | None = None) -> np.ndarray:
    """Single-linkage components: i and j are linked when |xy_i - xy_j| <= max(eps_i, eps_j) (or listed in
    links). Returns each point's component label, the least index in its component."""
    n = len(xy)
    eps = np.broadcast_to(np.asarray(eps, dtype=np.float64), (n,))
    rows, cols = [], []
    for s in range(0, n, _BLOCK):
        b = xy[s:s + _BLOCK]
        d2 = (b[:, None, 0] - xy[None, :, 0]) ** 2 + (b[:, None, 1] - xy[None, :, 1]) ** 2
        lim = np.maximum(eps[s:s + _BLOCK, None], eps[None, :])
        i, j = np.nonzero(d2 <= lim * lim)
        rows.append(i + s)
        cols.append(j)
    i, j = np.concatenate(rows), np.concatenate(cols)
    if links is not None:
        i, j = np.concatenate([i, links[0], links[1]]), np.concatenate([j, links[1], links[0]])
    label = np.arange(n)
    while True:
        new = label.copy()
        np.minimum.at(new, i, label[j])
        new = new[new]
        if np.array_equal(new, label):
            return label
        label = new


def clusters(points_xy: np.ndarray, eps: float = SEED_EPS) -> list[np.ndarray]:
    """Single-linkage clusters in XY at eps (N x 2, or N x 3 using x and y): index arrays into the input rows,
    each ascending, in the order of their first point. Rows with a non-finite x or y belong to no cluster."""
    p = np.asarray(points_xy, dtype=np.float64)
    if p.size == 0:
        return []
    p = p.reshape(-1, p.shape[-1])[:, :2]
    rows = np.flatnonzero(np.isfinite(p).all(axis=1))
    if len(rows) == 0:
        return []
    label = _components(p[rows], np.full(len(rows), float(eps)))
    _, inv = np.unique(label, return_inverse=True)
    order = np.argsort(inv, kind="stable")
    return [rows[gr] for gr in np.split(order, np.cumsum(np.bincount(inv))[:-1])]


def pick_cluster(points_world: np.ndarray, groups: list[np.ndarray], cam_xy: Sequence[float]) -> np.ndarray | None:
    """The points (K x 3) of the nearest substantial cluster: among the groups with at least
    max(MIN_CLUSTER_POINTS, MIN_CLUSTER_SHARE x the largest) points, the one whose nearest point is nearest
    to the camera in XY. The background loses on distance, a thin sliver in front on size (a big occluder in
    front wins: a documented limit). None when no group qualifies."""
    p = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
    if not groups:
        return None
    need = max(MIN_CLUSTER_POINTS, MIN_CLUSTER_SHARE * max(len(gr) for gr in groups))
    cam = np.asarray(cam_xy, dtype=np.float64)[:2]
    best, best_d = None, math.inf
    for gr in groups:
        if len(gr) < need:
            continue
        d = float(np.hypot(*(p[gr, :2] - cam).T).min())
        if d < best_d:
            best, best_d = gr, d
    return None if best is None else p[best]


def grow_eps(r):
    """The growth step at range r: max(GROW_EPS_MIN, 1.5 x the 1° point spacing) m (0.15 m up to 5.7 m)."""
    out = np.maximum(GROW_EPS_MIN, GROW_EPS_SPACING * np.asarray(r, dtype=np.float64))
    return float(out) if out.ndim == 0 else out


def grow(seed: np.ndarray, points_world: np.ndarray, radius: float, z_max: float, *,
         cam_xy: Sequence[float] | None = None) -> np.ndarray:
    """The seed cluster (K x 3) grown over the cut world points (all of them, 360°) by single linkage at
    grow_eps(range from cam_xy) (GROW_EPS_MIN without cam_xy), within radius (XY) of the seed's median and
    with GROUND_Z <= z <= z_max. Returns the seed's points followed by the points it reached."""
    s = _points(seed)
    p = _points(points_world)
    if len(s) == 0:
        return s
    centre = np.median(s[:, :2], axis=0)
    near = ((np.hypot(*(p[:, :2] - centre).T) <= radius) & (p[:, 2] >= GROUND_Z) & (p[:, 2] <= z_max))
    cand = p[near]
    known = {tuple(r) for r in s.tolist()}
    cand = cand[[tuple(r) not in known for r in cand.tolist()]] if len(cand) else cand
    xy = np.vstack([s[:, :2], cand[:, :2]])
    if cam_xy is None:
        eps = np.full(len(xy), GROW_EPS_MIN)
    else:
        eps = grow_eps(np.hypot(*(xy - np.asarray(cam_xy, dtype=np.float64)[:2]).T))
    k = len(s)
    label = _components(xy, eps, (np.zeros(k - 1, dtype=int), np.arange(1, k)))
    return np.vstack([s, cand[label[k:] == label[0]]])


# ---- footprint ----
def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.hypot(v[0], v[1]))
    return np.array([1.0, 0.0]) if n < 1e-9 else np.asarray(v, dtype=np.float64) / n


def _trim(p: np.ndarray, cam: np.ndarray, floor: float) -> np.ndarray:
    """Drop points beyond max(3 MAD + 0.1 m, floor) of the median, per axis of the ray frame."""
    r = _unit(np.median(p, axis=0) - cam)
    q = (p - cam) @ np.array([r, [-r[1], r[0]]]).T
    dev = np.abs(q - np.median(q, axis=0))
    keep = (dev <= np.maximum(TRIM_MAD * np.median(dev, axis=0) + TRIM_MARGIN, floor)).all(axis=1)
    return p[keep] if keep.any() else p


def _edge_distances(q: np.ndarray, phis: np.ndarray) -> np.ndarray:
    """N x K: each point's distance to the nearest edge of the points' bounding rectangle at each orientation."""
    c, s = np.cos(phis), np.sin(phis)
    u = q[:, :1] * c + q[:, 1:2] * s
    v = -q[:, :1] * s + q[:, 1:2] * c
    du = np.minimum(u.max(axis=0) - u, u - u.min(axis=0))
    dv = np.minimum(v.max(axis=0) - v, v - v.min(axis=0))
    return np.minimum(du, dv)


def _closeness(q: np.ndarray, phis: np.ndarray) -> np.ndarray:
    """L-shape closeness (Zhang et al. 2017): sum of 1 / max(edge distance, FACE_D0). Faces seen as lines
    score highest."""
    return (1.0 / np.maximum(_edge_distances(q, phis), FACE_D0)).sum(axis=0)


def _face_angle(p: np.ndarray, ray_angle: float) -> float:
    """The orientation (rad) of the object's faces: the rectangle whose edges the points lie closest to (the
    closeness, then the least squared edge distance among the orientations that tie on it); the ray frame
    when it scores within FACE_TOL of the best or p is a single spot."""
    q = p - p.mean(axis=0)
    if len(q) < 2 or float(np.ptp(q, axis=0).max()) < 1e-6:
        return ray_angle
    coarse = np.radians(np.arange(0.0, 90.0, FACE_STEP_DEG))
    best = coarse[int(np.argmax(_closeness(q, coarse)))]
    fine = best + np.radians(np.arange(-FACE_STEP_DEG, FACE_STEP_DEG + 1e-9, FACE_FINE_DEG))
    score = _closeness(q, fine)
    top = float(score.max())
    if _closeness(q, np.array([ray_angle]))[0] >= (1.0 - FACE_TOL) * top:
        return ray_angle
    tied = np.flatnonzero(score >= top * (1.0 - 1e-9))
    squares = (_edge_distances(q, fine[tied]) ** 2).sum(axis=0)
    return float(fine[tied[int(np.argmin(squares))]])


def _aabb(e1: np.ndarray, e2: np.ndarray, u0: float, u1: float, v0: float, v1: float) -> tuple[float, ...]:
    c = np.array([u * e1 + v * e2 for u in (u0, u1) for v in (v0, v1)])
    lo, hi = c.min(axis=0), c.max(axis=0)
    mid, size = (lo + hi) / 2.0, np.clip(hi - lo, SIZE_MIN, MAX_SIZE)
    return float(mid[0]), float(mid[1]), float(size[0]), float(size[1])


def footprint(points_xy: np.ndarray, cam_xy: Sequence[float], prior: tuple[float, float]) -> Footprint | None:
    """The axis-aligned footprint from one view (or the union of several) of one object's points (N x 2),
    with the vocabulary size prior (L, S) completing a depth the view cannot see. None without points."""
    p = _points(points_xy, 2)
    if len(p) == 0:
        return None
    big, small = float(max(prior)), float(min(prior))
    cam = np.asarray(cam_xy, dtype=np.float64)[:2]
    p = _trim(p, cam, math.hypot(big, small))
    r_hat = _unit(np.median(p, axis=0) - cam)
    phi = _face_angle(p, math.atan2(r_hat[1], r_hat[0]))
    a, b = np.array([math.cos(phi), math.sin(phi)]), np.array([-math.sin(phi), math.cos(phi)])
    seen = DEPTH_SEEN * small
    thin_a, thin_b = float(np.ptp(p @ a)) < seen, float(np.ptp(p @ b)) < seen
    corner = thin_a == thin_b
    if not corner:                           # one face seen: the depth runs along its normal
        depth = a if thin_a else b
    else:                                    # a corner (or a spot): the axis nearer the ray
        depth = a if abs(a @ r_hat) >= abs(b @ r_hat) else b
    if depth @ r_hat < 0:
        depth = -depth
    lateral = np.array([-depth[1], depth[0]])
    u, v = p @ depth, p @ lateral
    near, d_obs = float(u.min()), float(np.ptp(u))
    w_obs, v_mid = float(np.ptp(v)), float(v.min() + v.max()) / 2.0
    rays = p - cam
    # the near side is a face or a corner of two faces, both sampled on their plane: EDGE_PAD. The other sides
    # get one point spacing where a face ends there (side_pad)
    pad_seen, far_sampled = _side(u, v, rays, lateral, +1)
    pad_lo, pad_hi = side_pad(v, u, rays, depth, -1), side_pad(v, u, rays, depth, +1)
    d_prior = big if w_obs < SHORT_FACE * big else small
    # the observed depth: in a corner view (the depth axis nearer the ray, so the face along it is the more
    # grazing one), that face seen to its far end, also for an object smaller than its prior and for a union of
    # views; or, as for any view, a depth >= 0.6 S that reaches the prior's
    if ((corner and far_sampled and d_obs >= min(seen, FLAT_SPREAD))
            or (d_obs >= seen and d_obs + pad_seen >= d_prior)):
        d, pad_far, filled = d_obs, pad_seen, False
    else:                                    # the far side is hidden (or its face grazed): the prior's depth
        d, pad_far, filled = max(d_prior, d_obs), EDGE_PAD, True
    w_half = max(w_obs, SIZE_MIN) / 2.0
    v0, v1 = v_mid - w_half - pad_lo, v_mid + w_half + pad_hi
    box = _aabb(depth, lateral, near - EDGE_PAD, near + d + pad_far, v0, v1)
    observed = _aabb(depth, lateral, near - EDGE_PAD, near + d_obs + pad_seen, v0, v1) if filled else box
    return Footprint(box[0], box[1], (box[2], box[3]), filled, observed)


def side_pad(along: np.ndarray, across: np.ndarray, rays: np.ndarray, normal: np.ndarray, side: int) -> float:
    """The margin beyond the extreme point of `along` on one side (+1: the maximum, -1: the minimum).
    EDGE_PAD where a face lies along that side (its points spread >= FLAT_SPREAD across within FLAT_TOL of the
    side): its plane was sampled. Where a face ends there, the true end can be up to one point spacing along the
    face beyond the last return: PAD_SPACINGS x range x 1° / cos(incidence), at the extreme point, with the
    face's normal `normal` (the incidence counted up to 60°)."""
    return _side(along, across, rays, normal, side)[0]


def _side(along: np.ndarray, across: np.ndarray, rays: np.ndarray, normal: np.ndarray,
          side: int) -> tuple[float, bool]:
    """side_pad's margin, and whether the returns reach the true end on that side (sampled): a face lies along
    the side, or the face ending there is sampled at least as densely as at 60° incidence. The density is the
    camera's incidence at the extreme point (one view), or the measured gap from the extreme point to its
    nearest other return, at most PAD_GAIN_MAX point spacings (a union of views: another camera may have seen
    that end face-on). Beyond 60° the last return can sit far short of the end (a grazing face)."""
    extreme = along.max() if side > 0 else along.min()
    on = np.flatnonzero(np.abs(along - extreme) <= FLAT_TOL)
    if float(np.ptp(across[on])) >= FLAT_SPREAD:
        return EDGE_PAD, True
    ranges = np.hypot(rays[on, 0], rays[on, 1])
    k = int(np.argmax(ranges))
    if ranges[k] < 1e-9:
        return EDGE_PAD, False
    r_end = float(ranges[k])
    facing = abs(float(rays[on[k]] @ normal)) / r_end
    pad = max(EDGE_PAD, PAD_SPACINGS * LIDAR_STEP * r_end / max(facing, 1.0 / PAD_GAIN_MAX))
    gap = np.hypot(along - along[on[k]], across - across[on[k]])
    gap = gap[gap > SAME_POINT]
    dense = len(gap) > 0 and float(gap.min()) <= PAD_GAIN_MAX * LIDAR_STEP * r_end
    return pad, bool(facing >= 1.0 / PAD_GAIN_MAX or dense)


def plausible(fp: Footprint, prior: tuple[float, float]) -> bool:
    """max side <= 2 L + 0.5 m; otherwise the growth merged with neighbouring geometry (a wall, a second object)."""
    return max(fp.size) <= PLAUSIBLE_GAIN * float(max(prior)) + PLAUSIBLE_MARGIN


def ground_ray(box: Sequence[float], camera: CameraModel, cam_pos: Sequence[float], cam_r: np.ndarray,
               prior: tuple[float, float]) -> Footprint | None:
    """Too few LiDAR points: the ray through the box's bottom-centre pixel meets the floor (z = 0) with the
    camera's world pose; the object is S/2 deeper along the ray, size [L, L] (any orientation), filled.
    Only when the box bottom is >= FALLBACK_BOTTOM_PX above the image's bottom edge (the base in view), the ray
    points below the horizon and the floor point is <= DET_RANGE away. observed is the floor point itself
    (SIZE_MIN square): no extent was seen."""
    x0, _, x1, y1 = (float(v) for v in box)
    if camera.height - y1 < FALLBACK_BOTTOM_PX:
        return None
    pos = np.asarray(cam_pos, dtype=np.float64)
    d = np.asarray(cam_r, dtype=np.float64) @ camera.ray((x0 + x1) / 2.0, y1)
    if d[2] >= -1e-9 or pos[2] <= 0:
        return None
    hit = pos + (-pos[2] / d[2]) * d
    ahead = hit[:2] - pos[:2]
    dist = float(np.hypot(*ahead))
    if dist > DET_RANGE or dist < 1e-9:
        return None
    big, small = float(max(prior)), float(min(prior))
    c = hit[:2] + (small / 2.0) * ahead / dist
    side = min(max(big, SIZE_MIN), MAX_SIZE)
    return Footprint(float(c[0]), float(c[1]), (side, side), True, (float(hit[0]), float(hit[1]), SIZE_MIN, SIZE_MIN))


# ---- vacancy (the tracker's forgetting, design-detector.md §8.6) ----
def vacancy_counts(points_world: np.ndarray, lidar_pos: Sequence[float], centre: Sequence[float],
                   size: Sequence[float], height: float) -> tuple[int, int]:
    """For a footprint grown by VACANCY_GROW and a box height h = max(height, VACANCY_MIN_HEIGHT), from one
    scan (any direction) seen from lidar_pos (odom, 3):
    - n_in: points inside the box with GROUND_Z <= z <= h;
    - n_through: returns beyond the footprint along their ray (XY) whose ray, at the footprint's range (the
      middle of its crossing), is inside [GROUND_Z, h]: the ray went through the place.
    A ray that stops before the footprint, hits the Go2's own body (those returns are removed) or passes over a
    low footprint is no evidence. From inside the footprint nothing passes through."""
    p = _points(points_world)
    lid = np.asarray(lidar_pos, dtype=np.float64)
    cx, cy = float(centre[0]), float(centre[1])
    hx, hy = float(size[0]) / 2.0 + VACANCY_GROW, float(size[1]) / 2.0 + VACANCY_GROW
    h = max(float(height), VACANCY_MIN_HEIGHT)
    if len(p) == 0:
        return 0, 0
    inside = (np.abs(p[:, 0] - cx) <= hx) & (np.abs(p[:, 1] - cy) <= hy)
    n_in = int((inside & (p[:, 2] >= GROUND_Z) & (p[:, 2] <= h)).sum())
    if abs(lid[0] - cx) <= hx and abs(lid[1] - cy) <= hy:
        return n_in, 0
    d = p[:, :2] - lid[:2]
    r = np.hypot(d[:, 0], d[:, 1])
    ok = r > 1e-9
    unit = d[ok] / r[ok, None]
    unit = np.where(np.abs(unit) < 1e-12, 1e-12, unit)
    lo, hi = np.array([cx - hx, cy - hy]) - lid[:2], np.array([cx + hx, cy + hy]) - lid[:2]
    t1, t2 = lo / unit, hi / unit
    t_in, t_out = np.minimum(t1, t2).max(axis=1), np.maximum(t1, t2).min(axis=1)
    rk = r[ok]
    beyond = (t_out >= t_in) & (t_out > 0) & (rk > t_out)
    r_fp = (np.maximum(t_in, 0.0) + t_out) / 2.0
    z_ray = lid[2] + (p[ok, 2] - lid[2]) * r_fp / rk
    n_through = int((beyond & (z_ray >= GROUND_Z) & (z_ray <= h)).sum())
    return n_in, n_through


def is_vacant(n_in: int, n_through: int) -> bool:
    """Geometric evidence that the place is empty: n_in < 3 and n_through >= 5."""
    return n_in < VACANT_MAX_IN and n_through >= VACANT_MIN_THROUGH


# ---- one detection ----
def observe(box: Sequence[float], mask: np.ndarray | None, points_world: np.ndarray, base_pos: Sequence[float],
            base_quat: Sequence[float], camera: CameraModel, label: str) -> Sighting:
    """The geometry of one detection (box (x0, y0, x1, y1) in pixels, mask an image-sized boolean or None)
    from one scan's world points and the base pose at the image stamp. label must be a vocabulary label."""
    entry = vocabulary.VOCABULARY.entry(label)
    if entry is None:
        raise ValueError(f"not a vocabulary label: {label!r}")
    prior = entry.size
    cam_pos, cam_r = camera_pose(base_pos, base_quat)
    cxy = (float(cam_pos[0]), float(cam_pos[1]))
    pts = cut(points_world)
    uv, in_image = camera.project(world_to_camera(pts, base_pos, base_quat))
    seed = seed_points(pts, uv, in_image, box, mask)
    picked = pick_cluster(seed, clusters(seed), cxy)
    if picked is None:
        fp = ground_ray(box, camera, cam_pos, cam_r, prior)
        return Sighting(fp, False, fp is not None, None, cxy)
    grown = grow(picked, pts, objects.measure_radius(label), entry.height + GROW_Z_MARGIN, cam_xy=cxy)
    for part in (grown, picked):
        xy = np.ascontiguousarray(part[:, :2])
        fp = footprint(xy, cxy, prior)
        if fp is not None and plausible(fp, prior):
            return Sighting(fp, True, False, xy, cxy)
    return Sighting(None, False, False, None, cxy)
