"""The front camera of the detector path: pinhole model, extrinsics and frames (spec §11.4, §11.6;
design-detector.md §7.1). numpy only, so it runs in the app image (numpy 1.26) and the detector image (2.x).

Frames: the Gazebo camera frame is x forward, y left, z up (not the ROS optical frame, which is (-y, -z, x)).
The camera sits on the Go2's base at objects.CAMERA_IN_BODY without rotation (the logical camera's link), the
LiDAR at bridge_core.LIDAR_IN_BODY without rotation, so the LiDAR-to-camera transform is a translation here.
World = odom = the Gazebo world.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from gazebo.bridge_core import LIDAR_IN_BODY, quat_to_matrix
from voice_ltl.objects import CAMERA_IN_BODY

# The D7 ruling (one place; Task 2.9 matches the xacro to these)
CAMERA_W = 640
CAMERA_H = 480
CAMERA_RATE_HZ = 2.0          # sim time, periodic
CAMERA_HFOV = 2.094           # rad (120°), the logical camera's
CAMERA_CLIP = (0.1, 10.0)     # m, the LiDAR's reach
PROJ_NEAR = 0.1               # m: only points farther ahead than this are projected


@dataclass(frozen=True)
class CameraModel:
    """Pinhole intrinsics without distortion: u = cx - fx * y / x, v = cy - fy * z / x (camera frame)."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_hfov(cls, width: int = CAMERA_W, height: int = CAMERA_H, hfov: float = CAMERA_HFOV) -> "CameraModel":
        """Until CameraInfo arrives: fx = fy = (W/2) / tan(hfov/2), cx = W/2, cy = H/2 (Gazebo's square pixels)."""
        f = (width / 2.0) / math.tan(hfov / 2.0)
        return cls(int(width), int(height), f, f, width / 2.0, height / 2.0)

    @classmethod
    def from_k(cls, k: Sequence[float], width: int, height: int) -> "CameraModel":
        """From CameraInfo.k (row-major 3 x 3)."""
        k = [float(v) for v in k]
        if len(k) != 9:
            raise ValueError(f"CameraInfo.k has 9 numbers, got {len(k)}")
        return cls(int(width), int(height), k[0], k[4], k[2], k[5])

    def agrees(self, other: "CameraModel", tol: float = 0.01) -> bool:
        """Same image size, and fx, fy, cx, cy each within tol (relative) of the other's (warn otherwise)."""
        if (self.width, self.height) != (other.width, other.height):
            return False
        return all(abs(a - b) <= tol * max(abs(a), abs(b))
                   for a, b in ((self.fx, other.fx), (self.fy, other.fy), (self.cx, other.cx), (self.cy, other.cy)))

    def project(self, p_cam: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Camera-frame points (N x 3) -> pixels (N x 2) and a mask: x > PROJ_NEAR and inside the image
        (0 <= u < width, 0 <= v < height). Points not beyond PROJ_NEAR get nan pixels."""
        p = np.asarray(p_cam, dtype=np.float64).reshape(-1, 3)
        uv = np.full((len(p), 2), np.nan)
        ahead = np.isfinite(p).all(axis=1) & (p[:, 0] > PROJ_NEAR)
        x = p[ahead, 0]
        uv[ahead, 0] = self.cx - self.fx * p[ahead, 1] / x
        uv[ahead, 1] = self.cy - self.fy * p[ahead, 2] / x
        inside = np.zeros(len(p), dtype=bool)
        u, v = uv[ahead, 0], uv[ahead, 1]
        inside[ahead] = (u >= 0) & (u < self.width) & (v >= 0) & (v < self.height)
        return uv, inside

    def ray(self, u: float, v: float) -> np.ndarray:
        """The unit direction (camera frame) through pixel (u, v)."""
        d = np.array([1.0, (self.cx - float(u)) / self.fx, (self.cy - float(v)) / self.fy])
        return d / np.linalg.norm(d)


def t_cam_lidar(lidar_in_body: Sequence[float] = LIDAR_IN_BODY,
                camera_in_body: Sequence[float] = CAMERA_IN_BODY) -> np.ndarray:
    """4 x 4 transform of LiDAR-frame points into the camera frame. Both sensors sit on the base without
    rotation here, so it is the translation lidar_in_body - camera_in_body (part B passes its own)."""
    t = np.eye(4)
    t[:3, 3] = np.asarray(lidar_in_body, dtype=np.float64) - np.asarray(camera_in_body, dtype=np.float64)
    return t


def camera_pose(base_pos: Sequence[float], base_quat: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
    """The camera's position (3) and rotation R (3 x 3, camera frame -> odom) for the base pose
    (base_quat = (w, x, y, z))."""
    r = quat_to_matrix(tuple(base_quat))
    pos = np.asarray(base_pos, dtype=np.float64) + r @ np.asarray(CAMERA_IN_BODY, dtype=np.float64)
    return pos, r


def world_to_camera(points_world: np.ndarray, base_pos: Sequence[float], base_quat: Sequence[float]) -> np.ndarray:
    """Odom points (N x 3) -> the camera frame (x forward, y left, z up) for the base pose at the image stamp."""
    pos, r = camera_pose(base_pos, base_quat)
    return (np.asarray(points_world, dtype=np.float64).reshape(-1, 3) - pos) @ r
