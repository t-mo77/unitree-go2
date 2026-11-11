"""環境認識：LiDAR 点群から「前方に障害物があるか」を判定する。

- 点群を機体の座標系（x 前, y 左, z 上）に変換し、前方の箱の中の点を数える。
- 止まる距離と動き出す距離を分ける（ヒステリシス）。境目でガタガタ止まったり動いたりしないため。
- センサーが一定時間届かなければ SENSOR_LOST（安全側に倒して前進を止める）。

LiDAR の取り付け位置・角度は機体ごとに rviz2 で確認して、パラメータで合わせること。
"""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .supervisor import BLOCKED, CLEAR, SENSOR_LOST


@dataclass
class PerceptionConfig:
    stop_distance: float = 0.50     # これより近い点があれば止まる [m]
    resume_distance: float = 0.70   # これより遠くなったら動き出す [m]
    half_width: float = 0.30        # 前方の箱の左右の半幅 [m]（Go2 の胴幅 + 余裕）
    z_min: float = -0.25            # 地面を除くための下限（機体基準）[m]
    z_max: float = 0.40             # 天井などを除くための上限 [m]
    min_points: int = 5             # ノイズで止まらないよう、この数以上で障害物とみなす
    stale_timeout: float = 0.5      # これ以上点群が来なければ SENSOR_LOST [s]
    # LiDAR 座標系 → 機体座標系（並進 [m] と、y 軸まわりの下向き傾き [rad]）
    lidar_x: float = 0.28
    lidar_z: float = 0.10
    lidar_pitch: float = 0.0
    lidar_yaw: float = 0.0
    remove_ground: bool = False    # 実機用。広く支持された、機体より下の床平面だけを除く


class ObstacleMonitor:
    def __init__(self, cfg: PerceptionConfig):
        self.cfg = cfg
        self.state = SENSOR_LOST
        self.nearest: Optional[float] = None
        self._last_stamp: Optional[float] = None

    def _to_body(self, pts: np.ndarray) -> np.ndarray:
        c = self.cfg
        cp, sp = math.cos(c.lidar_pitch), math.sin(c.lidar_pitch)
        cy, sy = math.cos(c.lidar_yaw), math.sin(c.lidar_yaw)
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        out = pts @ (rz @ ry).T
        out[:, 0] += c.lidar_x
        out[:, 2] += c.lidar_z
        return out

    def update(self, points_xyz: np.ndarray, now: float) -> str:
        """点群（N×3, LiDAR 座標系）を受け取り、状態を更新して返す。"""
        self._last_stamp = now
        c = self.cfg
        pts = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
        pts = pts[np.isfinite(pts).all(axis=1)]
        body = self._to_body(pts) if len(pts) else pts
        if c.remove_ground:
            body = remove_ground(body)
        if len(body):
            m = ((body[:, 0] > 0.0) & (np.abs(body[:, 1]) <= c.half_width)
                 & (body[:, 2] >= c.z_min) & (body[:, 2] <= c.z_max))
            front = body[m, 0]
        else:
            front = np.empty(0)

        if len(front) >= c.min_points:
            # 最も近い数点の平均ではなく、min_points 番目に近い点を距離とする（外れ値対策）
            self.nearest = float(np.partition(front, c.min_points - 1)[c.min_points - 1])
        else:
            self.nearest = None

        if self.state == BLOCKED:
            if self.nearest is None or self.nearest > c.resume_distance:
                self.state = CLEAR
        else:
            if self.nearest is not None and self.nearest < c.stop_distance:
                self.state = BLOCKED
            else:
                self.state = CLEAR
        return self.state

    def current(self, now: float) -> str:
        if self._last_stamp is None or now - self._last_stamp > self.cfg.stale_timeout:
            self.state = SENSOR_LOST
        return self.state


def remove_ground(body: np.ndarray) -> np.ndarray:
    """床の近くの点だけを除く。平面を確定できないときは点を残す。

    壁・小さい台・少数点を床と誤認しないよう、傾き・高さ・支持点数・広がりを検査する。
    通路全体や近距離の点を一律に捨てる処理は行わない。
    """
    candidates = body[(np.linalg.norm(body[:, :2], axis=1) < 2.0)
                      & (body[:, 2] < -0.15) & (body[:, 2] > -0.8)]
    if len(candidates) < 100:
        return body
    sample = candidates[np.linspace(0, len(candidates) - 1, min(500, len(candidates)), dtype=int)]
    rng = np.random.default_rng(7)
    best = None
    support = 0
    for _ in range(80):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal /= length
        if normal[2] < 0:
            normal = -normal
        if normal[2] < 0.85:  # 壁は床にしない
            continue
        offset = normal @ a
        if not -0.65 <= offset / normal[2] <= -0.20:
            continue
        mask = np.abs(sample @ normal - offset) < 0.015
        if mask.sum() > support:
            best, support = mask, int(mask.sum())
    if best is None or support < max(60, len(sample) * 0.45):
        return body
    points = sample[best]
    if np.ptp(points[:, 0]) < 0.7 or np.ptp(points[:, 1]) < 0.7:
        return body
    center = points.mean(axis=0)
    _, _, vectors = np.linalg.svd(points - center, full_matrices=False)
    normal = vectors[-1]
    if normal[2] < 0:
        normal = -normal
    offset = normal @ center
    if normal[2] < 0.85 or not -0.65 <= offset / normal[2] <= -0.20:
        return body
    floor = (np.abs(body @ normal - offset) < 0.025) & (body[:, 2] < -0.15)
    return body[~floor]
