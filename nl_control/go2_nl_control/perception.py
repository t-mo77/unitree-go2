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
