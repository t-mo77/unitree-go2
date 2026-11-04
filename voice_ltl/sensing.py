"""前方・後方のセンサー判定。

nl_control の ObstacleMonitor を前方にそのまま使い、後方は機体座標の点を
(x, y) -> (-x, -y) に反転して、取り付け0の2つ目のモニタに渡す。
判定の中身（ヒステリシス、外れ値対策、途切れ）は前後で同じになる。
"""

from dataclasses import dataclass, replace
from typing import Optional

import numpy as np

from go2_nl_control.perception import ObstacleMonitor, PerceptionConfig
from go2_nl_control.supervisor import CLEAR


@dataclass(frozen=True)
class Reading:
    front: str
    back: str
    front_dist: Optional[float]
    back_dist: Optional[float]

    @property
    def front_blocked(self) -> bool:
        return self.front != CLEAR      # 途切れも「あり」として安全側に倒す

    @property
    def back_blocked(self) -> bool:
        return self.back != CLEAR


class Sensing:
    def __init__(self, cfg: PerceptionConfig):
        self._front = ObstacleMonitor(cfg)
        self._back = ObstacleMonitor(
            replace(cfg, lidar_x=0.0, lidar_z=0.0, lidar_pitch=0.0, lidar_yaw=0.0))

    def update(self, points_lidar: np.ndarray, now: float) -> Reading:
        pts = np.asarray(points_lidar, dtype=np.float64).reshape(-1, 3)
        pts = pts[np.isfinite(pts).all(axis=1)]
        self._front.update(pts, now)
        body = self._front._to_body(pts) if len(pts) else pts
        flipped = body * np.array([-1.0, -1.0, 1.0])
        self._back.update(flipped, now)
        return self.current(now)

    def current(self, now: float) -> Reading:
        return Reading(self._front.current(now), self._back.current(now),
                       self._front.nearest, self._back.nearest)
