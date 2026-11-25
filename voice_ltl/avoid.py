"""UWB 追尾で障害物をよけて進む向きを選ぶ。ROS に依存しない。

LiDAR の点群を機体座標（x 前、y 左）の平面の点にし（obstacle_points）、
Go2 の胴幅に余裕を足した帯を各方向へ伸ばして、ぶつかるまでの空き（free_length）を測る。
リモコンへまっすぐ進めればその方向、ふさがっていれば、道のりが空いている方向のうちリモコンの
方向にいちばん近いものを選ぶ（choose_heading）。左右どちらへよけるかは、一度決めたら
その側を少し優先して、左右に迷わないようにする。
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from go2_nl_control.perception import ObstacleMonitor, PerceptionConfig, remove_ground


@dataclass(frozen=True)
class AvoidConfig:
    half_width: float = 0.30       # 進む帯の左右の半幅（Go2 の胴の半幅 0.16 m + 余裕）[m]
    body_front: float = 0.35       # 機体の中心から前端まで [m]
    lookahead: float = 1.5         # この先まで空いていれば十分とみなす [m]
    margin: float = 0.10           # 進む距離に足す余裕 [m]
    min_free: float = 0.25         # まっすぐ進むときも、これだけは空いていてほしい [m]
    detour_free: float = 0.8       # 道のりが空いた向きが無いとき、仮に進む向きに求める空き [m]
    step: float = math.radians(5)  # 調べる向きの刻み
    max_detour: float = math.radians(110)  # リモコンの方向からこれ以上それる向きは選ばない
    side_penalty: float = math.radians(20)  # 前回と逆の側へよけるときの不利
    self_x: float = 0.40           # 自分の体の点として除く範囲（前後の半分）[m]
    self_y: float = 0.22           # 同じく左右の半分 [m]
    grid: float = 0.05             # 点を間引く格子の大きさ [m]


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def obstacle_points(points_lidar: np.ndarray, perception: PerceptionConfig,
                    cfg: AvoidConfig = AvoidConfig()) -> np.ndarray:
    """LiDAR 座標の点群（N×3）を、障害物になる高さの機体座標の平面の点（M×2）にする。"""
    pts = np.asarray(points_lidar, dtype=np.float64).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]
    if not len(pts):
        return np.zeros((0, 2))
    body = ObstacleMonitor(perception)._to_body(pts)
    if perception.remove_ground:
        body = remove_ground(body)
    keep = ((body[:, 2] >= perception.z_min) & (body[:, 2] <= perception.z_max)
            & ~((np.abs(body[:, 0]) < cfg.self_x) & (np.abs(body[:, 1]) < cfg.self_y)))
    xy = body[keep, :2]
    if not len(xy):
        return np.zeros((0, 2))
    return np.unique(np.round(xy / cfg.grid), axis=0) * cfg.grid


def free_length(points: np.ndarray, heading: float, cfg: AvoidConfig = AvoidConfig()) -> float:
    """向き heading [rad] へ進んだとき、前端が点にぶつかるまでの距離 [m]（何もなければ inf）。"""
    if not len(points):
        return math.inf
    c, s = math.cos(heading), math.sin(heading)
    along = points[:, 0] * c + points[:, 1] * s
    across = -points[:, 0] * s + points[:, 1] * c
    hit = along[(along > 0.0) & (np.abs(across) <= cfg.half_width)]
    if not len(hit):
        return math.inf
    return max(0.0, float(hit.min()) - cfg.body_front)


def choose_heading(points: Optional[np.ndarray], bearing: float, travel: float, side: int = 0,
                   cfg: AvoidConfig = AvoidConfig()) -> Tuple[Optional[float], float, int]:
    """進む向きを選ぶ。返り値は (向き [rad]・進めないなら None, その向きの空き [m], よける側)。

    bearing はリモコンの方向（左が正）、travel は進みたい距離 [m]。
    よける側 side は 0（まっすぐ）・1（左へ）・-1（右へ）。次の呼び出しに渡す。
    """
    if points is None or not len(points):
        return bearing, math.inf, 0
    need = min(cfg.lookahead, max(cfg.min_free, travel + cfg.margin))
    direct = free_length(points, bearing, cfg)
    if direct >= need:
        return bearing, direct, 0
    # まず、リモコンへの道のり（lookahead まで）が空いている向きのうち、リモコンの方向に近いもの。
    # 無ければ、detour_free 以上空いている向きのうち、いちばん空いているもの。
    clear = partial = None
    count = int(cfg.max_detour / cfg.step)
    for k in range(1, count + 1):
        for sign in (1, -1):
            heading = _wrap(bearing + sign * k * cfg.step)
            free = free_length(points, heading, cfg)
            cost = k * cfg.step + (cfg.side_penalty if side and sign != side else 0.0)
            if free >= need:
                if clear is None or cost < clear[0]:
                    clear = (cost, heading, free, sign)
            elif free >= cfg.detour_free:
                if partial is None or (free, -cost) > (partial[2], -partial[0]):
                    partial = (cost, heading, free, sign)
    best = clear or partial
    if best is None:
        return None, direct, side
    return best[1], best[2], best[3]
