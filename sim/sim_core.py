"""シミュレーションの Go2 の中身（ROS にも Gazebo にも依存しない）。

実機の状態表（actions._ERROR_CODE_TO_STATE）に合わせた error_code の遷移、
速度の積分、箱との衝突、箱の LiDAR 点群を持つ。時刻は呼び出し側が渡す。
"""

import json
import math
from dataclasses import dataclass
from typing import List, Literal, Optional

import numpy as np

from go2_nl_control.actions import (BALANCE_STAND, CONTENT, DANCE1, HELLO, HEART, LYING,
                                    MOVE, RISE_SIT, SCRAPE, SIT, STAND_DOWN, STAND_UP,
                                    STANDING, STOP_MOVE, STRETCH, SITTING,
                                    state_from_error_code)
from go2_nl_control.perception import PerceptionConfig

FOOTPRINT = 0.35      # 中心から箱までの、これより近づく動きは打ち切る [m]
MOVE_HOLD_S = 1.0     # Move は送り直しが無ければこの秒数で止まる
GROUND_Z = -0.30      # 機体座標での地面の高さ [m]
POINT_STEP = 0.05     # 点群の間隔 [m]
BOX_HEIGHT = 0.4      # 箱の高さ [m]
GESTURE_S = 3.0
GESTURES = (HELLO, STRETCH, CONTENT, DANCE1, SCRAPE, HEART)
_EPS = 1e-9
_START_CODE = {LYING: 1004, STANDING: 1013, SITTING: 1007}


@dataclass
class Box:
    x: float
    y: float
    size: float = 0.3


class SimGo2:
    def __init__(self, perception: PerceptionConfig = PerceptionConfig(), posture: str = LYING):
        self.cfg = perception
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.error_code = _START_CODE[posture]
        self.collided = False
        self.boxes: List[Box] = []
        self.accepted: List[int] = []
        self._vel = (0.0, 0.0, 0.0)
        self._vel_t0 = 0.0          # 速度を設定した時刻
        self._move_until = 0.0      # この時刻で Move が切れる
        self._busy_until = 0.0      # この時刻まで他の指令を受け付けない
        self._pending: Optional[tuple] = None   # (時刻, 到着 error_code)
        self._last_step: Optional[float] = None

    @property
    def posture(self) -> str:
        return state_from_error_code(self.error_code)

    # ---- 指令 ----
    def handle(self, api_id: int, parameter: str, now: float) -> None:
        if api_id == STOP_MOVE:                  # いつでも受け付ける（安全側）
            self._set_vel((0.0, 0.0, 0.0), now)
            if self.error_code == 1015:
                self.error_code = 1013
            self.accepted.append(api_id)
            return
        if now < self._busy_until - _EPS:        # 動作中は受け付けない
            return
        posture = self.posture
        if api_id == STAND_UP and posture == LYING:
            self._start(api_id, now, 1.5, 1002)
        elif api_id == BALANCE_STAND and self.error_code in (1002, 1013):
            self._start(api_id, now, 0.5, 1013)
        elif api_id == STAND_DOWN and posture == STANDING:
            self._set_vel((0.0, 0.0, 0.0), now)
            self._start(api_id, now, 1.5, 1004)
        elif api_id == SIT and posture == STANDING:
            self._set_vel((0.0, 0.0, 0.0), now)
            self._start(api_id, now, 2.0, 1007)
        elif api_id == RISE_SIT and posture == SITTING:
            self._start(api_id, now, 1.5, 1013)
        elif api_id in GESTURES and posture == STANDING:
            self._set_vel((0.0, 0.0, 0.0), now)
            self._start(api_id, now, GESTURE_S, self.error_code if self.error_code != 1015 else 1013)
        elif api_id == MOVE and self.error_code in (1013, 1015):
            try:
                p = json.loads(parameter)
                vel = (float(p.get("x", 0.0)), float(p.get("y", 0.0)), float(p.get("z", 0.0)))
            except (ValueError, TypeError, AttributeError):
                return
            if self._vel == (0.0, 0.0, 0.0):    # 送り直しでは積分の起点を動かさない
                self._vel_t0 = now
            self._vel = vel
            self._move_until = now + MOVE_HOLD_S
            self.error_code = 1015
            self.collided = False
            self.accepted.append(api_id)

    def _start(self, api_id: int, now: float, duration: float, to_code: int) -> None:
        self._pending = (now + duration, to_code)
        self._busy_until = now + duration
        self.accepted.append(api_id)

    def _set_vel(self, vel, now: float) -> None:
        self._vel = vel
        self._vel_t0 = now

    # ---- 時間を進める ----
    def step(self, now: float) -> None:
        prev = self._last_step if self._last_step is not None else now
        self._last_step = now
        if self._pending and now >= self._pending[0] - _EPS:
            self.error_code = self._pending[1]
            self._pending = None
        if self._vel != (0.0, 0.0, 0.0):
            t_end = min(now, self._move_until)
            t_start = max(prev, self._vel_t0)
            if t_end > t_start:
                self._integrate(t_end - t_start)
            if now >= self._move_until - _EPS:
                self._stop_walking(now)

    def _stop_walking(self, now: float) -> None:
        self._set_vel((0.0, 0.0, 0.0), now)
        if self.error_code == 1015:
            self.error_code = 1013

    def _integrate(self, dt: float) -> None:
        vx, vy, wz = self._vel
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        nx = self.x + (vx * c - vy * s) * dt
        ny = self.y + (vx * s + vy * c) * dt
        old = self._nearest_box_dist(self.x, self.y)
        new = self._nearest_box_dist(nx, ny)
        if new < FOOTPRINT and new < old:       # 箱に近づく動きは打ち切る（離れる動きは許す）
            self.collided = True
            self._stop_walking(self._last_step if self._last_step is not None else 0.0)
            return
        self.x, self.y = nx, ny
        self.yaw += wz * dt

    def _nearest_box_dist(self, px: float, py: float) -> float:
        best = math.inf
        for b in self.boxes:
            h = b.size / 2.0
            dx = max(abs(px - b.x) - h, 0.0)
            dy = max(abs(py - b.y) - h, 0.0)
            best = min(best, math.hypot(dx, dy))
        return best

    # ---- 箱と点群 ----
    def place_box(self, where: Literal["front", "back"], distance: float = 1.2) -> Box:
        sign = 1.0 if where == "front" else -1.0
        box = Box(self.x + sign * distance * math.cos(self.yaw),
                  self.y + sign * distance * math.sin(self.yaw))
        self.boxes.append(box)
        return box

    def clear_boxes(self) -> None:
        self.boxes.clear()
        self.collided = False

    def cloud_lidar(self) -> np.ndarray:
        """箱の4側面（0.05 m 間隔、高さ 0〜0.4 m）を LiDAR 座標の点群にする（N×3）。"""
        if not self.boxes:
            return np.zeros((0, 3))
        hs = np.arange(0.0, BOX_HEIGHT + _EPS, POINT_STEP)
        world = []
        for b in self.boxes:
            h = b.size / 2.0
            u = np.arange(-h, h + _EPS, POINT_STEP)
            edges = ([(b.x + e, b.y - h) for e in u] + [(b.x + e, b.y + h) for e in u]
                     + [(b.x - h, b.y + e) for e in u[1:-1]] + [(b.x + h, b.y + e) for e in u[1:-1]])
            for ex, ey in edges:
                for z in hs:
                    world.append((ex, ey, z))
        w = np.array(world)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = w[:, 0] - self.x, w[:, 1] - self.y
        body = np.column_stack([c * dx + s * dy, -s * dx + c * dy, GROUND_Z + w[:, 2]])
        # perception._to_body の逆変換
        k = self.cfg
        body[:, 0] -= k.lidar_x
        body[:, 2] -= k.lidar_z
        cp, sp = math.cos(k.lidar_pitch), math.sin(k.lidar_pitch)
        cy, sy = math.cos(k.lidar_yaw), math.sin(k.lidar_yaw)
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        return body @ (rz @ ry)
