"""シミュレーションの Go2 の中身（ROS にも Gazebo にも依存しない）。

実機の状態表（actions._ERROR_CODE_TO_STATE）に合わせた error_code の遷移、
速度の積分、箱との衝突、箱の LiDAR 点群を持つ。時刻は呼び出し側が渡す。

名前つきの物（SimGo2.objects：box_1・chair_1 など）は、画面のボタンで置く箱（boxes）とは別に持つ。
物も衝突と LiDAR 点群には入るが、/sim/world と clear_boxes には入らない（R22）。
FakeCamera は Gazebo の logical camera と同じ視野（120°、0.1〜8 m）で物を見て、見た物を覚えておく。
"""

import json
import math
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Dict, List, Literal, Optional

import numpy as np

from go2_nl_control.actions import (BALANCE_STAND, CONTENT, DANCE1, HEART, HELLO, LYING,
                                    MOVE, RISE_SIT, SCRAPE, SIT, SITTING, STAND_DOWN,
                                    STAND_UP, STANDING, STOP_MOVE, STRETCH,
                                    state_from_error_code)
from go2_nl_control.perception import PerceptionConfig
from voice_ltl.objects import (CAMERA_FAR, CAMERA_HFOV, CAMERA_IN_BODY, CAMERA_NEAR,
                               DEFAULT_LAYOUT, Obj, wrap_rad)

FOOTPRINT = 0.35      # 体の前後の半分の長さ（本物の Go2 は約 0.70 m）[m]
FOOTPRINT_HALF_WIDTH = 0.16   # 体の左右の半分の幅（本物の Go2 は約 0.31 m）[m]
CONTACT_MARGIN = 0.02         # 体の長方形と箱の隙間がこれより狭くなる動きは打ち切る [m]
MOVE_HOLD_S = 1.0     # Move は送り直しが無ければこの秒数で止まる
GROUND_Z = -0.30      # 機体座標での地面の高さ [m]
POINT_STEP = 0.05     # 点群の間隔 [m]
BOX_HEIGHT = 0.4      # 箱の高さ [m]
GESTURE_S = 3.0
GESTURES = (HELLO, STRETCH, CONTENT, DANCE1, SCRAPE, HEART)
_EPS = 1e-9
_START_CODE = {LYING: 1004, STANDING: 1013, SITTING: 1007}


def _point_segment_dist(p, a, b) -> float:
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)
    t = min(1.0, max(0.0, t))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def _polygon_gap(p, q) -> float:
    """凸な四角形 2 つの隙間。重なっていれば 0（分離軸で調べ、離れていれば頂点と辺の距離の最小）。"""
    for poly in (p, q):
        for i in range(4):
            ax, ay = poly[i]
            bx, by = poly[(i + 1) % 4]
            nx, ny = ay - by, bx - ax
            pp = [nx * x + ny * y for x, y in p]
            qq = [nx * x + ny * y for x, y in q]
            if max(pp) < min(qq) or max(qq) < min(pp):
                break
        else:
            continue
        break
    else:
        return 0.0
    return min(_point_segment_dist(v, b[i], b[(i + 1) % 4])
               for a, b in ((p, q), (q, p)) for v in a for i in range(4))


@dataclass
class Box:
    x: float
    y: float
    size: float = 0.3


@dataclass
class SimObject:
    """名前つきの物。底面は軸に平行な sx × sy の長方形（odom の x 方向・y 方向の長さ）。"""
    id: str
    label: str
    x: float
    y: float
    sx: float
    sy: float


def in_camera_view(pose: tuple[float, float, float], x: float, y: float) -> bool:
    """位置 pose = (x, y, yaw) の Go2 の前カメラ（CAMERA_IN_BODY）から (x, y) が見えるか。

    カメラからの距離が CAMERA_NEAR〜CAMERA_FAR で、向きの差が CAMERA_HFOV/2 以内。
    SimGo2.yaw は折り返さないので、差は (−π, π] に折り返してから比べる。
    """
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    bx, by = CAMERA_IN_BODY[0], CAMERA_IN_BODY[1]
    dx = x - (px + c * bx - s * by)
    dy = y - (py + s * bx + c * by)
    if not CAMERA_NEAR <= math.hypot(dx, dy) <= CAMERA_FAR:
        return False
    return abs(wrap_rad(math.atan2(dy, dx) - yaw)) <= CAMERA_HFOV / 2


class SimGo2:
    def __init__(self, perception: PerceptionConfig = PerceptionConfig(), posture: str = LYING):
        self.cfg = perception
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.error_code = _START_CODE[posture]
        self.collided = False
        self.boxes: List[Box] = []
        self.objects: List[SimObject] = []    # 名前つきの物（boxes とは別。/sim/world・clear_boxes に入らない）
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
            if self.error_code == 1015:          # 歩きながらでも、受け付けたら止まって 1013 に戻る
                self.error_code = 1013
            self._start(api_id, now, GESTURE_S, self.error_code)
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
            # 速度がすべて 0 の Move は実機と同じく歩行ではない（StopMove を待たず 1013）
            self.error_code = 1015 if vel != (0.0, 0.0, 0.0) else 1013
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
        nyaw = self.yaw + wz * dt
        old = self._nearest_box_dist(self.x, self.y, self.yaw)
        new = self._nearest_box_dist(nx, ny, nyaw)   # 回った後の向きで判定する（その場回転でもめり込まない）
        # 隙間が狭いときは、離れる動き（隙間が広がる）以外を止める。すでに重なっていても同じ。
        if new < CONTACT_MARGIN and new <= old and not self._moving_out_of_overlap(nx, ny, old, new):
            self.collided = True
            self._stop_walking(self._last_step if self._last_step is not None else 0.0)
            return
        self.x, self.y = nx, ny
        self.yaw = nyaw
        if new > old:                           # 箱から離れる動きが実際に進んだときだけ解除
            self.collided = False

    def _rects(self) -> List[tuple[float, float, float, float]]:
        """ぶつかる物すべて（箱と名前つきの物）を、軸に平行な長方形 (中心 x, 中心 y, 半幅 x, 半幅 y) で。"""
        return ([(b.x, b.y, b.size / 2.0, b.size / 2.0) for b in self.boxes]
                + [(o.x, o.y, o.sx / 2.0, o.sy / 2.0) for o in self.objects])

    def _moving_out_of_overlap(self, nx: float, ny: float, old: float, new: float) -> bool:
        """すでに重なっている（隙間 0）とき、いちばん近い箱（または物）の中心から遠ざかる動きか。"""
        rects = self._rects()
        if old > 0.0 or new > 0.0 or not rects:
            return False
        bx, by, _, _ = min(rects, key=lambda r: math.hypot(r[0] - self.x, r[1] - self.y))
        return math.hypot(bx - nx, by - ny) > math.hypot(bx - self.x, by - self.y)

    def _nearest_box_dist(self, px: float, py: float, yaw: float) -> float:
        """体（位置 px, py・向き yaw の長方形）と箱・物の隙間の最小値 [m]。重なっていれば 0。"""
        c, s = math.cos(yaw), math.sin(yaw)
        body = [(px + c * u - s * v, py + s * u + c * v)
                for u, v in ((FOOTPRINT, FOOTPRINT_HALF_WIDTH), (FOOTPRINT, -FOOTPRINT_HALF_WIDTH),
                             (-FOOTPRINT, -FOOTPRINT_HALF_WIDTH), (-FOOTPRINT, FOOTPRINT_HALF_WIDTH))]
        best = math.inf
        for x, y, hx, hy in self._rects():
            rect = [(x + hx, y + hy), (x - hx, y + hy), (x - hx, y - hy), (x + hx, y - hy)]
            best = min(best, _polygon_gap(body, rect))
        return best

    # ---- 箱と点群 ----
    def place_box(self, where: Literal["front", "back"], distance: float = 1.2) -> Box:
        sign = 1.0 if where == "front" else -1.0
        box = Box(self.x + sign * distance * math.cos(self.yaw),
                  self.y + sign * distance * math.sin(self.yaw))
        self.boxes.append(box)
        return box

    def clear_boxes(self) -> None:
        """画面のボタンで置いた箱だけをどける（名前つきの物はそのまま）。"""
        self.boxes.clear()
        self.collided = False

    def place_objects(self, layout: Iterable[Obj] = DEFAULT_LAYOUT) -> None:
        """名前つきの物を layout の通りに置き直す（それまでの物は消える。何度呼んでも同じ）。"""
        self.objects[:] = [SimObject(o.id, o.label, o.x, o.y, o.size[0], o.size[1]) for o in layout]

    def cloud_lidar(self) -> np.ndarray:
        """箱と物の4側面（0.05 m 間隔、高さ 0〜0.4 m）を LiDAR 座標の点群にする（N×3）。"""
        rects = self._rects()
        if not rects:
            return np.zeros((0, 3))
        hs = np.arange(0.0, BOX_HEIGHT + _EPS, POINT_STEP)
        world = []
        for x, y, hx, hy in rects:
            ux = np.arange(-hx, hx + _EPS, POINT_STEP)
            uy = np.arange(-hy, hy + _EPS, POINT_STEP)
            edges = ([(x + e, y - hy) for e in ux] + [(x + e, y + hy) for e in ux]
                     + [(x - hx, y + e) for e in uy[1:-1]] + [(x + hx, y + e) for e in uy[1:-1]])
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


class FakeCamera:
    """SimGo2 の前カメラ（in_camera_view）で物を見て、一度見た物を覚えておく。

    いま見えている物は visible True・seen = now、前に見た物は visible False・seen = 最後に見た時刻。
    一度も見ていない物は出さない。並びは id 順。
    """

    def __init__(self) -> None:
        self._last_seen: Dict[str, float] = {}

    def see(self, sim: SimGo2, now: float) -> List[Obj]:
        pose = (sim.x, sim.y, sim.yaw)
        out = []
        for o in sorted(sim.objects, key=lambda k: k.id):
            visible = in_camera_view(pose, o.x, o.y)
            if visible:
                self._last_seen[o.id] = now
            seen = self._last_seen.get(o.id)
            if seen is not None:
                out.append(Obj(o.id, o.label, o.x, o.y, (o.sx, o.sy), visible, seen))
        return out
