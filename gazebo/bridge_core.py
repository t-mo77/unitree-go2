"""Gazebo の Go2 の橋渡しの中身（ROS にも Gazebo にも依存しない）。

アプリからの指令（/api/sport/request の api_id）を、unitree_guide controller の
/control_input（command と lx・ly・rx・ry）に変え、実機と同じ error_code を持つ。
時刻は呼び出し側が渡す。時計は 2 つ：
- now：アプリと同じ時計（橋渡しのノードは実際の時間）。error_code が変わるまでの時間、
  動作中に受け付けない時間、Move の 1 秒の送り直しの期限はこれで数える。アプリは実際の時間で
  待つので（例：StandUp の 3 秒後に BalanceStand）、Gazebo が実時間より遅くても窓口の時間は実機と同じ。
- sim_now：シミュレーションの時間。controller は Gazebo の 1 歩ごとに動くので、command を送る
  間隔はこれで決める。省くと now と同じ（テストや、実時間で進むとき）。

controller の状態の番号・切り替え・速度の倍率は docs/reference/gazebo_feasibility_jazzy.md の表のとおり。
- command は次のメッセージが来るまで controller に残り、新しい状態に入ったときに 0 に戻る。
- 伏せ・立つは、入って 1.8 秒（シミュレーションの時間。Gazebo で測ると 1.9 秒前後）たつまで次の状態へ行かない。
  その前に来た command は残っていて、1.8 秒たった時点で効く。だから、前の command が効く前に
  次の command を送ると上書きして消してしまう。前の command が効く時刻を見積もってから送る。
- Gazebo が遅いと、error_code が先に変わり、体は後から追いつく（command は controller の中で待つ）。
  だから Move の速度は、controller が歩くに入って TROT_WAIT_S 秒たつまで 0 のまま待たせる（Ruling R22）。
- 今の状態を知らせるトピックは無いので、自分が送った command から controller の状態を持つ。
  ただし「歩くに入った」ことだけは、橋渡しのノードが controller のログ（/rosout）で知らせる（trot_entered）。
- 転んだことは、橋渡しのノードが Gazebo の本当の胴の傾きと高さで知らせる（observe、Ruling R38）。
  立っているつもりで転んだら error_code を 1001（伏せ扱い）にし、速度を 0 にする。起き上がらせはしない。
"""

import json
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from go2_nl_control.actions import (BALANCE_STAND, CONTENT, DANCE1, HEART, HELLO, LYING, MOVE,
                                    RISE_SIT, SCRAPE, SIT, STAND_DOWN, STAND_UP, STANDING,
                                    STOP_MOVE, STRETCH, state_from_error_code)
from sim.sim_core import BOX_HEIGHT, FOOTPRINT, Box

# ---- /control_input の command（unitree_guide controller） ----
STAND_CMD = 2    # 伏せ → 立つ
DOWN_CMD = 2     # 立つ → 伏せ（立つと同じ値。今の状態で行き先が変わる）
TROT_CMD = 4     # 立つ → 歩く。歩いている間に送っても状態は変わらない

# controller の状態
PASSIVE = "passive"
FIXED_DOWN = "fixed down"
FIXED_STAND = "fixed stand"
TROTTING = "trotting"
_NEXT = {(PASSIVE, 2): FIXED_DOWN, (FIXED_DOWN, 2): FIXED_STAND, (FIXED_STAND, 2): FIXED_DOWN,
         (FIXED_STAND, 4): TROTTING, (TROTTING, 2): FIXED_STAND}
_ROUTE = {  # (今の状態, 行きたい状態) → 送る command の列
    (PASSIVE, FIXED_STAND): (2, 2), (FIXED_DOWN, FIXED_STAND): (2,), (TROTTING, FIXED_STAND): (2,),
    (FIXED_STAND, FIXED_DOWN): (2,), (TROTTING, FIXED_DOWN): (2, 2),
    (FIXED_STAND, TROTTING): (4,), (TROTTING, TROTTING): (),
}
# 以下 5 つは sim_now の時計
HOLD_S = {FIXED_DOWN: 1.8, FIXED_STAND: 1.8}   # 入ってからこの秒数は次の状態へ行かない
SETTLE_S = 0.2   # 状態が切り替わってから次の command を送るまでの余裕（controller の 1 周期は 5 ms）
RISE_WALK_S = 2.0  # RiseSit：「立つ」を送ってから「歩く」を送るまで
TROT_WAIT_S = 0.5  # 歩くに入ってからこの秒数は、Move の速度を入れず 0 のまま待たせる（Ruling R22）。
                   # 窓口の 1013 は体が歩くに入るより先に出るので、すぐ来た Move を入れると歩き出しでよろける
TROT_REPORT_LATE_S = 1.0  # 歩くに入った知らせ（trot_entered）が来ないとき、見積もりよりこれだけ遅らせる。
                          # 本当の伏せ・立つは 1.8 秒より長い（Gazebo で 1.9 秒前後）ので、見積もりは早めに出る

# ---- 実機の窓口から見た時間（設計書 §9.1 の表。now の時計） ----
STAND_S = 2.0      # StandUp・StandDown・Sit・RiseSit が終わるまで
BALANCE_S = 0.5
GESTURE_S = 3.0
MOVE_HOLD_S = 1.0  # Move は送り直しが無ければこの秒数で止まる
GESTURES = (HELLO, STRETCH, CONTENT, DANCE1, SCRAPE, HEART)
STANDING_CODES = (1002, 1013, 1015)   # この橋渡しが、立っているときに出す error_code

# ---- 転んだとき（Ruling R38。Gazebo の本当の胴の傾きと高さで見る） ----
FALL_TILT = 1.0    # 立っているつもりで |roll| か |pitch| がこれを超えたら転んだ [rad]（歩く揺れは 0.8 まで）
FALL_Z = 0.15      # 立っているつもりで胴の高さがこれ未満なら転んだ [m]（立つと 0.33〜0.345）
UPRIGHT_Z = 0.25   # 胴がこの高さまで上がってから見張る [m]。窓口の 1002 は体が立ち上がりきる前に出るので
FELL = "（Gazebo で転びました）"

# ---- 速度の倍率（StateTrotting の上限） ----
V_X_MAX = 0.4    # ly = 1 で前へ 0.4 m/s
V_Y_MAX = 0.3    # lx = 1 で右へ 0.3 m/s
W_Z_MAX = 0.5    # rx = 1 で右回りに 0.5 rad/s

# ---- LiDAR と箱 ----
LIDAR_IN_BODY = (0.28, 0.0, 0.10)   # base から見た LiDAR（go2_sensors.urdf.xacro・perception.py の既定と同じ）
SELF_HALF_X = 0.40                  # 機体の外形：中心から前後 0.40 m・左右 0.20 m・高さ −0.35〜0.15 m
SELF_HALF_Y = 0.20
SELF_Z = (-0.35, 0.15)
GROUND_Z = 0.05                     # 世界の z がこれ未満の点は地面とみなして除く（Ruling R23）[m]
BOX_DISTANCE = 1.2                  # 箱は Go2 の今の位置から前（後ろ）にこの距離 [m]

_EPS = 1e-6
_API_NAMES = {
    BALANCE_STAND: "BalanceStand", STOP_MOVE: "StopMove", STAND_UP: "StandUp",
    STAND_DOWN: "StandDown", MOVE: "Move", SIT: "Sit", RISE_SIT: "RiseSit", HELLO: "Hello",
    STRETCH: "Stretch", CONTENT: "Content", DANCE1: "Dance1", SCRAPE: "Scrape", HEART: "Heart",
}


def api_name(api_id: int) -> str:
    return f"{_API_NAMES.get(api_id, '?')} {api_id}"


@dataclass(frozen=True)
class ControllerInput:
    """/control_input（control_input_msgs/Inputs）に送る 1 通。"""
    command: int
    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0


def _clip(v: float) -> float:
    return max(-1.0, min(1.0, v)) + 0.0   # + 0.0 で −0.0 を 0.0 にする


def move_input(x: float, y: float, z: float) -> ControllerInput:
    """Move の速度（x 前・y 左 [m/s]、z 左回り [rad/s]）を、歩いている controller への入力にする。"""
    return ControllerInput(TROT_CMD, lx=_clip(-y / V_Y_MAX), ly=_clip(x / V_X_MAX),
                           rx=_clip(-z / W_Z_MAX))


class BridgeCore:
    """指令を受けて、controller に送る入力と error_code を決める。"""

    def __init__(self, now: float, sim_now: Optional[float] = None,
                 reports_trot_entry: bool = False):
        """reports_trot_entry：呼び出し側が controller の本当の「歩くに入った」を trot_entered で
        知らせる（橋渡しのノードは /rosout で知る）。False なら、送った command からの見積もりで数える。"""
        sim_now = now if sim_now is None else sim_now
        self.error_code = 1001        # 起動直後は脱力（伏せ扱い）
        self.notes: List[str] = []
        # (送ってよいシミュレーションの時刻, 入力, 速度だけか)。step 1 回で 1 通だけ出す
        self._queue: List[Tuple[float, ControllerInput, bool]] = []
        self._ctl = PASSIVE           # 送った（送る予定の）command がすべて効いた後の controller の状態
        self._ctl_since = sim_now     # その状態に入る時刻（sim）
        self._next_cmd_t = sim_now    # 次の状態を変える command を送ってよい時刻（sim）
        self._vel = (0.0, 0.0, 0.0)   # 送った（送る予定の）(lx, ly, rx)
        self._vel_sent = (0.0, 0.0, 0.0)   # 本当に送った最後の (lx, ly, rx)
        self._reports_trot_entry = reports_trot_entry
        self._trot_entered: Optional[float] = None   # 知らされた「歩くに入った」時刻（sim）
        self._move_until: Optional[float] = None
        self._busy_until = now
        self._done: Optional[Tuple[float, int]] = None   # (時刻, 終わったときの error_code)
        self._watching = False        # 立って、胴が UPRIGHT_Z まで上がった。転ぶのを見張っている
        self._fallen = False          # 転んだ。この後は StopMove のほか受け付けない

    # ---- 指令 ----
    def handle(self, api_id: int, parameter: str, now: float,
               sim_now: Optional[float] = None) -> None:
        sim = now if sim_now is None else sim_now
        self._advance(now, sim)
        name = api_name(api_id)
        if api_id == STOP_MOVE:                   # いつでも受け付ける（安全側）
            self._stop(sim)
            return
        if self._fallen:
            self._note(f"{name} は受け付けません{FELL}。Gazebo を起動し直してください")
            return
        if now < self._busy_until - _EPS:
            self._note(f"{name} は受け付けません（前の動作の途中）")
            return
        code = self.error_code
        posture = state_from_error_code(code)
        if api_id == STAND_UP and posture == LYING:
            self._go(FIXED_STAND, sim)
            self._finish(now, STAND_S, 1002)
        elif api_id == BALANCE_STAND and code in (1002, 1013):
            self._go(TROTTING, sim)
            self._finish(now, BALANCE_S, 1013)
        elif api_id in (STAND_DOWN, SIT) and posture == STANDING:
            self._stop(sim)
            self._go(FIXED_DOWN, sim)
            self._finish(now, STAND_S, 1004 if api_id == STAND_DOWN else 1007)
            if api_id == SIT:
                self._note(f"{name}（Gazebo では伏せで代用）")
        elif api_id == RISE_SIT and code == 1007:
            self._go(FIXED_STAND, sim)
            self._go(TROTTING, sim + RISE_WALK_S)
            self._finish(now, STAND_S, 1013)
            self._note(f"{name}（Gazebo では伏せからの立ち上がりで代用）")
        elif api_id in GESTURES and posture == STANDING:
            self._stop(sim)
            self._finish(now, GESTURE_S, 1013)
            self._note(f"{name}（Gazebo では再現なし）")
        elif api_id == MOVE and code in (1013, 1015):
            self._move(parameter, now, sim)
        else:
            self._note(f"{name} は受け付けません（error_code {code}）")

    def _move(self, parameter: str, now: float, sim: float) -> None:
        try:
            p = json.loads(parameter)
            x, y, z = (float(p.get(k, 0.0)) for k in ("x", "y", "z"))
        except (ValueError, TypeError, AttributeError):
            x = y = z = math.nan
        if not all(math.isfinite(v) for v in (x, y, z)):
            self._note(f"Move の parameter が読めません: {parameter!r}")
            return
        i = move_input(x, y, z)
        self._set_velocity((i.lx, i.ly, i.rx), sim)
        self._move_until = now + MOVE_HOLD_S
        self.error_code = 1015

    # ---- 時間を進める ----
    def step(self, now: float, sim_now: Optional[float] = None) -> Optional[ControllerInput]:
        """今送るべき controller への入力。無ければ None。"""
        sim = now if sim_now is None else sim_now
        self._advance(now, sim)
        if not self._queue or sim < self._queue[0][0] - _EPS:
            return None
        _, i, velocity_only = self._queue[0]
        if velocity_only:
            if sim < self._walk_from() - _EPS:      # 速度は歩くに入って TROT_WAIT_S 秒たってから（R22）
                return None
            if self._reports_trot_entry and self._trot_entered is None:
                self._note("controller が歩くに入ったことを /rosout で確かめられないので、見積もりで速度を入れます")
        self._queue.pop(0)
        self._vel_sent = (i.lx, i.ly, i.rx)
        return i

    def trot_entered(self, sim_now: float) -> None:
        """controller が本当に「歩く」（trotting）に入った（橋渡しのノードが /rosout で知って呼ぶ）。"""
        self._trot_entered = sim_now

    def observe(self, roll: float, pitch: float, height: float, now: float,
                sim_now: Optional[float] = None) -> None:
        """Gazebo の本当の胴の傾き（roll・pitch [rad]）と高さ [m]（橋渡しのノードが odometry ごとに呼ぶ）。

        立っているつもりのときに |roll| か |pitch| が FALL_TILT を超えるか、高さが FALL_Z 未満になったら
        転んだとみなし、error_code を 1001（伏せ扱い）にして速度を 0 にする（Ruling R38）。
        起き上がらせはしない（利用者が Gazebo を起動し直す）。立つの途中（窓口は 1002 でも胴はまだ低い）と、
        伏せ・おすわりへ下りる途中は見ない。"""
        sim = now if sim_now is None else sim_now
        self._advance(now, sim)
        if self._fallen or not self._believes_standing():
            self._watching = False
            return
        tilted = abs(roll) > FALL_TILT or abs(pitch) > FALL_TILT
        if not self._watching:
            self._watching = height >= UPRIGHT_Z and not tilted
        elif tilted or height < FALL_Z:
            self._fall(roll, pitch, height, sim)

    def _believes_standing(self) -> bool:
        """窓口が立っていて、伏せ・おすわりへ移る途中でもない。"""
        return self.error_code in STANDING_CODES and (
            self._done is None or self._done[1] in STANDING_CODES)

    def _fall(self, roll: float, pitch: float, height: float, sim: float) -> None:
        self._fallen, self._watching = True, False
        self._done = None              # 動作の終わりに立ちの error_code へ戻さない
        self._stop(sim)
        self.error_code = 1001
        self._note(f"伏せ扱い（error_code 1001）にして、速度を 0 にします{FELL}。"
                   f"roll {roll:+.2f}・pitch {pitch:+.2f} rad、胴の高さ {height:.2f} m。"
                   "起き上がらせないので、Gazebo を起動し直してください")

    def _walk_from(self) -> float:
        """Move の速度を入れてよい時刻（sim）：歩くに入って TROT_WAIT_S 秒後（Ruling R22）。"""
        if self._trot_entered is not None:
            return self._trot_entered + TROT_WAIT_S
        late = TROT_REPORT_LATE_S if self._reports_trot_entry else 0.0
        return self._ctl_since + TROT_WAIT_S + late

    def _advance(self, now: float, sim: float) -> None:
        if self._done is not None and now >= self._done[0] - _EPS:
            self.error_code = self._done[1]
            self._done = None
        if self._move_until is not None and now >= self._move_until - _EPS:
            self._stop(sim)

    # ---- 中身 ----
    def _note(self, text: str) -> None:
        self.notes.append(text)

    def _finish(self, now: float, duration: float, code: int) -> None:
        self._done = (now + duration, code)
        self._busy_until = now + duration

    def _stop(self, sim: float) -> None:
        self._move_until = None
        self._set_velocity((0.0, 0.0, 0.0), sim)
        if self.error_code == 1015:
            self.error_code = 1013

    def _drop_queued_velocity(self) -> None:
        self._queue = [q for q in self._queue if not q[2]]

    def _go(self, goal: str, not_before: float) -> None:
        """controller を goal へ移す command を、前の command が効いた後に届く時刻で並べる。
        状態を変える入力は速度 0 で送る。"""
        self._drop_queued_velocity()
        self._vel = (0.0, 0.0, 0.0)
        route = _ROUTE[(self._ctl, goal)]
        if route:
            self._trot_entered = None   # 前の「歩く」の知らせは、次に歩くに入るときには使わない
        t = max(not_before, self._next_cmd_t)
        for cmd in route:
            self._queue.append((t, ControllerInput(cmd), False))
            switch = max(t, self._ctl_since + HOLD_S.get(self._ctl, 0.0))
            self._ctl, self._ctl_since = _NEXT[(self._ctl, cmd)], switch
            t = switch + SETTLE_S
        self._next_cmd_t = t

    def _set_velocity(self, vel: Tuple[float, float, float], sim: float) -> None:
        """歩いている controller の速度を変える。歩くに入って TROT_WAIT_S 秒たつまでは速度 0 のまま
        待たせ、それから入れる（Ruling R22）。command は TROT_CMD のまま送る（残っている 4 を消さない）。"""
        if vel == self._vel or self._ctl != TROTTING:
            return
        self._drop_queued_velocity()
        # 待たせていた速度を捨てた後に controller に残る速度（状態を変える command は速度 0）
        if vel == ((0.0, 0.0, 0.0) if self._queue else self._vel_sent):
            self._vel = vel
            return
        # 状態を変える command を追い越さない。歩くに入る前には step が出さない（_walk_from）
        t = max([sim] + [q[0] for q in self._queue])
        self._queue.append((t, ControllerInput(TROT_CMD, *vel), True))
        self._vel = vel


# ---- LiDAR ----
def drop_self_points(points_lidar: np.ndarray) -> np.ndarray:
    """LiDAR 座標の点群（N×3）から、当たらなかった本（inf・nan）と、機体の外形の中の点を除く。"""
    p = np.asarray(points_lidar, dtype=np.float64).reshape(-1, 3)
    p = p[np.isfinite(p).all(axis=1)]
    body = p + np.asarray(LIDAR_IN_BODY)          # 傾きなしで付けているので足すだけ
    inside = ((np.abs(body[:, 0]) <= SELF_HALF_X) & (np.abs(body[:, 1]) <= SELF_HALF_Y)
              & (body[:, 2] >= SELF_Z[0]) & (body[:, 2] <= SELF_Z[1]))
    return p[~inside]


def drop_ground_and_self_points(points_lidar: np.ndarray, base_z: float,
                                base_quat: Tuple[float, float, float, float]) -> np.ndarray:
    """/utlidar/cloud に出す点（LiDAR 座標、N×3）。drop_self_points の後、地面の点（世界の z が
    GROUND_Z 未満）も除く（Ruling R23）。伏せていると LiDAR が地面に近く、地面の点が perception の
    前方の箱に入って「障害物あり」になるため。base_z・base_quat (w, x, y, z) は Gazebo の本当の
    base の高さと向き（傾いていても地面を地面として除く）。"""
    p = drop_self_points(points_lidar)
    w, x, y, z = base_quat
    up_row = np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])  # 回転行列の 3 行目
    world_z = base_z + (p + np.asarray(LIDAR_IN_BODY)) @ up_row
    return p[world_z >= GROUND_Z]


# ---- 箱 ----
def box_sdf(name: str, box: Box) -> str:
    """Gazebo に出す箱（動かない・ぶつかる）の SDF。高さはにせ Go2 と同じ BOX_HEIGHT。"""
    size = f"{box.size} {box.size} {BOX_HEIGHT}"
    geometry = f"<geometry><box><size>{size}</size></box></geometry>"
    return (f'<?xml version="1.0"?><sdf version="1.6"><model name="{name}"><static>true</static>'
            f'<link name="link"><collision name="collision">{geometry}</collision>'
            f'<visual name="visual">{geometry}<material><ambient>0.8 0.4 0.1 1</ambient>'
            f'<diffuse>0.8 0.4 0.1 1</diffuse></material></visual></link></model></sdf>')


class BoxWorld:
    """Gazebo に出した箱の控え。/sim/world の中身（にせ Go2 と同じ形）を作る。"""

    def __init__(self):
        self._boxes: Dict[str, Box] = {}
        self._count = 0

    def place(self, where: str, x: float, y: float, yaw: float,
              distance: float = BOX_DISTANCE) -> Tuple[str, Box]:
        """Go2 の今の位置 (x, y)・向き yaw から前（"front"）か後ろ（"back"）に置く箱と名前。まだ控えには入れない。"""
        sign = 1.0 if where == "front" else -1.0
        self._count += 1
        return (f"sim_box_{self._count}",
                Box(x + sign * distance * math.cos(yaw), y + sign * distance * math.sin(yaw)))

    def add(self, name: str, box: Box) -> None:
        self._boxes[name] = box

    def clear(self) -> List[str]:
        """控えを空にし、消す箱の名前を返す。"""
        names = list(self._boxes)
        self._boxes.clear()
        return names

    def world(self, x: float, y: float) -> dict:
        """{"boxes": [[x, y, size], ...], "collided": 中心から箱までが FOOTPRINT 未満か}"""
        near = min((_dist_to_box(x, y, b) for b in self._boxes.values()), default=math.inf)
        return {"boxes": [[round(b.x, 3), round(b.y, 3), b.size] for b in self._boxes.values()],
                "collided": bool(near < FOOTPRINT)}


def _dist_to_box(px: float, py: float, b: Box) -> float:
    h = b.size / 2.0
    return math.hypot(max(abs(px - b.x) - h, 0.0), max(abs(py - b.y) - h, 0.0))
