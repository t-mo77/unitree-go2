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
  次の command を送ると上書きして消してしまう。伏せは 1・2 のほかを無視するので、伏せの間に 4 が届くと
  伏せたまま動かなくなる（2026-11-11：Mac が重く、伏せが見積もりより長くかかって、4 が 2 を上書きした）。
- Gazebo が遅いと、error_code が先に変わり、体は後から追いつく（command は controller の中で待つ）。
  だから Move の速度は、controller が歩くに入って TROT_WAIT_S 秒たつまで 0 のまま待たせる（Ruling R22）。
  待たせた速度は一度に入れず、実行器の加速度の上限（RELEASE_ACC）で 0 から上げる。待つ間に実行器は加速を
  終えているので、一度に入れると旋回が 0 → 0.5 rad/s に跳ぶ（Task I）。遅くする・止めるのはすぐ入れる。
- 今の状態を知らせるトピックは無い。橋渡しのノードは、controller が /rosout に出す
  "Switched from <a> to <b>" をすべて controller_switched で知らせる（reports_switches=True）。そのときは
  確かめた状態からだけ送る（閉じた制御）：確かめた状態に入った知らせから HOLD_S + SETTLE_S たってから、
  その状態に効く command を 1 通。次はその切り替えの知らせを待ってから。RESEND_S たっても知らせが来なければ、
  確かめた状態から送り直す（待つ時間は送り直すたびに倍。送るのは MAX_SENDS 回まで）。知らせがつもりと違えば、
  知らされた状態から続ける。行きたい状態は StandUp などで決まり、それまで（起動直後）は何も送らない。
  知らせにはログを出した時刻（Log.stamp）も付く。知らせが遅れて、送り直しが切り替わりの後に届いたときは、
  その時刻で分かるので、送り直しが効く次の切り替えを待ち、その間は伏せかもしれないとして 4 を送らず、立ちの
  error_code も返さない。遅れを測ったら、送り直す前にその分も待つ（LAG_FACTOR・LAG_MARGIN_S）。
  それでも行きたい状態から MAX_REVERSALS 回遠ざかったら、あきらめてノートに書く。
  知らせ無し（テストと偽の道）は今までどおり、送った command から状態を見積もって送る。
- 知らせ有りのとき、controller が脱力・伏せにいる（かもしれない）間は、立ち・歩きの error_code（1002・1013・1015）
  を返さない。窓口の時間（STAND_S など）がたっても、立つに入った知らせが来るまで前の code のまま（_refresh_code）。
  BalanceStand・StandDown・Sit は、返す code でなく窓口の code で受け付ける（立つ途中なので）。
- 転んだことは、橋渡しのノードが Gazebo の本当の胴の傾きと高さで知らせる（observe、Ruling R38）。
  立っているつもりで転んだら error_code を 1001（伏せ扱い）にし、速度を 0 にする。起き上がらせはしない。

物（box_1・chair_1）の部品も持つ：logical camera が見た model を世界（= odom）の位置に直し（compose）、
名前で絞り（object_label）、画面のボタンで置いた箱の点を除いて（drop_box_points、BoxWorld.sizing_boxes）
胴が止まっている間（StillGate）の LiDAR の点で底面の大きさを測り（footprint_size）、覚えておく（ObjectMemory）。
/go2/objects の形は voice_ltl/objects.py。
"""

import json
import math
import re
from collections.abc import Sequence
from dataclasses import dataclass, replace
from typing import Dict, List, Optional, Tuple

import numpy as np

from go2_nl_control.actions import (BALANCE_STAND, CONTENT, DANCE1, HEART, HELLO, LYING, MOVE,
                                    RISE_SIT, SCRAPE, SIT, STAND_DOWN, STAND_UP, STANDING,
                                    STOP_MOVE, STRETCH, state_from_error_code)
from sim.sim_core import BOX_HEIGHT, FOOTPRINT, Box
from voice_ltl.objects import DEFAULT_SIZE, ID_RE, MAX_SIZE, Obj, format_message

# ---- /control_input の command（unitree_guide controller） ----
STAND_CMD = 2    # 伏せ → 立つ
DOWN_CMD = 2     # 立つ → 伏せ（立つと同じ値。今の状態で行き先が変わる）
TROT_CMD = 4     # 立つ → 歩く。歩いている間に送っても状態は変わらない

# controller の状態（/rosout の "Switched from <a> to <b>" の名前。ほかに free stand・swing test・balance test）
PASSIVE = "passive"
FIXED_DOWN = "fixed down"
FIXED_STAND = "fixed stand"
TROTTING = "trotting"
_LYING_STATES = (PASSIVE, FIXED_DOWN)   # 知らせ有りなら、この間は立ち・歩きの error_code を返さない
_NEXT = {(PASSIVE, 2): FIXED_DOWN, (FIXED_DOWN, 2): FIXED_STAND, (FIXED_STAND, 2): FIXED_DOWN,
         (FIXED_STAND, 4): TROTTING, (TROTTING, 2): FIXED_STAND}
_ROUTE = {  # (今の状態, 行きたい状態) → 送る command の列（知らせ無しのとき）
    (PASSIVE, FIXED_STAND): (2, 2), (FIXED_DOWN, FIXED_STAND): (2,), (TROTTING, FIXED_STAND): (2,),
    (FIXED_STAND, FIXED_DOWN): (2,), (TROTTING, FIXED_DOWN): (2, 2),
    (FIXED_STAND, TROTTING): (4,), (TROTTING, TROTTING): (),
}
# 以下は sim_now の時計
HOLD_S = {FIXED_DOWN: 1.8, FIXED_STAND: 1.8}   # 入ってからこの秒数は次の状態へ行かない
SETTLE_S = 0.2   # 状態が切り替わってから次の command を送るまでの余裕（controller の 1 周期は 5 ms）。
                 # controller はログを出した次の周期に enter で command を 0 に戻すので、知らせの直後には送らない
RISE_WALK_S = 2.0  # RiseSit：「立つ」を送ってから「歩く」を送るまで（知らせ無しのとき）
TROT_WAIT_S = 0.5  # 歩くに入ってからこの秒数は、Move の速度を入れず 0 のまま待たせる（Ruling R22）。
                   # 窓口の 1013 は体が歩くに入るより先に出るので、すぐ来た Move を入れると歩き出しでよろける
# 知らせ有り（橋渡しのノード）のとき
RESEND_S = 2.8   # 送ってこの秒数たっても切り替わりの知らせが来なければ、確かめた状態から送り直す。
                 # 送るのは見積もりで HOLD_S がたった後なので、普通は次の周期に切り替わる。伏せ・立つの 1.8 秒
                 # + 1.0 秒：controller がまだ切り替わっていなければ（伏せ・立つが見積もりより長い、command が
                 # 落ちた）、同じ状態に同じ command をもう一度送るだけ（残っている command を同じ値で上書き）。
                 # 切り替わった後で知らせだけが遅れていると、送り直しは次の状態で効く（2 なら伏せ → 立つ、
                 # 立つ → 伏せ）。それは知らせのログの時刻（logged_at）が送り直しより前なので分かり、
                 # その状態から続ける（controller_switched）。遅れを測った後は LAG_MARGIN_S も使う
LAG_FACTOR = 1.5     # 送り直すのは、送ってから「最後に測った知らせの遅れ × LAG_FACTOR + LAG_MARGIN_S」（実際の
LAG_MARGIN_S = 1.0   # 時間）たった後。遅れは 1 通ごとにばらつく（重いときほど大きく）ので、遅れに比べた余裕も持つ
MAX_SENDS = 3    # 切り替わりの知らせが来ないまま、この回数送ったらあきらめてノートに書く
MAX_REVERSALS = 3    # 行きたい状態から遠ざかる切り替えをこの回数知らされたら、あきらめてノートに書く（行き来を
                     # 止める。数え直すのは、行きたい状態が変わったときと、そこに STABLE_S 着いていたとき）
STABLE_S = 5.0       # 行きたい状態にこの秒数（sim）着いていたら、遠ざかった回数を 0 に戻す
MAX_REPORT_AGE_S = 60.0   # Log.stamp が受けた時刻からこれより離れていたら、この機械の時計ではないとみなす
MASK_NOTE_S = 1.0  # 立ち・歩きの error_code をこの秒数より長く控えたら、ノートに書く（普通に立つときは 0.1 秒）
_SWITCH = re.compile(r"Switched from (.+?) to (.+)")   # UnitreeGuideController::update の RCLCPP_INFO

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

# ---- 待たせた速度の上げ方（R22 の後） ----
# controller が歩くに入ると速度は 0。待たせた Move の速度は、歩いてよくなった時刻（_walk_from）から、
# 前後・左右 0.6 m/s²・旋回 1.0 rad/s²（GridExecutor の GRID_EXEC_CONFIG.acc と同じ）で上げる。
# シミュレーションの時間で数える。Task I：一度に入れていたとき、旋回が 0 から 0.5 rad/s（rx −1.00）に跳び
# （FL1〜FL3・FN1 の橋渡しのログ）、FL3 は立って最初の旋回で転んだ。
RELEASE_ACC = (0.6, 0.6, 1.0)   # (前後 x [m/s²], 左右 y [m/s²], 旋回 z [rad/s²])
_RELEASE_RATE = (RELEASE_ACC[1] / V_Y_MAX, RELEASE_ACC[0] / V_X_MAX, RELEASE_ACC[2] / W_Z_MAX)   # (lx, ly, rx) [1/s]

# ---- LiDAR と箱 ----
LIDAR_IN_BODY = (0.28, 0.0, 0.10)   # base から見た LiDAR（go2_sensors.urdf.xacro・perception.py の既定と同じ）
SELF_HALF_X = 0.40                  # 機体の外形：中心から前後 0.40 m・左右 0.20 m・高さ −0.35〜0.15 m
SELF_HALF_Y = 0.20
SELF_Z = (-0.35, 0.15)
GROUND_Z = 0.05                     # 世界の z がこれ未満の点は地面とみなして除く（Ruling R23）[m]
BOX_DISTANCE = 1.2                  # 箱は Go2 の今の位置から前（後ろ）にこの距離 [m]

# ---- 物（box_1・chair_1）。位置は logical camera、大きさは LiDAR の点で測る ----
SIZE_RADIUS = 0.6          # 物の中心から水平にこの距離より近い点を、その物の点とみなす [m]
SIZE_MIN_POINTS = 10       # これより点が少なければ測らない
# 胴が止まっている間に撮った点群だけで測る（StillGate）。動いていると、点群と位置の時刻のずれで点が広がり、
# 地面の点が GROUND_Z より上に出る（Gazebo で、出した直後に床へ落ちる間の 1 枚で 0.5 m の箱が 1.07 m になった）
SIZE_STILL_RATE = 0.1     # 胴の角速度（roll・pitch・yaw の軸ごと）がこれ以下 [rad/s]
SIZE_STILL_SPEED = 0.05   # 胴の速さがこれ以下 [m/s]
SIZE_STILL_S = 0.3        # 止まってからこの秒数（シミュレーションの時間）たった後に撮った点群だけを使う
SIZE_MIN = 0.1             # 測った大きさの下限 [m]（上限は objects.MAX_SIZE）
SIZE_BOX_MARGIN = 0.05     # 画面のボタンで置いた箱の点を除くとき、箱をこれだけ広げる [m]
SIZE_BOX_GRACE_S = 2.0     # 箱を消すよう頼んでからこの秒数（実際の時間）は、まだ点群に写りうるので点を除き続ける
OBJECT_SPAWN_Z = {"box": 0.25, "chair": 0.45}   # 出す高さ = model の原点の高さ [m]（箱は中心、椅子は座面の上）

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


def parse_switch(text: str) -> Optional[Tuple[str, str]]:
    """controller のログ（/rosout）の 1 行 "Switched from <a> to <b>" → (a, b)。ほかの行は None。"""
    m = _SWITCH.fullmatch(text.strip())
    return (m.group(1), m.group(2)) if m else None


def log_time(now: float, wall: float, stamp: float) -> Optional[float]:
    """/rosout の Log.stamp（ログを出した時刻。use_sim_time でも rcutils はこの機械の時計 = time.time() で付ける）を、
    now の時計（橋渡しのノードは time.monotonic()）の時刻にする。wall は受けたときの time.time()。
    受けた時刻から MAX_REPORT_AGE_S より離れた stamp（この機械の時計ではない）なら None。"""
    age = wall - stamp
    if abs(age) > MAX_REPORT_AGE_S:
        return None
    return now - max(0.0, age)


def _route_cmd(state: str, goal: Optional[str]) -> Optional[int]:
    """知らせ有り：controller が state のとき、goal へ向かって送る command。着いている・goal が無いなら None。
    脱力へは送らない（1 は使わない）。立つからは行き先で 4（歩く）か 2（伏せ）。ほかの状態からは 2：脱力 → 伏せ、
    伏せ → 立つ、歩く・free stand・swing test・balance test → 立つ。4 は state が立つのときだけ。"""
    if goal is None or goal == state or goal == PASSIVE:
        return None
    return TROT_CMD if state == FIXED_STAND and goal == TROTTING else STAND_CMD


def _after(state: str, cmd: int) -> Optional[str]:
    """state の controller が cmd を受けて移る状態。移らない（歩くに 4、伏せ・脱力に 4）なら None。
    知らない状態（free stand など）は 2 で立つへ。"""
    nxt = _NEXT.get((state, cmd))
    if nxt is None and cmd == STAND_CMD and state not in (PASSIVE, FIXED_DOWN, FIXED_STAND, TROTTING):
        return FIXED_STAND
    return nxt


def _distance(state: str, goal: str) -> int:
    """state から goal まで、_route_cmd で送る command の数（着けなければ大きな数）。"""
    n = 0
    while state != goal and n < 5:
        cmd = _route_cmd(state, goal)
        nxt = None if cmd is None else _after(state, cmd)
        if nxt is None:
            break
        state, n = nxt, n + 1
    return n if state == goal else 99


class BridgeCore:
    """指令を受けて、controller に送る入力と error_code を決める。"""

    def __init__(self, now: float, sim_now: Optional[float] = None,
                 reports_switches: bool = False):
        """reports_switches：呼び出し側が controller の状態の切り替えをすべて controller_switched で知らせる
        （橋渡しのノードは /rosout で知る）。True なら確かめた状態からだけ次の command を送り（閉じた制御）、
        controller が脱力・伏せの間は立ち・歩きの error_code を返さない。False（テストと偽の道）なら、
        今までどおり送った command からの見積もりで送る。"""
        sim_now = now if sim_now is None else sim_now
        self._code = 1001             # 窓口の error_code（起動直後は脱力 = 伏せ扱い）。実機の時間で変わる
        self.error_code = 1001        # 返す error_code（_refresh_code）。知らせ無しなら _code と同じ
        self.notes: List[str] = []
        # (送ってよいシミュレーションの時刻, 入力, 速度だけか)。step 1 回で 1 通だけ出す。知らせ有りなら速度だけ
        self._queue: List[Tuple[float, ControllerInput, bool]] = []
        # 知らせ無し：送った（送る予定の）command がすべて効いた後の controller の状態。
        # 知らせ有り：行きたい状態。None なら無い（起動直後・あきらめた・転んだ）ので、何も送らない。
        # 起動直後に脱力を行きたい状態にすると、ほかの送り手の切り替えに 2 を送り返し、伏せ ↔ 立つを行き来させる
        self._ctl: Optional[str] = None if reports_switches else PASSIVE
        self._ctl_since = sim_now     # その状態に入る時刻（sim。知らせ無し）
        self._next_cmd_t = sim_now    # 次の状態を変える command を送ってよい時刻（sim。知らせ無し）
        self._vel = (0.0, 0.0, 0.0)   # 送った（送る予定の）(lx, ly, rx)
        self._vel_sent = (0.0, 0.0, 0.0)   # 本当に送った最後の (lx, ly, rx)
        # 状態を変えた後の最初の速度を、_walk_from から RELEASE_ACC で上げている途中か。
        # _ramp_t：上げる途中で最後に送った時刻（sim）。None なら _walk_from から数える
        self._ramping = False
        self._ramp_t: Optional[float] = None
        self._trot_entered: Optional[float] = None   # 知らされた「歩くに入った」時刻（sim。知らせ無し）
        # 知らせ有り：確かめた controller の状態
        self._reports = reports_switches
        self._conf = PASSIVE          # 知らせで確かめた状態（起動直後の controller は脱力）
        self._conf_since = -math.inf  # その知らせを受けた時刻（sim）
        # 知らせを待っている (command, 送った sim, 行くはずの状態, 送った now)
        self._sent: Optional[Tuple[int, float, str, float]] = None
        self._tries = 0               # 切り替わりの知らせが来ないまま送った回数
        self._lag = 0.0               # 最後に測った知らせの遅れ（受けた now − ログを出した now）
        self._last_logged = -math.inf  # 最後に使った知らせのログの時刻（now の時計）
        self._reversals = 0           # 行きたい状態から遠ざかる切り替えを知らされた回数
        # 知らせが遅れて切り替わりの後に届いた command で、controller がもう移ったかもしれない状態（知らせ待ち）
        self._maybe: Optional[str] = None
        self._lying_code = 1001       # 立ち・歩きの error_code を控える間に返す code（立つ前の code）
        self._masked_since: Optional[float] = None   # 控え始めた時刻（sim）
        self._mask_noted = False
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
        self._handle(api_id, parameter, now, sim)
        self._refresh_code(sim)

    def _handle(self, api_id: int, parameter: str, now: float, sim: float) -> None:
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
        # BalanceStand・StandDown・Sit は窓口の code で見る：controller がまだ伏せで 1001 を返していても、
        # StandUp を終えた（1002）・BalanceStand を終えた（1013）なら立つ途中。知らせ無しなら返す code と同じ
        elif api_id == BALANCE_STAND and self._code in (1002, 1013):
            self._go(TROTTING, sim)
            self._finish(now, BALANCE_S, 1013)
        elif api_id in (STAND_DOWN, SIT) and state_from_error_code(self._code) == STANDING:
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
        self._code = 1015

    # ---- 時間を進める ----
    def step(self, now: float, sim_now: Optional[float] = None) -> Optional[ControllerInput]:
        """今送るべき controller への入力。無ければ None。"""
        sim = now if sim_now is None else sim_now
        self._advance(now, sim)
        out = self._step_reported(now, sim) if self._reports else self._take_queued(sim)
        self._refresh_code(sim)
        return out

    def _take_queued(self, sim: float) -> Optional[ControllerInput]:
        """並べた入力の先頭を、送ってよい時刻なら出す（知らせ無しは状態を変える command も、知らせ有りは速度だけ）。"""
        if not self._queue or sim < self._queue[0][0] - _EPS:
            return None
        _, i, velocity_only = self._queue[0]
        if velocity_only:
            walk_from = self._walk_from()
            if sim < walk_from - _EPS:              # 速度は歩くに入って TROT_WAIT_S 秒たってから（R22）
                return None
            if self._ramping:                       # 歩くに入った後の最初の速度は 0 から上げる
                part = self._ramped(i, sim, walk_from)
                if part != i:
                    if part is None:                # 前に送ってから sim が進んでいない
                        return None
                    self._ramp_t = sim
                    self._vel_sent = (part.lx, part.ly, part.rx)
                    return part
                self._ramping = False
        self._queue.pop(0)
        self._vel_sent = (i.lx, i.ly, i.rx)
        return i

    def _step_reported(self, now: float, sim: float) -> Optional[ControllerInput]:
        """知らせ有り：確かめた状態から行きたい状態へ、command を 1 通ずつ。送った後は、その切り替えの知らせが
        来るまで何も送らない（速度も）。状態が行きたい状態（歩く）に着いていれば、待たせた速度を出す。
        送り直すのは、送って RESEND_S（sim。送り直すたびに倍）と、最後に測った知らせの遅れ × LAG_FACTOR +
        LAG_MARGIN_S（now）の両方がたってから。伏せかもしれない（_maybe）controller には 4 でなく 2 を送る。"""
        if self._sent is not None:
            cmd, at, expected, at_now = self._sent
            wait = RESEND_S * 2 ** max(0, self._tries - 1)   # 2.8 → 5.6 → 11.2（遅れを知る前の送り直しを減らす）
            if sim < at + wait - _EPS or now < at_now + LAG_FACTOR * self._lag + LAG_MARGIN_S - _EPS:
                return None
            self._sent = None
            if self._toward() is not None:
                if self._tries >= MAX_SENDS:
                    self._give_up(f"controller が {self._conf} から切り替わりません"
                                  f"（command {cmd} を {MAX_SENDS} 回送りました）")
                    return None
                self._note(f"controller が {expected} に切り替わったことを {RESEND_S:g} 秒（シミュレーションの"
                           f"時間）たっても /rosout で確かめられないので、{self._conf} から送り直します")
        cmd = self._toward()
        if cmd is not None:
            if sim < self._conf_since + HOLD_S.get(self._conf, 0.0) + SETTLE_S - _EPS:
                return None
            base = self._conf
            if cmd == TROT_CMD and self._maybe in _LYING_STATES:
                # 伏せかもしれない controller に 4 は送らない。2 なら伏せでも立つでも害が無い（たぶん伏せ → 立つ）
                cmd, base = STAND_CMD, self._maybe
            self._sent = (cmd, sim, _after(base, cmd), now)
            self._tries += 1
            self._vel_sent = (0.0, 0.0, 0.0)        # 状態を変える入力は速度 0
            return ControllerInput(cmd)
        if self._ctl is not None and self._conf == self._ctl and sim >= self._conf_since + STABLE_S - _EPS:
            self._reversals = 0                     # 行きたい状態に落ち着いた：行き来は数え直す
        if self._conf != TROTTING:
            return None
        return self._take_queued(sim)

    def _toward(self) -> Optional[int]:
        """確かめた状態から行きたい状態へ向かう次の command（_route_cmd）。"""
        return _route_cmd(self._conf, self._ctl)

    def _give_up(self, why: str) -> None:
        """送っても行きたい状態へ進まない。送るのをやめ、窓口は確かめた状態に合わせる（脱力・伏せなら
        立つ前の code、立つなら立ち固め 1002、歩くなら 1013）。歩くと確かめてあれば行きたい状態を歩くにして、
        Move の速度は送り続ける（窓口が 1015 なのに何も送らない、にしない）。ほかは行きたい状態を無くし、
        次の StandUp・BalanceStand・StandDown などでまた初めから送る。"""
        self._note(f"{why}。Gazebo を起動し直してください")
        self._ctl = TROTTING if self._conf == TROTTING else None
        self._done, self._sent, self._tries, self._reversals, self._maybe = None, None, 0, 0, None
        self._forget_motion()
        if self._conf in _LYING_STATES:
            if self._code in STANDING_CODES:
                self._code = self._lying_code
        else:
            self._code = 1013 if self._conf == TROTTING else 1002

    def controller_switched(self, state: str, sim_now: float, from_state: Optional[str] = None,
                            now: Optional[float] = None, logged_at: Optional[float] = None) -> None:
        """controller が state（passive・fixed down・fixed stand・trotting など）に切り替わった（橋渡しのノードが
        /rosout の "Switched from <from_state> to <state>" で知って呼ぶ）。sim_now・now は受けた時刻（now を
        省くと sim_now）、logged_at は controller がそのログを出した時刻（now の時計。log_time）。
        知らせ有りなら、ここからの command はこの状態から決める。知らせ無しなら、歩くに入った時刻だけを使う。

        最後に送った command が、このログより後に出ていれば（知らせが遅れて、送り直しが切り替わりの後に
        届いた）、その command は state で効く：state から送り直さず、その切り替えを待つ。"""
        if not self._reports:
            if state == TROTTING:
                self._trot_entered = sim_now
            return
        now = sim_now if now is None else now
        if logged_at is not None:
            logged_at = min(logged_at, now)
            if logged_at <= self._last_logged + _EPS:
                return    # 前に使った知らせより前のログ（同じ知らせが二度・順が入れ替わった）。logged_at は受けた
                          # 時刻を超えないので、この機械の時計が飛んでも、落とすのはそのとき届く途中の知らせだけ
            self._last_logged, self._lag = logged_at, now - logged_at
        if state == self._conf and from_state in (None, state):
            return                                     # もう知っている（trot_entered と重なったときなど）
        was, goal, sent = self._conf, self._ctl, self._sent
        prev = was if from_state is None else from_state
        self._conf, self._conf_since = state, sim_now
        self._sent, self._tries, self._maybe = None, 0, None
        if sent is not None and logged_at is not None and logged_at < sent[3] - _EPS:
            nxt = _after(state, sent[0])               # 送った command は state に届いた
            if nxt is not None:
                self._sent, self._tries, self._maybe = (sent[0], sent[1], nxt, sent[3]), 1, nxt
            self._note(f"command {sent[0]} は、controller が {state} に切り替わった後に届きました（/rosout の知らせが"
                       f" {self._lag:.1f} 秒遅れています）。"
                       + (f"{nxt} に切り替わるのを待ちます" if nxt else f"{state} から続けます"))
        elif sent is None:
            self._note(f"controller が {prev} から {state} に切り替わりました"
                       "（橋渡しが待っていた切り替えではありません）")
        elif state != sent[2]:
            self._note(f"controller は {state} に切り替わりました（{sent[2]} のはずでした）。"
                       f"{state} から続けます")
        if goal is not None and _distance(state, goal) > _distance(prev, goal):
            self._reversals += 1
        if state == TROTTING:
            self._ramping, self._ramp_t = True, None   # 歩くに入ったら、速度は 0 から上げる
        elif was == TROTTING:
            self._forget_motion()                      # 橋渡しが伏せさせたのでなくても、歩くのは終わった
        if goal is not None and self._reversals >= MAX_REVERSALS:
            self._give_up(f"controller が {goal} へ進まず、{MAX_REVERSALS} 回戻りました（今は {state}。"
                          "/rosout の知らせが遅れているか、ほかにも /control_input に送っています）")
        self._refresh_code(sim_now)

    def trot_entered(self, sim_now: float) -> None:
        """controller が本当に「歩く」（trotting）に入った（controller_switched(TROTTING) と同じ）。"""
        self.controller_switched(TROTTING, sim_now)

    def observe(self, roll: float, pitch: float, height: float, now: float,
                sim_now: Optional[float] = None) -> None:
        """Gazebo の本当の胴の傾き（roll・pitch [rad]）と高さ [m]（橋渡しのノードが odometry ごとに呼ぶ）。

        立っているつもりのときに |roll| か |pitch| が FALL_TILT を超えるか、高さが FALL_Z 未満になったら
        転んだとみなし、error_code を 1001（伏せ扱い）にして速度を 0 にする（Ruling R38）。
        起き上がらせはしない（利用者が Gazebo を起動し直す）。立つの途中（窓口は 1002 でも胴はまだ低い）と、
        伏せ・おすわりへ下りる途中は見ない。"""
        sim = now if sim_now is None else sim_now
        self._advance(now, sim)
        self._watch(roll, pitch, height, sim)
        self._refresh_code(sim)

    def _watch(self, roll: float, pitch: float, height: float, sim: float) -> None:
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
        self._code = 1001
        if self._reports:
            self._ctl = None           # 起き上がらせない：この後は何も送らない
        self._note(f"伏せ扱い（error_code 1001）にして、速度を 0 にします{FELL}。"
                   f"roll {roll:+.2f}・pitch {pitch:+.2f} rad、胴の高さ {height:.2f} m。"
                   "起き上がらせないので、Gazebo を起動し直してください")

    def _walk_from(self) -> float:
        """Move の速度を入れてよい時刻（sim）：歩くに入って TROT_WAIT_S 秒後（Ruling R22）。"""
        if self._reports:
            return self._conf_since + TROT_WAIT_S
        if self._trot_entered is not None:
            return self._trot_entered + TROT_WAIT_S
        return self._ctl_since + TROT_WAIT_S

    def _ramped(self, target: ControllerInput, sim: float, walk_from: float) -> Optional[ControllerInput]:
        """target へ、最後に送った速度から RELEASE_ACC で近づけた入力（target に着けば target そのもの）。
        各成分、0 の方へ戻す（遅くする・止める）のはすぐ。前に送ってから sim が進まず何も変わらなければ None。"""
        dt = max(0.0, sim - (walk_from if self._ramp_t is None else self._ramp_t))
        out = []
        for cur, goal, rate in zip(self._vel_sent, (target.lx, target.ly, target.rx), _RELEASE_RATE):
            if (goal * cur >= 0.0 and abs(goal) <= abs(cur)) or abs(goal - cur) <= rate * dt + _EPS:
                out.append(goal)
            else:
                out.append(cur + math.copysign(rate * dt, goal - cur))
        if tuple(out) == (target.lx, target.ly, target.rx):
            return target
        if tuple(out) == self._vel_sent:
            return None
        return ControllerInput(TROT_CMD, *out)

    def _advance(self, now: float, sim: float) -> None:
        if self._done is not None and now >= self._done[0] - _EPS:
            self._code = self._done[1]
            self._done = None
        if self._move_until is not None and now >= self._move_until - _EPS:
            self._stop(sim)
        self._refresh_code(sim)

    def _refresh_code(self, sim: float) -> None:
        """返す error_code を決める。知らせ有りで、確かめた controller の状態が脱力・伏せなのに窓口が立ち・歩き
        （1002・1013・1015）なら、それを返さない：伏せ・おすわりへ下りる途中ならその code、ほかは立つ前の code
        （2026-11-11：窓口だけ 1013 になり、アプリは「立って」が終わったと思った）。MASK_NOTE_S より長く
        控えたらノートに書く。"""
        code = self._code
        lying = self._lying() if self._reports else None
        if code in STANDING_CODES and lying is not None:
            down = self._done[1] if self._done is not None and self._done[1] not in STANDING_CODES else None
            self.error_code = self._lying_code if down is None else down
            if self._masked_since is None:
                self._masked_since = sim
            elif (down is None and not self._mask_noted
                  and sim >= self._masked_since + MASK_NOTE_S - _EPS):
                self._mask_noted = True
                self._note(f"{lying}ので、error_code は {self.error_code} に"
                           f"しておきます（{code} にするのは、立ったことを /rosout で確かめてから）")
            return
        self._masked_since, self._mask_noted = None, False
        self._lying_code = 1001 if code in STANDING_CODES else code
        self.error_code = code

    def _lying(self) -> Optional[str]:
        """知らせ有りで、controller が脱力・伏せにいる（かもしれない）なら、そのわけ（ノートの文）。ほかは None。
        確かめた状態が脱力・伏せのときと、知らせが遅れて切り替わりの後に届いた command で伏せへ移ったかもしれない
        とき（_maybe。次の知らせまで）。"""
        if self._conf in _LYING_STATES:
            return f"controller が {self._conf} のままな"
        if self._maybe in _LYING_STATES:
            return f"controller は {self._maybe} へ移っているかもしれない"
        return None

    # ---- 中身 ----
    def _note(self, text: str) -> None:
        self.notes.append(text)

    def _finish(self, now: float, duration: float, code: int) -> None:
        self._done = (now + duration, code)
        self._busy_until = now + duration

    def _stop(self, sim: float) -> None:
        self._move_until = None
        self._set_velocity((0.0, 0.0, 0.0), sim)
        if self._code == 1015:
            self._code = 1013

    def _forget_motion(self) -> None:
        """知らせ有り：歩くのが終わった（controller が歩くを出た・あきらめた）。待たせた速度は捨て、窓口も止める。
        controller が次に歩くに入るときの入力は速度 0 なので、送った速度の控えも 0 から。"""
        self._move_until = None
        self._drop_queued_velocity()
        self._vel = (0.0, 0.0, 0.0)
        if self._code == 1015:
            self._code = 1013

    def _drop_queued_velocity(self) -> None:
        self._queue = [q for q in self._queue if not q[2]]

    def _changing(self) -> bool:
        """controller の状態を変えている途中（その間に送る入力は速度 0）。"""
        if self._reports:
            return self._sent is not None or self._toward() is not None
        return any(not q[2] for q in self._queue)

    def _go(self, goal: str, not_before: float) -> None:
        """controller を goal へ移す。知らせ有りなら行きたい状態を goal にするだけ（送るのは _step_reported）。
        知らせ無しなら、前の command が効いた後に届く時刻で command を並べる。状態を変える入力は速度 0 で送る。"""
        if self._reports:
            if goal != self._conf or self._sent is not None:
                self._drop_queued_velocity()
                self._vel = (0.0, 0.0, 0.0)
            if goal != self._ctl:      # 行き来は行きたい状態ごとに数える。_tries は数え直さない（あきらめると 0）
                self._reversals = 0
            self._ctl = goal
            return
        self._drop_queued_velocity()
        self._vel = (0.0, 0.0, 0.0)
        route = _ROUTE[(self._ctl, goal)]
        if route:
            self._trot_entered = None   # 前の「歩く」の知らせは、次に歩くに入るときには使わない
            self._ramping, self._ramp_t = True, None   # 次に歩くに入ったら、速度は 0 から上げる
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
        if vel == ((0.0, 0.0, 0.0) if self._changing() else self._vel_sent):
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
    """Gazebo に出した箱の控え。/sim/world の中身（にせ Go2 と同じ形）を作る。
    物（box_1・chair_1）の大きさを測るときに除く箱（sizing_boxes）も持つ。"""

    def __init__(self):
        self._boxes: Dict[str, Box] = {}
        self._pending: Dict[str, Box] = {}      # 置くよう頼んで、まだ返事の来ていない箱
        self._gone: List[Tuple[float, Box]] = []   # (消すよう頼んだ時刻, 箱)
        self._count = 0

    def place(self, where: str, x: float, y: float, yaw: float,
              distance: float = BOX_DISTANCE) -> Tuple[str, Box]:
        """Go2 の今の位置 (x, y)・向き yaw から前（"front"）か後ろ（"back"）に置く箱と名前。
        /sim/world の控えにはまだ入れない（add で入れる）が、返事を待つ間に Gazebo が出すこともあるので、
        大きさを測る点からは今から除く（sizing_boxes）。置けなかったら failed を呼ぶ。"""
        sign = 1.0 if where == "front" else -1.0
        self._count += 1
        name = f"sim_box_{self._count}"
        box = Box(x + sign * distance * math.cos(yaw), y + sign * distance * math.sin(yaw))
        self._pending[name] = box
        return name, box

    def add(self, name: str, box: Box) -> None:
        self._pending.pop(name, None)
        self._boxes[name] = box

    def failed(self, name: str) -> None:
        """place した箱を置けなかった。"""
        self._pending.pop(name, None)

    def clear(self, now: float = -math.inf) -> List[str]:
        """控えを空にし、消す箱の名前を返す。Gazebo が消すのは後なので、消した箱の点は now（実際の時間）から
        SIZE_BOX_GRACE_S 秒のあいだ大きさを測る点から除き続ける（now を省くと除き続けない）。"""
        names = list(self._boxes)
        self._gone += [(now, b) for b in self._boxes.values()]
        self._boxes.clear()
        return names

    def sizing_boxes(self, now: float) -> List[Box]:
        """物の大きさを測る点から除く箱：置いた箱、返事を待っている箱、消してまだ SIZE_BOX_GRACE_S 秒たたない箱。"""
        self._gone = [(t, b) for t, b in self._gone if now - t < SIZE_BOX_GRACE_S]
        return [*self._boxes.values(), *self._pending.values(), *(b for _, b in self._gone)]

    def world(self, x: float, y: float) -> dict:
        """{"boxes": [[x, y, size], ...], "collided": 中心から箱までが FOOTPRINT 未満か}"""
        near = min((_dist_to_box(x, y, b) for b in self._boxes.values()), default=math.inf)
        return {"boxes": [[round(b.x, 3), round(b.y, 3), b.size] for b in self._boxes.values()],
                "collided": bool(near < FOOTPRINT)}


def _dist_to_box(px: float, py: float, b: Box) -> float:
    h = b.size / 2.0
    return math.hypot(max(abs(px - b.x) - h, 0.0), max(abs(py - b.y) - h, 0.0))


# ---- 物（box_1・chair_1） ----
def quat_to_matrix(q: Tuple[float, float, float, float]) -> np.ndarray:
    """四元数 (w, x, y, z) の回転行列（3×3）。"""
    w, x, y, z = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def compose(parent_pos: Sequence[float], parent_quat: Tuple[float, float, float, float],
            rel_pos: Sequence[float]) -> Tuple[float, float, float]:
    """親の座標で rel_pos にある点の、世界の位置：parent_pos + R(parent_quat)·rel_pos。
    logical camera では、親 = カメラの世界の位置と向き、rel_pos = model の原点（カメラから見た位置）。"""
    p = np.asarray(parent_pos, dtype=np.float64) + quat_to_matrix(parent_quat) @ np.asarray(
        rel_pos, dtype=np.float64)
    return float(p[0]), float(p[1]), float(p[2])


def object_label(name: str) -> Optional[str]:
    """Gazebo の model の名前が物（box_<n>・chair_<n>、R21）なら "box" か "chair"。
    ground_plane・robot・画面のボタンで置く sim_box_<n> などは None。"""
    m = ID_RE.fullmatch(name) if isinstance(name, str) else None
    return m.group(1) if m else None


def lidar_points_world(points_lidar: np.ndarray, base_pos: Sequence[float],
                       base_quat: Tuple[float, float, float, float]) -> np.ndarray:
    """LiDAR 座標の点（N×3）を世界の座標にする：R(base_quat)·(p + LIDAR_IN_BODY) + base_pos。"""
    p = np.asarray(points_lidar, dtype=np.float64).reshape(-1, 3) + np.asarray(LIDAR_IN_BODY)
    return p @ quat_to_matrix(base_quat).T + np.asarray(base_pos, dtype=np.float64)


def drop_box_points(points_world: np.ndarray, boxes: Sequence[Box],
                    margin: float = SIZE_BOX_MARGIN) -> np.ndarray:
    """世界の点（N×3）から、画面のボタンで置いた箱（向きは世界の軸、高さ BOX_HEIGHT。box_sdf と同じ）の中の点を除く。
    箱は水平に margin、上に margin だけ広げる。footprint_size の前に使う：箱が物の近くにあると、その点で物の
    大きさが膨らみ、set_size が大きい方を残すのでずっと戻らない（近い帯が無くなり「〜まで歩いて」が NO_PATH になる）。"""
    p = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
    keep = np.ones(len(p), dtype=bool)
    for b in boxes:
        h = b.size / 2.0 + margin
        keep &= ~((np.abs(p[:, 0] - b.x) <= h) & (np.abs(p[:, 1] - b.y) <= h)
                  & (p[:, 2] <= BOX_HEIGHT + margin))
    return p[keep]


def footprint_size(points_world: np.ndarray, cx: float, cy: float, radius: float = SIZE_RADIUS,
                   min_points: int = SIZE_MIN_POINTS) -> Optional[Tuple[float, float]]:
    """中心 (cx, cy) から水平に radius より近く、地面より上（世界の z ≥ GROUND_Z）の点で測った底面の大きさ
    (x の幅, y の幅) = (2·max|dx|, 2·max|dy|)。中心は分かっているので、手前の面しか見えなくても測れる。
    点が min_points より少なければ None。"""
    p = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
    d = p[:, :2] - np.array([cx, cy])
    near = d[(np.hypot(d[:, 0], d[:, 1]) < radius) & (p[:, 2] >= GROUND_Z)]
    if len(near) < min_points:
        return None
    sx, sy = 2 * np.abs(near).max(axis=0)
    return float(sx), float(sy)


def object_sdf(obj_id: str, label: str) -> str:
    """Gazebo に出す物（動かない・ぶつかる）の SDF。見た目はぶつかる形と同じ。
    - box：0.5 m 角の立方体。原点は中心（出す高さ 0.25 m）。
    - chair：原点は座面の上（出す高さ 0.45 m）。座面 0.45 × 0.45、脚 4 本、背もたれは −x 側で 0.9 m まで。"""
    if label == "box":
        parts = (("box", (0.0, 0.0, 0.0), (0.5, 0.5, 0.5)),)
        color = "0.65 0.48 0.28 1"    # 段ボールの色（ボタンで置く sim_box の橙と分ける）
    elif label == "chair":
        parts = (("seat", (0.0, 0.0, -0.02), (0.45, 0.45, 0.04)),
                 ("leg_fl", (0.2, 0.2, -0.245), (0.04, 0.04, 0.41)),
                 ("leg_fr", (0.2, -0.2, -0.245), (0.04, 0.04, 0.41)),
                 ("leg_rl", (-0.2, 0.2, -0.245), (0.04, 0.04, 0.41)),
                 ("leg_rr", (-0.2, -0.2, -0.245), (0.04, 0.04, 0.41)),
                 ("back", (-0.205, 0.0, 0.225), (0.04, 0.45, 0.45)))
        color = "0.45 0.3 0.18 1"     # 木の色
    else:
        raise ValueError(f"unknown object label: {label!r}")
    material = f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>"
    shapes = ""
    for name, pos, size in parts:
        pose = "<pose>{:g} {:g} {:g} 0 0 0</pose>".format(*pos)
        geometry = "<geometry><box><size>{:g} {:g} {:g}</size></box></geometry>".format(*size)
        shapes += (f'<collision name="{name}">{pose}{geometry}</collision>'   # link の中で名前は重ねない
                   f'<visual name="{name}_visual">{pose}{geometry}{material}</visual>')
    return (f'<?xml version="1.0"?><sdf version="1.6"><model name="{obj_id}"><static>true</static>'
            f'<link name="link">{shapes}</link></model></sdf>')


class StillGate:
    """大きさを測ってよい点群か。胴が止まっている（角速度が軸ごとに SIZE_STILL_RATE 以下、速さが
    SIZE_STILL_SPEED 以下）間が、点群を撮った時刻の SIZE_STILL_S 前から今まで続いているときだけ。
    時刻はどちらもシミュレーションの時間（odometry と点群の header.stamp）。
    出した直後に床へ落ちる間・立ち上がる間・その場で足踏みする間・歩く間・回る間は測らない
    （set_size は大きい方を残すので、1 枚でも広がった点群を使うとずっと戻らない）。"""

    def __init__(self):
        self.still_since: Optional[float] = None   # 今の「止まっている」が始まった時刻。動いていれば None

    def observe(self, t: float, linear: Sequence[float], angular: Sequence[float]) -> None:
        """odometry 1 通：時刻 t、胴の速度 linear (x, y, z) [m/s] と角速度 angular (x, y, z) [rad/s]。"""
        still = (all(abs(w) <= SIZE_STILL_RATE for w in angular)
                 and math.sqrt(sum(v * v for v in linear)) <= SIZE_STILL_SPEED)   # nan は止まっていない
        if not still:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = t

    def allows(self, stamp: float) -> bool:
        """時刻 stamp に撮った点群で大きさを測ってよいか。"""
        return self.still_since is not None and stamp - self.still_since >= SIZE_STILL_S - _EPS


class ObjectMemory:
    """logical camera が見た物の控え（物は動かないので、見えなくなっても覚えておく。R20）。
    /go2/objects の中身（voice_ltl/objects.py の形、odom = Gazebo の世界）を作る。"""

    def __init__(self):
        self._objects: Dict[str, Obj] = {}
        self._measured: set = set()   # LiDAR で大きさを測った物の id

    def update(self, seen: Sequence[Tuple[str, str, float, float]], now: float) -> None:
        """カメラの 1 枚分：見えた物 (id, label, x, y)。見えた物は visible・seen = now、
        ほかの物は visible False（seen は最後に見た時刻のまま）。初めての物の大きさは DEFAULT_SIZE。"""
        ids = set()
        for obj_id, label, x, y in seen:
            ids.add(obj_id)
            old = self._objects.get(obj_id)
            size = DEFAULT_SIZE if old is None else old.size
            self._objects[obj_id] = Obj(obj_id, label, float(x), float(y), size, True, float(now))
        for obj_id, o in list(self._objects.items()):
            if obj_id not in ids and o.visible:
                self._objects[obj_id] = replace(o, visible=False)

    def set_size(self, obj_id: str, size: Tuple[float, float]) -> None:
        """LiDAR で測った大きさ (x の幅, y の幅) を [SIZE_MIN, MAX_SIZE] に収めて入れる。
        初めて測ったら DEFAULT_SIZE と置き換え（0.45 m の椅子を 0.5 のままにしない）、その後は軸ごとに
        測った中で大きい方を残す（一方からは手前の面しか見えないので）。知らない id は無視する。"""
        o = self._objects.get(obj_id)
        if o is None or not all(math.isfinite(v) for v in size):
            return
        sx, sy = (min(max(float(v), SIZE_MIN), MAX_SIZE) for v in size)
        if obj_id in self._measured:
            sx, sy = max(sx, o.size[0]), max(sy, o.size[1])
        self._measured.add(obj_id)
        self._objects[obj_id] = replace(o, size=(sx, sy))

    def objects(self) -> List[Obj]:
        """覚えている物（id の順）。"""
        return [self._objects[k] for k in sorted(self._objects)]

    def message(self, stamp: float) -> dict:
        """/go2/objects に出す dict（json.dumps して std_msgs/String に入れる）。"""
        return format_message(self.objects(), stamp)
