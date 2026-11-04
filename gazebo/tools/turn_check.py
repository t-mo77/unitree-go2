#!/usr/bin/env python3
"""Gazebo の Go2 が、回った後も立っていられるかを自動で確かめる（gazebo コンテナの ROS 2 Jazzy で動かす）。

アプリと同じ窓口（/api/sport/request に unitree_api/Request）だけで動かす：
  StandUp 1004 → 実際の 3 秒 → BalanceStand 1002 → controller が「歩く」（trotting）に入るのを待つ
  → Move 1008 {"x", "y": 0, "z"} を 10 Hz で送り続け、目標の角度まで回ったら StopMove 1003
  → 止めてから --watch 秒（シミュレーションの時間。実際の時間でも --watch 秒以上）見張る。
- 回った角度は時間ではなく、Gazebo の本当の向きで測る。位置と向きは橋渡し（gazebo_bridge.py）と同じ
  /model/robot/odometry（nav_msgs/Odometry、Gazebo の OdometryPublisher）から読む。
- 「歩く」に入ったことは、controller が /rosout に出す "Switched from fixed stand to trotting" で知る
  （窓口の error_code 1013 は、体が実際に歩くに入るより先に変わるため。Task 16 の Concern 2）。
- 合格：見張る間ずっと |roll| < 0.5 rad・|pitch| < 0.5 rad・胴の高さ ≥ 0.25 m。
- 出力：標準出力に JSON を 1 行（途中の様子は標準エラー）。
  終了コード 0 = 全部合格、1 = どれか不合格、2 = 確かめられなかった（Gazebo・橋渡し・controller が応えない）。
- --repeat N で N 回続けて試す。2 回目からは StandDown で伏せさせ、StandUp からやり直す。
  倒れた試しの後は、そこでやめる（残りは skipped）。controller の中の推定（Estimator）は倒れたときの
  ものが残り、Gazebo で起こして置き直しても起動したてと同じにならないため（置き直した後は、立つ・
  歩くに入るだけで倒れることがあった）。きれいにやり直すには gazebo サービスを再起動する
  （docker compose --profile gazebo restart gazebo）。--respawn を付けると、倒れていたら
  Gazebo の /world/empty/set_pose で起こして続ける（その試しは "start": "respawn" と書く）。
- --forward を 0 より大きくすると、前へ歩きながら回る（弧を描く）。

使い方（gazebo コンテナが動いているとき。ホストのこのリポジトリで）:
    docker compose exec gazebo bash -lc \
        'source ~/unitree_ros2/setup.sh && python3 /work/gazebo/tools/turn_check.py --angle 90 --repeat 3'
"""

import argparse
import json
import math
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional, TextIO, Tuple

ROOT = Path(__file__).resolve().parents[2]          # このリポジトリ（コンテナでは /work）
sys.path.insert(0, str(ROOT / "nl_control"))

import rclpy  # noqa: E402
from controller_manager_msgs.srv import ListControllers  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rcl_interfaces.msg import Log  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,  # noqa: E402
                       qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions  # noqa: E402
from unitree_api.msg import Request  # noqa: E402
from unitree_go.msg import SportModeState  # noqa: E402

from go2_nl_control.actions import (BALANCE_STAND, MOVE, RISE_SIT, SITTING, STAND_DOWN,  # noqa: E402
                                    STAND_UP, STANDING, STOP_MOVE, state_from_error_code)

# 窓口（実機・アプリと同じ）
REQUEST_TOPIC = "/api/sport/request"
STATE_TOPIC = "/sportmodestate"            # error_code を読むだけ
# Gazebo 側（gazebo_bridge.py の ODOM_TOPIC・WORLD_NAME・MODEL_NAME と同じ）
ODOM_TOPIC = "/model/robot/odometry"
SET_POSE_SERVICE = "/world/empty/set_pose"
MODEL_NAME = "robot"
ROSOUT_TOPIC = "/rosout"
LIST_CONTROLLERS = "/controller_manager/list_controllers"
CONTROLLER = "unitree_guide_controller"

MOVE_HZ = 10.0           # Move を送り直す周期（アプリの executor・demo_script と同じ。実際の時間）
STAND_WAIT_S = 3.0       # StandUp から BalanceStand まで（アプリの "stand" と同じ。実際の時間）

# 合格の線（見張る間ずっと）
TILT_MAX = 0.5           # |roll|・|pitch| [rad] はこれ未満
HEIGHT_MIN = 0.25        # 胴の高さ [m] はこれ以上
# 倒れたとみなす線（回っている間にこれを越えたら、回るのをやめて見張りに移る）
FALL_TILT = 1.0
FALL_HEIGHT = 0.15
# 置き直しの要不要（伏せた後にこれより傾いていたら Gazebo で起こす）
UPRIGHT_TILT = 0.3
RESPAWN_Z = 0.20         # 起こして置き直すときの胴の高さ [m]（伏せの高さは約 0.07 m）
ODOM_STALE_S = 1.0       # odometry がこの秒数（実際の時間）途切れたら確かめられない


class CheckError(RuntimeError):
    """Gazebo・橋渡し・controller が応えず、確かめられない。"""


@dataclass(frozen=True)
class Sample:
    """odometry 1 通。sim はシミュレーションの時刻（stamp）、wall は受け取った実際の時刻（monotonic）。
    yaw は −π〜π、yaw_u はつなげた（2π で飛ばない）向き。"""
    sim: float
    wall: float
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    yaw_u: float


def rpy(w: float, x: float, y: float, z: float) -> Tuple[float, float, float]:
    """四元数 → (roll, pitch, yaw)。gazebo_bridge._rpy と同じ式。"""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def wrap(a: float) -> float:
    """角度を −π〜π に。"""
    return (a + math.pi) % (2 * math.pi) - math.pi


def fallen(s: Sample) -> bool:
    return abs(s.roll) > FALL_TILT or abs(s.pitch) > FALL_TILT or s.z < FALL_HEIGHT


def tilted(s: Sample) -> bool:
    return abs(s.roll) > UPRIGHT_TILT or abs(s.pitch) > UPRIGHT_TILT


class Window:
    """ある区間の odometry の最大 |roll|・最大 |pitch|・最低の高さと、合格の線を初めて越えた時刻。"""

    def __init__(self, start_sim: float):
        self.start = start_sim
        self.max_roll = 0.0
        self.max_pitch = 0.0
        self.min_height = math.inf
        self.first_bad: Optional[float] = None   # 合格の線を越えた時刻（start からのシミュレーションの秒数）
        self.first_fall: Optional[float] = None  # 倒れた（fallen）時刻（同じ）
        self.count = 0

    def add(self, s: Sample) -> None:
        if s.sim < self.start:
            return
        self.count += 1
        self.max_roll = max(self.max_roll, abs(s.roll))
        self.max_pitch = max(self.max_pitch, abs(s.pitch))
        self.min_height = min(self.min_height, s.z)
        bad = abs(s.roll) >= TILT_MAX or abs(s.pitch) >= TILT_MAX or s.z < HEIGHT_MIN
        if bad and self.first_bad is None:
            self.first_bad = s.sim - self.start
        if fallen(s) and self.first_fall is None:
            self.first_fall = s.sim - self.start

    @property
    def ok(self) -> bool:
        return self.count > 0 and self.first_bad is None

    def summary(self) -> dict:
        return {"max_roll": _r(self.max_roll), "max_pitch": _r(self.max_pitch),
                "min_height": _r(self.min_height), "first_bad_s": _r(self.first_bad),
                "fell_s": _r(self.first_fall), "samples": self.count}


def _r(v: Optional[float], nd: int = 3) -> Optional[float]:
    return None if v is None or not math.isfinite(v) else round(v, nd)


def log(text: str) -> None:
    print(f"[turn_check] {text}", file=sys.stderr, flush=True)


class TurnCheck(Node):
    def __init__(self):
        super().__init__("go2_turn_check")
        self._pub = self.create_publisher(Request, REQUEST_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(SportModeState, STATE_TOPIC, self._on_state, qos_profile_sensor_data)
        # controller の "Switched from A to B"。過去の分は要らないので volatile で受ける
        rosout_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Log, ROSOUT_TOPIC, self._on_log, rosout_qos)
        self.sample: Optional[Sample] = None
        self.error_code: Optional[int] = None
        self.switches: List[Tuple[float, str]] = []    # (受け取った実際の時刻, "fixed stand → trotting")
        self.windows: List[Window] = []                # odometry を入れる区間
        self._yaw_u: Optional[float] = None
        self._list = self.create_client(ListControllers, LIST_CONTROLLERS)
        self.trial = 0
        self.phase = "start"
        self.trace: Optional[TextIO] = None            # --trace の CSV

    # ---------- 受け取り ----------
    def _on_odom(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        roll, pitch, yaw = rpy(q.w, q.x, q.y, q.z)
        if self._yaw_u is None or self.sample is None:
            self._yaw_u = yaw
        else:
            self._yaw_u += wrap(yaw - self.sample.yaw)
        sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.sample = Sample(sim, time.monotonic(), p.x, p.y, p.z, roll, pitch, yaw, self._yaw_u)
        for w in self.windows:
            w.add(self.sample)
        if self.trace is not None:
            s = self.sample
            self.trace.write(f"{self.trial},{self.phase},{s.sim:.3f},{s.wall:.3f},{s.x:.4f},{s.y:.4f},"
                             f"{s.z:.4f},{s.roll:.4f},{s.pitch:.4f},{s.yaw_u:.4f},{self.error_code}\n")

    def _on_state(self, msg: SportModeState) -> None:
        self.error_code = int(msg.error_code)

    def _on_log(self, msg: Log) -> None:
        text = msg.msg
        if text.startswith("Switched from "):
            a, _, b = text[len("Switched from "):].partition(" to ")
            self.switches.append((time.monotonic(), f"{a} → {b}"))
            log(f"controller: {a} → {b}")

    # ---------- 送る・待つ ----------
    def send(self, api_id: int, parameter: Optional[dict] = None) -> None:
        msg = Request()
        msg.header.identity.api_id = api_id
        if parameter is not None:
            msg.parameter = json.dumps(parameter)
        self._pub.publish(msg)

    def spin_until(self, wall_deadline: float) -> None:
        while True:
            left = wall_deadline - time.monotonic()
            if left <= 0:
                return
            rclpy.spin_once(self, timeout_sec=min(left, 0.02))

    def wait(self, sec: float) -> None:
        """実際の時間で sec 秒待つ（その間も受け取る）。"""
        self.spin_until(time.monotonic() + sec)

    def wait_until(self, cond: Callable[[], bool], timeout: float, what: str) -> None:
        end = time.monotonic() + timeout
        while not cond():
            if time.monotonic() > end:
                raise CheckError(f"{what}（実際の {timeout:g} 秒待った）")
            rclpy.spin_once(self, timeout_sec=0.02)
            self.check_fresh()

    def wait_sim(self, sec: float) -> None:
        """シミュレーションの時間で sec 秒待つ。"""
        t0 = self.sample.sim
        self.wait_until(lambda: self.sample.sim >= t0 + sec, 10 * sec + 10,
                        f"シミュレーションの時間が {sec:g} 秒進みません")

    def wait_switch(self, to: str, since: float, timeout: float) -> bool:
        """since（実際の時刻）より後に controller が to に入ったら True。timeout 秒で諦めて False。"""
        def seen() -> bool:
            return any(t >= since and s.endswith(f"→ {to}") for t, s in self.switches)
        try:
            self.wait_until(seen, timeout, f"controller が {to} に入りません")
            return True
        except CheckError:
            return False

    def check_fresh(self) -> None:
        if self.sample is not None and time.monotonic() - self.sample.wall > ODOM_STALE_S:
            raise CheckError(f"{ODOM_TOPIC} が途切れました（Gazebo が止まっている？）")

    def wait_ready(self, timeout: float = 30.0) -> None:
        self.wait_until(lambda: self.sample is not None, timeout,
                        f"{ODOM_TOPIC} が来ません（Gazebo が動いていない？）")
        self.wait_until(lambda: self.error_code is not None, timeout,
                        f"{STATE_TOPIC} が来ません（橋渡しが動いていない？）")
        self.wait_until(lambda: self._pub.get_subscription_count() > 0, timeout,
                        f"{REQUEST_TOPIC} の受け手（橋渡し）が見つかりません")
        # 起動したては controller がまだ active でない（StandUp が controller に届かない）
        end = time.monotonic() + timeout
        while self.controller_state() != "active":
            if time.monotonic() > end:
                raise CheckError(f"{CONTROLLER} が active になりません")
            self.wait(0.5)

    def controller_state(self) -> Optional[str]:
        if not self._list.wait_for_service(timeout_sec=1.0):
            return None
        future = self._list.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        res = future.result()
        if res is None:
            return None
        return next((c.state for c in res.controller if c.name == CONTROLLER), None)

    def wait_code(self, codes: Tuple[int, ...], timeout: float = 6.0) -> None:
        self.wait_until(lambda: self.error_code in codes, timeout,
                        f"error_code が {codes} になりません（今 {self.error_code}）")

    # ---------- 準備：伏せて、まっすぐ置く ----------
    def lie_down(self, respawn: bool) -> str:
        """どの姿勢からでも伏せ（error_code 1004 か 1001）にする。倒れていたら、respawn なら Gazebo で
        起こし、そうでなければ CheckError。どう始めたかを返す："fresh"（起動したまま。error_code 1001）、
        "standdown"（StandDown で伏せた）、"respawn"（倒れていたので置き直した）。"""
        start = "fresh" if self.error_code == 1001 else "standdown"
        if self.error_code == 1002:            # StandUp だけの立ち方。StandDown を受けないので歩くへ
            self.send(BALANCE_STAND)
            self.wait_code((1013,))
        if state_from_error_code(self.error_code) == SITTING:
            self.send(RISE_SIT)
            self.wait_code((1013,))
        if state_from_error_code(self.error_code) == STANDING:
            self.send(STOP_MOVE)
            self.wait(0.3)
            since = time.monotonic()
            self.send(STAND_DOWN)
            self.wait_code((1004,))
            if not self.wait_switch("fixed down", since, 20.0):
                raise CheckError("StandDown の後、controller が fixed down に入りません")
            self.wait_sim(1.0)
        if self.error_code not in (1001, 1004):
            raise CheckError(f"伏せにできません（error_code {self.error_code}）")
        if tilted(self.sample):
            if not respawn:
                raise CheckError("Go2 が倒れています。gazebo サービスを再起動する"
                                 "（docker compose --profile gazebo restart gazebo）か、--respawn を付けてください")
            self.respawn()
            start = "respawn"
        return start

    def respawn(self) -> None:
        s = self.sample
        log(f"倒れている（roll {s.roll:.2f}, pitch {s.pitch:.2f}, z {s.z:.2f}）ので Gazebo で起こして置き直します")
        req = (f'name: "{MODEL_NAME}", position: {{x: {s.x:.3f}, y: {s.y:.3f}, z: {RESPAWN_Z}}}, '
               f'orientation: {{w: {math.cos(s.yaw / 2):.6f}, z: {math.sin(s.yaw / 2):.6f}}}')
        out = subprocess.run(["gz", "service", "-s", SET_POSE_SERVICE, "--reqtype", "gz.msgs.Pose",
                              "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                             capture_output=True, text=True, timeout=15)
        if "true" not in out.stdout:
            raise CheckError(f"{SET_POSE_SERVICE} が失敗しました: {out.stdout.strip()} {out.stderr.strip()}")
        self.wait_sim(2.0)
        if tilted(self.sample):
            raise CheckError("置き直しても傾いたままです")

    # ---------- 立って、歩くに入る ----------
    def stand(self, settle: float) -> Tuple[str, float]:
        """StandUp → 3 秒 → BalanceStand → controller が trotting に入って settle 秒（sim）。
        trotting に入ったことを何で知ったか（"rosout" か "timing"）と、settle の間に動いた距離 [m]
        （止まっているはず。大きければ controller の推定がずれている）を返す。"""
        stand_sim = self.sample.sim
        self.send(STAND_UP)
        self.wait(STAND_WAIT_S)
        since = time.monotonic()
        self.send(BALANCE_STAND)
        self.wait_code((1013,))
        if self.wait_switch("trotting", since, 30.0):
            how = "rosout"
        else:   # /rosout が届かないとき：橋渡しの予定（伏せ 1.8 秒 + 立つ 1.8 秒の後）から見積もる
            how = "timing"
            self.wait_until(lambda: self.sample.sim >= stand_sim + 3.6 + 0.5, 60.0,
                            "シミュレーションの時間が進みません")
        t = self.sample
        self.wait_sim(settle)
        s = self.sample
        if s.z < HEIGHT_MIN or tilted(s):
            raise CheckError(f"立てていません（z {s.z:.3f}, roll {s.roll:.2f}, pitch {s.pitch:.2f}）")
        return how, math.hypot(s.x - t.x, s.y - t.y)


def run_trial(node: TurnCheck, args: argparse.Namespace, k: int) -> dict:
    node.trial = k
    node.phase = "prepare"
    node.wait_ready()
    begin = node.lie_down(args.respawn)
    node.phase = "stand"
    how, settle_move = node.stand(args.settle)

    target = math.radians(args.angle)
    sign = 1.0 if target >= 0 else -1.0
    start = node.sample
    turn = Window(start.sim)
    node.windows = [turn]
    node.phase = "turn"
    sim_limit = 4.0 * abs(target) / args.rate + 10.0    # 指令どおりに回らなくても待つ余裕
    wall_limit = 4.0 * sim_limit
    log(f"試し {k}: {args.angle:+g}° を {args.rate:g} rad/s、前へ {args.forward:g} m/s で回ります"
        f"（z {start.z:.3f} m, 向き {math.degrees(start.yaw):.1f}°, trotting は {how} で確認）")
    stop_reason = "reached"
    tick = time.monotonic()
    while True:
        node.send(MOVE, {"x": args.forward, "y": 0.0, "z": sign * args.rate})
        tick += 1.0 / MOVE_HZ
        node.spin_until(tick)
        node.check_fresh()
        s = node.sample
        if (s.yaw_u - start.yaw_u) * sign >= abs(target):
            break
        if fallen(s):
            stop_reason = "fell"
            break
        if s.sim - start.sim > sim_limit or time.monotonic() - start.wall > wall_limit:
            stop_reason = "timeout"
            break
    node.send(STOP_MOVE)
    stop = node.sample
    watch = Window(stop.sim)
    node.windows = [watch]
    node.phase = "watch"
    log(f"試し {k}: 止めました（{stop_reason}、回った {math.degrees(stop.yaw_u - start.yaw_u):+.1f}°、"
        f"シミュレーション {stop.sim - start.sim:.2f} 秒）。{args.watch:g} 秒見張ります")
    node.wait_until(lambda: (node.sample.sim >= stop.sim + args.watch
                             and time.monotonic() >= stop.wall + args.watch),
                    10 * args.watch + 30, "見張りの間にシミュレーションの時間が進みません")
    end = node.sample
    node.windows = []
    node.phase = "done"

    turned = end.yaw_u - start.yaw_u
    passed = stop_reason == "reached" and watch.ok
    if stop_reason != "reached":
        reason = f"turn {stop_reason}"
    elif not watch.ok:
        reason = f"watch: limit crossed {watch.first_bad:.2f} s after stop"
    else:
        reason = None
    turn_sim = stop.sim - start.sim
    seen = watch.summary()
    result = {
        "trial": k, "pass": passed, "reason": reason,
        "fell": turn.first_fall is not None or watch.first_fall is not None,
        "start": begin, "trot_detect": how,
        "settle_move_m": _r(settle_move),
        # 見張りの間（止めてから --watch 秒）の値。合否はこれで決める
        "max_roll": seen["max_roll"], "max_pitch": seen["max_pitch"],
        "min_height": seen["min_height"],
        "final_yaw_err_deg": _r(math.degrees(turned - target), 1),
        "rtf": _r((end.sim - start.sim) / max(end.wall - start.wall, 1e-6), 2),
        "turn": {**turn.summary(), "stop": stop_reason,
                 "turned_deg": _r(math.degrees(stop.yaw_u - start.yaw_u), 1),
                 "sim_s": _r(turn_sim, 2), "wall_s": _r(stop.wall - start.wall, 2),
                 "yaw_rate": _r(abs(stop.yaw_u - start.yaw_u) / turn_sim if turn_sim > 0 else None),
                 "dist_m": _r(math.hypot(stop.x - start.x, stop.y - start.y))},
        "watch": {"first_bad_s": seen["first_bad_s"], "fell_s": seen["fell_s"],
                  "samples": seen["samples"],
                  "sim_s": _r(end.sim - stop.sim, 2), "wall_s": _r(end.wall - stop.wall, 2),
                  "drift_after_stop_deg": _r(math.degrees(end.yaw_u - stop.yaw_u), 1)},
    }
    log(f"試し {k}: {'合格' if passed else '不合格'} " + json.dumps(result, ensure_ascii=False))
    return result


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Gazebo の Go2 を回して、その後も立っていられるか確かめる")
    ap.add_argument("--angle", type=float, default=90.0,
                    help="回る角度 [度]。正なら左回り（Move の z が正）。既定 90")
    ap.add_argument("--rate", type=float, default=0.5,
                    help="Move の z の大きさ [rad/s]（controller の上限は 0.5）。既定 0.5")
    ap.add_argument("--forward", type=float, default=0.0,
                    help="回りながら前へ歩く速さ（Move の x）[m/s]。既定 0（その場で回る）")
    ap.add_argument("--watch", type=float, default=8.0,
                    help="止めてから見張る秒数（シミュレーションの時間）。既定 8")
    ap.add_argument("--repeat", type=int, default=1, help="続けて試す回数。既定 1")
    ap.add_argument("--respawn", action="store_true",
                    help="倒れた後も続ける（Gazebo で起こして置き直す。controller の推定は残る）")
    ap.add_argument("--trace", type=Path, default=None,
                    help="odometry を 1 通ずつ CSV に書く（trial,phase,sim,wall,x,y,z,roll,pitch,yaw_u,error_code）")
    ap.add_argument("--settle", type=float, default=1.0,
                    help="trotting に入ってから回り始めるまで（シミュレーションの時間）[s]。既定 1")
    args = ap.parse_args(argv)
    if args.rate <= 0 or args.watch <= 0 or args.repeat < 1 or args.settle < 0:
        ap.error("--rate・--watch は正、--repeat は 1 以上、--settle は 0 以上")
    return args


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)

    def on_signal(signum, frame):
        raise KeyboardInterrupt
    for name in ("SIGTERM", "SIGHUP"):
        signal.signal(getattr(signal, name), on_signal)

    # Ctrl+C で ROS 2 が先に終わると StopMove を送れないので、ROS 2 には Ctrl+C を扱わせない
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TurnCheck()
    if args.trace is not None:
        node.trace = args.trace.open("w")
        node.trace.write("trial,phase,sim,wall,x,y,z,roll,pitch,yaw_u,error_code\n")
    trials: List[dict] = []
    error = None
    try:
        for k in range(1, args.repeat + 1):
            trials.append(run_trial(node, args, k))
            if trials[-1]["fell"] and not args.respawn and k < args.repeat:
                log(f"倒れたので、残りの {args.repeat - k} 回はやめます（--respawn で続けられる）")
                break
    except CheckError as e:
        error = str(e)
        log(f"確かめられません: {error}")
    except KeyboardInterrupt:
        error = "interrupted"
    finally:
        node.send(STOP_MOVE)
        node.wait(0.2)
    passed = sum(t["pass"] for t in trials)
    ok = error is None and passed == len(trials)
    print(json.dumps({
        "pass": ok, "passed": passed, "total": len(trials),
        "skipped": args.repeat - len(trials), "error": error,
        "args": {"angle": args.angle, "rate": args.rate, "forward": args.forward,
                 "watch": args.watch, "repeat": args.repeat, "settle": args.settle,
                 "respawn": args.respawn},
        "limits": {"tilt_rad": TILT_MAX, "height_m": HEIGHT_MIN},
        "trials": trials,
    }, ensure_ascii=False), flush=True)
    if node.trace is not None:
        node.trace.close()
    node.destroy_node()
    rclpy.try_shutdown()
    if error is not None:
        return 2
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
