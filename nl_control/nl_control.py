"""自然言語の指示で Go2 を動かす（LLM → 計画 → 監督器 → ROS 2）。前方に障害物があれば止まり、なくなれば続きから動く。

使い方 (Go2 と有線 LAN でつないだ PC のターミナルで):
    source ~/unitree_ros2/setup.sh
    python3 nl_control.py

- 起動したら、指示を日本語で打って Enter（例: 立って1メートル前に進んで、左を向いて挨拶して）。
- 「止まって」「ストップ」は LLM を通さず、すぐ止める。q で終了。
- 前方 1 m と周り (特に後ろ) を空け、リモコンですぐ止められる人がそばにいる状態で動かす。
- Ctrl+C・kill・SSH の切断でも、StopMove を送ってから終わる。

LLM の設定は、このファイルと同じフォルダの env.txt（または .env）に書く:
    LLM_BASE_URL=https://api.groq.com/openai/v1     (Ollama なら http://localhost:11434/v1)
    LLM_API_KEY=...
    LLM_MODEL=openai/gpt-oss-20b

しくみ:
- LLM は「言葉 → 決まった語彙の動作の列 (JSON)」の翻訳だけをする。形は構造化出力で固定し、
  check_plan() が語彙・値の範囲・状態遷移 (立っていないのに歩く、など) を検証して、だめなら理由を添えて聞き直す。
- 実行中は毎周期、LiDAR (/utlidar/cloud) で前方の箱の中に点があるかを見て、前へ進む動作だけを保留する。
- 移動は /sportmodestate の位置と向きで、決めた距離・角度に届いたら止める。
- このプログラムは 1 本だけで動く。/api/sport/request に自分以外の送り手 (keyboard_velocity.py など) がいれば動かない。
"""

import fcntl
import json
import os
import select
import signal
import sys
import time

import numpy as np

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from unitree_api.msg import Request
from unitree_go.msg import SportModeState

from go2_nl_control.actions import ACTIONS, state_from_error_code, UNKNOWN
from go2_nl_control.executor import STOP, ExecConfig, Executor
from go2_nl_control.perception import ObstacleMonitor, PerceptionConfig
from go2_nl_control.supervisor import SENSOR_LOST
from go2_nl_control.translator import OpenAICompatLLM, Translator, is_stop_command, load_env_file

# === 設定 (ここを変えると速さや止まる距離などを変えられる) ====================
RATE = 10.0                 # 指令を送る周期 [Hz]
STATE_TIMEOUT = 0.5         # /sportmodestate がこの秒数途切れたら止める [s]
EXEC = ExecConfig(speed=(0.3, 0.3, 0.8), block_timeout=15.0)
PERCEPTION = PerceptionConfig(
    stop_distance=0.50,     # 前方にこれより近い物があれば止まる [m]
    resume_distance=0.70,   # これより遠くなったら動き出す [m]
    half_width=0.30,        # 見張る幅（中心から左右）[m]
    z_min=-0.25, z_max=0.40,  # 見張る高さ（機体基準）。地面と天井を除く [m]
    # LiDAR の取り付け位置と傾き。機体ごとに rviz2 で確かめて合わせる（README 参照）
    lidar_x=0.28, lidar_z=0.10, lidar_pitch=0.0, lidar_yaw=0.0,
)
LIDAR_TOPIC = "/utlidar/cloud"
STATE_TOPIC = "/sportmodestate"
REQUEST_TOPIC = "/api/sport/request"
LOCK_FILE = "/tmp/go2_nl_control.lock"
# ==========================================================================


def say(text: str):
    print(f"\r[GO2] {text}", flush=True)


class Go2NL:
    def __init__(self):
        # Ctrl+C で ROS2 が先に終わると止める指令を送れなくなるので、ROS2 には Ctrl+C を扱わせない
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("go2_nl_control")
        self.pub = self.node.create_publisher(Request, REQUEST_TOPIC, 10)
        self.node.create_subscription(SportModeState, STATE_TOPIC, self.on_state, qos_profile_sensor_data)
        self.node.create_subscription(PointCloud2, LIDAR_TOPIC, self.on_cloud, qos_profile_sensor_data)

        self.pose = None          # (x, y, yaw)
        self.pose_time = 0.0
        self.robot_posture = UNKNOWN
        self.monitor = ObstacleMonitor(PERCEPTION)
        self.executor = Executor(ACTIONS, EXEC)
        self.last_env = None
        self.req_id = 0

        here = os.path.dirname(os.path.abspath(__file__))
        for name in ("env.txt", ".env"):
            load_env_file(os.path.join(here, name))
        self.translator = Translator(OpenAICompatLLM(
            base_url=os.environ.get("LLM_BASE_URL", "https://api.groq.com/openai/v1"),
            api_key=os.environ.get("LLM_API_KEY", ""),
            model=os.environ.get("LLM_MODEL", "openai/gpt-oss-20b")))

    # ---------- 受信 ----------
    def on_state(self, msg):
        self.pose = (msg.position[0], msg.position[1], msg.imu_state.rpy[2])
        self.pose_time = time.time()
        self.robot_posture = state_from_error_code(msg.error_code)

    def on_cloud(self, msg):
        if hasattr(point_cloud2, "read_points_numpy"):  # Humble 以降
            pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        else:  # Foxy
            pts = np.array([list(p) for p in point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)], dtype=np.float64).reshape(-1, 3)
        self.monitor.update(pts, time.time())

    # ---------- 送信 ----------
    def send(self, cmd):
        msg = Request()
        self.req_id += 1
        msg.header.identity.id = self.req_id
        msg.header.identity.api_id = cmd.api_id
        if cmd.parameter is not None:
            msg.parameter = json.dumps(cmd.parameter)
        self.pub.publish(msg)

    def stop(self, times=1):
        for _ in range(times):
            self.send(STOP)
            if times > 1:
                time.sleep(0.05)

    def spin_for(self, sec):
        end = time.time() + sec
        while True:
            rclpy.spin_once(self.node, timeout_sec=max(0.0, min(0.01, end - time.time())))
            if time.time() >= end:
                break

    # ---------- 確認 ----------
    def other_senders(self) -> int:
        """/api/sport/request に指令を送っている、自分以外のプログラムの数。"""
        return max(0, self.node.count_publishers(REQUEST_TOPIC) - 1)

    def wait_ready(self, timeout=5.0):
        t0 = time.time()
        while self.pub.get_subscription_count() == 0 or self.pose is None:
            if time.time() - t0 > timeout:
                raise RuntimeError("Go2 が見つかりません（有線 LAN の接続と、source ~/unitree_ros2/setup.sh をしたかを確かめてください）")
            self.spin_for(0.1)
        self.spin_for(1.0)  # 他の送り手が見えるまで少し待つ
        if self.other_senders():
            raise RuntimeError(f"{REQUEST_TOPIC} に指令を送っている別のプログラムがあります。そちらを終了してから起動してください")
        if time.time() - (self.monitor._last_stamp or 0) > 1.0:
            say(f"注意: {LIDAR_TOPIC} が届いていません。届くまで移動は保留されます")

    def posture(self) -> str:
        # ロボットが報告する姿勢がわかればそれを使い、わからなければ実行器が覚えている姿勢を使う
        return self.robot_posture if self.robot_posture != UNKNOWN else self.executor.posture

    def env(self, now) -> str:
        if now - self.pose_time > STATE_TIMEOUT:
            return SENSOR_LOST
        return self.monitor.current(now)

    # ---------- 指示を受けたとき ----------
    def on_text(self, text: str):
        if is_stop_command(text):
            for c in self.executor.cancel("止める指示"):
                self.send(c)
            say("止めました")
            return
        # 新しい指示を受けたら、まず今の動作を止める（LLM を待つ間は動かない）
        for c in self.executor.cancel("新しい指示"):
            self.send(c)
        if self.other_senders():
            say(f"{REQUEST_TOPIC} に別のプログラムが指令を送っています。そちらを終了してから指示してください")
            return
        state = self.posture()
        say(f"考え中…（今の姿勢: {state}）")
        steps, log = self.translator.translate(text, state, ACTIONS)
        for line in log:
            say("LLM " + line)
        if steps is None:
            say("この指示は実行できませんでした。言い方を変えてもう一度どうぞ")
            return
        self.executor.posture = state
        self.spin_for(0.3)  # LLM を待つ間に溜まったセンサーを読み直してから動く
        self.executor.load(steps)

    # ---------- メインループ ----------
    def run(self):
        period = 1.0 / RATE
        print("指示を入力して Enter（止まって = 停止 / q = 終了）")
        print("> ", end="", flush=True)
        while True:
            t0 = time.time()
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line:  # 入力が閉じられた
                    raise KeyboardInterrupt
                text = line.strip()
                if text in ("q", "quit", "終了"):
                    raise KeyboardInterrupt
                if text:
                    self.on_text(text)
                print("> ", end="", flush=True)

            now = time.time()
            env = self.env(now)
            if env != self.last_env:
                near = self.monitor.nearest
                say({"CLEAR": "前方: なし",
                     "BLOCKED": f"前方: 障害物 {near:.2f} m" if near is not None else "前方: 障害物",
                     "SENSOR_LOST": "センサー: 途切れ"}[env])
                self.last_env = env
            for c in self.executor.tick(now, env, self.pose):
                self.send(c)
            for e in self.executor.events:
                say(e)
            self.executor.events.clear()

            self.spin_for(max(0.0, period - (time.time() - t0)))

    def close(self):
        self.node.destroy_node()
        rclpy.try_shutdown()


def main():
    # 同じプログラムの二重起動を防ぐ
    lock = open(LOCK_FILE, "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("nl_control.py はすでに動いています。2つ同時には起動できません")
        sys.exit(1)

    # kill (SIGTERM) や SSH の切断 (SIGHUP) でも、Ctrl+C と同じく止めてから終わる
    def on_signal(signum, frame):
        raise KeyboardInterrupt
    for name in ("SIGTERM", "SIGHUP"):
        signal.signal(getattr(signal, name), on_signal)

    go2 = Go2NL()
    try:
        go2.wait_ready()
        go2.run()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, EOFError) as e:
        print(f"\n中止しました: {e}")
    finally:
        go2.stop(times=3)  # 取りこぼしに備えて 3 回送る
        print("\nStopMove を送って終了しました。立ったままなら、リモコンで伏せさせてください")
        go2.close()


if __name__ == "__main__":
    main()
