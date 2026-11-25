"""キーボードで Go2 を歩かせる (High Level の Move で速度を送る)。

使い方 (Go2 と有線 LAN でつないだ Linux PC のターミナルで。SSH 先でもよい):
    source ~/unitree_ros2/setup.sh
    python3 keyboard_velocity.py

- 起動すると操作キーの一覧が出る。まず 1 で立たせてから W/A/S/D などで歩かせる。
- 階段は、立ってから 3 で公式の階段も歩ける歩き方 (ClassicWalk) にして、W で登る。
  登り終えたら 4 でふだんの歩き方に戻す (docs/stairs.md の「キーボードで試す」)。
- 速度キーは押している間だけ動き、離すと止まる。
  反応が鈍い・止まるのが遅いときは、キーのリピートを速くする: xset r rate 200 30
  (キーボードをつないでいる側の PC のデスクトップのターミナルで実行する)
- 周りを空け、リモコンですぐ止められる人がそばにいる状態で動かす。

しくみ:
- /api/sport/request に Request を publish すると Go2 が動く (High Level の Sport API)。
  どの動きをするかは header.identity.api_id で選び、速度などは parameter に JSON で入れる。
- キーが押されている間、Move を RATE [Hz] で送り続け、離したら StopMove を送る。
- /sportmodestate の error_code (Go2 の今の状態の番号) が変わるたびに 1 行出す。
"""

import json
import os
import signal
import sys
import termios  # Linux 専用 (Windows では動かない)
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from unitree_api.msg import Request
from unitree_go.msg import SportModeState

# === 設定 (ここを変えると速さなどを変えられる) ===========================
SPEED = (0.3, 0.5, 0.8)  # キーを押したときの [前後 m/s, 左右 m/s, 旋回 rad/s]
RATE = 10.0  # Move を送る周期 [Hz]
KEY_HOLD_TIMEOUT = 0.5  # キーのリピートがこの秒数途切れたら、キーを離したとみなす [s]
STANDUP_WAIT = 3.0  # StandUp を送ってから BalanceStand を送るまでの待ち [s]
# ======================================================================

TOPIC = "/api/sport/request"
STATE_TOPIC = "/sportmodestate"

# api_id (動きの番号)
BALANCE_STAND = 1002  # 歩ける立ち方にする (StandUp のあとに送る)
STOP_MOVE = 1003  # 移動を止める
STAND_UP = 1004  # 立つ (関節を固めた立ち方。このままでは歩けない)
STAND_DOWN = 1005  # 伏せる
MOVE = 1008  # 速度で動く。parameter = {"x": 前後 [m/s], "y": 左右 [m/s], "z": 旋回 [rad/s]}
CLASSIC_WALK = 2049  # 公式の ClassicWalk (Go2 のソフト V1.1.6 以上)。parameter = {"data": true} で
                     # 階段も歩ける歩き方 (error_code 2010)、{"data": false} でやめる (Agile、error_code 100)

# キー → ClassicWalk の data
GAIT_KEYS = {"3": True, "4": False}

# error_code → 表示 (公式の Motion Services Interface V2.0 の表から。表に無い番号は番号だけ出す)
STATE_NAMES = {
    1001: "伏せ (Damp)", 1002: "立ち (StandUp。このままでは歩けない)", 1004: "伏せ", 2006: "伏せ",
    1007: "座り", 1013: "立ち (歩ける)", 1015: "歩き", 1016: "走り", 1017: "歩き (Endurance)",
    2010: "階段も歩ける歩き方 (ClassicWalk)", 100: "ふだんの歩き方 (Agile)",
}

# キー → (前後, 左右, 旋回) の向き。+ が 前 / 左 / 左回り
VELOCITY_KEYS = {
    "w": (1, 0, 0),
    "s": (-1, 0, 0),
    "a": (0, 1, 0),
    "d": (0, -1, 0),
    "q": (0, 0, 1),
    "e": (0, 0, -1),
}
# 矢印キーは ESC [ A のような文字列で届くので、同じ動きの文字に置き換える
ARROW_KEYS = {"\x1b[A": "w", "\x1b[B": "s", "\x1b[D": "a", "\x1b[C": "d"}

HELP = f"""\
================================================================
 Unitree Go2 - キーボード操作 (High Level / Move)
================================================================
 1          : 立つ (StandUp → BalanceStand)。歩く前に押す
 2          : 伏せる (StandDown)。終了する前に押しておく
 3          : 階段も歩ける歩き方にする (ClassicWalk オン)。立っているときに押す
 4          : ふだんの歩き方に戻す (ClassicWalk オフ)
 W / S (↑↓) : 前進 / 後退      {SPEED[0]:.2f} m/s
 A / D (←→) : 左 / 右          {SPEED[1]:.2f} m/s
 Q / E      : 左回り / 右回り  {SPEED[2]:.2f} rad/s
 Space      : 止まる
 Z / Ctrl+C : 終了 (StopMove を送る)
 ※ 小文字で押す (CapsLock だと反応しない)
================================================================"""


def read_keys(fd):
    """押されたキーを 1 文字ずつのリストで返す (キーが来ていなければ待たずに [] を返す)。

    矢印キーは ESC [ A のような文字列で届くので、先に w などに置き換える (ARROW_KEYS)。
    大文字は小文字にしない (置き換えられなかった矢印キーの A や D が a・d と間違われないように)。
    """
    try:
        text = os.read(fd, 64).decode("utf-8", "ignore")
    except OSError:
        return []
    for seq, key in ARROW_KEYS.items():
        text = text.replace(seq, key)
    return list(text)


class Go2:
    """Go2 に指令を送る。"""

    def __init__(self):
        # Ctrl+C で ROS2 が先に終わると、止める指令を送れなくなる。
        # そこで ROS2 には Ctrl+C を扱わせず、main() で受けて StopMove を送ってから終わる。
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("go2_keyboard_velocity")
        self.pub = self.node.create_publisher(Request, TOPIC, 10)
        self.error_code = None  # 最後に届いた /sportmodestate の error_code (まだ届いていなければ None)
        self.node.create_subscription(SportModeState, STATE_TOPIC, self._on_state, qos_profile_sensor_data)

    def _on_state(self, msg):
        self.error_code = int(msg.error_code)

    def receive(self):
        """届いている /sportmodestate を読む (待たない)。"""
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.0)

    def wait_connected(self, timeout=5.0):
        """Go2 が見つかるまで待つ (見つかる前に送ると届かないことがある)。"""
        t0 = time.time()
        while self.pub.get_subscription_count() == 0:
            if time.time() - t0 > timeout:
                print(f"[ERROR] {TOPIC} を受ける相手 (Go2) が見つかりません。\n"
                      "        有線 LAN の接続と、source ~/unitree_ros2/setup.sh をしたかを確かめてください。")
                sys.exit(1)
            time.sleep(0.1)
        print(f"[ROS2] {TOPIC} → Go2 につながりました")

    def send(self, api_id, parameter=None):
        """api_id の指令を 1 回送る。parameter (dict) は JSON にして入れる。"""
        msg = Request()
        msg.header.identity.api_id = api_id
        if parameter is not None:
            msg.parameter = json.dumps(parameter)
        self.pub.publish(msg)

    def close(self):
        self.node.destroy_node()
        rclpy.try_shutdown()


def state_line(code):
    """error_code を表示する 1 行。"""
    name = STATE_NAMES.get(code)
    return f"[STATE] error_code {code}" + (f" {name}" if name else "")


def velocity(last_pressed, now):
    """いま押されている速度キーから (前後, 左右, 旋回) の速度を求める。

    last_pressed は キー → 最後にそのキーが届いた時刻。KEY_HOLD_TIMEOUT 以内に届いたキーを
    押されているとみなし、W と A のように同時に押したキーは足し合わせる (各向き SPEED まで)。
    """
    v = [0.0, 0.0, 0.0]
    for key, t in last_pressed.items():
        if now - t <= KEY_HOLD_TIMEOUT:
            for i, d in enumerate(VELOCITY_KEYS[key]):
                v[i] += d * SPEED[i]
    return tuple(max(-s, min(s, x)) for x, s in zip(v, SPEED))


def main():
    # kill (SIGTERM) や SSH の切断 (SIGHUP) でも、Ctrl+C と同じく止めてから終わる
    def on_signal(signum, frame):
        raise KeyboardInterrupt
    for name in ("SIGTERM", "SIGHUP"):
        signal.signal(getattr(signal, name), on_signal)

    go2 = Go2()
    go2.wait_connected()
    print(HELP)
    print("周りを空け、リモコンですぐ止められる状態で操作してください。")

    # ターミナルを「Enter を待たずに 1 文字ずつ読む・押したキーを表示しない・読むときに待たない」設定にする。
    # 終わるときに元の設定 (old) に戻す。
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] &= ~(termios.ICANON | termios.ECHO)
    new[6][termios.VMIN] = 0
    new[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, new)

    last_pressed = {}  # 速度キー → 最後に届いた時刻
    moving = False  # いま Move を送っているか (止まったときに StopMove を 1 回だけ送るため)
    shown_code = None  # 最後に表示した error_code
    classic = False  # 最後に送った ClassicWalk が true か (終わるときの注意のため)
    try:
        while True:
            now = time.time()
            for key in read_keys(fd):
                if key == "z":
                    raise KeyboardInterrupt
                if key in ("1", "2", *GAIT_KEYS) and moving:  # 歩いている途中なら、先に止める
                    go2.send(STOP_MOVE)
                    moving = False
                if key == "1":
                    last_pressed.clear()
                    print("\r[GO2] 立ちます (StandUp → BalanceStand)")
                    go2.send(STAND_UP)
                    time.sleep(STANDUP_WAIT)
                    go2.send(BALANCE_STAND)
                elif key == "2":
                    last_pressed.clear()
                    print("\r[GO2] 伏せます (StandDown)")
                    go2.send(STAND_DOWN)
                elif key in GAIT_KEYS:
                    last_pressed.clear()
                    classic = GAIT_KEYS[key]
                    print("\r[GO2] " + ("階段も歩ける歩き方にします (ClassicWalk オン)" if classic
                                         else "ふだんの歩き方に戻します (ClassicWalk オフ)"))
                    go2.send(CLASSIC_WALK, {"data": classic})
                elif key == " ":
                    last_pressed.clear()
                elif key in VELOCITY_KEYS:
                    last_pressed[key] = now

            go2.receive()
            if go2.error_code is not None and go2.error_code != shown_code:
                shown_code = go2.error_code
                print("\r" + state_line(shown_code) + "            ")

            v = velocity(last_pressed, time.time())
            if any(v):
                go2.send(MOVE, {"x": v[0], "y": v[1], "z": v[2]})
                moving = True
                print(f"\r[MOVE] x={v[0]:+.2f} y={v[1]:+.2f} z={v[2]:+.2f}   ", end="", flush=True)
            elif moving:
                go2.send(STOP_MOVE)
                moving = False
                print("\r[STOP] StopMove                    ")
            time.sleep(1.0 / RATE)
    except KeyboardInterrupt:
        pass
    finally:
        # 先に止める (SSH が切れたときは、ターミナルの設定を戻すのや print が失敗することがある)
        for _ in range(3):  # 取りこぼしに備えて 3 回送る
            go2.send(STOP_MOVE)
            time.sleep(0.05)
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            print("\n[STOP] StopMove を送って終了しました。立ったままなら、リモコンで伏せさせてください")
            if classic:
                print("[注意] 階段も歩ける歩き方のままです。戻すときは、もう一度起動して 4 を押してください")
        except (termios.error, OSError):
            pass
        go2.close()


if __name__ == "__main__":
    main()
