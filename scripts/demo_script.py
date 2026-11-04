"""Go2 デモ: 立つ → 前進 0.5 m → 左に 90° 向く → 挨拶 → 座る → 立ち上がる。

使い方 (Go2 と有線 LAN でつないだ PC のターミナルで):
    source ~/unitree_ros2/setup.sh
    python3 demo_script.py

- 前方 1 m と周り (特に後ろ) を空け、リモコンですぐ止められる人がそばにいる状態で動かす。
- 途中で Ctrl+C を押すと、移動を止める指令 (StopMove) を送ってから終わる (kill・SSH の切断でも同じ)。

しくみ:
- /api/sport/request に Request を publish すると Go2 が動く (High Level の Sport API)。
  どの動きをするかは header.identity.api_id で選び、速度などは parameter に JSON で入れる。
- Go2 の位置と向きは /sportmodestate (SportModeState) で受け取れる。
  前進や旋回は「決まった時間だけ動く」のではなく、これを見て決まった距離・角度で止めている。
"""

import json
import math
import signal
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from unitree_api.msg import Request
from unitree_go.msg import SportModeState

# /api/sport/request の api_id (動きの番号)
BALANCE_STAND = 1002  # 歩ける立ち方にする (StandUp のあとに送る)
STOP_MOVE = 1003  # 移動を止める
STAND_UP = 1004  # 立つ (関節を固めた立ち方。このままでは歩けない)
MOVE = 1008  # 速度で動く。parameter = {"x": 前後 [m/s], "y": 左右 [m/s], "z": 旋回 [rad/s]}
SIT = 1009  # 座る (急に動くので注意)
RISE_SIT = 1010  # 座った状態から立ち上がる
HELLO = 1016  # 挨拶 (前足を振る)

STATE_TIMEOUT = 0.5  # /sportmodestate がこの秒数途切れたら、位置を測れないとみなして止める [s]


class Go2:
    """Go2 に指令を送り、位置と向きを受け取る。"""

    def __init__(self):
        # Ctrl+C で ROS2 が先に終わると、止める指令を送れなくなる。
        # そこで ROS2 には Ctrl+C を扱わせず、main() で受けて StopMove を送ってから終わる。
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("go2_demo_script")
        self.pub = self.node.create_publisher(Request, "/api/sport/request", 10)
        self.node.create_subscription(SportModeState, "/sportmodestate", self.on_state, qos_profile_sensor_data)
        self.pose = None  # 最新の (x [m], y [m], yaw [rad])。まだ届いていなければ None
        self.pose_time = 0.0  # pose が届いた時刻

    def on_state(self, msg):
        """/sportmodestate が届くたびに呼ばれる。"""
        self.pose = (msg.position[0], msg.position[1], msg.imu_state.rpy[2])
        self.pose_time = time.time()

    def wait(self, sec):
        """sec 秒待つ (その間も /sportmodestate を受け取る)。"""
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_ready(self, timeout=5.0):
        """Go2 が見つかり、/sportmodestate が届くまで待つ (見つかる前に送ると届かないことがある)。"""
        t0 = time.time()
        while self.pub.get_subscription_count() == 0 or self.pose is None:
            if time.time() - t0 > timeout:
                raise RuntimeError("Go2 が見つかりません (有線 LAN の接続と、source ~/unitree_ros2/setup.sh をしたかを確かめてください)")
            self.wait(0.1)

    def send(self, api_id, parameter=None):
        """api_id の指令を 1 回送る。parameter (dict) は JSON にして入れる。"""
        msg = Request()
        msg.header.identity.api_id = api_id
        if parameter is not None:
            msg.parameter = json.dumps(parameter)
        self.pub.publish(msg)

    def move_until(self, x=0.0, z=0.0, distance=0.0, angle=0.0, timeout=10.0):
        """速度 x [m/s] (前後)・z [rad/s] (旋回、+ が左回り) で動き、
        distance [m] 進むか angle [度] 回ったら止める。

        動いている間は Move を 10 Hz で送り続け、そのたびに進んだ距離と回った角度を確かめる。
        位置が届かなくなったら (測れないまま歩き続けないように) 止める。
        """
        x0, y0, last_yaw = self.pose
        turned = 0.0  # これまでに回った角度 [rad]
        t0 = time.time()
        while True:
            self.send(MOVE, {"x": x, "y": 0, "z": z})
            self.wait(0.1)
            if time.time() - self.pose_time > STATE_TIMEOUT:
                self.send(STOP_MOVE)
                raise RuntimeError("/sportmodestate が途切れ、位置を測れなくなりました")
            px, py, yaw = self.pose
            # yaw は -180°〜+180° で飛ぶので、前回からの差を -180°〜+180° に直して足していく
            turned += (yaw - last_yaw + math.pi) % (2 * math.pi) - math.pi
            last_yaw = yaw
            if distance and math.hypot(px - x0, py - y0) >= distance:
                break
            if angle and abs(math.degrees(turned)) >= angle:
                break
            if time.time() - t0 > timeout:
                self.send(STOP_MOVE)
                raise RuntimeError(f"{timeout:g} 秒たっても目標の距離・角度に届きません")
        self.send(STOP_MOVE)

    def close(self):
        self.node.destroy_node()
        rclpy.try_shutdown()


def main():
    # kill (SIGTERM) や SSH の切断 (SIGHUP) でも、Ctrl+C と同じく止めてから終わる
    def on_signal(signum, frame):
        raise KeyboardInterrupt
    for name in ("SIGTERM", "SIGHUP"):
        signal.signal(getattr(signal, name), on_signal)

    go2 = Go2()
    try:
        go2.wait_ready()
        input("周りを空け、リモコンですぐ止められる状態で Enter (やめるなら Ctrl+C) > ")

        print("立つ")
        go2.send(STAND_UP)
        go2.wait(3)
        go2.send(BALANCE_STAND)
        go2.wait(1)

        print("前進 0.5 m")
        go2.move_until(x=0.3, distance=0.5)
        go2.wait(1)

        print("左に 90°")
        go2.move_until(z=0.8, angle=90)
        go2.wait(1)

        print("挨拶")
        go2.send(HELLO)
        go2.wait(5)

        print("座る")  # 急に動くので、周り (特に後ろ) を空けておく
        go2.send(SIT)
        go2.wait(5)

        print("立ち上がる")
        go2.send(RISE_SIT)
        go2.wait(3)
        print("終わりました")
    except KeyboardInterrupt:
        go2.send(STOP_MOVE)
        print("\n止めました。必要ならリモコンで伏せさせてください")
    except (RuntimeError, EOFError) as e:
        go2.send(STOP_MOVE)
        print(f"中止しました: {e}")
    finally:
        go2.close()


if __name__ == "__main__":
    main()
