"""ROS 2 の窓口：実機（またはにせ Go2）と同じトピックで指令を送り、状態と点群を受け取る。

RosLink は Session の Link（send / posture / pose / reading）を満たす。
安全の仕組みは zip の nl_control.py から移した:
- 二重起動を /tmp/go2_voice_ltl.lock の flock で防ぐ（作った時点でロックを取る）。
- /api/sport/request に自分以外の送り手がいたら、起動を断る（wait_ready）。
- SIGTERM・SIGHUP を Ctrl+C と同じ KeyboardInterrupt にする（メインスレッドで作ったとき）。
  呼び出し側は finally で close(3) を呼ぶ。呼び忘れても、プロセスの終わりに close(3) が走る（atexit）。
- close は StopMove を送り切るまで、Ctrl+C・SIGTERM・SIGHUP を受け流す。
- /sportmodestate が 0.5 秒途切れたら、前後とも SENSOR_LOST にする（移動を保留させる）。

時刻 now はすべて time.time() で渡す。
"""

import atexit
import fcntl
import json
import signal
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor, TimeoutException
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from unitree_api.msg import Request
from unitree_go.msg import SportModeState

from go2_nl_control.actions import state_from_error_code
from go2_nl_control.executor import STOP, Command
from go2_nl_control.perception import PerceptionConfig
from go2_nl_control.supervisor import SENSOR_LOST

from voice_ltl.sensing import Reading, Sensing

# 実機と同じトピック
REQUEST_TOPIC = "/api/sport/request"
STATE_TOPIC = "/sportmodestate"
LIDAR_TOPIC = "/utlidar/cloud"
# シミュレーションだけのトピック（にせ Go2・Gazebo の橋渡しが受ける）
OBSTACLES_TOPIC = "/sim/obstacles"
WORLD_TOPIC = "/sim/world"
OBSTACLE_CMDS = ("front", "back", "clear")

LOCK_FILE = "/tmp/go2_voice_ltl.lock"
STATE_TIMEOUT = 0.5     # /sportmodestate がこの秒数途切れたら SENSOR_LOST [s]
_MAX_CALLBACKS = 500    # spin_once 1 回で処理する受信の上限（届き続けても戻ってこられるように）

NOT_FOUND = "Go2 が見つかりません (有線 LAN の接続と、source ~/unitree_ros2/setup.sh をしたかを確かめてください)"

Pose = Tuple[float, float, float]   # (x [m], y [m], yaw [rad])


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


class RosLink:
    def __init__(self, perception: PerceptionConfig, node_name: str = "go2_voice_ltl"):
        # 同じプログラムの二重起動を防ぐ。2つ目が送り手として見えないよう、ROS 2 より先に取る
        self._lock = open(LOCK_FILE, "w")
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self._lock.close()
            raise RuntimeError("go2_voice_ltl はすでに動いています。2つ同時には起動できません")

        try:
            # Ctrl+C で ROS 2 が先に終わると止める指令を送れなくなるので、ROS 2 には Ctrl+C を扱わせない
            self._owns_rclpy = not rclpy.ok()
            if self._owns_rclpy:
                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            self.node = rclpy.create_node(node_name)
            self._pub = self.node.create_publisher(Request, REQUEST_TOPIC, 10)
            self.node.create_subscription(SportModeState, STATE_TOPIC, self._on_state, qos_profile_sensor_data)
            self.node.create_subscription(PointCloud2, LIDAR_TOPIC, self._on_cloud, qos_profile_sensor_data)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self.node)
        except BaseException:
            # 失敗したらロックを離す（開いたまま残さない）
            self._lock.close()
            raise

        self._sensing = Sensing(perception)
        self._pose: Optional[Pose] = None
        self._error_code: Optional[int] = None
        self._state_time: Optional[float] = None
        self._cloud_time: Optional[float] = None
        self._req_id = 0
        self._send_lock = threading.Lock()   # Web の「止める」は別のスレッドから send する
        self._closed = False

        # kill (SIGTERM) や SSH の切断 (SIGHUP) でも、Ctrl+C と同じく止めてから終わる
        if threading.current_thread() is threading.main_thread():
            for sig in (signal.SIGTERM, signal.SIGHUP):
                signal.signal(sig, _raise_keyboard_interrupt)
        atexit.register(self.close)

    # ---------- 受信 ----------
    def _on_state(self, msg: SportModeState) -> None:
        self._pose = (float(msg.position[0]), float(msg.position[1]), float(msg.imu_state.rpy[2]))
        self._error_code = int(msg.error_code)
        self._state_time = time.time()

    def _on_cloud(self, msg: PointCloud2) -> None:
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        self._cloud_time = time.time()
        self._sensing.update(pts, self._cloud_time)

    def spin_once(self) -> None:
        """届いている受信をすべて処理する（待たない）。制御ループから毎周期呼ぶ。"""
        for _ in range(_MAX_CALLBACKS):
            try:
                handler, _, _ = self._executor.wait_for_ready_callbacks(timeout_sec=0.0)
            except TimeoutException:
                return
            handler()
            if handler.exception() is not None:
                raise handler.exception()

    def _spin_for(self, sec: float) -> None:
        end = time.time() + sec
        while True:
            self._executor.spin_once(timeout_sec=max(0.0, min(0.01, end - time.time())))
            if time.time() >= end:
                break

    # ---------- Link ----------
    def send(self, cmd: Command) -> None:
        msg = Request()
        with self._send_lock:
            self._req_id += 1
            msg.header.identity.id = self._req_id
            msg.header.identity.api_id = cmd.api_id
            if cmd.parameter is not None:
                msg.parameter = json.dumps(cmd.parameter)
            self._pub.publish(msg)

    def posture(self) -> str:
        """Go2 が報告する姿勢（error_code から）。まだ届いていなければ UNKNOWN。"""
        return state_from_error_code(self._error_code)

    def pose(self) -> Optional[Pose]:
        return self._pose

    def error_code(self) -> Optional[int]:
        """最後に受けた /sportmodestate の error_code。まだ届いていなければ None。"""
        return self._error_code

    def reading(self, now: float) -> Reading:
        r = self._sensing.current(now)
        if self._state_time is None or now - self._state_time > STATE_TIMEOUT:
            # 位置が測れないまま動かないよう、前後とも途切れ扱いにする（nl_control.py の env と同じ）
            return Reading(SENSOR_LOST, SENSOR_LOST, r.front_dist, r.back_dist)
        return r

    # ---------- 確認 ----------
    def other_senders(self) -> int:
        """/api/sport/request に指令を送っている、自分以外のプログラムの数。"""
        return max(0, self.node.count_publishers(REQUEST_TOPIC) - 1)

    def wait_ready(self, timeout_s: float = 5.0) -> None:
        """Go2 が見つかり /sportmodestate が届くまで待つ。見つからない・別の送り手がいれば RuntimeError。"""
        t0 = time.time()
        while self._pub.get_subscription_count() == 0 or self._pose is None:
            if time.time() - t0 > timeout_s:
                raise RuntimeError(NOT_FOUND)
            self._spin_for(0.1)
        self._spin_for(1.0)  # 他の送り手が見えるまで少し待つ
        if self.other_senders():
            raise RuntimeError(f"{REQUEST_TOPIC} に指令を送っている別のプログラムがあります。"
                               "そちらを終了してから起動してください")
        if self._cloud_time is None or time.time() - self._cloud_time > 1.0:
            print(f"注意: {LIDAR_TOPIC} が届いていません。届くまで移動は保留されます", flush=True)

    # ---------- 終了 ----------
    def close(self, stop_times: int = 3) -> None:
        """StopMove を stop_times 回送り（取りこぼしに備える）、ノードを閉じてロックを外す。2回目からは何もしない。"""
        if self._closed:
            return
        self._closed = True
        atexit.unregister(self.close)
        # 送っている間に Ctrl+C や SIGTERM が来ても、StopMove を送り切る
        saved = {}
        if threading.current_thread() is threading.main_thread():
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                saved[sig] = signal.signal(sig, signal.SIG_IGN)
        try:
            for _ in range(stop_times):
                self.send(STOP)
                time.sleep(0.05)
        except Exception as e:  # 送れなくても、後片付けは続ける
            print(f"StopMove を送れませんでした: {e}", flush=True)
        finally:
            for sig, handler in saved.items():
                signal.signal(sig, handler if handler is not None else signal.SIG_DFL)
            self._executor.shutdown()
            self.node.destroy_node()
            if self._owns_rclpy:
                rclpy.try_shutdown()
            self._lock.close()


class SimControl:
    """シミュレーションの箱を出し入れし（/sim/obstacles）、最後に届いた様子（/sim/world）を覚える。

    node には RosLink.node を渡す。受信は RosLink.spin_once で処理される。
    """

    def __init__(self, node):
        self._pub = node.create_publisher(String, OBSTACLES_TOPIC, 10)
        node.create_subscription(String, WORLD_TOPIC, self._on_world, qos_profile_sensor_data)
        self._world: Optional[dict] = None

    def obstacle(self, cmd: str) -> None:
        """cmd = "front"（前に箱を置く）/ "back"（後ろに箱を置く）/ "clear"（全部どける）。"""
        if cmd not in OBSTACLE_CMDS:
            raise ValueError(f"cmd は {' / '.join(OBSTACLE_CMDS)} のどれかです: {cmd!r}")
        self._pub.publish(String(data=json.dumps({"cmd": cmd})))

    def latest_world(self) -> Optional[dict]:
        """{"boxes": [[x, y, size], ...], "collided": bool}。まだ届いていなければ None。"""
        return self._world

    def _on_world(self, msg: String) -> None:
        try:
            world = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(world, dict):   # list など dict 以外は保存しない
            self._world = world
