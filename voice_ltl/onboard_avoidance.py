"""Go2本体の障害物回避サービス。速度指令をSport.Moveへ直接送らない。"""
import json
import logging
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from unitree_api.msg import Request, Response

from voice_ltl.publisher_guard import PublisherGuard

REQUEST_TOPIC = "/api/obstacles_avoid/request"
RESPONSE_TOPIC = "/api/obstacles_avoid/response"
SWITCH_GET, MOVE, USE_API = 1002, 1003, 1004
log = logging.getLogger(__name__)


class OnboardAvoidance:
    def __init__(self, node=None, start_thread=True):
        self.node = node or rclpy.create_node("go2_onboard_avoidance")
        self.pub = self.node.create_publisher(Request, REQUEST_TOPIC, 10)
        # SwitchGetはGo2自身も高頻度に送る読み取り照会で、操作の競合ではない。
        self.guard = PublisherGuard(own=1, topic=REQUEST_TOPIC,
                                    read_only_apis=(SWITCH_GET,), activity_only=True)
        self.node.create_subscription(Request, REQUEST_TOPIC, self.guard.on_request, qos_profile_sensor_data)
        self.node.create_subscription(Response, RESPONSE_TOPIC, self._on_response, 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._pending = {}
        self._identity = time.monotonic_ns()
        self.owned = False
        self.prepared = False
        self.thread = None
        if start_thread:
            self.thread = threading.Thread(target=self._spin, name="go2-avoidance-replies", daemon=True)
            self.thread.start()

    def _spin(self):
        try:
            while not self._stop.is_set():
                self.executor.spin_once(timeout_sec=0.05)
        except (ExternalShutdownException, RuntimeError):
            if not self._stop.is_set():
                log.exception("本体回避サービスの受信処理が停止しました")
                self.prepared = False

    def _on_response(self, message):
        with self._lock:
            pending = self._pending.get(int(message.header.identity.id))
            if pending is not None and pending["api"] == int(message.header.identity.api_id):
                pending["response"] = message
                pending["event"].set()

    def _publish(self, api, parameter, reply=False):
        with self._lock:
            self._identity += 1
            identity = self._identity
            pending = {"api": api, "event": threading.Event()}
            if reply:
                self._pending[identity] = pending
        message = Request()
        message.header.identity.id = identity
        message.header.identity.api_id = api
        message.header.policy.noreply = not reply
        message.parameter = json.dumps(parameter)
        self.guard.remember_own(message)
        try:
            self.pub.publish(message)
        except BaseException:
            with self._lock:
                self._pending.pop(identity, None)
            raise
        return identity, pending

    def _call(self, api, parameter):
        identity, pending = self._publish(api, parameter, reply=True)
        try:
            if not pending["event"].wait(2):
                raise RuntimeError(f"Go2本体の障害物回避サービスが応答しません（API {api}）")
            response = pending["response"]
            if response.header.status.code != 0:
                raise RuntimeError(f"Go2本体の障害物回避サービスが拒否しました（API {api}, code {response.header.status.code}）")
            try:
                data = json.loads(response.data or "{}")
            except ValueError as exc:
                raise RuntimeError("Go2の回避サービスの応答を読めません。") from exc
            if not isinstance(data, dict):
                raise RuntimeError("Go2の回避サービスの応答形式が不正です。")
            return data
        finally:
            with self._lock:
                self._pending.pop(identity, None)

    def prepare(self):
        end = time.monotonic() + 3
        # 1つは自分の監視用の購読者。
        while self.pub.get_subscription_count() <= 1 and time.monotonic() < end:
            time.sleep(0.05)
        if self.pub.get_subscription_count() <= 1:
            raise RuntimeError("Go2の障害物回避サービスが見つかりません。通常Moveには切り替えません。")
        state = self._call(SWITCH_GET, {})
        if state.get("enable") is not True:
            raise RuntimeError("Go2本体の障害物回避が無効です。Unitreeアプリで有効にしてから起動してください。")
        self.guard.freeze(self.node)
        if self.guard.conflicts(self.node):
            raise RuntimeError("Go2の障害物回避APIにほかの操作元があります。")
        # 利用者が実機モードを起動したときのみAPI操作に切り替える。終了時に戻す。
        self.owned = True  # 応答が失われた場合にもcloseで解除を試す
        self._call(USE_API, {"is_remote_commands_from_api": True})
        self.prepared = True
        log.warning("障害物による移動可否はGo2本体が判断します。API操作モードを使用し、終了時に解除します。")

    def move(self, parameter):
        if not self.prepared or self.guard.conflicts(self.node):
            raise RuntimeError("本体回避サービスが未準備か、ほかの操作元を検出したため移動しません。")
        self._publish(MOVE, {"x": parameter["x"], "y": parameter["y"],
                             "yaw": parameter["z"], "mode": 0})

    def stop(self):
        if self.owned:
            self._publish(MOVE, {"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": 0})

    def close(self):
        try:
            if self.owned:
                self.stop()
                self._call(USE_API, {"is_remote_commands_from_api": False})
        finally:
            self.owned = self.prepared = False
            self._stop.set()
            if self.thread is not None:
                self.thread.join(timeout=3)
            self.executor.shutdown()
            self.node.destroy_node()
