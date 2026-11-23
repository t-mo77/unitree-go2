"""Go2 の前のカメラの最新の 1 枚を、ページに出せる画像（JPEG か PNG のバイト列）として持つ。

- シミュレーション（SimCamera）：Gazebo の画像カメラ（GO2_SIM_CAMERA=1 で付く）の
  /gazebo/front_camera/image（sensor_msgs/Image）を受け、PNG にする。偽の Go2 には画像が無い。
- 実機（VideoHubCamera）：Go2 の動画サービス videohub に GetImageSample（API 1001）を頼み、
  返ってきた JPEG（Response.binary）をそのまま使う。unitree_sdk2 の VideoClient と同じ窓口。
  **まだ実機で試していない。** ページが画像を見ている間（最後の要求から DEMAND_S 秒）だけ頼む。

どちらも latest() で (受けた時刻 time.monotonic(), Content-Type, バイト列) か None を返す。
"""

import logging
import struct
import threading
import time
import zlib

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

SIM_IMAGE_TOPIC = "/gazebo/front_camera/image"
VIDEO_REQUEST_TOPIC = "/api/videohub/request"
VIDEO_RESPONSE_TOPIC = "/api/videohub/response"
GET_IMAGE_SAMPLE = 1001
FRAME_STALE_S = 2.0   # これより古い画像は出さない [s]
DEMAND_S = 2.0        # ページの最後の要求からこの秒数は、実機に画像を頼み続ける [s]
VIDEO_PERIOD_S = 0.2  # 実機に画像を頼む間隔 [s]
REPLY_TIMEOUT_S = 1.0
log = logging.getLogger(__name__)

_CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}


def png_bytes(pixels: np.ndarray) -> bytes:
    """H×W×3（RGB）か H×W（グレー）の uint8 の配列を PNG にする（標準ライブラリだけ）。"""
    pixels = np.ascontiguousarray(pixels, dtype=np.uint8)
    height, width = pixels.shape[:2]
    color = 2 if pixels.ndim == 3 else 0
    rows = pixels.reshape(height, -1)
    raw = np.hstack([np.zeros((height, 1), dtype=np.uint8), rows]).tobytes()   # 各行に filter 0

    def chunk(kind, data):
        return (struct.pack(">I", len(data)) + kind + data
                + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF))

    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, color, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 1)) + chunk(b"IEND", b""))


def image_pixels(msg) -> np.ndarray:
    """sensor_msgs/Image を H×W×3（RGB）か H×W の配列にする。扱えない形式は ValueError。"""
    channels = _CHANNELS.get(msg.encoding)
    if channels is None:
        raise ValueError(f"扱えない画像の形式です: {msg.encoding}")
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    rows = data.reshape(msg.height, msg.step)[:, :msg.width * channels]
    if channels == 1:
        return rows
    pixels = rows.reshape(msg.height, msg.width, channels)[:, :, :3]
    return pixels[:, :, ::-1] if msg.encoding.startswith("bgr") else pixels


class SimCamera:
    """Gazebo の画像カメラの最新の 1 枚。PNG にするのはページが求めたときだけ。"""

    def __init__(self, node):
        self._msg = None
        self._cache = None
        node.create_subscription(Image, SIM_IMAGE_TOPIC, self._on_image, qos_profile_sensor_data)

    def _on_image(self, msg):
        self._msg = (time.monotonic(), msg)

    def latest(self):
        received = self._msg
        if received is None or time.monotonic() - received[0] > FRAME_STALE_S:
            return None
        if self._cache is None or self._cache[0] != received[0]:
            try:
                self._cache = (received[0], "image/png", png_bytes(image_pixels(received[1])))
            except ValueError as exc:
                log.warning("%s", exc)
                return None
        return self._cache

    def close(self):
        pass


class VideoHubCamera:
    """実機の動画サービスに JPEG を頼む。ページが見ている間だけ、VIDEO_PERIOD_S ごとに頼む。"""

    def __init__(self):
        from unitree_api.msg import Request, Response
        self._request_type = Request
        self.node = rclpy.create_node("go2_follow_camera")
        self._pub = self.node.create_publisher(Request, VIDEO_REQUEST_TOPIC, 10)
        self.node.create_subscription(Response, VIDEO_RESPONSE_TOPIC, self._on_response, 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._identity = time.monotonic_ns()
        self._waiting = None
        self._frame = None
        self._demand = 0.0
        self._warned = False
        self._halt = threading.Event()
        self._thread = threading.Thread(target=self._run, name="go2-follow-camera", daemon=True)
        self._thread.start()

    def _on_response(self, msg):
        identity = msg.header.identity
        if (identity.id, identity.api_id) != self._waiting:
            return
        self._waiting = None
        if msg.header.status.code != 0 or not len(msg.binary):
            if not self._warned:
                log.warning("Go2 の動画サービスが画像を返しません（code %s）", msg.header.status.code)
                self._warned = True
            return
        data = np.asarray(msg.binary, dtype=np.int8).tobytes()
        kind = "image/jpeg" if data[:2] == b"\xff\xd8" else "application/octet-stream"
        self._frame = (time.monotonic(), kind, data)

    def _run(self):
        while not self._halt.is_set():
            if time.monotonic() - self._demand > DEMAND_S:
                self._halt.wait(VIDEO_PERIOD_S)
                continue
            self._identity += 1
            request = self._request_type()
            request.header.identity.id = self._identity
            request.header.identity.api_id = GET_IMAGE_SAMPLE
            self._waiting = (self._identity, GET_IMAGE_SAMPLE)
            self._pub.publish(request)
            end = time.monotonic() + REPLY_TIMEOUT_S
            while self._waiting is not None and not self._halt.is_set() and time.monotonic() < end:
                self._executor.spin_once(timeout_sec=0.05)
            self._halt.wait(VIDEO_PERIOD_S)

    def latest(self):
        self._demand = time.monotonic()
        frame = self._frame
        if frame is None or time.monotonic() - frame[0] > FRAME_STALE_S:
            return None
        return frame

    def close(self):
        self._halt.set()
        self._thread.join(timeout=2)
        self._executor.shutdown()
        self.node.destroy_node()
