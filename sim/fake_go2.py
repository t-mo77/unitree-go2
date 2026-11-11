"""にせ Go2 の ROS 2 ノード。トピックと型は実機と同じで、中身は sim_core.SimGo2。

- /api/sport/request（unitree_api/Request）を受けて SimGo2.handle に渡す。
- 50 Hz で step を回し、/sportmodestate（unitree_go/SportModeState）を出す。
  position[0..1]・imu_state.rpy[2]・error_code を入れる。
- 10 Hz で /utlidar/cloud（PointCloud2、frame utlidar_lidar、x/y/z float32）を出す。
- /sim/obstacles（std_msgs/String、{"cmd": "front" | "back" | "clear"}）で箱を出し入れし、
  /sim/world（{"boxes": [[x, y, size], ...], "collided": bool}）を 10 Hz と箱を動かしたときに出す。
- 名前つきの物を環境の GO2_LAYOUT の配置ファイル（gazebo/layouts/<名前>.json、既定 default：box_1・chair_1）の
  通りに置き、前カメラ（FakeCamera）で見た物の一覧を 2 Hz で /go2/objects（std_msgs/String の JSON、odom、
  時刻は time.time()、"source": "fake"）に出す。物は /sim/world には入らない。知らない GO2_LAYOUT なら、
  わけ（名前と、ある配置の名前）を 1 行書いて止まる。

実機に移したら、このノードを起動しないだけで本物の Go2 と入れ替わる。
使い方: python3 -m sim.fake_go2
"""

import json
import math
import os
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from unitree_api.msg import Request
from unitree_go.msg import SportModeState

from go2_nl_control import actions as A
from go2_nl_control.perception import PerceptionConfig
from sim.sim_core import FakeCamera, SimGo2, layout_objects
from voice_ltl.layout import LayoutError
from voice_ltl.objects import OBJECTS_HZ, format_message
# トピック名は voice_ltl.ros_link を正とする（実機・にせ Go2 でずれないように）
from voice_ltl.ros_link import (LIDAR_TOPIC, OBJECTS_TOPIC, OBSTACLES_TOPIC, REQUEST_TOPIC, STATE_TOPIC,
                                WORLD_TOPIC)

LIDAR_FRAME = "utlidar_lidar"

STATE_HZ = 50.0
CLOUD_HZ = 10.0

# 動作名は go2_nl_control.actions の api_id 定数（BALANCE_STAND = 1002 など）から作る
_API_NAMES = {v: "".join(w.capitalize() for w in k.split("_"))
              for k, v in vars(A).items()
              if k.isupper() and isinstance(v, int) and not isinstance(v, bool) and v >= 1000}


def _api_name(api_id: int) -> str:
    return f"{api_id} ({_API_NAMES.get(api_id, '?')})"


def objects_message(camera: FakeCamera, sim: SimGo2, now: float) -> dict:
    """/go2/objects の 1 通：camera が sim の中に見た物（行は Obj.to_row() のまま。confidence・measured はない）。"""
    return format_message(camera.see(sim, now), now, source="fake")


class FakeGo2(Node):
    def __init__(self, sim: Optional[SimGo2] = None):
        if sim is None:                    # the layout first: a wrong GO2_LAYOUT raises before any node exists
            sim = SimGo2(PerceptionConfig())
            sim.place_objects(*layout_objects(os.environ.get("GO2_LAYOUT", "default")))
        super().__init__("fake_go2")       # 渡された sim の物はそのまま
        self.sim = sim
        self.camera = FakeCamera()
        # 実機と同じく、状態と点群は best effort で出す
        self._state_pub = self.create_publisher(SportModeState, STATE_TOPIC, qos_profile_sensor_data)
        self._cloud_pub = self.create_publisher(PointCloud2, LIDAR_TOPIC, qos_profile_sensor_data)
        self._world_pub = self.create_publisher(String, WORLD_TOPIC, 10)
        self._objects_pub = self.create_publisher(String, OBJECTS_TOPIC, 10)
        self.create_subscription(Request, REQUEST_TOPIC, self._on_request, qos_profile_sensor_data)
        self.create_subscription(String, OBSTACLES_TOPIC, self._on_obstacles, 10)
        self.create_timer(1.0 / STATE_HZ, self._on_state_timer)
        self.create_timer(1.0 / CLOUD_HZ, self._on_cloud_timer)
        self.create_timer(1.0 / OBJECTS_HZ, self._on_objects_timer)
        self._last_code = self.sim.error_code
        self._last_collided = self.sim.collided

    # ---------- 受信 ----------
    def _on_request(self, msg: Request) -> None:
        api_id = int(msg.header.identity.api_id)
        n = len(self.sim.accepted)
        self.sim.handle(api_id, msg.parameter, time.monotonic())
        if api_id == A.MOVE:   # Move は 10 Hz で届くので、歩き出し・止まりは error_code の変化で出す
            return
        if len(self.sim.accepted) > n:
            self.get_logger().info(f"受け付け: {_api_name(api_id)}")
        else:
            self.get_logger().info(f"受け付けません: {_api_name(api_id)}（error_code {self.sim.error_code}）")

    def _on_obstacles(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data).get("cmd")
        except (ValueError, AttributeError):
            cmd = None
        if cmd in ("front", "back"):
            box = self.sim.place_box(cmd)
            self.get_logger().info(f"箱を置きました（{cmd}）: x={box.x:.2f} y={box.y:.2f}")
        elif cmd == "clear":
            self.sim.clear_boxes()
            self.get_logger().info("箱を全部どけました")
        else:
            self.get_logger().warning(f"{OBSTACLES_TOPIC} の中身がわかりません: {msg.data!r}")
            return
        self._publish_world()

    # ---------- 送信 ----------
    def _on_state_timer(self) -> None:
        self.sim.step(time.monotonic())
        msg = SportModeState()
        sec, nanosec = self.get_clock().now().seconds_nanoseconds()
        msg.stamp.sec, msg.stamp.nanosec = sec, nanosec
        msg.error_code = self.sim.error_code
        msg.position = [float(self.sim.x), float(self.sim.y), 0.0]
        yaw = math.atan2(math.sin(self.sim.yaw), math.cos(self.sim.yaw))   # 実機の IMU と同じ -π〜π
        msg.imu_state.rpy = [0.0, 0.0, yaw]
        self._state_pub.publish(msg)

        if self.sim.error_code != self._last_code:
            self.get_logger().info(f"error_code {self._last_code} → {self.sim.error_code}（{self.sim.posture}）")
            self._last_code = self.sim.error_code
        if self.sim.collided != self._last_collided:
            if self.sim.collided:
                self.get_logger().warning(f"箱にぶつかりました: x={self.sim.x:.2f} y={self.sim.y:.2f}")
            self._last_collided = self.sim.collided

    def _on_cloud_timer(self) -> None:
        header = Header(frame_id=LIDAR_FRAME, stamp=self.get_clock().now().to_msg())
        pts = np.ascontiguousarray(self.sim.cloud_lidar(), dtype=np.float32)
        self._cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))
        self._publish_world()

    def _on_objects_timer(self) -> None:
        msg = objects_message(self.camera, self.sim, time.time())
        self._objects_pub.publish(String(data=json.dumps(msg)))

    def _publish_world(self) -> None:
        world = {"boxes": [[round(b.x, 3), round(b.y, 3), b.size] for b in self.sim.boxes],
                 "collided": bool(self.sim.collided)}
        self._world_pub.publish(String(data=json.dumps(world)))


def main() -> None:
    rclpy.init()
    try:
        node = FakeGo2()
    except LayoutError as e:               # one line naming the file, the value and the layouts, no traceback
        rclpy.try_shutdown()
        raise SystemExit(f"fake_go2: GO2_LAYOUT: {e}") from None
    node.get_logger().info(f"にせ Go2 を起動しました（{node.sim.posture}、error_code {node.sim.error_code}）")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
