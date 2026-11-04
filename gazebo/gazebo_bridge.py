"""Gazebo（Harmonic）の Go2 の橋渡し（gazebo コンテナの ROS 2 Jazzy で動くノード）。

アプリから見ると実機の Go2 と同じ窓口になる。中身は bridge_core.BridgeCore。

- /api/sport/request（unitree_api/Request）を受け、unitree_guide controller の /control_input
  （control_input_msgs/Inputs）に変えて送る。
- Gazebo の本当の位置（/model/robot/odometry、nav_msgs/Odometry）から /sportmodestate
  （unitree_go/SportModeState）を作る。position・imu_state.rpy・error_code などを入れる。
- Gazebo の LiDAR の点群（/gazebo/lidar/points）から自分の体の点と地面の点（世界の z < 0.05 m）を除き、
  /utlidar/cloud（frame utlidar_lidar）に出し直す。
- /sim/obstacles（std_msgs/String、{"cmd": "front" | "back" | "clear"}）で Gazebo に箱を出し入れし
  （/world/empty/create・/world/empty/remove）、/sim/world（{"boxes": [[x, y, size], ...], "collided": bool}）を出す。

時計は 2 つ。窓口の時間（error_code が変わるまで、Move の 1 秒の期限）はアプリと同じ実際の時間、
controller へ command を送る間隔はシミュレーションの時間（/clock）で数える（bridge_core.py の説明）。
controller が本当に「歩く」に入った時刻は、controller が /rosout に出す "Switched from fixed stand to
trotting" で知り、Move の速度はその 0.5 秒後（シミュレーションの時間）から入れる（Ruling R22）。
位置はいつも Gazebo の本当の位置なので、距離は Gazebo の速さ（RTF）によらない。
Humble 側の相手のノードの名前は分からない（Gid の長さの違い）ので使わない。

go2_gazebo.launch.py が起動する。自分で起動するなら（gazebo コンテナの /work で）:
    PYTHONPATH=/work:/work/nl_control:$PYTHONPATH python3 -m gazebo.gazebo_bridge
"""

import json
import math
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from control_input_msgs.msg import Inputs
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from unitree_api.msg import Request
from unitree_go.msg import SportModeState

from go2_nl_control.actions import MOVE
from gazebo.bridge_core import (BoxWorld, BridgeCore, ControllerInput, api_name, box_sdf,
                                drop_ground_and_self_points)
from sim.sim_core import BOX_HEIGHT

# 実機と同じトピック
REQUEST_TOPIC = "/api/sport/request"
STATE_TOPIC = "/sportmodestate"
LIDAR_TOPIC = "/utlidar/cloud"
LIDAR_FRAME = "utlidar_lidar"
# シミュレーションだけのトピック
OBSTACLES_TOPIC = "/sim/obstacles"
WORLD_TOPIC = "/sim/world"
# Gazebo 側（go2_gazebo.launch.py の ros_gz_bridge でつなぐ）
WORLD_NAME = "empty"          # gz_quadruped_playground の worlds/default.sdf の world の名前
MODEL_NAME = "robot"
CONTROL_TOPIC = "/control_input"
ODOM_TOPIC = f"/model/{MODEL_NAME}/odometry"
RAW_CLOUD_TOPIC = "/gazebo/lidar/points"
ROSOUT_TOPIC = "/rosout"
TROT_SWITCH_LOG = "Switched from fixed stand to trotting"   # unitree_guide controller のログ
CREATE_SERVICE = f"/world/{WORLD_NAME}/create"
REMOVE_SERVICE = f"/world/{WORLD_NAME}/remove"

TICK_HZ = 20.0     # BridgeCore.step を回す周期（実際の時間）
WORLD_HZ = 10.0

# (x, y, z, roll, pitch, yaw, vx, vy, vz, yaw_speed)。速度は世界の座標
Pose = Tuple[float, float, float, float, float, float, float, float, float, float]


def _rpy(q) -> Tuple[float, float, float]:
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


class GazeboBridge(Node):
    def __init__(self):
        super().__init__("go2_gazebo_bridge",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.core = BridgeCore(*self._now(), reports_trot_entry=True)
        self.boxes = BoxWorld()
        self._pose: Optional[Pose] = None
        self._quat = (1.0, 0.0, 0.0, 0.0)   # (w, x, y, z)
        self._last_note = ""
        self._last_code = self.core.error_code

        self._control_pub = self.create_publisher(Inputs, CONTROL_TOPIC, 10)
        # 実機と同じく、状態と点群は best effort で出す
        self._state_pub = self.create_publisher(SportModeState, STATE_TOPIC, qos_profile_sensor_data)
        self._cloud_pub = self.create_publisher(PointCloud2, LIDAR_TOPIC, qos_profile_sensor_data)
        self._world_pub = self.create_publisher(String, WORLD_TOPIC, 10)
        self.create_subscription(Request, REQUEST_TOPIC, self._on_request, qos_profile_sensor_data)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, RAW_CLOUD_TOPIC, self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(String, OBSTACLES_TOPIC, self._on_obstacles, 10)
        # controller のログ。過去の分は要らないので volatile で受ける
        self.create_subscription(Log, ROSOUT_TOPIC, self._on_log, QoSProfile(
            depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
        self._create = self.create_client(SpawnEntity, CREATE_SERVICE)
        self._remove = self.create_client(DeleteEntity, REMOVE_SERVICE)
        wall = Clock(clock_type=ClockType.STEADY_TIME)   # Gazebo が止まっても回る
        self.create_timer(1.0 / TICK_HZ, self._on_tick, clock=wall)
        self.create_timer(1.0 / WORLD_HZ, self._publish_world, clock=wall)

    def _now(self) -> Tuple[float, float]:
        """(実際の時間, シミュレーションの時間)"""
        return time.monotonic(), self.get_clock().now().nanoseconds * 1e-9

    # ---------- 指令 ----------
    def _on_request(self, msg: Request) -> None:
        api_id = int(msg.header.identity.api_id)
        if self._pose is None or self._control_pub.get_subscription_count() == 0:
            self.get_logger().warning(
                f"Gazebo の Go2 か controller がまだ起動していません。{api_name(api_id)} を無視します")
            return
        if api_id != MOVE:   # Move は 10 Hz で届くので、歩き出し・止まりは error_code の変化で出す
            self.get_logger().info(f"受信: {api_name(api_id)}")
        self.core.handle(api_id, msg.parameter, *self._now())
        self._log_notes()

    def _on_tick(self) -> None:
        sent = self.core.step(*self._now())
        if sent is not None:
            self._send(sent)
        self._log_notes()
        if self.core.error_code != self._last_code:
            self.get_logger().info(f"error_code {self._last_code} → {self.core.error_code}")
            self._last_code = self.core.error_code

    def _on_log(self, msg: Log) -> None:
        if msg.msg.startswith(TROT_SWITCH_LOG):
            self.core.trot_entered(self._now()[1])

    def _send(self, i: ControllerInput) -> None:
        self._control_pub.publish(Inputs(command=i.command, lx=float(i.lx), ly=float(i.ly),
                                         rx=float(i.rx), ry=float(i.ry)))
        self.get_logger().info(
            f"controller へ: command {i.command}, lx {i.lx:.2f}, ly {i.ly:.2f}, rx {i.rx:.2f}")

    def _log_notes(self) -> None:
        for note in self.core.notes:
            if note != self._last_note:   # 伏せたまま 10 Hz で届く Move などを何度も出さない
                self.get_logger().info(note)
                self._last_note = note
        self.core.notes.clear()           # ログに出したものは持たない（長く動かしても増えない）

    # ---------- 位置と向き ----------
    def _on_odom(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        roll, pitch, yaw = _rpy(q)
        v, w = msg.twist.twist.linear, msg.twist.twist.angular   # base の座標
        c, s = math.cos(yaw), math.sin(yaw)
        self._pose = (p.x, p.y, p.z, roll, pitch, yaw, c * v.x - s * v.y, s * v.x + c * v.y, v.z, w.z)
        self._quat = (q.w, q.x, q.y, q.z)

        st = SportModeState()
        st.stamp.sec, st.stamp.nanosec = msg.header.stamp.sec, msg.header.stamp.nanosec
        st.error_code = int(self.core.error_code)
        st.position = [float(p.x), float(p.y), float(p.z)]
        st.body_height = float(p.z)
        st.velocity = [float(self._pose[6]), float(self._pose[7]), float(v.z)]
        st.yaw_speed = float(w.z)
        st.imu_state.rpy = [float(roll), float(pitch), float(yaw)]
        st.imu_state.quaternion = [float(x) for x in self._quat]
        self._state_pub.publish(st)

    # ---------- LiDAR ----------
    def _on_cloud(self, msg: PointCloud2) -> None:
        if self._pose is None:   # 地面の点を除くには base の高さと向きが要る（odometry はすぐ届く）
            return
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
        kept = np.ascontiguousarray(
            drop_ground_and_self_points(pts, self._pose[2], self._quat), dtype=np.float32)
        header = Header(frame_id=LIDAR_FRAME, stamp=msg.header.stamp)
        self._cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, kept))

    # ---------- 箱 ----------
    def _on_obstacles(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data).get("cmd")
        except (ValueError, AttributeError):
            cmd = None
        if cmd in ("front", "back"):
            self._place_box(cmd)
        elif cmd == "clear":
            self._clear_boxes()
        else:
            self.get_logger().warning(f"{OBSTACLES_TOPIC} の中身がわかりません: {msg.data!r}")

    def _place_box(self, where: str) -> None:
        if self._pose is None:
            self.get_logger().warning("Go2 の位置がまだ分からないので、箱を置けません")
            return
        if not self._create.service_is_ready():
            self.get_logger().warning(f"{CREATE_SERVICE} がまだ使えないので、箱を置けません")
            return
        x, y, _, _, _, yaw = self._pose[:6]
        name, box = self.boxes.place(where, x, y, yaw)
        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.sdf = box_sdf(name, box)
        req.entity_factory.pose.position.x = float(box.x)
        req.entity_factory.pose.position.y = float(box.y)
        req.entity_factory.pose.position.z = BOX_HEIGHT / 2.0

        def done(future) -> None:
            res = future.result()
            if res is not None and res.success:
                self.boxes.add(name, box)
                self.get_logger().info(f"箱を置きました（{where}）: {name} x={box.x:.2f} y={box.y:.2f}")
                self._publish_world()
            else:
                self.get_logger().warning(f"箱を置けませんでした（{where}）: {name}")

        self._create.call_async(req).add_done_callback(done)

    def _clear_boxes(self) -> None:
        for name in self.boxes.clear():
            req = DeleteEntity.Request()
            req.entity.name = name
            req.entity.type = Entity.MODEL

            def done(future, name=name) -> None:
                res = future.result()
                if res is None or not res.success:
                    self.get_logger().warning(f"Gazebo から箱を消せませんでした: {name}")

            self._remove.call_async(req).add_done_callback(done)
        self.get_logger().info("箱を全部どけました")
        self._publish_world()

    def _publish_world(self) -> None:
        x, y = (self._pose[0], self._pose[1]) if self._pose is not None else (math.inf, math.inf)
        self._world_pub.publish(String(data=json.dumps(self.boxes.world(x, y))))


def main() -> None:
    rclpy.init()
    node = GazeboBridge()
    node.get_logger().info(f"Gazebo の Go2 の橋渡しを起動しました（error_code {node.core.error_code}）")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
