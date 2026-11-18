"""Gazebo（Harmonic）の Go2 の橋渡し（gazebo コンテナの ROS 2 Jazzy で動くノード）。

アプリから見ると実機の Go2 と同じ窓口になる。中身は bridge_core.BridgeCore。

- /api/sport/request（unitree_api/Request）を受け、unitree_guide controller の /control_input
  （control_input_msgs/Inputs）に変えて送る。
- Gazebo の本当の位置（/model/robot/odometry、nav_msgs/Odometry）から /sportmodestate
  （unitree_go/SportModeState）を作る。position・imu_state.rpy・error_code などを入れる。
  立っているつもりで転んだら（|roll|・|pitch| > 1.0 rad か胴の高さ < 0.15 m）、error_code を 1001
  （伏せ扱い）にして止める（BridgeCore.observe、Ruling R38）。起き上がらせないので、Gazebo を起動し直す。
- Gazebo の LiDAR の点群（/gazebo/lidar/points）から自分の体の点と地面の点（世界の z < 0.05 m）を除き、
  /utlidar/cloud（frame utlidar_lidar）に出し直す。
- /sim/obstacles（std_msgs/String、{"cmd": "front" | "back" | "clear"}）で Gazebo に箱を出し入れし
  （/world/empty/create・/world/empty/remove）、/sim/world（{"boxes": [[x, y, size], ...], "collided": bool}）を出す。
  OBJECTS_SOURCE=detector のときだけ、箱の見た目を段ボールにする（bridge_core.box_sdf、open-objects spec §11.13）。
- 起動のときに OBJECTS_SOURCE（logical か detector、既定 logical）と GO2_LAYOUT（既定 household）を 1 回読み、
  配置ファイル gazebo/layouts/<GO2_LAYOUT>.json（voice_ltl/layout.py）を読む。知らない値・知らない配置・
  GZ_SIM_RESOURCE_PATH にない model:// のフォルダがあれば、わけを 1 行ログに書き、何も置かずに 0 でない値で終わる。
  配置の物を Gazebo に 1 回だけ置く（bridge_core.spawn_fields。すでにあれば、あるものとして扱う）。
- logical camera（/gazebo/logical_camera、ros_gz_interfaces/LogicalCameraImage）が見た model のうち配置の id だけを、
  世界（= odom）の足跡の中心に直して覚える（bridge_core.logical_rows。ground_plane・robot・sim_box_<n> は除く）。
  - 正解：配置の大きさのまま、2 Hz で /gazebo/objects_truth に出す（どちらのときも。"layout" に配置の名前）。
  - logical のときだけ：胴が止まっている間（角速度が軸ごとに 0.1 rad/s 以下・速さ 0.05 m/s 以下が 0.3 秒続いた後。
    bridge_core.StillGate）に撮った LiDAR の点で、種類ごとの半径（objects.measure_radius）で底面の大きさを測り
    （画面のボタンで置いた箱の点は除く）、2 Hz で /go2/objects に出す。detector のときは検出器が /go2/objects を出す。
  どちらも std_msgs/String の JSON（voice_ltl/objects.py の形、"source": "logical"）。時刻（stamp・seen）は
  実際の時間（time.time()）。10 秒ごとに /go2/objects の送り手の数を確かめ、1 つでなければエラーを書く
  （bridge_core.objects_publisher_note）。

時計は 2 つ。窓口の時間（error_code が変わるまで、Move の 1 秒の期限）はアプリと同じ実際の時間、
controller へ command を送る間隔はシミュレーションの時間（/clock）で数える（bridge_core.py の説明）。
controller の状態の切り替えは、controller が /rosout に出す "Switched from <a> to <b>" をすべて
BridgeCore.controller_switched に渡して知らせる。橋渡しは確かめた状態からだけ次の command を送り
（2026-11-11：見積もりで送った 4 が伏せの controller に残り、伏せたまま動かなかった）、controller が
脱力・伏せの間は立ち・歩きの error_code を返さない。Move の速度は、歩くに入った知らせの 0.5 秒後
（シミュレーションの時間）から入れる（Ruling R22）。
位置はいつも Gazebo の本当の位置なので、距離は Gazebo の速さ（RTF）によらない。
Humble 側の相手のノードの名前は分からない（Gid の長さの違い）ので使わない。

go2_gazebo.launch.py が起動する。自分で起動するなら（gazebo コンテナの /work で）:
    PYTHONPATH=/work:/work/nl_control:$PYTHONPATH python3 -m gazebo.gazebo_bridge
"""

import json
import math
import sys
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
import rclpy.logging
from control_input_msgs.msg import Inputs
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity, LogicalCameraImage
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from unitree_api.msg import Request
from unitree_go.msg import SportModeState

from go2_nl_control.actions import MOVE
from gazebo.bridge_core import (PUBLISHER_NOTE_S, TRUTH_TOPIC, BoxWorld, BridgeCore, ControllerInput,
                                ObjectMemory, StillGate, api_name, box_sdf, bridge_settings, compose,
                                drop_box_points, drop_ground_and_self_points, footprint_size,
                                lidar_points_world, log_time, logical_rows, objects_publisher_note,
                                objects_topics, parse_switch, spawn_fields)
from sim.sim_core import BOX_HEIGHT
from voice_ltl.layout import Layout
from voice_ltl.objects import OBJECTS_HZ, OBJECTS_TOPIC, measure_radius

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
LOGICAL_CAMERA_TOPIC = "/gazebo/logical_camera"
ROSOUT_TOPIC = "/rosout"           # controller の "Switched from <a> to <b>"（bridge_core.parse_switch）
CREATE_SERVICE = f"/world/{WORLD_NAME}/create"
REMOVE_SERVICE = f"/world/{WORLD_NAME}/remove"

TICK_HZ = 20.0     # BridgeCore.step を回す周期（実際の時間）
WORLD_HZ = 10.0
SPAWN_HZ = 1.0     # 物を置けるようになったか（create のサービスと Go2 の位置）を見る周期

# (x, y, z, roll, pitch, yaw, vx, vy, vz, yaw_speed)。速度は世界の座標
Pose = Tuple[float, float, float, float, float, float, float, float, float, float]


def _stamp(stamp) -> float:
    """builtin_interfaces/Time → 秒（header.stamp はシミュレーションの時間、/rosout の Log.stamp は time.time()）。"""
    return stamp.sec + stamp.nanosec * 1e-9


def _rpy(q) -> Tuple[float, float, float]:
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


NODE_NAME = "go2_gazebo_bridge"


class GazeboBridge(Node):
    def __init__(self, source: str, lay: Layout):
        super().__init__(NODE_NAME,
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.core = BridgeCore(*self._now(), reports_switches=True)
        self.boxes = BoxWorld()
        self.source = source           # OBJECTS_SOURCE（起動のときに 1 回読んだ）
        self.layout = lay              # GO2_LAYOUT の配置（置く物と正解の大きさ）
        topics = objects_topics(source)
        # 正解（配置の大きさ）。logical camera が見た配置の物
        sizes = {e.id: e.world_size() for e in lay.entries}
        self.truth = ObjectMemory(sizes=sizes)
        # /go2/objects（logical のときだけ）：LiDAR で測った大きさ。測るまでは [L, L] と配置の大きさの軸ごとの
        # 大きい方（pre-final review F2）。detector のときは検出器が出すので持たない
        self.memory = ObjectMemory(seed_sizes=sizes) if OBJECTS_TOPIC in topics else None
        self.still = StillGate()       # 胴が止まっている間の点群だけで物の大きさを測る
        self._pose: Optional[Pose] = None
        self._quat = (1.0, 0.0, 0.0, 0.0)   # (w, x, y, z)
        self._last_note = ""
        self._last_code = self.core.error_code

        self._control_pub = self.create_publisher(Inputs, CONTROL_TOPIC, 10)
        # 実機と同じく、状態と点群は best effort で出す
        self._state_pub = self.create_publisher(SportModeState, STATE_TOPIC, qos_profile_sensor_data)
        self._cloud_pub = self.create_publisher(PointCloud2, LIDAR_TOPIC, qos_profile_sensor_data)
        self._world_pub = self.create_publisher(String, WORLD_TOPIC, 10)
        self._truth_pub = self.create_publisher(String, TRUTH_TOPIC, 10)
        self._objects_pub = self.create_publisher(String, OBJECTS_TOPIC, 10) if self.memory is not None else None
        self.create_subscription(Request, REQUEST_TOPIC, self._on_request, qos_profile_sensor_data)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, RAW_CLOUD_TOPIC, self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(LogicalCameraImage, LOGICAL_CAMERA_TOPIC, self._on_logical_camera,
                                 qos_profile_sensor_data)
        self.create_subscription(String, OBSTACLES_TOPIC, self._on_obstacles, 10)
        # controller のログ。過去の分は要らないので volatile で受ける
        self.create_subscription(Log, ROSOUT_TOPIC, self._on_log, QoSProfile(
            depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
        self._create = self.create_client(SpawnEntity, CREATE_SERVICE)
        self._remove = self.create_client(DeleteEntity, REMOVE_SERVICE)
        wall = Clock(clock_type=ClockType.STEADY_TIME)   # Gazebo が止まっても回る
        self.create_timer(1.0 / TICK_HZ, self._on_tick, clock=wall)
        self.create_timer(1.0 / WORLD_HZ, self._publish_world, clock=wall)
        self.create_timer(1.0 / OBJECTS_HZ, self._publish_objects, clock=wall)
        self.create_timer(PUBLISHER_NOTE_S, self._check_publishers, clock=wall)
        self._spawn_timer = self.create_timer(1.0 / SPAWN_HZ, self._spawn_objects, clock=wall)

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
        """controller の状態の切り替えを、1 行も落とさず橋渡しの中身に知らせる。時刻は受けたときの sim
        （切り替えより少し後なので、次の command を送るのは見積もりより遅い側になる）と、ログを出した時刻
        （Log.stamp。use_sim_time でもこの機械の時計なので、time.time() との差で now の時計に直す）。
        橋渡しの中身は後者で、知らせが遅れたときに、送り直した command がどの状態に届いたかを知る。"""
        if msg.name == self.get_logger().name:   # 自分のログ（ノートなど）は見ない
            return
        switch = parse_switch(msg.msg)
        if switch is None:
            return
        now, sim = self._now()
        logged = log_time(now, time.time(), _stamp(msg.stamp))
        late = "" if logged is None or now - logged < 0.5 else f"（{now - logged:.1f} 秒遅れて届きました）"
        self.get_logger().info(f"controller の切り替え: {switch[0]} → {switch[1]}{late}")
        self.core.controller_switched(switch[1], sim, from_state=switch[0], now=now, logged_at=logged)
        self._log_notes()

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
        self.core.observe(roll, pitch, p.z, *self._now())   # 転んだら 1001（Ruling R38）
        self.still.observe(_stamp(msg.header.stamp), (v.x, v.y, v.z), (w.x, w.y, w.z))

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
        # 動いていると点群と位置の時刻のずれで点が広がるので、胴が止まっている間に撮った点群だけで大きさを測る。
        # detector のときは測らない（/go2/objects は検出器が出し、正解は配置の大きさ）
        if self.memory is not None and self.still.allows(_stamp(msg.header.stamp)):
            self._measure_sizes(kept)

    # ---------- 物（配置ファイルの物） ----------
    def _measure_sizes(self, kept: np.ndarray) -> None:
        """覚えている物ごとに、足跡の中心から種類ごとの半径（objects.measure_radius）の中の LiDAR の点で底面の
        大きさを測って入れる。画面のボタンで置いた箱の点は先に除く（箱が物のそばにあると、物の大きさが
        膨らんだまま戻らないので）。"""
        objects = self.memory.objects()
        if not objects or len(kept) == 0:
            return
        world = drop_box_points(lidar_points_world(kept, self._pose[0:3], self._quat),
                                self.boxes.sizing_boxes(time.time()))
        for o in objects:
            size = footprint_size(world, o.x, o.y, radius=measure_radius(o.label))
            if size is not None:
                self.memory.set_size(o.id, size)

    def _on_logical_camera(self, msg: LogicalCameraImage) -> None:
        """msg.pose はカメラの世界の位置と向き、model[i].pose はカメラから見た model の原点。"""
        p, q = msg.pose.position, msg.pose.orientation
        camera_pos, camera_quat = (p.x, p.y, p.z), (q.w, q.x, q.y, q.z)
        models = []
        for model in msg.model:
            if self.layout.find(model.name) is None:   # ground_plane・robot・画面のボタンで置いた sim_box_<n> など
                continue
            r = model.pose.position
            x, y, _ = compose(camera_pos, camera_quat, (r.x, r.y, r.z))
            models.append((model.name, x, y))
        rows = logical_rows(models, self.layout)       # 原点 → 足跡の中心（C3）
        now = time.time()
        self.truth.update(rows, now)
        if self.memory is not None:
            self.memory.update(rows, now)

    def _publish_objects(self) -> None:
        now = time.time()
        self._truth_pub.publish(String(data=json.dumps(self.truth.message(now, layout=self.layout.name))))
        if self._objects_pub is not None:
            self._objects_pub.publish(String(data=json.dumps(self.memory.message(now))))

    def _check_publishers(self) -> None:
        """/go2/objects の送り手が 1 つでなければ、エラーを書く（止めはしない。spec §11.3・§11.9）。"""
        note = objects_publisher_note(self.source, self.count_publishers(OBJECTS_TOPIC))
        if note is not None:
            self.get_logger().error(note)

    def _spawn_objects(self) -> None:
        """create のサービスが使えて Go2 の位置が分かったら、配置の物を 1 回だけ置く（モデルの原点の位置と向き）。"""
        if self._pose is None or not self._create.service_is_ready():
            return
        self._spawn_timer.cancel()
        for e in self.layout.entries:
            sdf, x, y, z, yaw = spawn_fields(e)
            req = SpawnEntity.Request()
            req.entity_factory.name = e.id
            req.entity_factory.sdf = sdf
            req.entity_factory.pose.position.x = float(x)
            req.entity_factory.pose.position.y = float(y)
            req.entity_factory.pose.position.z = float(z)
            req.entity_factory.pose.orientation.w = math.cos(yaw / 2.0)
            req.entity_factory.pose.orientation.z = math.sin(yaw / 2.0)

            def done(future, e=e) -> None:
                res = future.result()
                if res is not None and res.success:
                    self.get_logger().info(f"物を置きました: {e.id}（{e.model}）x={e.x:.2f} y={e.y:.2f} "
                                           f"向き {e.yaw_deg}°")
                else:   # 橋渡しだけを起動し直したときなど。同じ名前の物はもうある
                    self.get_logger().info(f"すでにあります（{e.id}）")

            self._create.call_async(req).add_done_callback(done)

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
        # detector のときは段ボールの見た目（D9：箱として見られうる。ぶつかる形は同じ）
        req.entity_factory.sdf = box_sdf(name, box, cardboard=self.source == "detector")
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
                self.boxes.failed(name)
                self.get_logger().warning(f"箱を置けませんでした（{where}）: {name}")

        self._create.call_async(req).add_done_callback(done)

    def _clear_boxes(self) -> None:
        for name in self.boxes.clear(time.time()):   # 消えるまでの点群は、しばらく大きさを測る点から除く
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
    try:
        source, lay = bridge_settings()
    except ValueError as e:     # 何も置かずに止まる（launch のログに 1 行。Review Focus 5）
        rclpy.logging.get_logger(NODE_NAME).error(str(e))
        sys.exit(1)
    rclpy.init()
    node = GazeboBridge(source, lay)
    node.get_logger().info(f"Gazebo の Go2 の橋渡しを起動しました（error_code {node.core.error_code}、"
                           f"OBJECTS_SOURCE={source}、GO2_LAYOUT={lay.name}：物 {len(lay.entries)} 個）")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
