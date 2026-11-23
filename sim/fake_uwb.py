"""にせ UWB リモコン。実機と同じ /uwbstate（unitree_go/UwbState）を出す。

- リモコンの位置はオドメトリ座標（/sportmodestate の position と同じ座標）で持つ。
- /sportmodestate の Go2 の位置と向きから、距離 distance_est と相対方向 orientation_est
  （Go2 の正面が 0、左が正 [rad]）を求め、10 Hz で /uwbstate に出す。
- /sim/uwb（std_msgs/String、{"x": 2.0, "y": 0.5}）でリモコンを動かす。
  位置がまだ決まっていなければ、最初に届いた Go2 の位置の 2 m 前に置く。
- 実機の測位の揺れを真似て、距離と方向に小さな雑音を足す（UwbTag の noise_*）。

follow.py はシミュレーションのとき、このノードを同じプロセスの中で動かす。単独でも動く:
  python3 -m sim.fake_uwb
"""

import json
import math
import random
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from unitree_go.msg import SportModeState, UwbState

UWB_TOPIC = "/uwbstate"
SIM_UWB_TOPIC = "/sim/uwb"
STATE_TOPIC = "/sportmodestate"
UWB_HZ = 10.0
START_AHEAD = 2.0   # 最初に置く位置：Go2 の前 [m]
MAX_RANGE = 30.0    # これより遠い位置には置かない [m]

Pose = Tuple[float, float, float]


class UwbTag:
    """リモコンの位置と、Go2 から見た距離・方向の計算（ROS に依存しない）。"""

    def __init__(self, noise_distance: float = 0.03, noise_bearing: float = math.radians(2.0),
                 rng: Optional[random.Random] = None):
        self.position: Optional[Tuple[float, float]] = None
        self.noise_distance = noise_distance
        self.noise_bearing = noise_bearing
        self.rng = rng or random.Random()
        self._lock = threading.Lock()

    def place(self, x: float, y: float) -> Tuple[float, float]:
        x, y = float(x), float(y)
        if not (math.isfinite(x) and math.isfinite(y)) or math.hypot(x, y) > MAX_RANGE:
            raise ValueError(f"リモコンの位置は原点から {MAX_RANGE:g} m 以内の数で指定してください")
        with self._lock:
            self.position = (x, y)
        return self.position

    def place_ahead(self, pose: Pose, distance: float = START_AHEAD) -> None:
        x, y, yaw = pose
        with self._lock:
            if self.position is None:
                self.position = (x + distance * math.cos(yaw), y + distance * math.sin(yaw))

    def measure(self, pose: Pose) -> Optional[Tuple[float, float]]:
        """Go2 の位置 pose から見た (距離 [m], 方向 [rad]、左が正)。位置が未定なら None。"""
        with self._lock:
            position = self.position
        if position is None:
            return None
        x, y, yaw = pose
        dx, dy = position[0] - x, position[1] - y
        distance = math.hypot(dx, dy) + self.rng.gauss(0.0, self.noise_distance)
        bearing = math.atan2(dy, dx) - yaw + self.rng.gauss(0.0, self.noise_bearing)
        return max(0.0, distance), math.atan2(math.sin(bearing), math.cos(bearing))


class FakeUwb(Node):
    def __init__(self, tag: Optional[UwbTag] = None):
        super().__init__("fake_uwb")
        self.tag = tag or UwbTag()
        self.pose: Optional[Pose] = None
        self._pub = self.create_publisher(UwbState, UWB_TOPIC, qos_profile_sensor_data)
        self.create_subscription(SportModeState, STATE_TOPIC, self._on_state, qos_profile_sensor_data)
        self.create_subscription(String, SIM_UWB_TOPIC, self._on_place, 10)
        self.create_timer(1.0 / UWB_HZ, self._on_timer)

    def _on_state(self, msg: SportModeState) -> None:
        pose = (float(msg.position[0]), float(msg.position[1]), float(msg.imu_state.rpy[2]))
        if all(map(math.isfinite, pose)):
            self.pose = pose
            self.tag.place_ahead(pose)

    def _on_place(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.tag.place(data["x"], data["y"])
        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().warning(f"{SIM_UWB_TOPIC} の中身がわかりません: {msg.data!r}（{exc}）")

    def _on_timer(self) -> None:
        pose = self.pose
        if pose is None:
            return
        measured = self.tag.measure(pose)
        if measured is None:
            return
        msg = UwbState()
        msg.distance_est, msg.orientation_est = float(measured[0]), float(measured[1])
        msg.channel = 0
        msg.error_state = 0
        msg.enabled_from_app = 1
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = FakeUwb()
    node.get_logger().info(f"にせ UWB リモコンを起動しました（{SIM_UWB_TOPIC} で位置を変えられます）")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
