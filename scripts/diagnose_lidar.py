"""LiDAR・姿勢を受信するだけの診断。動作指令は送らない。"""
import os
from pathlib import Path
import time
import numpy as np

from voice_ltl.local import environment

os.environ.update(environment("robot"))
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from unitree_go.msg import SportModeState
from voice_ltl.sensing import Sensing
from go2_nl_control.perception import PerceptionConfig


def main():
    rclpy.init()
    node = rclpy.create_node("go2_lidar_readonly_diagnostic")
    data = {}

    def cloud(msg):
        p = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        data.update(points=np.column_stack([p[k] for k in ("x", "y", "z")]),
                    frame=msg.header.frame_id)

    def state(msg):
        data.update(code=int(msg.error_code), height=float(msg.body_height),
                    rpy=np.asarray(msg.imu_state.rpy, dtype=float))

    node.create_subscription(PointCloud2, "/utlidar/cloud", cloud, qos_profile_sensor_data)
    node.create_subscription(SportModeState, "/sportmodestate", state, qos_profile_sensor_data)
    try:
        end = time.monotonic() + 3
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if "points" not in data:
        raise SystemExit("点群未受信")
    Path("out").mkdir(exist_ok=True)
    np.savez_compressed("out/lidar_diagnostic.npz", **data)
    pts = data.pop("points")
    print("受信情報:", data, "点数:", len(pts))
    print("XYZ分位点:", np.quantile(pts, [0.05, 0.5, 0.95], axis=0))
    for name, cfg in (
        ("現設定", PerceptionConfig()),
        ("取り付けモデル", PerceptionConfig(lidar_x=.28945, lidar_z=-.046825, lidar_pitch=2.8782)),
        ("変換なし", PerceptionConfig(lidar_x=0, lidar_z=0)),
    ):
        sensing = Sensing(cfg)
        print(name, sensing.update(pts, time.time()))
        body = sensing._front._to_body(pts)
        near = body[np.linalg.norm(body[:, :2], axis=1) < 2]
        if len(near):
            print("近傍z分位点:", np.quantile(near[:, 2], [.05, .25, .5, .75, .95]))


if __name__ == "__main__":
    main()
