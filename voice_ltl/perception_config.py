"""シミュレーションと、実機Go2のLiDAR取り付けを分ける。"""
import os
from go2_nl_control.perception import PerceptionConfig


def perception_config(env=None):
    env = os.environ if env is None else env
    if env.get("GO2_LIDAR_PROFILE") == "go2":
        return PerceptionConfig(lidar_x=0.28945, lidar_z=-0.046825,
                                lidar_pitch=2.8782, remove_ground=True)
    return PerceptionConfig()
