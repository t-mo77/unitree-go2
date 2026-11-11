"""Gazebo の launch（gazebo/launch/go2_gazebo.launch.py）の、OBJECTS_SOURCE で決まる部分（open-objects spec §2 の 7、
§11.3、§11.4。design-detector.md §4.1）。ROS を import しないので、app のイメージのテストでも読める。

- 主の ros_gz_bridge の引数 GZ_BRIDGE_ARGS は 0f6e06a の launch と同じ 6 つ（バイト単位）。logical のときは
  ロボットの記述も 0f6e06a と同じなので、今の RTF と e2e の記録がそのまま使える。
- detector のときだけ、xacro に camera:=true を渡して描くカメラ front_camera を付け（gazebo/go2_sensors.urdf.xacro）、
  別の parameter_bridge のノード front_camera_bridge（CAMERA_BRIDGE_NAME）で、その絵と CameraInfo を ROS に渡す
  （CAMERA_BRIDGE_ARGS）。別のノードにするのは、主の bridge の引数を変えずに、M1（spec §11.4）でカメラの bridge
  だけを変えられるようにするため。
- OBJECTS_SOURCE のほかの値は ValueError（launch は起動しない。値を確かめるのは bridge_core.objects_topics）。
"""

from __future__ import annotations

from gazebo.bridge_core import objects_topics

WORLD = "empty"        # worlds/default.sdf の world の名前
MODEL = "robot"        # gazebo_bridge.py と同じ

# Gazebo ↔ ROS：時計、LiDAR、logical camera（画像は無く、見えている model の名前と位置だけ）、
# 本当の位置、箱と物を出し入れするサービス
GZ_BRIDGE_ARGS: tuple[str, ...] = (
    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    "/gazebo/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
    "/gazebo/logical_camera@ros_gz_interfaces/msg/LogicalCameraImage[gz.msgs.LogicalCameraImage",
    f"/model/{MODEL}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    f"/world/{WORLD}/create@ros_gz_interfaces/srv/SpawnEntity",
    f"/world/{WORLD}/remove@ros_gz_interfaces/srv/DeleteEntity",
)

# detector のときだけ：描くカメラの絵（rgb8 640×480、frame front_camera）と CameraInfo（spec §11.3 の表）
CAMERA_BRIDGE_NAME = "front_camera_bridge"
CAMERA_BRIDGE_ARGS = ("/gazebo/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                      "/gazebo/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo")


def xacro_mappings(source: str) -> dict[str, str]:
    """xacro.process_file に渡す mappings：logical → {}、detector → {"camera": "true"}。ほかの値なら ValueError。
    xacro は渡した dict に xacro:arg の既定値を書き足すので、呼ぶたびに新しい dict を返す。"""
    objects_topics(source)
    return {"camera": "true"} if source == "detector" else {}


def camera_bridge_args(source: str) -> tuple[str, ...] | None:
    """front_camera_bridge の引数（detector のときだけ。logical なら None でノードを作らない）。ほかの値なら ValueError。"""
    objects_topics(source)
    return CAMERA_BRIDGE_ARGS if source == "detector" else None
