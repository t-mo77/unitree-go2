"""Gazebo Harmonic に Go2 を出し、unitree_guide controller と橋渡し（gazebo.gazebo_bridge）を起動する。

docs/reference/gazebo_feasibility_jazzy.md の「構成 B」：Gazebo の画面あり、センサーは LiDAR と
logical camera（画像は出さず、見えている model の名前と位置だけ。画像を出すカメラは無い）、rviz2 なし、
位置は OdometryPublisher。upstream の gz_quadruped_playground の gazebo.launch.py は、カメラ 2 台と
rviz2 で CPU が足りず RTF が 0.13 になる（NO-GO）ので使わない。
物は橋渡しが起動時に置く（GO2_LAYOUT の配置 gazebo/layouts/<名前>.json。既定 household は 6 個、default は box_1・chair_1）。

OBJECTS_SOURCE（spec §11.3）は起動のときに 1 回読む（bridge_core.objects_source。ほかの値なら起動しない）。
detector のときだけ、xacro に camera:=true を渡して描くカメラ front_camera（640×480、2 Hz）を付け、その絵と
CameraInfo を別の parameter_bridge のノード front_camera_bridge で ROS に渡す（gazebo/launch_config.py）。
logical（既定）のときのロボットの記述と主の ros_gz_bridge の引数は 0f6e06a と同じ（spec §2 の 7）。

使い方（gazebo コンテナの中）: ros2 launch /work/gazebo/launch/go2_gazebo.launch.py
画面: http://localhost:6080/vnc.html?autoconnect=1
"""

import os
import sys
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[2]                 # このリポジトリ（コンテナでは /work）
# gazebo.bridge_core（OBJECTS_SOURCE）と gazebo.launch_config を読むため（bridge_core は go2_nl_control も使う）
for _path in (ROOT / "nl_control", ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from gazebo import bridge_core  # noqa: E402
from gazebo.launch_config import (CAMERA_BRIDGE_NAME, GZ_BRIDGE_ARGS, MODEL,  # noqa: E402
                                  camera_bridge_args, xacro_mappings)

XACRO = ROOT / "gazebo" / "go2_sensors.urdf.xacro"
# 画面のカメラ：Go2 の右前 (1.3, -1.1, 0.7) から、Go2 が出る場所の少し前 (0.3, 0.1, 0.15) を見る
# （既定の (-6, 0, 6) からでは Go2 が小さすぎる）。画面が立ち上がるまで待ってから送る。
CAMERA = ("pose: {position: {x: 1.3, y: -1.1, z: 0.7}, "
          "orientation: {w: 0.4181, x: -0.1526, y: 0.0715, z: 0.8926}}")
SET_CAMERA = ("for i in $(seq 60); do gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera "
              "--reptype gz.msgs.Boolean --timeout 2000 --req '" + CAMERA + "' "
              "2>/dev/null | grep -q true && exit 0; sleep 2; done; "
              "echo 'go2_gazebo.launch.py: could not set the GUI camera' >&2")


def generate_launch_description() -> LaunchDescription:
    source = bridge_core.objects_source()        # ほかの値なら ValueError で、何も作る前に止まる
    playground = get_package_share_directory("gz_quadruped_playground")
    world_sdf = os.path.join(playground, "worlds", "default.sdf")
    # logical なら mappings は {} で、出力は 0f6e06a と同じ。detector なら camera:=true で front_camera が付く
    robot_description = xacro.process_file(str(XACRO), mappings=xacro_mappings(source)).toxml()
    sim_time = {"use_sim_time": True}

    # Gazebo（サーバーと画面）。-r：すぐ動かし始める。gui:=false なら -s（サーバーだけ）
    gui = LaunchConfiguration("gui")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments=[("gz_args", [
            "-r -v 2 ", PythonExpression(["'' if '", gui, "' == 'true' else '-s '"]), world_sdf])])

    # gz_quadruped_hardware と unitree_guide controller が robot_description を使う
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "publish_frequency": 20.0,
                     "use_tf_static": True, "ignore_timestamp": True}, sim_time])

    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", MODEL, "-z", LaunchConfiguration("height")])

    # joint_state_broadcaster → imu_sensor_broadcaster → unitree_guide_controller の順に起動する
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(playground, "launch", "unitree_guide.launch.py")))

    # Gazebo ↔ ROS：時計、LiDAR、logical camera（画像は無く、見えている model の名前と位置だけ）、
    # 本当の位置、箱と物を出し入れするサービス（GZ_BRIDGE_ARGS。0f6e06a と同じ）
    gz_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=list(GZ_BRIDGE_ARGS),
        parameters=[sim_time])

    # detector のときだけ：描くカメラの絵と CameraInfo を別のノードで（主の bridge の引数は変えない）
    camera_args = camera_bridge_args(source)
    front_camera_bridge = [] if camera_args is None else [Node(
        package="ros_gz_bridge", executable="parameter_bridge", name=CAMERA_BRIDGE_NAME, output="screen",
        arguments=list(camera_args), parameters=[sim_time])]

    # 実機と同じ窓口（/api/sport/request・/sportmodestate・/utlidar/cloud）、/go2/objects と /sim/*
    bridge = ExecuteProcess(
        cmd=["python3", "-m", "gazebo.gazebo_bridge"], cwd=str(ROOT), output="screen",
        additional_env={"PYTHONPATH": os.pathsep.join(
            [str(ROOT), str(ROOT / "nl_control"), os.environ.get("PYTHONPATH", "")])})

    camera = ExecuteProcess(cmd=["bash", "-c", SET_CAMERA], output="log", condition=IfCondition(gui))

    return LaunchDescription([
        DeclareLaunchArgument("height", default_value="0.5", description="Go2 を出す高さ [m]"),
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo の画面を出すか"),
        gazebo, robot_state_publisher, spawn, controllers, gz_bridge, *front_camera_bridge, bridge, camera,
    ])
