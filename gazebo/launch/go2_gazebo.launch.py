"""Gazebo Harmonic に Go2 を出し、unitree_guide controller と橋渡し（gazebo.gazebo_bridge）を起動する。

docs/reference/gazebo_feasibility_jazzy.md の「構成 B」：Gazebo の画面あり、センサーは LiDAR だけ
（カメラなし）、rviz2 なし、位置は OdometryPublisher。upstream の gz_quadruped_playground の
gazebo.launch.py は、カメラ 2 台と rviz2 で CPU が足りず RTF が 0.13 になる（NO-GO）ので使わない。

使い方（gazebo コンテナの中）: ros2 launch /work/gazebo/launch/go2_gazebo.launch.py
画面: http://localhost:6080/vnc.html?autoconnect=1
"""

import os
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
XACRO = ROOT / "gazebo" / "go2_sensors.urdf.xacro"
WORLD = "empty"                                            # worlds/default.sdf の world の名前
MODEL = "robot"                                            # gazebo_bridge.py と同じ
# 画面のカメラ：Go2 の右前 (1.3, -1.1, 0.7) から、Go2 が出る場所の少し前 (0.3, 0.1, 0.15) を見る
# （既定の (-6, 0, 6) からでは Go2 が小さすぎる）。画面が立ち上がるまで待ってから送る。
CAMERA = ("pose: {position: {x: 1.3, y: -1.1, z: 0.7}, "
          "orientation: {w: 0.4181, x: -0.1526, y: 0.0715, z: 0.8926}}")
SET_CAMERA = ("for i in $(seq 60); do gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera "
              "--reptype gz.msgs.Boolean --timeout 2000 --req '" + CAMERA + "' "
              "2>/dev/null | grep -q true && exit 0; sleep 2; done; "
              "echo 'go2_gazebo.launch.py: could not set the GUI camera' >&2")


def generate_launch_description() -> LaunchDescription:
    playground = get_package_share_directory("gz_quadruped_playground")
    world_sdf = os.path.join(playground, "worlds", "default.sdf")
    robot_description = xacro.process_file(str(XACRO)).toxml()
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

    # Gazebo ↔ ROS：時計、LiDAR、本当の位置、箱を出し入れするサービス（カメラは無い）
    gz_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/gazebo/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            f"/model/{MODEL}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            f"/world/{WORLD}/create@ros_gz_interfaces/srv/SpawnEntity",
            f"/world/{WORLD}/remove@ros_gz_interfaces/srv/DeleteEntity",
        ],
        parameters=[sim_time])

    # 実機と同じ窓口（/api/sport/request・/sportmodestate・/utlidar/cloud）と /sim/*
    bridge = ExecuteProcess(
        cmd=["python3", "-m", "gazebo.gazebo_bridge"], cwd=str(ROOT), output="screen",
        additional_env={"PYTHONPATH": os.pathsep.join(
            [str(ROOT), str(ROOT / "nl_control"), os.environ.get("PYTHONPATH", "")])})

    camera = ExecuteProcess(cmd=["bash", "-c", SET_CAMERA], output="log", condition=IfCondition(gui))

    return LaunchDescription([
        DeclareLaunchArgument("height", default_value="0.5", description="Go2 を出す高さ [m]"),
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo の画面を出すか"),
        gazebo, robot_state_publisher, spawn, controllers, gz_bridge, bridge, camera,
    ])
