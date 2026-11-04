#!/bin/bash
# Installed in the gazebo image as ~/unitree_ros2/setup.sh. Use: source ~/unitree_ros2/setup.sh
# Same as unitree_setup.sh (the Humble image), but for ROS 2 Jazzy.
# GO2_IFACE: the LAN port wired to the Go2 (e.g. enp3s0). Empty = leave it to the container network.
source /opt/ros/jazzy/setup.bash
source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -n "${GO2_IFACE:-}" ]; then
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="'"$GO2_IFACE"'" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
fi
