#!/bin/bash
# Entry point of the `gazebo` service (installed as /usr/local/bin/gazebo_entry.sh).
# 1. virtual screen :1   2. window manager + VNC server   3. noVNC on port 6080
# 4. source ROS 2 / unitree_ros2 / the Gazebo workspace, then run the given command.
# Screen in the browser: http://localhost:6080/vnc.html

# a container that was restarted keeps the old lock of display :1
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1

Xvfb :1 -screen 0 1280x800x24 >/tmp/xvfb.log 2>&1 &
for _ in $(seq 100); do
    [ -S /tmp/.X11-unix/X1 ] && break
    sleep 0.1
done
if [ ! -S /tmp/.X11-unix/X1 ]; then
    echo "gazebo_entry.sh: Xvfb did not start (see /tmp/xvfb.log)" >&2
    exit 1
fi

fluxbox >/tmp/fluxbox.log 2>&1 &
x11vnc -display :1 -forever -nopw -shared -rfbport 5900 >/tmp/x11vnc.log 2>&1 &
websockify --web /usr/share/novnc 6080 localhost:5900 >/tmp/websockify.log 2>&1 &

source ~/unitree_ros2/setup.sh
source ~/gz_ws/install/setup.bash
exec "$@"
