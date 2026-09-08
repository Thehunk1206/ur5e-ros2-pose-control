#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash

colcon build --base-paths /workspace/src --symlink-install \
  --event-handlers console_direct+
source install/setup.bash

# A Linux display is streamed to the Mac browser using noVNC.
Xvfb :99 -screen 0 1280x800x24 +extension GLX +render -noreset &
for attempt in {1..100}; do
  if xdpyinfo >/dev/null 2>&1; then break; fi
  sleep 0.1
done
xdpyinfo >/dev/null
openbox --sm-disable >/tmp/openbox.log 2>&1 &
x11vnc -display :99 -localhost -nopw -forever -shared \
  -rfbport 5900 >/tmp/x11vnc.log 2>&1 &
websockify --web=/usr/share/novnc 6080 localhost:5900 >/tmp/novnc.log 2>&1 &

echo 'RViz: http://localhost:6080/vnc.html?autoconnect=true&resize=scale'
ros2 launch ur5e_pose_control demo.launch.py &
ros_pid=$!
trap 'kill -INT "$ros_pid" 2>/dev/null; wait "$ros_pid"; exit 0' TERM INT
wait "$ros_pid"
