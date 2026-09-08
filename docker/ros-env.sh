#!/usr/bin/env bash
# docker exec does not inherit the startup script's sourced ROS environment.
set -e
source /opt/ros/jazzy/setup.bash
source /opt/ur5e_ws/install/setup.bash
exec "$@"
