# ARM64 Linux runs in Colima's Linux VM on an Apple Silicon Mac.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ur ros-jazzy-moveit ros-jazzy-rviz2 \
    python3-colcon-common-extensions \
    xvfb x11vnc novnc websockify openbox x11-utils x11-xserver-utils \
    libgl1-mesa-dri mesa-utils tini \
    && rm -rf /var/lib/apt/lists/*

ENV DISPLAY=:99 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

COPY docker/ /opt/demo/
WORKDIR /opt/ur5e_ws
ENTRYPOINT ["/usr/bin/tini", "--", "bash", "/opt/demo/start.sh"]
