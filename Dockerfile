FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

# System dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-vision-msgs \
    ros-humble-visualization-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-cv-bridge \
    ros-humble-rviz2 \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Python runtime dependencies
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir \
        ultralytics \
        pyyaml \
        scipy

# Build a colcon workspace containing this package
WORKDIR /opt/ws
RUN mkdir -p /opt/ws/src
COPY . /opt/ws/src/vision_yolo_3d_tracker

RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
