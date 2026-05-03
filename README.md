# vision_yolo_3d_tracker

ROS 2 (Humble) `ament_python` package that:

1. Runs YOLOv8 on camera images (Ultralytics)
2. Produces approximate 3D detections via fixed-depth pinhole back-projection
3. Tracks detections with a constant-velocity 3D Kalman filter + Hungarian assignment
4. Publishes RViz markers for visualization

This is a clean rebuild intended to be used as a standalone module.

## Nodes

- `yolo_detector_node`
- `kalman_tracker_node`
- `visualization_node`

## Topics (defaults)

**Inputs**

- `/kitti/image/color/left` (`sensor_msgs/Image`)
- `/kitti/image/color/right` (`sensor_msgs/Image`, optional)

**Outputs**

- `/vision_yolo_3d_tracker/detections_3d` (`vision_msgs/Detection3DArray`)
- `/vision_yolo_3d_tracker/tracked_objects` (`vision_msgs/Detection3DArray`)
- `/vision_yolo_3d_tracker/visualization_markers` (`visualization_msgs/MarkerArray`)
- `/kitti/image/color/left/labeled` (`sensor_msgs/Image`, optional)
- `/kitti/image/color/right/labeled` (`sensor_msgs/Image`, optional)

## Configuration

Edit the YAML files in `config/`:

- `config/detector_params.yaml`
- `config/tracker_params.yaml`
- `config/viz_params.yaml`

Each node takes a `config_path` parameter (the launch file sets packaged defaults).

## Build (native)

Inside a ROS 2 workspace:

```bash
mkdir -p ~/ws/src
cd ~/ws/src
# clone this repository so that package.xml is at repo root
# git clone <your-remote-url>

cd ~/ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch vision_yolo_3d_tracker vision_yolo_3d_tracker.launch.py
```

## Docker

Build:

```bash
docker build -t vision_yolo_3d_tracker:dev .
```

Run (host networking makes ROS 2 discovery easiest):

```bash
docker run --rm -it --net=host vision_yolo_3d_tracker:dev \
  ros2 launch vision_yolo_3d_tracker vision_yolo_3d_tracker.launch.py
```
