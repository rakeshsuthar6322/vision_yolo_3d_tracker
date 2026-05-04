# vision_yolo_3d_tracker

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)
![License](https://img.shields.io/badge/License-Apache%202.0-green)
![Python](https://img.shields.io/badge/Python-3.10%2B-blue?logo=python)
![YOLOv8](https://img.shields.io/badge/Model-YOLOv8-purple)

<video src="files/demo.webm" width="800" autoplay loop muted playsinline></video>

---

## Project Overview

`vision_yolo_3d_tracker` is a ROS 2 package that implements a real-time, camera-based **3D multi-object detection and tracking** pipeline. It is designed for autonomous-driving research using the [KITTI Vision Benchmark Suite](https://www.cvlibs.net/datasets/kitti/) as its data source.

**Pipeline at a glance:**

```
KITTI Image Topics
        │
        ▼
┌─────────────────────┐
│  yolo_detector_node │  ← YOLOv8 inference + pinhole 2D→3D back-projection
└─────────┬───────────┘
          │  Detection3DArray
          ▼
┌──────────────────────┐
│ kalman_tracker_node  │  ← Kalman filter + Hungarian assignment (SORT-style)
└─────────┬────────────┘
          │  Detection3DArray (with persistent IDs)
          ▼
┌──────────────────────┐
│  visualization_node  │  ← RViz2 MarkerArray (bounding boxes, labels, centers)
└──────────────────────┘
```

| Component | Technology |
|-----------|------------|
| **Framework** | ROS 2 (Humble) |
| **Detector** | YOLOv8 (`ultralytics`) |
| **Tracker** | 3D Kalman filter + Hungarian algorithm (`scipy`) |
| **Dataset** | KITTI Vision Benchmark Suite |
| **Visualization** | RViz2 via `visualization_msgs/MarkerArray` |

---

## Prerequisites & Dependencies

### System Requirements

| Requirement | Version |
|-------------|---------|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Python | 3.10+ |
| CUDA (optional) | 11.8+ for GPU inference |

### ROS 2 Package Dependencies

| Package | Purpose |
|---------|---------|
| `rclpy` | ROS 2 Python client library |
| `sensor_msgs` | `Image` message type |
| `vision_msgs` | `Detection3DArray`, `Detection3D` message types |
| `visualization_msgs` | `MarkerArray` for RViz2 |
| `geometry_msgs` | `Point`, `Vector3`, `Quaternion` |
| `std_msgs` | `Header`, `ColorRGBA` |
| `cv_bridge` | OpenCV ↔ ROS image conversion |
| `rviz2` | 3D visualization |

> **Note:** `vision_msgs` must be built from source (included in this workspace as a submodule). See [Installation](#installation--building).

### Python Libraries

| Library | Install |
|---------|---------|
| `ultralytics` | `pip install ultralytics` |
| `numpy` | `pip install numpy` |
| `scipy` | `pip install scipy` |
| `opencv-python` | `pip install opencv-python` |
| `PyYAML` | `pip install pyyaml` |

### KITTI Data Publisher

This package consumes images from the [`ros2_kitti_publishers`](https://github.com/ROS2-Programme/ros2_kitti_publishers) package, which must be present in the same workspace and configured with a downloaded KITTI sequence.

---

## Installation & Building

### 1. Create a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone the Repository

```bash
git clone https://github.com/<your-username>/vision_yolo_3d_tracker.git
```

This repository should also include (or you should separately clone) the required packages into `src/`:

```bash
# vision_msgs (custom build required for Detection3DArray)
git clone https://github.com/ros-perception/vision_msgs.git

# KITTI data publisher
git clone https://github.com/ROS2-Programme/ros2_kitti_publishers.git
```

### 3. Install Python Dependencies

```bash
pip install ultralytics numpy scipy opencv-python pyyaml
```

### 4. Install ROS 2 Dependencies via rosdep

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select vision_yolo_3d_tracker vision_msgs ros2_kitti_publishers
source install/setup.bash
```

### Docker (Alternative)

A `Dockerfile` is provided for a self-contained environment:

```bash
# Build
docker build -t vision_yolo_3d_tracker:dev .

# Run (host networking simplifies ROS 2 discovery)
docker run --rm -it --net=host vision_yolo_3d_tracker:dev \
  ros2 launch vision_yolo_3d_tracker vision_yolo_3d_tracker.launch.py
```

---

## Usage

### Step 1 — Play KITTI Data

Configure `ros2_kitti_publishers` with the path to your downloaded KITTI sequence, then launch it:

```bash
ros2 launch ros2_kitti_publishers process_kitti.launch.py
```

This publishes raw images to `/kitti/image/color/left` and `/kitti/image/color/right`.

### Step 2 — Launch the Detection & Tracking Pipeline

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch vision_yolo_3d_tracker vision_yolo_3d_tracker.launch.py
```

RViz2 will open automatically with the pre-configured layout. To disable it:

```bash
ros2 launch vision_yolo_3d_tracker vision_yolo_3d_tracker.launch.py run_rviz:=false
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use `/clock` topic for simulated time |
| `log_level` | `info` | ROS 2 log verbosity (`debug`, `info`, `warn`, `error`) |
| `run_rviz` | `true` | Start RViz2 with the packaged configuration |

---

## Configuration

All parameters are defined in YAML files under `config/`. The launch file passes each file to its respective node via the `config_path` parameter. Edit these files before building (or after building if you used `--symlink-install`):

### `config/detector_params.yaml`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_type` | `yolov8m` | YOLOv8 variant (`n`, `s`, `m`, `l`, `x`) |
| `model_path` | `""` | Path to custom weights; auto-downloads if empty |
| `confidence_threshold` | `0.45` | Minimum detection confidence |
| `iou_threshold` | `0.5` | NMS IoU threshold |
| `device` | `0` | Inference device (`0`, `1`, … or `"cpu"`) |
| `max_det` | `300` | Maximum detections per frame |
| `default_depth_m` | `10.0` | Placeholder depth for 2D→3D projection (meters) |
| `publish_labeled_images` | `true` | Publish annotated images with bounding boxes |
| `camera_intrinsics.fx` | `721.5` | KITTI left-camera focal length (pixels) |
| `camera_intrinsics.fy` | `721.5` | KITTI left-camera focal length (pixels) |
| `camera_intrinsics.cx` | `609.5` | KITTI left-camera principal point x (pixels) |
| `camera_intrinsics.cy` | `172.9` | KITTI left-camera principal point y (pixels) |

### `config/tracker_params.yaml`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_distance` | `2.5` | Maximum 3D Euclidean gate distance (m) for association |
| `max_age` | `60` | Frames a track survives without a matching detection |
| `min_hits` | `3` | Frames before a new track is promoted to "confirmed" |
| `enable_class_gating` | `true` | Prevent cross-class track-to-detection assignments |
| `dt` | `0.1` | Kalman filter time step (seconds) |
| `position_noise` | `0.01` | Process noise for position states |
| `velocity_noise` | `0.01` | Process noise for velocity states |
| `measurement_noise` | `0.1` | Observation noise covariance |

### `config/viz_params.yaml`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `marker_lifetime_sec` | `0.25` | RViz2 marker lifetime before auto-deletion |

---

## ROS 2 API

### Subscribed Topics

| Topic | Type | Node | Description |
|-------|------|------|-------------|
| `/kitti/image/color/left` | `sensor_msgs/Image` | `yolo_detector_node` | Left monocular camera image |
| `/kitti/image/color/right` | `sensor_msgs/Image` | `yolo_detector_node` | Right camera image (optional, configurable) |
| `/vision_yolo_3d_tracker/detections_3d` | `vision_msgs/Detection3DArray` | `kalman_tracker_node` | Raw 3D detections from YOLO |
| `/vision_yolo_3d_tracker/tracked_objects` | `vision_msgs/Detection3DArray` | `visualization_node` | Confirmed tracked objects |

### Published Topics

| Topic | Type | Node | Description |
|-------|------|------|-------------|
| `/vision_yolo_3d_tracker/detections_3d` | `vision_msgs/Detection3DArray` | `yolo_detector_node` | Per-frame 3D detections (back-projected from 2D) |
| `/kitti/image/color/left/labeled` | `sensor_msgs/Image` | `yolo_detector_node` | Left image annotated with detection bounding boxes |
| `/kitti/image/color/right/labeled` | `sensor_msgs/Image` | `yolo_detector_node` | Right image annotated with detection bounding boxes |
| `/vision_yolo_3d_tracker/tracked_objects` | `vision_msgs/Detection3DArray` | `kalman_tracker_node` | Confirmed tracks with persistent integer IDs |
| `/vision_yolo_3d_tracker/visualization_markers` | `visualization_msgs/MarkerArray` | `visualization_node` | RViz2 markers: bounding boxes, track labels, centers |

### Node Parameters

| Node | Parameter | Type | Description |
|------|-----------|------|-------------|
| `yolo_detector_node` | `config_path` | `string` | Absolute path to `detector_params.yaml` |
| `yolo_detector_node` | `use_sim_time` | `bool` | Synchronize with `/clock` topic |
| `kalman_tracker_node` | `config_path` | `string` | Absolute path to `tracker_params.yaml` |
| `kalman_tracker_node` | `use_sim_time` | `bool` | Synchronize with `/clock` topic |
| `visualization_node` | `config_path` | `string` | Absolute path to `viz_params.yaml` |
| `visualization_node` | `use_sim_time` | `bool` | Synchronize with `/clock` topic |

---

## Architecture Notes

### 2D → 3D Projection

The detector performs a **pinhole back-projection** using KITTI camera intrinsics and a configurable fixed depth (`default_depth_m`). This is an approximation — true metric depth is not recovered from the monocular image. The `utils.py` module is intentionally structured as the seam where future camera–LiDAR fusion will be integrated.

### Tracking Algorithm

The tracker follows the SORT (Simple Online and Realtime Tracking) paradigm:

1. **Predict** — advance all existing Kalman filter states (constant-velocity model in 3D: `[x, y, z, vx, vy, vz]`).
2. **Associate** — build a Euclidean distance cost matrix and solve with the Hungarian algorithm (`scipy.optimize.linear_sum_assignment`). Class gating prevents cross-class assignments.
3. **Update** — update matched tracks; initialise new tracks for unmatched detections; delete tracks that exceed `max_age` consecutive misses.

A track is only published once it has accumulated `min_hits` consecutive detections, suppressing false-positive initialization.

### Tracked Classes (COCO IDs)

| Class Name | COCO ID |
|------------|---------|
| `pedestrian` | 0 |
| `bicycle` | 1 |
| `car` | 2 |
| `motorcycle` | 3 |
| `bus` | 5 |
| `truck` | 7 |

---

## TODOs / Future Work

- [ ] **LiDAR fusion** — replace the fixed-depth heuristic with actual depth from KITTI point clouds (PointPillars / sensor fusion).
- [ ] **True 3D bounding boxes** — estimate oriented 3D bounding boxes rather than using class-prior dimensions.
- [ ] **Stereo depth** — leverage the right camera image for disparity-based depth estimation.
- [ ] **Coordinate frame alignment** — add proper TF2 transforms between the camera optical frame and `base_link`.
- [ ] **Model fine-tuning** — fine-tune YOLOv8 on KITTI labels rather than using COCO-pretrained weights.
- [ ] **Integration tests** — add `pytest`-based ROS 2 integration tests with a bag-file fixture.
- [ ] **Docker image** — publish a pre-built Docker image to reduce environment setup friction.

---

## Acknowledgments

- **YOLOv8** by Ultralytics — [https://github.com/ultralytics/ultralytics](https://github.com/ultralytics/ultralytics)
  > Jocher, G. et al. (2023). *Ultralytics YOLOv8*. [https://github.com/ultralytics/ultralytics](https://github.com/ultralytics/ultralytics)

- **KITTI Vision Benchmark Suite** — [https://www.cvlibs.net/datasets/kitti/](https://www.cvlibs.net/datasets/kitti/)
  > Geiger, A., Lenz, P., Stiller, C., & Urtasun, R. (2013). *Vision meets robotics: The KITTI dataset*. The International Journal of Robotics Research, 32(11), 1231–1237.

- **SORT** — Simple Online and Realtime Tracking
  > Bewley, A., Ge, Z., Ott, L., Ramos, F., & Upcroft, B. (2016). *Simple online and realtime tracking*. ICIP 2016.

- **`ros2_kitti_publishers`** — [https://github.com/ROS2-Programme/ros2_kitti_publishers](https://github.com/ROS2-Programme/ros2_kitti_publishers)

---

## License

This package is released under the [Apache 2.0 License](https://www.apache.org/licenses/LICENSE-2.0).
