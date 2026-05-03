# Study README — `vision_yolo_3d_tracker`

This document is a “project memory + rebuild playbook” for the ROS 2 Humble package **vision_yolo_3d_tracker**.

It is written so you can:

- Understand what exists today (architecture, nodes, message flow, configs).
- Rebuild the same thing from scratch cleanly (as a senior software engineer).
- Evolve it toward the future target: **true camera–LiDAR depth fusion, calibration/extrinsics, and TF correctness across sensors**, plus experimenting with modern real‑time detectors (e.g., RT‑DETR / fast detection transformers).

---

## 0) What this package is (and is not)

**Today’s pipeline**

1. Subscribes to camera images.
2. Runs **YOLOv8** inference (Ultralytics).
3. Creates **approximate 3D detections** by pinhole back‑projection using a **fixed, placeholder depth**.
4. Tracks detections using a **3D constant‑velocity Kalman filter** with **Hungarian assignment**.
5. Publishes **RViz markers** for visualization.

**What it is not (yet)**

- Not true 3D from vision: the depth is currently a constant (`default_depth_m`).
- Not camera‑info driven: intrinsics are loaded from YAML (no `CameraInfo` subscription).
- Not a calibrated multi‑sensor fusion stack: no extrinsics estimation, no TF validation tools, no LiDAR association.

This is intentionally a clean, minimal rebuild meant to be used as a **standalone module**.

---

## 1) Repository tour (what each file/folder does)

- `vision_yolo_3d_tracker/`
  - `yolo_detector_node.py`: YOLOv8 inference + placeholder 2D→3D projection.
  - `kalman_tracker_node.py`: multi‑object tracking (Kalman + Hungarian).
  - `visualization_node.py`: converts tracked detections into RViz `MarkerArray`.
  - `utils.py`: projection + class dimension defaults (the “fusion seam”).
- `config/`
  - `detector_params.yaml`: model + topics + intrinsics + fixed depth.
  - `tracker_params.yaml`: association + filter noise + topics.
  - `viz_params.yaml`: marker publishing config.
- `launch/vision_yolo_3d_tracker.launch.py`: brings up detector + tracker + viz (+ RViz) with packaged configs.
- `rviz/tracking_visualization.rviz`: RViz view (Fixed Frame is `base_link`).
- `Dockerfile` + `docker/entrypoint.sh`: builds a colcon workspace into an image.
- `package.xml`, `setup.py`, `setup.cfg`: ROS2 ament_python packaging + entry points.
- `test/`: ament linters (`ament_flake8`, `ament_pep257`, etc.).

---

## 2) Runtime architecture (data flow)

```mermaid
flowchart LR
  L[Left Image\n(sensor_msgs/Image)] --> D[yolo_detector_node\nYOLOv8 + 2D→3D placeholder]
  R[Right Image (optional)\n(sensor_msgs/Image)] --> D
  D -->|Detection3DArray| T[kalman_tracker_node\nKalman + Hungarian]
  T -->|Detection3DArray| V[visualization_node\nMarkerArray]
  V -->|MarkerArray| RVIZ[RViz]
  D -->|optional labeled Image| RVIZ
```

### Default topics (as configured)

Inputs:

- `/kitti/image/color/left` (`sensor_msgs/Image`)
- `/kitti/image/color/right` (`sensor_msgs/Image`, optional)

Outputs:

- `/vision_yolo_3d_tracker/detections_3d` (`vision_msgs/Detection3DArray`)
- `/vision_yolo_3d_tracker/tracked_objects` (`vision_msgs/Detection3DArray`)
- `/vision_yolo_3d_tracker/visualization_markers` (`visualization_msgs/MarkerArray`)
- `/kitti/image/color/left/labeled` and `/kitti/image/color/right/labeled` (`sensor_msgs/Image`, optional)

---

## 3) Node-by-node deep dive

### 3.1) `yolo_detector_node`

**Role**: Convert `Image` → YOLO boxes → `Detection3DArray` (approx 3D).

**Config source**: `config/detector_params.yaml` (loaded via the ROS parameter `config_path`).

**Algorithm (current)**

- Convert ROS image → OpenCV BGR (`cv_bridge`).
- Run `ultralytics.YOLO(weights)` inference.
- For each kept class:
  - Compute bbox center pixel $(u,v)$.
  - Back‑project using pinhole intrinsics and **fixed** depth $z$:

$$
X = (u - c_x)\cdot z / f_x,\quad
Y = (v - c_y)\cdot z / f_y,\quad
Z = z
$$

- Publish `vision_msgs/Detection3DArray`.
- Optionally publish a labeled image with boxes.

**Message fields used**

- `Detection3D.header`: copied from incoming image header.
- `Detection3D.id`: set to `class_name` (note: tracker overwrites IDs downstream).
- `Detection3D.bbox.center.position`: (X,Y,Z) from back‑projection.
- `Detection3D.bbox.size`: class default dims from `utils.class_dims_m()`.
- `Detection3D.results[0].hypothesis.class_id`: `class_name`.
- `Detection3D.results[0].hypothesis.score`: YOLO confidence.

**Important limitation (multi‑camera)**

The node can subscribe to left and right images and publishes detections for both into **one** topic.

- If left and right images have different `header.frame_id` values (typical), then the output detections can arrive with **mixed frames**.
- Downstream tracking assumes all detections live in a single consistent coordinate frame.

For future correctness, either:

- Run **two detector nodes** (one per camera) and fuse later, or
- Publish detections on separate topics per camera, or
- Transform everything into a common frame before tracking.

---

### 3.2) `kalman_tracker_node`

**Role**: Maintain persistent track IDs and smooth 3D positions.

**Inputs/Outputs**

- Subscribes: `vision_msgs/Detection3DArray` (default `/vision_yolo_3d_tracker/detections_3d`)
- Publishes: `vision_msgs/Detection3DArray` (default `/vision_yolo_3d_tracker/tracked_objects`)

**Association**

- Predict each track forward (constant velocity in 3D).
- Build a cost matrix using Euclidean distance between predicted position and detection position.
- Optionally gate by class name (`enable_class_gating`).
- Solve assignment using Hungarian algorithm (`scipy.optimize.linear_sum_assignment`).
- Apply a distance gate (`max_distance`).

**Track lifecycle**

- New detections spawn new tracks.
- Tracks that miss detections increment `consecutive_misses`.
- Tracks are deleted after `max_age` misses.
- A track is published only after it is “confirmed” (`min_hits`).

**Output semantics**

- `Detection3D.id` becomes the numeric track ID as a string (`"0"`, `"1"`, ...).
- `results[0].hypothesis.class_id` and score are kept per track.

---

### 3.3) `visualization_node`

**Role**: Turn tracked detections into RViz markers.

- Input: `vision_msgs/Detection3DArray` (default `/vision_yolo_3d_tracker/tracked_objects`)
- Output: `visualization_msgs/MarkerArray` (default `/vision_yolo_3d_tracker/visualization_markers`)

Markers per track:

- `CUBE`: bounding box (namespace `bounding_boxes`)
- `TEXT_VIEW_FACING`: label (`ID` + class)
- `SPHERE`: center point

**Coloring** uses a fixed class→RGB mapping inside the node.

**RViz fixed frame note**

The packaged RViz config uses **Fixed Frame = `base_link`**.

That means RViz needs a valid TF chain from each marker’s `header.frame_id` to `base_link`. This package does not publish TF.

---

## 4) Configuration reference

### 4.1) Detector (`config/detector_params.yaml`)

Key knobs:

- `model_type`: e.g. `yolov8n/s/m/l/x` (used as `<model_type>.pt` if `model_path` empty)
- `model_path`: explicit weights path
- `confidence_threshold`, `iou_threshold`, `max_det`
- `device`: `0`, `1`, ... or `"cpu"`
- `classes_to_track`: name→COCO ID map (names become published `class_id` strings)
- `camera_intrinsics`: `{fx, fy, cx, cy}`
- `default_depth_m`: placeholder depth
- `publish_labeled_images`: enable drawing + publishing labeled images
- `input.*` / `output.*`: topics and queue sizes

### 4.2) Tracker (`config/tracker_params.yaml`)

- `max_distance`: distance gate for matching
- `max_age`: frames allowed without match
- `min_hits`: hits required to publish track
- `dt`: filter time step (assumed constant)
- `position_noise`, `velocity_noise`, `measurement_noise`: Kalman tuning
- `enable_class_gating`: only match detections of the same class
- `input_topic`, `output_topic`

### 4.3) Visualization (`config/viz_params.yaml`)

- `input_topic`
- `marker_topic`
- `marker_lifetime_sec`

---

## 5) How to build and run (native)

Inside a ROS 2 Humble workspace:

```bash
mkdir -p ~/ws/src
cd ~/ws/src
# clone repo here so that package.xml is at: ~/ws/src/vision_yolo_3d_tracker/package.xml

cd ~/ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Run:

```bash
ros2 launch vision_yolo_3d_tracker vision_yolo_3d_tracker.launch.py
```

Override configs (example):

```bash
ros2 run vision_yolo_3d_tracker yolo_detector_node --ros-args \
  -p config_path:=/absolute/path/to/detector_params.yaml
```

---

## 6) Docker (current image behavior)

The `Dockerfile`:

- Uses `ros:humble-ros-base`.
- Installs ROS message deps (`vision_msgs`, etc.) and `cv_bridge`, `rviz2`.
- Installs Python deps via pip: `ultralytics`, `pyyaml`, `scipy`.
- Copies the package into `/opt/ws/src/vision_yolo_3d_tracker`.
- Builds with `colcon build --symlink-install`.

Build:

```bash
docker build -t vision_yolo_3d_tracker:dev .
```

Run (host networking is simplest for ROS 2 discovery):

```bash
docker run --rm -it --net=host vision_yolo_3d_tracker:dev \
  ros2 launch vision_yolo_3d_tracker vision_yolo_3d_tracker.launch.py
```

### GPU note

If you want GPU inference later, plan for:

- NVIDIA Container Toolkit on host.
- A CUDA‑capable base image (or installing CUDA libs).
- Passing `--gpus all` and ensuring PyTorch/TensorRT builds match.

---

## 7) “Approved and clean build” checklist

For a ROS2 Python package, a practical “clean build” definition is:

- `colcon build --symlink-install` succeeds
- `colcon test` succeeds
- Lint passes: `ament_flake8`, `ament_pep257` (and optional copyright)

Typical commands:

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

If ament linters aren’t installed on a dev machine, install them via ROS apt packages for Humble.

---

## 8) Rebuild-from-scratch guide (senior-friendly)

This is the clean way to recreate this module in an isolated repo.

### Step 1 — Create a new ROS2 package

```bash
mkdir -p ~/ws/src
cd ~/ws/src
ros2 pkg create \
  --build-type ament_python \
  vision_yolo_3d_tracker
```

Then add:

- `package.xml` dependencies (match the current package)
- `setup.py` console scripts
- `config/`, `launch/`, `rviz/` packaged via `data_files`
- `test/` linters

### Step 2 — Implement minimal vertical slice

1. Implement a detector node that publishes a `Detection3DArray` (even with dummy depth).
2. Implement a tracker node that consumes/publishes `Detection3DArray`.
3. Implement a viz node that publishes `MarkerArray`.
4. Add launch file wiring them together.

This keeps the first milestone tiny but end‑to‑end runnable.

### Step 3 — Add Docker last (but early enough)

- Build from a known ROS2 base image.
- Run `rosdep install` during Docker build (or apt/pip equivalents).
- Build with colcon.
- Provide an entrypoint that sources `/opt/ros/$ROS_DISTRO/setup.bash` and the workspace overlay.

### Step 4 — Progressive git commits (recommended)

Commit in small, reviewable units:

1. `chore: init ament_python package skeleton`
2. `feat(detector): yolo node publishes Detection3DArray`
3. `feat(tracker): kalman + hungarian association`
4. `feat(viz): RViz markers for tracks`
5. `feat(launch): pipeline launch + packaged configs`
6. `build(docker): docker image builds colcon workspace`
7. `docs: add study_readme and usage`

This sequence gives you a clean bisectable history.

---

## 9) Future target: camera–LiDAR depth fusion + calibration + TF correctness

This is the core roadmap for turning the placeholder depth into a real system.

### 9.1) Make frames and TF non-negotiable

Goal: Every published detection has a correct `header.frame_id`, and there is a valid TF chain across sensors.

Recommended frame conventions (typical ROS practice):

- `map` (global)
- `odom` (local)
- `base_link` (vehicle)
- `lidar_link`
- `camera_link` and `camera_optical_frame`

Rules:

- Publish detections in **one frame** consistently (often `camera_optical_frame` or `base_link`).
- Track in the same frame you care about for downstream modules (commonly `base_link` or `map`).
- Validate with `tf2_tools` (`view_frames`) and RViz.

### 9.2) Replace hard-coded intrinsics with `CameraInfo`

Today intrinsics come from YAML. For a production pipeline:

- Subscribe to `sensor_msgs/CameraInfo`.
- Use `image_geometry.PinholeCameraModel` (or your own small model) for projection.
- Keep YAML as a fallback only.

### 9.3) LiDAR depth-from-ROI fusion (pragmatic first method)

Inputs you will likely need:

- Image (`sensor_msgs/Image`)
- Point cloud (`sensor_msgs/PointCloud2`)
- TF extrinsics between LiDAR and camera frames
- Camera intrinsics (`CameraInfo`)

Core idea:

1. Transform LiDAR points into the camera optical frame.
2. Project 3D points to pixel coordinates.
3. For each 2D detection bbox, collect projected points inside the bbox.
4. Estimate depth using a robust statistic (median / trimmed mean).
5. Produce a fused 3D position (and optionally size/orientation priors).

Design tips:

- Use gating on depth range and minimum points count.
- Cache transforms and intrinsics.
- Consider downsampling the cloud (voxel grid) for real-time.

### 9.4) Calibration / extrinsics workflow

You’ll want an explicit calibration story:

- Offline calibration tool produces extrinsics.
- Extrinsics are published as `static_transform_publisher` or a dedicated calibration node.
- Runtime sanity checks assert TF availability and reasonable transforms.

### 9.5) Tracking in a stable frame

Once depth is real, you can move tracking to `base_link` or `map`:

- Transform fused detections into the tracking frame.
- Track with motion constraints consistent with the platform (vehicle kinematics if needed).

---

## 10) Future target: experimenting with fast detection transformers (RT‑DETR / similar)

Best practice is to avoid hard‑wiring one model into the node.

Recommended approach:

- Create a detector “backend” interface (YOLO, RT‑DETR, etc.).
- Switch with config (`detector_backend: yolo | rtdetr | onnx | tensorrt`).
- Prefer ONNX/TensorRT for deployment performance.
- Keep postprocessing + class mapping consistent across backends.

---

## 11) Modularity: separate ROS package repo vs Docker image repo

You said you want to commit the ROS package in an isolated repo and build the Docker image separately.

Two clean options:

### Option A (simple, common): single repo, separate build artifacts

- Keep Dockerfile in this repo.
- CI builds and publishes images tagged by git SHA / tag.

### Option B (more modular): two repos

- Repo 1: `vision_yolo_3d_tracker` (ROS package only)
- Repo 2: `vision_yolo_3d_tracker_docker` (Docker build that pulls a released tag or tarball)

If you choose Option B, ensure the Docker repo builds from a **versioned release** (git tag) so you can reproduce images.

---

## 12) Explicitly ignoring `object_3d_tracker`

This package is already a clean rebuild and does not depend on an old `object_3d_tracker` module.

If you still have that legacy package elsewhere in your workspace, treat it as out-of-scope and remove it from the build workspace when you want a truly minimal, isolated build.
