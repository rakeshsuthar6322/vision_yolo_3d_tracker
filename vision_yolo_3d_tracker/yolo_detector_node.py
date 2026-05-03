"""YOLOv8 detector node.

Subscribes to camera images, runs YOLOv8 inference, and publishes:
- `vision_msgs/Detection3DArray` (approx 3D detections)
- optional labeled images

2D→3D uses a placeholder fixed depth and pinhole back-projection.
"""

from __future__ import annotations

import os
from typing import Dict, Optional, Tuple

import cv2
import yaml

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

from ament_index_python.packages import get_package_share_directory

from vision_yolo_3d_tracker.utils import (
    CameraIntrinsics,
    bbox_xyxy_center,
    class_dims_m,
    project_pixel_to_3d,
)


class YoloDetectorNode(Node):
    """YOLOv8 detector producing approximate 3D detections."""

    def __init__(self) -> None:
        super().__init__('yolo_detector_node')

        self.declare_parameter('config_path', '')
        config_path = (
            self.get_parameter('config_path')
            .get_parameter_value()
            .string_value
        )

        self.config = self._load_config(config_path)
        self.bridge = CvBridge()

        self._init_class_maps()
        self._init_intrinsics()
        self._init_model()
        self._init_io()

        self.frame_count = 0
        self.get_logger().info('✓ vision_yolo_3d_tracker YOLO detector ready')

    def _default_config_path(self) -> str:
        share_dir = get_package_share_directory('vision_yolo_3d_tracker')
        return os.path.join(share_dir, 'config', 'detector_params.yaml')

    def _load_config(self, config_path: str) -> dict:
        resolved_path = config_path or self._default_config_path()
        self.get_logger().info(f'Loading detector config: {resolved_path}')

        with open(resolved_path, 'r', encoding='utf-8') as f:
            raw = yaml.safe_load(f)

        if not isinstance(raw, dict) or 'detector' not in raw:
            raise RuntimeError(
                'Invalid detector config: expected top-level "detector" key'
            )

        return raw['detector']

    def _init_class_maps(self) -> None:
        classes_to_track: Dict[str, int] = self.config.get(
            'classes_to_track',
            {},
        )
        if not classes_to_track:
            raise RuntimeError(
                'detector.classes_to_track is empty; nothing to detect'
            )

        self.tracked_class_ids = set(int(v) for v in classes_to_track.values())

        # Prefer the user-provided names (e.g., "pedestrian" for COCO=0).
        self.class_id_to_name = {}
        for name, class_id in classes_to_track.items():
            self.class_id_to_name[int(class_id)] = str(name)

        self.colors_bgr: Dict[int, Tuple[int, int, int]] = self.config.get(
            'class_colors_bgr',
            {
                0: (0, 255, 0),
                1: (255, 0, 0),
                2: (0, 0, 255),
                3: (0, 255, 255),
                5: (255, 0, 255),
                7: (255, 255, 0),
            },
        )

    def _init_intrinsics(self) -> None:
        intr = self.config.get('camera_intrinsics', {})
        self.intrinsics = CameraIntrinsics(
            fx=float(intr.get('fx', 721.5)),
            fy=float(intr.get('fy', 721.5)),
            cx=float(intr.get('cx', 609.5)),
            cy=float(intr.get('cy', 172.9)),
        )
        self.default_depth_m = float(self.config.get('default_depth_m', 10.0))

    def _init_model(self) -> None:
        try:
            from ultralytics import YOLO  # type: ignore
        except Exception as exc:  # pragma: no cover
            raise RuntimeError(
                'Missing Python dependency "ultralytics". Install it with pip.'
            ) from exc

        model_path = self.config.get('model_path')
        model_type = self.config.get('model_type', 'yolov8m')
        weights = model_path or f'{model_type}.pt'

        self.device = self.config.get('device', 0)
        self.conf_threshold = float(
            self.config.get('confidence_threshold', 0.45)
        )
        self.iou_threshold = float(self.config.get('iou_threshold', 0.5))
        self.max_det = int(self.config.get('max_det', 300))

        # If CUDA_VISIBLE_DEVICES is set to empty string,
        # torch may behave oddly.
        if os.environ.get('CUDA_VISIBLE_DEVICES') == '':
            del os.environ['CUDA_VISIBLE_DEVICES']

        self.get_logger().info(f'Loading YOLO weights: {weights}')
        self.model = YOLO(weights)

    def _init_io(self) -> None:
        in_cfg = self.config.get('input', {})
        out_cfg = self.config.get('output', {})

        self.left_topic = str(
            in_cfg.get(
                'left_image_topic',
                '/kitti/image/color/left',
            )
        )
        self.right_topic = str(
            in_cfg.get(
                'right_image_topic',
                '/kitti/image/color/right',
            )
        )
        self.enable_right_camera = bool(
            in_cfg.get('enable_right_camera', True)
        )
        queue_size = int(in_cfg.get('queue_size', 10))

        self.detections_topic = str(
            out_cfg.get(
                'detections_topic',
                '/vision_yolo_3d_tracker/detections_3d',
            )
        )
        out_queue_size = int(out_cfg.get('queue_size', 10))

        self.publish_labeled_images = bool(
            self.config.get('publish_labeled_images', True)
        )
        self.left_labeled_topic = str(
            self.config.get(
                'left_labeled_topic',
                '/kitti/image/color/left/labeled',
            )
        )
        self.right_labeled_topic = str(
            self.config.get(
                'right_labeled_topic',
                '/kitti/image/color/right/labeled',
            )
        )

        self.sub_left = self.create_subscription(
            Image,
            self.left_topic,
            lambda msg: self._image_callback(msg, camera_side='left'),
            queue_size,
        )

        self.sub_right = None
        if self.enable_right_camera:
            self.sub_right = self.create_subscription(
                Image,
                self.right_topic,
                lambda msg: self._image_callback(msg, camera_side='right'),
                queue_size,
            )

        self.pub_detections = self.create_publisher(
            Detection3DArray,
            self.detections_topic,
            out_queue_size,
        )

        self.pub_labeled_left = None
        self.pub_labeled_right = None
        if self.publish_labeled_images:
            self.pub_labeled_left = self.create_publisher(
                Image,
                self.left_labeled_topic,
                10,
            )
            if self.enable_right_camera:
                self.pub_labeled_right = self.create_publisher(
                    Image,
                    self.right_labeled_topic,
                    10,
                )

        self.get_logger().info(f'Input left:  {self.left_topic}')
        self.get_logger().info(
            f'Input right: {self.right_topic} '
            f'(enabled={self.enable_right_camera})'
        )
        self.get_logger().info(f'Output detections: {self.detections_topic}')

    def _image_callback(self, msg: Image, camera_side: str) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(
                f'cv_bridge conversion failed: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        annotated = frame.copy() if self.publish_labeled_images else None

        try:
            results = self.model(
                frame,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                max_det=self.max_det,
                device=self.device,
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().error(
                f'YOLO inference failed: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        detections_msg = Detection3DArray()
        detections_msg.header = msg.header

        detection_count = 0
        for result in results:
            boxes = getattr(result, 'boxes', None)
            if boxes is None:
                continue

            for box in boxes:
                try:
                    class_id = int(box.cls[0])
                    if class_id not in self.tracked_class_ids:
                        continue

                    class_name = self.class_id_to_name.get(
                        class_id,
                        str(class_id),
                    )
                    confidence = float(box.conf[0])

                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()

                    if annotated is not None:
                        self._draw_box(
                            annotated,
                            x1,
                            y1,
                            x2,
                            y2,
                            class_id,
                            class_name,
                            confidence,
                        )

                    det3d = self._make_detection(
                        msg.header,
                        x1,
                        y1,
                        x2,
                        y2,
                        class_name,
                        confidence,
                    )
                    detections_msg.detections.append(det3d)
                    detection_count += 1

                except Exception as exc:
                    self.get_logger().error(
                        f'Failed to process a detection: {exc}',
                        throttle_duration_sec=5.0,
                    )

        self.pub_detections.publish(detections_msg)

        if annotated is not None:
            labeled_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            labeled_msg.header = msg.header
            if camera_side == 'left' and self.pub_labeled_left is not None:
                self.pub_labeled_left.publish(labeled_msg)
            elif camera_side == 'right' and self.pub_labeled_right is not None:
                self.pub_labeled_right.publish(labeled_msg)

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Frames: {self.frame_count}, '
                f'last frame detections: {detection_count}'
            )

    def _make_detection(
        self,
        header,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        class_name: str,
        confidence: float,
    ) -> Detection3D:
        det = Detection3D()
        det.header = header
        det.id = class_name

        u, v = bbox_xyxy_center(x1, y1, x2, y2)
        x, y, z = project_pixel_to_3d(
            u,
            v,
            self.default_depth_m,
            self.intrinsics,
        )

        det.bbox.center.position = Point(x=float(x), y=float(y), z=float(z))
        det.bbox.center.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        length_m, width_m, height_m = class_dims_m(class_name)
        det.bbox.size = Vector3(
            x=float(length_m),
            y=float(width_m),
            z=float(height_m),
        )

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = class_name
        hyp.hypothesis.score = float(confidence)
        det.results.append(hyp)

        return det

    def _draw_box(
        self,
        frame,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        class_id: int,
        class_name: str,
        confidence: float,
    ) -> None:
        color = self.colors_bgr.get(class_id, (128, 128, 128))
        p1 = (int(x1), int(y1))
        p2 = (int(x2), int(y2))
        cv2.rectangle(frame, p1, p2, color, 2)

        label = f'{class_name}: {confidence:.2f}'
        (tw, th), baseline = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            2,
        )
        y_text = max(0, int(y1) - th - baseline - 4)

        cv2.rectangle(
            frame,
            (int(x1), y_text),
            (int(x1) + tw, y_text + th + baseline + 4),
            color,
            -1,
        )
        cv2.putText(
            frame,
            label,
            (int(x1), y_text + th + 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )


def main(args: Optional[list] = None) -> None:
    """Entry point."""

    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':  # pragma: no cover
    main()
