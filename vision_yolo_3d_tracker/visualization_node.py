"""RViz visualization node.

Subscribes to tracked `vision_msgs/Detection3DArray` and publishes markers.
"""

from __future__ import annotations

import os
from typing import Dict, Optional, Tuple

import yaml

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import ColorRGBA, Header
from vision_msgs.msg import Detection3D, Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


class VisualizationNode(Node):
    """Create RViz markers for tracked objects."""

    COLORS: Dict[str, Tuple[float, float, float]] = {
        'person': (0.0, 1.0, 0.0),
        'pedestrian': (0.0, 1.0, 0.0),
        'bicycle': (0.0, 0.0, 1.0),
        'car': (1.0, 0.0, 0.0),
        'motorcycle': (1.0, 1.0, 0.0),
        'truck': (1.0, 0.0, 1.0),
        'bus': (0.0, 1.0, 1.0),
        'unknown': (0.5, 0.5, 0.5),
    }

    DEFAULT_DIMS_M: Dict[str, Tuple[float, float, float]] = {
        'car': (4.5, 2.0, 1.6),
        'person': (0.5, 0.5, 1.7),
        'pedestrian': (0.5, 0.5, 1.7),
        'bicycle': (1.8, 0.6, 1.0),
        'motorcycle': (2.2, 0.8, 1.3),
        'truck': (6.0, 2.5, 2.0),
        'bus': (10.0, 2.5, 3.0),
        'unknown': (2.0, 2.0, 1.8),
    }

    def __init__(self) -> None:
        super().__init__('visualization_node')

        self.declare_parameter('config_path', '')
        config_path = (
            self.get_parameter('config_path')
            .get_parameter_value()
            .string_value
        )

        self.config = self._load_config(config_path)

        self.input_topic = str(
            self.config.get(
                'input_topic',
                '/vision_yolo_3d_tracker/tracked_objects',
            )
        )
        self.marker_topic = str(
            self.config.get(
                'marker_topic',
                '/vision_yolo_3d_tracker/visualization_markers',
            )
        )
        self.queue_size = int(self.config.get('queue_size', 10))

        lifetime_sec = float(self.config.get('marker_lifetime_sec', 0.25))
        self.lifetime_sec = int(lifetime_sec)
        self.lifetime_nanosec = int((lifetime_sec - self.lifetime_sec) * 1e9)

        self.sub = self.create_subscription(
            Detection3DArray,
            self.input_topic,
            self._callback,
            self.queue_size,
        )
        self.pub = self.create_publisher(MarkerArray, self.marker_topic, 100)

        self.get_logger().info('✓ vision_yolo_3d_tracker visualization ready')
        self.get_logger().info(f'Input:  {self.input_topic}')
        self.get_logger().info(f'Output: {self.marker_topic}')

    def _default_config_path(self) -> str:
        share_dir = get_package_share_directory('vision_yolo_3d_tracker')
        return os.path.join(share_dir, 'config', 'viz_params.yaml')

    def _load_config(self, config_path: str) -> dict:
        resolved_path = config_path or self._default_config_path()
        self.get_logger().info(f'Loading viz config: {resolved_path}')

        with open(resolved_path, 'r', encoding='utf-8') as f:
            raw = yaml.safe_load(f)

        if not isinstance(raw, dict) or 'viz' not in raw:
            raise RuntimeError(
                'Invalid viz config: expected top-level "viz" key'
            )

        return raw['viz']

    def _callback(self, msg: Detection3DArray) -> None:
        marker_array = MarkerArray()

        for det in msg.detections:
            class_name = self._extract_class(det)
            color = self.COLORS.get(class_name, self.COLORS['unknown'])

            base_id = self._marker_base_id(det.id)
            markers = self._make_markers(
                base_id,
                det,
                class_name,
                color,
                msg.header,
            )
            marker_array.markers.extend(markers)

        self.pub.publish(marker_array)

    def _extract_class(self, det: Detection3D) -> str:
        if det.results:
            try:
                return str(det.results[0].hypothesis.class_id)
            except Exception:
                return 'unknown'
        return 'unknown'

    def _marker_base_id(self, track_id: str) -> int:
        try:
            return int(track_id)
        except Exception:
            return abs(hash(track_id)) % 100000

    def _make_markers(
        self,
        base_id: int,
        det: Detection3D,
        class_name: str,
        color: Tuple[float, float, float],
        header: Header,
    ) -> list:
        markers = [
            self._box_marker(base_id * 10 + 0, det, class_name, color, header),
            self._label_marker(base_id * 10 + 1, det, class_name, header),
            self._center_marker(base_id * 10 + 2, det, color, header),
        ]
        return markers

    def _box_marker(
        self,
        marker_id: int,
        det: Detection3D,
        class_name: str,
        color: Tuple[float, float, float],
        header: Header,
    ) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = 'bounding_boxes'
        marker.id = int(marker_id)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = det.bbox.center

        sx, sy, sz = (
            float(det.bbox.size.x),
            float(det.bbox.size.y),
            float(det.bbox.size.z),
        )
        if sx <= 0.0 or sy <= 0.0 or sz <= 0.0:
            sx, sy, sz = self.DEFAULT_DIMS_M.get(
                class_name,
                self.DEFAULT_DIMS_M['unknown'],
            )

        marker.scale.x = float(sx)
        marker.scale.y = float(sy)
        marker.scale.z = float(sz)

        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.6)
        marker.lifetime.sec = self.lifetime_sec
        marker.lifetime.nanosec = self.lifetime_nanosec

        return marker

    def _label_marker(
        self,
        marker_id: int,
        det: Detection3D,
        class_name: str,
        header: Header,
    ) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = 'track_labels'
        marker.id = int(marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position = Point(
            x=det.bbox.center.position.x,
            y=det.bbox.center.position.y,
            z=det.bbox.center.position.z + 2.0,
        )
        marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        marker.scale.z = 0.5
        marker.text = f'ID: {det.id}\n{class_name}'
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.lifetime.sec = self.lifetime_sec
        marker.lifetime.nanosec = self.lifetime_nanosec

        return marker

    def _center_marker(
        self,
        marker_id: int,
        det: Detection3D,
        color: Tuple[float, float, float],
        header: Header,
    ) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = 'track_centers'
        marker.id = int(marker_id)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose = det.bbox.center

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.8)
        marker.lifetime.sec = self.lifetime_sec
        marker.lifetime.nanosec = self.lifetime_nanosec

        return marker


def main(args: Optional[list] = None) -> None:
    """Entry point."""

    rclpy.init(args=args)
    node = VisualizationNode()
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
