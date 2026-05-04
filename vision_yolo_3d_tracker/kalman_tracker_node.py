"""Kalman multi-object tracker node.

Consumes `vision_msgs/Detection3DArray` and outputs a tracked
`vision_msgs/Detection3DArray` with persistent IDs.

Association: Hungarian assignment on Euclidean distance with gating.
Motion model: constant velocity in 3D.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml
from scipy.optimize import linear_sum_assignment

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import Header
from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)


class KalmanFilter3D:
    """3D constant-velocity Kalman filter tracking [x,y,z,vx,vy,vz]."""

    def __init__(
        self,
        initial_position: np.ndarray,
        dt: float,
        pos_noise: float,
        vel_noise: float,
        meas_noise: float,
    ) -> None:
        self.dt = float(dt)

        self.x = np.array(
            [
                float(initial_position[0]),
                float(initial_position[1]),
                float(initial_position[2]),
                0.0,
                0.0,
                0.0,
            ],
            dtype=float,
        )

        self.F = np.eye(6, dtype=float)
        self.F[0, 3] = self.dt
        self.F[1, 4] = self.dt
        self.F[2, 5] = self.dt

        self.H = np.zeros((3, 6), dtype=float)
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        self.P = np.eye(6, dtype=float) * 1.0

        self.Q = np.eye(6, dtype=float)
        self.Q[0:3, 0:3] *= float(pos_noise)
        self.Q[3:6, 3:6] *= float(vel_noise)

        self.R = np.eye(3, dtype=float) * float(meas_noise)

    def predict(self) -> np.ndarray:
        """Predict next state; return predicted position."""

        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.position()

    def update(self, measurement: np.ndarray) -> None:
        """Update state from a measured position."""

        z = measurement.astype(float)
        z_pred = self.H @ self.x
        y = z - z_pred
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + (K @ y)
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def position(self) -> np.ndarray:
        """Return current position [x, y, z]."""

        return self.x[0:3].copy()

    def velocity(self) -> np.ndarray:
        """Return current velocity [vx, vy, vz]."""

        return self.x[3:6].copy()


@dataclass
class Track:
    """A single tracked object."""

    track_id: int
    class_name: str
    kalman: KalmanFilter3D
    bbox_size: np.ndarray
    last_confidence: float

    hits: int = 1
    consecutive_misses: int = 0

    def predict(self) -> np.ndarray:
        return self.kalman.predict()

    def update(self, detection: Detection3D, confidence: float) -> None:
        pos = np.array(
            [
                detection.bbox.center.position.x,
                detection.bbox.center.position.y,
                detection.bbox.center.position.z,
            ],
            dtype=float,
        )
        self.kalman.update(pos)

        self.bbox_size = np.array(
            [
                detection.bbox.size.x,
                detection.bbox.size.y,
                detection.bbox.size.z,
            ],
            dtype=float,
        )

        self.last_confidence = float(confidence)
        self.hits += 1
        self.consecutive_misses = 0

    def mark_missed(self) -> None:
        self.consecutive_misses += 1

    def is_confirmed(self, min_hits: int) -> bool:
        return self.hits >= int(min_hits)

    def is_dead(self, max_age: int) -> bool:
        return self.consecutive_misses > int(max_age)


class KalmanTrackerNode(Node):
    """Tracker node: detections → tracked detections."""

    def __init__(self) -> None:
        super().__init__('kalman_tracker_node')

        self.declare_parameter('config_path', '')
        config_path = (
            self.get_parameter('config_path')
            .get_parameter_value()
            .string_value
        )

        self.config = self._load_config(config_path)

        self.max_distance = float(self.config.get('max_distance', 2.5))
        self.max_age = int(self.config.get('max_age', 60))
        self.min_hits = int(self.config.get('min_hits', 3))
        self.dt = float(self.config.get('dt', 0.1))
        self.pos_noise = float(self.config.get('position_noise', 0.01))
        self.vel_noise = float(self.config.get('velocity_noise', 0.01))
        self.meas_noise = float(self.config.get('measurement_noise', 0.1))

        self.enable_class_gating = bool(
            self.config.get('enable_class_gating', True)
        )

        self.input_topic = str(
            self.config.get(
                'input_topic',
                '/vision_yolo_3d_tracker/detections_3d',
            )
        )
        self.output_topic = str(
            self.config.get(
                'output_topic',
                '/vision_yolo_3d_tracker/tracked_objects',
            )
        )
        self.queue_size = int(self.config.get('queue_size', 10))

        self._next_track_id = 0
        self.tracks: Dict[int, Track] = {}
        self.frame_idx = 0

        self.sub = self.create_subscription(
            Detection3DArray,
            self.input_topic,
            self._detections_callback,
            self.queue_size,
        )
        self.pub = self.create_publisher(
            Detection3DArray,
            self.output_topic,
            self.queue_size,
        )

        self.get_logger().info('✓ vision_yolo_3d_tracker Kalman tracker ready')
        self.get_logger().info(f'Input:  {self.input_topic}')
        self.get_logger().info(f'Output: {self.output_topic}')

    def _default_config_path(self) -> str:
        share_dir = get_package_share_directory('vision_yolo_3d_tracker')
        return os.path.join(share_dir, 'config', 'tracker_params.yaml')

    def _load_config(self, config_path: str) -> dict:
        resolved_path = config_path or self._default_config_path()
        self.get_logger().info(f'Loading tracker config: {resolved_path}')

        with open(resolved_path, 'r', encoding='utf-8') as f:
            raw = yaml.safe_load(f)

        if not isinstance(raw, dict) or 'tracker' not in raw:
            raise RuntimeError(
                'Invalid tracker config: expected top-level "tracker" key'
            )

        return raw['tracker']

    def _detections_callback(self, msg: Detection3DArray) -> None:
        self.frame_idx += 1

        detections = list(msg.detections)
        if not detections and not self.tracks:
            return

        predictions = self._predict_tracks()
        track_ids = list(predictions.keys())

        cost = self._cost_matrix(track_ids, predictions, detections)
        matched, unmatched_tracks, unmatched_dets = self._assign(cost)

        for track_i, det_i in matched:
            tid = track_ids[track_i]
            det = detections[det_i]
            class_name, conf = self._extract_class_and_score(det)
            self.tracks[tid].update(det, confidence=conf)

        for det_i in unmatched_dets:
            det = detections[det_i]
            class_name, conf = self._extract_class_and_score(det)
            self._create_track(det, class_name, conf)

        for track_i in unmatched_tracks:
            tid = track_ids[track_i]
            self.tracks[tid].mark_missed()

        dead = [
            tid
            for tid, trk in self.tracks.items()
            if trk.is_dead(self.max_age)
        ]
        for tid in dead:
            del self.tracks[tid]

        self._publish(msg.header)

        if self.frame_idx % 30 == 0:
            confirmed = sum(
                1
                for t in self.tracks.values()
                if t.is_confirmed(self.min_hits)
            )
            self.get_logger().info(
                f'Frame {self.frame_idx}: {confirmed} confirmed, '
                f'{len(detections)} det, {len(self.tracks)} total'
            )

    def _predict_tracks(self) -> Dict[int, np.ndarray]:
        predictions: Dict[int, np.ndarray] = {}
        for tid, track in self.tracks.items():
            predictions[tid] = track.predict()
        return predictions

    def _cost_matrix(
        self,
        track_ids: List[int],
        predictions: Dict[int, np.ndarray],
        detections: List[Detection3D],
    ) -> np.ndarray:
        n_tracks = len(track_ids)
        n_dets = len(detections)

        high_cost = self.max_distance + 1.0
        cost = np.full((n_tracks, n_dets), high_cost, dtype=float)

        if n_tracks == 0 or n_dets == 0:
            return cost

        det_classes = []
        for det in detections:
            cls, _ = self._extract_class_and_score(det)
            det_classes.append(cls)

        for i, tid in enumerate(track_ids):
            pred = predictions[tid]
            trk = self.tracks[tid]

            for j, det in enumerate(detections):
                if (
                    self.enable_class_gating
                    and det_classes[j] != trk.class_name
                ):
                    continue

                det_pos = np.array(
                    [
                        det.bbox.center.position.x,
                        det.bbox.center.position.y,
                        det.bbox.center.position.z,
                    ],
                    dtype=float,
                )

                dist = float(np.linalg.norm(pred - det_pos))
                if dist <= self.max_distance:
                    cost[i, j] = dist

        return cost

    def _assign(
        self,
        cost: np.ndarray,
    ) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        n_tracks, n_dets = cost.shape if cost.size else (0, 0)

        if n_tracks == 0 or n_dets == 0:
            return [], list(range(n_tracks)), list(range(n_dets))

        row_idx, col_idx = linear_sum_assignment(cost)

        matched: List[Tuple[int, int]] = []
        unmatched_tracks = set(range(n_tracks))
        unmatched_dets = set(range(n_dets))

        for r, c in zip(row_idx, col_idx):
            if cost[r, c] <= self.max_distance:
                matched.append((int(r), int(c)))
            unmatched_tracks.discard(int(r))
            unmatched_dets.discard(int(c))

        return matched, sorted(unmatched_tracks), sorted(unmatched_dets)

    def _create_track(
        self,
        detection: Detection3D,
        class_name: str,
        confidence: float,
    ) -> None:
        pos = np.array(
            [
                detection.bbox.center.position.x,
                detection.bbox.center.position.y,
                detection.bbox.center.position.z,
            ],
            dtype=float,
        )

        kf = KalmanFilter3D(
            initial_position=pos,
            dt=self.dt,
            pos_noise=self.pos_noise,
            vel_noise=self.vel_noise,
            meas_noise=self.meas_noise,
        )

        size = np.array(
            [
                detection.bbox.size.x,
                detection.bbox.size.y,
                detection.bbox.size.z,
            ],
            dtype=float,
        )

        tid = self._next_track_id
        self._next_track_id += 1

        self.tracks[tid] = Track(
            track_id=tid,
            class_name=class_name,
            kalman=kf,
            bbox_size=size,
            last_confidence=float(confidence),
        )

    def _extract_class_and_score(
        self,
        detection: Detection3D,
    ) -> Tuple[str, float]:
        if detection.results:
            try:
                hyp = detection.results[0].hypothesis
                class_id = str(hyp.class_id)
                score = float(hyp.score)
                return class_id, score
            except Exception:
                pass

        if detection.id:
            return str(detection.id), 0.0

        return 'unknown', 0.0

    def _publish(self, header: Header) -> None:
        out = Detection3DArray()
        out.header = header

        for tid, track in self.tracks.items():
            if not track.is_confirmed(self.min_hits):
                continue

            det = Detection3D()
            det.header = header
            det.id = str(tid)

            pos = track.kalman.position()
            det.bbox.center.position = Point(
                x=float(pos[0]),
                y=float(pos[1]),
                z=float(pos[2]),
            )
            det.bbox.center.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=0.0,
                w=1.0,
            )

            det.bbox.size = Vector3(
                x=float(track.bbox_size[0]),
                y=float(track.bbox_size[1]),
                z=float(track.bbox_size[2]),
            )

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = track.class_name
            hyp.hypothesis.score = float(track.last_confidence)
            det.results.append(hyp)

            out.detections.append(det)

        self.pub.publish(out)


def main(args: Optional[list] = None) -> None:
    """Entry point."""

    rclpy.init(args=args)
    node = KalmanTrackerNode()
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
