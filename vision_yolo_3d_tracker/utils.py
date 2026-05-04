"""Shared utility functions.

This package intentionally keeps the 2D→3D projection simple for now. The
projection and depth assumptions are the seam where future camera–LiDAR fusion
will be integrated.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera intrinsics."""

    fx: float
    fy: float
    cx: float
    cy: float


DEFAULT_CLASS_DIMS_M: Dict[str, Tuple[float, float, float]] = {
    # (length, width, height) in meters
    'car': (4.5, 2.0, 1.6),
    'truck': (6.0, 2.5, 2.0),
    'bus': (10.0, 2.5, 3.0),
    'person': (0.5, 0.5, 1.7),
    'pedestrian': (0.5, 0.5, 1.7),
    'bicycle': (1.8, 0.6, 1.0),
    'motorcycle': (2.2, 0.8, 1.3),
    'unknown': (2.0, 2.0, 1.8),
}


def bbox_xyxy_center(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> Tuple[float, float]:
    """Return the bbox center (u, v) in pixel coordinates."""

    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def project_pixel_to_3d(
    u: float,
    v: float,
    depth_m: float,
    intrinsics: CameraIntrinsics,
) -> Tuple[float, float, float]:
    """Project image pixel (u, v) to a 3D point using assumed depth.

    Notes:
    - This is a basic pinhole back-projection with a fixed depth.
    - The resulting coordinates are in the camera optical frame convention
      implied by the intrinsics; downstream TF alignment is future work.
    """

    x = (u - intrinsics.cx) * depth_m / intrinsics.fx
    y = (v - intrinsics.cy) * depth_m / intrinsics.fy
    z = depth_m
    return x, y, z


def class_dims_m(class_name: str) -> Tuple[float, float, float]:
    """Return default (L, W, H) dimensions for a class."""

    return DEFAULT_CLASS_DIMS_M.get(
        class_name,
        DEFAULT_CLASS_DIMS_M['unknown'],
    )
