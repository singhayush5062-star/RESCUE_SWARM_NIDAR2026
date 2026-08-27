"""Camera projection: world point -> image pixel, and the bounding box of a
world-space box.

Used two ways:
  * nidar_detection's dataset capture projects a survivor's known world
    position into the frame to produce an exact ground-truth label -- the
    simulator knows precisely where every actor is, so labels are computed,
    never hand-drawn or guessed.
  * Phase 3's geotag node runs the same geometry in reverse (pixel -> ray ->
    ground-plane intersection -> lat/lon). Keeping the forward direction
    here, tested, means that inverse has a verified reference to agree with.

Frames: `xyz` inputs are ENU metres in whatever frame the caller's transform
is expressed in (this project uses AS2's `earth`). The camera transform is
taken as the ROS *optical* frame convention -- +Z forward along the view
axis, +X right, +Y down -- which is what sensor_msgs/CameraInfo's K matrix
is defined against and what the sim publishes as `.../camera/optical_frame`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole intrinsics, matching sensor_msgs/CameraInfo's K."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_k(cls, k: Sequence[float], width: int, height: int) -> 'Intrinsics':
        """Build from CameraInfo.k, a row-major 3x3 [fx 0 cx; 0 fy cy; 0 0 1]."""
        return cls(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]),
                   width=int(width), height=int(height))

    @classmethod
    def from_hfov(cls, width: int, height: int, hfov_deg: float) -> 'Intrinsics':
        """Build from a horizontal field of view, for when CameraInfo is not
        available. Square pixels are assumed (fy == fx), which is what
        Gazebo's camera sensor produces."""
        f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        return cls(fx=f, fy=f, cx=width / 2.0, cy=height / 2.0,
                   width=int(width), height=int(height))


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Rotation matrix for a quaternion in ROS (x, y, z, w) order.

    Normalises first: a transform round-tripped through float32 message
    fields can arrive very slightly non-unit, and squaring that error into
    the rotation shows up as a projection that drifts with distance.
    """
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        raise ValueError('zero-length quaternion')
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


class CameraPose:
    """A camera's optical frame expressed in the world frame."""

    def __init__(self, position: Sequence[float], quaternion: Sequence[float]):
        self.position = np.asarray(position, dtype=float).reshape(3)
        self.rotation = quaternion_to_matrix(*quaternion)

    def world_to_camera(self, point_world: Sequence[float]) -> np.ndarray:
        """Express a world point in optical-frame coordinates."""
        delta = np.asarray(point_world, dtype=float).reshape(3) - self.position
        return self.rotation.T @ delta


def project_point(point_world: Sequence[float], pose: CameraPose,
                  intr: Intrinsics) -> Optional[Tuple[float, float]]:
    """Project a world point to a pixel, or None if it is behind the camera.

    Behind-camera points are rejected rather than projected: with z <= 0 the
    perspective divide flips their sign and places them at a perfectly
    plausible-looking pixel on the opposite side of the image, which as a
    training label is worse than no label at all.
    """
    p = pose.world_to_camera(point_world)
    if p[2] <= 1e-6:
        return None
    return (intr.fx * p[0] / p[2] + intr.cx,
            intr.fy * p[1] / p[2] + intr.cy)


def aabb_corners(center: Sequence[float], size: Sequence[float],
                 yaw_rad: float = 0.0) -> List[Tuple[float, float, float]]:
    """The 8 corners of a box of `size` (dx, dy, dz) centred at `center`,
    rotated by `yaw_rad` about the world Z axis."""
    cx, cy, cz = center
    hx, hy, hz = (s / 2.0 for s in size)
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    out = []
    for sx in (-hx, hx):
        for sy in (-hy, hy):
            for sz in (-hz, hz):
                out.append((cx + sx * c - sy * s, cy + sx * s + sy * c, cz + sz))
    return out


def project_box(corners: Iterable[Sequence[float]], pose: CameraPose,
                intr: Intrinsics, min_visible_px: float = 4.0
                ) -> Optional[Tuple[float, float, float, float]]:
    """Axis-aligned pixel bbox (x, y, w, h) enclosing a world-space box.

    Returns None when the box is behind the camera, entirely outside the
    frame, or smaller than `min_visible_px` on either side once clipped --
    a two-pixel sliver of a person at the frame edge is not something a
    detector can learn from, and labelling it teaches the model that
    near-invisible blobs are positives.
    """
    pts = [project_point(c, pose, intr) for c in corners]
    pts = [p for p in pts if p is not None]
    if not pts:
        return None

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    x1, x2 = min(xs), max(xs)
    y1, y2 = min(ys), max(ys)

    # Clip to the frame.
    cx1, cy1 = max(0.0, x1), max(0.0, y1)
    cx2, cy2 = min(float(intr.width), x2), min(float(intr.height), y2)
    if cx2 - cx1 < min_visible_px or cy2 - cy1 < min_visible_px:
        return None
    return (cx1, cy1, cx2 - cx1, cy2 - cy1)


def to_yolo_label(bbox: Sequence[float], intr: Intrinsics, class_id: int = 0) -> str:
    """Format a pixel bbox as a YOLO label line: normalised centre + size."""
    x, y, w, h = bbox
    return (f'{class_id} {(x + w / 2) / intr.width:.6f} {(y + h / 2) / intr.height:.6f} '
            f'{w / intr.width:.6f} {h / intr.height:.6f}')
