"""Pixel -> ground point. Pure geometry, no ROS, no simulator.

Turning a bounding box into a GPS coordinate is four steps, and only the
middle two are interesting:

  1. take the bbox centre pixel (u, v)
  2. back-project it through the camera intrinsics into a ray in the camera's
     own optical frame                                    -- pixel_to_ray
  3. rotate that ray into the world frame using the camera's actual
     orientation                                          -- rotate
  4. intersect it with the ground plane                    -- ray_ground_hit

Step 3 deliberately takes a rotation obtained from TF rather than a mount
angle from a config file. The camera hangs off a gimbal (world_swarm.yaml
sets its pitch to 1.5707963 rad for nadir), and a config constant describes
where the gimbal was *told* to point, not where it is. TF carries the
composed drone attitude and gimbal state as actually reported. Verified live
on this sim -- `tf2_echo earth <optical_frame>` returns a rotation whose
third column is (0.001, 0.001, -1.000), i.e. the optical +Z axis pointing
along earth -Z: straight down, as intended.

Frame conventions, both REP-103 / OpenCV standard and both worth stating
because mixing them up produces plausible, wrong coordinates:
  * camera OPTICAL frame: +x right across the image, +y DOWN the image,
    +z forward along the view direction.
  * earth/ENU frame: +x East, +y North, +z Up.
"""

import math
from typing import Optional, Sequence, Tuple

# The forward direction of this same geometry (world point -> pixel) already
# lives in nidar_detection, is already tested, and its own module docstring
# names this node as the consumer of the inverse. Importing the shared pieces
# rather than restating them means there is exactly one quaternion convention
# and one intrinsics parser in the workspace, and it lets the tests here
# round-trip against a reference implementation instead of only against
# hand-computed numbers. nidar_detection/__init__.py is empty, so this costs
# nothing at import time -- no ultralytics, no torch.
from nidar_detection.projection import Intrinsics, quaternion_to_matrix  # noqa: F401

Vec3 = Tuple[float, float, float]

#: Below this, a ray is treated as parallel to the ground and rejected rather
#: than producing a nearly-infinite intersection. cos(89.94 deg): a camera
#: within 0.06 deg of horizontal cannot usefully locate anything.
MIN_DOWNWARD_COMPONENT = 1e-3


def pixel_to_ray(u: float, v: float,
                 fx: float, fy: float, cx: float, cy: float) -> Vec3:
    """Back-project a pixel into a unit ray in the camera optical frame.

    The pinhole model inverted: a pixel (u, v) is the image of every point
    along ((u-cx)/fx, (v-cy)/fy, 1) * t. Returned normalised so callers can
    treat the ray length as metres along the ray.

    fx/fy come from CameraInfo.k, not from a constant -- the sim publishes
    554.25 for a 640x480 frame, but that is a property of the rendered
    sensor and changes with any resolution change.
    """
    if fx == 0.0 or fy == 0.0:
        raise ValueError('camera focal length must be non-zero')
    x = (u - cx) / fx
    y = (v - cy) / fy
    z = 1.0
    n = math.sqrt(x * x + y * y + z * z)
    return (x / n, y / n, z / n)


def rotate(matrix, vec: Sequence[float]) -> Vec3:
    """Apply a 3x3 rotation (numpy array or nested sequence) to a vector."""
    return tuple(float(sum(matrix[r][c] * vec[c] for c in range(3)))
                 for r in range(3))


def ray_ground_hit(origin: Sequence[float], direction: Sequence[float],
                   ground_z: float = 0.0) -> Optional[Vec3]:
    """Where a ray meets the horizontal plane z = ground_z, or None.

    None -- rather than an extrapolated point -- whenever the ray does not
    actually reach the ground: pointing up, pointing along the horizon, or
    starting below the plane. Those cases are real (a drone still on the pad,
    a gimbal mid-slew, a detection in the top rows of a tilted frame) and a
    returned coordinate would be a fabricated survivor position, which is
    worse than no detection at all.
    """
    oz, dz = float(origin[2]), float(direction[2])
    # The camera must be looking DOWN for a ground hit: dz strictly negative.
    if dz > -MIN_DOWNWARD_COMPONENT:
        return None
    t = (ground_z - oz) / dz
    if t <= 0.0:
        return None
    return (origin[0] + t * direction[0],
            origin[1] + t * direction[1],
            origin[2] + t * direction[2])


def project_pixel_to_ground(u: float, v: float,
                            fx: float, fy: float, cx: float, cy: float,
                            camera_position: Sequence[float],
                            camera_quaternion: Sequence[float],
                            ground_z: float = 0.0) -> Optional[Vec3]:
    """The whole pipeline: a pixel and a camera pose in, a ground point out.

    `camera_position` and `camera_quaternion` (x, y, z, w) describe the
    camera OPTICAL frame expressed in the target (earth/ENU) frame -- exactly
    what `tf2_ros.Buffer.lookup_transform(earth, optical_frame, ...)` returns.
    Returns the ground point in that same frame, or None if the ray never
    reaches the ground (see ray_ground_hit).
    """
    ray_cam = pixel_to_ray(u, v, fx, fy, cx, cy)
    rot = quaternion_to_matrix(*camera_quaternion)
    ray_world = rotate(rot, ray_cam)
    return ray_ground_hit(camera_position, ray_world, ground_z)
