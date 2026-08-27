"""Unit tests for the camera projection geometry.

The reference case throughout is the simulator's real configuration: a
1280x960 hd_camera with a 60 deg HFOV, mounted nadir, which the live
CameraInfo reports as fx = fy = 1108.5, cx = 640.5, cy = 480.5.
"""

import math

import pytest

from nidar_detection.projection import (
    CameraPose,
    Intrinsics,
    aabb_corners,
    project_box,
    project_point,
    quaternion_to_matrix,
    to_yolo_label,
)

# Nadir camera in the ROS optical convention (+Z forward along the view
# axis, +X right, +Y down). Looking straight down puts the camera's +Z on
# world -Z; keeping +X on world East then forces +Y onto world South, so
# North reads as "up" in the image. Those three axes are a 180 deg rotation
# about world X, i.e. quaternion (x=1, y=0, z=0, w=0).
NADIR_Q = (1.0, 0.0, 0.0, 0.0)


@pytest.fixture
def intr():
    return Intrinsics.from_hfov(1280, 960, 60.0)


def test_hfov_matches_the_sims_reported_intrinsics(intr):
    """Guards the whole projection against a wrong FOV assumption."""
    assert intr.fx == pytest.approx(1108.5, abs=0.5)
    assert intr.fy == pytest.approx(1108.5, abs=0.5)


def test_from_k_reads_camera_info_layout():
    k = [1108.5, 0.0, 640.5, 0.0, 1108.5, 480.5, 0.0, 0.0, 1.0]
    i = Intrinsics.from_k(k, 1280, 960)
    assert (i.fx, i.fy, i.cx, i.cy) == (1108.5, 1108.5, 640.5, 480.5)


def test_point_directly_below_projects_to_image_centre(intr):
    pose = CameraPose(position=(0, 0, 10), quaternion=NADIR_Q)
    u, v = project_point((0, 0, 0), pose, intr)
    assert u == pytest.approx(intr.cx)
    assert v == pytest.approx(intr.cy)


def test_ground_offset_scales_inversely_with_altitude(intr):
    """A 1 m offset should span fx/height pixels -- the same relationship
    path_planner's ground-footprint maths depends on."""
    for altitude in (5.0, 10.0, 25.0):
        pose = CameraPose(position=(0, 0, altitude), quaternion=NADIR_Q)
        u, _ = project_point((1.0, 0, 0), pose, intr)
        assert u - intr.cx == pytest.approx(intr.fx / altitude, rel=1e-6)


def test_footprint_at_10m_matches_the_documented_camera_model(intr):
    """At 10 m the frame should span ~11.55 m across -- the figure this
    project's lawnmower swath is derived from."""
    altitude = 10.0
    half_width_m = (intr.width / 2.0) * altitude / intr.fx
    assert 2 * half_width_m == pytest.approx(11.55, abs=0.05)


def test_point_behind_the_camera_is_rejected(intr):
    pose = CameraPose(position=(0, 0, 10), quaternion=NADIR_Q)
    assert project_point((0, 0, 30), pose, intr) is None


def test_a_person_below_gives_a_plausible_box(intr):
    """A 1.8 x 0.6 m standing person at 10 m should be tens of pixels."""
    pose = CameraPose(position=(0, 0, 10), quaternion=NADIR_Q)
    corners = aabb_corners(center=(0, 0, 0.9), size=(0.6, 0.6, 1.8))
    box = project_box(corners, pose, intr)
    assert box is not None
    _, _, w, h = box
    # Widest at the top of the head (9.1 m away), narrowest at the feet.
    assert 60 < w < 90
    assert 60 < h < 90


def test_box_size_grows_as_the_drone_descends(intr):
    corners = aabb_corners(center=(0, 0, 0.9), size=(0.6, 0.6, 1.8))
    widths = []
    for altitude in (25.0, 10.0, 5.0):
        pose = CameraPose(position=(0, 0, altitude), quaternion=NADIR_Q)
        widths.append(project_box(corners, pose, intr)[2])
    assert widths[0] < widths[1] < widths[2]


def test_box_entirely_outside_the_frame_is_rejected(intr):
    pose = CameraPose(position=(0, 0, 10), quaternion=NADIR_Q)
    corners = aabb_corners(center=(500, 500, 0.9), size=(0.6, 0.6, 1.8))
    assert project_box(corners, pose, intr) is None


def test_barely_visible_sliver_is_rejected(intr):
    """A person at 200 m is a couple of pixels: a label there teaches the
    model that near-invisible blobs are positives."""
    pose = CameraPose(position=(0, 0, 200.0), quaternion=NADIR_Q)
    corners = aabb_corners(center=(0, 0, 0.9), size=(0.6, 0.6, 1.8))
    assert project_box(corners, pose, intr, min_visible_px=10.0) is None


def test_yaw_rotates_the_footprint(intr):
    """A person lying down is longer along their own axis; yaw must show up
    in the projected box."""
    pose = CameraPose(position=(0, 0, 10), quaternion=NADIR_Q)
    flat = dict(center=(0, 0, 0.15), size=(1.8, 0.5, 0.3))
    box_0 = project_box(aabb_corners(**flat, yaw_rad=0.0), pose, intr)
    box_90 = project_box(aabb_corners(**flat, yaw_rad=math.pi / 2), pose, intr)
    assert box_0[2] > box_0[3]     # wider than tall
    assert box_90[3] > box_90[2]   # taller than wide
    assert box_0[2] == pytest.approx(box_90[3], rel=1e-6)


def test_aabb_corners_returns_eight_distinct_points():
    corners = aabb_corners(center=(1, 2, 3), size=(2, 4, 6))
    assert len(corners) == 8
    assert len(set(corners)) == 8
    xs = [c[0] for c in corners]
    assert min(xs) == pytest.approx(0.0) and max(xs) == pytest.approx(2.0)


def test_quaternion_normalises_non_unit_input():
    m = quaternion_to_matrix(0, 0, 0, 2.0)  # 2x identity quaternion
    assert m[0][0] == pytest.approx(1.0)
    assert m[1][1] == pytest.approx(1.0)


def test_zero_quaternion_is_an_error():
    with pytest.raises(ValueError):
        quaternion_to_matrix(0, 0, 0, 0)


def test_yolo_label_is_normalised_centre_and_size(intr):
    line = to_yolo_label((640.0 - 64, 480.0 - 48, 128, 96), intr)
    cls, cx, cy, w, h = line.split()
    assert cls == '0'
    assert float(cx) == pytest.approx(0.5, abs=1e-3)
    assert float(cy) == pytest.approx(0.5, abs=1e-3)
    assert float(w) == pytest.approx(0.1, abs=1e-3)
    assert float(h) == pytest.approx(0.1, abs=1e-3)
