"""Projection geometry, checked against hand-computed answers.

No model, no simulator, no TF: every case here is a pinhole camera whose
answer can be worked out on paper, so a failure points at the math rather
than at the sim.
"""
import math

import pytest

from nidar_detection.projection import (
    CameraPose, Intrinsics, project_point)

from nidar_geotag import projection as p

# The sim's actual hd_camera intrinsics (CameraInfo.k, measured live).
FX = FY = 554.25
CX, CY = 320.5, 240.5

# earth <- optical rotation for a true nadir camera: optical +z (view
# direction) maps to earth -z, optical +x maps to earth -y, optical +y to
# earth -x. Quaternion form of that rotation, exact.
NADIR_Q = (-math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)


def test_centre_pixel_is_a_forward_ray():
    assert p.pixel_to_ray(CX, CY, FX, FY, CX, CY) == pytest.approx((0.0, 0.0, 1.0))


def test_ray_is_unit_length():
    r = p.pixel_to_ray(10.0, 470.0, FX, FY, CX, CY)
    assert math.sqrt(sum(c * c for c in r)) == pytest.approx(1.0)


def test_pixel_right_of_centre_tilts_ray_positive_x():
    r = p.pixel_to_ray(CX + 100, CY, FX, FY, CX, CY)
    assert r[0] > 0 and r[1] == pytest.approx(0.0)


def test_zero_focal_length_is_rejected():
    with pytest.raises(ValueError):
        p.pixel_to_ray(CX, CY, 0.0, FY, CX, CY)


def test_unnormalised_quaternion_still_gives_a_rotation():
    """TF quaternions are not exactly unit length; scaling must not leak into
    the ray, or the ground intersection shifts."""
    m = p.quaternion_to_matrix(0.0, 0.0, 0.0, 2.0)
    assert p.rotate(m, (1.0, 0.0, 0.0)) == pytest.approx((1.0, 0.0, 0.0))


def test_nadir_rotation_points_view_axis_straight_down():
    m = p.quaternion_to_matrix(*NADIR_Q)
    assert p.rotate(m, (0.0, 0.0, 1.0)) == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)


def test_ground_hit_straight_down():
    hit = p.ray_ground_hit((5.0, 7.0, 10.0), (0.0, 0.0, -1.0))
    assert hit == pytest.approx((5.0, 7.0, 0.0))


def test_ground_hit_respects_a_raised_ground_plane():
    hit = p.ray_ground_hit((5.0, 7.0, 10.0), (0.0, 0.0, -1.0), ground_z=2.0)
    assert hit == pytest.approx((5.0, 7.0, 2.0))


def test_upward_ray_never_reaches_the_ground():
    assert p.ray_ground_hit((0.0, 0.0, 10.0), (0.0, 0.0, 1.0)) is None


def test_horizontal_ray_is_rejected_not_extrapolated():
    """A grazing ray would otherwise yield a point kilometres away and be
    published as a survivor position."""
    assert p.ray_ground_hit((0.0, 0.0, 10.0), (1.0, 0.0, 0.0)) is None


def test_camera_below_the_ground_plane_yields_nothing():
    assert p.ray_ground_hit((0.0, 0.0, -1.0), (0.0, 0.0, -1.0)) is None


def test_nadir_centre_pixel_lands_directly_beneath_the_camera():
    hit = p.project_pixel_to_ground(CX, CY, FX, FY, CX, CY,
                                    (12.0, -4.0, 20.0), NADIR_Q)
    assert hit == pytest.approx((12.0, -4.0, 0.0), abs=1e-6)


def test_nadir_offset_pixel_scales_with_altitude():
    """100 px off centre at 10 m must be 100/554.25*10 = 1.804 m on the
    ground, and exactly twice that at 20 m."""
    expected = 100.0 / FX * 10.0
    hit = p.project_pixel_to_ground(CX + 100, CY, FX, FY, CX, CY,
                                    (0.0, 0.0, 10.0), NADIR_Q)
    assert math.hypot(hit[0], hit[1]) == pytest.approx(expected, rel=1e-6)

    hit2 = p.project_pixel_to_ground(CX + 100, CY, FX, FY, CX, CY,
                                     (0.0, 0.0, 20.0), NADIR_Q)
    assert math.hypot(hit2[0], hit2[1]) == pytest.approx(2 * expected, rel=1e-6)


def test_nadir_image_axes_map_to_expected_ground_directions():
    """Guards the axis convention itself: with this rotation, +u (image
    right) must move the ground point along earth -Y, and +v (image down)
    along earth -X. Getting this pair swapped or sign-flipped produces
    positions that look entirely plausible and are wrong."""
    base = p.project_pixel_to_ground(CX, CY, FX, FY, CX, CY, (0.0, 0.0, 10.0), NADIR_Q)
    right = p.project_pixel_to_ground(CX + 50, CY, FX, FY, CX, CY, (0.0, 0.0, 10.0), NADIR_Q)
    down = p.project_pixel_to_ground(CX, CY + 50, FX, FY, CX, CY, (0.0, 0.0, 10.0), NADIR_Q)
    assert right[1] < base[1] and right[0] == pytest.approx(base[0], abs=1e-9)
    assert down[0] < base[0] and down[1] == pytest.approx(base[1], abs=1e-9)


def test_a_survivor_at_a_known_spot_round_trips():
    """The Phase 3.4 case in miniature: a drone at 15 m over (3, 4) sees a
    person whose image lands 60 px right and 80 px down of centre. Work the
    ground offset out by hand and require the projection to agree."""
    alt = 15.0
    du, dv = 60.0, 80.0
    hit = p.project_pixel_to_ground(CX + du, CY + dv, FX, FY, CX, CY,
                                    (3.0, 4.0, alt), NADIR_Q)
    assert hit[0] == pytest.approx(3.0 - dv / FY * alt, abs=1e-6)
    assert hit[1] == pytest.approx(4.0 - du / FX * alt, abs=1e-6)


# --- agreement with the forward projection -------------------------------
#
# nidar_detection.projection maps a world point to a pixel and is already
# tested in its own right. This node is that map inverted, so the strongest
# available check is the round trip: take a point on the ground, project it
# to a pixel, unproject that pixel back, and require the original point.
# A sign error, a swapped axis or a transposed rotation all survive
# hand-written cases that happen to be symmetric; none survive this.

def _round_trip(world_xy, cam_pos, quat=NADIR_Q):
    intr = Intrinsics(fx=FX, fy=FY, cx=CX, cy=CY, width=641, height=481)
    pose = CameraPose(cam_pos, quat)
    pixel = project_point((world_xy[0], world_xy[1], 0.0), pose, intr)
    assert pixel is not None, 'test point is behind the camera'
    return p.project_pixel_to_ground(pixel[0], pixel[1], FX, FY, CX, CY,
                                     cam_pos, quat, 0.0)


@pytest.mark.parametrize('world_xy', [
    (0.0, 0.0), (3.0, 4.0), (-6.5, 2.25), (11.0, -8.0), (-1.0, -14.0),
])
def test_round_trip_nadir(world_xy):
    hit = _round_trip(world_xy, (0.0, 0.0, 18.0))
    assert hit[0] == pytest.approx(world_xy[0], abs=1e-6)
    assert hit[1] == pytest.approx(world_xy[1], abs=1e-6)


def test_round_trip_with_the_camera_offset_from_the_origin():
    hit = _round_trip((7.0, -3.0), (25.0, 25.0, 12.0))
    assert hit[0] == pytest.approx(7.0, abs=1e-6)
    assert hit[1] == pytest.approx(-3.0, abs=1e-6)


def test_round_trip_survives_a_tilted_camera():
    """The gimbal is not perfectly nadir in flight -- the live transform read
    0.046 deg off. The inverse must track whatever attitude TF reports, not
    assume the commanded one."""
    tilt = math.radians(8.0)
    # NADIR_Q composed with a small rotation about the optical x axis.
    qx, qy, qz, qw = NADIR_Q
    sx, cx_ = math.sin(tilt / 2), math.cos(tilt / 2)
    quat = (qw * sx + qx * cx_, qy * cx_ + qz * sx,
            qz * cx_ - qy * sx, qw * cx_ - qx * sx)
    hit = _round_trip((2.0, 5.0), (0.0, 0.0, 20.0), quat)
    assert hit[0] == pytest.approx(2.0, abs=1e-5)
    assert hit[1] == pytest.approx(5.0, abs=1e-5)
