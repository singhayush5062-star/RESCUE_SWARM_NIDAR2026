"""Unit tests for silhouette-based label extraction."""

import numpy as np
import pytest

from nidar_detection.silhouette import (
    blob_boxes,
    box_center,
    match_box_to_point,
    pad_box,
    saturation_mask,
)


def grey_frame(h=200, w=300, value=140):
    """A frame of bare ground: uniform grey, zero saturation."""
    return np.full((h, w, 3), value, dtype=np.uint8)


def paint(frame, x, y, w, h, bgr=(40, 160, 60)):
    frame[y:y + h, x:x + w] = bgr
    return frame


def test_bare_ground_has_no_coloured_pixels():
    assert not saturation_mask(grey_frame()).any()


def test_shadow_is_not_mistaken_for_a_survivor():
    """A shadow is a luminance change on grey -- still zero saturation."""
    frame = grey_frame()
    frame[50:120, 60:130] = 70  # darker grey
    assert not saturation_mask(frame).any()


def test_coloured_actor_is_detected():
    frame = paint(grey_frame(), 100, 60, 30, 20)
    mask = saturation_mask(frame)
    assert mask.any()
    boxes = blob_boxes(mask)
    assert len(boxes) == 1
    assert boxes[0] == (100.0, 60.0, 30.0, 20.0)


def test_two_actors_give_two_boxes():
    frame = grey_frame()
    paint(frame, 20, 20, 15, 15)
    paint(frame, 200, 150, 25, 18, bgr=(20, 40, 200))
    boxes = blob_boxes(saturation_mask(frame))
    assert len(boxes) == 2


def test_tiny_speckle_is_ignored():
    frame = paint(grey_frame(), 100, 60, 2, 2)
    assert blob_boxes(saturation_mask(frame), min_area_px=12) == []


def test_box_center():
    assert box_center((10, 20, 30, 40)) == (25.0, 40.0)


def test_match_picks_the_nearest_blob():
    boxes = [(0, 0, 10, 10), (200, 200, 10, 10)]
    got = match_box_to_point(boxes, (205, 203), max_distance_px=50)
    assert got == (200.0, 200.0, 10.0, 10.0)


def test_match_rejects_a_blob_outside_the_radius():
    """A blob far from where projection says the actor is belongs to
    something else -- a wrong label is worse than a dropped sample."""
    boxes = [(0, 0, 10, 10)]
    assert match_box_to_point(boxes, (500, 500), max_distance_px=50) is None


def test_match_with_no_blobs_is_none():
    assert match_box_to_point([], (10, 10), max_distance_px=50) is None


def test_pad_box_grows_and_clips_to_the_frame():
    assert pad_box((10, 10, 20, 20), 2, 300, 200) == (8.0, 8.0, 24.0, 24.0)
    # Clipped at the top-left corner.
    assert pad_box((0, 0, 20, 20), 3, 300, 200) == (0.0, 0.0, 23.0, 23.0)
    # Clipped at the bottom-right corner.
    assert pad_box((280, 180, 20, 20), 5, 300, 200) == (275.0, 175.0, 25.0, 25.0)


def test_end_to_end_label_extraction_on_a_synthetic_scene():
    """The full capture path: mask -> blobs -> match by projected position."""
    frame = grey_frame(480, 640)
    paint(frame, 300, 220, 24, 18)          # the actor we care about
    paint(frame, 600, 60, 30, 30)           # the drone's own airframe, off in a corner
    frame[400:460, 100:200] = 90            # a shadow

    boxes = blob_boxes(saturation_mask(frame))
    projected = (312, 229)                  # where projection says the actor is
    match = match_box_to_point(boxes, projected, max_distance_px=40)
    assert match == (300.0, 220.0, 24.0, 18.0)
    assert box_center(match) == pytest.approx((312.0, 229.0))
