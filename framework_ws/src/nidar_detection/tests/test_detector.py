"""Unit tests for the detection filtering / bbox conversion rules.

No model, no GPU, no camera: these cover the logic that decides which model
outputs become DetectionResult messages and what geometry ends up in them.
Whether YOLO itself finds a person is a model question, checked live against
the simulator, not something a unit test can meaningfully assert.
"""

import numpy as np
import pytest

from nidar_detection.detector import (
    UNTRACKED,
    PERSON_CLASS_ID,
    Detection,
    annotate,
    boxes_to_detections,
    resolve_device,
    scale_to_width,
)


def test_person_boxes_above_threshold_are_kept():
    dets = boxes_to_detections(
        xyxy=[[10, 20, 50, 100]],
        confidences=[0.9],
        class_ids=[PERSON_CLASS_ID],
        confidence_threshold=0.5,
    )
    assert len(dets) == 1
    assert dets[0] == Detection(confidence=0.9, x=10, y=20, w=40, h=80)


def test_non_person_classes_are_dropped():
    dets = boxes_to_detections(
        xyxy=[[10, 20, 50, 100], [0, 0, 10, 10]],
        confidences=[0.99, 0.99],
        class_ids=[2, 15],  # car, cat
        confidence_threshold=0.5,
    )
    assert dets == []


def test_low_confidence_is_dropped():
    dets = boxes_to_detections(
        xyxy=[[10, 20, 50, 100]],
        confidences=[0.31],
        class_ids=[PERSON_CLASS_ID],
        confidence_threshold=0.5,
    )
    assert dets == []


def test_threshold_is_inclusive():
    dets = boxes_to_detections(
        xyxy=[[0, 0, 4, 4]], confidences=[0.5],
        class_ids=[PERSON_CLASS_ID], confidence_threshold=0.5)
    assert len(dets) == 1


def test_reversed_corners_still_give_positive_size():
    """A negative width would silently poison Phase 3's geotag projection."""
    dets = boxes_to_detections(
        xyxy=[[50, 100, 10, 20]],  # x2<x1 and y2<y1
        confidences=[0.8],
        class_ids=[PERSON_CLASS_ID],
        confidence_threshold=0.5,
    )
    assert dets[0].w == 40
    assert dets[0].h == 80
    assert dets[0].x == 10
    assert dets[0].y == 20


def test_center_is_the_bbox_midpoint():
    """Phase 3 projects the box centre through the camera intrinsics."""
    d = Detection(confidence=0.9, x=10, y=20, w=40, h=80)
    assert d.center == (30.0, 60.0)


def test_mixed_batch_keeps_only_qualifying_persons():
    dets = boxes_to_detections(
        xyxy=[[0, 0, 10, 10], [20, 20, 40, 60], [5, 5, 9, 9]],
        confidences=[0.9, 0.4, 0.7],
        class_ids=[PERSON_CLASS_ID, PERSON_CLASS_ID, 2],
        confidence_threshold=0.5,
    )
    assert len(dets) == 1
    assert dets[0].x == 0 and dets[0].w == 10


def test_empty_input_gives_no_detections():
    assert boxes_to_detections([], [], [], 0.5) == []


def test_numpy_arrays_are_accepted():
    """The real caller passes numpy arrays off the GPU, not Python lists."""
    dets = boxes_to_detections(
        xyxy=np.array([[10.0, 20.0, 50.0, 100.0]], dtype=np.float32),
        confidences=np.array([0.77], dtype=np.float32),
        class_ids=np.array([0.0], dtype=np.float32),
        confidence_threshold=0.5,
    )
    assert len(dets) == 1
    assert dets[0].confidence == pytest.approx(0.77, abs=1e-6)


@pytest.mark.parametrize('requested,expected', [
    ('cpu', 'cpu'),
    ('0', '0'),
    ('cuda:1', 'cuda:1'),
    ('CPU', 'cpu'),
])
def test_explicit_device_is_passed_through(requested, expected):
    assert resolve_device(requested) == expected


def test_auto_device_resolves_to_something_usable():
    assert resolve_device('auto') in ('cpu', '0')
    assert resolve_device('') in ('cpu', '0')


def test_annotate_does_not_mutate_the_input_frame():
    frame = np.zeros((100, 200, 3), dtype=np.uint8)
    before = frame.copy()
    out = annotate(frame, [Detection(confidence=0.9, x=10, y=10, w=40, h=40)])
    assert np.array_equal(frame, before)
    assert not np.array_equal(out, before)  # something was actually drawn


def test_annotate_handles_boxes_running_past_the_frame_edge():
    frame = np.zeros((100, 200, 3), dtype=np.uint8)
    out = annotate(frame, [Detection(confidence=0.9, x=180, y=90, w=90, h=90)])
    assert out.shape == frame.shape


def test_annotate_with_no_detections_is_a_clean_copy():
    frame = np.full((20, 30, 3), 7, dtype=np.uint8)
    out = annotate(frame, [])
    assert np.array_equal(out, frame)


def test_scale_to_width_downscales_and_preserves_aspect():
    frame = np.zeros((960, 1280, 3), dtype=np.uint8)
    out = scale_to_width(frame, 640)
    assert out.shape[1] == 640
    assert out.shape[0] == 480


def test_scale_to_width_leaves_smaller_frames_alone():
    frame = np.zeros((100, 200, 3), dtype=np.uint8)
    assert scale_to_width(frame, 640) is frame
    assert scale_to_width(frame, 0) is frame
    assert scale_to_width(frame, None) is frame


# --- ByteTrack identity -------------------------------------------------
# These cover the rule that makes "how many survivors" answerable: one
# physical person must keep one id across frames, so counting people means
# counting distinct track_ids, not detections.

def test_track_ids_are_carried_through():
    dets = boxes_to_detections(
        xyxy=[[10, 20, 50, 100], [200, 200, 240, 280]],
        confidences=[0.9, 0.8],
        class_ids=[PERSON_CLASS_ID, PERSON_CLASS_ID],
        confidence_threshold=0.5,
        track_ids=[7, 12],
    )
    assert [d.track_id for d in dets] == [7, 12]


def test_missing_track_ids_give_untracked_not_a_crash():
    """ultralytics returns boxes.id is None until a track is confirmed; those
    detections are real and must still be published."""
    dets = boxes_to_detections(
        xyxy=[[10, 20, 50, 100]], confidences=[0.9],
        class_ids=[PERSON_CLASS_ID], confidence_threshold=0.5)
    assert dets[0].track_id == UNTRACKED


def test_track_ids_stay_aligned_when_a_detection_is_filtered_out():
    """The id array is indexed by the model's raw output, so filtering a
    low-confidence box must not shift every later id by one -- that would
    silently attribute one person's identity to a different person."""
    dets = boxes_to_detections(
        xyxy=[[0, 0, 10, 10], [20, 20, 60, 100], [200, 200, 240, 280]],
        confidences=[0.2, 0.9, 0.8],          # first one is dropped
        class_ids=[PERSON_CLASS_ID] * 3,
        confidence_threshold=0.5,
        track_ids=[3, 4, 5],
    )
    assert [d.track_id for d in dets] == [4, 5]


def test_track_ids_stay_aligned_when_a_non_person_is_filtered_out():
    dets = boxes_to_detections(
        xyxy=[[0, 0, 10, 10], [20, 20, 60, 100]],
        confidences=[0.9, 0.9],
        class_ids=[2, PERSON_CLASS_ID],       # car, then person
        confidence_threshold=0.5,
        track_ids=[3, 4],
    )
    assert [d.track_id for d in dets] == [4]


def test_counting_unique_tracks_collapses_repeated_observations():
    """The whole point: the same person seen over many frames is one person."""
    frames = [
        boxes_to_detections([[10, 20, 50, 100]], [0.9], [PERSON_CLASS_ID], 0.5, track_ids=[1])
        for _ in range(30)
    ]
    observations = sum(len(f) for f in frames)
    unique = {d.track_id for f in frames for d in f}
    assert observations == 30
    assert len(unique) == 1


def test_annotate_labels_the_track_id_when_there_is_one():
    frame = np.zeros((200, 300, 3), dtype=np.uint8)
    tracked = annotate(frame, [Detection(0.9, 10, 10, 60, 60, track_id=5)])
    untracked = annotate(frame, [Detection(0.9, 10, 10, 60, 60)])
    # Different labels are drawn, so the rendered pixels must differ.
    assert not np.array_equal(tracked, untracked)
