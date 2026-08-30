"""Person detection on a single image, with no ROS dependency.

Split from detection_node.py for the same reason nidar_mission_clock/clock.py
is split from its node: the parts worth testing (which boxes survive
filtering, how a bbox becomes a DetectionResult's x/y/w/h, how a device
string resolves) are testable directly, without a ROS graph, a camera, or a
GPU.

Backend note: this wraps ultralytics' YOLO, which dispatches on what
`model_path` points at -- `.pt` (PyTorch), `.onnx` (ONNX Runtime),
`.engine` (TensorRT), or a `*_ncnn_model/` DIRECTORY (ncnn). That means the
Jetson deployment in Phase 8 changes the `model_path` parameter and nothing
else in this file, and a retrained model is likewise a weights swap, not a
code change.

ncnn is not just one more backend, it is the one that matters when there is
no usable GPU -- measured on this project's own dev box, on a live 640x480
sim frame, same YOLO26n weights:

    .pt  on CUDA, imgsz 640 ....  13.2 ms   (76.0 FPS)
    .pt  on CPU,  imgsz 640 ...  379.4 ms   ( 2.6 FPS)
    .pt  on CPU,  imgsz 416 ...  213.2 ms   ( 4.7 FPS)
    ncnn on CPU,  imgsz 640 ...  122.2 ms   ( 8.2 FPS)

So on CPU ncnn is 3.1x faster than PyTorch at the SAME input size, and still
1.7x faster than PyTorch degraded to 416 -- i.e. it buys speed without
giving up the resolution the model was trained at. That is why
resolve_model_path() below prefers an ncnn sibling automatically whenever
inference lands on CPU, and why nothing prefers it when CUDA is available.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

#: COCO class index for "person". Stock YOLOv8 weights are COCO-trained, and
#: implementation-plan section 2.1's eventual fine-tune is single-class
#: (person only), which also lands at index 0 -- so this holds either way.
PERSON_CLASS_ID = 0


#: track_id value meaning "the tracker gave this detection no identity".
UNTRACKED = -1


@dataclass(frozen=True)
class Detection:
    """One detected person, in pixel coordinates of the frame it came from.

    x/y are the bounding box's top-left corner and w/h its size, matching
    nidar_msgs/DetectionResult's bbox_x/bbox_y/bbox_w/bbox_h fields (and the
    convention Phase 3's geotag node will project from: the box *centre*,
    computed as x + w/2, y + h/2).

    track_id is a ByteTrack identity that stays the same across frames for
    the same physical person, or UNTRACKED when the tracker has not assigned
    one. It is what makes "how many survivors" answerable -- without it, a
    person visible for 30 frames looks like 30 survivors.
    """

    confidence: float
    x: float
    y: float
    w: float
    h: float
    track_id: int = UNTRACKED

    @property
    def center(self) -> tuple:
        return (self.x + self.w / 2.0, self.y + self.h / 2.0)


def boxes_to_detections(
    xyxy: Sequence[Sequence[float]],
    confidences: Sequence[float],
    class_ids: Sequence[float],
    confidence_threshold: float,
    person_class_id: int = PERSON_CLASS_ID,
    track_ids: Optional[Sequence[float]] = None,
) -> List[Detection]:
    """Convert raw model output (corner boxes + scores + classes) into
    Detections, keeping only persons above the confidence threshold.

    Pulled out as a free function so the filtering rules can be tested
    against hand-written arrays -- no model, no GPU, no image.

    `track_ids` is optional because the tracker only emits ids once a track
    is confirmed: ultralytics returns `boxes.id is None` for a whole frame
    of brand-new detections, and those detections are still real and must
    still be published -- just without an identity yet.
    """
    ids = list(track_ids) if track_ids is not None else None
    out: List[Detection] = []
    for i, (box, conf, cls) in enumerate(zip(xyxy, confidences, class_ids)):
        if int(cls) != person_class_id:
            continue
        conf = float(conf)
        if conf < confidence_threshold:
            continue
        x1, y1, x2, y2 = (float(v) for v in box)
        # Guard against a model returning corners in either order rather
        # than assuming x1<x2: a negative width would produce a nonsense
        # bbox that silently poisons the geotag projection downstream.
        left, right = min(x1, x2), max(x1, x2)
        top, bottom = min(y1, y2), max(y1, y2)
        tid = int(ids[i]) if ids is not None and i < len(ids) else UNTRACKED
        out.append(Detection(confidence=conf, x=left, y=top,
                             w=right - left, h=bottom - top, track_id=tid))
    return out


def resolve_device(requested: str) -> str:
    """Map a device parameter to something ultralytics accepts.

    'auto' picks CUDA when it is genuinely usable and CPU otherwise. Checked
    at runtime rather than assumed: this project has seen
    torch.cuda.is_available() report False on a machine whose nvidia-smi
    looked healthy, and a hard-coded 'cuda' turns that into a crash loop
    instead of a slower node.
    """
    requested = (requested or 'auto').strip().lower()
    if requested != 'auto':
        return requested
    try:
        import torch
        return '0' if torch.cuda.is_available() else 'cpu'
    except Exception:  # noqa: BLE001 - a missing/broken torch means CPU, not a crash
        return 'cpu'


#: Directory-name suffix `yolo export format=ncnn` produces, e.g.
#: nidar_person.pt -> nidar_person_ncnn_model/. Following ultralytics' own
#: convention is what lets resolve_model_path() find the sibling without any
#: extra configuration: re-exporting a retrained model puts it exactly here.
NCNN_MODEL_SUFFIX = '_ncnn_model'


def is_ncnn_model(model_path: str) -> bool:
    """True when `model_path` is an ncnn export directory.

    Checks for the .param file rather than trusting the directory name: an
    empty or half-copied `*_ncnn_model/` would otherwise be handed to
    ultralytics and fail deep inside its loader, where the message says
    nothing about which of several model paths was wrong.
    """
    return bool(model_path) and os.path.isdir(model_path) and \
        os.path.isfile(os.path.join(model_path, 'model.ncnn.param'))


def resolve_model_path(model_path: str, device: str,
                       prefer_ncnn_on_cpu: bool = True) -> str:
    """Pick the backend artifact to actually load for `device`.

    `model_path` stays the single swap point for the whole system -- drop in
    retrained weights there and every drone picks them up. This only chooses
    WHICH export of those same weights to run, and only in the one case where
    the choice is free: on CPU, where the ncnn export is 3x faster than the
    .pt for identical output (see the module docstring's measurements).

    Returns `model_path` unchanged when there is nothing to gain: on GPU,
    when opted out, when already pointed at an ncnn directory, or when no
    ncnn export sits beside the weights. A retrained .pt with no ncnn export
    yet therefore keeps working -- it just runs on the slower backend, and
    the node says so.
    """
    device = (device or '').strip().lower()
    if device != 'cpu' or not prefer_ncnn_on_cpu:
        return model_path
    if is_ncnn_model(model_path):
        return model_path
    sibling = os.path.splitext(model_path)[0] + NCNN_MODEL_SUFFIX
    return sibling if is_ncnn_model(sibling) else model_path


class PersonDetector:
    """Runs a YOLO person detector over BGR frames."""

    def __init__(
        self,
        model_path: str,
        confidence_threshold: float = 0.5,
        iou_threshold: float = 0.45,
        input_size: int = 640,
        device: str = 'auto',
        person_class_id: int = PERSON_CLASS_ID,
        tracker: str = 'bytetrack.yaml',
        prefer_ncnn_on_cpu: bool = True,
    ):
        from ultralytics import YOLO  # imported here so the module stays importable without it

        self.requested_model_path = model_path
        self.confidence_threshold = float(confidence_threshold)
        self.iou_threshold = float(iou_threshold)
        self.input_size = int(input_size)
        self.person_class_id = int(person_class_id)
        self.device = resolve_device(device)
        self.model_path = resolve_model_path(
            model_path, self.device, prefer_ncnn_on_cpu)
        self.backend = 'ncnn' if is_ncnn_model(self.model_path) else 'torch'
        if self.backend == 'ncnn':
            # ncnn has no CUDA path in ultralytics' backend. Pinning this
            # rather than passing the caller's device through keeps
            # self.device an honest statement of where inference runs -- the
            # node logs it, and a device that lies about that is how "why is
            # this slow" investigations start in the wrong place.
            self.device = 'cpu'
        # Empty string disables tracking and falls back to plain detection.
        self.tracker = (tracker or '').strip()
        self._model = YOLO(self.model_path)

    def detect(self, bgr_image: np.ndarray) -> List[Detection]:
        """Detect persons in one BGR frame, assigning stable track ids.

        Uses ultralytics' `track(persist=True)` rather than `predict()` when a
        tracker is configured. `persist=True` is what carries tracker state
        from one call to the next -- without it every frame starts a fresh
        tracker and every detection gets id 1, which is worse than no ids at
        all because it looks like it is working.

        Tracker state lives in this model instance, and there is one instance
        per drone (one node per drone), so ids never collide across drones by
        accident -- they are simply unrelated between them, which is exactly
        what they are.
        """
        common = dict(
            source=bgr_image,
            imgsz=self.input_size,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            device=self.device,
            # classes= filters inside the model's own NMS, so non-person
            # COCO classes never reach us at all -- cheaper than detecting
            # everything and discarding it here.
            classes=[self.person_class_id],
            verbose=False,
        )
        if self.tracker:
            results = self._model.track(persist=True, tracker=self.tracker, **common)
        else:
            results = self._model.predict(**common)

        if not results:
            return []
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return []
        # boxes.id is None for a frame where no track is confirmed yet; those
        # detections are still real and are published as UNTRACKED.
        track_ids = None
        if getattr(boxes, 'id', None) is not None:
            track_ids = boxes.id.cpu().numpy()
        return boxes_to_detections(
            boxes.xyxy.cpu().numpy(),
            boxes.conf.cpu().numpy(),
            boxes.cls.cpu().numpy(),
            self.confidence_threshold,
            self.person_class_id,
            track_ids=track_ids,
        )


def annotate(image: np.ndarray, detections: Sequence[Detection],
             color: tuple = (0, 220, 60), thickness: int = 2) -> np.ndarray:
    """Draw detection boxes and confidences onto a copy of `image`.

    Returns a copy, never mutating the caller's frame: the same frame is
    also the one handed to the detector, and drawing on it in place would
    feed annotated pixels back into the next inference if a caller ever
    reordered those two steps.
    """
    import cv2

    out = image.copy()
    h, w = out.shape[:2]
    for det in detections:
        # Clamp to the frame: a box can extend past the edge when the model
        # extrapolates a partially-visible person, and cv2.rectangle with
        # out-of-range points draws nothing at all rather than clipping.
        x1 = max(0, min(w - 1, int(round(det.x))))
        y1 = max(0, min(h - 1, int(round(det.y))))
        x2 = max(0, min(w - 1, int(round(det.x + det.w))))
        y2 = max(0, min(h - 1, int(round(det.y + det.h))))
        cv2.rectangle(out, (x1, y1), (x2, y2), color, thickness)
        # Lead with the track id: on a moving feed it is the only way to see
        # at a glance whether one person is holding one identity or flickering
        # between several.
        label = (f'#{det.track_id} {det.confidence:.2f}' if det.track_id != UNTRACKED
                 else f'person {det.confidence:.2f}')
        cv2.putText(out, label, (x1, max(12, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return out


def scale_to_width(image: np.ndarray, max_width: Optional[int]) -> np.ndarray:
    """Downscale `image` so it is at most `max_width` wide, preserving aspect.

    Used only for the GCS preview stream. Full-resolution frames over
    rosbridge are what drove this project's browser out of memory once
    already (1280x960 BGR is 3.6 MB raw, ~4.9 MB base64-encoded, per frame,
    per drone) -- the operator preview does not need those pixels, and the
    detector has already run at full resolution by this point.
    """
    import cv2

    if not max_width or image.shape[1] <= max_width:
        return image
    scale = max_width / float(image.shape[1])
    return cv2.resize(image, (max_width, max(1, int(round(image.shape[0] * scale)))),
                      interpolation=cv2.INTER_AREA)
