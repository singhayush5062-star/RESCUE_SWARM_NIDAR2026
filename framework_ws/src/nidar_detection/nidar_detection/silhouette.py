"""Extracts a survivor's exact pixel silhouette from a simulator frame.

Used only by dataset capture, to turn "the simulator knows this actor is at
world (x, y)" into a pixel-tight training label.

Why not just project the actor's bounding box: a person is not a box. A
projected 3D AABB covers the volume a standing figure *could* occupy, which
at nadir is roughly twice the area the figure actually covers -- measured
live, a projected 0.6 x 0.5 x 1.8 m box came out 100 px wide against a real
silhouette of 66 px. Training on boxes that loose teaches the detector that
a person is a large mostly-empty blob, and it then localises badly, which
Phase 3 turns directly into geotag error.

The extraction exploits a property of this specific simulator scene, not a
general one: the ground plane is untextured grey and the actor mesh is
coloured, so a saturation threshold separates them exactly. Shadows are a
luminance change on grey and stay unsaturated, so they are excluded for
free. This assumption is checked at capture time -- a frame where it does
not hold yields no label rather than a wrong one -- and it is why this lives
in the capture tool and not in the detection node, which never sees a
synthetic scene.
"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np

#: HSV saturation above which a pixel is considered "not bare ground".
DEFAULT_SATURATION_THRESHOLD = 40

#: Smallest blob (in pixels) that can be a survivor rather than render noise.
DEFAULT_MIN_AREA_PX = 12


def saturation_mask(bgr: np.ndarray, threshold: int = DEFAULT_SATURATION_THRESHOLD) -> np.ndarray:
    """Boolean mask of coloured (non-grey) pixels."""
    import cv2

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return hsv[:, :, 1] > threshold


def blob_boxes(mask: np.ndarray, min_area_px: int = DEFAULT_MIN_AREA_PX
               ) -> List[Tuple[float, float, float, float]]:
    """Tight (x, y, w, h) boxes of each connected component in `mask`."""
    import cv2

    n, _, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    out = []
    for i in range(1, n):  # 0 is the background component
        x, y, w, h, area = stats[i]
        if area < min_area_px:
            continue
        out.append((float(x), float(y), float(w), float(h)))
    return out


def box_center(box: Sequence[float]) -> Tuple[float, float]:
    return (box[0] + box[2] / 2.0, box[1] + box[3] / 2.0)


def match_box_to_point(boxes: Sequence[Sequence[float]],
                       point: Sequence[float],
                       max_distance_px: float) -> Optional[Tuple[float, float, float, float]]:
    """Pick the blob whose centre is nearest `point`, within a radius.

    `point` is where projection says the actor should be. The radius is what
    makes this safe: if the nearest blob is far from where the actor
    provably is, it belongs to something else (another actor, the drone's
    own airframe at the frame edge) and matching it would produce a
    confidently wrong label. Returning None there drops the sample instead.
    """
    best = None
    best_d2 = max_distance_px * max_distance_px
    px, py = float(point[0]), float(point[1])
    for box in boxes:
        cx, cy = box_center(box)
        d2 = (cx - px) ** 2 + (cy - py) ** 2
        if d2 <= best_d2:
            best_d2 = d2
            best = tuple(float(v) for v in box)
    return best


def pad_box(box: Sequence[float], pad_px: float, width: int, height: int
            ) -> Tuple[float, float, float, float]:
    """Grow a box by `pad_px` on each side, clipped to the frame.

    A one-pixel margin around the silhouette: the saturation threshold cuts
    at the mesh's anti-aliased edge, so the tight box is very slightly
    inside the visible figure.
    """
    x, y, w, h = (float(v) for v in box)
    x1 = max(0.0, x - pad_px)
    y1 = max(0.0, y - pad_px)
    x2 = min(float(width), x + w + pad_px)
    y2 = min(float(height), y + h + pad_px)
    return (x1, y1, x2 - x1, y2 - y1)
