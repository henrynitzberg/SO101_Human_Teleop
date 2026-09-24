# Thanks, Claude!

import cv2
import numpy as np


def combine_side_by_side(frame_a, frame_b):
    """Resize B to match A's shape (if needed) and horizontally stack them -
    the standard two-camera preview layout used throughout this project."""
    if frame_a.shape[:2] != frame_b.shape[:2]:
        frame_b = cv2.resize(frame_b, (frame_a.shape[1], frame_a.shape[0]))
    return np.hstack((frame_a, frame_b))


def _rounded_rect_points(pt1, pt2, radius):
    """The rounded rectangle's boundary as a single closed polygon: 4
    quarter-circle arcs (one per corner) joined by the straight edges."""
    x1, y1 = pt1
    x2, y2 = pt2
    pts = []
    pts += list(cv2.ellipse2Poly((x2 - radius, y1 + radius), (radius, radius), 0, 270, 360, 5))
    pts += list(cv2.ellipse2Poly((x2 - radius, y2 - radius), (radius, radius), 0, 0, 90, 5))
    pts += list(cv2.ellipse2Poly((x1 + radius, y2 - radius), (radius, radius), 0, 90, 180, 5))
    pts += list(cv2.ellipse2Poly((x1 + radius, y1 + radius), (radius, radius), 0, 180, 270, 5))
    return np.array(pts, dtype=np.int32)


def draw_rounded_rect(image, pt1, pt2, radius, color, thickness=-1):
    """Rounded rectangle - filled if thickness < 0 (the default), outlined
    otherwise. OpenCV has no native primitive for this."""
    x1, y1 = pt1
    x2, y2 = pt2
    radius = max(0, min(radius, (x2 - x1) // 2, (y2 - y1) // 2))
    pts = _rounded_rect_points((x1, y1), (x2, y2), radius)
    if thickness < 0:
        cv2.fillPoly(image, [pts], color, cv2.LINE_AA)
    else:
        cv2.polylines(image, [pts], True, color, thickness, cv2.LINE_AA)


def draw_status_panel(
    image,
    lines,
    origin=(24, 46),
    line_gap=45,
    padding=18,
    radius=12,
    border_color=(0, 0, 0),
):
    """White rounded-rect backdrop behind stacked lines of black text, for
    readability against a busy camera feed. lines: [(text, font_scale), ...]."""
    font = cv2.FONT_HERSHEY_COMPLEX
    thickness = 2
    origin_x, first_y = origin

    max_w, first_h = 0, 0
    for i, (text, scale) in enumerate(lines):
        (w, h), _ = cv2.getTextSize(text, font, scale, thickness)
        max_w = max(max_w, w)
        if i == 0:
            first_h = h

    top_left = (origin_x - padding, first_y - first_h - padding)
    bottom_right = (
        origin_x + max_w + padding,
        first_y + line_gap * (len(lines) - 1) + padding,
    )
    draw_rounded_rect(image, top_left, bottom_right, radius, (255, 255, 255))
    draw_rounded_rect(image, top_left, bottom_right, radius, border_color, thickness=2)

    for i, (text, scale) in enumerate(lines):
        y = first_y + i * line_gap
        cv2.putText(image, text, (origin_x, y), font, scale, (0, 0, 0), thickness, cv2.LINE_AA)


def draw_status_banner(
    image,
    text,
    center_x,
    max_width,
    y=175,
    max_font_scale=1.3,
    padding=18,
    radius=14,
    border_color=(0, 0, 0),
):
    """Large centered status text on a white rounded-rect backdrop, e.g. top-middle
    of the screen. Shrinks font_scale down from max_font_scale if needed so even
    the longest status string fits within max_width, regardless of camera resolution."""
    font = cv2.FONT_HERSHEY_COMPLEX
    thickness = 3
    (w, h), _ = cv2.getTextSize(text, font, max_font_scale, thickness)
    font_scale = max_font_scale
    if w > max_width:
        font_scale = max_font_scale * max_width / w
        (w, h), _ = cv2.getTextSize(text, font, font_scale, thickness)

    origin_x = center_x - w // 2
    top_left = (origin_x - padding, y - h - padding)
    bottom_right = (origin_x + w + padding, y + padding)
    draw_rounded_rect(image, top_left, bottom_right, radius, (255, 255, 255))
    draw_rounded_rect(image, top_left, bottom_right, radius, border_color, thickness=3)

    cv2.putText(image, text, (origin_x, y), font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)
