import math

import numpy as np
from lib.angle_calc.calculate_angle import calculate_angle
import cv2


class HandLandmark:
    """Landmark indices for MediaPipe's 21-point hand model (fixed by the model spec)."""

    WRIST = 0
    THUMB_TIP = 4
    INDEX_FINGER_MCP = 5
    MIDDLE_FINGER_TIP = 12
    PINKY_MCP = 17


def _landmark_point(hand_landmarks, index, w, h):
    """
    Convert a normalized MediaPipe landmark to pixel-scale (x, y, z). z is
    scaled by w since MediaPipe emits it in roughly the same normalized
    units as x.
    """
    lm = hand_landmarks[index]
    return np.array([lm.x * w, lm.y * h, lm.z * w])


def get_wrist_point(hand_detection_result, w, h):
    """Return the wrist landmark's pixel (u, v), or None if no hand detected
    this frame. Deliberately ignores MediaPipe's per-landmark z - that's a
    relative monocular guess, not metric."""
    hand_landmarks_list = hand_detection_result.hand_landmarks
    if not hand_landmarks_list:
        return None
    lm = hand_landmarks_list[0][HandLandmark.WRIST]
    return (lm.x * w, lm.y * h)


def get_landmark_point(hand_detection_result, index, w, h):
    """Pixel (u, v) for the given landmark index, or None if no hand detected."""
    hand_landmarks_list = hand_detection_result.hand_landmarks
    if not hand_landmarks_list:
        return None
    lm = hand_landmarks_list[0][index]
    return (lm.x * w, lm.y * h)


# MediaPipe's standard 21-point hand topology: palm + one chain per finger,
# fixed by the model spec (mediapipe.python.solutions.hands_connections).
HAND_CONNECTIONS = (
    (0, 1),
    (0, 5),
    (9, 13),
    (13, 17),
    (5, 9),
    (0, 17),  # palm
    (1, 2),
    (2, 3),
    (3, 4),  # thumb
    (5, 6),
    (6, 7),
    (7, 8),  # index
    (9, 10),
    (10, 11),
    (11, 12),  # middle
    (13, 14),
    (14, 15),
    (15, 16),  # ring
    (17, 18),
    (18, 19),
    (19, 20),  # pinky
)


def hand_landmark_pixels(hand_detection_result, w, h):
    """All 21 landmark pixel positions (int (x, y) tuples) for the detected
    hand, or None if no hand detected this frame."""
    hand_landmarks_list = hand_detection_result.hand_landmarks
    if not hand_landmarks_list:
        return None
    return [(int(lm.x * w), int(lm.y * h)) for lm in hand_landmarks_list[0]]


def draw_landmark_points(image, points, color, radius=3, thickness=2):
    """Skeleton (HAND_CONNECTIONS lines + a dot per point) from already-extracted
    pixel points - the ghost-drawing primitive, reused by draw_hand_skeleton for
    the live case and directly for a stored snapshot ("ghost") of a past frame."""
    for i, j in HAND_CONNECTIONS:
        cv2.line(image, points[i], points[j], color, thickness, cv2.LINE_AA)
    for pt in points:
        cv2.circle(image, pt, radius, color, -1, cv2.LINE_AA)
    return image


def draw_hand_skeleton(image, hand_detection_result, w, h, color=(255, 255, 255)):
    """Full 21-point hand skeleton for the live detection this frame, in a
    single flat color. No-op if no hand is detected."""
    points = hand_landmark_pixels(hand_detection_result, w, h)
    if points is None:
        return image
    return draw_landmark_points(image, points, color)


def int_pt(pt):
    return (int(pt[0]), int(pt[1]))


def draw_curved_arrow(
    image, center, radius, angle_start_deg, angle_end_deg, color, thickness=3
):
    """Arc from angle_start_deg to angle_end_deg with a small arrowhead at
    the end pointing in the sweep direction - shows a rotation rather than a
    straight-line displacement. Angle convention matches cv2.ellipse: 0 deg
    is the +x axis, increasing clockwise in image coordinates."""
    center_i = int_pt(center)
    radius_i = max(1, int(radius))
    cv2.ellipse(
        image,
        center_i,
        (radius_i, radius_i),
        0,
        angle_start_deg,
        angle_end_deg,
        color,
        thickness,
        cv2.LINE_AA,
    )

    direction = 1 if angle_end_deg >= angle_start_deg else -1
    near_end_deg = angle_end_deg - direction * 8
    end_rad = math.radians(angle_end_deg)
    near_rad = math.radians(near_end_deg)
    end_point = (
        int(center_i[0] + radius_i * math.cos(end_rad)),
        int(center_i[1] + radius_i * math.sin(end_rad)),
    )
    near_point = (
        int(center_i[0] + radius_i * math.cos(near_rad)),
        int(center_i[1] + radius_i * math.sin(near_rad)),
    )
    cv2.arrowedLine(
        image, near_point, end_point, color, thickness, cv2.LINE_AA, tipLength=0.8
    )
    return image


def _hand_grip_points(hand_landmarks, w, h):
    wrist = _landmark_point(hand_landmarks, HandLandmark.WRIST, w, h)
    thumb_tip = _landmark_point(hand_landmarks, HandLandmark.THUMB_TIP, w, h)
    middle_tip = _landmark_point(hand_landmarks, HandLandmark.MIDDLE_FINGER_TIP, w, h)
    return wrist, thumb_tip, middle_tip


def angle_to_pos(angle, min_deg, max_deg, min_pos, max_pos):
    """Convert an angle in degrees to a position in the range [min_pos, max_pos]."""
    # Clamp the angle to the defined range
    clamped_angle = max(min(angle, max_deg), min_deg)

    # Normalize the angle to a value between 0 and 1
    normalized_position = (clamped_angle - min_deg) / (max_deg - min_deg)

    return normalized_position * (max_pos - min_pos) + min_pos


class GripAngleTracker:
    """Tracks the last-known grip angle for one camera's hand stream.

    One instance per camera: sharing a single "last known angle" across two
    concurrent camera threads would leak one camera's fallback value into the
    other's and race on the write.
    """

    def __init__(self):
        self.last = None

    def update(self, hand_detection_result, w, h):
        """Return the grip angle for the (single) detected hand, or the last known angle if not detected this frame."""
        hand_landmarks_list = hand_detection_result.hand_landmarks
        if not hand_landmarks_list:
            return self.last

        wrist, thumb_tip, middle_tip = _hand_grip_points(hand_landmarks_list[0], w, h)
        self.last = calculate_angle(thumb_tip, wrist, middle_tip)
        return self.last
