import math

import numpy as np

from follow_helpers import HandLandmark, draw_curved_arrow, get_landmark_point
from lerobot.utils.rotation import Rotation
from lib.angle_calc.hand_frame import average_rotations, compute_hand_frame
from lib.calibration.record_two_samples import record_two_samples
from lib.vision.triangulate import triangulate_point

MIN_ROTATION_DEG = 10.0
ARC_COLOR = (0, 0, 255)


def _take_sample(result_a, w_a, h_a, result_b, w_b, h_b, calib):
    points_3d = {}
    for label, index in (
        ("wrist", HandLandmark.WRIST),
        ("index_mcp", HandLandmark.INDEX_FINGER_MCP),
        ("pinky_mcp", HandLandmark.PINKY_MCP),
    ):
        point_a = get_landmark_point(result_a, index, w_a, h_a)
        point_b = get_landmark_point(result_b, index, w_b, h_b)
        if point_a is None or point_b is None:
            return None
        points_3d[label] = triangulate_point(calib, point_a, point_b)
    return compute_hand_frame(points_3d["wrist"], points_3d["index_mcp"], points_3d["pinky_mcp"])


def _heading_angle_deg(ghost_points):
    """Angle (degrees, cv2.ellipse convention) of this snapshot's own
    wrist -> middle-finger-MCP direction, in its own pixel coordinates."""
    wrist = ghost_points[HandLandmark.WRIST]
    middle_mcp = ghost_points[9]
    return math.degrees(math.atan2(middle_mcp[1] - wrist[1], middle_mcp[0] - wrist[0]))


def _wrapped_delta_deg(start_deg, end_deg):
    """Signed difference start->end, wrapped to (-180, 180] (shortest way around)."""
    return (end_deg - start_deg + 180) % 360 - 180


def _draw_connector(preview_a, preview_b, ghost_1, ghost_2):
    for before, after, preview in (
        (ghost_1[0], ghost_2[0], preview_a),
        (ghost_1[1], ghost_2[1], preview_b),
    ):
        center = before[HandLandmark.WRIST]
        radius = np.linalg.norm(np.array(before[9]) - np.array(before[HandLandmark.WRIST])) * 1.3
        angle_start = _heading_angle_deg(before)
        angle_end = angle_start + _wrapped_delta_deg(angle_start, _heading_angle_deg(after))
        draw_curved_arrow(preview, center, radius, angle_start, angle_end, ARC_COLOR, 3)


def _finalize(rotation_before, rotation_after):
    r_delta = rotation_after @ rotation_before.T
    rotvec = Rotation.from_matrix(r_delta).as_rotvec()
    angle_deg = math.degrees(float(np.linalg.norm(rotvec)))
    print(f"rotation: {angle_deg:.1f} deg")

    if angle_deg < MIN_ROTATION_DEG:
        print(
            f"Rotation is only {angle_deg:.1f} deg - too small for a reliable axis. "
            "Not saved; rerun with a larger rotation between the two holds."
        )
        return None

    axis = rotvec / np.linalg.norm(rotvec)
    print(f"axis: x={axis[0]:+.4f} y={axis[1]:+.4f} z={axis[2]:+.4f}")
    return {
        "axis": axis.tolist(),
        "angle_deg": angle_deg,
        "rotvec": rotvec.tolist(),
        "rotation_before": rotation_before.tolist(),
        "rotation_after": rotation_after.tolist(),
    }


def record_rotation_vector(name, args):
    """Record two hand orientations and derive a rotation axis (before ->
    after), saved under `name` in args.out. SPACE holds+records an
    orientation (up to 2). ENTER saves the pair once both are recorded.
    'c' clears both so you can start over. ESC aborts without saving."""
    record_two_samples(
        name,
        args,
        window_name="Record rotation vector",
        intro=(
            f"Recording rotation vector '{name}'. SPACE holds still for {args.hold_seconds:.1f}s and "
            "records 'before', then rotate your hand about the axis you care about and do the same "
            "for 'after' - a larger rotation makes the resulting axis less sensitive to tracking "
            "noise. Keep your palm facing roughly toward the cameras so wrist/index/pinky stay "
            "visible. ENTER saves the pair. 'c' clears both to start over. ESC aborts."
        ),
        first_label="'before'",
        second_label="'after'",
        noun="rotation",
        take_sample=_take_sample,
        average_samples=average_rotations,
        draw_connector=_draw_connector,
        finalize=_finalize,
    )
