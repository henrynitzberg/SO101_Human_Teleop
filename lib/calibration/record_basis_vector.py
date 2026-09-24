import cv2
import numpy as np

from follow_helpers import HandLandmark, get_wrist_point
from lib.calibration.record_two_samples import record_two_samples
from lib.vision.triangulate import triangulate_point

MIN_BASELINE_M = 0.02


def _take_sample(result_a, w_a, h_a, result_b, w_b, h_b, calib):
    wrist_a = get_wrist_point(result_a, w_a, h_a)
    wrist_b = get_wrist_point(result_b, w_b, h_b)
    if wrist_a is None or wrist_b is None:
        return None
    return triangulate_point(calib, wrist_a, wrist_b)


def _average(samples):
    return np.mean(samples, axis=0)


def _draw_connector(preview_a, preview_b, ghost_1, ghost_2):
    for ghost_before, ghost_after, preview in (
        (ghost_1[0], ghost_2[0], preview_a),
        (ghost_1[1], ghost_2[1], preview_b),
    ):
        cv2.arrowedLine(
            preview,
            ghost_before[HandLandmark.WRIST],
            ghost_after[HandLandmark.WRIST],
            (0, 0, 255),
            3,
            cv2.LINE_AA,
            tipLength=0.08,
        )


def _finalize(point_1, point_2):
    raw_delta = point_2 - point_1
    norm = float(np.linalg.norm(raw_delta))
    print(
        f"raw delta (m): x={raw_delta[0]:+.3f} y={raw_delta[1]:+.3f} z={raw_delta[2]:+.3f} "
        f"(norm={norm:.3f}m)"
    )

    if norm < MIN_BASELINE_M:
        print(
            f"Points are only {norm * 100:.1f}cm apart - too close for a reliable direction. "
            "Not saved; rerun with the two points farther apart."
        )
        return None

    unit = raw_delta / norm
    print(f"unit vector: x={unit[0]:+.4f} y={unit[1]:+.4f} z={unit[2]:+.4f}")
    return {
        "unit": unit.tolist(),
        "raw_delta": raw_delta.tolist(),
        "point_1": point_1.tolist(),
        "point_2": point_2.tolist(),
    }


def record_basis_vector(name, args):
    """Record two hand positions and derive a unit basis vector (point 1 ->
    point 2), saved under `name` in args.out. SPACE holds+records a point
    (up to 2). ENTER saves the pair once both are recorded. 'c' clears both
    so you can start over. ESC aborts without saving."""
    record_two_samples(
        name,
        args,
        window_name="Record basis vector",
        intro=(
            f"Recording basis vector '{name}'. SPACE holds still for {args.hold_seconds:.1f}s "
            "and records a point, then the same for a second point - a longer baseline between "
            "the two makes the direction less sensitive to tracking noise. ENTER saves the pair. "
            "'c' clears both to start over. ESC aborts."
        ),
        first_label="the first point",
        second_label="the second point",
        noun="vector",
        take_sample=_take_sample,
        average_samples=_average,
        draw_connector=_draw_connector,
        finalize=_finalize,
    )
