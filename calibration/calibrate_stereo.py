import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lib.vision.camera_pair import open_camera_pair
from lib.vision.cli_args import add_camera_args
from lib.vision.status_ui import combine_side_by_side, draw_status_panel, draw_status_banner

MIN_CAPTURES_FOR_LIVE_ERROR = 4


def parse_args():
    parser = argparse.ArgumentParser(
        description="Stereo-calibrate two cameras against a checkerboard (see generate_checkerboard.py)."
    )
    add_camera_args(parser)
    parser.add_argument(
        "--board-cols", type=int, default=9, help="Inner corner columns."
    )
    parser.add_argument("--board-rows", type=int, default=6, help="Inner corner rows.")
    parser.add_argument(
        "--square-size",
        type=float,
        default=0.02395,
        help="Checkerboard square size in meters (matches generate_checkerboard.py's default board).",
    )
    parser.add_argument("--min-captures", type=int, default=10)
    parser.add_argument("--max-captures", type=int, default=40)
    parser.add_argument(
        "--stability-window",
        type=float,
        default=0.1,
        help="Seconds the board must be held still before it auto-captures.",
    )
    parser.add_argument(
        "--stability-threshold-px",
        type=float,
        default=1.5,
        help="Max mean corner movement (pixels) between frames to count as 'still'.",
    )
    parser.add_argument(
        "--diversity-threshold",
        type=float,
        default=0.05,
        help="Min normalized difference (position or scale) from every previous capture to accept a new one.",
    )
    parser.add_argument("--out", type=str, default="calibration/data/stereo_calib.json")
    return parser.parse_args()


def find_corners(gray, board_size):
    found, corners = cv2.findChessboardCorners(
        gray, board_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def object_points(board_size, square_size):
    cols, rows = board_size
    pts = np.zeros((cols * rows, 3), np.float32)
    pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size
    return pts


def mean_corner_displacement(corners_a, corners_b):
    """Mean per-corner pixel movement between two same-shape corner sets."""
    return float(
        np.mean(
            np.linalg.norm(corners_a.reshape(-1, 2) - corners_b.reshape(-1, 2), axis=1)
        )
    )


def pose_signature(corners, image_width):
    """Cheap proxy for "where/how big" a detected board is: centroid + bounding-box
    diagonal, normalized by image width so the diversity threshold is resolution-independent.
    """
    pts = corners.reshape(-1, 2)
    centroid = pts.mean(axis=0) / image_width
    diag = float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0))) / image_width
    return centroid, diag


def pose_distance(sig_a, sig_b):
    centroid_a, diag_a = sig_a
    centroid_b, diag_b = sig_b
    centroid_dist = float(np.linalg.norm(centroid_a - centroid_b))
    scale_dist = abs(diag_a - diag_b) / max(diag_a, diag_b)
    return max(centroid_dist, scale_dist)


def is_diverse_enough(candidate_sig_a, candidate_sig_b, previous_sigs, threshold):
    """A pose is 'diverse' enough to be incorporated into the transform calculation
        if either it is a far enough distance away from other poses,
        or it is angled differently from other poses. """
    return all(
        max(
            pose_distance(candidate_sig_a, prev_sig_a),
            pose_distance(candidate_sig_b, prev_sig_b),
        )
        > threshold
        for prev_sig_a, prev_sig_b in previous_sigs
    )


def board_outline(corners, board_size):
    """Board corners in perimeter order. """
    cols, rows = board_size
    grid = corners.reshape(rows, cols, 2)
    return np.array(
        [grid[0, 0], grid[0, -1], grid[-1, -1], grid[-1, 0]], dtype=np.int32
    )


def draw_detected_board(image, corners, board_size, color=(100, 255, 100)):
    """Plain outline + a dot at every corner for the currently-detected
    board, instead of cv2.drawChessboardCorners' rainbow-by-row coloring."""
    outline = board_outline(corners, board_size)
    cv2.polylines(image, [outline], True, color, 2, cv2.LINE_AA)
    for x, y in corners.reshape(-1, 2):
        cv2.circle(image, (int(x), int(y)), 4, color, -1, cv2.LINE_AA)


def run_calibration(object_pts, img_pts_a, img_pts_b, image_size_a, image_size_b):
    err_a, mtx_a, dist_a, _, _ = cv2.calibrateCamera(
        object_pts, img_pts_a, image_size_a, None, None
    )
    err_b, mtx_b, dist_b, _, _ = cv2.calibrateCamera(
        object_pts, img_pts_b, image_size_b, None, None
    )

    # Solves only for rotation and transformation.
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
    err_stereo, mtx_a, dist_a, mtx_b, dist_b, R, T, _, _ = cv2.stereoCalibrate(
        object_pts,
        img_pts_a,
        img_pts_b,
        mtx_a,
        dist_a,
        mtx_b,
        dist_b,
        image_size_a,
        criteria=criteria,
        flags=cv2.CALIB_FIX_INTRINSIC,
    )
    return {
        "err_a": err_a,
        "err_b": err_b,
        "err_stereo": err_stereo,
        "mtx_a": mtx_a,
        "dist_a": dist_a,
        "mtx_b": mtx_b,
        "dist_b": dist_b,
        "R": R,
        "T": T,
    }


def auto_capture_pairs(cap_a, cap_b, board_size, obj_pts_template, args):
    object_pts, img_pts_a, img_pts_b, pose_sigs = [], [], [], []
    capture_outlines_a, capture_outlines_b = [], []
    image_size_a = image_size_b = None
    prev_corners_a = prev_corners_b = None
    still_start = None
    awaiting_movement = False
    live_result = None
    live_result_count = 0

    print(
        "Show the checkerboard to both cameras and hold it still - it captures "
        f"automatically. Move it to a new position/angle/depth between captures "
        f"(aim for {args.min_captures}+ varied captures). 'q'/ENTER finishes and "
        "calibrates. ESC aborts immediately and discards everything captured so far."
    )

    while True:
        ok_a, frame_a = cap_a.read()
        ok_b, frame_b = cap_b.read()
        if not ok_a or not ok_b:
            print("Failed to read from a camera.")
            break

        gray_a = cv2.cvtColor(frame_a, cv2.COLOR_BGR2GRAY)
        gray_b = cv2.cvtColor(frame_b, cv2.COLOR_BGR2GRAY)
        image_size_a = gray_a.shape[::-1]
        image_size_b = gray_b.shape[::-1]

        corners_a = find_corners(gray_a, board_size)
        corners_b = find_corners(gray_b, board_size)

        status = "Show the checkerboard to both cameras"
        if corners_a is not None and corners_b is not None:
            if prev_corners_a is not None and prev_corners_b is not None:
                displacement = max(
                    mean_corner_displacement(corners_a, prev_corners_a),
                    mean_corner_displacement(corners_b, prev_corners_b),
                )
                still = displacement < args.stability_threshold_px
            else:
                still = False

            if still:
                if still_start is None:
                    still_start = time.perf_counter()
                held_for = time.perf_counter() - still_start

                if awaiting_movement:
                    status = "Captured - move to a new pose"
                elif held_for < args.stability_window:
                    status = (
                        f"Hold still... ({held_for:.1f}s/{args.stability_window:.1f}s)"
                    )
                else:
                    candidate_sig_a = pose_signature(corners_a, image_size_a[0])
                    candidate_sig_b = pose_signature(corners_b, image_size_b[0])
                    if is_diverse_enough(
                        candidate_sig_a,
                        candidate_sig_b,
                        pose_sigs,
                        args.diversity_threshold,
                    ):
                        object_pts.append(obj_pts_template.copy())
                        img_pts_a.append(corners_a)
                        img_pts_b.append(corners_b)
                        pose_sigs.append((candidate_sig_a, candidate_sig_b))
                        capture_outlines_a.append(board_outline(corners_a, board_size))
                        capture_outlines_b.append(board_outline(corners_b, board_size))
                        awaiting_movement = True
                        print(f"Captured pair {len(object_pts)}")

                        if len(object_pts) >= MIN_CAPTURES_FOR_LIVE_ERROR:
                            live_result = run_calibration(
                                object_pts,
                                img_pts_a,
                                img_pts_b,
                                image_size_a,
                                image_size_b,
                            )
                            live_result_count = len(object_pts)

                        status = "Captured - move to a new pose"
                        if len(object_pts) >= args.max_captures:
                            break
                    else:
                        status = "Too similar to a previous capture - move further"
            else:
                still_start = None
                awaiting_movement = False
        else:
            still_start = None
            awaiting_movement = False

        prev_corners_a, prev_corners_b = corners_a, corners_b

        preview_a, preview_b = frame_a.copy(), frame_b.copy()
        for outline in capture_outlines_a:
            cv2.polylines(preview_a, [outline], True, (255, 100, 255), 2, cv2.LINE_AA)
        for outline in capture_outlines_b:
            cv2.polylines(preview_b, [outline], True, (255, 100, 255), 2, cv2.LINE_AA)
        if corners_a is not None:
            draw_detected_board(preview_a, corners_a, board_size)
        if corners_b is not None:
            draw_detected_board(preview_b, corners_b, board_size)
        combined = combine_side_by_side(preview_a, preview_b)

        error_text = (
            f"error: {live_result['err_stereo']:.3f}px"
            if live_result is not None
            else "error: --"
        )
        border_color = (
            (0, 0, 255) if len(object_pts) < args.min_captures else (0, 255, 0)
        )
        draw_status_panel(
            combined,
            [
                (
                    f"captures: {len(object_pts)}/{args.min_captures}+   {error_text}",
                    1.0,
                ),
                ("q/ENTER: finish   ESC: abort", 0.8),
            ],
            border_color=border_color,
        )
        draw_status_banner(
            combined,
            status,
            center_x=combined.shape[1] // 2,
            max_width=combined.shape[1] * 0.9,
            border_color=border_color,
        )
        cv2.imshow("Stereo calibration", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            print("Aborted.")
            return None
        if key in (ord("q"), 13):
            break

    return (
        object_pts,
        img_pts_a,
        img_pts_b,
        image_size_a,
        image_size_b,
        live_result,
        live_result_count,
    )


def main():
    args = parse_args()
    board_size = (args.board_cols, args.board_rows)
    obj_pts_template = object_points(board_size, args.square_size)

    with open_camera_pair(args.cam_a, args.cam_b) as (cap_a, cap_b):
        captured = auto_capture_pairs(cap_a, cap_b, board_size, obj_pts_template, args)

    if captured is None:
        return
    (
        object_pts,
        img_pts_a,
        img_pts_b,
        image_size_a,
        image_size_b,
        live_result,
        live_result_count,
    ) = captured

    if len(object_pts) < args.min_captures:
        print(
            f"Only captured {len(object_pts)} pair(s), need at least {args.min_captures} - "
            "nothing written. Run again and capture more before quitting."
        )
        return

    if live_result is not None and live_result_count == len(object_pts):
        result_data = live_result
    else:
        print("Calibrating...")
        result_data = run_calibration(
            object_pts, img_pts_a, img_pts_b, image_size_a, image_size_b
        )

    print(f"camera A reprojection error: {result_data['err_a']:.3f}px")
    print(f"camera B reprojection error: {result_data['err_b']:.3f}px")
    print(f"stereo reprojection error: {result_data['err_stereo']:.3f}px")

    result = {
        "camera_a": {
            "image_size": list(image_size_a),
            "camera_matrix": result_data["mtx_a"].tolist(),
            "dist_coeffs": result_data["dist_a"].flatten().tolist(),
        },
        "camera_b": {
            "image_size": list(image_size_b),
            "camera_matrix": result_data["mtx_b"].tolist(),
            "dist_coeffs": result_data["dist_b"].flatten().tolist(),
        },
        "R": result_data["R"].tolist(),
        "T": result_data["T"].flatten().tolist(),
        "reprojection_error_px": result_data["err_stereo"],
    }

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(result, f, indent=4)
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
