import time

import cv2

from follow_helpers import draw_hand_skeleton, draw_landmark_points, hand_landmark_pixels
from lib.calibration.capture_ui import read_and_detect, save_named_result
from lib.vision.camera_pair import open_camera_pair
from lib.vision.cli_args import add_calibration_arg, add_camera_args
from lib.vision.get_detector import get_hand_detector
from lib.vision.load_calibration import load_stereo_calibration
from lib.vision.status_ui import combine_side_by_side, draw_status_banner, draw_status_panel

MIN_SAMPLES = 5
HINT_TEXT = "SPACE: record   ENTER: save   c: clear   ESC: abort"


def add_common_args(parser):
    """CLI args shared by the record_*_base_vec.py wrapper scripts."""
    add_camera_args(parser)
    add_calibration_arg(parser)
    parser.add_argument("--out", type=str, default="calibration/data/basis_vectors.json")
    parser.add_argument("--hold-seconds", type=float, default=0.5)
    return parser


def _status_text(recorded, holding, sample_count, first_label, second_label, noun):
    if holding:
        return f"Hold still... ({sample_count} samples)"
    if len(recorded) == 0:
        return f"SPACE to record {first_label}"
    if len(recorded) == 1:
        return f"SPACE to record {second_label}"
    return f"ENTER to save this {noun}, or 'c' to clear and start over"


def _draw_frame(window_name, frame_a, frame_b, result_a, result_b, ghosts, status, draw_connector):
    """Live white hand skeleton, magenta ghost skeletons for any already-recorded
    samples, a connector between them (drawn by draw_connector) once both are
    recorded, and the status panel/banner - then shows the combined preview."""
    h_a, w_a = frame_a.shape[:2]
    h_b, w_b = frame_b.shape[:2]
    preview_a, preview_b = frame_a.copy(), frame_b.copy()

    for ghost in ghosts:
        if ghost is not None:
            ghost_a, ghost_b = ghost
            draw_landmark_points(preview_a, ghost_a, (255, 100, 255))
            draw_landmark_points(preview_b, ghost_b, (255, 100, 255))

    draw_hand_skeleton(preview_a, result_a, w_a, h_a, (255, 255, 255))
    draw_hand_skeleton(preview_b, result_b, w_b, h_b, (255, 255, 255))

    if ghosts[0] is not None and ghosts[1] is not None:
        draw_connector(preview_a, preview_b, ghosts[0], ghosts[1])

    combined = combine_side_by_side(preview_a, preview_b)
    draw_status_panel(combined, [(HINT_TEXT, 0.9)])
    draw_status_banner(combined, status, center_x=combined.shape[1] // 2, max_width=combined.shape[1] * 0.9)
    cv2.imshow(window_name, combined)


def record_two_samples(
    name,
    args,
    *,
    window_name,
    intro,
    first_label,
    second_label,
    noun,
    take_sample,
    average_samples,
    draw_connector,
    finalize,
):
    """Generic 'record two hand samples, derive something from before ->
    after' driver shared by record_basis_vector.py (two points -> a unit
    vector) and record_rotation_vector.py (two orientations -> a rotation
    axis). SPACE holds+records a sample (up to 2, via `take_sample` each
    frame while held, averaged by `average_samples` once the hold ends).
    ENTER saves the pair once both are recorded - `finalize(value_1,
    value_2)` does the domain-specific validation/math and either returns
    the dict to save or None to abort without saving. 'c' clears both so you
    can start over. ESC aborts without saving."""
    calib = load_stereo_calibration(args.calibration)

    with open_camera_pair(args.cam_a, args.cam_b) as (cap_a, cap_b):
        hand_detector_a = get_hand_detector()
        hand_detector_b = get_hand_detector()

        print(intro)

        recorded = []  # oldest -> newest, up to 2 entries of (value, ghost)
        holding = False
        hold_end = 0.0
        samples = []
        last_ghost = None

        while True:
            frames = read_and_detect(cap_a, cap_b, hand_detector_a, hand_detector_b)
            if frames is None:
                print("Failed to read from a camera.")
                return
            frame_a, frame_b, result_a, result_b = frames
            h_a, w_a = frame_a.shape[:2]
            h_b, w_b = frame_b.shape[:2]

            if holding:
                sample = take_sample(result_a, w_a, h_a, result_b, w_b, h_b, calib)
                if sample is not None:
                    samples.append(sample)
                    last_ghost = (
                        hand_landmark_pixels(result_a, w_a, h_a),
                        hand_landmark_pixels(result_b, w_b, h_b),
                    )
                if time.perf_counter() >= hold_end:
                    holding = False
                    if len(samples) < MIN_SAMPLES:
                        print(f"Only {len(samples)} valid sample(s) - too few, retry.")
                    else:
                        averaged = average_samples(samples)
                        recorded.append((averaged, last_ghost))
                        print(f"Recorded ({len(samples)} samples) - {len(recorded)}/2 in the current pair.")
                    samples = []

            status = _status_text(recorded, holding, len(samples), first_label, second_label, noun)
            ghosts = [r[1] for r in recorded] + [None] * (2 - len(recorded))
            _draw_frame(window_name, frame_a, frame_b, result_a, result_b, ghosts, status, draw_connector)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                print("Aborted.")
                return
            if key == ord("c"):
                recorded = []
                holding = False
                samples = []
            elif key == 13:  # ENTER
                if len(recorded) == 2:
                    break
                print(f"Need 2 recorded {noun}s before saving - record more first.")
            elif key == ord(" ") and not holding and len(recorded) < 2:
                holding = True
                hold_end = time.perf_counter() + args.hold_seconds
                samples = []

    value_1, value_2 = recorded[0][0], recorded[1][0]
    result_data = finalize(value_1, value_2)
    if result_data is None:
        return
    save_named_result(args.out, name, result_data)
