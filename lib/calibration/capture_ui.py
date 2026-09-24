import json
import os

from lib.vision.get_detector import detect_hand


def read_and_detect(cap_a, cap_b, hand_detector_a, hand_detector_b):
    """One frame from each camera, hand-detected. Returns (frame_a, frame_b,
    result_a, result_b), or None if either camera read fails."""
    ok_a, frame_a = cap_a.read()
    ok_b, frame_b = cap_b.read()
    if not ok_a or not ok_b:
        return None

    result_a = detect_hand(frame_a, hand_detector_a)
    result_b = detect_hand(frame_b, hand_detector_b)
    return frame_a, frame_b, result_a, result_b


def save_named_result(out_path, name, data):
    """Read-modify-write out_path's JSON, merging data under data[name]."""
    merged = {}
    if os.path.exists(out_path):
        with open(out_path, "r") as f:
            merged = json.load(f)
    merged[name] = data
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(merged, f, indent=4)
    print(f"Saved '{name}' to {out_path}")
