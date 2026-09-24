import json

import numpy as np


def load_stereo_calibration(path):
    """R, T map a point in camera_a's frame into camera_b's frame: X_b = R @ X_a + T."""
    with open(path, "r") as f:
        data = json.load(f)

    def cam(block):
        return {
            "image_size": tuple(block["image_size"]),
            "camera_matrix": np.array(block["camera_matrix"]),
            "dist_coeffs": np.array(block["dist_coeffs"]),
        }

    return {
        "camera_a": cam(data["camera_a"]),
        "camera_b": cam(data["camera_b"]),
        "R": np.array(data["R"]),
        "T": np.array(data["T"]),
    }
