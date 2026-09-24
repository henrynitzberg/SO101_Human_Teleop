from contextlib import contextmanager

import cv2


@contextmanager
def open_camera_pair(cam_a_idx, cam_b_idx):
    """Open two cv2.VideoCapture devices, raising if either fails to open."""
    cap_a = cv2.VideoCapture(cam_a_idx)
    cap_b = cv2.VideoCapture(cam_b_idx)
    if not cap_a.isOpened() or not cap_b.isOpened():
        cap_a.release()
        cap_b.release()
        raise RuntimeError(
            "Could not open both cameras. Check --cam-a/--cam-b indices."
        )
    try:
        yield cap_a, cap_b
    finally:
        cap_a.release()
        cap_b.release()
        cv2.destroyAllWindows()
