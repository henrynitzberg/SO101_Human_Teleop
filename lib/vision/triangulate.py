import cv2
import numpy as np


def triangulate_point(calib, pt_a_px, pt_b_px):
    """Triangulate a landmark seen at pixel pt_a_px in camera A and pt_b_px
    in camera B into a 3D point in camera A's coordinate frame (meters)."""
    cam_a, cam_b = calib["camera_a"], calib["camera_b"]

    norm_a = cv2.undistortPoints(
        np.array([[pt_a_px]], dtype=np.float64), cam_a["camera_matrix"], cam_a["dist_coeffs"]
    )
    norm_b = cv2.undistortPoints(
        np.array([[pt_b_px]], dtype=np.float64), cam_b["camera_matrix"], cam_b["dist_coeffs"]
    )

    # P_a = [I|0] puts camera A's optical center at the origin; R/T (camera
    # A's frame -> camera B's frame) then place camera B relative to it.
    P_a = np.hstack([np.eye(3), np.zeros((3, 1))])
    P_b = np.hstack([calib["R"], calib["T"].reshape(3, 1)])

    point_4d = cv2.triangulatePoints(P_a, P_b, norm_a.reshape(2, 1), norm_b.reshape(2, 1))
    return (point_4d[:3] / point_4d[3]).flatten()
