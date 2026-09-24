# Thanks, Claude!

import numpy as np


def compute_hand_frame(wrist, index_mcp, pinky_mcp):
    """Orthonormal rotation matrix (columns e1,e2,e3) from 3 triangulated
    points: e1 along wrist->index_mcp, e3 the palm normal, e2 completing a
    right-handed frame. wrist/index_mcp/pinky_mcp are 3D points (meters)."""
    v1 = index_mcp - wrist
    v2 = pinky_mcp - wrist
    e1 = v1 / np.linalg.norm(v1)
    e3 = np.cross(v1, v2)
    e3 = e3 / np.linalg.norm(e3)
    e2 = np.cross(e3, e1)
    return np.column_stack([e1, e2, e3])


def average_rotations(rotations):
    """Chordal-mean average of nearby rotation matrices: mean + re-orthonormalize
    via SVD. Valid for a set of rotations spanning a small angle (e.g. samples
    from one held-still hand pose), which is the only case this is used for."""
    mean = np.mean(rotations, axis=0)
    u, _, vt = np.linalg.svd(mean)
    r = u @ vt
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = u @ vt
    return r


def compute_heading(wrist, index_mcp, pinky_mcp):
    """Direction the hand faces (roughly along the fingers): the bisector of
    wrist->index_mcp and wrist->pinky_mcp. Distinct from compute_hand_frame's
    e1/e2/e3 - those are chosen to form an orthonormal frame for tracking
    roll/pitch, not to point along the hand's length."""
    heading = (index_mcp - wrist) + (pinky_mcp - wrist)
    return heading / np.linalg.norm(heading)


def signed_angle_about_axis(a, b, axis):
    """Signed angle (radians) from unit vector a to unit vector b, measured
    in the plane perpendicular to the given unit axis (right-hand rule).
    a and b must already lie in that plane (project out any component along
    axis first) - otherwise dot(a, b) mixes in their out-of-plane agreement
    and the result isn't a meaningful planar angle."""
    cross = np.cross(a, b)
    return np.arctan2(np.dot(cross, axis), np.dot(a, b))
