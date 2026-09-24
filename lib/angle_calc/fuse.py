def fuse_grip_angles(
    angle_a, age_a, angle_b, age_b, last_fused, max_age=0.5, max_disagreement=25.0
):
    """Combine two per-camera grip angle readings into one.
    """
    a_ok = angle_a is not None and age_a < max_age
    b_ok = angle_b is not None and age_b < max_age

    if a_ok and b_ok:
        if abs(angle_a - angle_b) <= max_disagreement:
            return (angle_a + angle_b) / 2
        return last_fused
    if a_ok:
        return angle_a
    if b_ok:
        return angle_b
    return last_fused
