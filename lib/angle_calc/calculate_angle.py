import numpy as np

def calculate_angle(a, b, c):
    """
    Calculate the angle between three points (a, b, c) in degrees.
    The angle is at point b formed by the line segments ab and bc.
    """
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)

    # Calculate the vectors
    ba = a - b
    bc = c - b

    # Calculate the cosine of the angle using the dot product formula
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))

    # Ensure the cosine value is within the valid range [-1, 1] to avoid numerical issues
    cosine_angle = np.clip(cosine_angle, -1.0, 1.0)

    # Calculate the angle in radians and then convert to degrees
    angle = np.arccos(cosine_angle)
    return np.degrees(angle)