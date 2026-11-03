import math


def quaternion_to_yaw(q) -> float:
    """Extracts yaw (rotation around Z axis in radians) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """Normalizes an angle in radians to the range [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def euclidean_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Computes 2D Euclidean distance between two points."""
    return math.hypot(x2 - x1, y2 - y1)


def cross_track_error(px: float, py: float, line_y: float = 0.0) -> float:
    """
    Computes cross-track error of point (px, py) relative to a horizontal centerline at y = line_y.
    Positive indicates the robot is left of the centerline (py > line_y).
    """
    return py - line_y
