import numpy as np
from nav_msgs.msg import Path
from ros_monitors_functions.utils import register_monitor


@register_monitor("average_curvature")
def average_curvature(_, path: Path) -> float:
    poses = path.poses
    if len(poses) < 2:
        return 0.0

    total_normalised_change = 0.0
    total_heading_change = 0.0
    total_length = 0.0

    for i in range(1, len(poses)):
        p0 = poses[i - 1].pose.position
        p1 = poses[i].pose.position

        dx = p1.x - p0.x
        dy = p1.y - p0.y

        segment_length = np.hypot(dx, dy)
        # Heading derived from position deltas, not pose quaternion, for robustness
        segment_heading = np.arctan2(dy, dx)

        if i == 1:
            prev_heading = segment_heading
        
        heading_change = abs(np.arctan2(
            np.sin(segment_heading - prev_heading),
            np.cos(segment_heading - prev_heading)
        ))

        # Normalise heading change to [0, 1] where 1 = complete reversal (π rad)
        normalised_change = heading_change / np.pi

        total_normalised_change += normalised_change * segment_length
        total_length += segment_length

        prev_heading = segment_heading

    if total_length == 0.0:
        return 0.0

    return total_normalised_change / total_length
