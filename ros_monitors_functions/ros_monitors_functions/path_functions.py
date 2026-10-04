# Copyright 2026 KAS-Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Monitors that calculate geometric properties of planned ROS paths."""

from nav_msgs.msg import Path

import numpy as np

from rclpy.node import Node

from ros_monitors_functions.utils import register_monitor


@register_monitor('average_curvature')
def average_curvature(_node: Node | None, path: Path) -> float:
    """
    Calculate the length-weighted average curvature of a path.

    Curvature is represented as the normalized heading change between
    consecutive segments. A value of ``0.0`` represents a straight path and
    ``1.0`` represents complete reversals along the entire path.

    Parameters
    ----------
    _node : Node or None
        ROS node supplied by the monitor interface, unused by this monitor.
    path : Path
        Planned path whose pose positions define the line segments.

    Returns
    -------
    float
        Length-weighted normalized heading change. Returns ``0.0`` when the
        path has fewer than two poses or has zero total length.

    Examples
    --------
    >>> from geometry_msgs.msg import PoseStamped
    >>> path = Path()
    >>> path.poses = [PoseStamped(), PoseStamped(), PoseStamped()]
    >>> path.poses[1].pose.position.x = 1.0
    >>> path.poses[2].pose.position.x = 1.0
    >>> path.poses[2].pose.position.y = 1.0
    >>> average_curvature(None, path)
    0.25

    """
    poses = path.poses
    if len(poses) < 2:
        return 0.0

    total_normalised_change = 0.0
    total_length = 0.0

    for i in range(1, len(poses)):
        p0 = poses[i - 1].pose.position
        p1 = poses[i].pose.position

        dx = p1.x - p0.x
        dy = p1.y - p0.y

        segment_length = np.hypot(dx, dy)
        # Derive heading from position deltas rather than the pose quaternion.
        segment_heading = np.arctan2(dy, dx)

        if i == 1:
            prev_heading = segment_heading

        heading_change = abs(np.arctan2(
            np.sin(segment_heading - prev_heading),
            np.cos(segment_heading - prev_heading)
        ))

        # Normalize to [0, 1], where 1 is a complete reversal (pi radians).
        normalised_change = heading_change / np.pi

        total_normalised_change += normalised_change * segment_length
        total_length += segment_length

        prev_heading = segment_heading

    if total_length == 0.0:
        return 0.0

    return float(total_normalised_change / total_length)
