# Copyright 2025 KAS-Lab
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
"""Monitors that calculate obstacle distances from ROS laser scans."""

import math

import numpy as np

from rclpy.duration import Duration
from rclpy.node import Node

from ros_monitors_functions.utils import _get_tf_buffer_for_node
from ros_monitors_functions.utils import _parse_bool
from ros_monitors_functions.utils import _parse_float
from ros_monitors_functions.utils import register_monitor

from sensor_msgs.msg import LaserScan

import tf_transformations


@register_monitor('get_nearest_laser_scan_ratio')
def get_nearest_laser_scan_ratio(
    _node: Node | None,
    msg: LaserScan,
) -> float:
    """
    Return the normalized distance to the nearest valid laser hit.

    Readings at or below 0.28 meters are ignored to avoid detecting the robot
    body. The result is normalized between that lower bound and the sensor's
    maximum range.

    Parameters
    ----------
    _node : Node or None
        ROS node supplied by the monitor interface; unused by this monitor.
    msg : LaserScan
        Laser scan containing range limits and distance readings.

    Returns
    -------
    float
        Normalized nearest-hit distance. Values greater than ``1.0`` indicate
        that no valid hit was found.

    Examples
    --------
    >>> scan = LaserScan(range_min=0.1, range_max=10.0)
    >>> scan.ranges = [0.2, 5.14, float('inf')]
    >>> get_nearest_laser_scan_ratio(None, scan)
    0.5

    """
    laser_min = msg.range_min
    laser_max = msg.range_max

    # Initialize nearest_object to a value greater than laser_max
    nearest_object = laser_max * 2

    # MIRTE sees its own body, so enforce a suitable minimum distance.
    laser_min = 0.28
    # Iterate through the laser ranges to find the nearest object
    for laser_dist in msg.ranges:
        if laser_min < laser_dist < laser_max:
            nearest_object = min(nearest_object, laser_dist)

    # Normalize the nearest object distance
    _fitted_nearest = (nearest_object - laser_min) / (laser_max - laser_min)
    return _fitted_nearest


@register_monitor('get_nearest_laser_scan_distance')
def get_nearest_laser_scan_distance(
    _node: Node | None,
    msg: LaserScan,
) -> float:
    """
    Return the nearest finite laser range.

    Parameters
    ----------
    _node : Node or None
        ROS node supplied by the monitor interface; unused by this monitor.
    msg : LaserScan
        Laser scan whose range readings are inspected.

    Returns
    -------
    float
        Smallest finite reading, or positive infinity when none exists.

    Examples
    --------
    >>> scan = LaserScan()
    >>> scan.ranges = [float('inf'), 2.5, 1.0]
    >>> get_nearest_laser_scan_distance(None, scan)
    1.0

    """
    return min(
        (r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)),
        default=float('inf')
    )


@register_monitor('get_nearest_scan_distance_in_base')
def get_nearest_scan_distance_in_base(
    node: Node,
    scan: LaserScan,
    target_frame: str = 'base_link',
    timeout_sec: str = '0.1',
    planar: str = 'True',
) -> float:
    """
    Compute the nearest valid scan distance in a target frame.

    Parameters
    ----------
    node : Node
        ROS node that owns the cached TF buffer and provides logging.
    scan : LaserScan
        Scan containing ranges and source-frame metadata.
    target_frame : str, optional
        Frame in which distances are measured.
    timeout_sec : str, optional
        TF lookup timeout in seconds.
    planar : str, optional
        Textual boolean selecting planar distance when true or 3D distance
        when false.

    Returns
    -------
    float
        Nearest distance in meters, or positive infinity when no valid reading
        or transform is available.

    Examples
    --------
    Use ``get_nearest_scan_distance_in_base(node, scan)`` to measure valid
    scan endpoints after transforming them into ``base_link``.

    """
    # Parse string args
    timeout_f = _parse_float(timeout_sec)
    planar_b = _parse_bool(planar)

    tf_buffer = _get_tf_buffer_for_node(node)

    # Lookup transform at the scan time
    try:
        tf_msg = tf_buffer.lookup_transform(
            target_frame if target_frame != '' else 'base_link',
            scan.header.frame_id,
            scan.header.stamp,
            Duration(nanoseconds=int(timeout_f * 1_000_000_000)),
        ).transform
    except Exception as ex:  # noqa: B902  TF backends raise varied exceptions.
        node.get_logger().warn(f'TF lookup failed: {ex}')
        return float('inf')

    # Build homogeneous transform T from quaternion + translation
    quat = [
        tf_msg.rotation.x,
        tf_msg.rotation.y,
        tf_msg.rotation.z,
        tf_msg.rotation.w]
    trans = [tf_msg.translation.x, tf_msg.translation.y, tf_msg.translation.z]

    T = tf_transformations.quaternion_matrix(quat)  # 4x4; upper-left is R
    T[0:3, 3] = trans                                # inject translation

    # Iterate rays once; keep min distance
    angle = scan.angle_min
    inc = scan.angle_increment
    rmin, rmax = scan.range_min, scan.range_max

    best = float('inf')

    for r in scan.ranges:
        # Filter invalid readings early
        if math.isnan(r) or math.isinf(r) or r < rmin or r > rmax:
            angle += inc
            continue

        # Endpoint in laser frame (planar scan => z=0)
        lx = r * math.cos(angle)
        ly = r * math.sin(angle)

        # Homogeneous point in laser frame
        vec = np.array([lx, ly, 0.0, 1.0], dtype=float)

        # Transform to target frame
        bx, by, bz, _ = T @ vec

        # Planar or full 3D distance in target frame
        d = math.hypot(
            bx, by) if planar_b else math.sqrt(
            bx * bx + by * by + bz * bz)

        if d < best:
            best = d

        angle += inc

    return best
