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
"""Monitors that calculate instantaneous and rolling odometry metrics."""

from collections import deque
import math

from nav_msgs.msg import Odometry

from rclpy.node import Node

from ros_monitors_functions.utils import register_monitor


@register_monitor('calc_linear_velocity')
def calc_linear_velocity(
    _node: Node | None,
    odom_msg: Odometry,
    tolerance: float = 0.09,
) -> float:
    """
    Calculate planar linear speed from an odometry message.

    Components below ``tolerance`` are set to zero on the supplied message
    before the Euclidean speed is calculated.

    Parameters
    ----------
    _node : Node or None
        ROS node supplied by the monitor interface; unused by this monitor.
    odom_msg : Odometry
        Message containing the X and Y linear velocity components.
    tolerance : float, optional
        Component values below this threshold are treated as zero.

    Returns
    -------
    float
        Magnitude of the filtered planar linear velocity.

    Examples
    --------
    >>> odometry = Odometry()
    >>> odometry.twist.twist.linear.x = 3.0
    >>> odometry.twist.twist.linear.y = 4.0
    >>> calc_linear_velocity(None, odometry)
    5.0

    """
    # if linear x is super small, replace with 0.0
    if odom_msg.twist.twist.linear.x < tolerance:
        odom_msg.twist.twist.linear.x = 0.0
    if odom_msg.twist.twist.linear.y < tolerance:
        odom_msg.twist.twist.linear.y = 0.0
    return math.hypot(abs(odom_msg.twist.twist.linear.x),
                      abs(odom_msg.twist.twist.linear.y))


@register_monitor('MeanLinearVelocity')
class MeanLinearVelocity:
    """
    Maintain a rolling mean linear speed over a time window.

    Samples are timestamped using the odometry message header. Values older
    than the configured window are removed before the mean is returned.

    Examples
    --------
    >>> monitor = MeanLinearVelocity(window_seconds=5.0)
    >>> odometry = Odometry()
    >>> odometry.twist.twist.linear.x = 2.0
    >>> monitor(None, odometry)
    2.0

    """

    def __init__(
        self,
        window_seconds: float = 5.0,
        threshold: float = 0.001,
    ) -> None:
        """
        Initialize the rolling mean monitor.

        Parameters
        ----------
        window_seconds : float, optional
            Duration of the rolling window in seconds.
        threshold : float, optional
            Absolute component values below this threshold are treated as zero.

        Returns
        -------
        None
            The constructor initializes an empty sample window.

        """
        self.window_seconds = float(window_seconds)
        self.threshold = threshold
        self._samples: deque[tuple[float, float]] = deque()
        self._running_sum: float = 0.0

    def __call__(
        self,
        _node: Node | None,
        odom_msg: Odometry,
    ) -> float:
        """
        Add an odometry sample and return the current rolling mean.

        Parameters
        ----------
        _node : Node or None
            ROS node supplied by the monitor interface; unused by this monitor.
        odom_msg : Odometry
            Timestamped odometry message containing planar velocity.

        Returns
        -------
        float
            Mean planar speed for samples inside the configured time window.

        """
        # Convert message timestamp to float seconds
        ts = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9

        # Use the same small-value clamp as calc_linear_velocity.
        vx = 0.0 if abs(odom_msg.twist.twist.linear.x) < self.threshold \
            else odom_msg.twist.twist.linear.x
        vy = 0.0 if abs(odom_msg.twist.twist.linear.y) < self.threshold \
            else odom_msg.twist.twist.linear.y
        speed = math.hypot(abs(vx), abs(vy))

        # Append new sample and trim old ones outside the window
        self._samples.append((ts, speed))
        self._running_sum += speed

        window_start = ts - self.window_seconds
        while self._samples and self._samples[0][0] < window_start:
            _, old_speed = self._samples.popleft()
            self._running_sum -= old_speed

        if not self._samples:
            return 0.0

        return self._running_sum / len(self._samples)
