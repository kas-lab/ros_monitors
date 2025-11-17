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
import math
from collections import deque
from ros_monitors_functions.utils import register_monitor
from nav_msgs.msg import Odometry


@register_monitor("calc_linear_velocity")
def calc_linear_velocity(_, odom_msg: Odometry) -> float:
    # if linear x is super small, replace with 0.0
    if odom_msg.twist.twist.linear.x < 0.001:
        odom_msg.twist.twist.linear.x = 0.0
    if odom_msg.twist.twist.linear.y < 0.001:
        odom_msg.twist.twist.linear.y = 0.0
    return math.hypot(abs(odom_msg.twist.twist.linear.x),
                      abs(odom_msg.twist.twist.linear.y))


@register_monitor("MeanLinearVelocity")
class MeanLinearVelocity:
    """
    Callable monitor that maintains a rolling mean linear speed over a time window.

    Example:
    -------
        mean_vel = MeanLinearVelocity(window_seconds=5.0)
        node.create_subscription(Odometry, 'odom', mean_vel, qos_profile)

    """

    def __init__(self, window_seconds: float = 5.0, threshold: float = 0.001):
        self.window_seconds = float(window_seconds)
        self.threshold = threshold
        self._samples = deque()  # holds (timestamp_sec, speed)
        self._running_sum = 0.0

    def __call__(self, _, odom_msg: Odometry) -> float:
        # Convert message timestamp to float seconds
        ts = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9

        # Compute instantaneous speed with the same small-value clamp as calc_linear_velocity
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
