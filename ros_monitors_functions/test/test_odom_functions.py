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

from nav_msgs.msg import Odometry

import pytest

from ros_monitors_functions.odom_functions import calc_linear_velocity
from ros_monitors_functions.odom_functions import MeanLinearVelocity


def test_calc_linear_velocity():
    odom_msg = Odometry()
    odom_msg.twist.twist.linear.x = 0.003
    odom_msg.twist.twist.linear.y = 0.004
    vel = calc_linear_velocity(None, odom_msg, tolerance=0.001)
    assert vel == pytest.approx(math.hypot(0.003, 0.004))

    odom_msg.twist.twist.linear.x = 0.01
    odom_msg.twist.twist.linear.y = 0.02
    vel = calc_linear_velocity(None, odom_msg, tolerance=0.001)
    expected_vel = math.hypot(0.01, 0.02)
    assert vel == pytest.approx(expected_vel)


def test_mean_linear_velocity():
    mean_vel_monitor = MeanLinearVelocity(window_seconds=2.0, threshold=0.001)

    # Simulate odom messages at different timestamps
    timestamps = [1.0, 1.5, 2.0, 3.0, 4.0]  # in seconds
    linear_x_values = [0.0, 0.01, 0.02, 0.03, 0.04]
    linear_y_values = [0.0, 0.01, 0.02, 0.03, 0.04]

    for ts, lx, ly in zip(timestamps, linear_x_values, linear_y_values):
        odom_msg = Odometry()
        odom_msg.header.stamp.sec = int(ts)
        odom_msg.header.stamp.nanosec = int((ts - int(ts)) * 1e9)
        odom_msg.twist.twist.linear.x = lx
        odom_msg.twist.twist.linear.y = ly

        mean_velocity = mean_vel_monitor(None, odom_msg)

        # Manually compute expected mean velocity over the window
        window_start = ts - mean_vel_monitor.window_seconds
        relevant_speeds = [
            math.hypot(abs(lx_val), abs(ly_val))
            for t, lx_val, ly_val in zip(timestamps, linear_x_values, linear_y_values)
            if t >= window_start and t <= ts
        ]
        expected_mean = sum(relevant_speeds) / len(relevant_speeds) if relevant_speeds else 0.0

        assert mean_velocity == pytest.approx(expected_mean)
