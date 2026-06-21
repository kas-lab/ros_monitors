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
"""Tests for odometry monitor functions."""

import math

from nav_msgs.msg import Odometry

import pytest

from ros_monitors_functions.odom_functions import calc_linear_velocity
from ros_monitors_functions.odom_functions import MeanLinearVelocity


def make_odometry(
    seconds: int,
    nanoseconds: int,
    linear_x: float,
) -> Odometry:
    """Create timestamped odometry with an X velocity component."""
    odom_msg = Odometry()
    odom_msg.header.stamp.sec = seconds
    odom_msg.header.stamp.nanosec = nanoseconds
    odom_msg.twist.twist.linear.x = linear_x
    return odom_msg


def test_calc_linear_velocity():
    """Calculate planar speed from components above the tolerance."""
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


def test_calc_linear_velocity_preserves_significant_negative_components():
    """Preserve negative components whose magnitudes exceed tolerance."""
    odom_msg = Odometry()
    odom_msg.twist.twist.linear.x = -0.003
    odom_msg.twist.twist.linear.y = -0.004

    vel = calc_linear_velocity(None, odom_msg, tolerance=0.001)

    assert vel == pytest.approx(0.005)
    assert odom_msg.twist.twist.linear.x == pytest.approx(-0.003)
    assert odom_msg.twist.twist.linear.y == pytest.approx(-0.004)


def test_calc_linear_velocity_zeros_small_components_by_magnitude():
    """Zero positive and negative components below tolerance magnitude."""
    odom_msg = Odometry()
    odom_msg.twist.twist.linear.x = -0.0005
    odom_msg.twist.twist.linear.y = 0.0005

    vel = calc_linear_velocity(None, odom_msg, tolerance=0.001)

    assert vel == 0.0
    assert odom_msg.twist.twist.linear.x == 0.0
    assert odom_msg.twist.twist.linear.y == 0.0


def test_calc_linear_velocity_preserves_tolerance_boundary():
    """Preserve components whose magnitudes equal the tolerance."""
    odom_msg = Odometry()
    odom_msg.twist.twist.linear.x = -0.001
    odom_msg.twist.twist.linear.y = 0.001

    velocity = calc_linear_velocity(None, odom_msg, tolerance=0.001)

    assert velocity == pytest.approx(math.sqrt(2.0) * 0.001)


def test_mean_linear_velocity():
    """Maintain a rolling mean over timestamped odometry samples."""
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
            for t, lx_val, ly_val in zip(
                timestamps,
                linear_x_values,
                linear_y_values,
            )
            if t >= window_start and t <= ts
        ]
        expected_mean = (
            sum(relevant_speeds) / len(relevant_speeds)
            if relevant_speeds else 0.0
        )

        assert mean_velocity == pytest.approx(expected_mean)


def test_mean_linear_velocity_retains_window_boundary_then_evicts():
    """Retain samples on the boundary and evict them immediately after."""
    monitor = MeanLinearVelocity(window_seconds=2.0, threshold=0.0)

    first = monitor(None, make_odometry(0, 0, 1.0))
    boundary = monitor(None, make_odometry(2, 0, 3.0))
    after_boundary = monitor(None, make_odometry(2, 1, 5.0))

    assert first == pytest.approx(1.0)
    assert boundary == pytest.approx(2.0)
    assert after_boundary == pytest.approx(4.0)
