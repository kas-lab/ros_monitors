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
"""Tests for path geometry monitors."""

import math

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Path

import pytest

from ros_monitors_functions.path_functions import average_curvature


def make_path(points: list[tuple[float, float]]) -> Path:
    """Create a path from planar coordinates."""
    path = Path()
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        path.poses.append(pose)
    return path


@pytest.mark.parametrize('points', [[], [(0.0, 0.0)]])
def test_average_curvature_short_paths_return_zero(
    points: list[tuple[float, float]],
) -> None:
    """Return zero when a path has fewer than two poses."""
    assert average_curvature(None, make_path(points)) == 0.0


def test_average_curvature_two_pose_path_returns_zero():
    """Return zero for a two-pose path — one segment, no turn possible."""
    path = make_path([(0.0, 0.0), (1.0, 0.0)])

    assert average_curvature(None, path) == pytest.approx(0.0)


def test_average_curvature_straight_path_returns_zero():
    """Return zero for collinear segments with the same heading."""
    path = make_path([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])

    assert average_curvature(None, path) == pytest.approx(0.0)


def test_average_curvature_right_angle_is_length_weighted():
    """Weight a right-angle heading change by segment length."""
    path = make_path([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])

    assert average_curvature(None, path) == pytest.approx(0.25)


def test_average_curvature_complete_reversal():
    """Normalize a 180-degree heading change to one before averaging."""
    path = make_path([(0.0, 0.0), (1.0, 0.0), (0.0, 0.0)])

    assert average_curvature(None, path) == pytest.approx(0.5)


def test_average_curvature_repeated_poses_return_zero():
    """Return zero when every segment has zero length."""
    path = make_path([(1.0, 1.0), (1.0, 1.0), (1.0, 1.0)])

    assert average_curvature(None, path) == 0.0


def test_average_curvature_wraps_heading_difference():
    """Treat headings around opposite sides of pi as nearby angles."""
    positive = math.radians(179.0)
    negative = math.radians(-179.0)
    first = (math.cos(positive), math.sin(positive))
    second = (
        first[0] + math.cos(negative),
        first[1] + math.sin(negative),
    )
    path = make_path([(0.0, 0.0), first, second])

    assert average_curvature(None, path) == pytest.approx(1.0 / 180.0)
