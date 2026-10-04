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
"""Tests for direct laser scan monitors."""

import math

import pytest

from ros_monitors_functions.laser_scan_functions import (
    get_nearest_laser_scan_distance,
)
from ros_monitors_functions.laser_scan_functions import (
    get_nearest_laser_scan_ratio,
)

from sensor_msgs.msg import LaserScan


def make_scan(ranges: list[float], range_max: float = 10.0) -> LaserScan:
    """Create a laser scan with ranges and a maximum distance."""
    scan = LaserScan()
    scan.range_max = range_max
    scan.ranges = ranges
    return scan


def test_nearest_laser_scan_ratio_uses_default_minimum():
    """Ignore robot-body readings using the default minimum."""
    scan = make_scan([0.2, 5.14, math.inf])

    ratio = get_nearest_laser_scan_ratio(None, scan)

    assert ratio == pytest.approx(0.5)


def test_nearest_laser_scan_ratio_accepts_custom_minimum():
    """Normalize valid readings against a caller-provided minimum."""
    scan = make_scan([1.0, 6.0, 9.0])

    ratio = get_nearest_laser_scan_ratio(None, scan, laser_min=2.0)

    assert ratio == pytest.approx(0.5)


def test_nearest_laser_scan_ratio_excludes_boundaries():
    """Treat readings equal to the minimum or maximum as invalid."""
    scan = make_scan([2.0, 10.0, math.inf, math.nan])

    ratio = get_nearest_laser_scan_ratio(None, scan, laser_min=2.0)

    assert ratio == pytest.approx(2.25)


def test_nearest_laser_scan_ratio_returns_above_one_for_empty_ranges():
    """Return a value above 1.0 when the scan contains no readings."""
    ratio = get_nearest_laser_scan_ratio(None, make_scan([]))

    assert ratio > 1.0


@pytest.mark.parametrize(
    ('ranges', 'expected'),
    [
        ([math.inf, math.nan, 2.5, 1.0], 1.0),
        ([math.inf, math.nan], math.inf),
        ([], math.inf),
    ],
)
def test_nearest_laser_scan_distance_filters_nonfinite_values(
    ranges: list[float],
    expected: float,
) -> None:
    """Return the nearest finite value or infinity when none exists."""
    distance = get_nearest_laser_scan_distance(None, make_scan(ranges))

    if math.isinf(expected):
        assert math.isinf(distance)
    else:
        assert distance == pytest.approx(expected)
