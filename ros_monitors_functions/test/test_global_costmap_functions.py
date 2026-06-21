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
"""Tests for global costmap monitors."""

from nav_msgs.msg import OccupancyGrid

from rclpy.time import Time

from ros_monitors_functions.global_costmap_functions import GlobalCostmapChange
from ros_monitors_functions.global_costmap_functions import StableCostmap


class FakeClock:
    """Controllable clock used by stable-costmap tests."""

    def __init__(self) -> None:
        """Initialize the clock at zero seconds."""
        self.nanoseconds = 0

    def set_seconds(self, seconds: float) -> None:
        """Set the current fake time in seconds."""
        self.nanoseconds = int(seconds * 1_000_000_000)

    def now(self) -> Time:
        """Return the configured fake time."""
        return Time(nanoseconds=self.nanoseconds)


class FakeNode:
    """Minimal node exposing a controllable clock."""

    def __init__(self, clock: FakeClock) -> None:
        """Store the clock returned by :meth:`get_clock`."""
        self.clock = clock

    def get_clock(self) -> FakeClock:
        """Return the fake clock."""
        return self.clock


def make_grid(values: list[int]) -> OccupancyGrid:
    """Create an occupancy grid with the supplied cell values."""
    grid = OccupancyGrid()
    grid.data = values
    return grid


def test_global_costmap_change_threshold_and_baseline_updates():
    """Detect only changes above threshold and update the baseline."""
    monitor = GlobalCostmapChange(significance_threshold=50)

    assert monitor(None, make_grid([0, 0])) is False
    assert monitor(None, make_grid([0, 49])) is False
    assert monitor(None, make_grid([0, 99])) is False
    assert monitor(None, make_grid([0, 20])) is True
    assert monitor(None, make_grid([0, 20])) is False


def test_stable_costmap_resets_interval_after_change():
    """Report stability only after a full unchanged interval."""
    clock = FakeClock()
    node = FakeNode(clock)
    monitor = StableCostmap(significance_threshold=50, time_frame=5.0)

    assert monitor(node, make_grid([0])) is False

    clock.set_seconds(1.0)
    assert monitor(node, make_grid([0])) is False

    clock.set_seconds(4.0)
    assert monitor(node, make_grid([0])) is False

    clock.set_seconds(6.0)
    assert monitor(node, make_grid([100])) is False

    clock.set_seconds(10.0)
    assert monitor(node, make_grid([100])) is False

    clock.set_seconds(12.0)
    assert monitor(node, make_grid([100])) is True
