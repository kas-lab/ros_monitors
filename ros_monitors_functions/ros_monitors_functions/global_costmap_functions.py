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
"""Monitors for detecting changes and stability in global costmaps."""

from nav_msgs.msg import OccupancyGrid

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from ros_monitors_functions.utils import register_monitor


@register_monitor('GlobalCostmapChange')
class GlobalCostmapChange:
    """
    Detect significant changes between consecutive occupancy grids.

    The first call stores a baseline and returns ``False``. Later calls return
    ``True`` when at least one cell differs from the previous grid by more than
    the configured threshold.

    Examples
    --------
    >>> monitor = GlobalCostmapChange(significance_threshold=50)
    >>> previous = OccupancyGrid()
    >>> previous.data = [0, 0, 0]
    >>> current = OccupancyGrid()
    >>> current.data = [0, 100, 0]
    >>> monitor(None, previous)
    False
    >>> monitor(None, current)
    True

    """

    def __init__(self, significance_threshold: int = 50) -> None:
        """
        Initialize the costmap change monitor.

        Parameters
        ----------
        significance_threshold : int, optional
            Minimum absolute cell-value difference considered significant.

        Returns
        -------
        None
            The constructor initializes the monitor state.

        """
        self._prev_map: OccupancyGrid | None = None
        self.threshold = significance_threshold

    def __call__(self, _node: Node | None, grid_msg: OccupancyGrid) -> bool:
        """
        Compare an occupancy grid with the previously received grid.

        Parameters
        ----------
        _node : Node or None
            ROS node supplied by the monitor interface, unused by this monitor.
        grid_msg : OccupancyGrid
            Current global costmap message.

        Returns
        -------
        bool
            ``True`` if any cell changed by more than the threshold, otherwise
            ``False``. The first call always returns ``False``.

        """
        if (self._prev_map is None):
            self._prev_map = grid_msg
            return False

        result = False
        for i, tile in enumerate(self._prev_map.data):
            new_tile = grid_msg.data[i]
            if abs(tile - new_tile) > self.threshold:
                result = True
                break
        self._prev_map = grid_msg

        return result


@register_monitor('StableCostmap')
class StableCostmap(GlobalCostmapChange):
    """
    Detect when a global costmap remains stable for a time interval.

    Significant changes restart the stability interval. After the configured
    interval elapses without such a change, the monitor returns ``True`` and
    starts a new interval.

    Examples
    --------
    Create a monitor that reports stability after ten seconds:

    >>> monitor = StableCostmap(significance_threshold=50, time_frame=10.0)
    >>> is_stable = monitor(node, costmap)

    """

    def __init__(
        self,
        significance_threshold: int = 50,
        time_frame: float = 5.0,
    ) -> None:
        """
        Initialize the stable-costmap monitor.

        Parameters
        ----------
        significance_threshold : int, optional
            Minimum absolute cell-value difference considered significant.
        time_frame : float, optional
            Required stability interval in seconds.

        Returns
        -------
        None
            The constructor initializes the monitor state.

        """
        self.time_stamp: Time | None = None
        self.time_frame = Duration(
            nanoseconds=int(time_frame * 1_000_000_000)
        )
        super().__init__(significance_threshold)

    def __call__(self, node: Node, grid_msg: OccupancyGrid) -> bool:
        """
        Evaluate whether the costmap has remained stable long enough.

        Parameters
        ----------
        node : Node
            ROS node whose clock is used to measure the stability interval.
        grid_msg : OccupancyGrid
            Current global costmap message.

        Returns
        -------
        bool
            ``True`` when the stability interval has elapsed without a
            significant change; otherwise ``False``.

        """
        if (self.time_stamp is None):
            self.time_stamp = node.get_clock().now()
            return False

        new_time_stamp = node.get_clock().now()
        whether_change = super().__call__(node, grid_msg)

        if (whether_change is True):
            self.time_stamp = new_time_stamp
            return False
        else:
            time_since_last = new_time_stamp - self.time_stamp
            if (time_since_last > self.time_frame):
                self.time_stamp = new_time_stamp
                return True

            return False
