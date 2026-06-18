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
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from ros_monitors_functions.utils import register_monitor


@register_monitor("GlobalCostmapChange")
class GlobalCostmapChange:
    """
    Callable monitor that checks if a change has occurred in a continually published costmap.

    Example:
    -------
        TODO

    """

    def __init__(self, significance_threshold=50):
        self._prev_map = None
        self.threshold = significance_threshold

    def __call__(self, _, grid_msg: OccupancyGrid) -> bool:
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


@register_monitor("StableCostmap")
class StableCostmap(GlobalCostmapChange):
    """
    Callable monitor that checks if a change has occurred in a continually published costmap.

    Example:
    -------
        TODO

    """

    def __init__(self, significance_threshold=50, time_frame=5):
        self.time_stamp = None
        self.time_frame = Duration(seconds=time_frame)
        super().__init__(significance_threshold)

    def __call__(self, node, grid_msg: OccupancyGrid) -> bool:
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
