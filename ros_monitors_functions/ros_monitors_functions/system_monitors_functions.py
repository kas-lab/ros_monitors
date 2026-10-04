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
"""Monitors that extract system metrics from ROS diagnostic messages."""

from diagnostic_msgs.msg import DiagnosticArray

from rclpy.node import Node

from ros_monitors_functions.utils import register_monitor


@register_monitor('AverageCPULoad')
class AverageCPULoad:
    """
    Extract a CPU load average from diagnostic messages.

    The monitor searches statuses whose names contain ``cpu_monitor`` and
    returns the load matching its configured averaging window. If a message
    lacks that reading, the monitor returns the last value found, initially
    ``0.0``.

    Examples
    --------
    >>> from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
    >>> diagnostics = DiagnosticArray()
    >>> status = DiagnosticStatus(name='cpu_monitor')
    >>> status.values = [
    ...     KeyValue(key='Load Average (5min)', value='23.5%'),
    ... ]
    >>> diagnostics.status = [status]
    >>> monitor = AverageCPULoad(minutes=5)
    >>> monitor(None, diagnostics)
    23.5

    """

    def __init__(self, minutes: int) -> None:
        """
        Initialize a CPU load monitor for an averaging window.

        Parameters
        ----------
        minutes : int
            Averaging window used in the diagnostic key, in minutes.

        Returns
        -------
        None
            The constructor initializes the monitor state.

        """
        self._prev_reading: float = 0.0
        self.minutes = minutes

    def _x_min(self, diagnostics_msg: DiagnosticArray) -> float | None:
        """
        Extract the configured load average from a diagnostic message.

        Parameters
        ----------
        diagnostics_msg : DiagnosticArray
            Diagnostic message containing CPU monitor status values.

        Returns
        -------
        float or None
            Parsed CPU load, or ``None`` when the expected value is absent.

        """
        for status in diagnostics_msg.status:
            if 'cpu_monitor' in status.name:
                for kv in status.values:
                    if f'Load Average ({self.minutes}min)' in kv.key:
                        return float(kv.value.strip('%'))
        return None

    def __call__(
        self,
        _node: Node | None,
        diagnostics_msg: DiagnosticArray,
    ) -> float:
        """
        Read the current CPU load or reuse the previous reading.

        Parameters
        ----------
        _node : Node or None
            ROS node supplied by the monitor interface; unused by this monitor.
        diagnostics_msg : DiagnosticArray
            Diagnostic message to inspect.

        Returns
        -------
        float
            Current CPU load when present, otherwise the previous reading.

        """
        avg_load = self._x_min(diagnostics_msg)
        if (avg_load is None):
            return self._prev_reading
        self._prev_reading = avg_load
        return avg_load


@register_monitor('AverageCPULoad1min')
class AverageCPULoad1min(AverageCPULoad):
    """
    Extract the one-minute CPU load average from diagnostics.

    Examples
    --------
    >>> monitor = AverageCPULoad1min()
    >>> monitor.minutes
    1

    """

    def __init__(self) -> None:
        """
        Initialize a CPU load monitor with a one-minute window.

        Returns
        -------
        None
            The constructor initializes the monitor with a one-minute window.

        """
        super().__init__(1)
