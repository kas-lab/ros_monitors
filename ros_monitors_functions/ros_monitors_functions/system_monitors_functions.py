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
from diagnostic_msgs.msg import DiagnosticArray
from ros_monitors_functions.utils import register_monitor


@register_monitor("AverageCPULoad")
class AverageCPULoad:
    """
    TODO
    Example:
    -------
        TODO

    """

    def __init__(self, minutes: int):
        self._prev_reading = 0.0
        self.minutes = minutes

    def _x_min(self, diagnostics_msg):
        for status in diagnostics_msg.status:
            if 'cpu_monitor' in status.name:
                for kv in status.values:
                    if f"Load Average ({self.minutes}min)" in kv.key:
                        return float(kv.value.strip("%"))
        return None

    def __call__(self, _, diagnostics_msg: DiagnosticArray) -> float:
        avg_load = self._x_min(diagnostics_msg)
        if (avg_load is None):
            return self._prev_reading
        self._prev_reading = avg_load
        return avg_load


@register_monitor("AverageCPULoad1min")
class AverageCPULoad1min(AverageCPULoad):
    """
    TODO
    Example:
    -------
        TODO

    """

    def __init__(self):
        super().__init__(1)
