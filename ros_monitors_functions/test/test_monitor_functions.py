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
"""Tests for reusable monitor functions."""

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

import pytest

from ros_monitors_functions.monitor_functions import get_data_array_from_diagnostic_array
from ros_monitors_functions.monitor_functions import get_data_from_diagnostic_array


def test_get_data_from_diagnostic_array():
    """Test extracting typed values from diagnostic arrays."""
    battery_level = 70.35

    key_value = KeyValue()
    key_value.key = 'battery_level'
    key_value.value = str(battery_level)

    status_msg = DiagnosticStatus()
    status_msg.level = DiagnosticStatus.OK
    status_msg.name = 'battery_monitor: Battery level'
    status_msg.message = 'QA status'
    status_msg.values.append(key_value)

    diag_msg = DiagnosticArray()
    diag_msg.status.append(status_msg)

    battery_level_observed = get_data_from_diagnostic_array(
        None, diag_msg, 'QA status', 'battery_level', 'float')
    assert battery_level_observed == pytest.approx(70.35)

    key_value = KeyValue()
    key_value.key = 'c_thruster_0'
    key_value.value = 'RECOVERED'

    key_value_2 = KeyValue()
    key_value_2.key = 'c_thruster_0'
    key_value_2.value = 'OK'

    status_msg.name = 'thruster_monitor: Thruster status'
    status_msg.message = 'Component status'
    status_msg.values.append(key_value)
    status_msg.values.append(key_value_2)

    diag_msg = DiagnosticArray()
    diag_msg.status.append(status_msg)

    thruster_status_observed = get_data_array_from_diagnostic_array(
        None, diag_msg, 'Component status', 'c_thruster_0', 'str')
    assert thruster_status_observed == ['RECOVERED', 'OK']
