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
from ros_monitors_functions.monitor_functions import get_data_field
from ros_monitors_functions.monitor_functions import get_data_from_diagnostic_array

from std_msgs.msg import String


def make_diagnostics(
    entries: list[tuple[str, str, str]],
) -> DiagnosticArray:
    """Create diagnostics from message, key, and value tuples."""
    diagnostics = DiagnosticArray()
    for message, key, value in entries:
        status = DiagnosticStatus()
        status.message = message
        status.values.append(KeyValue(key=key, value=value))
        diagnostics.status.append(status)
    return diagnostics


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


def test_get_data_field_uses_data_attribute_by_default():
    """Return msg.data when no field argument is supplied."""
    assert get_data_field(None, String(data='hello')) == 'hello'


def test_get_data_field_returns_requested_attribute():
    """Extract the default or explicitly selected message attribute."""
    key_value = KeyValue(key='state', value='ready')

    assert get_data_field(None, key_value, 'value') == 'ready'
    assert get_data_field(None, key_value, 'key') == 'state'


def test_diagnostic_extractors_return_empty_results_without_matches():
    """Return None or an empty list when status and key do not match."""
    diagnostics = make_diagnostics([('network', 'latency', '12')])

    assert get_data_from_diagnostic_array(
        None, diagnostics, 'battery', 'level') is None
    assert get_data_array_from_diagnostic_array(
        None, diagnostics, 'network', 'missing') == []


@pytest.mark.parametrize(
    ('return_type', 'raw_value', 'expected'),
    [
        ('str', '12', '12'),
        ('STRING', 'ready', 'ready'),
        ('int', '12', 12),
        ('INTEGER', '-3', -3),
        ('float', '12.5', 12.5),
    ],
)
def test_diagnostic_return_type_aliases(
    return_type: str,
    raw_value: str,
    expected: str | int | float,
) -> None:
    """Support every documented conversion name without case sensitivity."""
    diagnostics = make_diagnostics([('system', 'value', raw_value)])

    observed = get_data_from_diagnostic_array(
        None,
        diagnostics,
        'system',
        'value',
        return_type,
    )

    if isinstance(expected, float):
        assert observed == pytest.approx(expected)
    else:
        assert observed == expected


def test_diagnostic_extractors_preserve_message_order():
    """Return the first or all matches in diagnostic message order."""
    diagnostics = make_diagnostics([
        ('thruster', 'state', 'RECOVERED'),
        ('other', 'state', 'IGNORED'),
        ('thruster', 'state', 'OK'),
    ])

    assert get_data_from_diagnostic_array(
        None, diagnostics, 'thruster', 'state') == 'RECOVERED'
    assert get_data_array_from_diagnostic_array(
        None, diagnostics, 'thruster', 'state') == ['RECOVERED', 'OK']


def test_diagnostic_extractor_rejects_unsupported_return_type():
    """Reject return type names that have no registered converter."""
    diagnostics = make_diagnostics([('system', 'value', '12')])

    with pytest.raises(ValueError, match='Unsupported return type'):
        get_data_from_diagnostic_array(
            None, diagnostics, 'system', 'value', 'bool')


def test_diagnostic_extractor_propagates_conversion_failure():
    """Raise ValueError when a matching value cannot be converted."""
    diagnostics = make_diagnostics([('system', 'value', 'not-an-int')])

    with pytest.raises(ValueError):
        get_data_from_diagnostic_array(
            None, diagnostics, 'system', 'value', 'int')
