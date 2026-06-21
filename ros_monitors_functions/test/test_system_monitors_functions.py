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
"""Tests for system diagnostic monitors."""

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

import pytest

from ros_monitors_functions.system_monitors_functions import AverageCPULoad
from ros_monitors_functions.system_monitors_functions import AverageCPULoad1min


def make_diagnostics(
    name: str,
    key: str,
    value: str,
) -> DiagnosticArray:
    """Create a diagnostic array containing one status value."""
    status = DiagnosticStatus(name=name)
    status.values.append(KeyValue(key=key, value=value))
    diagnostics = DiagnosticArray()
    diagnostics.status.append(status)
    return diagnostics


def test_average_cpu_load_selects_window_and_strips_percent():
    """Extract the configured load window and remove percent notation."""
    monitor = AverageCPULoad(minutes=5)
    diagnostics = make_diagnostics(
        'system/cpu_monitor',
        'Load Average (5min)',
        '23.5%',
    )

    assert monitor(None, diagnostics) == pytest.approx(23.5)


def test_average_cpu_load_ignores_unrelated_diagnostics():
    """Return the initial fallback for unrelated status names or keys."""
    monitor = AverageCPULoad(minutes=5)
    wrong_status = make_diagnostics(
        'memory_monitor',
        'Load Average (5min)',
        '90%',
    )
    wrong_key = make_diagnostics(
        'cpu_monitor',
        'Temperature',
        '90',
    )

    assert monitor(None, wrong_status) == 0.0
    assert monitor(None, wrong_key) == 0.0


def test_average_cpu_load_reuses_previous_reading_when_absent():
    """Return the latest valid reading when a later message lacks one."""
    monitor = AverageCPULoad(minutes=5)
    valid = make_diagnostics(
        'cpu_monitor',
        'Load Average (5min)',
        '42.0%',
    )

    assert monitor(None, valid) == pytest.approx(42.0)
    assert monitor(None, DiagnosticArray()) == pytest.approx(42.0)


def test_average_cpu_load_one_minute_specialization():
    """Configure and extract the one-minute CPU load average."""
    monitor = AverageCPULoad1min()
    diagnostics = make_diagnostics(
        'cpu_monitor',
        'Load Average (1min)',
        '7.5%',
    )

    assert monitor.minutes == 1
    assert monitor(None, diagnostics) == pytest.approx(7.5)


def test_average_cpu_load_rejects_malformed_value():
    """Propagate conversion errors from malformed diagnostic values."""
    monitor = AverageCPULoad(minutes=5)
    diagnostics = make_diagnostics(
        'cpu_monitor',
        'Load Average (5min)',
        'unknown%',
    )

    with pytest.raises(ValueError):
        monitor(None, diagnostics)
