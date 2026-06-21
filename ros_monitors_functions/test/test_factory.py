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
"""Tests for monitor factory behavior."""

from types import ModuleType

import pytest

from ros_monitors_functions.factory import MonitorFactory
from ros_monitors_functions.utils import monitor_registry


def first_monitor() -> str:
    """Return a marker for factory registration tests."""
    return 'first'


def second_monitor() -> str:
    """Return a second marker for duplicate-name tests."""
    return 'second'


def test_factory_contains_registered_builtin_monitors():
    """Populate a new factory from the global monitor registry."""
    factory = MonitorFactory()

    expected = {
        'GlobalCostmapChange',
        'StableCostmap',
        'average_curvature',
        'calc_linear_velocity',
        'get_data_field',
        'get_nearest_laser_scan_ratio',
    }
    assert expected <= set(factory)


def test_factory_accepts_constructor_and_mapping_registrations():
    """Register custom monitors through both supported public interfaces."""
    factory = MonitorFactory(custom=first_monitor)

    assert factory['custom'] is first_monitor

    factory['assigned'] = second_monitor
    assert factory['assigned'] is second_monitor


def test_factory_rejects_noncallable_values():
    """Reject invalid monitors through methods and mapping assignment."""
    factory = MonitorFactory()

    with pytest.raises(ValueError, match='Monitor must be callable'):
        factory.register_monitor('invalid', 42)

    with pytest.raises(ValueError, match='Monitor must be callable'):
        factory['invalid'] = 42


def test_factory_duplicate_name_uses_latest_assignment():
    """Replace an existing monitor when assigning the same name again."""
    factory = MonitorFactory(custom=first_monitor)

    factory['custom'] = second_monitor

    assert factory['custom'] is second_monitor


def test_factory_load_module_adds_only_new_registry_entries(monkeypatch):
    """Synchronize newly decorated monitors once per factory."""
    factory = MonitorFactory()
    module = ModuleType('test_monitors')
    monkeypatch.setitem(monitor_registry, 'late_monitor', first_monitor)

    assert factory.load_module(module) == 1
    assert factory['late_monitor'] is first_monitor
    assert factory.load_module(module) == 0
