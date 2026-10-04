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
"""Tests for monitor registration, parsing, and TF utilities."""

import pytest

from ros_monitors_functions import utils


@pytest.mark.parametrize(
    'value',
    ['1', 'true', ' TRUE ', 'yes', 'Y', 'on'],
)
def test_parse_bool_recognizes_true_values(value: str):
    """Recognize supported true values regardless of case or whitespace."""
    assert utils._parse_bool(value) is True


@pytest.mark.parametrize(
    'value',
    ['0', 'false', 'no', 'off', '', 'unexpected'],
)
def test_parse_bool_treats_other_values_as_false(value: str):
    """Treat unsupported boolean strings as false."""
    assert utils._parse_bool(value) is False


def test_numeric_parsers_convert_valid_strings():
    """Convert integer and floating-point strings."""
    assert utils._parse_int('-12') == -12
    assert utils._parse_float('1.25') == pytest.approx(1.25)


@pytest.mark.parametrize(
    'parser',
    [utils._parse_int, utils._parse_float],
)
def test_numeric_parsers_reject_invalid_strings(parser):
    """Propagate ValueError for invalid numeric strings."""
    with pytest.raises(ValueError):
        parser('invalid')


def test_register_monitor_stores_and_returns_callable(monkeypatch):
    """Register a decorated callable without replacing its identity."""
    monkeypatch.delitem(
        utils.monitor_registry,
        'test_registered_monitor',
        raising=False,
    )

    @utils.register_monitor('test_registered_monitor')
    def monitor() -> str:
        return 'registered'

    assert utils.monitor_registry['test_registered_monitor'] is monitor
    assert monitor() == 'registered'


def test_tf_buffer_is_created_once_and_cached(monkeypatch):
    """Create one TF buffer/listener pair and reuse it on later calls."""
    created_buffers = []
    created_listeners = []

    class FakeBuffer:
        """Record TF buffer construction."""

        def __init__(self) -> None:
            created_buffers.append(self)

    class FakeTransformListener:
        """Record TF listener construction arguments."""

        def __init__(self, buffer, node, spin_thread):
            created_listeners.append((buffer, node, spin_thread))

    class FakeNode:
        """Permit utility code to attach cached TF attributes."""

    monkeypatch.setattr(utils.tf2_ros, 'Buffer', FakeBuffer)
    monkeypatch.setattr(
        utils.tf2_ros,
        'TransformListener',
        FakeTransformListener,
    )
    node = FakeNode()

    first = utils._get_tf_buffer_for_node(node)
    second = utils._get_tf_buffer_for_node(node)

    assert first is second
    assert created_buffers == [first]
    assert created_listeners == [(first, node, True)]
