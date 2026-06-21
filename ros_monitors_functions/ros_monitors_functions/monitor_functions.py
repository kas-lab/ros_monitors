# Copyright 2025 KAS-Lab
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
"""Generic monitors for extracting values from ROS messages."""
from typing import Any
from typing import Iterator
from typing import List

from diagnostic_msgs.msg import DiagnosticArray

from rclpy.node import Node

from ros_monitors_functions.utils import register_monitor


DiagnosticValue = str | int | float
ReturnType = type[str] | type[int] | type[float]
RETURN_TYPES: dict[str, ReturnType] = {
    'str': str,
    'string': str,
    'int': int,
    'integer': int,
    'float': float,
}


@register_monitor('get_data_field')
def get_data_field(
    _node: Node | None,
    msg: Any,
    field: str = 'data',
) -> Any:
    """
    Return a named attribute from a ROS message.

    Parameters
    ----------
    _node : Node or None
        ROS node supplied by the monitor interface; unused by this monitor.
    msg : Any
        Message or object containing the requested attribute.
    field : str, optional
        Attribute name to read. Defaults to ``data``.

    Returns
    -------
    Any
        Value stored in the requested attribute.

    Examples
    --------
    >>> from std_msgs.msg import String
    >>> get_data_field(None, String(data='ready'))
    'ready'

    """
    if field == '':
        return getattr(msg, field)
    return getattr(msg, field)


def _iter_diagnostic_values(
    diagnostic_array: DiagnosticArray,
    status_message: str,
    kv_key: str,
) -> Iterator[str]:
    for status in diagnostic_array.status:
        if status.message == status_message:
            for key_value in status.values:
                if key_value.key == kv_key:
                    yield key_value.value


def _get_return_type(return_type: str) -> ReturnType:
    try:
        return RETURN_TYPES[return_type.lower()]
    except KeyError:
        supported_types = ', '.join(sorted(RETURN_TYPES))
        raise ValueError(
            f'Unsupported return type {return_type!r}. '
            f'Supported types: {supported_types}'
        )


@register_monitor('get_data_array_from_diagnostic_array')
def get_data_array_from_diagnostic_array(
    _,
    diagnostic_array: DiagnosticArray,
    status_message: str,
    kv_key: str,
    return_type: str = 'str',
) -> List[DiagnosticValue]:
    """Return diagnostic values converted to the requested type."""
    converter = _get_return_type(return_type)
    value_array: List[DiagnosticValue] = []
    for value in _iter_diagnostic_values(diagnostic_array, status_message, kv_key):
        value_array.append(converter(value))
    return value_array


@register_monitor('get_data_from_diagnostic_array')
def get_data_from_diagnostic_array(
    _,
    diagnostic_array: DiagnosticArray,
    status_message: str,
    kv_key: str,
    return_type: str = 'str',
) -> DiagnosticValue | None:
    """Return the first diagnostic value converted to the requested type."""
    converter = _get_return_type(return_type)
    for value in _iter_diagnostic_values(diagnostic_array, status_message, kv_key):
        return converter(value)
    return None
