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
"""Parsing, TF, and registration utilities shared by monitor functions."""

from collections.abc import Callable
from typing import Any, TypeVar

from rclpy.node import Node

import tf2_ros


def _parse_bool(s: str) -> bool:
    """
    Parse common textual representations of a true value.

    Parameters
    ----------
    s : str
        Value to parse, ignoring surrounding whitespace and letter case.

    Returns
    -------
    bool
        ``True`` for 1, true, yes, y, or on; otherwise ``False``.

    """
    return str(s).strip().lower() in {'1', 'true', 'yes', 'y', 'on'}


def _parse_float(s: str) -> float:
    """
    Convert a string to a floating-point value.

    Parameters
    ----------
    s : str
        Numeric string to convert.

    Returns
    -------
    float
        Parsed floating-point value.

    """
    return float(s)


def _parse_int(s: str) -> int:
    """
    Convert a string to an integer value.

    Parameters
    ----------
    s : str
        Numeric string to convert.

    Returns
    -------
    int
        Parsed integer value.

    """
    return int(s)


def _get_tf_buffer_for_node(node: Node) -> tf2_ros.Buffer:
    """
    Return the TF buffer cached on a ROS node.

    Parameters
    ----------
    node : Node
        Node that owns the lazily created TF buffer and listener.

    Returns
    -------
    tf2_ros.Buffer
        Existing or newly created TF buffer associated with the node.

    """
    # attribute names chosen to be unlikely to clash
    if not hasattr(node, '_nearestscan_tf_buffer'):
        node._nearestscan_tf_buffer = tf2_ros.Buffer()
        node._nearestscan_tf_listener = tf2_ros.TransformListener(
            node._nearestscan_tf_buffer, node, spin_thread=True)
    return node._nearestscan_tf_buffer


Monitor = TypeVar('Monitor', bound=Callable[..., Any])

monitor_registry: dict[str, Callable[..., Any]] = {}


def register_monitor(name: str) -> Callable[[Monitor], Monitor]:
    """
    Create a decorator that registers a monitor by name.

    Parameters
    ----------
    name : str
        Registry key used to retrieve the decorated monitor.

    Returns
    -------
    Callable[[Monitor], Monitor]
        Decorator that stores and returns a monitor function or class.

    """
    def decorator(func: Monitor) -> Monitor:
        """Store a monitor callable in the global registry."""
        monitor_registry[name] = func
        return func
    return decorator
