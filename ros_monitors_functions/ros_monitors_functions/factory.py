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
"""Factory for accessing monitor callables registered by this package."""

from collections import UserDict
from collections.abc import Callable
from types import ModuleType
from typing import Any

from . import global_costmap_functions  # noqa: F401  registration side effect
from . import laser_scan_functions  # noqa: F401  registration side effect
from . import monitor_functions  # noqa: F401  registration side effect
from . import odom_functions  # noqa: F401  registration side effect
from . import path_functions  # noqa: F401  registration side effect
from . import system_monitors_functions  # noqa: F401  registration side effect
from .utils import monitor_registry


class MonitorFactory(UserDict[str, Callable[..., Any]]):
    """
    Map registered monitor names to their callable implementations.

    Examples
    --------
    >>> factory = MonitorFactory()
    >>> velocity_monitor = factory['calc_linear_velocity']

    """

    def register_monitor(
        self,
        key: str,
        monitor_callable: Callable[..., Any],
    ) -> None:
        """
        Add a callable monitor to this factory.

        Parameters
        ----------
        key : str
            Name used to retrieve the monitor.
        monitor_callable : Callable[..., Any]
            Monitor function or class to register.

        Returns
        -------
        None
            The monitor is stored in the factory.

        Raises
        ------
        ValueError
            If ``monitor_callable`` is not callable.

        """
        if callable(monitor_callable):
            self.data[key] = monitor_callable
        else:
            raise ValueError('Monitor must be callable')

    def __init__(self, **monitors: Callable[..., Any]) -> None:
        """
        Initialize the factory with built-in and optional monitors.

        Parameters
        ----------
        **monitors : Callable[..., Any]
            Additional monitors keyed by their desired registry names.

        Returns
        -------
        None
            The constructor populates the factory.

        """
        super().__init__()
        for key, value in monitors.items():
            self.register_monitor(key, value)
        for name, monitor_callable in monitor_registry.items():
            self.register_monitor(name, monitor_callable)

    def __setitem__(
        self,
        key: str,
        value: Callable[..., Any],
    ) -> None:
        """
        Register a monitor using mapping assignment syntax.

        Parameters
        ----------
        key : str
            Name used to retrieve the monitor.
        value : Callable[..., Any]
            Monitor function or class to register.

        Returns
        -------
        None
            The monitor is stored in the factory.

        """
        self.register_monitor(key, value)

    def load_module(self, module: ModuleType) -> int:
        """
        Register newly discovered monitors from an imported module.

        Importing ``module`` must execute :func:`register_monitor` decorators
        that populate the global registry. This method then copies registry
        entries that are not already present in the factory.

        Parameters
        ----------
        module : ModuleType
            Imported module whose decorators have populated the registry.

        Returns
        -------
        int
            Number of newly registered monitors.

        """
        initial_count = len(self.data)

        # Sync all monitors from the global registry
        for name, monitor_callable in monitor_registry.items():
            if name not in self.data:
                self.register_monitor(name, monitor_callable)

        newly_loaded = len(self.data) - initial_count
        return newly_loaded
