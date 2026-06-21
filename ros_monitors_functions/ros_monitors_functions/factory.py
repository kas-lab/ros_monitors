from collections import UserDict
from .utils import monitor_registry
from .odom_functions import *
from .laser_scan_functions import *
from .monitor_functions import *
from .global_costmap_functions import *
from .path_functions import *
from .system_monitors_functions import *


class MonitorFactory(UserDict):
    def register_monitor(self, key, monitor_callable):
        if callable(monitor_callable):
            self.data[key] = monitor_callable
        else:
            raise ValueError("Monitor must be callable")

    def __init__(self, **monitors):
        super().__init__()
        for key, value in monitors.items():
            self.register_monitor(key, value)
        for name, monitor_callable in monitor_registry.items():
            self.register_monitor(name, monitor_callable)

    def __setitem__(self, key, value):
        self.register_monitor(key, value)

    def load_module(self, module):
        """
        Register all monitors from an imported module.

        The module should use @register_monitor decorator to register monitors.
        When the module is imported, the decorators populate the monitor_registry.
        This method syncs those monitors into this factory.

        Args:
            module: An imported module object

        Returns:
            int: Number of new monitors loaded from this module
        """
        initial_count = len(self.data)

        # Sync all monitors from the global registry
        for name, monitor_callable in monitor_registry.items():
            if name not in self.data:
                self.register_monitor(name, monitor_callable)

        newly_loaded = len(self.data) - initial_count
        return newly_loaded
