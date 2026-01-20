from collections import UserDict
from .utils import monitor_registry
from .odom_functions import *
from .laser_scan_functions import *
from .monitor_functions import *


class MonitorFactory(UserDict):
    def register_monitor(self, key, monitor_callable):
        if callable(monitor_callable):
            self.data[key] = monitor_callable
        else:
            raise ValueError("Monitor must be callable")

    def __init__(self, **monitors):
        super().__init__()
        for key, value in monitors.items():
            self.register_monitor(self, key, value)
        for name, monitor_callable in monitor_registry.items():
            self.register_monitor(name, monitor_callable)

    def __setitem__(self, key, value):
        self.register_monitor(self, key, value)
