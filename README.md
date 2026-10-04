# ros_monitors

ROS 2 packages with generic monitoring nodes and functions.

## Packages

- `ros_monitors`: C++ Wi-Fi monitor. It publishes signal strength, link
  quality, and RX/TX rates as `diagnostic_msgs/DiagnosticArray` messages.
- `ros_monitors_functions`: Python monitors for odometry, laser scans, paths,
  costmaps, diagnostics, and system load. Contain a factory to enable adding new monitors.
- `ros_monitors_samples`: Example node that calculates linear velocity from
  `/odom`.

## Build

Run from the ROS workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

Start the Wi-Fi monitor for interface `wlan0`:

```bash
ros2 run ros_monitors wifi_monitor_node --ros-args -p iface:=wlan0
```

It publishes to `diagnostics` every 1000 ms by default. Configure
`wifi_diagnostics_topic`, `period_ms`, and `ema_alpha` with ROS parameters.

Run the odometry example:

```bash
ros2 run ros_monitors_samples example_monitor
```

Use a registered Python monitor directly:

```python
from ros_monitors_functions.factory import MonitorFactory

monitors = MonitorFactory()
speed = monitors['calc_linear_velocity'](node, odometry_msg)
```

## Register a Monitor

Decorate a function with a unique registry name:

```python
from nav_msgs.msg import Odometry
from rclpy.node import Node

from ros_monitors_functions.utils import register_monitor


@register_monitor('forward_velocity')
def forward_velocity(node: Node, msg: Odometry) -> float:
    return msg.twist.twist.linear.x
```

A monitor must:

- Be callable. Use a function or a class that implements `__call__`.
- Accept the ROS node first and message second: `(node, msg, ...)`.
- Use a unique registration name. A duplicate name replaces the previous
  registry entry.
- Return the value consumed by the monitoring application.
- Include type hints, a NumPy-style docstring, and focused tests.

The module containing the decorator must be imported before creating
`MonitorFactory`. For a new built-in module, add an explicit side-effect import
in `factory.py`. For an external module, import it and synchronize the factory:

```python
import my_monitors

monitors = MonitorFactory()
monitors.load_module(my_monitors)
```

Declare new ROS and Python dependencies in `package.xml`.

## Test

```bash
colcon test --packages-select \
  ros_monitors ros_monitors_functions ros_monitors_samples
colcon test-result --verbose
```

Licensed under Apache-2.0.
