# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Packages

Three ROS 2 Humble packages in one repo:

| Package | Type | Role |
|---|---|---|
| `ros_monitors/` | C++ (ament_cmake) | `WifiMonitor` node + shared library |
| `ros_monitors_functions/` | Python (ament_python) | Monitor function library and `MonitorFactory` |
| `ros_monitors_samples/` | Python (ament_python) | Installable usage examples |

## Build & Test

Commands run from the workspace root (directory containing `src/`):

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Test all three packages
colcon test --packages-select ros_monitors ros_monitors_functions ros_monitors_samples --event-handlers console_direct+
colcon test-result --verbose

# Run a single linter pass (flake8 or pep257)
colcon test --packages-select ros_monitors_functions --event-handlers console_direct+ --pytest-args -k test_flake8

# Run the example node
ros2 run ros_monitors_samples example_monitor

# Run the wifi monitor node (requires a physical WiFi interface)
ros2 run ros_monitors wifi_monitor_node
```

`--symlink-install` keeps Python edits live without rebuilding.

## Architecture

### Python: register/factory pattern

`ros_monitors_functions/utils.py` holds a module-level `monitor_registry: dict[str, Callable]` and the `@register_monitor('key')` decorator that populates it.

All `*_functions.py` modules decorate their callables with `@register_monitor`. The `factory.py` module imports every `*_functions.py` as a side effect, then `MonitorFactory.__init__` copies the populated registry into itself. Any new module must be added to `factory.py`'s imports to appear in the factory.

```python
from ros_monitors_functions.factory import MonitorFactory
factory = MonitorFactory()
result = factory['calc_linear_velocity'](node, odom_msg)
```

### Monitor callable signature

All monitors (functions and stateful classes) share the same call signature:

```python
def monitor(node_or_none, msg, param1='default', ...) -> value
```

- First argument is the ROS `Node` (needed for TF lookups) or `None` for simple monitors.
- Remaining parameters beyond `msg` are strings — callers pass ROS parameter values directly. Parsers `_parse_bool`, `_parse_float`, `_parse_int` live in `utils.py`.
- Stateful monitors (e.g. `MeanLinearVelocity`, `AverageCPULoad`) are callable classes registered via `@register_monitor` on the class; `MonitorFactory` stores the **class** (not an instance), so each caller instantiates their own.

### TF-using monitors

`_get_tf_buffer_for_node(node)` in `utils.py` lazily creates a `tf2_ros.Buffer` and `TransformListener` as `node._nearestscan_tf_buffer` / `node._nearestscan_tf_listener`. Monitors that transform scan points into another frame call this helper.

### C++: WifiMonitor

`ros_monitors/` builds a shared library (`wifi_monitor`) and a node executable (`wifi_monitor_node`). `WifiMonitor` (namespace `ros_monitors`) extends `rclcpp::Node`, reads `/sys/class/net/<iface>/statistics/` for throughput and `/proc/net/wireless` for signal strength, applies EMA smoothing, and publishes `diagnostic_msgs/DiagnosticArray` on the `diagnostics` topic.

## Adding a New Python Monitor

1. Add the function or callable class to the relevant `*_functions.py` (or create a new `*_functions.py`).
2. Decorate with `@register_monitor('your_key')`.
3. If creating a new module, add an import line to `factory.py` with `# noqa: F401  registration side effect`.
4. Add tests in `ros_monitors_functions/test/`.

## Coding Conventions

- Python: PEP 8, four-space indent, NumPy-style docstrings, pep257-compliant.
- C++: C++17, two-space indent, `snake_case` methods, `-Wall -Wextra -Wpedantic`.
- All new files require the Apache-2.0 copyright header (see workspace CLAUDE.md for the template).
- Keep ROS names (nodes, topics, parameters) lowercase with underscores.

## CI

GitHub Actions (`.github/workflows/test.yml`) runs on push to `main`/`dev`, PRs, and nightly. Uses `rostooling/setup-ros-docker:ubuntu-jammy-latest` + `ros-tooling/action-ros-ci` to build and test all three packages on ROS 2 Humble.
