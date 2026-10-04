# Repository Guidelines

## Project Structure & Module Organization

This repository contains three ROS 2 Humble packages:

- `ros_monitors/`: C++17 library and `wifi_monitor_node`; headers are in
  `include/ros_monitors/`, implementations in `src/`.
- `ros_monitors_functions/`: registered Python monitor helpers.
- `ros_monitors_samples/`: Python examples and the `example_monitor` entry point.

Tests live in each package's `test/` directory. `ros_monitors` uses
`ament_cmake`; both Python packages use `ament_python`, `setup.py`, `setup.cfg`,
and an ament resource marker.

## Build, Test, and Development Commands

Run commands from the ROS workspace root (the directory containing `src/`):

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select ros_monitors ros_monitors_functions ros_monitors_samples
colcon test-result --verbose
```

`--symlink-install` keeps Python edits available without reinstalling. The host
may lack ROS Python modules even after sourcing `/opt/ros/humble`. In that case,
run tests in the mounted `ros_monitors` container:

```bash
docker exec <container> bash -lc 'source /opt/ros/humble/setup.bash && source /home/ubuntu-user/ros_monitors_ws/install/setup.bash && cd /home/ubuntu-user/ros_monitors_ws/src/ros_monitors/ros_monitors_functions && PYTHONPATH=$PWD:$PYTHONPATH python3 -m pytest -q test'
```

Prepend to `PYTHONPATH`; replacing it hides `rclpy` and ROS message packages.

## Coding Style & Naming Conventions

Use four spaces in Python, `snake_case` for functions/modules, and `PascalCase`
for classes. Docstrings follow NumPy sections. For multiline docstrings, put the
summary on the line after opening quotes (D213) and leave a blank line after the
last section (D413). Preserve Apache-2.0 headers. C++ is C++17 with two-space
indentation and `-Wall -Wextra -Wpedantic`.

## Monitor Implementation Notes

Register monitors with `@register_monitor`. `factory.py` imports monitor modules
for decorator side effects; use explicit module imports with
`# noqa: F401  registration side effect`, never wildcard imports. The monitor
call signature convention is `(node, message, ...)`. Annotate ignored nodes as
`Node | None`. `rclpy.duration.Duration` expects integer values; convert
fractional seconds to nanoseconds. Apply numeric tolerances to magnitudes when
sign should not affect filtering.

## Testing Guidelines

Use `pytest`; name files and functions `test_*.py` and `test_*`. Prefer ROS
messages plus fake clocks, nodes, and TF buffers. Isolate global registry
mutations with `monkeypatch`. Cover boundaries, invalid values, state changes,
transform failures, and parsing. `OccupancyGrid.data` accepts only signed int8
values (`-128..127`).
Run the complete package suite because it includes Flake8, PEP 257, and
copyright checks. Follow the container's ament lint results if host plugin
versions disagree on import ordering.

## Commit & Pull Request Guidelines

Use concise, lowercase, imperative summaries such as `add wifi monitor and
monitoring functions`. Keep commits focused. Pull requests should describe the
change, affected packages, related issues, and exact test results.
