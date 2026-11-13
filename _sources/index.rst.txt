ros_monitors documentation
==========================

``ros_monitors`` provides reusable monitor components for
ROS systems. It ships three packages:

- **ros_monitors** — C++ ``WifiMonitor`` node that publishes network
  diagnostics via ``diagnostic_msgs/DiagnosticArray``.
- **ros_monitors_functions** — Python library of monitor callables
  (functions and stateful classes) addressable by name through
  ``MonitorFactory``.
- **ros_monitors_samples** — installable examples showing how to wire
  ``MonitorFactory`` into a ROS node.

.. toctree::
   :maxdepth: 1

   Home <self>
   api

Links
-----

- Source: https://github.com/kas-lab/ros_monitors
- Issue tracker: https://github.com/kas-lab/ros_monitors/issues
- License: Apache 2.0
