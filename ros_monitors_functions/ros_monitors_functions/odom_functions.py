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
import math
from collections import deque
from ros_monitors_functions.utils import register_monitor
from nav_msgs.msg import Odometry
import rclpy
import numpy as np
from ros_monitors_functions.utils import _get_tf_buffer_for_node
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration

def quat_to_rotmat(q):
    """Convert geometry_msgs/Quaternion to a 3x3 rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    if n < 1e-9:
        return np.eye(3)
    s = 2.0 / n
    X, Y, Z = x * s, y * s, z * s
    wx, wy, wz = w * X, w * Y, w * Z
    xx, xy, xz = x * X, x * Y, x * Z
    yy, yz, zz = y * Y, y * Z, z * Z
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])

@register_monitor("calc_linear_velocity")
def calc_linear_velocity(_, odom_msg: Odometry, tolerance=0.09) -> float:
    # if linear x is super small, replace with 0.0
    if odom_msg.twist.twist.linear.x < tolerance:
        odom_msg.twist.twist.linear.x = 0.0
    if odom_msg.twist.twist.linear.y < tolerance:
        odom_msg.twist.twist.linear.y = 0.0
    return math.hypot(abs(odom_msg.twist.twist.linear.x),
                      abs(odom_msg.twist.twist.linear.y))


@register_monitor("orientation_matches_motion")
def orientation_matches_motion(node, odom_msg: Odometry, base_frame, other_frame, min_speed=0.03, angle_threshold_deg=30.0):
    angle_threshold = math.radians(angle_threshold_deg)
    
    vx = odom_msg.twist.twist.linear.x
    vy = odom_msg.twist.twist.linear.y
    speed = math.hypot(vx, vy)

    if speed < min_speed:
        node.get_logger().info("Not moving ", throttle_duration_sec=3)
        return None

    # msg.twist is expressed in msg.child_frame_id, which should be base_frame
    motion_dir_base = np.array([vx, vy, 0.0])
    motion_dir_base /= np.linalg.norm(motion_dir_base)

    tf_buffer = _get_tf_buffer_for_node(node)

    try:
        tf = tf_buffer.lookup_transform(
            base_frame, other_frame, rclpy.time.Time(), #use msg time with bigger buffer later
            timeout=Duration(seconds=5.0))
    except (LookupException, ConnectivityException, ExtrapolationException) as e:
        node.get_logger().warn(f'tf lookup failed: {e}')
        return

    R = quat_to_rotmat(tf.transform.rotation)
    axis_index = {'x': 0, 'y': 1, 'z': 2}['x']
    camera_forward_base = R[:, axis_index]  # that axis expressed in base_frame

    # Compare heading only (ignore camera pitch/roll from arm tilt)
    cam_planar = np.array([camera_forward_base[0], camera_forward_base[1], 0.0])
    norm = np.linalg.norm(cam_planar)
    if norm < 1e-6:
        node.get_logger().warn('Camera is pointing straight up/down; no planar heading.')
        return
    cam_planar /= norm

    cos_angle = np.clip(np.dot(cam_planar, motion_dir_base), -1.0, 1.0)
    angle = math.acos(cos_angle)

    facing_motion = angle <= angle_threshold

    return facing_motion

@register_monitor("orientation_vs_motion")
def orientation_vs_motion(node, odom_msg: Odometry, base_frame, other_frame, min_speed=0.03):
    vx = odom_msg.twist.twist.linear.x
    vy = odom_msg.twist.twist.linear.y
    speed = math.hypot(vx, vy)

    if speed < min_speed:
        node.get_logger().info("Not moving ", throttle_duration_sec=3)
        return None

    # msg.twist is expressed in msg.child_frame_id, which should be base_frame
    motion_dir_base = np.array([vx, vy, 0.0])
    motion_dir_base /= np.linalg.norm(motion_dir_base)

    tf_buffer = _get_tf_buffer_for_node(node)

    try:
        tf = tf_buffer.lookup_transform(
            base_frame, other_frame, rclpy.time.Time(), #use msg time with bigger buffer later
            timeout=Duration(seconds=5.0))
    except (LookupException, ConnectivityException, ExtrapolationException) as e:
        node.get_logger().warn(f'tf lookup failed: {e}')
        return

    R = quat_to_rotmat(tf.transform.rotation)
    axis_index = {'x': 0, 'y': 1, 'z': 2}['x']
    camera_forward_base = R[:, axis_index]  # that axis expressed in base_frame

    # Compare heading only (ignore camera pitch/roll from arm tilt)
    cam_planar = np.array([camera_forward_base[0], camera_forward_base[1], 0.0])
    norm = np.linalg.norm(cam_planar)
    if norm < 1e-6:
        node.get_logger().warn('Camera is pointing straight up/down; no planar heading.')
        return
    cam_planar /= norm

    cos_angle = np.clip(np.dot(cam_planar, motion_dir_base), -1.0, 1.0)
    angle = math.acos(cos_angle)

    return angle

@register_monitor("MeanLinearVelocity")
class MeanLinearVelocity:
    """
    Callable monitor that maintains a rolling mean linear speed over a time window.

    Example:
    -------
        mean_vel = MeanLinearVelocity(window_seconds=5.0)
        node.create_subscription(Odometry, 'odom', mean_vel, qos_profile)

    """

    def __init__(self, window_seconds: float = 5.0, threshold: float = 0.001):
        self.window_seconds = float(window_seconds)
        self.threshold = threshold
        self._samples = deque()  # holds (timestamp_sec, speed)
        self._running_sum = 0.0

    def __call__(self, _, odom_msg: Odometry) -> float:
        # Convert message timestamp to float seconds
        ts = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9

        # Compute instantaneous speed with the same small-value clamp as calc_linear_velocity
        vx = 0.0 if abs(odom_msg.twist.twist.linear.x) < self.threshold \
            else odom_msg.twist.twist.linear.x
        vy = 0.0 if abs(odom_msg.twist.twist.linear.y) < self.threshold \
            else odom_msg.twist.twist.linear.y
        speed = math.hypot(abs(vx), abs(vy))

        # Append new sample and trim old ones outside the window
        self._samples.append((ts, speed))
        self._running_sum += speed

        window_start = ts - self.window_seconds
        while self._samples and self._samples[0][0] < window_start:
            _, old_speed = self._samples.popleft()
            self._running_sum -= old_speed

        if not self._samples:
            return 0.0

        return self._running_sum / len(self._samples)
