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

from nav_msgs.msg import Odometry

def calc_linear_velocity(_, odom_msg: Odometry) -> float:
	# if linear x is super small, replace with 0.0
	if odom_msg.twist.twist.linear.x < 0.001:
		odom_msg.twist.twist.linear.x = 0.0
	if odom_msg.twist.twist.linear.y < 0.001:
		odom_msg.twist.twist.linear.y = 0.0
	return math.hypot(abs(odom_msg.twist.twist.linear.x),
						abs(odom_msg.twist.twist.linear.y))