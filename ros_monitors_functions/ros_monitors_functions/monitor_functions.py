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

from ros_monitors_functions.utils import register_monitor


@register_monitor("get_data_field")
def get_data_field(_, msg, field='data'):
    if field == '':
        return getattr(msg, field)
    return getattr(msg, field)
