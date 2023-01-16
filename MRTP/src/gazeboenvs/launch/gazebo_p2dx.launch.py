# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""
Demo for spawn_entity.
Launches Gazebo and spawns a model
"""

"""
Tweaked to load MRTP files
"""


import os  
from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import ExecuteProcess 
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description():
    
    pkg_dir = get_package_share_directory('gazeboenvs')
    world = os.path.join(pkg_dir, 'worlds', 'empty.world')
    sdf_path = os.path.join(pkg_dir, 'models', 'pioneer2dx', 'model.sdf')
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'demo', '-file', sdf_path],
                        output='screen')
  
    return LaunchDescription([
         ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        spawn_entity,
    ])
