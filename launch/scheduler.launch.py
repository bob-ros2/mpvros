#
# Copyright 2023 BobRos
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
#
import os
import random
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.actions import EmitEvent
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition


def generate_launch_description():

    ns_ = os.environ.get("SCHEDULER_NS", "/scheduler")

    # used name for the node
    launch_name = DeclareLaunchArgument('name', 
        default_value=TextSubstitution(text='scheduler'))
    
    # used namespace for the nodes
    launch_ns = DeclareLaunchArgument('ns', 
        default_value=TextSubstitution(text=ns_))

    # respawn node if exiting abnormal
    launch_respawn = DeclareLaunchArgument('respawn', 
        default_value="false")
    
    scheduler = Node(
        package='mpvros',
        executable='scheduler',
        name=LaunchConfiguration('name'),
        namespace=LaunchConfiguration('ns'),
        respawn=LaunchConfiguration('respawn'),
        output='screen'
    )

    return LaunchDescription([
        launch_name,
        launch_ns,
        launch_respawn,
        scheduler
    ])
