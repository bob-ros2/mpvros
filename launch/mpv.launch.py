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

    random.seed(time.time_ns())
    rand = "%x" % random.getrandbits(24)
    ns = os.environ.get("SCHEDULER_NS", "/")

    # MPV arguments
    launch_args = DeclareLaunchArgument('args', 
        default_value=TextSubstitution(text=''))
    
    launch_ipc = DeclareLaunchArgument('ipc_server', 
        default_value=TextSubstitution(text='/tmp/mpvros-'+rand+'.sock'))
    
    # used namespace for the nodes
    launch_ns = DeclareLaunchArgument('ns', 
        default_value=TextSubstitution(text=ns))
    
    # start with or without media title updater node
    launch_title = DeclareLaunchArgument('title', 
        default_value="true")

    # nodes

    mpv = Node(
        package='mpvros',
        executable='mpv',
        name='mpvros',
        namespace=LaunchConfiguration('ns'),
        output='screen',
        parameters=[{ 
            "args": LaunchConfiguration('args'),
            "ipc_server": LaunchConfiguration('ipc_server')
        }]
    )

    title = Node(
        condition=IfCondition(LaunchConfiguration("title")),
        package='mpvros',
        executable='title',
        name='title',
        namespace=LaunchConfiguration('ns'),
        output='screen'
    )

    return LaunchDescription([
        launch_args,
        launch_ns,
        launch_title,
        launch_ipc,
        mpv,
        RegisterEventHandler(
            OnProcessStart(
                target_action=mpv, 
                on_start=[title])
        ),
        RegisterEventHandler( # Shutdown if mpv player ends
            OnProcessExit(
                target_action=mpv,
                on_exit=[
                    EmitEvent(event=Shutdown(reason='MPV player ended'))
                ]
            )
        )
    ])
