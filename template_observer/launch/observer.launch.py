#!/usr/bin/env python3

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os
from tmr4243_utilities.utilities import anon

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('template_observer'),
        'param',
        'config.yaml'
    )

    node_observer = launch_ros.actions.Node(
        package='template_observer',
        executable='observer_node.py',
        name=f'{anon()}observer',
        parameters=[config],
        output='screen'
    )

    return launch.LaunchDescription([
        node_observer
    ])
