#!/usr/bin/env python3

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os
from tmr4243_utilities.utilities import anon


def generate_launch_description():

    # Observer Node Configuration
    config_observer = os.path.join(
        get_package_share_directory('template_observer'),
        'param',
        'config.yaml'
    )
    node_observer = launch_ros.actions.Node(
        package='template_observer',
        executable='observer_node.py',
        name=f'{anon()}observer',
        parameters=[config_observer],
        output='screen'
    )

    # Controller Node Configuration
    config_controller = os.path.join(
        get_package_share_directory('template_controller'),
        'config',
        'param.yaml'
    )
    node_controller = launch_ros.actions.Node(
        package='template_controller',
        executable='controller_node.py',
        name=f'{anon()}controller',
        parameters=[config_controller],
        output='screen'
    )

    # # Thrust Allocation Node Configuration
    # config_thrust_allocation = os.path.join(
    #     get_package_share_directory('template_thrust_allocation')
    # )
    node_thrust_allocation = launch_ros.actions.Node(
        package='template_thrust_allocation',
        executable='thrust_allocation_node.py',
        name=f'{anon()}thrust_allocation',
        # parameters=[config_thrust_allocation],
        output='screen'
    )

    # Launch Description
    return launch.LaunchDescription([
        node_observer,
        node_controller,
        node_thrust_allocation
    ])
