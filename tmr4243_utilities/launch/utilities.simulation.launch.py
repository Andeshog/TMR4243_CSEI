#!/usr/bin/env python3

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os
from tmr4243_utilities.utilities import anon

def generate_launch_description():

    node_utilities = launch_ros.actions.Node(
        package='tmr4243_utilities',
        executable='utility_node.py',
        name=f"{anon()}utilities",
        output='screen'
    )

    node_simulator = launch_ros.actions.Node(
        package='cybership_simulator',
        executable='cybership_enterprise1.py',
        name=f"{anon()}simulation",
        output='screen'
    )

    node_rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name=f'{anon()}rviz',
        output='screen',
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory('cybership_viz'),
                'config',
                'cybership.rviz'
            )
        ]
    )

    urdf = os.path.join(
        get_package_share_directory('cybership_description'),
        'urdf','cybership_enterprise1_base.urdf')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    node_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'{anon()}robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }],
        arguments=[urdf]

    )

    group_gui = launch.actions.GroupAction(
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('use_gui', default='false')),
        actions=[node_rviz]
    )

    ld = launch.LaunchDescription()
    ld.add_action(node_utilities)
    ld.add_action(node_simulator)
    ld.add_action(group_gui)
    ld.add_action(node_robot_state_publisher)

    return ld