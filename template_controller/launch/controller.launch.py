#!/usr/bin/env python3

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from tmr4243_utilities.utilities import anon


def generate_launch_description():

    arg_task = launch.actions.DeclareLaunchArgument(
        'task',
        default_value=launch.substitutions.TextSubstitution(text='PD_FF_controller'),
        description='Controller task type. Choose between PD_FF_controller, PID_controller, backstepping_controller.',
        choices=['PD_FF_controller', 'PID_controller', 'backstepping_controller']
    )

    arg_param_file = launch.actions.DeclareLaunchArgument(
        'param_file',
        default_value=launch.substitutions.PathJoinSubstitution(
            [launch_ros.substitutions.FindPackageShare('template_controller'), 'config', 'param.yaml']
        )
    )

    node_controller = launch_ros.actions.Node(
        package='template_controller',
        executable='controller_node.py',
        name=f'{anon()}controller',
        parameters=[
            {'task': launch.substitutions.LaunchConfiguration('task')},
            launch.substitutions.LaunchConfiguration('param_file')
        ]
    )
    return launch.LaunchDescription([
        arg_param_file,
        arg_task,
        node_controller
    ])