import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():

    arg_task = launch.actions.DeclareLaunchArgument(
        'task',
        default_value=launch.substitutions.TextSubstitution(text='stationkeeping'),
        description='Guidance task type. Choose between stationkeeping, straight_line.',
        choices=['stationkeeping', 'straight_line', 'hybridpath']
    )

    node_guidance = launch_ros.actions.Node(
        package='template_guidance',
        executable='guidance_node.py',
        name='guidance',
        parameters=[
                {'task': launch.substitutions.LaunchConfiguration('task')},
        ]
    )
    return launch.LaunchDescription([
        arg_task,
        node_guidance
    ])
