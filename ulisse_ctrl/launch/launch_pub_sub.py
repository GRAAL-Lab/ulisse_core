"""Launch a talker and a listener."""

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():

    print('Starting publisher and subscriber...')
    
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ulisse_ctrl', node_executable='subscriber', output='screen'),
        launch_ros.actions.Node(
            package='ulisse_ctrl', node_executable='publisher', output='screen'),
])
