from launch import LaunchDescription
import launch_ros.actions
import os

os.environ['RCUTILS_COLORIZED_OUTPUT'] = "1" # visible in this process + all children

def generate_launch_description():

    print('Starting Simulator...')
    # Node arguments must be comma separated strings
    sim_node = launch_ros.actions.Node(
        package='ulisse_sim',
        executable='simulator_node',
        output='screen',
        arguments=[])

    return LaunchDescription([
        sim_node])
