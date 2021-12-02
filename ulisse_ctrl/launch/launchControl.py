from launch import LaunchDescription
import launch_ros.actions
import os

os.environ['RCUTILS_COLORIZED_OUTPUT'] = "1" # visible in this process + all children

def generate_launch_description():

    print('Starting Controller...')
    # Node arguments must be comma separated strings
    dcl_node = launch_ros.actions.Node(
            package='ulisse_ctrl',
            executable='dynamic_control_node',
            output='screen',
            arguments=[])
    kcl_node = launch_ros.actions.Node(
            package='ulisse_ctrl',
            executable='kinematic_control_node',
            output='screen',
            arguments=[])
    nav_filter_node = launch_ros.actions.Node(
            package='nav_filter',
            executable='nav_filter_node',
            output='screen',
            arguments=[])
    nav_filter_udp_sender = launch_ros.actions.Node(
            package='nav_filter',
            executable='nav_filter_udp_sender',
            output='screen',
            arguments=["192.168.1.153"])
    
    return LaunchDescription([
        dcl_node,
        kcl_node,
        nav_filter_node,
        # nav_filter_udp_sender
        # more nodes can be added here
    ])
