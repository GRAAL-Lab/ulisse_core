from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    print('Starting controller...')
    # Node arguments must be comma separated strings
    dcl_node = launch_ros.actions.Node( package='ulisse_ctrl', node_executable='dynamic_control_node', output='screen', arguments=[])
    kcl_node = launch_ros.actions.Node (package='ulisse_ctrl', node_executable='kinematic_control_node', output='screen', arguments=[])
    nav_filter_node = launch_ros.actions.Node( package='nav_filter', node_executable='navigation_filter_node', output='screen', arguments=[])

    return LaunchDescription([
        dcl_node,
        kcl_node,
        nav_filter_node
        # more nodes can be added here
    ])
