from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    print('Starting controller...')
    configfile = '__params:=' + get_package_share_directory('ulisse_ctrl') + '/conf/ulisse.yaml'
    print("Config file: ", configfile)
    # Node arguments must be comma separated strings
    ctrl_node = launch_ros.actions.Node(
            package='ulisse_ctrl', node_executable='controller_node', output='screen', arguments=[configfile])
    nav_filter_node = launch_ros.actions.Node(
            package='ulisse_ctrl', node_executable='navigation_filter_node', output='screen', arguments=[configfile])

    return LaunchDescription([
        ctrl_node,
        nav_filter_node
        # more nodes can be added here
    ])
