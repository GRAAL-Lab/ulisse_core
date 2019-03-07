from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    print('Starting controller...')
    kclconfigfile = '__params:=' + get_package_share_directory('ulisse_ctrl') + '/conf/kcl_ulisse.yaml'
    dclconfigfile = '__params:=' + get_package_share_directory('ulisse_ctrl') + '/conf/dcl_ulisse.yaml'
    navconfigfile = '__params:=' + get_package_share_directory('nav_filter') + '/conf/navfilter.yaml'
    # print("Config file: ", configfile)
    # Node arguments must be comma separated strings
    dcl_node = launch_ros.actions.Node(
            package='ulisse_ctrl', node_executable='dynamic_control_node', output='screen', arguments=[dclconfigfile])
    kcl_node = launch_ros.actions.Node(
            package='ulisse_ctrl', node_executable='kinematic_control_node', output='screen', arguments=[kclconfigfile])
    nav_filter_node = launch_ros.actions.Node(
            package='nav_filter', node_executable='navigation_filter_node', output='screen', arguments=[navconfigfile])

    return LaunchDescription([
        dcl_node,
        kcl_node,
        nav_filter_node
        # more nodes can be added here
    ])
