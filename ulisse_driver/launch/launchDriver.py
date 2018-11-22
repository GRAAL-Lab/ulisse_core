from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    print('Starting driver...')
    configfile = '__params:=' + get_package_share_directory('ulisse_driver') + '/conf/ulisse_driver.yaml'
    print("Config file: ", configfile)
    # Node arguments must be comma separated strings
    ees_driver_node = launch_ros.actions.Node(
            package='ulisse_driver', node_executable='ees_driver_node', output='screen', arguments=[configfile])
    gps_setup_node = launch_ros.actions.Node(
            package='ulisse_driver', node_executable='gps_setup_node', output='screen', arguments=[configfile])
    gps_publisher_node = launch_ros.actions.Node(
            package='ulisse_driver', node_executable='gps_publisher_node', output='screen', arguments=[configfile])

    return LaunchDescription([
        ees_driver_node,
        #gps_setup_node,
        gps_publisher_node,
    ])
