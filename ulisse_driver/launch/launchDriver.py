from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    #print('Starting driver...')
    #configfile = get_package_share_directory('ulisse_driver') + '/conf/ulisse_driver.yaml'
    #print("Config file: ", configfile)
    
    # Node arguments must be comma separated strings
    llc_driver_node = launch_ros.actions.Node(
            package='ulisse_driver',
            executable='llc_driver_node',
            output='screen',
            arguments=[])
    gps_publisher_node = launch_ros.actions.Node(
            package='ulisse_driver',
            executable='gps_publisher_node',
            output='screen',
            arguments=[])

    return LaunchDescription([
        llc_driver_node,
        gps_publisher_node,
    ])
