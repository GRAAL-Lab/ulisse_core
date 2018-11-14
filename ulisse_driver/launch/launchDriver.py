from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    print('Starting simulator...')
    configfile = '__params:=' + get_package_share_directory('ulisse_driver') + '/conf/ulisse_driver.yaml'
    print("Config file: ", configfile)
    # Node arguments must be comma separated strings
    driver_node = launch_ros.actions.Node(
            package='ulisse_driver', node_executable='driver_node', output='screen', arguments=[configfile])

    return LaunchDescription([
        driver_node,
        # more nodes can be added here
    ])
