from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    print('Starting simulator...')
    configfile = '__params:=' + get_package_share_directory('ulisse_sim') + '/conf/simparams.yaml'

    # Node arguments must be comma separated strings
    sim_node = launch_ros.actions.Node(
            package='ulisse_sim', node_executable='simulator', output='screen', arguments=[configfile])

    return LaunchDescription([
        sim_node,
        # more nodes can be added here
    ])
