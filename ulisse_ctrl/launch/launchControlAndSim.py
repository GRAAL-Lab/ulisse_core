from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    print('Starting controller...')
    ctrlConfigfile = '__params:=' + get_package_share_directory('ulisse_ctrl') + '/conf/ulisse.yaml'
    print("Controller config file: ", ctrlConfigfile)
    
    print('Starting simulator...')
    simConfigfile = '__params:=' + get_package_share_directory('ulisse_sim') + '/conf/simparams.yaml'
    print("Simulator config file: ", simConfigfile)
    
    # Node arguments must be comma separated strings
    ctrl_node = launch_ros.actions.Node(package='ulisse_ctrl', executable='controller_node', output='screen', arguments=[ctrlConfigfile])
    sim_node = launch_ros.actions.Node(package='ulisse_sim', executable='simulator_node', output='screen', arguments=[simConfigfile])        
            

    return LaunchDescription([
        sim_node,
        ctrl_node
        # more nodes can be added here
    ])
