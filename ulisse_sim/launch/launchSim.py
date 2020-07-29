from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    print('Starting simulator...')
    # Node arguments must be comma separated strings
    sim_node = launch_ros.actions.Node(package='ulisse_sim', executable='simulator_node', output='screen', arguments=[])

    return LaunchDescription([sim_node])
