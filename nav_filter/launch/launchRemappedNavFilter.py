from launch import LaunchDescription
import launch_ros.actions
import os

os.environ['RCUTILS_COLORIZED_OUTPUT'] = "1"

def generate_launch_description():

    print('Starting Controller...')
    # Node arguments must be comma separated strings
    nav_filter_node_aux = launch_ros.actions.Node(
            package='nav_filter',
            executable='nav_filter_node',
            name='nav_filter_node_aux',
            remappings=[
                ('/ulisse/nav_filter/data', '/ulisse/nav_filter/data_auxiliary'),
                # more remappings can be added here
            ],
            output='screen',
            arguments=[])
    
    return LaunchDescription([
        nav_filter_node_aux
        # more nodes can be added here
    ])

