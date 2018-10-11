from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    print('Starting publisher and subscriber...')
    
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ulisse_sim', node_executable='simulator', output='screen',
            arguments=["__params:=src/ulisse_core/ulisse_sim/conf/simparams.yaml"]),
])
