from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    print('Starting GPS Setup...')
    configfile = get_package_share_directory('ulisse_driver') + '/conf/ulisse_driver.yaml'
    # print("Config file: ", configfile)
    
    # Node arguments must be comma separated strings
    gps_setup_node = Node(
        package='ulisse_driver',
        executable='gps_setup_node',
        name='gps_setup_node',
        output='screen',
        emulate_tty=True,
        parameters=[]
    )

    return LaunchDescription([
        gps_setup_node,
    ])
    
