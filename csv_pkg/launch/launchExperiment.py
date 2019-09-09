from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    print('Starting experiment...')
    # print("Config file: ", configfile)
    # Node arguments must be comma separated strings
    log_control_context = launch_ros.actions.Node(
            package='csv_pkg', node_executable='log_control_context', output='screen')
    log_llc_gps = launch_ros.actions.Node(
            package='csv_pkg', node_executable='log_llc_gps', output='screen')
    log_nav_filter_data = launch_ros.actions.Node(
            package='csv_pkg', node_executable='log_nav_filter_data', output='screen')
    log_status_context = launch_ros.actions.Node(
            package='csv_pkg', node_executable='log_status_context', output='screen')
    log_llc_imu = launch_ros.actions.Node(
            package='csv_pkg', node_executable='log_llc_imu', output='screen')
    log_llc_magnetometer = launch_ros.actions.Node(
            package='csv_pkg', node_executable='log_llc_magnetometer', output='screen')
    log_llc_compass = launch_ros.actions.Node(
            package='csv_pkg', node_executable='log_llc_compass', output='screen')
    log_generic = launch_ros.actions.Node(
            package='csv_pkg', node_executable='log_generic', output='screen')
    log_dcl = launch_ros.actions.Node(
            package='csv_pkg', node_executable='dcl_log', output='screen')


    return LaunchDescription([
        log_control_context,
        log_llc_gps,
        log_nav_filter_data,
        log_status_context,
        log_llc_imu,
        log_llc_magnetometer,
        log_llc_compass,
        log_generic,
        log_dcl
        # more nodes can be added here
    ])
