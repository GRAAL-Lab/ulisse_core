#ifndef TOPICNAMES_HPP
#define TOPICNAMES_HPP

#include <string>
namespace ulisse_msgs {

namespace topicnames {

    const std::string micro_loop_count = "/ulisse/llc/micro_loop_count";

    const std::string sensor_gps_data = "/ulisse/llc/sensor/gps_data";
    const std::string sensor_gps_status = "/ulisse/llc/sensor/gps_status";
    const std::string sensor_compass = "/ulisse/llc/sensor/compass";
    const std::string sensor_imu = "/ulisse/llc/sensor/imu";
    const std::string sensor_ambient = "/ulisse/llc/sensor/ambient";
    const std::string sensor_magnetometer = "/ulisse/llc/sensor/magnetometer";

    const std::string motor_applied_ref = "/ulisse/llc/motor_applied_ref";
    const std::string llc_status = "/ulisse/llc/status";
    const std::string llc_config = "/ulisse/llc/config";
    const std::string llc_motors = "/ulisse/llc/motors";
    const std::string llc_version = "/ulisse/llc/version";
    const std::string llc_ack = "/ulisse/llc/ack";
    const std::string llc_battery_left = "/ulisse/llc/battery_left";
    const std::string llc_battery_right = "/ulisse/llc/battery_right";
    const std::string llc_sw485status = "/ulisse/llc/sw485status";

    const std::string nav_filter_data = "/ulisse/nav_filter/data";

    const std::string control_context = "/ulisse/ctrl/ctrl_context";
    const std::string status_context = "/ulisse/ctrl/status_context";
    const std::string goal_context = "/ulisse/ctrl/pos_context";
    const std::string thrusters_data = "/ulisse/ctrl/thruster_ref";

    const std::string rosbag_service = "/record_bag";
    const std::string llc_cmd_service = "/ulisse/service/llc_cmd";
    const std::string control_cmd_service = "/ulisse/service/control_cmd";
    const std::string navfilter_cmd_service = "/ulisse/service/navfilter_cmd";
    const std::string set_boundaries_service = "/ulisse/ctrl/set_boundaries";

    const std::string set_cruise_control_service = "/ulisse/ctrl/set_cruise_control";
    const std::string reset_configuration_service = "/ulisse/ctrl/reset_configuration";
}
}

#endif // TOPICNAMES_HPP
