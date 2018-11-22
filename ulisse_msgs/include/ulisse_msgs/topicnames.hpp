#ifndef TOPICNAMES_HPP
#define TOPICNAMES_HPP

#include <string>
namespace ulisse_msgs {

namespace topicnames {

    const std::string micro_loop_count = "/ees/time/micro_loop_count";
    const std::string sensor_gps_data = "/gps/data";
    const std::string sensor_gps_status = "/gps/status";
    const std::string sensor_compass = "/ees/sensor/compass";
    const std::string sensor_imu = "/ees/sensor/imu";
    const std::string sensor_ambient = "/ees/sensor/ambient";
    const std::string sensor_magnetometer = "/ees/sensor/magnetometer";
    const std::string motor_applied_ref = "/ees/vehicle/applied_reference";
    const std::string ees_status = "/ees/status";
    const std::string ees_config = "/ees/config";
    const std::string ees_motors = "/ees/motors";
    const std::string ees_version = "/ees/version";
    const std::string ees_ack = "/ees/ack";
    const std::string ees_battery_left = "/ees/battery_left";
    const std::string ees_battery_right = "/ees/battery_right";
    const std::string ees_sw485status = "/ees/sw485status";

    const std::string vehicle_ctrl_state = "/vehicle/ctrl_state";
    const std::string control_context = "/vehicle/ctrl_context";
    const std::string position_context = "/vehicle/pos_context";

    const std::string command_halt = "/command/halt";
    const std::string command_move = "/command/move";

    const std::string rosbag_service = "/record/record_bag";
    const std::string ees_cmd_service = "/ees_command";
}
}

#endif // TOPICNAMES_HPP
