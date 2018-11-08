#ifndef TOPICNAMES_HPP
#define TOPICNAMES_HPP

#include <string>
namespace ulisse_msgs {

namespace topicnames {

    const std::string micro_loop_count = "/time/micro_loop_count";
    const std::string sensor_gps = "/sensor/gps";
    const std::string sensor_compass = "/sensor/compass";
    const std::string sensor_imu = "/sensor/imu";
    const std::string sensor_ambient = "/sensor/ambient";
    const std::string sensor_magnetometer = "/sensor/magnetometer";
    const std::string motor_applied_ref = "/vehicle/applied_reference";

    const std::string vehicle_ctrl_state = "/vehicle/ctrl_state";
    const std::string control_context = "/vehicle/ctrl_context";
    const std::string position_context = "/vehicle/pos_context";

    const std::string command_halt = "/command/halt";
    const std::string command_move = "/command/move";
}
}

#endif // TOPICNAMES_HPP
