#ifndef TOPICNAMES_HPP
#define TOPICNAMES_HPP

#include <string>
namespace ulisse_msgs {

namespace topicnames {

    const std::string time_info = "/time/info";
    const std::string sensor_gps = "/sensor/gps";
    const std::string sensor_compass = "/sensor/compass";
    const std::string sensor_imu = "/sensor/imu";
    const std::string sensor_ambient = "/sensor/ambient";
    const std::string sensor_magnetometer = "/sensor/magnetometer";
    const std::string motor_ctrl_ref = "/motor/ctrl_reference";
    const std::string motor_applied_ref = "/motor/applied_reference";

    const std::string command_halt = "/command/halt";

}
}

#endif // TOPICNAMES_HPP
