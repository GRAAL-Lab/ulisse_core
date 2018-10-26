#ifndef ULISSE_CTRL_VEHICLECONTROLLER_HPP
#define ULISSE_CTRL_VEHICLECONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"

#include "surface_vehicle_model/surfacevehiclemodel.hpp"

#include "std_msgs/msg/empty.hpp"
#include "ulisse_msgs/msg/command_move.hpp"

#include "ulisse_msgs/msg/ambient_sensors.hpp"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/motor_reference.hpp"
#include "ulisse_msgs/msg/time_info.hpp"


#include "GeographicLib/Geodesic.hpp"
#include "eigen3/Eigen/Dense"
#include "rml/RML.h"

#include <ulisse_ctrl/commands/commandhalt.hpp>
#include <ulisse_ctrl/commands/commandmove.hpp>
#include <ulisse_ctrl/states/statehalt.hpp>
#include <ulisse_ctrl/states/statemove.hpp>

namespace ulisse {
class VehicleController {
    rclcpp::Node::SharedPtr nh_;
    SurfaceVehicleModel ulisseModel_;

    GeographicLib::Geodesic geod_;

    // FSM
    fsm::FSM u_fsm_;
    ulisse::states::StateHalt state_halt_;
    ulisse::states::StateMove state_move_;
    ulisse::commands::CommandHalt command_halt_;
    ulisse::commands::CommandMove command_move_;



    void SetUpFSM();

    void CommandHalt_cb(const std_msgs::msg::Empty::SharedPtr msg);
    void CommandMove_cb(const ulisse_msgs::msg::CommandMove::SharedPtr msg);

public:
    VehicleController(const rclcpp::Node::SharedPtr& nh);
    virtual ~VehicleController();
    void Run();
};
}
#endif // ULISSE_CTRL_VEHICLECONTROLLER_HPP
