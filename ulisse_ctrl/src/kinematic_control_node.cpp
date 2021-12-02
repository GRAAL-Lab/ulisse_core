#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/terminal_utils.hpp"
#include "ulisse_ctrl/kinematic_vehicle_controller.hpp"
#include "rml/RML.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::string filename = "kcl_ulisse.conf";
    auto vehicleController = std::make_shared<ulisse::VehicleController>(filename);

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(vehicleController);
    exe.spin();

    rclcpp::shutdown();

    return 0;
}
