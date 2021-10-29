#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/terminal_utils.hpp"
#include "ulisse_ctrl/dynamic_vehicle_controller.hpp"
#include "rml/RML.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    int rate = 10;

    // Name of conf file
    std::string filename = "dcl_ulisse.conf";
    auto dynamicController = std::make_shared<ulisse::DynamicVehicleController>(rate, filename);

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(dynamicController);
    exe.spin();

    rclcpp::shutdown();

    return 0;
}
