#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps.hpp"
#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_msgs/msg/motor_reference.hpp"

#include <ulisse_ctrl/vehiclecontroller.hpp>

#include "rml/RML.h"

#include <chrono>

using namespace std::chrono_literals;

static rclcpp::Node::SharedPtr g_node = nullptr;

int main(int argc, char* argv[])
{
    //    std::cout << "Argv Test:" << std::endl;
    //    for (int i = 1; i < argc; i++) {
    //        std::cout << "argv[" << i << "]: " << std::string(argv[i]) << std::endl;
    //    }
    //    std::cout << std::endl;
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("controller_node");

    rclcpp::WallRate loop_rate(10ms);

    Eigen::TransfMatrix wTv;
    auto myModel = std::make_shared<rml::RobotModel>(wTv, "myVehicle");

    ulisse::VehicleController myVC(g_node);

    while (rclcpp::ok()) {

        myVC.Run();
        myVC.PublishControl();

        rclcpp::spin_some(g_node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    // TODO(clalancette): It would be better to remove both of these nullptr
    // assignments and let the destructors handle it, but we can't because of
    // https://github.com/eProsima/Fast-RTPS/issues/235 .  Once that is fixed
    // we should probably look at removing these two assignments.
    g_node = nullptr;

    return 0;
}
