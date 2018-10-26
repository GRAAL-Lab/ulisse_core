#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include <ulisse_ctrl/vehiclecontroller.hpp>

#include "rml/RML.h"

#include <chrono>

using namespace std::chrono_literals;

static rclcpp::Node::SharedPtr g_node = nullptr;

void gps_callback(const ulisse_msgs::msg::GPS::SharedPtr msg)
{
    RCLCPP_INFO(g_node->get_logger(), "I heard: 'time:%f, lat:%f, long:%f'", msg->time, msg->latitude, msg->longitude)
}

int main(int argc, char* argv[])
{
    //    std::cout << "Argv Test:" << std::endl;
    //    for (int i = 1; i < argc; i++) {
    //        std::cout << "argv[" << i << "]: " << std::string(argv[i]) << std::endl;
    //    }
    //    std::cout << std::endl;
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("controller_node");

    auto gps_sub = g_node->create_subscription<ulisse_msgs::msg::GPS>(ulisse_msgs::topicnames::sensor_gps, gps_callback);

    rclcpp::WallRate loop_rate(10ms);

    Eigen::TransfMatrix wTv;
    rml::RobotModel myModel(wTv, "myVehicle");

    ulisse::VehicleController myVC(g_node);

    while (rclcpp::ok()) {

        myVC.Run();

        rclcpp::spin_some(g_node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    // TODO(clalancette): It would be better to remove both of these nullptr
    // assignments and let the destructors handle it, but we can't because of
    // https://github.com/eProsima/Fast-RTPS/issues/235 .  Once that is fixed
    // we should probably look at removing these two assignments.
    gps_sub = nullptr;
    g_node = nullptr;

    return 0;
}
