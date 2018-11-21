#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ulisse_msgs/msg/control_context.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/position_context.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ulisse_ctrl/data_structs.hpp"

#include "rml/RML.h"

static ulisse_msgs::msg::GPSData gps_msg;
static ulisse_msgs::msg::PositionContext position_cxt;
static ulisse_msgs::msg::ControlContext control_cxt;
static std_msgs::msg::String vehicle_state;

void GPS_cb(const ulisse_msgs::msg::GPSData::SharedPtr msg);
void PositionContext_cb(const ulisse_msgs::msg::PositionContext::SharedPtr msg);
void ControlContext_cb(const ulisse_msgs::msg::ControlContext::SharedPtr msg);
void VehicleState_cb(const std_msgs::msg::String::SharedPtr msg);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("monitor_node");

    int rate = 10;
    rclcpp::WallRate loop_rate(rate);

    rclcpp::Subscription<ulisse_msgs::msg::GPSData>::SharedPtr gps_sub;
    rclcpp::Subscription<ulisse_msgs::msg::PositionContext>::SharedPtr poscxt_sub;
    rclcpp::Subscription<ulisse_msgs::msg::ControlContext>::SharedPtr ctrlcxt_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vehiclestate_sub;

    gps_sub = nh->create_subscription<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data, GPS_cb);
    poscxt_sub = nh->create_subscription<ulisse_msgs::msg::PositionContext>(
        ulisse_msgs::topicnames::position_context, PositionContext_cb);
    ctrlcxt_sub = nh->create_subscription<ulisse_msgs::msg::ControlContext>(
        ulisse_msgs::topicnames::position_context, ControlContext_cb);
    vehiclestate_sub = nh->create_subscription<std_msgs::msg::String>(
        ulisse_msgs::topicnames::position_context, VehicleState_cb);

    vehicle_state.data = "undefined";

    while (rclcpp::ok()) {

        std::cout << tc::white << "GPS time:\t" << tc::none << std::setprecision(10) << gps_msg.time << std::endl;
        std::cout << tc::white << "Vehicle State:\t" << tc::none << vehicle_state.data << std::endl;

        std::cout << tc::green << "Current Pos:\t" << tc::none << position_cxt.currentpos.latitude << ", " << position_cxt.currentpos.longitude << std::endl;
        std::cout << tc::green << "Goal Pos:\t" << tc::none << position_cxt.currentgoal.latitude << ", " << position_cxt.currentgoal.longitude << std::endl;
        std::cout << tc::green << "Current Heading:" << tc::none << position_cxt.currentheading << std::endl;

        std::cout << tc::green << "goal_distance:\t" << tc::none << std::setprecision(10) << position_cxt.goaldistance << std::endl;
        std::cout << tc::green << "goal_heading:\t" << tc::none << std::setprecision(10) << position_cxt.goalheading << std::endl;

        std::cout << tc::blu << "Desired Speed:\t" << tc::none << control_cxt.pidspeed.reference << std::endl;
        std::cout << tc::blu << "Desired Jog:\t" << tc::none << control_cxt.pidheading.reference << std::endl;

        std::cout << tc::blu << "Motor Map Out:\t" << tc::none << control_cxt.mapout.left << ", " << control_cxt.mapout.right << std::endl;
        std::cout << tc::blu << "Motor Ctrl Ref:\t" << tc::none << control_cxt.ctrlref.left << ", " << control_cxt.ctrlref.right << std::endl;

        std::cout << "------------------------------------" << std::endl;

        rclcpp::spin_some(nh);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}

void GPS_cb(const ulisse_msgs::msg::GPSData::SharedPtr msg)
{
    gps_msg = *msg;
}

void PositionContext_cb(const ulisse_msgs::msg::PositionContext::SharedPtr msg)
{
    position_cxt = *msg;
}

void ControlContext_cb(const ulisse_msgs::msg::ControlContext::SharedPtr msg)
{
    control_cxt = *msg;
}

void VehicleState_cb(const std_msgs::msg::String::SharedPtr msg)
{
    vehicle_state = *msg;
}
