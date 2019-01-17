#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ulisse_msgs/msg/control_context.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/position_context.hpp"
#include "ulisse_msgs/msg/ees_battery.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include "ulisse_ctrl/terminal_utils.hpp"

#include "rml/RML.h"

using namespace ulisse;

static ulisse_msgs::msg::GPSData gps_data;
static ulisse_msgs::msg::PositionContext position_cxt;
static ulisse_msgs::msg::ControlContext control_cxt;
static std_msgs::msg::String vehicle_state;
static ulisse_msgs::msg::EESBattery battery_left;
static ulisse_msgs::msg::EESBattery battery_right;


void GPS_cb(const ulisse_msgs::msg::GPSData::SharedPtr msg);
void PositionContext_cb(const ulisse_msgs::msg::PositionContext::SharedPtr msg);
void ControlContext_cb(const ulisse_msgs::msg::ControlContext::SharedPtr msg);
void VehicleState_cb(const std_msgs::msg::String::SharedPtr msg);
void BatteryLeft_cb(const ulisse_msgs::msg::EESBattery::SharedPtr msg);
void BatteryRight_cb(const ulisse_msgs::msg::EESBattery::SharedPtr msg);


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("monitor_node");

    int rate = 10;
    rclcpp::WallRate loop_rate(rate);

    /*rclcpp::Subscription<ulisse_msgs::msg::GPSData>::SharedPtr gps_sub;
    rclcpp::Subscription<ulisse_msgs::msg::PositionContext>::SharedPtr poscxt_sub;
    rclcpp::Subscription<ulisse_msgs::msg::ControlContext>::SharedPtr ctrlcxt_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vehiclestate_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr batteryleft_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr batteryright_sub;*/

    auto gps_sub = nh->create_subscription<ulisse_msgs::msg::GPSData>(
        ulisse_msgs::topicnames::sensor_gps_data, GPS_cb);
    auto poscxt_sub = nh->create_subscription<ulisse_msgs::msg::PositionContext>(
        ulisse_msgs::topicnames::position_context, PositionContext_cb);
    auto ctrlcxt_sub = nh->create_subscription<ulisse_msgs::msg::ControlContext>(
        ulisse_msgs::topicnames::control_context, ControlContext_cb);
    auto vehiclestate_sub = nh->create_subscription<std_msgs::msg::String>(
        ulisse_msgs::topicnames::vehicle_ctrl_state, VehicleState_cb);

    auto batteryleft_sub = nh->create_subscription<ulisse_msgs::msg::EESBattery>(
        ulisse_msgs::topicnames::ees_battery_left, BatteryLeft_cb);
    auto batteryright_sub = nh->create_subscription<ulisse_msgs::msg::EESBattery>(
        ulisse_msgs::topicnames::ees_battery_right, BatteryRight_cb);

    vehicle_state.data = "undefined";

    while (rclcpp::ok()) {

        std::cout << tc::white << "GPS time:\t" << tc::none << std::setprecision(10) << gps_data.time << std::endl;
        std::cout << tc::white << "Vehicle State:\t" << tc::none << vehicle_state.data << std::endl;

        std::cout << tc::green << "GPS Pos:\t" << tc::none << gps_data.latitude << ", " << gps_data.longitude << std::endl;
        std::cout << tc::green << "Filtered Pos:\t" << tc::none << position_cxt.filtered_pos.latitude << ", " << position_cxt.filtered_pos.longitude << std::endl;
        std::cout << tc::green << "Heading:\t" << tc::none << position_cxt.current_heading << std::endl;

        std::cout << tc::green << "Goal Pos:\t" << tc::none << position_cxt.current_goal.latitude << ", " << position_cxt.current_goal.longitude << std::endl;
        std::cout << tc::green << "Goal Distance:\t" << tc::none << std::setprecision(10) << position_cxt.goal_distance << std::endl;
        std::cout << tc::green << "Goal Heading:\t" << tc::none << std::setprecision(10) << position_cxt.goal_heading << std::endl;

        std::cout << tc::blu << "Desired Speed:\t" << tc::none << control_cxt.desired_speed << std::endl;
        std::cout << tc::blu << "Desired Jog:\t" << tc::none << control_cxt.desired_jog << std::endl;

        std::cout << tc::blu << "Motor Map Out:\t" << tc::none << control_cxt.motor_mapout.left << ", " << control_cxt.motor_mapout.right << std::endl;
        std::cout << tc::blu << "Motor Ctrl Ref:\t" << tc::none << control_cxt.motor_ctrlref.left << ", " << control_cxt.motor_ctrlref.right << std::endl;

        std::cout << tc::mag << "Battery Level (L,R): " << tc::none << battery_left.charge_percent << "%, " << battery_right.charge_percent << "%" << std::endl;

        std::cout << "------------------------------------" << std::endl;

        rclcpp::spin_some(nh);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}

void GPS_cb(const ulisse_msgs::msg::GPSData::SharedPtr msg)
{
    gps_data = *msg;
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

void BatteryLeft_cb(const ulisse_msgs::msg::EESBattery::SharedPtr msg)
{
    battery_left = *msg;
}

void BatteryRight_cb(const ulisse_msgs::msg::EESBattery::SharedPtr msg)
{
    battery_right = *msg;
}


