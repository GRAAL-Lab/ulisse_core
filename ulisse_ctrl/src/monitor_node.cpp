#include <iomanip>

#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/llc_battery.hpp"
#include "ulisse_msgs/msg/thrusters_data.hpp"

#include "ulisse_msgs/terminal_utils.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ulisse_ctrl/ctrl_data_structs.hpp"

#include "rml/RML.h"

using namespace ulisse;

static ulisse_msgs::msg::GPSData gps_data;
static ulisse_msgs::msg::ThrustersData thrusters_data;
static ulisse_msgs::msg::LLCBattery battery_left;
static ulisse_msgs::msg::LLCBattery battery_right;

void GpsCB(const ulisse_msgs::msg::GPSData::SharedPtr msg);
void ThrustersDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg);

void BatteryLeftCB(const ulisse_msgs::msg::LLCBattery::SharedPtr msg);
void BatteryRightCB(const ulisse_msgs::msg::LLCBattery::SharedPtr msg);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("monitor_node");

    int rate = 10;
    rclcpp::WallRate loop_rate(rate);

//    auto gps_sub = nh->create_subscription<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data, 10, GpsCB);
//    auto poscxt_sub = nh->create_subscription<ulisse_msgs::msg::GoalContext>(ulisse_msgs::topicnames::goal_context, 10, GoalContextCB);
//    auto ctrlcxt_sub = nh->create_subscription<ulisse_msgs::msg::ControlContext>(ulisse_msgs::topicnames::control_context, 10, ControlContextCB);
//    auto statuscxt_sub = nh->create_subscription<ulisse_msgs::msg::StatusContext>(ulisse_msgs::topicnames::status_context, 10, StatusContextCB);
    auto thrusterdata_sub = nh->create_subscription<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data, 10, ThrustersDataCB);
    auto batteryleft_sub = nh->create_subscription<ulisse_msgs::msg::LLCBattery>(ulisse_msgs::topicnames::llc_battery_left, 10, BatteryLeftCB);
    auto batteryright_sub = nh->create_subscription<ulisse_msgs::msg::LLCBattery>(ulisse_msgs::topicnames::llc_battery_right, 10, BatteryRightCB);

    while (rclcpp::ok()) {

        // TODO: Update fields to print!

//        std::cout << tc::white << "GPS time:\t" << tc::none << std::setprecision(10) << gps_data.time << std::endl;
//        std::cout << tc::white << "Vehicle State:\t" << tc::none << status_cxt.vehicle_state << std::endl;

//        std::cout << tc::green << "GPS Pos:\t" << tc::none << gps_data.latitude << ", " << gps_data.longitude << std::endl;
//        std::cout << tc::green << "Filtered Pos:\t" << tc::none << status_cxt.vehicle_pos.latitude << ", " << status_cxt.vehicle_pos.longitude << std::endl;
//        std::cout << tc::green << "Heading:\t" << tc::none << status_cxt.vehicle_heading * 180 / M_PI << std::endl;
//        std::cout << tc::green << "Surge:\t\t" << tc::none << status_cxt.vehicle_speed << std::endl;

//        std::cout << tc::green << "Goal Pos:\t" << tc::none << goal_cxt.current_goal.latitude << ", " << goal_cxt.current_goal.longitude << std::endl;
//        std::cout << tc::green << "Goal Distance:\t" << tc::none << std::setprecision(10) << goal_cxt.goal_distance << std::endl;
//        std::cout << tc::green << "Goal Heading:\t" << tc::none << std::setprecision(10) << goal_cxt.goal_heading << std::endl;

//        std::cout << tc::blu << "Desired Speed:\t" << tc::none << control_cxt.desired_speed << std::endl;
//        std::cout << tc::blu << "Desired Jog:\t" << tc::none << control_cxt.desired_jog << std::endl;

//        std::cout << tc::blu << "Motor Map Out:\t" << tc::none << thrusters_data.motor_mapout.left << ", " << thrusters_data.motor_mapout.right << std::endl;
//        std::cout << tc::blu << "Motor Ctrl Ref:\t" << tc::none << thrusters_data.motor_ctrlref.left << ", " << thrusters_data.motor_ctrlref.right << std::endl;

        std::cout << tc::mag << "Battery Level (L,R): " << tc::none << battery_left.charge_percent << "%, " << battery_right.charge_percent << "%" << std::endl;

        std::cout << "------------------------------------" << std::endl;

        rclcpp::spin_some(nh);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}

void GpsCB(const ulisse_msgs::msg::GPSData::SharedPtr msg) { gps_data = *msg; }

//void GoalContextCB(const ulisse_msgs::msg::GoalContext::SharedPtr msg) { goal_cxt = *msg; }

//void ControlContextCB(const ulisse_msgs::msg::ControlContext::SharedPtr msg) { control_cxt = *msg; }

//void StatusContextCB(const ulisse_msgs::msg::StatusContext::SharedPtr msg) { status_cxt = *msg; }

void ThrustersDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg) { thrusters_data = *msg; }

void BatteryLeftCB(const ulisse_msgs::msg::LLCBattery::SharedPtr msg) { battery_left = *msg; }

void BatteryRightCB(const ulisse_msgs::msg::LLCBattery::SharedPtr msg) { battery_right = *msg; }
