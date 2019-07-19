#include <iomanip>

#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/control_context.hpp"
#include "ulisse_msgs/msg/goal_context.hpp"
#include "ulisse_msgs/msg/status_context.hpp"
#include "ulisse_msgs/msg/llc_battery.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/thrusters_data.hpp"

#include "ulisse_msgs/srv/control_command.hpp"
#include "ulisse_msgs/srv/control_command.h"

#include "ulisse_msgs/terminal_utils.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"

#include "rml/RML.h"
#include "ulisse_ctrl/concave_segments.h"

using namespace ulisse;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("spline");

    int rate = 10;
    rclcpp::WallRate loop_rate(rate);


    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    serviceReq->command_type = ulisse::commands::ID::navigate;
    serviceReq->nav_cmd.centroid_latitude = 44.3931;
    serviceReq->nav_cmd.centroid_longitude = 8.94382;

    serviceReq->nav_cmd.number_of_curves = 1;

    std::vector<std::string> curves;

    curves.push_back("{ \"degree\": 1, \"points\": [[44.393, 8.945], [44.3935, 8.9462]], \"weigths\": [1, 1], \"knots\": [0, 0, 1, 1], \"reverse\": 0}");
    //curves.push_back("{ \"degree\": 3, \"points\": [[0, 2], [1, 3], [4, 7], [3, 6]], \"weigths\": [1, 1/3.0, 1/3.0, 1], \"knots\": [0, 0, 0, 0, 1, 1, 1, 1] }");
    //curves.push_back("{ \"degree\": 1, \"points\": [[0, 10], [5, 5]], \"weigths\": [1, 1], \"knots\": [0, 0, 1, 1] }");

    serviceReq->nav_cmd.curves = curves;

    std::string result_msg;

    auto command_srv_ = nh->create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service);

    while (rclcpp::ok()) {

        if (command_srv_->service_is_ready())
            break;
    }

    if (command_srv_->service_is_ready()) {
        auto result_future = command_srv_->async_send_request(serviceReq);
        std::cout << "Sent Request to controller" << std::endl;
        if (rclcpp::spin_until_future_complete(nh, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS) {
            result_msg = "service call failed :(";
            RCLCPP_ERROR(nh->get_logger(), result_msg.c_str());
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + result->res;
            RCLCPP_INFO(nh->get_logger(), result_msg.c_str());
        }
    } else {
        result_msg = "No Command Server Available";
        std::cout << "No Command Server Available" << std::endl;
    }

    while (rclcpp::ok()) {

    }

    rclcpp::shutdown();

    return 0;
}

