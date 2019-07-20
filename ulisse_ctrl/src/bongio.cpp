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
    serviceReq->nav_cmd.nurbs_json = "{\"centroid\":[44.39202059265811,8.9439365547717],\"curves\":["
                                     "{\"degree\":1,\"knots\":[0,0,1,1],\"points\":[[18.78383769505253,-12.24823752552843],[27.451122566636002,-7.244178271776541]],\"weigths\":[1,1]},"
                                     "{\"degree\":1,\"knots\":[0,0,1,1],\"points\":[[-8.672366134091167,-16.553538559619152],[22.528601390628666,1.460348439754616]],\"weigths\":[1,1]},"
                                     "{\"degree\":1,\"knots\":[0,0,1,1],\"points\":[[-13.594887310098501,-7.849011848087996],[20.585018549962264,11.884766001094151]],\"weigths\":[1,1]},"
                                     "{\"degree\":1,\"knots\":[0,0,1,1],\"points\":[[-18.88993247421921,0.64043803862631],[15.662497373954928,20.58929271262531]],\"weigths\":[1,1]}]}";

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

