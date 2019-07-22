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
#include "ulisse_msgs/srv/set_boundaries.hpp"

#include "rml/RML.h"

using namespace ulisse;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("spline");

    int rate = 10;
    rclcpp::WallRate loop_rate(rate);

    auto serviceReq = std::make_shared<ulisse_msgs::srv::SetBoundaries::Request>();
    serviceReq->bound_min = 20;
    serviceReq->bound_max = 10;

    /*
    serviceReq->boundaries_json = "{\"name\":\"SecurityPoly\",\"values\":[{\"latitude\":44.393416932512764,\"longitude\":8.94322709917634},{\"latitude\":44.393156728311524,"
                                  "\"longitude\":8.948750022202859},{\"latitude\":44.38738857152622,\"longitude\":8.95281635015084},{\"latitude\":44.37780263284662,\"longitude\":8.955911614706025},"
                                  "{\"latitude\":44.3777158750978,\"longitude\":8.945411991804804},{\"latitude\":44.389210156133196,\"longitude\":8.946565129574338},{\"latitude\":44.38669461961795,"
                                  "\"longitude\":8.934244762819674},{\"latitude\":44.391248528844066,\"longitude\":8.935094443280292},{\"latitude\":44.39116179102005,\"longitude\":8.941224280924615},"
                                  "{\"latitude\":44.393416932512764,\"longitude\":8.94322709917634}]}";
    */

    serviceReq->boundaries_json = "{\"name\":\"SecurityPoly\",\"values\":[{\"latitude\":44.3925,\"longitude\":8.94373},{\"latitude\":44.3926,\"longitude\":8.94629}"
                                   ",{\"latitude\":44.3942,\"longitude\":8.94508},{\"latitude\":44.3925,\"longitude\":8.94373}]}";



    std::string result_msg;

    auto command_srv_ = nh->create_client<ulisse_msgs::srv::SetBoundaries>(ulisse_msgs::topicnames::set_boundaries_service);

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

    rclcpp::shutdown();

    return 0;
}

