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
    serviceReq->bound_min = 5;
    serviceReq->bound_max = 3;
    serviceReq->boundaries_json = "{\"name\":\"SecurityPoly\",\"values\":[{\"latitude\":44.40023036224755,\"longitude\":8.93858892352111},{\"latitude\":44.3990433094173,\"longitude\":8.938937899408046},"
                                  "{\"latitude\":44.400062333404875,\"longitude\":8.940667606083906},{\"latitude\":44.40140655066518,\"longitude\":8.940144142221214},{\"latitude\":44.40023036224755,"
                                  "\"longitude\":8.93858892352111}]}";

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

