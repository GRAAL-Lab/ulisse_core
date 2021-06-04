#include <rclcpp/rclcpp.hpp>

#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "nav_filter/futils.hpp"

struct NavFilterData {
    double inertialF_linearPosition[2];
    double bodyF_angularPosition[3];
    double bodyF_linearVelocity[3];
    double bodyF_angularVelocity[3];
    double inertialF_waterCurrent[2];
};

NavFilterData navFilterData;
ulisse_msgs::msg::NavFilterData navFilterMsg;
bool dataReceived = false;

void NavFilterCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg) {
    navFilterMsg = *msg;
    std::cout << "Received ROS2 NavFilter message" << std::endl;
    dataReceived = true;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("nav_filter_udp_sender");
    rclcpp::WallRate loop_rate(10);

    rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr navFilterSub_;
    navFilterSub_ = node->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10, NavFilterCB);

    std::string ip, port;
    ip = "130.251.6.42";
    port = "8888";

    futils::UDPSenderSocket udpSender(ip.c_str(), port.c_str());

    while (rclcpp::ok()) {

        if(dataReceived){
            dataReceived = false;

            navFilterData.inertialF_linearPosition[0] = navFilterMsg.inertialframe_linear_position.latlong.latitude;
            navFilterData.inertialF_linearPosition[1] = navFilterMsg.inertialframe_linear_position.latlong.longitude;

            navFilterData.bodyF_angularPosition[0] = 0.0;
            navFilterData.bodyF_angularPosition[1] = 0.0;
            navFilterData.bodyF_angularPosition[2] = navFilterMsg.bodyframe_angular_position.yaw;

            navFilterData.bodyF_linearVelocity[0] = navFilterMsg.bodyframe_linear_velocity[0];
            navFilterData.bodyF_linearVelocity[1] = navFilterMsg.bodyframe_linear_velocity[1];
            navFilterData.bodyF_linearVelocity[2] = navFilterMsg.bodyframe_linear_velocity[2];

            navFilterData.bodyF_angularVelocity[0] = navFilterMsg.bodyframe_angular_velocity[0];
            navFilterData.bodyF_angularVelocity[1] = navFilterMsg.bodyframe_angular_velocity[1];
            navFilterData.bodyF_angularVelocity[2] = navFilterMsg.bodyframe_angular_velocity[2];

            navFilterData.inertialF_waterCurrent[0] = navFilterMsg.inertialframe_water_current[0];
            navFilterData.inertialF_waterCurrent[1] = navFilterMsg.inertialframe_water_current[1];

            udpSender.Send(&navFilterData, sizeof(navFilterData));
            std::cout << "Sent UDP navFilterData message" << std::endl;
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
