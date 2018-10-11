#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/motor_reference.hpp"

#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_sim/vehiclesimulator.h"

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <random>

using namespace std::chrono_literals;

void ReadMappingParameters(const std::shared_ptr<rclcpp::SyncParametersClient> pc, ThrusterMappingParameters &tmp);

int main(int argc, char* argv[])
{
    // Force flush of the stdout buffer.
    //setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("simulator");

    auto publisher = node->create_publisher<std_msgs::msg::String>("topic");
    auto message = std::make_shared<std_msgs::msg::String>();

    auto compass_pub = node->create_publisher<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::compass_sensor);
    auto compass_msg = std::make_shared<ulisse_msgs::msg::Compass>();

    rclcpp::WallRate loop_rate(500ms);

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
    while (!parameters_client->wait_for_service(1ms)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.")
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "service not available, waiting again...")
    }

    double dt = parameters_client->get_parameter("sim_params.dt", 0.1);
    std::cout << "dt=" << dt << std::endl;

    ThrusterMappingParameters myTMP;
    ReadMappingParameters(parameters_client, myTMP);

    VehicleSimulator myVehSim;
    myVehSim.SetParameters(dt, myTMP);

    auto publish_count = 0;
    /*std::default_random_engine generator;
            std::uniform_real_distribution<double> distribution(0.0, 2.0 * M_PI);
            auto random_compass = std::bind(distribution, generator);*/

    while (rclcpp::ok()) {
        message->data = "Hello, world! " + std::to_string(publish_count++);
        RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message->data.c_str())
        publisher->publish(message);

        /*compass_msg->yaw   = (float)random_compass();
        compass_msg->pitch = (float)random_compass();
        compass_msg->roll  = (float)random_compass();
        compass_pub->publish(compass_msg);*/

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}


void ReadMappingParameters(const std::shared_ptr<rclcpp::SyncParametersClient> pc, ThrusterMappingParameters &tmp)
{
    tmp.lambda_pos = pc->get_parameter("thruster_mapping.lambda_pos", 0.0);
    tmp.lambda_neg = pc->get_parameter("thruster_mapping.lambda_neg", 0.0);
    tmp.cb         = Eigen::Vector4d((pc->get_parameter("thruster_mapping.cb", std::vector<double>(4, 0.0))).data());
    tmp.cX         = Eigen::Vector3d((pc->get_parameter("thruster_mapping.cX", std::vector<double>(3, 0.0))).data());
    tmp.cN         = Eigen::Vector3d((pc->get_parameter("thruster_mapping.cN", std::vector<double>(3, 0.0))).data());
    tmp.b1_pos     = pc->get_parameter("thruster_mapping.b1_pos", 0.0);
    tmp.b2_pos     = pc->get_parameter("thruster_mapping.b2_pos", 0.0);
    tmp.b1_neg     = pc->get_parameter("thruster_mapping.b1_neg", 0.0);
    tmp.b2_neg     = pc->get_parameter("thruster_mapping.b2_neg", 0.0);
    tmp.Inertia.diagonal()    = Eigen::Vector3d((pc->get_parameter("thruster_mapping.Inertia", std::vector<double>(3, 0.0))).data());

}


/*
// Set several different types of parameters.
auto set_parameters_results = parameters_client->set_parameters({
    rclcpp::Parameter("foo", 2),
    rclcpp::Parameter("bar", "hello"),
    rclcpp::Parameter("baz", 1.45),
    rclcpp::Parameter("foobar", true),
    rclcpp::Parameter("foobarbaz", std::vector<bool>({ true, false })),
    rclcpp::Parameter("toto", std::vector<uint8_t>({ 0xff, 0x7f })),
});

// Check to see if they were set.
for (auto& result : set_parameters_results) {
    if (!result.successful) {
        RCLCPP_ERROR(node->get_logger(), "Failed to set parameter: %s", result.reason.c_str())
    }
}

std::stringstream ss;
// Get a few of the parameters just set.
for (auto& parameter : parameters_client->get_parameters({ "foo", "baz", "foobarbaz", "toto" })) {
    ss << "\nParameter name: " << parameter.get_name();
    ss << "\nParameter value (" << parameter.get_type_name() << "): " << parameter.value_to_string();
}
RCLCPP_INFO(node->get_logger(), ss.str().c_str()) */
