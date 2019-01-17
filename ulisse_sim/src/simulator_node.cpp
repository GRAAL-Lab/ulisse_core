#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/control_context.hpp"
#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_sim/vehiclesimulator.hpp"

#include <chrono>
#include <cmath>
#include <ctime> // localtime
#include <fstream>
#include <functional>
#include <iomanip> // put_time
#include <iostream>
#include <random>
#include <sstream> // stringstream

using namespace std::chrono_literals;

static double test_h_p(0.0), test_h_s(0.0);

void ReadMappingParameters(const std::shared_ptr<rclcpp::SyncParametersClient> pc, ThrusterMappingParameters& tmp);

void ctrlCtx_cb(const ulisse_msgs::msg::ControlContext::SharedPtr msg)
{
    test_h_p = msg->motor_ctrlref.left;
    test_h_s = msg->motor_ctrlref.right;
}

void motorref_cb(const ulisse_msgs::msg::MotorReference::SharedPtr msg)
{
    test_h_p = msg->left;
    test_h_s = msg->right;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("simulator_node");
    auto subscriber = node->create_subscription<ulisse_msgs::msg::ControlContext>(
        ulisse_msgs::topicnames::control_context, ctrlCtx_cb);
    auto motorref_sub = node->create_subscription<ulisse_msgs::msg::MotorReference>(
        ulisse_msgs::topicnames::motorref, motorref_cb);
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);

    while (!parameters_client->wait_for_service(1ms)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }
    ThrusterMappingParameters myTMP;
    ReadMappingParameters(parameters_client, myTMP);

    int rate = 100;
    rclcpp::WallRate loop_rate(rate);
    double dt = 1.0 / rate;
    std::cout << "dt=" << dt << std::endl;

    ulisse::VehicleSimulator myVehSim(node);
    myVehSim.SetParameters(dt * 5.0, myTMP);
    //myVehSim.SetRealtime(false);

    /*std::stringstream ss;
    // Get a few of the parameters just set.
    for (auto& parameter : parameters_client->get_parameters(
             { "thruster_mapping.motors_distance", "thruster_mapping.lambda_pos", "thruster_mapping.lambda_neg" })) {
        ss << "\nParameter name: " << parameter.get_name();
        ss << "\nParameter value (" << parameter.get_type_name() << "): " << parameter.value_to_string();
    }
    RCLCPP_INFO(node->get_logger(), ss.str().c_str());*/

    // auto publish_count = 0;
    /*std::default_random_engine generator;
            std::uniform_real_distribution<double> distribution(0.0, 2.0 * M_PI);
            auto random_compass = std::bind(distribution, generator);*/

    std::cout.precision(3);
    std::cout << std::fixed;

    std::stringstream logss;
    std::ofstream logfile;

    std::stringstream datess;
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    datess << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H.%M.%S");

    std::string logfilename = "/home/graal/logs/sim/sim_log_" + datess.str() + ".txt";
    std::cout << "* Saving log to: \"" << logfilename << "\" *" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    logfile.open(logfilename, std::ios_base::app);
    logss << "Lat Long, Yaw, Velocity (world)" << std::endl;
    logfile << logss.str();

    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", ", ", "", "", "", "");

    while (rclcpp::ok()) {

        std::cout << "----------------------------------" << std::endl;
        std::cout << "time: " << std::setprecision(1) << myVehSim.GetCurrentTimestamp() << std::endl;
        std::cout << "lat, long: " << std::setprecision(8) << myVehSim.VehLatitude() << ", " << myVehSim.VehLongitude()
                  << std::endl;
        std::cout << "compass (deg): " << myVehSim.VehAtt().GetYaw() * 180.0 / M_PI << std::endl;
        std::cout << "velocity: " << myVehSim.VehVel_world().transpose() << std::endl;
        std::cout << "motorref: " << test_h_p << ", " << test_h_s << std::endl;

        /* LOGGING */
        logss.str(std::string());
        logss << std::setprecision(8) << myVehSim.VehLatitude() << " " << myVehSim.VehLongitude() << ", ";
        logss << myVehSim.VehAtt().GetYaw() << ", ";
        logss << myVehSim.VehVel_world().transpose().format(CommaInitFmt) << "\n";
        logfile << logss.str();
        /***********/

        myVehSim.ExecuteStep(test_h_p, test_h_s);
        myVehSim.PublishSensors();

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    logfile.close();

    rclcpp::shutdown();
    return 0;
}

void ReadMappingParameters(const std::shared_ptr<rclcpp::SyncParametersClient> pc, ThrusterMappingParameters& tmp)
{
    tmp.d = pc->get_parameter("thruster_mapping.motors_distance", 0.0);
    tmp.lambda_pos = pc->get_parameter("thruster_mapping.lambda_pos", 0.0);
    tmp.lambda_neg = pc->get_parameter("thruster_mapping.lambda_neg", 0.0);
    // tmp.cb = Eigen::Vector4d((pc->get_parameter("thruster_mapping.cb", std::vector<double>(4, 0.0))).data());
    tmp.cX = Eigen::Vector3d((pc->get_parameter("thruster_mapping.cX", std::vector<double>(3, 0.0))).data());
    tmp.cN = Eigen::Vector3d((pc->get_parameter("thruster_mapping.cN", std::vector<double>(3, 0.0))).data());
    tmp.b1_pos = pc->get_parameter("thruster_mapping.b1_pos", 0.0);
    tmp.b2_pos = pc->get_parameter("thruster_mapping.b2_pos", 0.0);
    tmp.b1_neg = pc->get_parameter("thruster_mapping.b1_neg", 0.0);
    tmp.b2_neg = pc->get_parameter("thruster_mapping.b2_neg", 0.0);
    tmp.Inertia.diagonal()
        = Eigen::Vector3d((pc->get_parameter("thruster_mapping.Inertia", std::vector<double>(3, 0.0))).data());

    // std::cout << "Parameters read!" << std::endl;
    // std::cout << "tmp.Inertia.diagonal()" << tmp.Inertia.diagonal() << std::endl;
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
