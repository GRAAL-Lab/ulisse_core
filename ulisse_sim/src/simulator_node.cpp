#include "rclcpp/rclcpp.hpp"

#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_sim/futils.h"
#include "ulisse_sim/vehiclesimulator.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>

#include <chrono>
#include <cmath>
#include <ctime> // localtime
#include <fstream>
#include <functional>
#include <iomanip> // put_time
#include <iostream>
#include <pwd.h>
#include <random>
#include <sstream> // stringstream

using namespace std::chrono_literals;

static double test_h_p(0.0), test_h_s(0.0);
static futils::Timer motor_timeout;

void ReadMappingParameters(UlisseModelParameters& tmp, std::string file_name);

void ThrusterDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg)
{
    test_h_p = msg->motor_percentage.left;
    test_h_s = msg->motor_percentage.right;
    motor_timeout.Start();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("simulator_node");
    auto thrusters_sub = node->create_subscription<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data, 10, ThrusterDataCB);

    UlisseModelParameters myTMP;
    std::string filename = "simparams.conf";
    ReadMappingParameters(myTMP, filename);

    int rate = 100;
    rclcpp::WallRate loop_rate(rate);
    double dt = 1.0 / rate;
    std::cout << "dt=" << dt << std::endl;

    ulisse::VehicleSimulator myVehSim(node);
    myVehSim.SetParameters(dt, myTMP);

    while (rclcpp::ok()) {

        // We reset the motor reference in case we don't receive any message for more than one second
        if (motor_timeout.Elapsed() > 1.0) {
            test_h_p = test_h_s = 0.0;
        }

        myVehSim.ExecuteStep(test_h_p, test_h_s);
        myVehSim.PublishSensors();

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

void ReadMappingParameters(UlisseModelParameters& tmp, std::string file_name)
{
    libconfig::Config confObj;

    //Inizialization
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("ulisse_sim");
    std::stringstream conf_path;
    conf_path << package_share_directory << "/conf/" << file_name;

    std::string confPath = conf_path.str().c_str();

    std::cout << "PATH TO CONF FILE : " << confPath << std::endl;

    //read conf file
    try {
        confObj.readFile(confPath.c_str());
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return;
    }

    ctb::SetParam(confObj, tmp.d, "sim.thruster_mapping.motors_distance");
    ctb::SetParam(confObj, tmp.lambda_pos, "sim.thruster_mapping.lambda_pos");
    ctb::SetParam(confObj, tmp.lambda_neg, "sim.thruster_mapping.lambda_neg");
    ctb::SetParam(confObj, tmp.b1_pos, "sim.thruster_mapping.b1_pos");
    ctb::SetParam(confObj, tmp.b2_pos, "sim.thruster_mapping.b2_pos");
    ctb::SetParam(confObj, tmp.b1_neg, "sim.thruster_mapping.b1_neg");
    ctb::SetParam(confObj, tmp.b2_neg, "sim.thruster_mapping.b2_neg");
    ctb::SetParamVector(confObj, tmp.cN, "sim.thruster_mapping.cN");
    ctb::SetParamVector(confObj, tmp.cX, "sim.thruster_mapping.cX");

    Eigen::Vector3d tmp_Inerzia;
    tmp_Inerzia.setZero();
    ctb::SetParamVector(confObj, tmp_Inerzia, "sim.thruster_mapping.Inertia");
    tmp.Inertia.diagonal() = Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_Inerzia.data());

    std::cout << "Parameters read!" << std::endl;
}
