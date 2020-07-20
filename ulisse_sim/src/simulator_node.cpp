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
static int rate;

bool ReadMappingParameters(UlisseModelParameters& modelParams, std::string file_name);

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

    UlisseModelParameters modelParams;
    std::string filename = "simparams.conf";
    ReadMappingParameters(modelParams, filename);

    rclcpp::WallRate loop_rate(rate);

    ulisse::VehicleSimulator myVehSim(node);
    myVehSim.SetParameters(1.0 / rate, modelParams);

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

bool ReadMappingParameters(UlisseModelParameters& modelParams, std::string file_name)
{
    //read conf file
    libconfig::Config confObj;

    //Inizialization
    std::string confPath = ament_index_cpp::get_package_share_directory("ulisse_sim").append("/conf/").append(file_name);

    std::cout << "PATH TO CONF FILE : " << confPath << std::endl;

    try {
        confObj.readFile(confPath.c_str());
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        return -1;
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return -1;
    }

    //rate
    ctb::SetParam(confObj, rate, "rate");

    //ulisse param
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& ulisseModel = root["ulisseModel"];
    modelParams.ConfigureFormFile(ulisseModel);

    //add noise on params
    double modelErrorFactor;
    ctb::SetParam(confObj, modelErrorFactor, "modelErrorFactor");

    modelParams.Inertia *= modelErrorFactor;
    modelParams.cN *= modelErrorFactor;
    modelParams.cX *= modelErrorFactor;
    modelParams.b1_pos *= modelErrorFactor;
    modelParams.b2_pos *= modelErrorFactor;
    modelParams.b1_neg *= modelErrorFactor;
    modelParams.b1_neg *= modelErrorFactor;

    return 0;
}
