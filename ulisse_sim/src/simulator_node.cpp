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
    test_h_p = msg->motor_ctrlref.left;
    test_h_s = msg->motor_ctrlref.right;
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

    int rate = 10000;
    rclcpp::WallRate loop_rate(rate);
    double dt = 1.0 / rate;
    std::cout << "dt=" << dt << std::endl;

    ulisse::VehicleSimulator myVehSim(node);
    myVehSim.SetParameters(dt * 5.0, myTMP);
    //myVehSim.SetRealtime(false);

    std::cout.precision(3);
    std::cout << std::fixed;

    std::stringstream logss;
    std::ofstream logfile;

    std::stringstream datess;
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    datess << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H.%M.%S");

    // Action Manager initialization
    std::string homedir;
    homedir = getenv("HOME");

    std::string logdir = "/group_project/logs/simulator";
    std::stringstream logfilename_stream;
    logfilename_stream << homedir << logdir << "/sim_log_" << datess.str() << ".txt";

    std::string logfilename = logfilename_stream.str().c_str();
    std::cout << "* Saving log to: \"" << logfilename << "\" *" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    mode_t mode = 0777;
    struct stat st;
    for (std::string::iterator iter = logdir.begin(); iter != logdir.end();) {
        std::string::iterator newIter = std::find(iter, logdir.end(), '/');
        std::string newPath = homedir + "/" + std::string(logdir.begin(), newIter);

        if (stat(newPath.c_str(), &st) != 0) {
            if (mkdir(newPath.c_str(), mode) != 0 && errno != EEXIST) {
                std::cout << "cannot create folder [" << newPath << "] : " << strerror(errno) << std::endl;
                return -1;
            }
        } else if (!S_ISDIR(st.st_mode)) {
            errno = ENOTDIR;
            std::cout << "path [" << newPath << "] not a dir " << std::endl;
            return -1;
        } else
            std::cout << "path [" << newPath << "] already exists " << std::endl;

        iter = newIter;
        if (newIter != logdir.end())
            ++iter;
    }

    logfile.open(logfilename, std::ios_base::app);
    logss << "Lat Long, Yaw, Velocity (world)" << std::endl;
    logfile << logss.str();

    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", ", ", "", "", "", "");

    futils::Timer print_timeout;
    print_timeout.Start();

    while (rclcpp::ok()) {

        // We reset the motor reference in case we don't receive any message for more than one second
        if (motor_timeout.Elapsed() > 1.0) {
            test_h_p = test_h_s = 0.0;
        }

        /* LOGGING */
        logss.str(std::string());
        logss << std::setprecision(8) << myVehSim.VehLatitude() << " " << myVehSim.VehLongitude() << ", ";
        logss << myVehSim.VehAtt().GetYaw() << ", ";
        logss << myVehSim.VehVel_world().transpose().format(CommaInitFmt) << "\n";
        logfile << logss.str();
        /***********/

        myVehSim.ExecuteStep(test_h_p, test_h_s);
        myVehSim.PublishSensors();

        if (print_timeout.GetCurrentLapTime() > 0.1) {
            print_timeout.Lap();
            std::cout << "----------------------------------" << std::endl;
            std::cout << "time: " << std::setprecision(1) << myVehSim.GetCurrentTimestamp() << std::endl;
            std::cout << "lat, long: " << std::setprecision(8) << myVehSim.VehLatitude() << ", " << myVehSim.VehLongitude()
                      << std::endl;
            std::cout << "compass (deg): " << myVehSim.VehAtt().GetYaw() * 180.0 / M_PI << std::endl;
            std::cout << "velocity: " << myVehSim.VehVel_world().transpose() << std::endl;
            std::cout << "motorref: " << test_h_p << ", " << test_h_s << std::endl;
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    logfile.close();

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
