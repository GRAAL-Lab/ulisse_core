#include "eigen3/Eigen/Dense"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_msgs/topicnames.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ctrl_toolbox/HelperFunctions.h>
#include <fstream>
#include <iomanip> //SETPRECISION
#include <math.h>
#include <rclcpp/rclcpp.hpp>

void ReadFile(std::string fileName, Eigen::MatrixXd& data, std::vector<long long>& ts);

bool LoadConfiguration(int& rate, std::string& gpsFileName, std::string& sensorFileName) noexcept(false);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("simulator_node");

    int rate = 0;
    std::string gpsFileName, sensorsFileName;

    if (!LoadConfiguration(rate, gpsFileName, sensorsFileName)) {
        std::cerr << "Errors in reading conf file" << std::endl;
        return -1;
    }

    rclcpp::WallRate loop_rate(rate);

    ulisse_msgs::msg::GPSData gpsMsg;
    ulisse_msgs::msg::IMUData imuMsg;
    ulisse_msgs::msg::Magnetometer magnetometerMsg;
    ulisse_msgs::msg::ThrustersData appliedMotorRefMsg;
    ulisse_msgs::msg::Compass compassMsg;

    ctb::LatLong centroid(44.393, 8.945); // Genova Harbour lat-long

    //Pubs of data
    auto gpsPub = node->create_publisher<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data, 1);
    auto imuPub = node->create_publisher<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu, 1);
    auto magnetometerPub = node->create_publisher<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer, 1);
    auto thrustersPub = node->create_publisher<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data, 1);
    auto compassPub = node->create_publisher<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass, 1);

    //open gps log file
    std::string line;
    std::string gpsConfPath = ament_index_cpp::get_package_share_directory("ulisse_sim_from_log").append("/logs/").append(gpsFileName);
    std::cout << gpsConfPath << std::endl;
    Eigen::MatrixXd gpsData;
    std::vector<long long int> gpsTs;
    ReadFile(gpsConfPath, gpsData, gpsTs);
    std::cout.setf(std::ios::fixed, std::ios::floatfield);

    Eigen::MatrixXd sensorsData;
    std::vector<long long int> sensorsTs;

    //open the sensors file
    std::string sensorsConfPath = ament_index_cpp::get_package_share_directory("ulisse_sim_from_log").append("/logs/").append(sensorsFileName);
    std::cout << sensorsFileName << std::endl;
    std::ifstream sensorsFile(sensorsConfPath);

    ReadFile(sensorsConfPath, sensorsData, sensorsTs);

    unsigned int i = 0, j = 0;
    int gpsCount = 20, sensorsCount = 20;
    ctb::LatLong NED_latlong(0.0, 0.0);

    while (rclcpp::ok()) {

        sensorsCount++;

        if (sensorsCount >= (sensorsTs[j + 1] - sensorsTs[j]) * 1E-9 * rate) {

            //fill sensors msgs
            long now_nanosecs_sensors = (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())).count();
            auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs_sensors / static_cast<int>(1E9));
            auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs_sensors % static_cast<int>(1E9));

            compassMsg.stamp.sec = now_stamp_secs;
            compassMsg.stamp.nanosec = now_stamp_nanosecs;
            compassMsg.orientation.roll = sensorsData.row(j + 1)(4);
            compassMsg.orientation.pitch = -sensorsData.row(j + 1)(3);
            compassMsg.orientation.yaw = sensorsData.row(j + 1)(2);

            magnetometerMsg.stamp.sec = now_stamp_secs;
            magnetometerMsg.stamp.nanosec = now_stamp_nanosecs;
            magnetometerMsg.orthogonalstrength[0] = sensorsData.row(j + 1)(5);
            magnetometerMsg.orthogonalstrength[1] = -sensorsData.row(j + 1)(6);
            magnetometerMsg.orthogonalstrength[2] = -sensorsData.row(j + 1)(7);

            imuMsg.stamp.sec = now_stamp_secs;
            imuMsg.stamp.nanosec = now_stamp_nanosecs;
            imuMsg.accelerometer[0] = sensorsData.row(j + 1)(8);
            imuMsg.accelerometer[1] = -sensorsData.row(j + 1)(9);
            imuMsg.accelerometer[2] = -sensorsData.row(j + 1)(10);
            imuMsg.gyro[0] = sensorsData.row(j + 1)(11);
            imuMsg.gyro[1] = -sensorsData.row(j + 1)(12);
            imuMsg.gyro[2] = -sensorsData.row(j + 1)(13);
            imuMsg.gyro4x[0] = sensorsData.row(j + 1)(14);
            imuMsg.gyro4x[1] = sensorsData.row(j + 1)(15);

            appliedMotorRefMsg.stamp.sec = now_stamp_secs;
            appliedMotorRefMsg.stamp.nanosec = now_stamp_nanosecs;
            appliedMotorRefMsg.motor_percentage.left = sensorsData.row(j + 1)(20) / 10;
            appliedMotorRefMsg.motor_percentage.right = sensorsData.row(j + 1)(21) / 10;

            std::cout << "Sto pubblicando " << std::endl;
            magnetometerPub->publish(magnetometerMsg);
            imuPub->publish(imuMsg);
            thrustersPub->publish(appliedMotorRefMsg);
            compassPub->publish(compassMsg);
            j++;
            sensorsCount = 0;
        }

        gpsCount++;

        if (gpsCount >= (gpsTs[i + 1] - gpsTs[i]) * 1E-9 * rate) {
            //fill the gps msg
            long now_nanosecs_gps = (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())).count();

            gpsMsg.time = static_cast<double>(now_nanosecs_gps / 1E9);
            gpsMsg.gpsfixmode = gpsData.row(i)(2);
            gpsMsg.flags = gpsData.row(i)(3);
            gpsMsg.latitude = gpsData.row(i)(4);
            gpsMsg.longitude = gpsData.row(i)(5);
            gpsMsg.altitude = gpsData.row(i)(6);
            gpsMsg.track = gpsData.row(i + 1)(7);
            gpsMsg.speed = gpsData.row(i + 1)(8);
            gpsMsg.climb = gpsData.row(i + 1)(9);
            gpsMsg.err = gpsData.row(i + 1)(10);
            gpsMsg.err_time = gpsData.row(i + 1)(11);
            gpsMsg.err_latitude = gpsData.row(i + 1)(12);
            gpsMsg.err_longitude = gpsData.row(i + 1)(13);
            gpsMsg.err_altitude = gpsData.row(i + 1)(14);
            gpsMsg.err_track = gpsData.row(i + 1)(15);
            gpsMsg.err_speed = gpsData.row(i + 1)(16);
            gpsMsg.err_climb = gpsData.row(i + 1)(17);
            gpsMsg.xdop = gpsData.row(i + 1)(18);
            gpsMsg.ydop = gpsData.row(i + 1)(19);
            gpsMsg.gdop = gpsData.row(i + 1)(20);
            gpsMsg.pdop = gpsData.row(i + 1)(21);
            gpsMsg.hdop = gpsData.row(i + 1)(22);
            gpsMsg.vdop = gpsData.row(i + 1)(23);
            gpsMsg.tdop = gpsData.row(i + 1)(24);

            std::cout << "Sto pubblicando " << std::endl;
            gpsPub->publish(gpsMsg);
            i++;
            gpsCount = 0;
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    node = nullptr;
    return 0;
}

void ReadFile(std::string fileName, Eigen::MatrixXd& data, std::vector<long long int>& ts)
{
    // File pointer
    std::fstream fin;

    // Open an existing file
    fin.open(fileName, std::ios::in);
    fin.ignore(500, '\n'); //skip the first line
    fin.setf(std::ios::fixed, std::ios::floatfield);

    // Read the Data from the file as String Vector
    std::vector<std::string> row;
    Eigen::VectorXd rowDouble;
    unsigned int rowNum = 0;

    std::string line, word, temp;

    while (getline(fin, line))
        ++rowNum;

    fin.clear();
    fin.seekg(0);
    fin.ignore(500, '\n'); //skip the first line

    while (!fin.eof()) {

        getline(fin, line);

        // used for breaking words

        std::stringstream s(line);
        // read every column data of a row and store it in a string variable, 'word'
        while (getline(s, word, '|')) {

            // add all the column data of a row to a vector
            row.push_back(word);
        }

        break;
    }

    fin.clear();
    fin.seekg(0);
    fin.ignore(500, '\n'); //skip the first line

    data = Eigen::MatrixXd::Zero(rowNum, row.size());

    fin.setf(std::ios_base::fixed);
    for (unsigned int j = 0; j < rowNum; j++) {

        row.clear();

        // read an entire row and store it in a string variable 'line'
        getline(fin, line);

        // used for breaking words
        std::stringstream s(line);
        // read every column data of a row and store it in a string variable, 'word'
        while (getline(s, word, '|')) {

            // add all the column data of a row to a vector
            row.push_back(word);
        }

        rowDouble = Eigen::VectorXd::Zero(row.size());

        ts.push_back(stoll(row[0]));

        // convert string to integer for comparision
        for (unsigned int i = 1; i < row.size(); i++) {

            rowDouble[i] = stod(row[i]);
        }

        data.row(j) = rowDouble.transpose();
    }

    fin.close();
}

bool LoadConfiguration(int& rate, std::string& gpsFileName, std::string& sensorFileName) noexcept(false)
{
    //read conf file
    libconfig::Config confObj;

    //Inizialization
    std::string confPath = ament_index_cpp::get_package_share_directory("ulisse_sim_from_log").append("/conf/simulator_node.conf");

    std::cout << "PATH TO CONF FILE : " << confPath << std::endl;

    try {
        confObj.readFile(confPath.c_str());
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        return false;
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return false;
    }

    ctb::SetParam(confObj, rate, "rate");

    const libconfig::Setting& root = confObj.getRoot();

    const libconfig::Setting& filesName = root["files_name"];

    ctb::SetParam(filesName, gpsFileName, "gps");

    ctb::SetParam(filesName, sensorFileName, "sensors");

    return true;
}
