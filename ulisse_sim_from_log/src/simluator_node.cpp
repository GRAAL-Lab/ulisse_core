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

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("simluator_node");
    int rate = 100;

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
    std::string gpsConfPath = ament_index_cpp::get_package_share_directory("ulisse_sim_from_log").append("/logs/").append("20180709-093500-GpsData.log");

    Eigen::MatrixXd gpsData;
    std::vector<long long int> gpsTs;
    ReadFile(gpsConfPath, gpsData, gpsTs);
    std::cout.setf(std::ios::fixed, std::ios::floatfield);

    Eigen::MatrixXd sensorsData;
    std::vector<long long int> sensorsTs;

    //open the sensors file
    std::string sensorsConfPath = ament_index_cpp::get_package_share_directory("ulisse_sim_from_log").append("/logs/").append("20180709-093500-Sensors.log");
    std::ifstream sensorsFile(sensorsConfPath);

    ReadFile(sensorsConfPath, sensorsData, sensorsTs);

    unsigned int i = 0, j = 0;
    int gpsCount = 0, sensorsCount = 0;
    Eigen::Vector3d NED_cartesian_p = Eigen::Vector3d::Zero();
    ctb::LatLong NED_latlong(0.0, 0.0);
    double NED_altitude = 0.0;
    bool isFirst = true;

    while (rclcpp::ok()) {

        if (isFirst) {
            //Read the first timestamp to decide which sensor has to publish before
            if (sensorsTs[0] >= gpsTs[0]) {

                //fill sensors msgs
                long now_nanosecs_sensors = (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())).count();
                auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs_sensors / static_cast<int>(1E9));
                auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs_sensors % static_cast<int>(1E9));

                compassMsg.stamp.sec = now_stamp_secs;
                compassMsg.stamp.nanosec = now_stamp_nanosecs;
                compassMsg.orientation.roll = sensorsData.row(j)(4);
                compassMsg.orientation.pitch = -sensorsData.row(j)(3);
                compassMsg.orientation.yaw = sensorsData.row(j)(2);

                magnetometerMsg.stamp.sec = now_stamp_secs;
                magnetometerMsg.stamp.nanosec = now_stamp_nanosecs;
                magnetometerMsg.orthogonalstrength[0] = sensorsData.row(j)(6);
                magnetometerMsg.orthogonalstrength[1] = sensorsData.row(j)(5);
                magnetometerMsg.orthogonalstrength[2] = -sensorsData.row(j)(7);

                imuMsg.stamp.sec = now_stamp_secs;
                imuMsg.stamp.nanosec = now_stamp_nanosecs;
                imuMsg.accelerometer[0] = sensorsData.row(j)(9);
                imuMsg.accelerometer[1] = sensorsData.row(j)(8);
                imuMsg.accelerometer[2] = -sensorsData.row(j)(10);
                imuMsg.gyro[0] = sensorsData.row(j)(12);
                imuMsg.gyro[1] = sensorsData.row(j)(11);
                imuMsg.gyro[2] = -sensorsData.row(j)(13);
                imuMsg.gyro4x[0] = sensorsData.row(j)(14);
                imuMsg.gyro4x[1] = sensorsData.row(j)(15);

                appliedMotorRefMsg.stamp.sec = now_stamp_secs;
                appliedMotorRefMsg.stamp.nanosec = now_stamp_nanosecs;
                appliedMotorRefMsg.motor_percentage.left = sensorsData.row(j)(20) / 10;
                appliedMotorRefMsg.motor_percentage.right = sensorsData.row(j)(21) / 10;

                std::cout << "Sto pubblicando " << std::endl;
                magnetometerPub->publish(magnetometerMsg);
                imuPub->publish(imuMsg);
                thrustersPub->publish(appliedMotorRefMsg);
                compassPub->publish(compassMsg);

                std::cout << "(sensorsTs[0] - gpsTs[0]) * 1E-9 " << (sensorsTs[0] - gpsTs[0]) * 1E-9 << std::endl;

                //wait the righ amount of time form other sensors
                while (gpsCount < (sensorsTs[0] - gpsTs[0]) * 1E-9 * rate) {
                    gpsCount++;
                    std::cout << "gpsCount " << gpsCount << std::endl;
                }

                //fill the gps msg
                long now_nanosecs_gps = (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())).count();

                //This data are in ENU reference coordinate. Change to NED
                ctb::LatLong2LocalNED(ctb::LatLong(gpsData.row(i)(4), gpsData.row(i)(5)), gpsData.row(i)(6), centroid, NED_cartesian_p);
                ctb::LocalNED2LatLong(NED_cartesian_p, centroid, NED_latlong, NED_altitude);

                gpsMsg.time = static_cast<double>(now_nanosecs_gps / 1E9);
                gpsMsg.gpsfixmode = gpsData.row(i)(2);
                gpsMsg.flags = gpsData.row(i)(3);
                gpsMsg.latitude = NED_latlong.latitude;
                gpsMsg.longitude = NED_latlong.longitude;
                gpsMsg.altitude = NED_altitude;
                gpsMsg.track = gpsData.row(i)(7);
                gpsMsg.speed = gpsData.row(i)(8);
                gpsMsg.climb = gpsData.row(i)(9);
                gpsMsg.err = gpsData.row(i)(10);
                gpsMsg.err_time = gpsData.row(i)(11);
                gpsMsg.err_latitude = gpsData.row(i)(12);
                gpsMsg.err_longitude = gpsData.row(i)(13);
                gpsMsg.err_altitude = gpsData.row(i)(14);
                gpsMsg.err_track = gpsData.row(i)(15);
                gpsMsg.err_speed = gpsData.row(i)(16);
                gpsMsg.err_climb = gpsData.row(i)(17);
                gpsMsg.xdop = gpsData.row(i)(18);
                gpsMsg.ydop = gpsData.row(i)(19);
                gpsMsg.gdop = gpsData.row(i)(20);
                gpsMsg.pdop = gpsData.row(i)(21);
                gpsMsg.hdop = gpsData.row(i)(22);
                gpsMsg.vdop = gpsData.row(i)(23);
                gpsMsg.tdop = gpsData.row(i)(24);

                std::cout << "Sto pubblicando " << std::endl;
                gpsPub->publish(gpsMsg);
                gpsCount = 0;
                isFirst = false;

            } else {
                //fill the gps msg
                long now_nanosecs_gps = (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())).count();

                //This data are in ENU reference coordinate. Change to NED
                ctb::LatLong2LocalNED(ctb::LatLong(gpsData.row(i)(4), gpsData.row(i)(5)), gpsData.row(i)(6), centroid, NED_cartesian_p);
                ctb::LocalNED2LatLong(NED_cartesian_p, centroid, NED_latlong, NED_altitude);

                gpsMsg.time = static_cast<double>(now_nanosecs_gps / 1E9);
                gpsMsg.gpsfixmode = gpsData.row(i)(2);
                gpsMsg.flags = gpsData.row(i)(3);
                gpsMsg.latitude = NED_latlong.latitude;
                gpsMsg.longitude = NED_latlong.longitude;
                gpsMsg.altitude = NED_altitude;
                gpsMsg.track = gpsData.row(i)(7);
                gpsMsg.speed = gpsData.row(i)(8);
                gpsMsg.climb = gpsData.row(i)(9);
                gpsMsg.err = gpsData.row(i)(10);
                gpsMsg.err_time = gpsData.row(i)(11);
                gpsMsg.err_latitude = gpsData.row(i)(12);
                gpsMsg.err_longitude = gpsData.row(i)(13);
                gpsMsg.err_altitude = gpsData.row(i)(14);
                gpsMsg.err_track = gpsData.row(i)(15);
                gpsMsg.err_speed = gpsData.row(i)(16);
                gpsMsg.err_climb = gpsData.row(i)(17);
                gpsMsg.xdop = gpsData.row(i)(18);
                gpsMsg.ydop = gpsData.row(i)(19);
                gpsMsg.gdop = gpsData.row(i)(20);
                gpsMsg.pdop = gpsData.row(i)(21);
                gpsMsg.hdop = gpsData.row(i)(22);
                gpsMsg.vdop = gpsData.row(i)(23);
                gpsMsg.tdop = gpsData.row(i)(24);

                std::cout << "Sto pubblicando " << std::endl;
                gpsPub->publish(gpsMsg);

                std::cout << "(gpsTs[0] - sensorsTs[0]) * 1E-9 " << (gpsTs[0] - sensorsTs[0]) * 1E-9 << std::endl;

                //wait the righ amount of time form other sensors
                while (sensorsCount < (gpsTs[0] - sensorsTs[0]) * 1E-9 * rate) {
                    sensorsCount++;
                    std::cout << "sensorsCount " << sensorsCount << std::endl;
                }

                //fill sensors msgs
                long now_nanosecs_sensors = (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())).count();
                auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs_sensors / static_cast<int>(1E9));
                auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs_sensors % static_cast<int>(1E9));

                compassMsg.stamp.sec = now_stamp_secs;
                compassMsg.stamp.nanosec = now_stamp_nanosecs;
                compassMsg.orientation.roll = sensorsData.row(j)(4);
                compassMsg.orientation.pitch = -sensorsData.row(j)(3);
                compassMsg.orientation.yaw = sensorsData.row(j)(2);

                magnetometerMsg.stamp.sec = now_stamp_secs;
                magnetometerMsg.stamp.nanosec = now_stamp_nanosecs;
                magnetometerMsg.orthogonalstrength[0] = sensorsData.row(j)(6);
                magnetometerMsg.orthogonalstrength[1] = sensorsData.row(j)(5);
                magnetometerMsg.orthogonalstrength[2] = -sensorsData.row(j)(7);

                imuMsg.stamp.sec = now_stamp_secs;
                imuMsg.stamp.nanosec = now_stamp_nanosecs;
                imuMsg.accelerometer[0] = sensorsData.row(j)(9);
                imuMsg.accelerometer[1] = sensorsData.row(j)(8);
                imuMsg.accelerometer[2] = -sensorsData.row(j)(10);
                imuMsg.gyro[0] = sensorsData.row(j)(12);
                imuMsg.gyro[1] = sensorsData.row(j)(11);
                imuMsg.gyro[2] = -sensorsData.row(j)(13);
                imuMsg.gyro4x[0] = sensorsData.row(j)(14);
                imuMsg.gyro4x[1] = sensorsData.row(j)(15);

                appliedMotorRefMsg.stamp.sec = now_stamp_secs;
                appliedMotorRefMsg.stamp.nanosec = now_stamp_nanosecs;
                appliedMotorRefMsg.motor_percentage.left = sensorsData.row(j)(20) / 10;
                appliedMotorRefMsg.motor_percentage.right = sensorsData.row(j)(21) / 10;

                std::cout << "Sto pubblicando " << std::endl;
                magnetometerPub->publish(magnetometerMsg);
                imuPub->publish(imuMsg);
                thrustersPub->publish(appliedMotorRefMsg);
                compassPub->publish(compassMsg);

                sensorsCount = 0;
                isFirst = false;
            }
        } else {
            sensorsCount++;
            std::cout << "sensorsCount " << sensorsCount << std::endl;
            std::cout << "(sensorsTs[i + 1] - sensorsTs[i]) * 1E-9 " << (sensorsTs[j + 1] - sensorsTs[j]) * 1E-9 << std::endl;
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
                magnetometerMsg.orthogonalstrength[0] = sensorsData.row(j + 1)(6);
                magnetometerMsg.orthogonalstrength[1] = sensorsData.row(j + 1)(5);
                magnetometerMsg.orthogonalstrength[2] = -sensorsData.row(j + 1)(7);

                imuMsg.stamp.sec = now_stamp_secs;
                imuMsg.stamp.nanosec = now_stamp_nanosecs;
                imuMsg.accelerometer[0] = sensorsData.row(j + 1)(9);
                imuMsg.accelerometer[1] = sensorsData.row(j + 1)(8);
                imuMsg.accelerometer[2] = -sensorsData.row(j + 1)(10);
                imuMsg.gyro[0] = sensorsData.row(j + 1)(12);
                imuMsg.gyro[1] = sensorsData.row(j + 1)(11);
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
                std::cout << "j: " << j << std::endl;
                sensorsCount = 0;
            }

            gpsCount++;
            std::cout << "gpsCount " << gpsCount << std::endl;
            std::cout << "(gpsTs[i + 1] - gpsTs[i]) * 1E-9 " << (gpsTs[i + 1] - gpsTs[i]) * 1E-9 << std::endl;
            if (gpsCount >= (gpsTs[i + 1] - gpsTs[i]) * 1E-9 * rate) {
                //fill the gps msg
                long now_nanosecs_gps = (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())).count();

                //This data are in ENU reference coordinate. Change to NED
                ctb::LatLong2LocalNED(ctb::LatLong(gpsData.row(i + 1)(4), gpsData.row(i + 1)(5)), gpsData.row(i + 1)(6), centroid, NED_cartesian_p);
                ctb::LocalNED2LatLong(NED_cartesian_p, centroid, NED_latlong, NED_altitude);

                gpsMsg.time = static_cast<double>(now_nanosecs_gps / 1E9);
                gpsMsg.gpsfixmode = gpsData.row(i + 1)(2);
                gpsMsg.flags = gpsData.row(i + 1)(3);
                gpsMsg.latitude = NED_latlong.latitude;
                gpsMsg.longitude = NED_latlong.longitude;
                gpsMsg.altitude = NED_altitude;
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
                std::cout << "i: " << i << std::endl;
                gpsCount = 0;
            }
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
