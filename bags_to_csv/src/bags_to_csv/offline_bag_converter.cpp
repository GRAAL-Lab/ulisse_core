#include <iomanip>

#include "ulisse_msgs/topicnames.hpp"
#include "bags_to_csv/offline_bag_converter.hpp"


OfflineBagConverter::OfflineBagConverter(const std::string& bagPath, const std::string& saveFolder)
    : Node("offline_bag2csv_node"), bagPath_(bagPath), saveFolder_(saveFolder)
{

    OpenFiles();
    if(ConvertToCSV()){
        std::cout << "Conversion Successful." << std::endl;
    } else {
        std::cout << "Conversion Failed." << std::endl;
    }
    CloseFiles();

    rclcpp::shutdown();
}

OfflineBagConverter::~OfflineBagConverter(){

}

bool OfflineBagConverter::ConvertToCSV()//, const std::string& csv_folder)
{


    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());

    rosbag2_cpp::StorageOptions storage_options{};
    storage_options.uri = bagPath_;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options{};
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    reader.open(storage_options, converter_options);

    auto topics = reader.get_all_topics_and_types();

    std::cout << "Topics: " << std::endl;
    for (auto&& t : topics){
        std::cout << t.name << std::endl;
    }

    bool sensorReceived = false;

    // Read and deserialize "serialized data"
    while (reader.has_next()) {
        auto bag_message = reader.read_next();

        if (bag_message->topic_name == ulisse_msgs::topicnames::sensor_gps_data) {

            rclcpp::Serialization<ulisse_msgs::msg::GPSData> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &gpsData_);
            gpsFile_ << std::fixed << std::setprecision(8)
                     << gpsData_.time << ", " << gpsData_.latitude << ", " << gpsData_.longitude << ", " << gpsData_.altitude << ", "
                     << gpsData_.speed << ", " << gpsData_.track
                     << "\n";
        } else if (bag_message->topic_name == ulisse_msgs::topicnames::sensor_imu){
            rclcpp::Serialization<ulisse_msgs::msg::IMUData> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &imuData_);
            sensorReceived = true;
            imuFile_ << std::fixed << std::setprecision(6)
                     << imuData_.stamp.sec + (imuData_.stamp.nanosec * 1e-9)<< ", "
                     << imuData_.accelerometer[0] << ", " << imuData_.accelerometer[1] << ", " << imuData_.accelerometer[2] << ", "
                     << imuData_.gyro[0] << ", " << imuData_.gyro[1] << ", " << imuData_.gyro[2]
                     << "\n";
        } else if (bag_message->topic_name == ulisse_msgs::topicnames::sensor_compass) {
            rclcpp::Serialization<ulisse_msgs::msg::Compass> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &compassData_);
            sensorReceived = true;
            compassFile_ << std::fixed << std::setprecision(6)
                         << compassData_.stamp.sec + (compassData_.stamp.nanosec * 1e-9) << ", "
                         << compassData_.orientation.roll << ", " << compassData_.orientation.pitch << ", " << compassData_.orientation.yaw
                         << "\n";
        } else if (bag_message->topic_name == ulisse_msgs::topicnames::sensor_magnetometer) {
            rclcpp::Serialization<ulisse_msgs::msg::Magnetometer> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &magnetometerData_);
            sensorReceived = true;
            magnetometerFile_ << std::fixed << std::setprecision(6)
                              << magnetometerData_.stamp.sec + (magnetometerData_.stamp.nanosec * 1e-9) << ", "
                              << magnetometerData_.orthogonalstrength[0] << ", " << magnetometerData_.orthogonalstrength[1] << ", " << magnetometerData_.orthogonalstrength[2]
                              << "\n";
        } else if (bag_message->topic_name == ulisse_msgs::topicnames::llc_motors) {
            rclcpp::Serialization<ulisse_msgs::msg::LLCMotors> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &llcMotorsData_);
            motorsFile_ << std::fixed << std::setprecision(6)
                        << llcMotorsData_.stamp.sec + (llcMotorsData_.stamp.nanosec * 1e-9) << ", "
                        << llcMotorsData_.left.motor_speed << ", " << llcMotorsData_.right.motor_speed
                        << "\n";
        } else if (bag_message->topic_name == ulisse_msgs::topicnames::nav_filter_data) {
            rclcpp::Serialization<ulisse_msgs::msg::NavFilterData> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &navFilterData_);
            navFilterFile_ << std::fixed << std::setprecision(6)
                        << navFilterData_.stamp.sec + (navFilterData_.stamp.nanosec * 1e-9) << ", "
                        << navFilterData_.inertialframe_linear_position.latlong.latitude << ", " << navFilterData_.inertialframe_linear_position.latlong.longitude << ", " << navFilterData_.inertialframe_linear_position.altitude << ", "
                        << navFilterData_.bodyframe_angular_position.roll << ", " << navFilterData_.bodyframe_angular_position.pitch << ", " << navFilterData_.bodyframe_angular_position.yaw << ", "
                        << navFilterData_.bodyframe_linear_velocity[0] << ", " << navFilterData_.bodyframe_linear_velocity[1] << ", " << navFilterData_.bodyframe_linear_velocity[2] << ", "
                        << navFilterData_.bodyframe_angular_velocity[0] << ", " << navFilterData_.bodyframe_angular_velocity[1] << ", " << navFilterData_.bodyframe_angular_velocity[2] << ", "
                        << navFilterData_.inertialframe_water_current[0] << ", " << navFilterData_.inertialframe_water_current[1] << ", "
                        << navFilterData_.gyro_bias[0] << ", " << navFilterData_.gyro_bias[1] << ", " << navFilterData_.gyro_bias[2]
                        << "\n";

        } else if (bag_message->topic_name == ulisse_msgs::topicnames::reference_velocities) {
            rclcpp::Serialization<ulisse_msgs::msg::ReferenceVelocities> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &referenceVel_);
            refVelFile_ << std::fixed << std::setprecision(6)
                        << referenceVel_.stamp.sec + (referenceVel_.stamp.nanosec * 1e-9) << ", "
                        << referenceVel_.desired_surge << ", " << referenceVel_.desired_yaw_rate
                        << "\n";
        } else if (bag_message->topic_name == ulisse_msgs::topicnames::thrusters_data) {
            rclcpp::Serialization<ulisse_msgs::msg::ThrustersData> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &thrustersData_);
            thrustersFile_ << std::fixed << std::setprecision(6)
                        << thrustersData_.stamp.sec + (thrustersData_.stamp.nanosec * 1e-9) << ", "
                        << thrustersData_.motor_percentage.left << ", " << thrustersData_.motor_percentage.right
                        << "\n";
        }
        // else if (bag_message->topic_name == ulisse_msgs::topicnames::llc_battery_left) {
        //    batteryFile_ << batteryLeft_.stamp.sec + (batteryLeft_.stamp.nanosec * 1e-9) << ", " << batteryLeft_.charge_percent  << "\n";
        //}

        if (sensorReceived) {
            sensorReceived = false;
            sensorsFile_ << std::fixed << std::setprecision(6)
                         << bag_message->time_stamp
                         << imuData_.stamp.sec + (imuData_.stamp.nanosec * 1e-9)<< ", "
                         << imuData_.accelerometer[0] << ", " << imuData_.accelerometer[1] << ", " << imuData_.accelerometer[2] << ", "
                         << imuData_.gyro[0] << ", " << imuData_.gyro[1] << ", " << imuData_.gyro[2] << ", "
                         << compassData_.stamp.sec + (compassData_.stamp.nanosec * 1e-9) << ", "
                         << compassData_.orientation.roll << ", " << compassData_.orientation.pitch << ", " << compassData_.orientation.yaw
                         << magnetometerData_.stamp.sec + (magnetometerData_.stamp.nanosec * 1e-9) << ", "
                         << magnetometerData_.orthogonalstrength[0] << ", " << magnetometerData_.orthogonalstrength[1] << ", " << magnetometerData_.orthogonalstrength[2]
                         << "\n";
        }
    }

    return true;
}

bool OfflineBagConverter::OpenFiles()
{
    gpsFile_          .open(std::string(saveFolder_ + "/gps.txt"));
    gpsFile_ << "time, lat, long, alt, speed, track\n";

    imuFile_          .open(std::string(saveFolder_ + "/imu.txt"));
    imuFile_ << "time, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z\n";

    compassFile_      .open(std::string(saveFolder_ + "/compass.txt"));
    compassFile_ << "time, roll, pitch, yaw\n";

    magnetometerFile_ .open(std::string(saveFolder_ + "/magnetometer.txt"));
    magnetometerFile_ << "time, orth_x, orth_y, orth_z\n";

    sensorsFile_      .open(std::string(saveFolder_ + "/sensors.txt"));
    sensorsFile_ << "time_imu, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, time_compass, compass_R, compass_P, compass_Y, time_magn, magn_x, magn_y, magn_z\n";

    motorsFile_       .open(std::string(saveFolder_ + "/motors.txt"));
    motorsFile_ << "time, rpm_L, rpm_R\n";

    navFilterFile_    .open(std::string(saveFolder_ + "/nav_filter.txt"));
    navFilterFile_ << "time, x, y, z, r, p, y, u, v, h, "
                   << "rRate, pRate, yRate, cx, cy, bx, by, bz\n";

    refVelFile_      .open(std::string(saveFolder_ + "/reference_vel.txt"));
    refVelFile_ << "time, surge, yawrate\n";

    thrustersFile_   .open(std::string(saveFolder_ + "/thrusters.txt"));
    thrustersFile_ << "time, perc_L, perc_R\n";
    //vehicleStatusFile_.open(std::string(saveFolder_ + "/vehicle_status.txt"));

    return true;
}

void OfflineBagConverter::CloseFiles()
{
    gpsFile_          .close();
    imuFile_          .close();
    compassFile_      .close();
    magnetometerFile_ .close();
    motorsFile_       .close();
    navFilterFile_    .close();
    refVelFile_       .close();
    thrustersFile_    .close();
    //vehicleStatusFile_.close();

}
