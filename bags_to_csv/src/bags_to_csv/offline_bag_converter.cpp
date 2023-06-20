#include <iomanip>

#include "ulisse_msgs/topicnames.hpp"
#include "bags_to_csv/offline_bag_converter.hpp"


OfflineBagConverter::OfflineBagConverter(const std::string& bagPath, const std::string& saveFolder)
    : Node("offline_bag2csv_node")
    , bagPath_(bagPath)
    , saveFolder_(saveFolder)
{

    RCLCPP_INFO(this->get_logger(), "Converting: %s", bagPath_.c_str());
    RCLCPP_INFO(this->get_logger(), "CSV path: %s", saveFolder_.c_str());

    OpenFiles();
    ConvertToCSV();

    RCLCPP_INFO(this->get_logger(), "Conversion Successful.");
    CloseFiles();

    rclcpp::shutdown();
}

OfflineBagConverter::~OfflineBagConverter(){

}

bool OfflineBagConverter::ConvertToCSV()
{
    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());

    //rosbag2_storage::StorageOptions storage_options{};  // ROS2 Galactic
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
            gpsFile_ << std::fixed << std::setprecision(10)
                     << bag_message->time_stamp  * 1e-9 << ", "
                     << gpsData_.time << ", " << gpsData_.latitude << ", " << gpsData_.longitude << ", " << gpsData_.altitude << ", "
                     << gpsData_.speed << ", " << gpsData_.track
                     << "\n";
        } else if (bag_message->topic_name == ulisse_msgs::topicnames::sensor_imu){
            rclcpp::Serialization<ulisse_msgs::msg::IMUData> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &imuData_);
            sensorReceived = true;
        } else if (bag_message->topic_name == ulisse_msgs::topicnames::sensor_compass) {
            rclcpp::Serialization<ulisse_msgs::msg::Compass> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &compassData_);
            sensorReceived = true;
        } else if (bag_message->topic_name == ulisse_msgs::topicnames::sensor_magnetometer) {
            rclcpp::Serialization<ulisse_msgs::msg::Magnetometer> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &magnetometerData_);
            sensorReceived = true;
        } else if (bag_message->topic_name == ulisse_msgs::topicnames::sensor_dvl) {
            rclcpp::Serialization<ulisse_msgs::msg::DVLData> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &dvlData_);
            sensorReceived = true;
        } else if (bag_message->topic_name == ulisse_msgs::topicnames::sensor_fog) {
            rclcpp::Serialization<ulisse_msgs::msg::FOGData> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &fogData_);
            sensorReceived = true;
        } else if (bag_message->topic_name == ulisse_msgs::topicnames::llc_thrusters) {
            rclcpp::Serialization<ulisse_msgs::msg::LLCThrusters> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &llcThrustersData_);
            motorsFile_ << std::fixed << std::setprecision(6)
                        << bag_message->time_stamp  * 1e-9 << ", "
                        << llcThrustersData_.stamp.sec + (llcThrustersData_.stamp.nanosec * 1e-9) << ", "
                        << llcThrustersData_.left.timestamp_485 << ", " << llcThrustersData_.left.motor_speed << ", "
                        << llcThrustersData_.right.timestamp_485 << ", " << llcThrustersData_.right.motor_speed
                        << "\n";
        } else if (bag_message->topic_name == ulisse_msgs::topicnames::nav_filter_data) {
            rclcpp::Serialization<ulisse_msgs::msg::NavFilterData> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &navFilterData_);
            navFilterFile_ << std::fixed << std::setprecision(6)
                           << bag_message->time_stamp  * 1e-9 << ", "
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
                        << bag_message->time_stamp  * 1e-9 << ", "
                        << referenceVel_.stamp.sec + (referenceVel_.stamp.nanosec * 1e-9) << ", "
                        << referenceVel_.desired_surge << ", " << referenceVel_.desired_yaw_rate
                        << "\n";
        } else if (bag_message->topic_name == ulisse_msgs::topicnames::llc_thrusters_reference_perc) {
            rclcpp::Serialization<ulisse_msgs::msg::ThrustersReference> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &thrustersReference_);
            thrustersFile_ << std::fixed << std::setprecision(6)
                           << bag_message->time_stamp  * 1e-9 << ", "
                           << thrustersReference_.stamp.sec + (thrustersReference_.stamp.nanosec * 1e-9) << ", "
                           << thrustersReference_.left_percentage << ", " << thrustersReference_.right_percentage
                           << "\n";
        } else if (bag_message->topic_name == ulisse_msgs::topicnames::simulated_system) {
            rclcpp::Serialization<ulisse_msgs::msg::SimulatedSystem> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &groundtruth_);
            groundtruthFile_ << std::fixed << std::setprecision(6)
                           << bag_message->time_stamp  * 1e-9 << ", "
                           << groundtruth_.stamp.sec + (groundtruth_.stamp.nanosec * 1e-9) << ", "
                           << groundtruth_.inertialframe_linear_position.latlong.latitude  << ", "
                           << groundtruth_.inertialframe_linear_position.latlong.longitude << ", "
                           << groundtruth_.inertialframe_linear_position.altitude          << ", "
                           << groundtruth_.bodyframe_angular_position.roll                 << ", "
                           << groundtruth_.bodyframe_angular_position.pitch                << ", "
                           << groundtruth_.bodyframe_angular_position.yaw                  << ", "
                           << groundtruth_.bodyframe_linear_velocity[0]                    << ", "
                           << groundtruth_.bodyframe_linear_velocity[1]                    << ", "
                           << groundtruth_.bodyframe_linear_velocity[2]                    << ", "
                           << groundtruth_.bodyframe_angular_velocity[0]                   << ", "
                           << groundtruth_.bodyframe_angular_velocity[1]                   << ", "
                           << groundtruth_.bodyframe_angular_velocity[2]                   << ", "
                           << groundtruth_.inertialframe_water_current[0]                  << ", "
                           << groundtruth_.inertialframe_water_current[1]                  << ", "
                           << groundtruth_.gyro_bias[0]                                    << ", "
                           << groundtruth_.gyro_bias[1]                                    << ", "
                           << groundtruth_.gyro_bias[2]                                    << ", "
                           << groundtruth_.n_p                                             << ", "
                           << groundtruth_.n_s
                           << "\n";
        } else if (bag_message->topic_name == ulisse_msgs::topicnames::pathfollowing) {
            rclcpp::Serialization<ulisse_msgs::msg::PathFollow> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, &pathFollow_);
            pathFollowingFile_ << std::fixed << std::setprecision(6)
                               << bag_message->time_stamp  * 1e-9 << ", "
                               << pathFollow_.stamp.sec + (pathFollow_.stamp.nanosec * 1e-9) << ", "
                               << pathFollow_.delta << ", " << pathFollow_.y << ", " << pathFollow_.y_real << ", "
                               << pathFollow_.y_int_dot << ", " << pathFollow_.y_int << ", " << pathFollow_.psi
                               << "\n";
        }
        // else if (bag_message->topic_name == ulisse_msgs::topicnames::llc_battery_left) {
        //    batteryFile_ << batteryLeft_.stamp.sec + (batteryLeft_.stamp.nanosec * 1e-9) << ", " << batteryLeft_.charge_percent  << "\n";
        //}

        if (sensorReceived) {
            sensorReceived = false;
            sensorsFile_ << std::fixed << std::setprecision(6)
                         << bag_message->time_stamp  * 1e-9 << ", "
                         << imuData_.stamp.sec + (imuData_.stamp.nanosec * 1e-9) << ", "
                         << imuData_.accelerometer[0] << ", " << imuData_.accelerometer[1] << ", " << imuData_.accelerometer[2] << ", "
                         << imuData_.gyro[0] << ", " << imuData_.gyro[1] << ", " << imuData_.gyro[2] << ", "
                         << compassData_.stamp.sec + (compassData_.stamp.nanosec * 1e-9) << ", "
                         << compassData_.orientation.roll << ", " << compassData_.orientation.pitch << ", " << compassData_.orientation.yaw << ", "
                         << magnetometerData_.stamp.sec + (magnetometerData_.stamp.nanosec * 1e-9) << ", "
                         << magnetometerData_.orthogonalstrength[0] << ", " << magnetometerData_.orthogonalstrength[1] << ", " << magnetometerData_.orthogonalstrength[2] << ", "
                         << dvlData_.bottom_velocity[0] << ", " << dvlData_.bottom_velocity[1] << ", " << dvlData_.bottom_velocity[2] << ", "
                         << dvlData_.water_tracking[0] << ", " << dvlData_.water_tracking[1] << ", " << dvlData_.water_tracking[2] << ", "
                         << fogData_.angular_velocity
                         << "\n";
        }
    }

    return true;
}

bool OfflineBagConverter::OpenFiles()
{
    gpsFile_          .open(std::string(saveFolder_ + "/gps.txt"));
    gpsFile_     << "ros_time, gps_time, lat, long, alt, speed, track\n";

    sensorsFile_      .open(std::string(saveFolder_ + "/sensors.txt"));
    sensorsFile_ << "ros_time, imu_time, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, "
                    "time_compass, compass_R, compass_P, compass_Y, time_magn, "
                    "magn_x, magn_y, magn_z, dvl_bvel_x, dvl_bvel_y, dvl_bvel_z, dvl_wvel_x, dvl_wvel_y, dvl_wvel_z, fog_w\n";

    motorsFile_       .open(std::string(saveFolder_ + "/motors.txt"));
    motorsFile_ << "ros_time, time, timestamp_485, rpm_L, timestamp_485, rpm_R\n";

    navFilterFile_    .open(std::string(saveFolder_ + "/nav_filter.txt"));
    navFilterFile_ << "ros_time, time, x, y, z, r, p, y, u, v, h, "
                   << "rRate, pRate, yRate, cx, cy, bx, by, bz\n";

    refVelFile_      .open(std::string(saveFolder_ + "/reference_vel.txt"));
    refVelFile_   << "ros_time, time, surge, yawrate\n";

    thrustersFile_   .open(std::string(saveFolder_ + "/thrusters.txt"));
    thrustersFile_ << "ros_time, time, time485_L, perc_L, time485_R, perc_R\n";

    groundtruthFile_  .open(std::string(saveFolder_ + "/groundtruth.txt"));
    groundtruthFile_ << "ros_time, time, lat, long, alt, b_roll, b_pitch, b_yaw,"
                        "b_lin_vel_x, b_lin_vel_y, b_lin_vel_z, b_ang_vel_x, b_ang_vel_y, b_ang_vel_z,"
                        "i_wat_vel_x, i_wat_vel_y, gyro_bias_x, gyro_bias_y, gyro_bias_z, n_p, n_s\n";

    pathFollowingFile_.open(std::string(saveFolder_ + "/pathFollowing.txt"));
    pathFollowingFile_ << "ros_time, time, delta, y, y_real, y_int_dot, y_int, psi\n";

    //vehicleStatusFile_.open(std::string(saveFolder_ + "/vehicle_status.txt"));

    return true;
}

void OfflineBagConverter::CloseFiles()
{
    gpsFile_          .close();
    /*imuFile_          .close();
    compassFile_      .close();
    magnetometerFile_ .close();*/
    sensorsFile_      .close();
    motorsFile_       .close();
    navFilterFile_    .close();
    refVelFile_       .close();
    thrustersFile_    .close();
    pathFollowingFile_.close();
    //vehicleStatusFile_.close();

}
