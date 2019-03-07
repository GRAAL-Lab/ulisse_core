#include <cmath>
#include <iomanip>

#include "GeographicLib/UTMUPS.hpp"
#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_sim/vehiclesimulator.hpp"

namespace ulisse {

/**
 * Helper clip function
 */
double clamp(double n, double lower, double upper)
{
    return std::max(lower, std::min(n, upper));
}

VehicleSimulator::VehicleSimulator(const rclcpp::Node::SharedPtr& nh)
    : nh_(nh)
    , geod_(GeographicLib::Geodesic::WGS84())
    , gpspubcounter_(0)
    , sensorpubcounter_(0)
    , realtime_(true)
{
    lat_now_ = 44.393; // Genova Harbour lat-long
    long_now_ =  8.945;

    lat_last_ = lat_now_;
    long_last_ = long_now_;

    t_start_ = t_last_ = t_now_ = std::chrono::system_clock::now();

    micro_loop_count_pub_ = nh_->create_publisher<ulisse_msgs::msg::MicroLoopCount>(ulisse_msgs::topicnames::micro_loop_count);
    gpsdata_pub_ = nh_->create_publisher<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data);
    compass_pub_ = nh_->create_publisher<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass);
    imudata_pub_ = nh_->create_publisher<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu);
    ambsens_pub_ = nh_->create_publisher<ulisse_msgs::msg::AmbientSensors>(ulisse_msgs::topicnames::sensor_ambient);
    magneto_pub_ = nh_->create_publisher<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer);
    applied_motorref_pub_ = nh_->create_publisher<ulisse_msgs::msg::MotorReference>(ulisse_msgs::topicnames::motor_applied_ref);
}

rml::EulerRPY VehicleSimulator::VehAtt() const
{
    return vehAtt_now_;
}

Eigen::Vector6d VehicleSimulator::VehVel_world() const
{
    return vehVel_world_;
}

double VehicleSimulator::VehLatitude() const
{
    return lat_now_;
}

double VehicleSimulator::VehLongitude() const
{
    return long_now_;
}

void VehicleSimulator::SetParameters(double Ts, const ThrusterMappingParameters& thmapparams)
{
    Ts_fixed_ = Ts;
    ulisseModel_.SetMappingParams(thmapparams);

    std::cout << "=====  THRUSTER MAPPING   =====" << std::endl;
    std::cout << thmapparams << std::endl;
    std::cout << "===============================" << std::endl;
}

void VehicleSimulator::SetRealtime(bool realtime)
{
    realtime_ = realtime;
}

void VehicleSimulator::ExecuteStep(double h_p, double h_s)
{
    clamp(h_p, -100.0, 100.0);
    clamp(h_s, -100.0, 100.0);

    if (realtime_) {
        t_now_ = std::chrono::system_clock::now();
    } else {
        t_now_ = t_now_ + std::chrono::milliseconds(static_cast<long>(Ts_ * 1000.0));
    }

    iter_elapsed_ = std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_ - t_last_);
    total_elapsed_ = std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_ - t_start_);

    if (realtime_) {
        Ts_ = iter_elapsed_.count() / 1E9;
    } else {
        Ts_ = Ts_fixed_;
    }

    SimulateActuation(h_p, h_s);
    SimulateSensors(h_p, h_s);

    t_last_ = t_now_;
    vehAtt_last_ = vehAtt_now_;
    lat_last_ = lat_now_;
    long_last_ = long_now_;
}

void VehicleSimulator::SimulateActuation(double h_p, double h_s)
{
    // Computing vehicle acceleration
    ulisseModel_.DirectDynamics(h_p, h_s, vehRelVel_body_, vehRelAcc_body_);

    // Integrating the acceleration to get the vehicle velocity
    vehRelVel_body_ = vehRelVel_body_ + vehRelAcc_body_ * Ts_;

    // Projecting the acceleration and velocity on the world frame
    vehRelAcc_world_ = vehAtt_now_.ToRotMatrix().GetCartesianRotationMatrix() * vehRelAcc_body_;
    vehRelVel_world_ = vehAtt_now_.ToRotMatrix().GetCartesianRotationMatrix() * vehRelVel_body_;

    // Get the vehicle absolute velocity by adding the water current velocity
    vehVel_world_ = vehRelVel_world_ + waterVel_world_;

    // Passing from angular vehicle acceleration to Euler rates
    Eigen::Matrix3d S;
    S << cos(vehAtt_now_.GetYaw()) * cos(vehAtt_now_.GetPitch()), -sin(vehAtt_now_.GetYaw()), 0,
        sin(vehAtt_now_.GetYaw()) * cos(vehAtt_now_.GetPitch()), cos(vehAtt_now_.GetYaw()), 0,
        -sin(vehAtt_now_.GetPitch()), 0, 1;
    rml::RegularizationData mySvd;
    Eigen::Vector3d rpyEulerRates = rml::RegularizedPseudoInverse(S, mySvd) * vehVel_world_.GetSecondVect3();

    // Integrating the linear velocity to get the new position
    vehTrack_ = std::atan2(vehVel_world_(1), vehVel_world_(0));
    //std::cout << "Track (not wrapped): " << vehTrack_ << std::endl;
    // Wrapping track around 2*PI
    vehTrack_ = std::fmod(vehTrack_ + 2 * M_PI, 2 * M_PI);

    vehSpeed_ = std::sqrt(vehVel_world_(0) * vehVel_world_(0) + vehVel_world_(1) * vehVel_world_(1));
    double distance_ = vehSpeed_ * Ts_;

    //std::cout << "Track (deg): " << vehTrack_ * 180.0 / M_PI << std::endl;
    //std::cout << "Yaw (deg): " << vehAtt_last_.GetYaw() * 180.0 / M_PI << std::endl;
    //std::cout << "Distance: " << distance_ << std::endl;

    geod_.Direct(lat_last_, long_last_, vehTrack_ * 180.0 / M_PI, distance_, lat_now_, long_now_);

    // Integrating the Euler rates to get the new Euler angles and wrapping around PI
    vehAtt_now_.SetRoll(std::fmod((vehAtt_last_.GetRoll() + rpyEulerRates(0) * Ts_) + 2 * M_PI, 2 * M_PI));
    vehAtt_now_.SetPitch(std::fmod((vehAtt_last_.GetPitch() + rpyEulerRates(1) * Ts_) + 2 * M_PI, 2 * M_PI));
    vehAtt_now_.SetYaw(std::fmod((vehAtt_last_.GetYaw() + rpyEulerRates(2) * Ts_) + 2 * M_PI, 2 * M_PI));

}

double VehicleSimulator::GetCurrentTimestamp() const
{
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
    return static_cast<double>(now_nanosecs / 1E9);
    ;
}

void VehicleSimulator::SimulateSensors(double h_p, double h_s)
{
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / (int)1E9);
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % (int)1E9);

    auto elapsed_secs = static_cast<double>(total_elapsed_.count()) / 1E9;
    timestamp_count_ = static_cast<uint32_t>(elapsed_secs * 200.0);
    stepssincepps_count_ = static_cast<uint32_t>(elapsed_secs * 200.0) % 200;

    micro_loop_count_msg_.timestamp = timestamp_count_;
    micro_loop_count_msg_.stepssincepps = stepssincepps_count_;

    gpsdata_msg_.time = static_cast<double>(now_nanosecs / 1E9);
    gpsdata_msg_.track = vehTrack_;
    gpsdata_msg_.speed = vehSpeed_;
    gpsdata_msg_.latitude = lat_now_;
    gpsdata_msg_.longitude = long_now_;
    gpsdata_msg_.gpsfixmode = 3u; //ulisse_msgs::msg::GPSData::MODE_3D;

    compassdata_msg_.stamp.sec = now_stamp_secs;
    compassdata_msg_.stamp.nanosec = now_stamp_nanosecs;
    compassdata_msg_.orientation.roll = 0.0;
    compassdata_msg_.orientation.pitch = 0.0;
    compassdata_msg_.orientation.yaw = vehAtt_now_.GetYaw(); // * M_PI / 180.0 ;

    imudata_msg_.stamp.sec = now_stamp_secs;
    imudata_msg_.stamp.nanosec = now_stamp_nanosecs;
    imudata_msg_.accelerometer[0] = vehRelAcc_body_(0);
    imudata_msg_.accelerometer[1] = vehRelAcc_body_(1);
    imudata_msg_.accelerometer[2] = 9.81;
    imudata_msg_.gyro[0] = vehRelVel_body_(3);
    imudata_msg_.gyro[1] = vehRelVel_body_(4);
    imudata_msg_.gyro[2] = vehRelVel_body_(5);

    ambsens_msg_.stamp.sec = now_stamp_secs;
    ambsens_msg_.stamp.nanosec = now_stamp_nanosecs;
    ambsens_msg_.temperaturectrlbox = 23.0 + (rand() / (double)RAND_MAX) * 2.0;
    ambsens_msg_.humidityctrlbox = 50.0 + (rand() / (double)RAND_MAX) * 2.0;

    magneto_msg_.stamp.sec = now_stamp_secs;
    magneto_msg_.stamp.nanosec = now_stamp_nanosecs;
    magneto_msg_.orthogonalstrength[0] = 23.464 / 1E-9; // Example of magnetic field at lat long: 44.4056° N, 8.9463° E
    magneto_msg_.orthogonalstrength[1] = -1.051 / 1E-9;
    magneto_msg_.orthogonalstrength[2] = 39.746 / 1E-9;

    applied_motorref_msg_.left = h_p;
    applied_motorref_msg_.right = h_s;
}

void VehicleSimulator::PublishSensors()
{
    micro_loop_count_pub_->publish(micro_loop_count_msg_);

    //std::cout << "timestamp_count_ % 200: " << timestamp_count_ % 200 << std::endl;
    if ((int)(timestamp_count_ / 20) > gpspubcounter_) {
        gpspubcounter_ = (int)(timestamp_count_ / 20);
        gpsdata_pub_->publish(gpsdata_msg_);
    }

    //std::cout << "(int)(timestamp_count_ % 20):" << (int)(timestamp_count_ % 20) << std::endl;
    //std::cout << "sensorpubcounter: " << sensorpubcounter_ << std::endl;
    if ((int)(timestamp_count_ / 20) > sensorpubcounter_) {

        sensorpubcounter_ = (int)(timestamp_count_ / 20);
        //std::cout << "Ma ci passiamo di qua?" << std::endl;
        compass_pub_->publish(compassdata_msg_);
        imudata_pub_->publish(imudata_msg_);
        ambsens_pub_->publish(ambsens_msg_);
        magneto_pub_->publish(magneto_msg_);
    }

    applied_motorref_pub_->publish(applied_motorref_msg_);
}
}
