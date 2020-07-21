#include <cmath>
#include <iomanip>

#include "GeographicLib/UTMUPS.hpp"
#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_sim/vehiclesimulator.hpp"
#include <chrono>
#include <random>

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
    long_now_ = 8.945;

    lat_last_ = lat_now_;
    long_last_ = long_now_;

    t_start_ = t_last_ = t_now_ = std::chrono::system_clock::now();


    micro_loop_count_pub_ = nh_->create_publisher<ulisse_msgs::msg::MicroLoopCount>(ulisse_msgs::topicnames::micro_loop_count, 10);
    gpsdata_pub_ = nh_->create_publisher<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data, 10);
    compass_pub_ = nh_->create_publisher<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass, 10);
    imudata_pub_ = nh_->create_publisher<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu, 10);
    ambsens_pub_ = nh_->create_publisher<ulisse_msgs::msg::AmbientSensors>(ulisse_msgs::topicnames::sensor_ambient, 10);
    magneto_pub_ = nh_->create_publisher<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer, 10);
    applied_motorref_pub_ = nh_->create_publisher<ulisse_msgs::msg::MotorReference>(ulisse_msgs::topicnames::motor_applied_ref, 10);
    ground_truth_pub_ = nh_->create_publisher<ulisse_msgs::msg::RealSystem>(ulisse_msgs::topicnames::real_system, 10);

    waterVel_world_(0) = 0.0;
    waterVel_world_(1) = 0.0;
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

void VehicleSimulator::SetParameters(double Ts, const UlisseModelParameters& thmapparams)
{
    Ts_fixed_ = Ts;
    ulisseModel_.params = thmapparams;

    std::cout << thmapparams << std::endl;
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

    waterVel_world_(0) = 0.1;
    waterVel_world_(1) = 0.1;

    // Integrating the acceleration to get the vehicle velocity
    vehRelVel_body_ = vehRelVel_body_ + vehRelAcc_body_ * Ts_;

    // Projecting the acceleration and velocity on the world frame
    vehRelAcc_world_ = vehAtt_now_.ToRotationMatrix().CartesianRotationMatrix() * vehRelAcc_body_;
    vehRelVel_world_ = vehAtt_now_.ToRotationMatrix().CartesianRotationMatrix() * vehRelVel_body_;

    // Get the vehicle absolute velocity by adding the water current velocity

    vehVel_world_ = vehRelVel_world_ + waterVel_world_;

    // Passing from angular vehicle acceleration to Euler rates
    Eigen::Matrix3d S;
    S << cos(vehAtt_now_.Yaw()) * cos(vehAtt_now_.Pitch()), -sin(vehAtt_now_.Yaw()), 0,
        sin(vehAtt_now_.Yaw()) * cos(vehAtt_now_.Pitch()), cos(vehAtt_now_.Yaw()), 0,
        -sin(vehAtt_now_.Pitch()), 0, 1;
    rml::RegularizationData mySvd;
    Eigen::Vector3d rpyEulerRates = rml::RegularizedPseudoInverse(S, mySvd) * vehVel_world_.AngularVector();

    // Integrating the linear velocity to get the new position
    vehTrack_ = std::atan2(vehVel_world_(1), vehVel_world_(0));

    vehTrack_ = std::fmod(vehTrack_ + 2 * M_PI, 2 * M_PI);

    vehSpeed_ = std::sqrt(vehVel_world_(0) * vehVel_world_(0) + vehVel_world_(1) * vehVel_world_(1));
    double distance_ = vehSpeed_ * Ts_;

    geod_.Direct(lat_last_, long_last_, vehTrack_ * 180.0 / M_PI, distance_, lat_now_, long_now_);

    // Integrating the Euler rates to get the new Euler angles and wrapping around PI
    vehAtt_now_.Roll(std::fmod((vehAtt_last_.Roll() + rpyEulerRates(0) * Ts_) + 2 * M_PI, 2 * M_PI));
    vehAtt_now_.Pitch(std::fmod((vehAtt_last_.Pitch() + rpyEulerRates(1) * Ts_) + 2 * M_PI, 2 * M_PI));
    vehAtt_now_.Yaw(std::fmod((vehAtt_last_.Yaw() + rpyEulerRates(2) * Ts_) + 2 * M_PI, 2 * M_PI));
}

double VehicleSimulator::GetCurrentTimestamp() const
{
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
    return static_cast<double>(now_nanosecs / 1E9);
}

void VehicleSimulator::SimulateSensors(double h_p, double h_s)
{
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

    auto elapsed_secs = static_cast<double>(total_elapsed_.count()) / 1E9;
    timestamp_count_ = static_cast<uint32_t>(elapsed_secs * 200.0);
    stepssincepps_count_ = static_cast<uint32_t>(elapsed_secs * 200.0) % 200;

    micro_loop_count_msg_.timestamp = timestamp_count_;
    micro_loop_count_msg_.stepssincepps = stepssincepps_count_;

    // construct a trivial random generator engine from a time-based seed:
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    std::normal_distribution<double> gpsNoise(0.0, 0.7);

    //add noise to gps. Transform to cartesian, add noise and come back to map coordinates
    static ctb::LatLong centroidLocation(44.414165, 8.942184);
    Eigen::Vector3d cartesian_p;
    ctb::Map2CartesianPoint(ctb::LatLong(lat_now_, long_now_), centroidLocation, cartesian_p);

    cartesian_p.x() = cartesian_p.x() + gpsNoise(generator);
    cartesian_p.y() = cartesian_p.y() + gpsNoise(generator);

    ctb::LatLong map_p;
    ctb::Cartesian2MapPoint(cartesian_p, centroidLocation, map_p);

    gpsdata_msg_.time = static_cast<double>(now_nanosecs / 1E9);
    gpsdata_msg_.track = vehTrack_;
    gpsdata_msg_.speed = vehSpeed_;
    gpsdata_msg_.latitude = map_p.latitude;
    gpsdata_msg_.longitude = map_p.longitude;
    gpsdata_msg_.altitude = 0.0;
    gpsdata_msg_.gpsfixmode = 3u; //ulisse_msgs::msg::GPSData::MODE_3D;

    std::normal_distribution<double> compassNoiseRP(0.0, 0.017453501);
    std::normal_distribution<double> compassNoiseY(0.0, 0.011038433);

    compassdata_msg_.stamp.sec = now_stamp_secs;
    compassdata_msg_.stamp.nanosec = now_stamp_nanosecs;
    compassdata_msg_.orientation.roll = vehAtt_now_.Roll() + compassNoiseRP(generator);
    compassdata_msg_.orientation.pitch = vehAtt_now_.Pitch() + compassNoiseRP(generator);
    compassdata_msg_.orientation.yaw = vehAtt_now_.Yaw() + compassNoiseY(generator);

    std::normal_distribution<double> accelerometerNoise(0.0, 0.0980663043);
    imudata_msg_.stamp.sec = now_stamp_secs;
    imudata_msg_.stamp.nanosec = now_stamp_nanosecs;
    imudata_msg_.accelerometer[0] = vehRelAcc_body_(0) + accelerometerNoise(generator);
    imudata_msg_.accelerometer[1] = vehRelAcc_body_(1) + accelerometerNoise(generator);
    imudata_msg_.accelerometer[2] = 9.81 + accelerometerNoise(generator);

    //gyro noise
    std::normal_distribution<double> gyroNoise(0.0, 0.017448782);
    //gyro bias model as a very low frequence sin + const
    double C = 0.08, f = 0.001, A = 0.001;
    double t = now_stamp_secs + (now_stamp_nanosecs * 1e-9);
    double bx = C + A * sin(2 * M_PI * f * t);
    double by = C + A * sin(2 * M_PI * f * t);
    double bz = C + A * sin(2 * M_PI * f * t);
    imudata_msg_.gyro[0] = vehRelVel_body_(3) + gyroNoise(generator) + bx;
    imudata_msg_.gyro[1] = vehRelVel_body_(4) + gyroNoise(generator) + by;
    imudata_msg_.gyro[2] = vehRelVel_body_(5) + gyroNoise(generator) + bz;

    ambsens_msg_.stamp.sec = now_stamp_secs;
    ambsens_msg_.stamp.nanosec = now_stamp_nanosecs;
    ambsens_msg_.temperaturectrlbox = 23.0 + (rand() / static_cast<double>(RAND_MAX)) * 2.0;
    ambsens_msg_.humidityctrlbox = 50.0 + (rand() / static_cast<double>(RAND_MAX)) * 2.0;

    Eigen::Vector3d m = { 23186.6 * 1E-9, 0.0 * 1E-9, 41122.0 * 1E-9 }; // Example of magnetic field at lat long: 44.4056° N, 8.9463° E

    Eigen::RotationMatrix Rz, Ry, Rx;
    Rz << cos(vehAtt_now_.Yaw()), -sin(vehAtt_now_.Yaw()), 0,
        sin(vehAtt_now_.Yaw()), cos(vehAtt_now_.Yaw()), 0,
        0, 0, 1;

    Ry << cos(vehAtt_now_.Pitch()), 0, sin(vehAtt_now_.Pitch()),
        0, 1, 0,
        -sin(vehAtt_now_.Pitch()), 0, cos(vehAtt_now_.Pitch());

    Rx << 1, 0, 0,
        0, cos(vehAtt_now_.Roll()), -sin(vehAtt_now_.Roll()),
        0, sin(vehAtt_now_.Roll()), cos(vehAtt_now_.Roll());

    Eigen::Vector3d ned_m = (Rz * Ry * Rx).transpose() * m;

    std::normal_distribution<double> magnetometerNoise(0.0, 0.1 * 1E-6);

    magneto_msg_.stamp.sec = now_stamp_secs;
    magneto_msg_.stamp.nanosec = now_stamp_nanosecs;
    magneto_msg_.orthogonalstrength[0] = ned_m.x() + magnetometerNoise(generator);
    magneto_msg_.orthogonalstrength[1] = ned_m.y() + magnetometerNoise(generator);
    magneto_msg_.orthogonalstrength[2] = ned_m.z() + magnetometerNoise(generator);

    ground_truth_msg_.stamp.sec = now_stamp_secs;
    ground_truth_msg_.stamp.nanosec = now_stamp_nanosecs;
    ground_truth_msg_.inertialframe_linear_position.latlong.latitude = lat_now_;
    ground_truth_msg_.inertialframe_linear_position.latlong.longitude = long_now_;
    ground_truth_msg_.inertialframe_linear_position.altitude = 0.0;
    ground_truth_msg_.bodyframe_angular_position.roll = vehAtt_now_.Roll();
    ground_truth_msg_.bodyframe_angular_position.pitch = vehAtt_now_.Pitch();
    ground_truth_msg_.bodyframe_angular_position.yaw = vehAtt_now_.Yaw();
    ground_truth_msg_.bodyframe_linear_velocity.surge = vehRelVel_body_(0);
    ground_truth_msg_.bodyframe_linear_velocity.sway = vehRelVel_body_(1);
    ground_truth_msg_.bodyframe_linear_velocity.heave = vehRelVel_body_(2);
    ground_truth_msg_.bodyframe_angular_velocity.roll_rate = vehRelVel_body_(3);
    ground_truth_msg_.bodyframe_angular_velocity.pitch_rate = vehRelVel_body_(4);
    ground_truth_msg_.bodyframe_angular_velocity.yaw_rate = vehRelVel_body_(5);
    ground_truth_msg_.inertialframe_water_current[0] = waterVel_world_[0];
    ground_truth_msg_.inertialframe_water_current[1] = waterVel_world_[1];
    ground_truth_msg_.gyro_bias[0] = bx;
    ground_truth_msg_.gyro_bias[1] = by;
    ground_truth_msg_.gyro_bias[2] = bz;

    applied_motorref_msg_.left = h_p;
    applied_motorref_msg_.right = h_s;
}

void VehicleSimulator::PublishSensors()
{
    micro_loop_count_pub_->publish(micro_loop_count_msg_);

    if (static_cast<int>(timestamp_count_ / 20) > gpspubcounter_) {
        gpspubcounter_ = static_cast<int>(timestamp_count_ / 20);
        gpsdata_pub_->publish(gpsdata_msg_);
    }

    if (static_cast<int>(timestamp_count_ / 20) > sensorpubcounter_) {

        sensorpubcounter_ = static_cast<int>(timestamp_count_ / 20);
        compass_pub_->publish(compassdata_msg_);
        imudata_pub_->publish(imudata_msg_);
        ambsens_pub_->publish(ambsens_msg_);
        magneto_pub_->publish(magneto_msg_);
        ground_truth_pub_->publish(ground_truth_msg_);
    }

    applied_motorref_pub_->publish(applied_motorref_msg_);
}
}
