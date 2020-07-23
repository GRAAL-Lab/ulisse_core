#include <cmath>
#include <iomanip>

#include "GeographicLib/UTMUPS.hpp"
#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_sim/vehiclesimulator.hpp"


namespace ulisse {
using namespace std::chrono_literals;
using std::placeholders::_1;

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
    , gpsPubCounter_(0)
    , compassPubCounter_(0)
    , imuPubCounter_(0)
    , magnetometerPubCounter_(0)
    , ambientPubCounter_(0)
    , realTime_(true)
{

    latitude_ = 44.393; // Genova Harbour lat-long
    longitude_ = 8.945;

    previousLatitude_ = latitude_;
    previousLongitude_ = longitude_;

    t_start_ = t_last_ = t_now_ = std::chrono::system_clock::now();

    microLoopCountPub_ = nh_->create_publisher<ulisse_msgs::msg::MicroLoopCount>(ulisse_msgs::topicnames::micro_loop_count, 1);
    gpsPub_ = nh_->create_publisher<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data, 1);
    compassPub_ = nh_->create_publisher<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass, 1);
    imuPub_ = nh_->create_publisher<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu, 1);
    ambsensPub_ = nh_->create_publisher<ulisse_msgs::msg::AmbientSensors>(ulisse_msgs::topicnames::sensor_ambient, 1);
    magnetometerPub_ = nh_->create_publisher<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer, 1);
    appliedMotorRefPub_ = nh_->create_publisher<ulisse_msgs::msg::MotorReference>(ulisse_msgs::topicnames::motor_applied_ref, 1);
    groundTruthPub_ = nh_->create_publisher<ulisse_msgs::msg::RealSystem>(ulisse_msgs::topicnames::real_system, 1);

    thrustersSub_ = nh_->create_subscription<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data, 1, std::bind(&VehicleSimulator::ThrusterDataCB, this, _1));

    worldF_waterVelocity_(0) = 0.0;
    worldF_waterVelocity_(1) = 0.0;
}

void VehicleSimulator::SetRealtime(bool realtime)
{
    realTime_ = realtime;
}

void VehicleSimulator::SetSampleTime(double ts)
{
    Ts_fixed_ = ts;
}

void VehicleSimulator::ExecuteStep()
{
    // We reset the motor reference in case we don't receive any message for more than one second
    if (motorTimeout_.Elapsed() > 1.0) {
        hp_ = hs_ = 0.0;
    }

    clamp(hp_, -100.0, 100.0);
    clamp(hs_, -100.0, 100.0);

    if (realTime_) {
        t_now_ = std::chrono::system_clock::now();
    } else {
        t_now_ = t_now_ + std::chrono::milliseconds(static_cast<long>(Ts_ * 1000.0));
    }

    iter_elapsed_ = std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_ - t_last_);
    total_elapsed_ = std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_ - t_start_);

    if (realTime_) {
        Ts_ = iter_elapsed_.count() / 1E9;
    } else {
        Ts_ = Ts_fixed_;
    }

    SimulateActuation();

    t_last_ = t_now_;
    previous_bodyF_orientation_ = bodyF_orientation_;
    previousLatitude_ = latitude_;
    previousLongitude_ = longitude_;
}

void VehicleSimulator::SimulateActuation()
{
    // Computing vehicle acceleration
    ulisseModel.DirectDynamics(hp_, hs_, bodyF_relativeVelocity_, bodyF_relativeAcceleration_);

    worldF_waterVelocity_(0) = config->inertialF_waterCurrent.x();
    worldF_waterVelocity_(1) = config->inertialF_waterCurrent.y();

    // Integrating the acceleration to get the vehicle velocity
    bodyF_relativeVelocity_ = bodyF_relativeVelocity_ + bodyF_relativeAcceleration_ * Ts_;

    // Projecting the acceleration and velocity on the world frame
    worldF_relativeAcceleration_ = bodyF_orientation_.ToRotationMatrix().CartesianRotationMatrix() * bodyF_relativeAcceleration_;
    worldF_relativeVelocity_ = bodyF_orientation_.ToRotationMatrix().CartesianRotationMatrix() * bodyF_relativeVelocity_;

    // Get the vehicle absolute velocity by adding the water current velocity

    worldF_velocity_ = worldF_relativeVelocity_ + worldF_waterVelocity_;

    // Passing from angular vehicle acceleration to Euler rates
    Eigen::Matrix3d S;
    S << cos(bodyF_orientation_.Yaw()) * cos(bodyF_orientation_.Pitch()), -sin(bodyF_orientation_.Yaw()), 0,
        sin(bodyF_orientation_.Yaw()) * cos(bodyF_orientation_.Pitch()), cos(bodyF_orientation_.Yaw()), 0,
        -sin(bodyF_orientation_.Pitch()), 0, 1;
    rml::RegularizationData mySvd;
    Eigen::Vector3d rpyEulerRates = rml::RegularizedPseudoInverse(S, mySvd) * worldF_velocity_.AngularVector();

    // Integrating the linear velocity to get the new position
    vehicleTrack_ = std::atan2(worldF_velocity_(1), worldF_velocity_(0));

    vehicleTrack_ = std::fmod(vehicleTrack_ + 2 * M_PI, 2 * M_PI);

    vehicleSpeed_ = std::sqrt(worldF_velocity_(0) * worldF_velocity_(0) + worldF_velocity_(1) * worldF_velocity_(1));
    double distance_ = vehicleSpeed_ * Ts_;

    geod_.Direct(previousLatitude_, previousLongitude_, vehicleTrack_ * 180.0 / M_PI, distance_, latitude_, longitude_);

    // Integrating the Euler rates to get the new Euler angles and wrapping around PI
    bodyF_orientation_.Roll(std::fmod((previous_bodyF_orientation_.Roll() + rpyEulerRates(0) * Ts_) + 2 * M_PI, 2 * M_PI));
    bodyF_orientation_.Pitch(std::fmod((previous_bodyF_orientation_.Pitch() + rpyEulerRates(1) * Ts_) + 2 * M_PI, 2 * M_PI));
    bodyF_orientation_.Yaw(std::fmod((previous_bodyF_orientation_.Yaw() + rpyEulerRates(2) * Ts_) + 2 * M_PI, 2 * M_PI));
}

void VehicleSimulator::SimulateSensors()
{
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

    auto elapsed_secs = static_cast<double>(total_elapsed_.count()) / 1E9;
    timestamp_count_ = static_cast<uint32_t>(elapsed_secs * 200.0);
    stepssincepps_count_ = static_cast<uint32_t>(elapsed_secs * 200.0) % 200;

    microLoopCountMsg_.timestamp = timestamp_count_;
    microLoopCountMsg_.stepssincepps = stepssincepps_count_;

    // construct a trivial random generator engine from a time-based seed:
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    std::normal_distribution<double> gpsNoiseX(0.0, config->sensorsNoise.gps_stdd.x());
    std::normal_distribution<double> gpsNoiseY(0.0, config->sensorsNoise.gps_stdd.y());

    //add noise to gps. Transform to cartesian, add noise and come back to map coordinates
    static ctb::LatLong centroidLocation(44.414165, 8.942184);
    Eigen::Vector3d cartesian_p;
    ctb::Map2CartesianPoint(ctb::LatLong(latitude_, longitude_), centroidLocation, cartesian_p);

    cartesian_p.x() = cartesian_p.x() + gpsNoiseX(generator);
    cartesian_p.y() = cartesian_p.y() + gpsNoiseY(generator);

    ctb::LatLong map_p;
    ctb::Cartesian2MapPoint(cartesian_p, centroidLocation, map_p);

    gpsMsg_.time = static_cast<double>(now_nanosecs / 1E9);
    gpsMsg_.track = vehicleTrack_;
    gpsMsg_.speed = vehicleSpeed_;
    gpsMsg_.latitude = map_p.latitude;
    gpsMsg_.longitude = map_p.longitude;
    gpsMsg_.altitude = 0.0;
    gpsMsg_.gpsfixmode = 3u; //ulisse_msgs::msg::GPSData::MODE_3D;

    std::normal_distribution<double> compassNoiseR(0.0, config->sensorsNoise.compass_stdd.x());
    std::normal_distribution<double> compassNoiseP(0.0, config->sensorsNoise.compass_stdd.y());
    std::normal_distribution<double> compassNoiseY(0.0, config->sensorsNoise.compass_stdd.z());

    //Compass
    compassMsg_.stamp.sec = now_stamp_secs;
    compassMsg_.stamp.nanosec = now_stamp_nanosecs;
    compassMsg_.orientation.roll = bodyF_orientation_.Roll() + compassNoiseR(generator);
    compassMsg_.orientation.pitch = bodyF_orientation_.Pitch() + compassNoiseP(generator);
    compassMsg_.orientation.yaw = bodyF_orientation_.Yaw() + compassNoiseY(generator);

    std::normal_distribution<double> accelerometerNoiseX(0.0, config->sensorsNoise.accelerometer_stdd.x());
    std::normal_distribution<double> accelerometerNoiseY(0.0, config->sensorsNoise.accelerometer_stdd.y());
    std::normal_distribution<double> accelerometerNoiseZ(0.0, config->sensorsNoise.accelerometer_stdd.z());

    imuMsg_.stamp.sec = now_stamp_secs;
    imuMsg_.stamp.nanosec = now_stamp_nanosecs;
    imuMsg_.accelerometer[0] = bodyF_relativeAcceleration_(0) + accelerometerNoiseX(generator);
    imuMsg_.accelerometer[1] = bodyF_relativeAcceleration_(1) + accelerometerNoiseY(generator);
    imuMsg_.accelerometer[2] = 9.81 + accelerometerNoiseZ(generator);

    //gyro noise
    std::normal_distribution<double> gyroNoiseX(0.0, config->sensorsNoise.gyro_stdd.x());
    std::normal_distribution<double> gyroNoiseY(0.0, config->sensorsNoise.gyro_stdd.y());
    std::normal_distribution<double> gyroNoiseZ(0.0, config->sensorsNoise.gyro_stdd.z());

    //gyro bias model as a very low frequence sin + const
    double t = now_stamp_secs + (now_stamp_nanosecs * 1e-9);
    double bx = config->sensorsNoise.bx.C + config->sensorsNoise.bx.A * sin(2 * M_PI * config->sensorsNoise.bx.f * t);
    double by = config->sensorsNoise.by.C + config->sensorsNoise.by.A * sin(2 * M_PI * config->sensorsNoise.by.f * t);
    double bz = config->sensorsNoise.bz.C + config->sensorsNoise.bz.A * sin(2 * M_PI * config->sensorsNoise.bz.f * t);
    imuMsg_.gyro[0] = bodyF_relativeVelocity_(3) + gyroNoiseX(generator) + bx;
    imuMsg_.gyro[1] = bodyF_relativeVelocity_(4) + gyroNoiseY(generator) + by;
    imuMsg_.gyro[2] = bodyF_relativeVelocity_(5) + gyroNoiseZ(generator) + bz;

    //ambient sensor
    ambsensMsg_.stamp.sec = now_stamp_secs;
    ambsensMsg_.stamp.nanosec = now_stamp_nanosecs;
    ambsensMsg_.temperaturectrlbox = 23.0 + (rand() / static_cast<double>(RAND_MAX)) * 2.0;
    ambsensMsg_.humidityctrlbox = 50.0 + (rand() / static_cast<double>(RAND_MAX)) * 2.0;

    //magnetometer
    Eigen::Vector3d m = { 23186.6 * 1E-9, 0.0 * 1E-9, 41122.0 * 1E-9 }; // Example of magnetic field at lat long: 44.4056° N, 8.9463° E

    Eigen::RotationMatrix Rz, Ry, Rx;
    Rz << cos(bodyF_orientation_.Yaw()), -sin(bodyF_orientation_.Yaw()), 0,
        sin(bodyF_orientation_.Yaw()), cos(bodyF_orientation_.Yaw()), 0,
        0, 0, 1;

    Ry << cos(bodyF_orientation_.Pitch()), 0, sin(bodyF_orientation_.Pitch()),
        0, 1, 0,
        -sin(bodyF_orientation_.Pitch()), 0, cos(bodyF_orientation_.Pitch());

    Rx << 1, 0, 0,
        0, cos(bodyF_orientation_.Roll()), -sin(bodyF_orientation_.Roll()),
        0, sin(bodyF_orientation_.Roll()), cos(bodyF_orientation_.Roll());

    Eigen::Vector3d ned_m = (Rz * Ry * Rx).transpose() * m;

    std::normal_distribution<double> magnetometerNoiseX(0.0, config->sensorsNoise.magnetometer_stdd.x());
    std::normal_distribution<double> magnetometerNoiseY(0.0, config->sensorsNoise.magnetometer_stdd.y());
    std::normal_distribution<double> magnetometerNoiseZ(0.0, config->sensorsNoise.magnetometer_stdd.z());

    magnetometerMsg_.stamp.sec = now_stamp_secs;
    magnetometerMsg_.stamp.nanosec = now_stamp_nanosecs;
    magnetometerMsg_.orthogonalstrength[0] = ned_m.x() + magnetometerNoiseX(generator);
    magnetometerMsg_.orthogonalstrength[1] = ned_m.y() + magnetometerNoiseY(generator);
    magnetometerMsg_.orthogonalstrength[2] = ned_m.z() + magnetometerNoiseZ(generator);

    //Fill the ground truth msg
    groundTruthMsg_.stamp.sec = now_stamp_secs;
    groundTruthMsg_.stamp.nanosec = now_stamp_nanosecs;
    groundTruthMsg_.inertialframe_linear_position.latlong.latitude = latitude_;
    groundTruthMsg_.inertialframe_linear_position.latlong.longitude = longitude_;
    groundTruthMsg_.inertialframe_linear_position.altitude = 0.0;
    groundTruthMsg_.bodyframe_angular_position.roll = bodyF_orientation_.Roll();
    groundTruthMsg_.bodyframe_angular_position.pitch = bodyF_orientation_.Pitch();
    groundTruthMsg_.bodyframe_angular_position.yaw = bodyF_orientation_.Yaw();
    groundTruthMsg_.bodyframe_linear_velocity.surge = bodyF_relativeVelocity_(0);
    groundTruthMsg_.bodyframe_linear_velocity.sway = bodyF_relativeVelocity_(1);
    groundTruthMsg_.bodyframe_linear_velocity.heave = bodyF_relativeVelocity_(2);
    groundTruthMsg_.bodyframe_angular_velocity.roll_rate = bodyF_relativeVelocity_(3);
    groundTruthMsg_.bodyframe_angular_velocity.pitch_rate = bodyF_relativeVelocity_(4);
    groundTruthMsg_.bodyframe_angular_velocity.yaw_rate = bodyF_relativeVelocity_(5);
    groundTruthMsg_.inertialframe_water_current[0] = worldF_waterVelocity_[0];
    groundTruthMsg_.inertialframe_water_current[1] = worldF_waterVelocity_[1];
    groundTruthMsg_.gyro_bias[0] = bx;
    groundTruthMsg_.gyro_bias[1] = by;
    groundTruthMsg_.gyro_bias[2] = bz;

    //motor ref
    appliedMotorRefMsg_.left = hp_;
    appliedMotorRefMsg_.right = hs_;
}

void VehicleSimulator::PublishSensors()
{
    microLoopCountPub_->publish(microLoopCountMsg_);
    groundTruthPub_->publish(groundTruthMsg_);
    appliedMotorRefPub_->publish(appliedMotorRefMsg_);

    if (static_cast<int>(timestamp_count_ / 20) > gpsPubCounter_) {
        gpsPubCounter_ = static_cast<int>(timestamp_count_ / 20);
        gpsPub_->publish(gpsMsg_);
    }

    if (static_cast<int>(timestamp_count_ / 20) > imuPubCounter_) {
        imuPubCounter_ = static_cast<int>(timestamp_count_ / 20);
        imuPub_->publish(imuMsg_);
    }

    if (static_cast<int>(timestamp_count_ / 20) > compassPubCounter_) {
        compassPubCounter_ = static_cast<int>(timestamp_count_ / 20);
        compassPub_->publish(compassMsg_);
    }

    if (static_cast<int>(timestamp_count_ / 20) > ambientPubCounter_) {
        ambientPubCounter_ = static_cast<int>(timestamp_count_ / 20);
        ambsensPub_->publish(ambsensMsg_);
    }

    if (static_cast<int>(timestamp_count_ / 20) > magnetometerPubCounter_) {
        magnetometerPubCounter_ = static_cast<int>(timestamp_count_ / 20);
        magnetometerPub_->publish(magnetometerMsg_);
    }
}

double VehicleSimulator::GetCurrentTimeStamp() const
{
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
    return static_cast<double>(now_nanosecs / 1E9);
}

void VehicleSimulator::ThrusterDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg)
{
    hp_ = msg->motor_percentage.left;
    hs_ = msg->motor_percentage.right;

    motorTimeout_.Start();
}
}
