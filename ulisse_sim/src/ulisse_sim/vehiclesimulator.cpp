#include <cmath>
#include <iomanip>

#include "ulisse_sim/vehiclesimulator.h"

VehicleSimulator::VehicleSimulator()
    : geod_(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f())
    , realtime_(true)
{
    vehAtt_now_.SetRPY(0.0, 0.0, 0.0);

    vehAtt_last_ = vehAtt_now_;

    lat_now_ = 44.4056; // Genova lat-long
    long_now_ = 8.9463;

    lat_last_ = lat_now_;
    long_last_ = long_now_;

    t_start_ = t_last_ = t_now_ = std::chrono::system_clock::now();
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
}

void VehicleSimulator::SetRealtime(bool realtime)
{
    realtime_ = realtime;
}

void VehicleSimulator::ExecuteStep(double h_p, double h_s)
{

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

    //std::cout << "Time since start (sec) = " << (total_elapsed_.count() / 1E9) << std::endl;

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
    vehSpeed_ = std::sqrt(vehVel_world_(0) * vehVel_world_(0) + vehVel_world_(1) * vehVel_world_(1));
    double distance_ = vehSpeed_ * Ts_;
    geod_.Direct(lat_last_, long_last_, vehTrack_, distance_, lat_now_, long_now_);

    // Integrating the Euler rates to get the new Euler angles and wrapping around PI
    vehAtt_now_.SetRoll(std::remainder(vehAtt_last_.GetRoll() + rpyEulerRates(0) * Ts_, M_PI));
    vehAtt_now_.SetPitch(std::remainder(vehAtt_last_.GetPitch() + rpyEulerRates(1) * Ts_, M_PI));
    vehAtt_now_.SetYaw(std::remainder(vehAtt_last_.GetYaw() + rpyEulerRates(2) * Ts_, M_PI));
}

void VehicleSimulator::SimulateSensors(double h_p, double h_s)
{
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
    //std::cout << std::setprecision(9) << "secs = " << now_nanosecs / 1E9 << std::endl;

    double elapsed_secs = static_cast<double>(total_elapsed_.count()) / 1E9;
    timeinfo_msg_.timestamp = static_cast<uint32_t>(elapsed_secs * 200.0);
    timeinfo_msg_.stepssincepps = static_cast<uint32_t>(elapsed_secs * 200.0) % 200;

    gpsdata_msg_.time = static_cast<double>(now_nanosecs / 1E9);
    gpsdata_msg_.track = vehTrack_;
    gpsdata_msg_.speed = vehSpeed_;
    gpsdata_msg_.latitude = lat_now_;
    gpsdata_msg_.longitude = long_now_;
    gpsdata_msg_.gpsfixmode = ulisse_msgs::msg::GPS::MODE_3D;

    compassdata_msg_.roll = 0.0;
    compassdata_msg_.pitch = 0.0;
    compassdata_msg_.yaw = vehAtt_now_.GetYaw(); // * M_PI / 180.0 ;

    imudata_msg_.accelerometer[0] = vehRelAcc_body_(0);
    imudata_msg_.accelerometer[1] = vehRelAcc_body_(1);
    imudata_msg_.accelerometer[2] = 9.81;

    imudata_msg_.gyro[0] = vehRelVel_body_(3);
    imudata_msg_.gyro[1] = vehRelVel_body_(4);
    imudata_msg_.gyro[2] = vehRelVel_body_(5);

    ambsens_msg_.temperaturectrlbox = 23.0 + (rand() / (double)RAND_MAX) * 2;
    ambsens_msg_.humidityctrlbox = 50.0 + (rand() / (double)RAND_MAX) * 2;

    motorref_msg_.left = h_p;
    motorref_msg_.right = h_s;

    magn_msg_.orthogonalstrength[0] = 23.464 / 1E-9; // Example of magnetic field at lat long: 44.4056° N, 8.9463° E
    magn_msg_.orthogonalstrength[1] = -1.051 / 1E-9;
    magn_msg_.orthogonalstrength[2] = 39.746 / 1E-9;

    std::cout << "timestamp:\t\t" << timeinfo_msg_.timestamp << std::endl;
    std::cout << "stepssincepps:\t" << timeinfo_msg_.stepssincepps << std::endl;
    std::cout << "total_elapsed_.count() / 1E9 = " << total_elapsed_.count() / 1E9 << std::endl;
    std::cout << "iter_elapsed_.count() / 1E9 = " << iter_elapsed_.count() / 1E9 << std::endl;
    std::cout << "Ts = " << Ts_ << std::endl;
    std::cout << "Ts_ - iter_elapsed = " << (Ts_ - iter_elapsed_.count() / 1E9) << std::endl;
}
