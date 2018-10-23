#ifndef VEHICLESIMULATOR_H
#define VEHICLESIMULATOR_H

#include <chrono>

#include "ulisse_msgs/msg/time_info.hpp"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/motor_reference.hpp"
#include "ulisse_msgs/msg/ambient_sensors.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"

#include "GeographicLib/Geodesic.hpp"
#include "eigen3/Eigen/Dense"
#include "rml/RML.h"
#include "surface_vehicle_model/surfacevehiclemodel.h"

class VehicleSimulator {

    SurfaceVehicleModel ulisseModel_;

    std::chrono::system_clock::time_point t_start_, t_now_, t_last_;
    std::chrono::nanoseconds iter_elapsed_, total_elapsed_;

    GeographicLib::Geodesic geod_;

    rml::EulerRPY vehAtt_now_, vehAtt_last_;
    Eigen::Vector6d vehRelVel_body_, vehRelVel_world_, vehVel_world_, waterVel_world_;
    Eigen::Vector6d vehRelAcc_body_, vehRelAcc_world_;
    double Ts_, Ts_fixed_;

    double lat_now_, long_now_, lat_last_, long_last_;

    double vehTrack_, vehSpeed_;

    ulisse_msgs::msg::TimeInfo timeinfo_msg_;
    ulisse_msgs::msg::GPS gpsdata_msg_;
    ulisse_msgs::msg::Compass compassdata_msg_;
    ulisse_msgs::msg::IMUData imudata_msg_;
    ulisse_msgs::msg::MotorReference motorref_msg_;
    ulisse_msgs::msg::AmbientSensors ambsens_msg_;
    ulisse_msgs::msg::Magnetometer magn_msg_;

    void SimulateSensors(double h_p, double h_s);
    void SimulateActuation(double h_p, double h_s);

    bool realtime_;

public:
    VehicleSimulator();

    void SetParameters(double dt, const ThrusterMappingParameters& thmapparams);
    void ExecuteStep(double h_p, double h_s);

    //Eigen::Vector3d VehPos() const;
    Eigen::Vector6d VehVel_world() const;
    rml::EulerRPY VehAtt() const;
    double VehLatitude() const;
    double VehLongitude() const;
    void SetRealtime(bool realtime);
};

#endif // VEHICLESIMULATOR_H
