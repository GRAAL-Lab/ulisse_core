#ifndef VEHICLESIMULATOR_H
#define VEHICLESIMULATOR_H

#include <chrono>

#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/motor_reference.hpp"
#include "ulisse_msgs/msg/gps.hpp"

#include "GeographicLib/Geodesic.hpp"
#include "eigen3/Eigen/Dense"
#include "rml/RML.h"
#include "surface_vehicle_model/surfacevehiclemodel.h"

class VehicleSimulator {

    std::time_t now_time_t_;
    std::chrono::system_clock::time_point start_, now_, last_;
    std::chrono::microseconds elapsed_, total_;
    SurfaceVehicleModel ulisseModel_;
    GeographicLib::Geodesic geod_;
    Eigen::Vector3d vehPos_now_, vehPos_last_, direction_;
    rml::EulerRPY vehAtt_now_, vehAtt_last_;
    Eigen::Vector6d vehRelVel_body_, vehRelVel_world_, vehVel_world, waterVel_world_;
    Eigen::Vector6d vehRelAcc_body_, vehRelAcc_world_;
    double Ts_;

    double lat_, long_;


    ulisse_msgs::msg::GPS gpsdata_msg_;

    void SimulateSensors();
    void SimulateActuation(double h_s_, double h_p_);

public:
    VehicleSimulator();

    void SetParameters(double dt, const ThrusterMappingParameters &thmapparams);
    void ExecuteStep(double h_s_, double h_p_);


    Eigen::Vector3d VehPos() const;
    Eigen::Vector6d VehVel_world() const;
    rml::EulerRPY VehAtt() const;
};

#endif // VEHICLESIMULATOR_H
