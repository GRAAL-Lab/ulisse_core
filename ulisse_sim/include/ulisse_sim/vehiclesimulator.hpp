#ifndef VEHICLESIMULATOR_H
#define VEHICLESIMULATOR_H

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "surface_vehicle_model/surfacevehiclemodel.hpp"
#include "ulisse_msgs/msg/ambient_sensors.hpp"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/motor_reference.hpp"
#include "ulisse_msgs/msg/micro_loop_count.hpp"

#include "GeographicLib/Geodesic.hpp"
#include "eigen3/Eigen/Dense"
#include "rml/RML.h"

namespace ulisse {

class VehicleSimulator {

    rclcpp::Node::SharedPtr nh_;
    SurfaceVehicleModel ulisseModel_;

    GeographicLib::Geodesic geod_;

    double Ts_, Ts_fixed_;
    std::chrono::system_clock::time_point t_start_, t_now_, t_last_;
    std::chrono::nanoseconds iter_elapsed_, total_elapsed_;

    rml::EulerRPY vehAtt_now_, vehAtt_last_;
    Eigen::Vector6d vehRelVel_body_, vehRelVel_world_, vehVel_world_, waterVel_world_;
    Eigen::Vector6d vehRelAcc_body_, vehRelAcc_world_;

    double lat_now_, long_now_, lat_last_, long_last_;
    double vehTrack_, vehSpeed_;

    uint32_t timestamp_count_; // [200Hz counter]
    uint32_t stepssincepps_count_;

    ulisse_msgs::msg::MicroLoopCount micro_loop_count_msg_;
    ulisse_msgs::msg::GPSData gpsdata_msg_;
    ulisse_msgs::msg::Compass compassdata_msg_;
    ulisse_msgs::msg::IMUData imudata_msg_;
    ulisse_msgs::msg::AmbientSensors ambsens_msg_;
    ulisse_msgs::msg::Magnetometer magneto_msg_;
    ulisse_msgs::msg::MotorReference applied_motorref_msg_;

    rclcpp::Publisher<ulisse_msgs::msg::MicroLoopCount>::SharedPtr micro_loop_count_pub_;
    rclcpp::Publisher<ulisse_msgs::msg::GPSData>::SharedPtr gpsdata_pub_;
    rclcpp::Publisher<ulisse_msgs::msg::Compass>::SharedPtr compass_pub_;
    rclcpp::Publisher<ulisse_msgs::msg::IMUData>::SharedPtr imudata_pub_;
    rclcpp::Publisher<ulisse_msgs::msg::AmbientSensors>::SharedPtr ambsens_pub_;
    rclcpp::Publisher<ulisse_msgs::msg::Magnetometer>::SharedPtr magneto_pub_;
    rclcpp::Publisher<ulisse_msgs::msg::MotorReference>::SharedPtr applied_motorref_pub_;

    int gpspubcounter_, sensorpubcounter_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool realtime_;

    void SimulateSensors(double h_p, double h_s);
    void SimulateActuation(double h_p, double h_s);

public:
    VehicleSimulator(const rclcpp::Node::SharedPtr& nh);

    void SetParameters(double dt, const ThrusterMappingParameters& thmapparams);
    void ExecuteStep(double h_p, double h_s);
    void PublishSensors();

    Eigen::Vector6d VehVel_world() const;
    rml::EulerRPY VehAtt() const;
    double VehLatitude() const;
    double VehLongitude() const;

    /**
     * @brief Set if simulation should run in Realtime or not
     *
     * By default `realtime` is set to true, so the simulator runs in real-time. This means that,
     * even if a Ts (sample time) has been set, the simulator will use actual time differences
     * calculated with std::chrono to simulate time. If instead we set `realtime` to false, we can
     * run the simulator at any frequency, and time will proceed by steps of Ts.
     *
     * @param[in] realtime
     */
    void SetRealtime(bool realtime);
    double GetCurrentTimestamp() const;
};

}

#endif // VEHICLESIMULATOR_H
