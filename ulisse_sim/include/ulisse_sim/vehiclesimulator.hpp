#ifndef VEHICLESIMULATOR_H
#define VEHICLESIMULATOR_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <random>

#include "surface_vehicle_model/surfacevehiclemodel.hpp"

#include "ulisse_msgs/msg/ambient_sensors.hpp"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/micro_loop_count.hpp"
#include "ulisse_msgs/msg/motor_reference.hpp"
#include "ulisse_msgs/msg/real_system.hpp"
#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_sim/futils.h"

#include "GeographicLib/Geodesic.hpp"
#include "eigen3/Eigen/Dense"
#include "rml/RML.h"

namespace ulisse {

struct SinusoidalWave {
    double A, C, f;

    void ConfigureFromFile(const libconfig::Setting& confObj) noexcept(false)
    {
        ctb::SetParam(confObj, A, "A");
        ctb::SetParam(confObj, C, "C");
        ctb::SetParam(confObj, f, "f");
    }
};

struct SensorsNoise {
    Eigen::Vector3d gps_stdd;
    Eigen::Vector3d compass_stdd;
    Eigen::Vector3d magnetometer_stdd;
    Eigen::Vector3d gyro_stdd;
    Eigen::Vector3d accelerometer_stdd;
    SinusoidalWave bx, by, bz;

    void ConfigureFromFile(const libconfig::Setting& confObj) noexcept(false)
    {
        ctb::SetParamVector(confObj, gps_stdd, "gps_stdd");
        ctb::SetParamVector(confObj, compass_stdd, "compass_stdd");
        ctb::SetParamVector(confObj, magnetometer_stdd, "magnetometer_stdd");
        ctb::SetParamVector(confObj, gyro_stdd, "gyro_stdd");
        ctb::SetParamVector(confObj, accelerometer_stdd, "accelerometer_stdd");

        const libconfig::Setting& gyro_bias = confObj["gyro_bias"];

        const libconfig::Setting& b_x = gyro_bias["bx"];
        bx.ConfigureFromFile(b_x);
        const libconfig::Setting& b_y = gyro_bias["by"];
        by.ConfigureFromFile(b_y);
        const libconfig::Setting& b_z = gyro_bias["bz"];
        bz.ConfigureFromFile(b_z);
    }
};

struct SimulatorConfiguration {
    SensorsNoise sensorsNoise;
    int rate;
    double modelErrorPercentage;
    UlisseModelParameters modelParams;
    Eigen::Vector2d inertialF_waterCurrent;
    SinusoidalWave wx, wy;

    void ConfigureFromFile(libconfig::Config& confObj) noexcept(false)
    {
        ctb::SetParam(confObj, rate, "rate");
        ctb::SetParam(confObj, modelErrorPercentage, "modelErrorPercentage");
        ctb::SetParamVector(confObj, inertialF_waterCurrent, "inertialF_waterCurrent");

        //ulisse param
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& ulisseModel = root["ulisseModel"];
        modelParams.ConfigureFormFile(ulisseModel);

        //Add model error
        // construct a trivial random generator engine from a time-based seed:
        auto seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        std::uniform_real_distribution<double> distribution(1.0 - modelErrorPercentage / 100, 1 + modelErrorPercentage / 100);

        modelParams.Inertia *= distribution(generator);
        modelParams.cN *= distribution(generator);
        modelParams.cX *= distribution(generator);
        modelParams.b1_pos *= distribution(generator);
        modelParams.b2_pos *= distribution(generator);
        modelParams.b1_neg *= distribution(generator);
        modelParams.b1_neg *= distribution(generator);

        const libconfig::Setting& sensorsnoise = root["sensorsNoise"];
        sensorsNoise.ConfigureFromFile(sensorsnoise);

        //load additive sinusoidal noise on wx wy to simulate wawes
        const libconfig::Setting& wavesSimulator = root["wavesSimulator"];
        const libconfig::Setting& omega_x = wavesSimulator["wx"];
        wx.ConfigureFromFile(omega_x);
        const libconfig::Setting& omega_y = wavesSimulator["wy"];
        wy.ConfigureFromFile(omega_y);
    }
};

class VehicleSimulator {

    rclcpp::Node::SharedPtr nh_;

    GeographicLib::Geodesic geod_;

    double Ts_, Ts_fixed_;
    std::chrono::system_clock::time_point t_start_, t_now_, t_last_;
    std::chrono::nanoseconds iter_elapsed_, total_elapsed_;

    rml::EulerRPY bodyF_orientation_, previous_bodyF_orientation_;
    Eigen::Vector6d bodyF_relativeVelocity_, worldF_relativeVelocity_, worldF_velocity_, worldF_waterVelocity_;
    Eigen::Vector6d bodyF_relativeAcceleration_, worldF_relativeAcceleration_, bodyF_relativeAcceleration_projected_, bodyF_wavesEffects_;

    double latitude_, longitude_, previousLatitude_, previousLongitude_, altitude_;

    Eigen::Matrix3d P_;
    Eigen::Matrix6d bodyF_projection_;

    Eigen::Vector3d bodyF_wFk_;

    double vehicleTrack_, vehicleSpeed_;

    uint32_t timestamp_count_; // [200Hz counter]
    uint32_t stepssincepps_count_;

    ulisse_msgs::msg::MicroLoopCount microLoopCountMsg_;
    ulisse_msgs::msg::GPSData gpsMsg_;
    ulisse_msgs::msg::Compass compassMsg_;
    ulisse_msgs::msg::IMUData imuMsg_;
    ulisse_msgs::msg::AmbientSensors ambsensMsg_;
    ulisse_msgs::msg::Magnetometer magnetometerMsg_;
    ulisse_msgs::msg::MotorReference appliedMotorRefMsg_;
    ulisse_msgs::msg::RealSystem groundTruthMsg_;

    rclcpp::Publisher<ulisse_msgs::msg::MicroLoopCount>::SharedPtr microLoopCountPub_;
    rclcpp::Publisher<ulisse_msgs::msg::GPSData>::SharedPtr gpsPub_;
    rclcpp::Publisher<ulisse_msgs::msg::Compass>::SharedPtr compassPub_;
    rclcpp::Publisher<ulisse_msgs::msg::IMUData>::SharedPtr imuPub_;
    rclcpp::Publisher<ulisse_msgs::msg::AmbientSensors>::SharedPtr ambsensPub_;
    rclcpp::Publisher<ulisse_msgs::msg::Magnetometer>::SharedPtr magnetometerPub_;
    rclcpp::Publisher<ulisse_msgs::msg::MotorReference>::SharedPtr appliedMotorRefPub_;
    rclcpp::Publisher<ulisse_msgs::msg::RealSystem>::SharedPtr groundTruthPub_;
    rclcpp::Subscription<ulisse_msgs::msg::ThrustersData>::SharedPtr thrustersSub_;

    int gpsPubCounter_, compassPubCounter_, imuPubCounter_, magnetometerPubCounter_, ambientPubCounter_;

    futils::Timer motorTimeout_;
    double hp_, hs_;

    bool realTime_;

    Eigen::RotationMatrix worldF_R_bodyF_;

    void SimulateActuation();

public:
    VehicleSimulator(const rclcpp::Node::SharedPtr& nh);

    void SetSampleTime(double ts);
    void ExecuteStep();
    void SimulateSensors();
    void PublishSensors();

    auto WorldF_Velocity() const -> const Eigen::Vector6d& { return worldF_velocity_; }
    auto Altitude() const -> const rml::EulerRPY& { return bodyF_orientation_; }
    auto Latitude() const -> double { return latitude_; }
    auto Longitude() const -> double { return longitude_; }

    std::shared_ptr<SimulatorConfiguration> config;
    SurfaceVehicleModel ulisseModel;

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
    double GetCurrentTimeStamp() const;

    void ThrusterDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg);
};
}

#endif // VEHICLESIMULATOR_H
