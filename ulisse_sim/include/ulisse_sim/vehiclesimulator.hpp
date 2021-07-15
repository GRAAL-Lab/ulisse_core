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
#include "ulisse_msgs/msg/simulated_system.hpp"
#include "ulisse_msgs/msg/llc_thrusters.hpp"
#include "ulisse_msgs/msg/thrusters_reference.hpp"
#include "ulisse_sim/futils.h"

#include "GeographicLib/Geodesic.hpp"
#include "eigen3/Eigen/Dense"
#include "rml/RML.h"

namespace ulisse {

struct SinusoidalWave {
    double A, C, f;

    bool ConfigureFromFile(const libconfig::Setting& confObj) noexcept(false)
    {
        if (!ctb::GetParam(confObj, A, "A"))
            return false;
        if (!ctb::GetParam(confObj, C, "C"))
            return false;
        if (!ctb::GetParam(confObj, f, "f"))
            return false;

        return true;
    }
};

struct SensorsNoise {
    Eigen::Vector3d gps_stdd;
    Eigen::Vector3d compass_stdd;
    Eigen::Vector3d magnetometer_stdd;
    Eigen::Vector3d gyro_stdd;
    Eigen::Vector3d accelerometer_stdd;
    SinusoidalWave bx, by, bz;

    bool ConfigureFromFile(const libconfig::Setting& confObj) noexcept(false)
    {
        if (!ctb::GetParamVector(confObj, gps_stdd, "gps_stdd"))
            return false;
        if (!ctb::GetParamVector(confObj, compass_stdd, "compass_stdd"))
            return false;
        if (!ctb::GetParamVector(confObj, magnetometer_stdd, "magnetometer_stdd"))
            return false;
        if (!ctb::GetParamVector(confObj, gyro_stdd, "gyro_stdd"))
            return false;
        if (!ctb::GetParamVector(confObj, accelerometer_stdd, "accelerometer_stdd"))
            return false;

        const libconfig::Setting& gyro_bias = confObj["gyro_bias"];

        const libconfig::Setting& b_x = gyro_bias["bx"];
        if (!bx.ConfigureFromFile(b_x)) {
            std::cerr << "Failed to laod gyro bias on x" << std::endl;
            return false;
        }

        const libconfig::Setting& b_y = gyro_bias["by"];
        if (!by.ConfigureFromFile(b_y)) {
            std::cerr << "Failed to laod gyro bias on x" << std::endl;
            return false;
        }

        const libconfig::Setting& b_z = gyro_bias["bz"];
        if (!bz.ConfigureFromFile(b_z)) {
            std::cerr << "Failed to laod gyro bias on x" << std::endl;
            return false;
        }

        return true;
    }
};

struct SimulatorConfiguration {
    SensorsNoise sensorsNoise;
    int rate;
    double modelErrorPercentage;
    UlisseModelParameters modelParams;
    Eigen::Vector2d inertialF_waterCurrent;
    Eigen::Vector3d bodyF_gps_position;
    SinusoidalWave wx, wy;

    bool ConfigureFromFile(libconfig::Config& confObj) noexcept(false)
    {
        if (!ctb::GetParam(confObj, rate, "rate"))
            return false;

        if (!ctb::GetParam(confObj, rate, "rate"))
            return false;
        if (!ctb::GetParam(confObj, modelErrorPercentage, "modelErrorPercentage"))
            return false;
        if (!ctb::GetParamVector(confObj, inertialF_waterCurrent, "inertialF_waterCurrent"))
            return false;
        if (!ctb::GetParamVector(confObj, bodyF_gps_position, "bodyF_gps_position"))
            return false;

        // Ulisse model parameters
        if (!modelParams.LoadConfiguration(confObj)) {
            std::cerr << "Failed to load ulisse model params " << std::endl;
            return false;
        }

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

        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& sensorsnoise = root["sensorsNoise"];
        if (!sensorsNoise.ConfigureFromFile(sensorsnoise)) {
            std::cerr << "Failed to load ulisse sensors noise " << std::endl;
            return false;
        }

        //load additive sinusoidal noise on wx wy to simulate wawes
        const libconfig::Setting& wavesSimulator = root["wavesSimulator"];
        const libconfig::Setting& omega_x = wavesSimulator["wx"];

        if (!wx.ConfigureFromFile(omega_x)) {
            std::cerr << "Failed to load waves params on x " << std::endl;
            return false;
        }

        const libconfig::Setting& omega_y = wavesSimulator["wy"];

        if (!wy.ConfigureFromFile(omega_y)) {
            std::cerr << "Failed to load waves params on y " << std::endl;
            return false;
        }

        return true;
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
    ulisse_msgs::msg::ThrustersReference appliedMotorRefMsg_;
    ulisse_msgs::msg::SimulatedSystem groundTruthMsg_;
    ulisse_msgs::msg::LLCThrusters motorsDataMsg_;

    rclcpp::Publisher<ulisse_msgs::msg::MicroLoopCount>::SharedPtr microLoopCountPub_;
    rclcpp::Publisher<ulisse_msgs::msg::GPSData>::SharedPtr gpsPub_;
    rclcpp::Publisher<ulisse_msgs::msg::Compass>::SharedPtr compassPub_;
    rclcpp::Publisher<ulisse_msgs::msg::IMUData>::SharedPtr imuPub_;
    rclcpp::Publisher<ulisse_msgs::msg::AmbientSensors>::SharedPtr ambsensPub_;
    rclcpp::Publisher<ulisse_msgs::msg::Magnetometer>::SharedPtr magnetometerPub_;
    rclcpp::Publisher<ulisse_msgs::msg::ThrustersReference>::SharedPtr appliedMotorRefPub_;
    rclcpp::Publisher<ulisse_msgs::msg::SimulatedSystem>::SharedPtr simulatedSystemPub_;
    rclcpp::Publisher<ulisse_msgs::msg::LLCThrusters>::SharedPtr motorsDataPub_;
    
    rclcpp::Subscription<ulisse_msgs::msg::ThrustersReference>::SharedPtr thrustersSub_;

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

    void ThrustersReferenceCB(const ulisse_msgs::msg::ThrustersReference::SharedPtr msg);
};
}

#endif // VEHICLESIMULATOR_H
