#ifndef ULISSE_CTRL_DYNAMIC_VEHICLE_CONTROLLER_HPP
#define ULISSE_CTRL_DYNAMIC_VEHICLE_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/dynamic_pid_control.hpp"
#include "ulisse_msgs/msg/stsm_control.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/reference_velocities.hpp"
#include "ulisse_msgs/msg/simulated_velocity_sensor.hpp"
#include "ulisse_msgs/msg/thruster_mapping_control.hpp"
#include "ulisse_msgs/msg/thrusters_reference.hpp"
#include "ulisse_msgs/msg/vehicle_status.hpp"
#include "ulisse_msgs/srv/min_srv.hpp"
#include "ulisse_msgs/srv/reset_configuration.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ctrl_toolbox/HelperFunctions.h"

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"

#include "surface_vehicle_model/surfacevehiclemodel.hpp"

#include "ulisse_msgs/terminal_utils.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ulisse {

class DynamicVehicleController : public rclcpp::Node {

    double sampleTime_;
    std::string confFileName_;
    ulisse_msgs::msg::NavFilterData filterData;
    ulisse_msgs::msg::ReferenceVelocities referenceVelocities;
    ulisse_msgs::msg::VehicleStatus vehicleStatus;

    rclcpp::TimerBase::SharedPtr runTimer_;

    //config struct
    std::shared_ptr<DCLConfiguration> dcl_conf;// = std::make_shared<DCLConfiguration>();
    // ulisse model
    SurfaceVehicleModel ulisseModel;

    rclcpp::Service<ulisse_msgs::srv::ResetConfiguration>::SharedPtr srvResetConf_;

    //Subscribers
    rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr filterSub_;
    rclcpp::Subscription<ulisse_msgs::msg::VehicleStatus>::SharedPtr vehicleStatusSub_;
    rclcpp::Subscription<ulisse_msgs::msg::ReferenceVelocities>::SharedPtr referenceVelocitiesSub_;

    //Publishers
    rclcpp::Publisher<ulisse_msgs::msg::ThrustersReference>::SharedPtr thrusterDataPub_;// = this->create_publisher<ulisse_msgs::msg::ThrustersReference>(ulisse_msgs::topicnames::llc_thrusters_reference_perc, 1);
    rclcpp::Publisher<ulisse_msgs::msg::ThrusterMappingControl>::SharedPtr thrusterMappigPub_;// = this->create_publisher<ulisse_msgs::msg::ThrusterMappingControl>(ulisse_msgs::topicnames::thruster_mapping_control, 1);
    rclcpp::Publisher<ulisse_msgs::msg::SimulatedVelocitySensor>::SharedPtr simulatedVelocitySensorPub_;// = this->create_publisher<ulisse_msgs::msg::SimulatedVelocitySensor>(ulisse_msgs::topicnames::simulated_velocity_sensor, 1);
    rclcpp::Publisher<ulisse_msgs::msg::DynamicPidControl>::SharedPtr classicPidControlPub_;// = this->create_publisher<ulisse_msgs::msg::DynamicPidControl>(ulisse_msgs::topicnames::classic_pid_control, 1);
    rclcpp::Publisher<ulisse_msgs::msg::DynamicPidControl>::SharedPtr computedTorqueControlPub_;// = this->create_publisher<ulisse_msgs::msg::DynamicPidControl>(ulisse_msgs::topicnames::computed_torque_control, 1);
    rclcpp::Publisher<ulisse_msgs::msg::STSMControl>::SharedPtr stsmControlPub_;//

    //local variables
    ulisse_msgs::msg::ThrusterMappingControl thrusterMappingMsg;
    ulisse_msgs::msg::ThrustersReference thrustersReference;
    ulisse_msgs::msg::DynamicPidControl classicPidControlMsg, computedTorqueMsg;
    ulisse_msgs::msg::SimulatedVelocitySensor simulatedVelocitySensor;
    ulisse_msgs::msg::STSMControl stsmControlMsg;

    //feedback from nav filter
    //double surgeFbk = 0.0;
    //double yawRateFbk = 0.0;

    double motorLeft = 0.0, motorRight = 0.0;

    //Surge pid for thrusterMapping control (TM)
    ctb::DigitalPID pidSurgeTM;

    //Pid for classic pid control (CP)
    ctb::DigitalPID pidYawRateCP;
    ctb::DigitalPID pidSurgeCP;

    //Pid for computed torque control (CT)
    ctb::DigitalPID pidYawRateCT;
    ctb::DigitalPID pidSurgeCT;

    //Variables for STSM
    Eigen::Vector3d sigma_stsm;
    Eigen::Vector3d z_stsm; 
    Eigen::Vector3d tau_stsm_2;
    Eigen::Matrix3d L;

    Eigen::Vector2d tau = Eigen::Vector2d::Zero();

    void ResetConfHandler(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Request> request,
        std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Response> response);

    bool LoadDclConfiguration(std::shared_ptr<DCLConfiguration> conf, std::string filename);

    void ThrusterMappingInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pid);
    void ClassicPidControlInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pidSurge, ctb::DigitalPID& pidYawRate);
    void ComputedTorqueControlInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pidSurge, ctb::DigitalPID& pidYawRate);
    void STSMControlInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime); //funzione da definire per inizializzare il controllo STSM
    void FilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg);
    void ReferenceVelocitiesCB(const ulisse_msgs::msg::ReferenceVelocities::SharedPtr msg);
    void VehicleStatusCB(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg);
    
    Eigen::Vector3d compute_z(Eigen::Vector3d z, Eigen::Matrix3d L, Eigen::Matrix3d M, Eigen::Vector3d v_r,
                                Eigen::Vector3d tau_DC, Eigen::Vector3d tau_controllo);
    Eigen::Vector3d compute_d_hat(Eigen::Vector3d z, Eigen::Matrix3d L, Eigen::Matrix3d M, Eigen::Vector3d v_r);
    Eigen::Vector3d compute_tau_eq(const Eigen::Vector3d &tau_DC, const Eigen::Vector3d &d_hat);
    Eigen::Vector3d compute_tau_stsm_1(const Eigen::Vector3d &k, const Eigen::Vector3d &sigma, double delta);
    Eigen::Vector3d compute_tau_stsm_2(const Eigen::Vector3d &K, const Eigen::Vector3d &sigma,
                                                        const Eigen::Vector3d &tau_stsm_2, double delta, double Ts);

public:
    DynamicVehicleController(std::string file_name);
    virtual ~DynamicVehicleController();

    void Run();
    void PublishControl();

};
}
#endif // ULISSE_CTRL_DYNAMIC_VEHICLE_CONTROLLER_HPP
