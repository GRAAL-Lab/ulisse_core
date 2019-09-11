#include "ulisse_ctrl/vehiclecontroller.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/helper_functions.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include <chrono>
#include <iomanip>

// Libraries
#include <fsm/fsm.h>
#include <ikcl/ikcl.h>
#include <iostream>
#include <pwd.h>
#include <rml/RML.h>
#include <tpik/Action.h>
#include <tpik/TPIKlib.h>
#include <unistd.h>

#include <std_msgs/msg/string.hpp>

#include <ulisse_ctrl/configuration.h>
#include <ulisse_ctrl/geometry_defines.h>
#include <ulisse_ctrl/tasks/SafetyBoundaries.h>
#include <ulisse_ctrl/ulisse_definitions.h>

#include "ulisse_msgs/srv/set_boundaries.hpp"

#include <jsoncpp/json/json.h>

#include <libconfig.h++>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace ulisse {

VehicleController::VehicleController(const rclcpp::Node::SharedPtr& nh, double sampleTime)
    : nh_(nh)
    , sampleTime_(sampleTime)
    , boundaries_set(false)
{
    par_client_ = std::make_shared<rclcpp::SyncParametersClient>(nh_);
    ctrlCxt_ = std::make_shared<ControlContext>();
    goalCxt_ = std::make_shared<GoalContext>();
    statusCxt_ = std::make_shared<StatusContext>();
    conf_ = std::make_shared<ControllerConfiguration>();

    cruise_ = 0;

    while (!par_client_->wait_for_service(1ms)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(0);
        }
        RCLCPP_INFO(nh_->get_logger(), "service not available, waiting again...");
    }

    LoadConfiguration();

    // Sensor Subscriptions
    gps_sub_ = nh_->create_subscription<ulisse_msgs::msg::GPSData>(
        ulisse_msgs::topicnames::sensor_gps_data, std::bind(&VehicleController::GPSSensorCB, this, _1));

    //    compass_sub_ = nh_->create_subscription<ulisse_msgs::msg::Compass>(
    //        ulisse_msgs::topicnames::sensor_compass, std::bind(&VehicleController::CompassSensorCB, this, _1));

    nav_filter_sub_ = nh_->create_subscription<ulisse_msgs::msg::NavFilterData>(
        ulisse_msgs::topicnames::nav_filter_data, std::bind(&VehicleController::NavFilterCB, this, _1));

    // Control Publishers
    ctrlcxt_pub_ = nh_->create_publisher<ulisse_msgs::msg::ControlContext>(ulisse_msgs::topicnames::control_context);
    goalcxt_pub_ = nh_->create_publisher<ulisse_msgs::msg::GoalContext>(ulisse_msgs::topicnames::goal_context);
    statuscxt_pub_ = nh_->create_publisher<ulisse_msgs::msg::StatusContext>(ulisse_msgs::topicnames::status_context);

    generic_log_pub_ = nh_->create_publisher<std_msgs::msg::String>("/ulisse/log/generic");

    /// TPIK Manager
    action_manager = std::make_shared<tpik::ActionManager>(tpik::ActionManager());

    /// ROBOT MODEL
    Eigen::TransfMatrix wTv;

    /// Jacobian
    Eigen::Matrix6d J_ASV;
    J_ASV.setIdentity();

    // Robot Model
    robot_model = std::make_shared<rml::RobotModel>(wTv, ulisse::robotModelID::ASV, J_ASV);

    // Vehicle Angles wrt world (x, y, z)
    vehiclePose_ = std::make_shared<Eigen::Vector6d>();

    // ***** SETUP TASKS *****

    // ASV CONTROL VELOCITY LINEAR
    asv_control_velocity_linear = std::make_shared<ikcl::LinearVelocity>(
        ikcl::LinearVelocity(ulisse::task::asv_control_velocity_linear, robot_model, ulisse::robotModelID::ASV));
    asv_control_velocity_linear->SetVelocity(Eigen::VectorXd::Zero(3));
    equality_task.push_back(asv_control_velocity_linear);
    task_hierarchy.push_back(asv_control_velocity_linear);
    taskIDMap.insert(std::make_pair(ulisse::task::asv_control_velocity_linear, asv_control_velocity_linear));
    taskLogPublisherMap.insert(std::make_pair(ulisse::task::asv_control_velocity_linear, nh_->create_publisher<ulisse_msgs::msg::TaskStatus>("/ulisse/log/task/asv_control_velocity_linear")));


    // AUV CONTROL VELOCITY ANGULAR
    asv_control_velocity_angular = std::make_shared<ikcl::AngularVelocity>(
        ikcl::AngularVelocity(ulisse::task::asv_control_velocity_angular, robot_model, ulisse::robotModelID::ASV));
    asv_control_velocity_angular->SetVelocity(Eigen::VectorXd::Zero(3));
    equality_task.push_back(asv_control_velocity_angular);
    task_hierarchy.push_back(asv_control_velocity_angular);
    taskIDMap.insert(std::make_pair(ulisse::task::asv_control_velocity_angular, asv_control_velocity_angular));
    taskLogPublisherMap.insert(std::make_pair(ulisse::task::asv_control_velocity_angular, nh_->create_publisher<ulisse_msgs::msg::TaskStatus>("/ulisse/log/task/asv_control_velocity_angular")));

    // ASV ANGULAR POSITION
    asv_angular_position = std::make_shared<ikcl::AngularPosition>(
        ikcl::AngularPosition(ulisse::task::asv_angular_position, robot_model, ulisse::robotModelID::ASV, tpik::CartesianTaskType::Equality));
    asv_angular_position->SetVehiclePoseStatus(vehiclePose_);
    cartesian_task.push_back(asv_angular_position);
    task_hierarchy.push_back(asv_angular_position);
    taskIDMap.insert(std::make_pair(ulisse::task::asv_angular_position, asv_angular_position));
    taskLogPublisherMap.insert(std::make_pair(ulisse::task::asv_angular_position, nh_->create_publisher<ulisse_msgs::msg::TaskStatus>("/ulisse/log/task/asv_angular_position")));


    // ASV CONTROL DISTANCE
    asv_control_distance = std::make_shared<ikcl::ControlDistance>(
        ikcl::ControlDistance(ulisse::task::asv_control_distance, robot_model, ulisse::robotModelID::ASV, tpik::CartesianTaskType::Equality));
    asv_control_distance->SetStatusContext(statusCxt_);
    asv_control_distance->SetGoalContext(goalCxt_);
    cartesian_task.push_back(asv_control_distance);
    task_hierarchy.push_back(asv_control_distance);
    taskIDMap.insert(std::make_pair(ulisse::task::asv_control_distance, asv_control_distance));
    taskLogPublisherMap.insert(std::make_pair(ulisse::task::asv_control_distance, nh_->create_publisher<ulisse_msgs::msg::TaskStatus>("/ulisse/log/task/asv_control_distance")));


    // ASV HOLD POSITION
    asv_hold_position = std::make_shared<ikcl::Hold>(
        ikcl::Hold(ulisse::task::asv_hold_position, robot_model, ulisse::robotModelID::ASV));
    asv_hold_position->SetStatusContext(statusCxt_);
    asv_hold_position->SetGoalContext(goalCxt_);
    asv_hold_position->SetConf(conf_);
    equality_task.push_back(asv_hold_position);
    task_hierarchy.push_back(asv_hold_position);
    taskIDMap.insert(std::make_pair(ulisse::task::asv_hold_position, asv_hold_position));
    taskLogPublisherMap.insert(std::make_pair(ulisse::task::asv_hold_position, nh_->create_publisher<ulisse_msgs::msg::TaskStatus>("/ulisse/log/task/asv_hold_position")));


    /*
    // ASV MAKE CURVE
    asv_make_curve = std::make_shared<ikcl::MakeCurve>(
        ikcl::MakeCurve(ulisse::task::asv_make_curve, robot_model, ulisse::robotModelID::ASV));
    asv_make_curve->SetAngularVelocityTask(asv_control_velocity_angular);
    asv_make_curve->SetLinearVelocityTask(asv_control_velocity_linear);
    asv_make_curve->SetStatusContext(statusCxt_);
    asv_make_curve->SetControlContext(ctrlCxt_);
    asv_make_curve->SetSaturationLimits(9.0, 9.0);
    equality_task.push_back(asv_make_curve);
    task_hierarchy.push_back(asv_make_curve);
    taskIDMap.insert(std::make_pair(ulisse::task::asv_make_curve, asv_make_curve));
     */

    // ASV SAFETY BOUNDARIES (INEQUALITY TASK)
    asv_safety_boundaries = std::make_shared<ikcl::SafetyBoundaries>(
        ikcl::SafetyBoundaries(ulisse::task::asv_safety_boundaries, robot_model, ulisse::robotModelID::ASV));
    asv_safety_boundaries->SetPose(vehiclePose_);
    asv_safety_boundaries->SetConf(conf_);
    asv_safety_boundaries->SetControlContext(ctrlCxt_);
   asv_safety_boundaries->SetGoalContext(goalCxt_);
    inequality_task.push_back(asv_safety_boundaries);
    task_hierarchy.push_back(asv_safety_boundaries);
    taskIDMap.insert(std::make_pair(ulisse::task::asv_safety_boundaries, asv_safety_boundaries));
    taskLogPublisherMap.insert(std::make_pair(ulisse::task::asv_safety_boundaries, nh_->create_publisher<ulisse_msgs::msg::TaskStatus>("/ulisse/log/task/asv_safety_boundaries")));


    // Initialize Solver and iCAT
    dof = 6;
    i_cat = std::make_shared<tpik::iCAT>(tpik::iCAT(dof));

    // Setup Params for Tasks and iCAT
    LoadKCLConfiguration();

    // Solver definition
    solver = std::make_shared<tpik::Solver>(tpik::Solver(action_manager, i_cat));

    // FSM Initialization
    SetUpFSM();

    // Command Server Setup
    SetupCommandServer();

    // Create a callback function for when service set boundaries requests are received.
    auto handle_set_boundaries = [this](
                                     const std::shared_ptr<rmw_request_id_t> request_header,
                                     const std::shared_ptr<ulisse_msgs::srv::SetBoundaries::Request> request,
                                     std::shared_ptr<ulisse_msgs::srv::SetBoundaries::Response> response) -> void {
        (void)request_header;
        RCLCPP_INFO(nh_->get_logger(), "Incoming request for set boundaries");

        Json::Reader reader;
        Json::Value obj, obj2;

        reader.parse(request->boundaries_json, obj);

        std::string polygon_ = "polygon((";

        std::string polygon_2_ = "polygon((";
        double latitude, longitude, lam, lom;
        try {
            bool first = true;
            for (Json::Value c : obj["values"]) {
                if (first) {
                    first = false;
                } else {
                    polygon_ = polygon_ + ", ";
                    polygon_2_ = polygon_2_ + ", ";
                }
                reader.parse(c.toStyledString(), obj2);

                latitude = obj2["latitude"].asDouble();
                longitude = obj2["longitude"].asDouble();

                lam = lat_to_m_coeff(statusCxt_->vehiclePos.latitude);
                lom = lon_to_m_coeff(statusCxt_->vehiclePos.longitude);
                double* p = point_map2euclidean(latitude, longitude, statusCxt_->vehiclePos, lam, lom);

                polygon_ = polygon_ + boost::lexical_cast<std::string>(p[0]) + " " + boost::lexical_cast<std::string>(p[1]);
                polygon_2_ = polygon_2_ + boost::lexical_cast<std::string>(latitude) + " " + boost::lexical_cast<std::string>(longitude);

            }
        } catch (Json::Exception& e) {
            // output exception information
            boundaries_set = false;
            response->res = "SetBound::error";
            std::cout << "Set Bound Error";
        }

        polygon_ = polygon_ + "))";
        polygon_2_ = polygon_2_ + "))";

        if (asv_safety_boundaries->InitializePoly(statusCxt_->vehiclePos, polygon_, polygon_2_)) {
            boundaries_set = true;
            boundaries_json = request->boundaries_json;
            response->res = "SetBound::ok";
        } else {
            boundaries_set = false;
            response->res = "SetBound::error";
        }

        if(request->bound_min > 0 && request->bound_max > 0){
            asv_safety_boundaries->SetBoundaries(request->bound_min, request->bound_max);
        }

        std::stringstream log;
        log << "Setting Bounding Box: " << request->boundaries_json;
        publishLog(log.str().c_str());
    };

    srv_boundaries = nh_->create_service<ulisse_msgs::srv::SetBoundaries>(
        ulisse_msgs::topicnames::set_boundaries_service, handle_set_boundaries);

    // Create a callback function for when service requests are received.
    auto handle_set_cruise_control = [this](
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<ulisse_msgs::srv::SetCruiseControl::Request> request,
            std::shared_ptr<ulisse_msgs::srv::SetCruiseControl::Response> response) -> void {
        (void)request_header;
        RCLCPP_INFO(nh_->get_logger(), "Incoming request for set cruise control");

        std::stringstream log;
        log << "Cruise Control set to: " << request->cruise_control;
        publishLog(log.str().c_str());

        state_navigate_.SetCruiseControl(request->cruise_control);
        state_latlong_.SetCruiseControl(request->cruise_control);
        goalCxt_->goalSurge = request->cruise_control;

        cruise_ = request->cruise_control;
        response->res = "SetCruiseControl::ok";
    };

    srv_cruise = nh_->create_service<ulisse_msgs::srv::SetCruiseControl>(
            ulisse_msgs::topicnames::set_cruise_control_service, handle_set_cruise_control);


    // Create a callback function for when service reset configuration requests are received.
    auto handle_reset_conf = [this](
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Request> request,
            std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Response> response) -> void {
        (void)request_header;
        RCLCPP_INFO(nh_->get_logger(), "Incoming request for reset conf");

        publishLog("Configuration Reset");
        LoadKCLConfiguration();

        response->res = "ResetConfiguration::ok";
    };

    srv_reset_conf = nh_->create_service<ulisse_msgs::srv::ResetConfiguration>(
            ulisse_msgs::topicnames::reset_configuration_service, handle_reset_conf);


    // Create a callback function for when service requests are received.
    auto handle_get_bounds = [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ulisse_msgs::srv::GetBoundaries::Request> request,
        std::shared_ptr<ulisse_msgs::srv::GetBoundaries::Response> response) -> void {
        (void)request_header;
        RCLCPP_INFO(nh_->get_logger(), "Incoming request for get boundaries");

        if(boundaries_set){
            response->res = boundaries_json;
        }
        else{
            response->res = "NoBoundSet";
        }
    };

    srv_get_boundaries = nh_->create_service<ulisse_msgs::srv::GetBoundaries>(
        ulisse_msgs::topicnames::get_boundaries_service, handle_get_bounds);
}

VehicleController::~VehicleController() {}

std::shared_ptr<ControlContext> VehicleController::CtrlContext() const { return ctrlCxt_; }

int VehicleController::LoadConfiguration()
{
    LoadControllerConfiguration(conf_, par_client_);

    std::cout << tc::grayD << *conf_ << tc::none << std::endl;

    return true;
}

void VehicleController::publishLog(std::string log){
    std_msgs::msg::String generic_log_pub_msg;
    generic_log_pub_msg.data = log;
    generic_log_pub_->publish(generic_log_pub_msg);
}

void VehicleController::LoadKCLConfiguration(){

    libconfig::Config confObj;

    // Action Manager initialization
    std::string homedir;
    homedir = getenv("HOME");
    std::stringstream conf_path;
    conf_path << homedir << "/group_project/ros2_ws/src/ulisse_core/ulisse_ctrl/conf/Tasks.conf";

    std::string confPath = conf_path.str().c_str();

    try {
        confObj.readFile(confPath.c_str());
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return;
    }

    asv_safety_boundaries->SetBoundaries(confObj.lookup("task.ASV_safety_boundaries.BoundaryMinimumDistance"),
                                         confObj.lookup("task.ASV_safety_boundaries.BoundaryMaximumDistance"));
    asv_safety_boundaries->SetAlphaMinOnTurning(confObj.lookup("task.ASV_safety_boundaries.AlphaMin"));
    asv_safety_boundaries->SetDesiredSpeedOnTurning(confObj.lookup("task.ASV_safety_boundaries.DesiredSpeed"));

    state_navigate_.SetMaxRangeAbscissa(confObj.lookup("task.PathFollowing.MaximumLookupAbscissa"));
    state_navigate_.SetDelta(confObj.lookup("task.PathFollowing.Delta"));
    state_navigate_.SetTolleranceStartingPoint(confObj.lookup("task.PathFollowing.TolleranceStartingPoint"));
    state_navigate_.SetTolleranceEndingPoint(confObj.lookup("task.PathFollowing.TolleranceEndingPoint"));
    state_navigate_.SetTolleranceStartingAngle(confObj.lookup("task.PathFollowing.TolleranceStartingAngle"));
    state_navigate_.SetLineOfSightMethod(confObj.lookup("task.PathFollowing.UseLineOfSight"));

    asv_angular_position->SetConfidence(confObj.lookup("task.ASV_angular_position.confidence"));

    // Action Manager initialization
    std::stringstream conf_path_priority_level;
    conf_path_priority_level << homedir << "/group_project/ros2_ws/src/ulisse_core/ulisse_ctrl/conf/PriorityLevel.conf";
    InitializeUnifiedHierarchyAndActions(action_manager, taskIDMap, conf_path_priority_level.str().c_str());

    // Configure all tasks, each with the correspondent parameters
    std::stringstream conf_tasks;
    conf_tasks << homedir << "/group_project/ros2_ws/src/ulisse_core/ulisse_ctrl/conf/Tasks.conf";
    ConfigureEqualityTaskFromFile(equality_task, conf_tasks.str().c_str());
    ConfigureInequalityTaskFromFile(inequality_task, conf_tasks.str().c_str());
    ConfigureCartesianTaskFromFile(cartesian_task, conf_tasks.str().c_str());
    ConfigureActionTaskFromFile(action_task, conf_tasks.str().c_str());

    // Set Saturation values for the iCAT (read from conf file)
    int dof = 6;
    Eigen::VectorXd saturationMax(dof);
    Eigen::VectorXd saturationMin(dof);
    std::string saturationMaxProperty
        = ulisse::priorityLevelParameter::priorityLevel + "." + ulisse::priorityLevelParameter::saturationMax;
    std::string saturationMinProperty
        = ulisse::priorityLevelParameter::priorityLevel + "." + ulisse::priorityLevelParameter::saturationMin;

    GetVectorEigen(conf_path_priority_level.str().c_str(), saturationMaxProperty, saturationMax);
    GetVectorEigen(conf_path_priority_level.str().c_str(), saturationMinProperty, saturationMin);
    i_cat->SetSaturation(saturationMax, saturationMin);
}

void VehicleController::SetUpFSM()
{
    // ***** COMMANDS *****

    // Halt
    command_halt_.SetFSM(&u_fsm_);

    // Hold
    command_hold_.SetFSM(&u_fsm_);
    command_hold_.SetGoalContext(goalCxt_);
    command_hold_.SetControlContext(ctrlCxt_);
    command_hold_.SetStatusContext(statusCxt_);
    command_hold_.SetAcceptanceRadius(conf_->holdData.defaultRadius);

    // LatLong
    command_latlong_.SetFSM(&u_fsm_);
    command_latlong_.SetGoalContext(goalCxt_);
    command_latlong_.SetControlContext(ctrlCxt_);

    // SpeedHeading
    command_speedheading_.SetFSM(&u_fsm_);
    command_speedheading_.SetGoalContext(goalCxt_);
    command_speedheading_.SetControlContext(ctrlCxt_);

    // Navigate
    command_navigate_.SetFSM(&u_fsm_);

    // ***** STATES *****

    // Halt
    state_halt_.SetFSM(&u_fsm_);
    state_halt_.SetStatusContext(statusCxt_);
    state_halt_.SetGoalContext(goalCxt_);
    state_halt_.SetCtrlContext(ctrlCxt_);
    state_halt_.SetConf(conf_);
    state_halt_.SetActionManager(action_manager);
    state_halt_.SetUnifiedHierarchy(task_hierarchy);
    state_halt_.SetRobotModel(robot_model);
    state_halt_.SetLinearVelocityTask(asv_control_velocity_linear);
    state_halt_.SetAngularVelocityTask(asv_control_velocity_angular);

    // Hold
    state_hold_.SetFSM(&u_fsm_);
    state_hold_.SetStatusContext(statusCxt_);
    state_hold_.SetGoalContext(goalCxt_);
    state_hold_.SetCtrlContext(ctrlCxt_);
    state_hold_.SetConf(conf_);
    state_hold_.SetActionManager(action_manager);
    state_hold_.SetUnifiedHierarchy(task_hierarchy);
    state_hold_.SetRobotModel(robot_model);
    state_hold_.SetHoldTask(asv_hold_position);

    // LatLong
    state_latlong_.SetFSM(&u_fsm_);
    state_latlong_.SetStatusContext(statusCxt_);
    state_latlong_.SetGoalContext(goalCxt_);
    state_latlong_.SetCtrlContext(ctrlCxt_);
    state_latlong_.SetConf(conf_);
    state_latlong_.SetActionManager(action_manager);
    state_latlong_.SetUnifiedHierarchy(task_hierarchy);
    state_latlong_.SetRobotModel(robot_model);
    state_latlong_.SetDistanceTask(asv_control_distance);
    state_latlong_.SetAngularPositionTask(asv_angular_position);
    state_latlong_.SetASVHoldTask(asv_hold_position);

    // SpeedHeading
    state_speedheading_.SetFSM(&u_fsm_);
    state_speedheading_.SetStatusContext(statusCxt_);
    state_speedheading_.SetGoalContext(goalCxt_);
    state_speedheading_.SetCtrlContext(ctrlCxt_);
    state_speedheading_.SetConf(conf_);
    state_speedheading_.SetActionManager(action_manager);
    state_speedheading_.SetUnifiedHierarchy(task_hierarchy);
    state_speedheading_.SetRobotModel(robot_model);
    state_speedheading_.SetLinearVelocityTask(asv_control_velocity_linear);
    state_speedheading_.SetAngularPositionTask(asv_angular_position);

    // Navigate
    state_navigate_.SetFSM(&u_fsm_);
    state_navigate_.SetStatusContext(statusCxt_);
    state_navigate_.SetGoalContext(goalCxt_);
    state_navigate_.SetCtrlContext(ctrlCxt_);
    state_navigate_.SetConf(conf_);
    state_navigate_.SetActionManager(action_manager);
    state_navigate_.SetUnifiedHierarchy(task_hierarchy);
    state_navigate_.SetRobotModel(robot_model);
    state_navigate_.SetLinearVelocityTask(asv_control_velocity_linear);
    state_navigate_.SetAngularVelocityTask(asv_control_velocity_angular);
    state_navigate_.SetAngularPositionTask(asv_angular_position);
    state_navigate_.SetASVHoldTask(asv_hold_position);
    state_navigate_.SetDistanceTask(asv_control_distance);

    // ***** EVENTS *****
    event_rc_enabled_.SetFSM(&u_fsm_);
    event_rc_enabled_.SetCtrlContext(ctrlCxt_);

    // ***** CONFIGURE FSM *****
    // ADD COMMANDS
    u_fsm_.AddCommand(ulisse::commands::ID::halt, &command_halt_);
    u_fsm_.AddCommand(ulisse::commands::ID::hold, &command_hold_);
    u_fsm_.AddCommand(ulisse::commands::ID::latlong, &command_latlong_);
    u_fsm_.AddCommand(ulisse::commands::ID::speedheading, &command_speedheading_);
    u_fsm_.AddCommand(ulisse::commands::ID::navigate, &command_navigate_);

    // ADD STATES
    u_fsm_.AddState(ulisse::states::ID::halt, &state_halt_);
    u_fsm_.AddState(ulisse::states::ID::hold, &state_hold_);
    u_fsm_.AddState(ulisse::states::ID::latlong, &state_latlong_);
    u_fsm_.AddState(ulisse::states::ID::speedheading, &state_speedheading_);
    u_fsm_.AddState(ulisse::states::ID::navigate, &state_navigate_);

    // ADD EVENTS
    u_fsm_.AddEvent(ulisse::events::names::rcenabled, &event_rc_enabled_);

    // ENABLE TRANSITIONS
    u_fsm_.EnableTransition(ulisse::states::ID::halt, ulisse::states::ID::hold, true);
    u_fsm_.EnableTransition(ulisse::states::ID::halt, ulisse::states::ID::latlong, true);
    u_fsm_.EnableTransition(ulisse::states::ID::halt, ulisse::states::ID::speedheading, true);
    u_fsm_.EnableTransition(ulisse::states::ID::halt, ulisse::states::ID::navigate, true);

    u_fsm_.EnableTransition(ulisse::states::ID::hold, ulisse::states::ID::halt, true);
    u_fsm_.EnableTransition(ulisse::states::ID::hold, ulisse::states::ID::latlong, true);
    u_fsm_.EnableTransition(ulisse::states::ID::hold, ulisse::states::ID::speedheading, true);
    u_fsm_.EnableTransition(ulisse::states::ID::hold, ulisse::states::ID::navigate, true);

    u_fsm_.EnableTransition(ulisse::states::ID::latlong, ulisse::states::ID::hold, true);
    u_fsm_.EnableTransition(ulisse::states::ID::latlong, ulisse::states::ID::halt, true);
    u_fsm_.EnableTransition(ulisse::states::ID::latlong, ulisse::states::ID::speedheading, true);
    u_fsm_.EnableTransition(ulisse::states::ID::latlong, ulisse::states::ID::navigate, true);

    u_fsm_.EnableTransition(ulisse::states::ID::speedheading, ulisse::states::ID::hold, true);
    u_fsm_.EnableTransition(ulisse::states::ID::speedheading, ulisse::states::ID::halt, true);
    u_fsm_.EnableTransition(ulisse::states::ID::speedheading, ulisse::states::ID::latlong, true);
    u_fsm_.EnableTransition(ulisse::states::ID::speedheading, ulisse::states::ID::navigate, true);

    u_fsm_.EnableTransition(ulisse::states::ID::navigate, ulisse::states::ID::hold, true);
    u_fsm_.EnableTransition(ulisse::states::ID::navigate, ulisse::states::ID::halt, true);
    u_fsm_.EnableTransition(ulisse::states::ID::navigate, ulisse::states::ID::latlong, true);
    u_fsm_.EnableTransition(ulisse::states::ID::navigate, ulisse::states::ID::speedheading, true);

    // ENABLE COMMANDS
    u_fsm_.EnableCommandInState(ulisse::states::ID::halt, ulisse::commands::ID::halt, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::hold, ulisse::commands::ID::halt, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::latlong, ulisse::commands::ID::halt, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::speedheading, ulisse::commands::ID::halt, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::navigate, ulisse::commands::ID::halt, true);

    u_fsm_.EnableCommandInState(ulisse::states::ID::halt, ulisse::commands::ID::hold, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::hold, ulisse::commands::ID::hold, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::latlong, ulisse::commands::ID::hold, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::speedheading, ulisse::commands::ID::hold, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::navigate, ulisse::commands::ID::hold, true);

    u_fsm_.EnableCommandInState(ulisse::states::ID::halt, ulisse::commands::ID::latlong, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::hold, ulisse::commands::ID::latlong, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::latlong, ulisse::commands::ID::latlong, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::speedheading, ulisse::commands::ID::latlong, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::navigate, ulisse::commands::ID::latlong, true);

    u_fsm_.EnableCommandInState(ulisse::states::ID::halt, ulisse::commands::ID::speedheading, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::hold, ulisse::commands::ID::speedheading, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::latlong, ulisse::commands::ID::speedheading, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::speedheading, ulisse::commands::ID::speedheading, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::navigate, ulisse::commands::ID::speedheading, true);

    u_fsm_.EnableCommandInState(ulisse::states::ID::halt, ulisse::commands::ID::navigate, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::hold, ulisse::commands::ID::navigate, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::latlong, ulisse::commands::ID::navigate, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::speedheading, ulisse::commands::ID::navigate, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::navigate, ulisse::commands::ID::navigate, true);

    u_fsm_.SetInitState(ulisse::states::ID::halt);
}

void VehicleController::SetupCommandServer()
{
    // Create a callback function for when service requests are received.
    auto handle_control_commands = [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                       const std::shared_ptr<ulisse_msgs::srv::ControlCommand::Request> request,
                                       std::shared_ptr<ulisse_msgs::srv::ControlCommand::Response> response) -> void {
        (void)request_header;
        RCLCPP_INFO(nh_->get_logger(), "Incoming request: %s", request->command_type.c_str());

        std::stringstream logg;
        logg << "Incoming request: " << request->command_type.c_str();
        publishLog(logg.str().c_str());
        fsm::retval ret = fsm::ok;

        if (!boundaries_set) {
            response->res = "CommandAnswer::NoBoundSet";
            return;
        }

        std::stringstream log;
        if (request->command_type == ulisse::commands::ID::halt) {
            std::cout << "Received Command Halt" << std::endl;
            publishLog("Received Command Halt");
        } else if (request->command_type == ulisse::commands::ID::hold) {
            std::cout << "Received Command Hold" << std::endl;
            ctb::LatLong goal_hold;
            goal_hold.latitude = statusCxt_->vehiclePos.latitude;
            goal_hold.longitude = statusCxt_->vehiclePos.longitude;
            asv_hold_position->SetGoalHold(goal_hold);

            command_hold_.SetAcceptanceRadius(request->hold_cmd.acceptance_radius);

            publishLog("Received Command Hold");
        } else if (request->command_type == ulisse::commands::ID::latlong) {
            std::cout << "Received Command LatLong" << std::endl;
            command_latlong_.SetGoal(request->latlong_cmd.goal.latitude, request->latlong_cmd.goal.longitude,
                request->latlong_cmd.acceptance_radius);
            state_latlong_.SetPointGoTo(request->latlong_cmd.goal.latitude, request->latlong_cmd.goal.longitude,
                request->latlong_cmd.acceptance_radius);

            log << "Received Command GoTo (lat: " << request->latlong_cmd.goal.latitude << " , long: " << request->latlong_cmd.goal.longitude << " )";
            publishLog(log.str().c_str());
        } else if (request->command_type == ulisse::commands::ID::speedheading) {
            std::cout << "Received Command SpeedHeading" << std::endl;
            state_speedheading_.SetSurgeRef(request->sh_cmd.speed);
            command_speedheading_.SetGoal(request->sh_cmd.speed, request->sh_cmd.heading, request->sh_cmd.timeout.sec);
            state_speedheading_.ResetTimer();

            log << "Received Command SpeedHeading (speed: " << request->sh_cmd.speed << " , heading: " << request->sh_cmd.heading << " )";
            publishLog(log.str().c_str());
        } else if (request->command_type == ulisse::commands::ID::navigate) {
            std::cout << "Received Command Path Following" << std::endl;

            if (!state_navigate_.LoadSpur(request->nav_cmd.nurbs_json)) {
                ret = fsm::retval::fail;
            }

            log << "Received Command PathFollowing (nurbs: " << request->nav_cmd.nurbs_json << " )";
            publishLog(log.str().c_str());

        } else {
            RCLCPP_INFO(nh_->get_logger(), "Unsupported command: %s", request->command_type.c_str());
            ret = fsm::retval::fail;
        }

        if (ret != fsm::retval::ok) {
            response->res = "CommandAnswer::fail";
            RCLCPP_INFO(nh_->get_logger(), "SendAnswer returned %s", response->res.c_str());
        } else {
            u_fsm_.ExecuteCommand(request->command_type);
            response->res = "CommandAnswer::ok";
        }
    };

    srv_ = nh_->create_service<ulisse_msgs::srv::ControlCommand>(
        ulisse_msgs::topicnames::control_cmd_service, handle_control_commands);
}

void VehicleController::GPSSensorCB(const ulisse_msgs::msg::GPSData::SharedPtr msg)
{
    timestamp_ = msg->time;
    statusCxt_->gpsSpeed = msg->speed;
    statusCxt_->gpsTrack = msg->track * M_PI / 180.0;

    //std::cout << "GPS (lat, long): " << msg->latitude << ", " << msg->longitude << std::endl;
    //std::cout << "GPS speed: " << msg->speed << std::endl;
    //std::cout << "GPS track: " << msg->track << std::endl;
}

void VehicleController::NavFilterCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg)
{
    statusCxt_->vehiclePos.latitude = msg->latitude;
    statusCxt_->vehiclePos.longitude = msg->longitude;
    statusCxt_->vehicleHeading = msg->orientation.yaw;

    statusCxt_->seacurrent[0] = msg->current[0];
    statusCxt_->seacurrent[1] = msg->current[1];
}

void VehicleController::LLCStatusCB(const ulisse_msgs::msg::LLCStatus::SharedPtr msg)
{
    statusCxt_->llcStatus = msg->status;
}

void VehicleController::Run()
{
    if (!boundaries_set) {
        std::cout << "Waiting for the Safety Bounding Box" << std::endl;
        return;
    }
    // Switch State (if something happens)
    u_fsm_.SwitchState();
    // Process Events
    u_fsm_.ProcessEventQueue();
    // Execute current state
    u_fsm_.ExecuteState();

    // Computing Kinematic Control via TPIK
    y_tpik = solver->ComputeVelocities();

    //
    for (int i = 0; i < y_tpik.size(); i++) {
        if (std::isnan(y_tpik(i))) {
            y_tpik(i) = 0.0;
            RCLCPP_INFO(nh_->get_logger(), "NaN requested velocity");
        }
    }

    if(cruise_ > 0 && y_tpik[3] > cruise_){
        y_tpik[3] = cruise_;
    }

    double headingError;
    if (conf_->enableSlowDownOnTurns) {
        headingError = ctb::HeadingErrorRad(goalCxt_->goalHeadingWithSafety, statusCxt_->vehicleHeading);
        ctrlCxt_->desiredSurge = SlowDownWhenTurning(headingError, y_tpik[3], *conf_);
    } else {
        ctrlCxt_->desiredSurge = y_tpik[3];
    }
    ctrlCxt_->desiredJog = y_tpik[2];

    // Update references for the tasks => x, y, angle on theta
    (*vehiclePose_)(0) = statusCxt_->vehiclePos.latitude;
    (*vehiclePose_)(1) = statusCxt_->vehiclePos.longitude;
    (*vehiclePose_)(5) = statusCxt_->vehicleHeading;

    // Verbose Status to Video
    std::cout << "Current Latitude: " << statusCxt_->vehiclePos.latitude << std::endl;
    std::cout << "Current Longitude: " << statusCxt_->vehiclePos.longitude << std::endl;
    std::cout << "Current Heading: " << statusCxt_->vehicleHeading << std::endl;
    std::cout << "Current Surge: " << statusCxt_->gpsSpeed << std::endl;
    std::cout << "Goal Heading: " << goalCxt_->goalHeading << std::endl;
    std::cout << "Goal Surge: " << goalCxt_->goalSurge << std::endl;
    std::cout << "----------------------------------" << std::endl;
    std::cout << "INTERFACE TO DCL" << std::endl;
    std::cout << "Desired Surge: " << ctrlCxt_->desiredSurge << std::endl;
    std::cout << "Desired Jog: " << ctrlCxt_->desiredJog << std::endl;
    std::cout << "----------------------------------" << std::endl;

    for (auto& task : task_hierarchy) {
        try {

            std::vector<double> diagonal_activation_function;
            for(int i = 0; i < task->GetInternalActivationFunction().rows(); i++){
                diagonal_activation_function.push_back(task->GetInternalActivationFunction().at(i, i));
            }
            std::vector<double> reference;
            for(int i = 0; i < task->GetReference().size(); i++){
                reference.push_back(task->GetReference().at(i));
            }

            ulisse_msgs::msg::TaskStatus taskstatus_msg;

            t_now_ = std::chrono::system_clock::now();
            long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
            auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / (int)1E9);
            auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % (int)1E9);

            taskstatus_msg.stamp.sec = now_stamp_secs;
            taskstatus_msg.stamp.nanosec = now_stamp_nanosecs;
            taskstatus_msg.id = task->GetID();
            taskstatus_msg.is_active = task->GetIsActive();
            taskstatus_msg.activation_function = diagonal_activation_function;
            taskstatus_msg.reference = reference;

            taskLogPublisherMap[task->GetID()]->publish(taskstatus_msg);

        } catch (tpik::ExceptionWithHow& e) {
            std::cerr << "LOG TASK EXCEPTION" << std::endl;
            std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
        }
    }

}

void VehicleController::PublishControl()
{
    ulisse_msgs::msg::StatusContext statuscxt_msg;

    t_now_ = std::chrono::system_clock::now();
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / (int)1E9);
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % (int)1E9);

    statuscxt_msg.stamp.sec = now_stamp_secs;
    statuscxt_msg.stamp.nanosec = now_stamp_nanosecs;
    statuscxt_msg.vehicle_pos.latitude = statusCxt_->vehiclePos.latitude;
    statuscxt_msg.vehicle_pos.longitude = statusCxt_->vehiclePos.longitude;
    statuscxt_msg.vehicle_heading = statusCxt_->vehicleHeading;
    statuscxt_msg.vehicle_speed = statusCxt_->gpsSpeed;
    statuscxt_msg.vehicle_track = statusCxt_->gpsTrack;
    statuscxt_msg.vehicle_state = u_fsm_.GetCurrentStateName();
    statuscxt_pub_->publish(statuscxt_msg);

    ulisse_msgs::msg::GoalContext goalcxt_msg;
    goalcxt_msg.stamp.sec = now_stamp_secs;
    goalcxt_msg.stamp.nanosec = now_stamp_nanosecs;
    goalcxt_msg.current_goal.latitude = goalCxt_->currentGoal.pos.latitude;
    goalcxt_msg.current_goal.longitude = goalCxt_->currentGoal.pos.longitude;
    goalcxt_msg.accept_radius = goalCxt_->currentGoal.acceptRadius;
    goalcxt_msg.goal_distance = goalCxt_->goalDistance;
    goalcxt_msg.goal_heading = goalCxt_->goalHeading;
    goalcxt_msg.goal_speed = goalCxt_->goalSurge;
    goalcxt_pub_->publish(goalcxt_msg);

    ulisse_msgs::msg::ControlContext ctrlcxt_msg;
    ctrlcxt_msg.stamp.sec = now_stamp_secs;
    ctrlcxt_msg.stamp.nanosec = now_stamp_nanosecs;
    ctrlcxt_msg.desired_speed = ctrlCxt_->desiredSurge;
    ctrlcxt_msg.desired_jog = ctrlCxt_->desiredJog;

    ctrlcxt_pub_->publish(ctrlcxt_msg);
}
}
