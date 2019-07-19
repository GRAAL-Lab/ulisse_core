#include "ulisse_ctrl/vehiclecontroller.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/helper_functions.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include <chrono>
#include <iomanip>

// Libraries
#include <fsm/fsm.h>
#include <ikcl/ikcl.h>
#include <rml/RML.h>
#include <tpik/TPIKlib.h>
#include <tpik/Action.h>
#include <pwd.h>
#include <iostream>
#include <unistd.h>

#include <ulisse_ctrl/geometry_defines.h>
#include <ulisse_ctrl/ulisse_definitions.h>
#include <ulisse_ctrl/configuration.h>
#include <ulisse_ctrl/tasks/SafetyBoundaries.h>


#include "example_interfaces/srv/add_two_ints.hpp"
#include "ulisse_msgs/srv/set_boundaries.hpp"

#include <jsoncpp/json/json.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace ulisse {

VehicleController::VehicleController(const rclcpp::Node::SharedPtr& nh, double sampleTime)
    : nh_(nh)
    , sampleTime_(sampleTime)
    , boundaries_set(false) //TODO: cambialo a false
    {
        par_client_ = std::make_shared<rclcpp::SyncParametersClient>(nh_);
        ctrlCxt_ = std::make_shared<ControlContext>();
        goalCxt_ = std::make_shared<GoalContext>();
        statusCxt_ = std::make_shared<StatusContext>();
        conf_ = std::make_shared<ControllerConfiguration>();

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

        /// TPIK AND IKCL VARIABLES DEFINITION
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

        // AUV CONTROL VELOCITY ANGULAR
        asv_control_velocity_angular = std::make_shared<ikcl::AngularVelocity>(
                ikcl::AngularVelocity(ulisse::task::asv_control_velocity_angular, robot_model, ulisse::robotModelID::ASV));
        asv_control_velocity_angular->SetVelocity(Eigen::VectorXd::Zero(3));
        equality_task.push_back(asv_control_velocity_angular);
        task_hierarchy.push_back(asv_control_velocity_angular);
        taskIDMap.insert(std::make_pair(ulisse::task::asv_control_velocity_angular, asv_control_velocity_angular));

        // ASV ANGULAR POSITION
        asv_angular_position = std::make_shared<ikcl::AngularPosition>(
                ikcl::AngularPosition(ulisse::task::asv_angular_position, robot_model, ulisse::robotModelID::ASV, tpik::CartesianTaskType::Equality));
        asv_angular_position->SetVehiclePoseStatus(vehiclePose_);
        cartesian_task.push_back(asv_angular_position);
        task_hierarchy.push_back(asv_angular_position);
        taskIDMap.insert(std::make_pair(ulisse::task::asv_angular_position, asv_angular_position));

        // ASV CONTROL DISTANCE
        asv_control_distance = std::make_shared<ikcl::ControlDistance>(
                ikcl::ControlDistance(ulisse::task::asv_control_distance, robot_model, ulisse::robotModelID::ASV, tpik::CartesianTaskType::Equality));
        asv_control_distance->SetStatusContext(statusCxt_);
        asv_control_distance->SetGoalContext(goalCxt_);
        asv_control_distance->SetConfiguration(conf_);
        cartesian_task.push_back(asv_control_distance);
        task_hierarchy.push_back(asv_control_distance);
        taskIDMap.insert(std::make_pair(ulisse::task::asv_control_distance, asv_control_distance));

        // ASV HOLD POSITION
        asv_hold_position = std::make_shared<ikcl::Hold>(
                ikcl::Hold(ulisse::task::asv_hold_position, robot_model, ulisse::robotModelID::ASV));
        asv_hold_position->SetStatusContext(statusCxt_);
        asv_hold_position->SetGoalContext(goalCxt_);
        asv_hold_position->SetConf(conf_);
        equality_task.push_back(asv_hold_position);
        task_hierarchy.push_back(asv_hold_position);
        taskIDMap.insert(std::make_pair(ulisse::task::asv_hold_position, asv_hold_position));

        // ASV LINEAR POSITION GO TO
        asv_linear_position_go_to = std::make_shared<ikcl::ControlCartesianDistance>(
                ikcl::ControlCartesianDistance(ulisse::task::asv_linear_position_go_to, robot_model, ulisse::robotModelID::ASV,
                                               false, tpik::CartesianTaskType::Equality));
        asv_linear_position_go_to->SetwTg(
                robot_model->GetTransformation(ulisse::robotModelID::ASV), rml::FrameID::WorldFrame);
        cartesian_task.push_back(asv_linear_position_go_to);
        task_hierarchy.push_back(asv_linear_position_go_to);
        taskIDMap.insert(std::make_pair(ulisse::task::asv_linear_position_go_to, asv_linear_position_go_to));

        // ASV DIRECTION OF ALIGNMENT
        asv_direction_alignment = std::make_shared<ikcl::AlignToTarget>(ikcl::AlignToTarget(ulisse::task::asv_direction_alignment, robot_model,
                                                                            ulisse::robotModelID::ASV, tpik::CartesianTaskType::Equality));
        Eigen::Vector3d alignment_axis_direction_alignment(1, 0, 0);
        Eigen::Vector3d normal_to_horizontal_plane(0, 0, 1);
        asv_direction_alignment->SetAlignmentAxis(alignment_axis_direction_alignment);
        asv_direction_alignment->SetDistanceToTarget(alignment_axis_direction_alignment);
        asv_direction_alignment->SetOnPlane(normal_to_horizontal_plane, rml::FrameID::WorldFrame);
        cartesian_task.push_back(asv_direction_alignment);
        task_hierarchy.push_back(asv_direction_alignment);
        taskIDMap.insert(std::make_pair(ulisse::task::asv_direction_alignment, asv_direction_alignment));

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

        // ASV SAFETY BOUNDARIES (INEQUALITY TASK)
        asv_safety_boundaries = std::make_shared<ikcl::SafetyBoundaries>(
                ikcl::SafetyBoundaries(ulisse::task::asv_safety_boundaries, robot_model, ulisse::robotModelID::ASV));

        asv_safety_boundaries->SetPose(vehiclePose_);
        asv_safety_boundaries->SetBoundaries(5.0, 3.0);
        /*
        inequality_task.push_back(asv_safety_boundaries);
        task_hierarchy.push_back(asv_safety_boundaries);
        taskIDMap.insert(std::make_pair(ulisse::task::asv_safety_boundaries, asv_safety_boundaries));
        */

        // Action Manager initialization
        std::string homedir;
        homedir = getenv("HOME");
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

        // Initialize Solver and iCAT
        dof = 6;
        i_cat = std::make_shared<tpik::iCAT>(tpik::iCAT(dof));

        // Set Saturation values for the iCAT (read from conf file)
        Eigen::VectorXd saturationMax(dof);
        Eigen::VectorXd saturationMin(dof);
        std::string saturationMaxProperty
                = ulisse::priorityLevelParameter::priorityLevel + "." + ulisse::priorityLevelParameter::saturationMax;
        std::string saturationMinProperty
                = ulisse::priorityLevelParameter::priorityLevel + "." + ulisse::priorityLevelParameter::saturationMin;

        GetVectorEigen(conf_path_priority_level.str().c_str(), saturationMaxProperty, saturationMax);
        GetVectorEigen(conf_path_priority_level.str().c_str(), saturationMinProperty, saturationMin);
        i_cat->SetSaturation(saturationMax, saturationMin);

        // Solver definition
        solver = std::make_shared<tpik::Solver>(tpik::Solver(action_manager, i_cat));

        // FSM Initialization
        SetUpFSM();

        // Command Server Setup
        SetupCommandServer();


        // Create a callback function for when service requests are received.
        auto handle_set_boundaries = [this](
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<ulisse_msgs::srv::SetBoundaries::Request> request,
                std::shared_ptr<ulisse_msgs::srv::SetBoundaries::Response> response) -> void {
            (void)request_header;
            RCLCPP_INFO(nh_->get_logger(), "Incoming request for set boundaries");

            Json::Reader reader;
            Json::Value obj, obj2;

            double bound_min = request->bound_min;
            double bound_max = request->bound_max;

            asv_safety_boundaries->SetBoundaries(bound_min, bound_max);
            reader.parse(request->boundaries_json, obj);

            std::string polygon_ = "polygon((";

            bool first = true;
            for(Json::Value c : obj["values"]){
                if(first){
                    first = false;
                } else{
                    polygon_ = polygon_ + ", ";
                }
                reader.parse(c.toStyledString(), obj2);
                polygon_ = polygon_ + obj2["latitude"].asString() + " " + obj2["longitude"].asString();
            }

            polygon_ = polygon_ + "))";
            std::cout << "***** PRODOTTO *****: " << polygon_ << std::endl;

            if (asv_safety_boundaries->InitializePoly(statusCxt_->vehiclePos, polygon_))
            {
                boundaries_set = true;
                response->res = "SetBound::ok";
            }
            else{
                boundaries_set = false;
                response->res = "SetBound::error";
            }

        };

        srv_boundaries = nh_->create_service<ulisse_msgs::srv::SetBoundaries>(
                ulisse_msgs::topicnames::set_boundaries_service, handle_set_boundaries);

    }

    VehicleController::~VehicleController() {}

    std::shared_ptr<ControlContext> VehicleController::CtrlContext() const { return ctrlCxt_; }

    int VehicleController::LoadConfiguration()
    {
        LoadControllerConfiguration(conf_, par_client_);

        std::cout << tc::grayD << *conf_ << tc::none << std::endl;

        return true;
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
        state_navigate_.SetASVMakeCurveTask(asv_make_curve);
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

            fsm::retval ret = fsm::ok;

            if (request->command_type == ulisse::commands::ID::halt) {
                std::cout << "Received Command Halt" << std::endl;
            } else if (request->command_type == ulisse::commands::ID::hold) {
                std::cout << "Received Command Hold" << std::endl;
                ctb::LatLong goal_hold;
                goal_hold.latitude = statusCxt_->vehiclePos.latitude;
                goal_hold.longitude = statusCxt_->vehiclePos.longitude;
                asv_hold_position->SetGoalHold(goal_hold);

                command_hold_.SetAcceptanceRadius(request->hold_cmd.acceptance_radius);
            } else if (request->command_type == ulisse::commands::ID::latlong) {
                std::cout << "Received Command LatLong" << std::endl;
                command_latlong_.SetGoal(request->latlong_cmd.goal.latitude, request->latlong_cmd.goal.longitude,
                    request->latlong_cmd.acceptance_radius);
                state_latlong_.SetPointGoTo(request->latlong_cmd.goal.latitude, request->latlong_cmd.goal.longitude,
                        request->latlong_cmd.acceptance_radius);
            } else if (request->command_type == ulisse::commands::ID::speedheading) {
                std::cout << "Received Command SpeedHeading" << std::endl;
                state_speedheading_.SetSurgeRef(request->sh_cmd.speed);
                command_speedheading_.SetGoal(request->sh_cmd.speed, request->sh_cmd.heading, request->sh_cmd.timeout.sec);
                state_speedheading_.ResetTimer();
            }  else if (request->command_type == ulisse::commands::ID::navigate) {
                std::cout << "Received Command Navigate" << std::endl;

                state_navigate_.LoadSpur(request->nav_cmd.centroid_latitude, request->nav_cmd.centroid_longitude, request->nav_cmd.number_of_curves,
                                           request->nav_cmd.curves);

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
        statusCxt_->gpsTrack = msg->track;

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
        if(! boundaries_set)
        {
            std::cout << "Waiting for the Safety Bounding Box";
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

        ctrlCxt_->desiredSurge = y_tpik[3];
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

    }

    void VehicleController::PublishControl()
    {
        ulisse_msgs::msg::StatusContext statuscxt_msg;
        statuscxt_msg.vehicle_pos.latitude = statusCxt_->vehiclePos.latitude;
        statuscxt_msg.vehicle_pos.longitude = statusCxt_->vehiclePos.longitude;
        statuscxt_msg.vehicle_heading = statusCxt_->vehicleHeading;
        statuscxt_msg.vehicle_speed = statusCxt_->gpsSpeed;
        statuscxt_msg.vehicle_track = statusCxt_->gpsTrack;
        statuscxt_msg.vehicle_state = u_fsm_.GetCurrentStateName();
        statuscxt_pub_->publish(statuscxt_msg);

        ulisse_msgs::msg::GoalContext goalcxt_msg;
        goalcxt_msg.current_goal.latitude = goalCxt_->currentGoal.pos.latitude;
        goalcxt_msg.current_goal.longitude = goalCxt_->currentGoal.pos.longitude;
        goalcxt_msg.accept_radius = goalCxt_->currentGoal.acceptRadius;
        goalcxt_msg.goal_distance = goalCxt_->goalDistance;
        goalcxt_msg.goal_heading = goalCxt_->goalHeading;
        goalcxt_msg.goal_speed = goalCxt_->goalSurge;
        goalcxt_pub_->publish(goalcxt_msg);

        ulisse_msgs::msg::ControlContext ctrlcxt_msg;
        ctrlcxt_msg.desired_speed = ctrlCxt_->desiredSurge;
        ctrlcxt_msg.desired_jog = ctrlCxt_->desiredJog;

        ctrlcxt_pub_->publish(ctrlcxt_msg);
    }
}
