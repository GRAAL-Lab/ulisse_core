#include "ulisse_ctrl/vehiclecontroller.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_msgs/terminal_utils.hpp"
#include "ulisse_msgs/topicnames.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <jsoncpp/json/json.h>
#include <ulisse_ctrl/configuration.h>
#include <ulisse_ctrl/geometry_defines.h>
#include <ulisse_ctrl/states/genericstate.hpp>
#include <ulisse_ctrl/ulisse_definitions.h>

using std::placeholders::_1;

namespace ulisse {

VehicleController::VehicleController(const rclcpp::Node::SharedPtr& nh, double sampleTime, std::string file_name)
    : nh_(nh)
    , sampleTime_(sampleTime)
    , boundariesSet_(false)
{
    conf_ = std::make_shared<ControllerConfiguration>();
    vehiclePosition_ = std::make_shared<LatLong>();

    fileName_ = file_name;

    stateHalt_ = std::make_shared<states::StateHalt>();
    stateHold_ = std::make_shared<states::StateHold>();
    statePathFollowing_ = std::make_shared<states::StateNavigate>();
    stateLatLong_ = std::make_shared<states::StateLatLong>();
    stateSpeedHeading_ = std::make_shared<states::StateSpeedHeading>();

    // Sensor Subscriptions
    navFilterSub_ = nh_->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10, std::bind(&VehicleController::NavFilterCB, this, _1));

    // Control Publishers
    genericLogPub_ = nh_->create_publisher<std_msgs::msg::String>("/ulisse/log/generic", 10);
    vehicleStatusPub_ = nh_->create_publisher<ulisse_msgs::msg::VehicleStatus>(ulisse_msgs::topicnames::vehicle_status, 10);
    referenceVelocitiesPub_ = nh_->create_publisher<ulisse_msgs::msg::ReferenceVelocities>(ulisse_msgs::topicnames::reference_velocities, 10);
    feedbackGuiPub_ = nh_->create_publisher<ulisse_msgs::msg::FeedbackGui>(ulisse_msgs::topicnames::feedback_gui, 10);

    /// TPIK Manager
    actionManager_ = std::make_shared<tpik::ActionManager>(tpik::ActionManager());

    /// ROBOT MODEL
    Eigen::TransformationMatrix world_T_vehicle;

    /// Jacobian
    Eigen::Matrix6d J_ASV;
    J_ASV.setIdentity();

    // Robot Model
    robotModel_ = std::make_shared<rml::RobotModel>(world_T_vehicle, ulisse::robotModelID::ASV, J_ASV);

    // ***** SETUP TASKS *****

    // ASV CONTROL VELOCITY LINEAR
    asvLinearVelocity_ = std::make_shared<ikcl::LinearVelocity>(ikcl::LinearVelocity(ulisse::task::asvLinearVelocity, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvLinearVelocity_;
    taskInfo_.taskPub = nh_->create_publisher<ulisse_msgs::msg::TaskStatus>("/ulisse/log/task/ASV_Linear_Velocity", 10);
    tasksMap_.insert(std::make_pair(ulisse::task::asvLinearVelocity, taskInfo_));

    // AUV CONTROL ANGULAR POSITION
    asvAngularPosition_ = std::make_shared<ikcl::AlignToTarget>(ikcl::AlignToTarget(ulisse::task::asvAngularPosition, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvAngularPosition_;
    taskInfo_.taskPub = nh_->create_publisher<ulisse_msgs::msg::TaskStatus>("/ulisse/log/task/ASV_Angular_Position", 10);
    tasksMap_.insert(std::make_pair(ulisse::task::asvAngularPosition, taskInfo_));

    // ASV CONTROL DISTANCE
    asvCartesianDistance_ = std::make_shared<ikcl::CartesianDistance>(ikcl::CartesianDistance(ulisse::task::asvCartesianDistance, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvCartesianDistance_;
    taskInfo_.taskPub = nh_->create_publisher<ulisse_msgs::msg::TaskStatus>("/ulisse/log/task/ASV_Cartesian_Distance", 10);
    tasksMap_.insert(std::make_pair(ulisse::task::asvCartesianDistance, taskInfo_));

    // ASV SAFETY BOUNDARIES (INEQUALITY TASK)
    asvSafetyBoundaries_ = std::make_shared<ikcl::SafetyBoundaries>(ikcl::SafetyBoundaries(ulisse::task::asvSafetyBoundaries, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvSafetyBoundaries_;
    taskInfo_.taskPub = nh_->create_publisher<ulisse_msgs::msg::TaskStatus>("/ulisse/log/task/ASV_Safety_Boundaries", 10);
    tasksMap_.insert(std::make_pair(ulisse::task::asvSafetyBoundaries, taskInfo_));

    // ASV absolute axis alignment task
    asvAbsoluteAxisAlignment_ = std::make_shared<ikcl::AbsoluteAxisAlignment>(ikcl::AbsoluteAxisAlignment(ulisse::task::asvAbsoluteAxisAlignment, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvAbsoluteAxisAlignment_;
    taskInfo_.taskPub = nh_->create_publisher<ulisse_msgs::msg::TaskStatus>("/ulisse/log/task/ASV_Absolute_Axis_Alignment", 10);
    tasksMap_.insert(std::make_pair(ulisse::task::asvAbsoluteAxisAlignment, taskInfo_));

    // ASV absolute axis alignment task
    asvAbsoluteAxisAlignmentSafety_ = std::make_shared<ikcl::AbsoluteAxisAlignment>(ikcl::AbsoluteAxisAlignment(ulisse::task::asvAbsoluteAxisAlignmentSafety, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvAbsoluteAxisAlignmentSafety_;
    taskInfo_.taskPub = nh_->create_publisher<ulisse_msgs::msg::TaskStatus>("/ulisse/log/task/ASV_Absolute_Axis_Alignment_Safety", 10);
    tasksMap_.insert(std::make_pair(ulisse::task::asvAbsoluteAxisAlignmentSafety, taskInfo_));

    // Initialize solver_ and iCAT
    int dof = 6;
    iCat_ = std::make_shared<tpik::iCAT>(tpik::iCAT(dof));
    yTpik_ = Eigen::VectorXd::Zero(dof);

    // load config file
    // Setup Params for Tasks and iCAT
    LoadConfiguration();

    // solver_ definition
    solver_ = std::make_shared<tpik::Solver>(tpik::Solver(actionManager_, iCat_));

    // FSM Initialization
    SetUpFSM();

    // Command Server Setup
    SetupCommandServer();

    // Create a callback function for when service set boundaries requests are  received.
    auto handle_set_boundaries = [this](const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<ulisse_msgs::srv::SetBoundaries::Request> request, std::shared_ptr<ulisse_msgs::srv::SetBoundaries::Response> response)
        -> void {
        (void)request_header;
        RCLCPP_INFO(nh_->get_logger(), "Incoming request for set boundaries");

        if (asvSafetyBoundaries_->InitializePolygon(request->boundaries)) {
            boundariesJson_ = request->boundaries.boundaries_string;
            response->res = "SetBound::ok";
            boundariesSet_ = true;
        } else {
            response->res = "SetBound::error";
        }

        std::stringstream log;
        log << "Setting Bounding Box: " << request->boundaries.boundaries_string;
        publishLog(log.str().c_str());
    };

    srvBoundaries_ = nh_->create_service<ulisse_msgs::srv::SetBoundaries>(ulisse_msgs::topicnames::set_boundaries_service, handle_set_boundaries);

    // Create a callback function for when service requests are received.
    auto handle_set_cruise_control = [this](const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<ulisse_msgs::srv::SetCruiseControl::Request> request, std::shared_ptr<ulisse_msgs::srv::SetCruiseControl::Response> response) -> void {
        (void)request_header;
        RCLCPP_INFO(nh_->get_logger(), "Incoming request for set cruise control");

        std::stringstream log;
        log << "Cruise Control set to: " << request->cruise_control;
        publishLog(log.str().c_str());

        Eigen::VectorXd satMin, satMax;
        iCat_->GetSaturation(satMin, satMax);

        satMax.at(0) = request->cruise_control;

        // Set Saturation values for the iCAT (read from conf file)
        iCat_->SetSaturation(satMin, satMax);

        response->res = "SetCruiseControl::ok";
    };

    srvCruise_ = nh_->create_service<ulisse_msgs::srv::SetCruiseControl>(ulisse_msgs::topicnames::set_cruise_control_service, handle_set_cruise_control);

    // Create a callback function for when service reset configuration requests
    // are received.
    auto handle_reset_conf = [this](const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Request> request, std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Response> response) -> void {
        (void)request_header;
        RCLCPP_INFO(nh_->get_logger(), "Incoming request for reset conf");

        publishLog("Configuration Reset");
        LoadConfiguration();

        response->res = "ResetConfiguration::ok";
    };

    srvResetConf_ = nh_->create_service<ulisse_msgs::srv::ResetConfiguration>(ulisse_msgs::topicnames::reset_configuration_service, handle_reset_conf);

    // Create a callback function for when service requests are received.
    auto handle_get_bounds = [this](const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<ulisse_msgs::srv::GetBoundaries::Request> request, std::shared_ptr<ulisse_msgs::srv::GetBoundaries::Response> response) -> void {
        (void)request_header;
        RCLCPP_INFO(nh_->get_logger(), "Incoming request for get boundaries");

        if (boundariesSet_) {
            response->res = boundariesJson_;
        } else {
            response->res = "NoBoundSet";
        }
    };

    srvGetBoundaries_ = nh_->create_service<ulisse_msgs::srv::GetBoundaries>(ulisse_msgs::topicnames::get_boundaries_service, handle_get_bounds);
}

VehicleController::~VehicleController() {}

bool VehicleController::LoadConfiguration()
{
    libconfig::Config confObj;

    // Inizialization
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("ulisse_ctrl");
    std::string confPath = package_share_directory;
    confPath.append("/conf/");
    confPath.append(fileName_);

    std::cout << "PATH TO CONF FILE : " << confPath << std::endl;

    // Read the file. If there is an error, report it and exit.
    try {
        confObj.readFile(confPath.c_str());
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        return -1;
    } catch (const libconfig::ParseException& pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
        return -1;
    }

    conf_->ConfigureFromFile(confObj);
    std::cout << tc::grayD << *conf_ << tc::none << std::endl;

    //acquired the centroid location
    Eigen::VectorXd centroidLocationTmp;
    ctb::SetParamVector(confObj, centroidLocationTmp, "centroidLocation");
    centroidLocation_.latitude = centroidLocationTmp[0];
    centroidLocation_.longitude = centroidLocationTmp[1];

    ConfigureTaskFromFile(tasksMap_, confObj);
    ConfigurePriorityLevelsFromFile(actionManager_, tasksMap_, confObj);

    // Set Saturation values for the iCAT (read from conf file)
    iCat_->SetSaturation(conf_->saturationMin, conf_->saturationMax);

    ConfigureActionsFromFile(actionManager_, confObj);

    //insert states in the map
    statesMap_.insert({ ulisse::states::ID::halt, stateHalt_ });
    statesMap_.insert({ ulisse::states::ID::hold, stateHold_ });
    statesMap_.insert({ ulisse::states::ID::latlong, stateLatLong_ });
    statesMap_.insert({ ulisse::states::ID::navigate, statePathFollowing_ });
    statesMap_.insert({ ulisse::states::ID::speedheading, stateSpeedHeading_ });

    ConfigureSatesFromFile(statesMap_, confObj);

    //insert command in the map
    commandsMap_.insert({ ulisse::commands::ID::halt, commandHalt_ });
    commandsMap_.insert({ ulisse::commands::ID::hold, commandHold_ });
    commandsMap_.insert({ ulisse::commands::ID::latlong, commandLatLong_ });
    commandsMap_.insert({ ulisse::commands::ID::navigate, commandPathFollowing_ });
    commandsMap_.insert({ ulisse::commands::ID::speedheading, commandSpeedHeading_ });

    return true;
}

void VehicleController::publishLog(std::string log)
{
    std_msgs::msg::String genericLogPub_msg;
    genericLogPub_msg.data = log;
    genericLogPub_->publish(genericLogPub_msg);
}

void VehicleController::SetUpFSM()
{
    // ***** COMMANDS *****
    commandHalt_.SetFSM(&uFsm_);
    commandHalt_.SetState(stateHalt_);

    commandHold_.SetFSM(&uFsm_);
    commandHold_.SetState(stateHold_);

    commandLatLong_.SetFSM(&uFsm_);
    commandLatLong_.SetState(stateLatLong_);

    commandSpeedHeading_.SetFSM(&uFsm_);
    commandSpeedHeading_.SetState(stateSpeedHeading_);

    commandPathFollowing_.SetFSM(&uFsm_);
    commandPathFollowing_.SetState(statePathFollowing_);

    // ***** STATES *****
    //Set the fms and the structure that the states need.
    for (auto& state : statesMap_) {
        state.second->actionManager = actionManager_;
        state.second->robotModel = robotModel_;
        state.second->tasksMap = tasksMap_;
        state.second->vehiclePosition = vehiclePosition_;
        state.second->SetFSM(&uFsm_);
    }

    // ***** EVENTS *****
    eventRcEnabled_.SetFSM(&uFsm_);

    // ***** CONFIGURE FSM *****
    // ADD COMMANDS
    for (auto& command : commandsMap_) {
        uFsm_.AddCommand(command.first, &command.second);
    }

    // ADD STATES
    for (auto& state : statesMap_) {
        uFsm_.AddState(state.first, state.second.get());
    }

    // ADD EVENTS
    uFsm_.AddEvent(ulisse::events::names::rcenabled, &eventRcEnabled_);

    // ENABLE TRANSITIONS
    for (auto& currentState : statesMap_) {

        for (auto& nextState : statesMap_) {

            if (nextState.first != currentState.first)
                uFsm_.EnableTransition(currentState.first, nextState.first, true);
        }
    }

    // ENABLE COMMANDS
    for (auto& state : statesMap_) {
        for (auto& command : commandsMap_) {

            uFsm_.EnableCommandInState(state.first, command.first, true);
        }
    }

    uFsm_.SetInitState(ulisse::states::ID::halt);
}

void VehicleController::SetupCommandServer()
{
    // Create a callback function for when service requests are received.
    auto handle_control_commands = [this](const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<ulisse_msgs::srv::ControlCommand::Request> request, std::shared_ptr<ulisse_msgs::srv::ControlCommand::Response> response) -> void {
        (void)request_header;
        RCLCPP_INFO(nh_->get_logger(), "Incoming request: %s", request->command_type.c_str());

        std::stringstream logg;
        logg << "Incoming request: " << request->command_type.c_str();
        publishLog(logg.str().c_str());
        fsm::retval ret = fsm::ok;

        if (!boundariesSet_) {
            response->res = "CommandAnswer::NoBoundSet";
            return;
        }

        std::stringstream log;
        if (request->command_type == ulisse::commands::ID::halt) {

            std::cout << "Received Command Halt" << std::endl;
            publishLog("Received Command Halt");

        } else if (request->command_type == ulisse::commands::ID::hold) {

            std::cout << "Received Command Hold" << std::endl;
            publishLog("Received Command Hold");

        } else if (request->command_type == ulisse::commands::ID::latlong) {

            std::cout << "Received Command LatLong" << std::endl;
            commandLatLong_.SetGoTo(LatLong(request->latlong_cmd.goal.latitude, request->latlong_cmd.goal.longitude), request->latlong_cmd.acceptance_radius);

            log << "Received Command GoTo (lat: " << request->latlong_cmd.goal.latitude << " , long: " << request->latlong_cmd.goal.longitude << " )";
            publishLog(log.str().c_str());

        } else if (request->command_type == ulisse::commands::ID::speedheading) {

            std::cout << "Received Command SpeedHeading" << std::endl;
            commandSpeedHeading_.SetSpeedHeading(request->sh_cmd.speed, request->sh_cmd.heading, request->sh_cmd.timeout.sec);
            stateSpeedHeading_->ResetTimer();
            log << "Received Command SpeedHeading (speed: " << request->sh_cmd.speed << " , heading: " << request->sh_cmd.heading << " )";
            publishLog(log.str().c_str());

        } else if (request->command_type == ulisse::commands::ID::navigate) {

            std::cout << "Received Command Path Following" << std::endl;

            if (!statePathFollowing_->LoadNurbs(request->nav_cmd.path)) {
                ret = fsm::retval::fail;
            }

            log << "Received Command PathFollowing (nurbs: " << request->nav_cmd.path.nurbs_string << " )";
            publishLog(log.str().c_str());

        } else {
            RCLCPP_INFO(nh_->get_logger(), "Unsupported command: %s", request->command_type.c_str());
            ret = fsm::retval::fail;
        }

        if (ret != fsm::retval::ok) {

            response->res = "CommandAnswer::fail";
            RCLCPP_INFO(nh_->get_logger(), "SendAnswer returned %s", response->res.c_str());
        } else {
            //task update
            for (auto& taskMap : tasksMap_) {
                try {
                    taskMap.second.task->Update();
                } catch (tpik::ExceptionWithHow& e) {
                    std::cerr << "UPDATE TASK EXCEPTION" << std::endl;
                    std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
                }
            }

            uFsm_.ExecuteCommand(request->command_type);
            response->res = "CommandAnswer::ok";
        }
    };

    srv_ = nh_->create_service<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service, handle_control_commands);
}

void VehicleController::NavFilterCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg)
{
    vehiclePosition_->latitude = msg->inertialframe_linear_position.latlong.latitude;
    vehiclePosition_->longitude = msg->inertialframe_linear_position.latlong.longitude;
    // Linear position in world frame
    Eigen::Vector3d worldF_vehicleLinearPosition(vehiclePosition_->latitude, vehiclePosition_->longitude, 0.0);

    // Angualr position in world frame
    rml::EulerRPY rpy{ 0.0, 0.0, msg->bodyframe_angular_position.yaw };

    Eigen::Vector6d velocity_fbk = Eigen::Vector6d::Zero();
    velocity_fbk(0) = msg->bodyframe_linear_velocity.surge;
    velocity_fbk(1) = msg->bodyframe_linear_velocity.sway;

    // Updating the robot model
    Eigen::TransformationMatrix worldF_T_vehicleF;
    worldF_T_vehicleF.TranslationVector(worldF_vehicleLinearPosition);
    worldF_T_vehicleF.RotationMatrix(rpy.ToRotationMatrix());

    robotModel_->PositionOnInertialFrame(worldF_T_vehicleF);
    robotModel_->VelocityVector(ulisse::robotModelID::ASV, velocity_fbk);
}

void VehicleController::Run()
{
    if (!boundariesSet_) {
        std::cout << "Waiting for the Safety Bounding Box" << std::endl;
        return;
    }

    // Switch State (if something happens)
    uFsm_.SwitchState();
    // Process Events
    uFsm_.ProcessEventQueue();
    // Execute current state
    uFsm_.ExecuteState();

    for (auto& taskMap : tasksMap_) {
        try {
            taskMap.second.task->Update();
        } catch (tpik::ExceptionWithHow& e) {
            std::cerr << "UPDATE TASK EXCEPTION" << std::endl;
            std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
        }
    }
    // Computing Kinematic Control via TPIK
    yTpik_ = solver_->ComputeVelocities();

    std::cout << "yTpik_: " << std::endl;
    std::cout << yTpik_ << std::endl;

    for (int i = 0; i < yTpik_.size(); i++) {
        if (std::isnan(yTpik_(i))) {
            yTpik_(i) = 0.0;
            RCLCPP_INFO(nh_->get_logger(), "NaN requested velocity");
        }
    }
}

void VehicleController::PublishControl()
{
    tNow_ = std::chrono::system_clock::now();
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow_.time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

    ulisse_msgs::msg::ReferenceVelocities referenceVelocities;
    referenceVelocities.stamp.sec = now_stamp_secs;
    referenceVelocities.stamp.nanosec = now_stamp_nanosecs;
    referenceVelocities.desired_surge = yTpik_[0];
    referenceVelocities.desired_yaw_rate = yTpik_[5];

    referenceVelocitiesPub_->publish(referenceVelocities);

    ulisse_msgs::msg::FeedbackGui feedbackGuiMsg;
    feedbackGuiMsg.stamp.sec = now_stamp_secs;
    feedbackGuiMsg.stamp.nanosec = now_stamp_nanosecs;
    feedbackGuiMsg.goal_position.latitude = stateLatLong_->goalPosition.latitude;
    feedbackGuiMsg.goal_position.longitude = stateLatLong_->goalPosition.longitude;
    feedbackGuiMsg.goal_heading = stateLatLong_->goalHeading;
    feedbackGuiMsg.acceptance_radius = stateLatLong_->acceptanceRadius;
    feedbackGuiMsg.goal_distance = stateLatLong_->goalDistance;

    feedbackGuiPub_->publish(feedbackGuiMsg);

    ulisse_msgs::msg::VehicleStatus vehicleStatusMsg;
    vehicleStatusMsg.stamp.sec = now_stamp_secs;
    vehicleStatusMsg.stamp.nanosec = now_stamp_nanosecs;
    vehicleStatusMsg.vehicle_state = uFsm_.GetCurrentStateName();

    vehicleStatusPub_->publish(vehicleStatusMsg);

    for (auto& taskMap : tasksMap_) {
        try {

            std::vector<double> diagonal_internal_activation_function;
            for (unsigned int i = 0; i < taskMap.second.task->InternalActivationFunction().rows(); i++) {
                diagonal_internal_activation_function.push_back(taskMap.second.task->InternalActivationFunction().at(i, i));
            }

            std::vector<double> diagonal_external_activation_function;
            for (unsigned int i = 0; i < taskMap.second.task->InternalActivationFunction().rows(); i++) {
                diagonal_external_activation_function.push_back(taskMap.second.task->InternalActivationFunction().at(i, i));
            }

            std::vector<double> referenceRate;
            for (unsigned int i = 0; i < taskMap.second.task->ReferenceRate().size(); i++) {
                referenceRate.push_back(taskMap.second.task->ReferenceRate().at(i));
            }

            ulisse_msgs::msg::TaskStatus taskstatus_msg;

            taskstatus_msg.stamp.sec = now_stamp_secs;
            taskstatus_msg.stamp.nanosec = now_stamp_nanosecs;
            taskstatus_msg.id = taskMap.second.task->ID();
            taskstatus_msg.is_active = taskMap.second.task->IsActive();
            taskstatus_msg.external_activation_function = diagonal_external_activation_function;
            taskstatus_msg.internal_activation_function = diagonal_internal_activation_function;
            taskstatus_msg.reference_rate = referenceRate;

            tasksMap_[taskMap.second.task->ID()].taskPub->publish(taskstatus_msg);

        } catch (tpik::ExceptionWithHow& e) {
            std::cerr << "LOG TASK EXCEPTION" << std::endl;
            std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
        }
    }
}
} // namespace ulisse
