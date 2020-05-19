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
    ctrlCxt_ = std::make_shared<ControlContext>();
    goalCxt_ = std::make_shared<GoalContext>();
    statusCxt_ = std::make_shared<StatusContext>();
    conf_ = std::make_shared<ControllerConfiguration>();
    fileName_ = file_name;

    // Sensor Subscriptions
    gpsSub_ = nh_->create_subscription<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data, 10, std::bind(&VehicleController::GPSSensorCB, this, _1));
    //compass_sub_ = nh_->create_subscription<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass, std::bind(&VehicleController::CompassSensorCB, this, _1));
    navFilterSub_ = nh_->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10, std::bind(&VehicleController::NavFilterCB, this, _1));

    // Control Publishers
    ctrlcxtPub_ = nh_->create_publisher<ulisse_msgs::msg::ControlContext>(ulisse_msgs::topicnames::control_context, 10);
    goalcxtPub_ = nh_->create_publisher<ulisse_msgs::msg::GoalContext>(ulisse_msgs::topicnames::goal_context, 10);
    statuscxtPub_ = nh_->create_publisher<ulisse_msgs::msg::StatusContext>(ulisse_msgs::topicnames::status_context, 10);
    genericLogPub_ = nh_->create_publisher<std_msgs::msg::String>("/ulisse/log/generic", 10);

    /// TPIK Manager
    actionManager_ = std::make_shared<tpik::ActionManager>(tpik::ActionManager());

    /// ROBOT MODEL
    Eigen::TransfMatrix world_T_vehicle;

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

        goalCxt_->goalSurge = request->cruise_control;

        Eigen::VectorXd satMin, satMax;
        iCat_->GetSaturation(satMin, satMax);

        satMax.at(3) = request->cruise_control;

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

std::shared_ptr<ControlContext> VehicleController::CtrlContext() const
{
    return ctrlCxt_;
}

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

    commandHold_.SetFSM(&uFsm_);

    commandLatLong_.SetFSM(&uFsm_);
    commandLatLong_.SetGoalCtx(goalCxt_);

    commandSpeedHeading_.SetFSM(&uFsm_);
    commandSpeedHeading_.SetGoalCtx(goalCxt_);

    commandPathFollowing_.SetFSM(&uFsm_);

    // ***** STATES *****
    ulisse::states::GenericState::StateCtx stateCtx;
    stateCtx.actionManager = actionManager_;
    stateCtx.robotModel = robotModel_;
    stateCtx.ctrlCxt = ctrlCxt_;
    stateCtx.statusCxt = statusCxt_;
    stateCtx.goalCxt = goalCxt_;
    stateCtx.tasksMap = tasksMap_;

    //Set the fms and the structure that the states need.
    for (auto& state : statesMap_) {
        state.second.SetStateCtx(stateCtx);
        state.second.SetFSM(&uFsm_);
    }

    // ***** EVENTS *****
    eventRcEnabled_.SetFSM(&uFsm_);
    eventRcEnabled_.SetCtrlContext(ctrlCxt_);

    // ***** CONFIGURE FSM *****
    // ADD COMMANDS
    for (auto& command : commandsMap_) {
        uFsm_.AddCommand(command.first, &command.second);
    }

    // ADD STATES
    for (auto& state : statesMap_) {
        uFsm_.AddState(state.first, &state.second);
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
            commandLatLong_.SetGoTo(request->latlong_cmd.goal.latitude, request->latlong_cmd.goal.longitude, request->latlong_cmd.acceptance_radius);

            log << "Received Command GoTo (lat: " << request->latlong_cmd.goal.latitude << " , long: " << request->latlong_cmd.goal.longitude << " )";
            publishLog(log.str().c_str());

        } else if (request->command_type == ulisse::commands::ID::speedheading) {

            std::cout << "Received Command SpeedHeading" << std::endl;
            commandSpeedHeading_.SetSpeedHeading(request->sh_cmd.speed, request->sh_cmd.heading, request->sh_cmd.timeout.sec);
            stateSpeedHeading_.ResetTimer();
            log << "Received Command SpeedHeading (speed: " << request->sh_cmd.speed << " , heading: " << request->sh_cmd.heading << " )";
            publishLog(log.str().c_str());

        } else if (request->command_type == ulisse::commands::ID::navigate) {

            std::cout << "Received Command Path Following" << std::endl;

            if (!statePathFollowing_.LoadNurbs(request->nav_cmd.path)) {
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

void VehicleController::GPSSensorCB(const ulisse_msgs::msg::GPSData::SharedPtr msg)
{
    timestamp_ = msg->time;
    statusCxt_->gpsSpeed = msg->speed;
    statusCxt_->gpsTrack = msg->track * M_PI / 180.0;
}

void VehicleController::NavFilterCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg)
{
    statusCxt_->vehiclePos.latitude = msg->latitude;
    statusCxt_->vehiclePos.longitude = msg->longitude;
    statusCxt_->vehicleHeading = msg->orientation.yaw;

    statusCxt_->seacurrent[0] = msg->current[0];
    statusCxt_->seacurrent[1] = msg->current[1];

    // Linear position in world frame
    Eigen::Vector3d w_position(statusCxt_->vehiclePos.latitude, statusCxt_->vehiclePos.longitude, 0);

    // Angualr position in world frame
    rml::EulerRPY rpy;
    rpy.SetRoll(0);
    rpy.SetPitch(0);
    rpy.SetYaw(msg->orientation.yaw);

    Eigen::Vector6d velocity_fbk;
    velocity_fbk.setZero();
    velocity_fbk(0) = msg->speed[0];
    velocity_fbk(1) = msg->speed[1];
    velocity_fbk(5) = msg->speed[5];

    // Updating the robot model
    Eigen::TransfMatrix auv_position_transf;
    auv_position_transf.SetTransl(w_position);
    auv_position_transf.SetRotMatrix(rpy.ToRotMatrix());
    robotModel_->SetBodyFramePosition(auv_position_transf);
    robotModel_->SetVelocityVector(ulisse::robotModelID::ASV, velocity_fbk);
}

void VehicleController::LLCStatusCB(const ulisse_msgs::msg::LLCStatus::SharedPtr msg)
{
    statusCxt_->llcStatus = msg->status;
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

    ctrlCxt_->desiredSurge = yTpik_[3];
    ctrlCxt_->desiredJog = yTpik_[2];

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

    for (auto& taskMap : tasksMap_) {
        try {

            std::vector<double> diagonal_activation_function;
            for (unsigned int i = 0; i < taskMap.second.task->InternalActivationFunction().rows(); i++) {
                diagonal_activation_function.push_back(taskMap.second.task->InternalActivationFunction().at(i, i));
            }
            std::vector<double> referenceRate;
            for (unsigned int i = 0; i < taskMap.second.task->ReferenceRate().size(); i++) {
                referenceRate.push_back(taskMap.second.task->ReferenceRate().at(i));
            }

            ulisse_msgs::msg::TaskStatus taskstatus_msg;

            tNow_ = std::chrono::system_clock::now();
            long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow_.time_since_epoch())).count();
            auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
            auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

            taskstatus_msg.stamp.sec = now_stamp_secs;
            taskstatus_msg.stamp.nanosec = now_stamp_nanosecs;
            taskstatus_msg.id = taskMap.second.task->ID();
            taskstatus_msg.is_active = taskMap.second.task->IsActive();
            taskstatus_msg.activation_function = diagonal_activation_function;
            taskstatus_msg.reference = referenceRate;

            tasksMap_[taskMap.second.task->ID()].taskPub->publish(taskstatus_msg);

        } catch (tpik::ExceptionWithHow& e) {
            std::cerr << "LOG TASK EXCEPTION" << std::endl;
            std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
        }
    }
}

void VehicleController::PublishControl()
{
    ulisse_msgs::msg::StatusContext statuscxt_msg;

    tNow_ = std::chrono::system_clock::now();
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow_.time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

    statuscxt_msg.stamp.sec = now_stamp_secs;
    statuscxt_msg.stamp.nanosec = now_stamp_nanosecs;
    statuscxt_msg.vehicle_pos.latitude = statusCxt_->vehiclePos.latitude;
    statuscxt_msg.vehicle_pos.longitude = statusCxt_->vehiclePos.longitude;
    statuscxt_msg.vehicle_heading = statusCxt_->vehicleHeading;
    statuscxt_msg.vehicle_speed = statusCxt_->gpsSpeed;
    statuscxt_msg.vehicle_track = statusCxt_->gpsTrack;
    statuscxt_msg.vehicle_state = uFsm_.GetCurrentStateName();
    statuscxtPub_->publish(statuscxt_msg);

    ulisse_msgs::msg::GoalContext goalcxt_msg;
    goalcxt_msg.stamp.sec = now_stamp_secs;
    goalcxt_msg.stamp.nanosec = now_stamp_nanosecs;
    goalcxt_msg.current_goal.latitude = goalCxt_->currentGoal.pos.latitude;
    goalcxt_msg.current_goal.longitude = goalCxt_->currentGoal.pos.longitude;
    goalcxt_msg.accept_radius = goalCxt_->currentGoal.acceptRadius;
    goalcxt_msg.goal_distance = goalCxt_->goalDistance;
    goalcxt_msg.goal_heading = goalCxt_->goalHeading;
    goalcxt_msg.goal_speed = goalCxt_->goalSurge;
    goalcxtPub_->publish(goalcxt_msg);

    ulisse_msgs::msg::ControlContext ctrlcxt_msg;
    ctrlcxt_msg.stamp.sec = now_stamp_secs;
    ctrlcxt_msg.stamp.nanosec = now_stamp_nanosecs;

    ctrlcxt_msg.desired_speed = ctrlCxt_->desiredSurge;
    ctrlcxt_msg.desired_jog = ctrlCxt_->desiredJog;

    ctrlcxtPub_->publish(ctrlcxt_msg);
}
} // namespace ulisse
