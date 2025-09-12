#include "ulisse_ctrl/kinematic_vehicle_controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <jsoncpp/json/json.h>

#include "ulisse_ctrl/configuration.hpp"
#include "ulisse_ctrl/states/generic_state.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"

#include "ulisse_msgs/futils.hpp"
#include "ulisse_msgs/topicnames.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace ulisse {

VehicleController::VehicleController(std::string conf_filename)
    : Node("kinematic_control_node")
    , boundariesSet_(false)
{
    conf_ = std::make_shared<KCLConfiguration>();

    ctrlData_ = std::make_shared<ControlData>();
    //ctrlDataReal_ = std::make_shared<ControlData>(); // no need for now
    real_position_ = std::make_shared<LatLong>();

    fileName_ = conf_filename;

    stateHalt_ = std::make_shared<states::StateHalt>();
    stateHold_ = std::make_shared<states::StateHold>();
    statePathFollowing_ = std::make_shared<states::StatePathFollow>();
    statePathFollowingILOS_ = std::make_shared<states::StatePathFollowILOS>(); //ILOS
    statePathFollowingCurrent_ = std::make_shared<states::StatePathFollowCurrent>(); //Current
    statePathFollowingILOSCurrent_ = std::make_shared<states::StatePathFollowILOSCurrent>(); //ILOSCurrent
    statePathFollowingALOS_ = std::make_shared<states::StatePathFollowALOS>(); //ALOS
    stateLatLong_ = std::make_shared<states::StateLatLong>();
    stateSurgeHeading_ = std::make_shared<states::StateSurgeHeading>();
    stateSurgeYawRate_ = std::make_shared<states::StateSurgeYawRate>();

    // Sensor Subscriptions
    navFilterSub_ = this->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10, std::bind(&VehicleController::NavFilterCB, this, _1));
    llcStatusSub_ = this->create_subscription<ulisse_msgs::msg::LLCStatus>(ulisse_msgs::topicnames::llc_status, 10, std::bind(&VehicleController::LLCStatusCB, this, _1));

    // Data Subscriptions
    surgeHeadingSub_ = this->create_subscription<ulisse_msgs::msg::SurgeHeading>(ulisse_msgs::topicnames::surge_heading, 10, std::bind(&VehicleController::SurgeHeadingCB, this, _1));
    surgeYawRateSub_ = this->create_subscription<ulisse_msgs::msg::SurgeYawRate>(ulisse_msgs::topicnames::surge_yawrate, 10, std::bind(&VehicleController::SurgeYawRateCB, this, _1));


    simulatedSystemSub_ = this->create_subscription<ulisse_msgs::msg::SimulatedSystem>(ulisse_msgs::topicnames::simulated_system,
        10, std::bind(&VehicleController::GroundTruthDataCB, this, _1));

    // Control Publishers

    genericLogPub_ = this->create_publisher<std_msgs::msg::String>("/ulisse/log/generic", 10);
    vehicleStatusPub_ = this->create_publisher<ulisse_msgs::msg::VehicleStatus>(ulisse_msgs::topicnames::vehicle_status, 10);
    referenceVelocitiesPub_ = this->create_publisher<ulisse_msgs::msg::ReferenceVelocities>(ulisse_msgs::topicnames::reference_velocities, 10);
    feedbackGuiPub_ = this->create_publisher<ulisse_msgs::msg::FeedbackGui>(ulisse_msgs::topicnames::feedback_gui, 10);
    tpikActionPub_ = this->create_publisher<ulisse_msgs::msg::TPIKAction>(ulisse_msgs::topicnames::tpik_action, 10);
    pathFollowPub_ = this->create_publisher<ulisse_msgs::msg::PathFollow>(ulisse_msgs::topicnames::pathfollowing, 10);

    safetyBoundarySetPub_ = this->create_publisher<std_msgs::msg::Bool>(ulisse_msgs::topicnames::safety_boundary_set, 10);

    /// TPIK Manager
    actionManager_ = std::make_shared<tpik::ActionManager>(tpik::ActionManager());

    /// ROBOT MODEL
    Eigen::TransformationMatrix world_T_vehicle;

    /// Jacobian
    Eigen::Matrix6d J_ASV;
    J_ASV.setIdentity();

    // Robot Model
    robotModel_ = std::make_shared<rml::RobotModel>(world_T_vehicle, ulisse::robotModelID::ASV, J_ASV);

    // ***** SETUP TASKS ***** //

    // ASV CONTROL VELOCITY LINEAR
    asvLinearVelocity_ = std::make_shared<ikcl::LinearVelocity>(ikcl::LinearVelocity(ulisse::task::asvLinearVelocity, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvLinearVelocity_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_linear_velocity, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvLinearVelocity, taskInfo_));

    // AUV CONTROL ANGULAR POSITION
    asvAngularPosition_ = std::make_shared<ikcl::AlignToTarget>(ikcl::AlignToTarget(ulisse::task::asvAngularPosition, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvAngularPosition_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_angular_position, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvAngularPosition, taskInfo_));

    // ASV CONTROL DISTANCE
    asvCartesianDistance_ = std::make_shared<ikcl::CartesianDistance>(ikcl::CartesianDistance(ulisse::task::asvCartesianDistance, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvCartesianDistance_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_cartesian_distance, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvCartesianDistance, taskInfo_));

    // ASV CONTROL DISTANCE PATH FOLLOWING
    asvCartesianDistancePathFollowing_ = std::make_shared<ikcl::CartesianDistance>(ikcl::CartesianDistance(ulisse::task::asvCartesianDistancePathFollowing, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvCartesianDistancePathFollowing_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_cartesian_distance_path_follow, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvCartesianDistancePathFollowing, taskInfo_));

    // ASV SAFETY BOUNDARIES (INEQUALITY TASK)
    asvSafetyBoundaries_ = std::make_shared<ikcl::SafetyBoundaries>(ikcl::SafetyBoundaries(ulisse::task::asvSafetyBoundaries, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvSafetyBoundaries_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_safety_boundaries, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvSafetyBoundaries, taskInfo_));

    // ASV absolute axis alignment task
    asvAbsoluteAxisAlignment_ = std::make_shared<ikcl::AbsoluteAxisAlignment>(ikcl::AbsoluteAxisAlignment(ulisse::task::asvAbsoluteAxisAlignment, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvAbsoluteAxisAlignment_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvAbsoluteAxisAlignment, taskInfo_));

    // ASV absolute axis alignment task
    asvAbsoluteAxisAlignmentSafety_ = std::make_shared<ikcl::AbsoluteAxisAlignment>(ikcl::AbsoluteAxisAlignment(ulisse::task::asvAbsoluteAxisAlignmentSafety, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvAbsoluteAxisAlignmentSafety_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment_safety, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvAbsoluteAxisAlignmentSafety, taskInfo_));

    // ASV absolute axis alignment task hold
    asvAbsoluteAxisAlignmentHold_ = std::make_shared<ikcl::AbsoluteAxisAlignment>(ikcl::AbsoluteAxisAlignment(ulisse::task::asvAbsoluteAxisAlignmentHold, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvAbsoluteAxisAlignmentHold_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment_hold, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvAbsoluteAxisAlignmentHold, taskInfo_));

    // ASV CONTROL VELOCITY LINEAR HOLD
    asvLinearVelocityHold_ = std::make_shared<ikcl::LinearVelocity>(ikcl::LinearVelocity(ulisse::task::asvLinearVelocityHold, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvLinearVelocityHold_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_linear_velocity_hold, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvLinearVelocityHold, taskInfo_));


    // AUV CONTROL ANGULAR POSITION ILOS
    //asvAngularPositionILOS_ = std::make_shared<ikcl::AlignToTarget>(ikcl::AlignToTarget(ulisse::task::asvAngularPositionILOS, robotModel_, ulisse::robotModelID::ASV));
    //taskInfo_.task = asvAngularPositionILOS_;
    //taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_angular_position_ilos, 1);
    //tasksMap_.insert(std::make_pair(ulisse::task::asvAngularPositionILOS, taskInfo_));

    // ASV absolute axis alignment task
    asvAbsoluteAxisAlignmentILOS_ = std::make_shared<ikcl::AbsoluteAxisAlignment>(ikcl::AbsoluteAxisAlignment(ulisse::task::asvAbsoluteAxisAlignmentILOS, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvAbsoluteAxisAlignmentILOS_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment_ilos, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvAbsoluteAxisAlignmentILOS, taskInfo_));

    // ASV CONTROL VELOCITY LINEAR
    asvLinearVelocityCurrentEst_ = std::make_shared<ikcl::LinearVelocity>(ikcl::LinearVelocity(ulisse::task::asvLinearVelocityCurrentEst, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvLinearVelocityCurrentEst_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_linear_velocity_current, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvLinearVelocityCurrentEst, taskInfo_));

    // ASV absolute axis alignment task
    asvAbsoluteAxisAlignmentCurrentEst_ = std::make_shared<ikcl::AbsoluteAxisAlignment>(ikcl::AbsoluteAxisAlignment(ulisse::task::asvAbsoluteAxisAlignmentCurrentEst, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvAbsoluteAxisAlignmentCurrentEst_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment_current, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvAbsoluteAxisAlignmentCurrentEst, taskInfo_));

    // ASV absolute axis alignment task
    asvAbsoluteAxisAlignmentALOS_ = std::make_shared<ikcl::AbsoluteAxisAlignment>(ikcl::AbsoluteAxisAlignment(ulisse::task::asvAbsoluteAxisAlignmentALOS, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvAbsoluteAxisAlignmentALOS_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment_alos, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvAbsoluteAxisAlignmentALOS, taskInfo_));

    // Initialize solver_ and iCAT
    int dof = 6;
    iCat_ = std::make_shared<tpik::iCAT>(tpik::iCAT(dof));
    yTpik_ = Eigen::VectorXd::Zero(dof);

    // load config file
    // Setup Params for Tasks and iCAT
    if (!LoadConfiguration(conf_)) {
        std::cerr << "Failed to load KCL configuration from file" << std::endl;
        return;
    }

    // solver_ definition
    solver_ = std::make_shared<tpik::Solver>(tpik::Solver(actionManager_, iCat_));

    // FSM Initialization
    SetUpFSM();

    srvCommand_ = this->create_service<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service,
        std::bind(&VehicleController::CommandsHandler, this, _1, _2, _3));

    srvSetBoundaries_ = this->create_service<ulisse_msgs::srv::SetBoundaries>(ulisse_msgs::topicnames::set_boundaries_service,
        std::bind(&VehicleController::SetBoundariesHandler, this, _1, _2, _3));

    srvGetBoundaries_ = this->create_service<ulisse_msgs::srv::GetBoundaries>(ulisse_msgs::topicnames::get_boundaries_service,
        std::bind(&VehicleController::GetBoundariesHandler, this, _1, _2, _3));

    srvResetConf_ = this->create_service<ulisse_msgs::srv::ResetConfiguration>(ulisse_msgs::topicnames::reset_kcl_conf_service,
        std::bind(&VehicleController::ResetConfHandler, this, _1, _2, _3));

    // Main function timer
    int msRunPeriod = 1.0 / (conf_->controlLoopRate) * 1000;
    //std::cout << "Controller Rate: " << conf_->controlLoopRate << "Hz" << std::endl;
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&VehicleController::Run, this));

    // Timer for slow check operations
    slow_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&VehicleController::SlowTimerCB, this));
}

VehicleController::~VehicleController() { }

bool VehicleController::LoadConfiguration(std::shared_ptr<KCLConfiguration>& conf)
{
    libconfig::Config confObj;

    ///////////////////////////////////////////////////////////////////////////////
    /////       LOAD CONFIGURATION FROM NAV FILTER TO READ CENTROID
    ///
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nav_filter");
    std::string confPath = package_share_directory;
    confPath.append("/conf/navigation_filter.conf");

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

    Eigen::VectorXd centroidLocationTmp;
    if (!ctb::GetParamVector(confObj, centroidLocationTmp, "centroidLocation")) {
        std::cerr << "Failed to load centroidLocation from file" << std::endl;
        return false;
    };

    centroidLocation_.latitude = centroidLocationTmp[0];
    centroidLocation_.longitude = centroidLocationTmp[1];

    std::cout << "Centroid: " << centroidLocation_.latitude << ", " << centroidLocation_.longitude << std::endl;

    asvSafetyBoundaries_->Centroid() = centroidLocation_;

    ///////////////////////////////////////////////////////////////////////////
    /////        LOAD KCL CONFIGURATION
    ///
    package_share_directory = ament_index_cpp::get_package_share_directory("ulisse_ctrl");
    confPath = package_share_directory;
    confPath.append("/conf/");
    confPath.append(fileName_);

    std::cout << "PATH TO KCL CONF FILE : " << confPath << std::endl;

    // Read the configuration file. If there is an error, report it and exit.
    try {
        confObj.readFile(confPath.c_str());
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        return -1;
    } catch (const libconfig::ParseException& pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
        return -1;
    }

    if (!conf->ConfigureFromFile(confObj)) {
        std::cerr << "Failed to load KCL configuration" << std::endl;
        return false;
    }
    // Set Saturation values for the iCAT (read from conf file)
    iCat_->SetSaturation(conf->saturationMin, conf->saturationMax);

    std::cout << tc::brown << *conf << tc::none << std::endl;

    if (!ConfigureTasksFromFile(tasksMap_, confObj)) {
        std::cerr << "Failed to load Tasks from file" << std::endl;
        return false;
    };
    if (!ConfigurePriorityLevelsFromFile(actionManager_, tasksMap_, confObj)) {
        std::cerr << "Failed to load Priority Levels from file" << std::endl;
        return false;
    };

    if (!ConfigureActionsFromFile(actionManager_, confObj)) {
        std::cerr << "Failed to load  Actions from file" << std::endl;
        return false;
    };
    // conf->pathFollowMode //////// ILOS or LOS

    //insert states in the map
    statesMap_.insert({ ulisse::states::ID::halt, stateHalt_ });
    statesMap_.insert({ ulisse::states::ID::hold, stateHold_ });
    statesMap_.insert({ ulisse::states::ID::latlong, stateLatLong_ });
    statesMap_.insert({ ulisse::states::ID::pathfollow, statePathFollowing_ });
    statesMap_.insert({ ulisse::states::ID::pathfollow_ilos, statePathFollowingILOS_ });
    statesMap_.insert({ ulisse::states::ID::pathfollow_current, statePathFollowingCurrent_ });
    statesMap_.insert({ ulisse::states::ID::pathfollow_iloscurrent, statePathFollowingILOSCurrent_ });
    statesMap_.insert({ ulisse::states::ID::pathfollow_alos, statePathFollowingALOS_ });
    statesMap_.insert({ ulisse::states::ID::surgeheading, stateSurgeHeading_ });
    statesMap_.insert({ ulisse::states::ID::surgeyawrate, stateSurgeYawRate_ });

    if (!ConfigureSatesFromFile(statesMap_, confObj)) {
        std::cerr << "Failed to load States from file" << std::endl;
        return false;
    };

    //insert command in the map
    commandsMap_.insert({ ulisse::commands::ID::halt, commandHalt_ });
    commandsMap_.insert({ ulisse::commands::ID::hold, commandHold_ });
    commandsMap_.insert({ ulisse::commands::ID::latlong, commandLatLong_ });
    commandsMap_.insert({ ulisse::commands::ID::pathfollow, commandPathFollowing_ });
    commandsMap_.insert({ ulisse::commands::ID::pathfollow_ilos, commandPathFollowingILOS_ });
    commandsMap_.insert({ ulisse::commands::ID::pathfollow_current, commandPathFollowingCurrent_ });
    commandsMap_.insert({ ulisse::commands::ID::pathfollow_iloscurrent, commandPathFollowingILOSCurrent_ });
    commandsMap_.insert({ ulisse::commands::ID::pathfollow_alos, commandPathFollowingALOS_ });
    commandsMap_.insert({ ulisse::commands::ID::surgeheading, commandSurgeHeading_ });
    commandsMap_.insert({ ulisse::commands::ID::surgeyawrate, commandSurgeYawRate_ });

    return true;
}

void VehicleController::PublishLog(std::string log)
{
    std_msgs::msg::String genericLogPub_msg;
    genericLogPub_msg.data = log;
    genericLogPub_->publish(genericLogPub_msg);
}

void VehicleController::SetUpFSM()
{

    // ***** COMMANDS ***** //
    commandHalt_.SetFSM(&uFsm_);
    commandHalt_.SetState(stateHalt_);

    commandHold_.SetFSM(&uFsm_);
    commandHold_.SetState(stateHold_);

    commandLatLong_.SetFSM(&uFsm_);
    commandLatLong_.SetState(stateLatLong_);

    commandSurgeHeading_.SetFSM(&uFsm_);
    commandSurgeHeading_.SetState(stateSurgeHeading_);

    commandSurgeYawRate_.SetFSM(&uFsm_);
    commandSurgeYawRate_.SetState(stateSurgeYawRate_);

    commandPathFollowing_.SetFSM(&uFsm_);
    commandPathFollowing_.SetState(statePathFollowing_);

    commandPathFollowingILOS_.SetFSM(&uFsm_);
    commandPathFollowingILOS_.SetState(statePathFollowingILOS_); //ILOS

    commandPathFollowingCurrent_.SetFSM(&uFsm_);
    commandPathFollowingCurrent_.SetState(statePathFollowingCurrent_); //Current

    commandPathFollowingILOSCurrent_.SetFSM(&uFsm_); // ILOS Current
    commandPathFollowingILOSCurrent_.SetState(statePathFollowingILOSCurrent_);

    commandPathFollowingALOS_.SetFSM(&uFsm_);
    commandPathFollowingALOS_.SetState(statePathFollowingALOS_); //ALOS

    // ***** STATES ***** //
    //Set the fsm and the structure that the states need.
    for (auto& state : statesMap_) {
        state.second->actionManager = actionManager_;
        state.second->robotModel = robotModel_;
        state.second->tasksMap = tasksMap_;
        state.second->ctrlData = ctrlData_;
        // TODO: CHECK UNNECESSARY VARIABLES !!!
        state.second->real_position = real_position_; // ILOS
        //state.second->ctrlDataReal = ctrlDataReal_; // ILOS
        state.second->SetFSM(&uFsm_);
    }

    // ***** EVENTS ***** //
    eventRcEnabled_.SetFSM(&uFsm_);
    eventNearGoalPosition_.SetFSM(&uFsm_);
    eventNearGoalPosition_.ControlData() = ctrlData_;
    eventNearGoalPosition_.GoToHoldAfterMove(conf_->goToHoldAfterMove);
    eventNearGoalPosition_.StateHold() = std::dynamic_pointer_cast<ulisse::states::StateHold>(statesMap_.find(ulisse::states::ID::hold)->second);

    // ***** FSM CONFIGURATION ***** //
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
    uFsm_.AddEvent(ulisse::events::names::neargoalposition, &eventNearGoalPosition_);

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

bool VehicleController::IsTaskInCurrentAction(const std::string task_id)
{

    try {
        tpik::Hierarchy hierarchy = actionManager_->GetAction(actionManager_->CurrentActionID())->PriorityLevels();

        for (auto& priorityLevel : hierarchy) {
            std::vector<std::shared_ptr<tpik::Task>> tasks = priorityLevel->Level();
            for (auto& task : tasks) {
                if (task->ID() == task_id)
                    return true;
            }
        }
    } catch (tpik::ActionManagerException& e) {
        std::cerr << e.how() << std::endl;
    }

    return false;
}

void VehicleController::CommandsHandler(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ulisse_msgs::srv::ControlCommand::Request> request,
    std::shared_ptr<ulisse_msgs::srv::ControlCommand::Response> response)
{
    // Create a callback function for when service requests are received.

    (void)request_header;
    RCLCPP_INFO(this->get_logger(), "Incoming request: %s", request->command_type.c_str());

    std::stringstream logg;
    logg << "Incoming request: " << request->command_type.c_str();
    PublishLog(logg.str().c_str());
    fsm::retval ret = fsm::ok;

    // Check if Boundaries are set before accepting commands
    if (!boundariesSet_) {
        response->res = "[KCL] No SafetyBound Set!";
    } else {

        std::stringstream log;
        if (request->command_type == ulisse::commands::ID::halt) {
            std::cout << "Received Command Halt" << std::endl;
            PublishLog("Received Command Halt");

        } else if (request->command_type == ulisse::commands::ID::hold) {
            commandHold_.SetPositionToHold(ctrlData_->inertialF_linearPosition);
            std::cout << "Received Command Hold" << std::endl;
            PublishLog("Received Command Hold");

        } else if (request->command_type == ulisse::commands::ID::latlong) {

            std::cout << "Received Command LatLong" << std::endl;
            if(!commandLatLong_.SetGoTo(LatLong(request->latlong_cmd.goal.latitude, request->latlong_cmd.goal.longitude),
                    request->latlong_cmd.acceptance_radius)){
                response->res = "CommandAnswer::fail - Malformed LatLong Message.";
                ret = fsm::retval::fail;
            }

            log << "Received Command GoTo (lat: " << request->latlong_cmd.goal.latitude << " , long: " << request->latlong_cmd.goal.longitude << " )";
            PublishLog(log.str().c_str());

        } else if (request->command_type == ulisse::commands::ID::surgeheading) {

            std::cout << "Received Command surgeheading" << std::endl;
            commandSurgeHeading_.SetTimeout(request->sh_cmd.timeout.sec);
            stateSurgeHeading_->ResetTimer();
            log << "Received Command surgeheading (data read from topic)";
            PublishLog(log.str().c_str());

        } else if (request->command_type == ulisse::commands::ID::surgeyawrate) {

            std::cout << "Received Command surgeyawrate" << std::endl;
            commandSurgeYawRate_.SetTimeout(request->sh_cmd.timeout.sec);
            stateSurgeYawRate_->ResetTimer();
            log << "Received Command surgeyawrate (data read from topic)";
            PublishLog(log.str().c_str());

        } else if (request->command_type == ulisse::commands::ID::pathfollow) {

            std::cout << "Received Command Path Following" << std::endl;

            if (!statePathFollowing_->LoadPath(request->path_cmd.path, request->path_cmd.loop)) {
                response->res = "CommandAnswer::fail - Malformed Path Message.";
                ret = fsm::retval::fail;
            }

            log << "Received Command PathFollowing LOS (id: " << request->path_cmd.path.id << " )";
            PublishLog(log.str().c_str());

        } else if (request->command_type == ulisse::commands::ID::pathfollow_ilos) {

            std::cout << "Received Command Path Following ILOS" << std::endl;

            if (!statePathFollowingILOS_->LoadPath(request->path_cmd.path)) {
                response->res = "CommandAnswer::fail - Malformed Path Message.";
                ret = fsm::retval::fail;
            }

            log << "Received Command PathFollowing ILOS (id: " << request->path_cmd.path.id << " )";
            PublishLog(log.str().c_str());

        } else if (request->command_type == ulisse::commands::ID::pathfollow_current) {

            std::cout << "Received Command Path Following LOS with current estimator" << std::endl;

            if (!statePathFollowingCurrent_->LoadPath(request->path_cmd.path)) {
                response->res = "CommandAnswer::fail - Malformed Path Message.";
                ret = fsm::retval::fail;
            }

            log << "Received Command PathFollowing LOS with current estimator (id: " << request->path_cmd.path.id << " )";
            PublishLog(log.str().c_str());

        } else if (request->command_type == ulisse::commands::ID::pathfollow_iloscurrent) {

            std::cout << "Received Command Path Following ILOS and LOS with current estimator" << std::endl;

            if (!statePathFollowingILOSCurrent_->LoadPath(request->path_cmd.path)) {
                response->res = "CommandAnswer::fail - Malformed Path Message.";
                ret = fsm::retval::fail;
            }

            log << "Received Command PathFollowing ILOS and LOS with current estimator(id: " << request->path_cmd.path.id << " )";
            PublishLog(log.str().c_str());

        } else if (request->command_type == ulisse::commands::ID::pathfollow_alos) {

            std::cout << "Received Command Path Following ALOS" << std::endl;

            if (!statePathFollowingALOS_->LoadPath(request->path_cmd.path)) {
                response->res = "CommandAnswer::fail - Malformed Path Message.";
                ret = fsm::retval::fail;
            }

            log << "Received Command PathFollowing ALOS (id: " << request->path_cmd.path.id << " )";
            PublishLog(log.str().c_str());

        } else {
            response->res = "CommandAnswer::fail - Unsupported command: " + request->command_type;
            ret = fsm::retval::fail;
        }

        if (ret == fsm::retval::ok) {
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
            response->res = "[KCL] CommandAnswer::ok";
        }
    }
    RCLCPP_INFO(this->get_logger(), "Service Response: %s", response->res.c_str());
}

void VehicleController::SetBoundariesHandler(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ulisse_msgs::srv::SetBoundaries::Request> request,
    std::shared_ptr<ulisse_msgs::srv::SetBoundaries::Response> response)
{
    (void)request_header;
    RCLCPP_INFO(this->get_logger(), "Incoming request for set boundaries");

    if (asvSafetyBoundaries_->InitializePolygon(request->boundaries)) {
        boundariesJson_ = request->boundaries.id;
        // TODO: The string "info_string" with all the vertices is now the "id"
        // When migrating to sisl_toolbox the points will be in "vertices" as a vector of LatLong
        response->res = "SetBound::ok";
        boundariesSet_ = true;
    } else {
        response->res = "[KCL] SetBound::error";
    }

    std::stringstream log;
    log << "Setting Bounding Box: " << request->boundaries.id;
    PublishLog(log.str().c_str());
}

void VehicleController::GetBoundariesHandler(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ulisse_msgs::srv::GetBoundaries::Request> request,
    std::shared_ptr<ulisse_msgs::srv::GetBoundaries::Response> response)
{
    (void)request_header;
    (void)request;
    RCLCPP_INFO(this->get_logger(), "Incoming request for get boundaries");

    if (boundariesSet_) {
        response->res = boundariesJson_;
    } else {
        response->res = "[KCL] NoBoundSet";
    }
}

void VehicleController::ResetConfHandler(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Request> request,
    std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Response> response)
{
    (void)request_header;
    (void)request;
    RCLCPP_INFO(this->get_logger(), "Incoming request for reset conf");

    auto previousConf = conf_;
    PublishLog("Configuration Reset:");
    if (!LoadConfiguration(conf_)) {
        LoadConfiguration(previousConf);
        std::cerr << "Failed to reload KCL configuration from file. Load the previous configuration" << std::endl;
    }

    response->res = "[KCL] ReloadConfiguration::ok";
}

void VehicleController::SlowTimerCB()
{

    /*if (!boundariesSet_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for the Safety Bounding Box");
    }
    return;*/
    std_msgs::msg::Bool sbSet;
    sbSet.data = boundariesSet_;
    safetyBoundarySetPub_->publish(sbSet);

    // Publish Hierarchy Info
    ulisse_msgs::msg::TPIKAction tpikActionMsg_;
    tpikActionMsg_.id = actionManager_->CurrentActionID();
    tpik::Hierarchy hierarchy = actionManager_->GetAction(tpikActionMsg_.id)->PriorityLevels();

    for (size_t i = 0; i < hierarchy.size(); i++) {

        tpikActionMsg_.priority_levels.push_back(ulisse_msgs::msg::TPIKPriorityLevel());
        tpikActionMsg_.priority_levels.at(i).id = hierarchy.at(i)->ID();
        std::vector<std::shared_ptr<tpik::Task>> tasks = hierarchy.at(i)->Level();

        for (size_t j = 0; j < tasks.size(); j++) {

            tpikActionMsg_.priority_levels.at(i).tasks_id.push_back(tasks.at(j)->ID());
        }
    }
    tpikActionPub_->publish(tpikActionMsg_);
}

void VehicleController::SurgeHeadingCB(const ulisse_msgs::msg::SurgeHeading::SharedPtr msg)
{
    stateSurgeHeading_->SetSurgeHeading(msg->surge, msg->heading);
}

void VehicleController::SurgeYawRateCB(const ulisse_msgs::msg::SurgeYawRate::SharedPtr msg)
{
    stateSurgeYawRate_->SetSurgeYawRate(msg->surge, msg->yawrate);
}

void VehicleController::NavFilterCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg)
{
    ctrlData_->inertialF_linearPosition.latitude = msg->inertialframe_linear_position.latlong.latitude;
    ctrlData_->inertialF_linearPosition.longitude = msg->inertialframe_linear_position.latlong.longitude;

    // Get the water current for hold state
    ctrlData_->inertialF_waterCurrent[0] = msg->inertialframe_water_current[0];
    ctrlData_->inertialF_waterCurrent[1] = msg->inertialframe_water_current[1];

    ctrlData_->bodyF_angularPosition.Pitch(0.0);
    ctrlData_->bodyF_angularPosition.Roll(0.0);
    ctrlData_->bodyF_angularPosition.Yaw(msg->bodyframe_angular_position.yaw);

    ctrlData_->bodyF_linearVelocity[0] = msg->bodyframe_linear_velocity[0];
    ctrlData_->bodyF_linearVelocity[1] = msg->bodyframe_linear_velocity[1];
    ctrlData_->bodyF_linearVelocity[2] = msg->bodyframe_linear_velocity[2];

    // Linear position in world frame
    Eigen::Vector3d bodyPosUTM;
    ctb::LatLong2LocalUTM(ctrlData_->inertialF_linearPosition, 0.0, centroidLocation_, bodyPosUTM);

    Eigen::Vector3d worldF_vehicleLinearPosition(static_cast<double>(bodyPosUTM.x()), static_cast<double>(bodyPosUTM.y()), 0.0);

    // Updating the robot model
    Eigen::TransformationMatrix worldF_T_vehicleF;
    worldF_T_vehicleF.TranslationVector(worldF_vehicleLinearPosition);
    worldF_T_vehicleF.RotationMatrix(ctrlData_->bodyF_angularPosition.ToRotationMatrix());

    Eigen::Vector6d velocity_fbk = Eigen::Vector6d::Zero();
    velocity_fbk(0) = ctrlData_->bodyF_linearVelocity[0];
    velocity_fbk(1) = ctrlData_->bodyF_linearVelocity[1];

    robotModel_->PositionOnInertialFrame(worldF_T_vehicleF);
    robotModel_->VelocityVector(ulisse::robotModelID::ASV, velocity_fbk);
}

void VehicleController::LLCStatusCB(const ulisse_msgs::msg::LLCStatus::SharedPtr msg)
{
    ctrlData_->radioControllerEnabled = msg->flags.ppm_remote_enabled;
}

void VehicleController::GroundTruthDataCB(const ulisse_msgs::msg::SimulatedSystem::SharedPtr msg)
{
     simulatedData_ = *msg;
     //ctrlDataReal_->inertialF_linearPosition.latitude = simulatedData_.inertialframe_linear_position.latlong.latitude;
     //ctrlDataReal_->inertialF_linearPosition.longitude = simulatedData_.inertialframe_linear_position.latlong.longitude;

     //ctrlDataReal_->inertialF_linearPosition.latitude = msg->inertialframe_linear_position.latlong.latitude;
     //ctrlDataReal_->inertialF_linearPosition.longitude = msg->inertialframe_linear_position.latlong.longitude;

     //ctrlData_->inertialF_linearPosition.latitude = msg->inertialframe_linear_position.latlong.latitude;
     //ctrlData_->inertialF_linearPosition.longitude = msg->inertialframe_linear_position.latlong.longitude;
     real_position_->latitude = msg->inertialframe_linear_position.latlong.latitude;
     real_position_->longitude = msg->inertialframe_linear_position.latlong.longitude;
}

void VehicleController::Run()
{
    if (boundariesSet_) {

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

        /*auto deltays = solver_->DeltaYs();
        int i = 0;
        Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", " / ", "", "", "", ";");
        for (auto& deltay : deltays) {
            std::cout << "[ VC 2 ] deltay_" << i << ": " << deltay.transpose().format(CommaInitFmt) << std::endl;
            i++;
        }*/

        for (int i = 0; i < yTpik_.size(); i++) {
            if (std::isnan(yTpik_(i))) {
                yTpik_(i) = 0.0;
                RCLCPP_INFO(this->get_logger(), "NaN requested velocity");
            }
        }
    }

    tNow_ = std::chrono::system_clock::now();
    PublishControl();
    PublishTasksInfo();
}

void VehicleController::PublishControl()
{

    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow_.time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

    // Publish vehicle status
    ulisse_msgs::msg::VehicleStatus vehicleStatusMsg;
    vehicleStatusMsg.stamp.sec = now_stamp_secs;
    vehicleStatusMsg.stamp.nanosec = now_stamp_nanosecs;
    vehicleStatusMsg.vehicle_state = uFsm_.GetCurrentStateName();
    vehicleStatusPub_->publish(vehicleStatusMsg);

    ulisse_msgs::msg::ReferenceVelocities referenceVelocities;
    referenceVelocities.stamp.sec = now_stamp_secs;
    referenceVelocities.stamp.nanosec = now_stamp_nanosecs;

    // Publish reference velocities, for the DCL, only if we are not in HALT state
    if (uFsm_.GetCurrentStateName() != ulisse::states::ID::halt) {
        // If we are in SURGEYAWRATE state we bypass the Tpik solutions
        if (uFsm_.GetCurrentStateName() == ulisse::states::ID::surgeyawrate) {
            referenceVelocities.desired_surge = stateSurgeYawRate_->goalSurge;
            referenceVelocities.desired_yaw_rate = stateSurgeYawRate_->goalYawRate;
        } else {
            referenceVelocities.desired_surge = yTpik_[0];
            referenceVelocities.desired_yaw_rate = yTpik_[5];
        }
        referenceVelocitiesPub_->publish(referenceVelocities);
    }
}

void VehicleController::PublishTasksInfo()
{
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow_.time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

    // Publish Tasks Information
    for (auto& taskMap : tasksMap_) {
        try {

            std::vector<double> diagonal_internal_activation_function;
            for (unsigned int i = 0; i < taskMap.second.task->InternalActivationFunction().rows(); i++) {
                diagonal_internal_activation_function.push_back(taskMap.second.task->InternalActivationFunction().at(i, i));
            }

            std::vector<double> diagonal_external_activation_function;
            for (unsigned int i = 0; i < taskMap.second.task->ExternalActivationFunction().rows(); i++) {
                diagonal_external_activation_function.push_back(taskMap.second.task->ExternalActivationFunction().at(i, i));
            }

            std::vector<double> referenceRate;
            for (unsigned int i = 0; i < taskMap.second.task->ReferenceRate().size(); i++) {
                referenceRate.push_back(taskMap.second.task->ReferenceRate().at(i));
            }

            ulisse_msgs::msg::TaskStatus taskstatus_msg;

            taskstatus_msg.stamp.sec = now_stamp_secs;
            taskstatus_msg.stamp.nanosec = now_stamp_nanosecs;
            taskstatus_msg.id = taskMap.second.task->ID();
            taskstatus_msg.in_current_action = actionManager_->IsTaskInCurrentAction(taskMap.second.task->ID());
            taskstatus_msg.enabled = taskMap.second.task->Enabled();
            taskstatus_msg.external_activation_function = diagonal_external_activation_function;
            taskstatus_msg.internal_activation_function = diagonal_internal_activation_function;
            taskstatus_msg.reference_rate = referenceRate;

            tasksMap_[taskMap.second.task->ID()].taskPub->publish(taskstatus_msg);

        } catch (tpik::ExceptionWithHow& e) {
            std::cerr << "LOG TASK EXCEPTION" << std::endl;
            std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
        }
    }

    // Publish simplified data for the ulisse_map GUI
    ulisse_msgs::msg::FeedbackGui feedbackGuiMsg;
    feedbackGuiMsg.stamp.sec = now_stamp_secs;
    feedbackGuiMsg.stamp.nanosec = now_stamp_nanosecs;

    if (uFsm_.GetCurrentStateName() == ulisse::states::ID::hold) {
        feedbackGuiMsg.goal_position.latitude = stateHold_->positionToHold.latitude;
        feedbackGuiMsg.goal_position.longitude = stateHold_->positionToHold.longitude;
        feedbackGuiMsg.acceptance_radius = stateHold_->maxAcceptanceRadius;
        feedbackGuiMsg.goal_distance = stateHold_->goalDistance;
    } else if (uFsm_.GetCurrentStateName() == ulisse::states::ID::latlong) {
        feedbackGuiMsg.goal_position.latitude = stateLatLong_->goalPosition.latitude;
        feedbackGuiMsg.goal_position.longitude = stateLatLong_->goalPosition.longitude;
        feedbackGuiMsg.goal_heading = stateLatLong_->goalHeading;
        feedbackGuiMsg.acceptance_radius = stateLatLong_->acceptanceRadius;
        feedbackGuiMsg.goal_distance = stateLatLong_->goalDistance;
    } else if (uFsm_.GetCurrentStateName() == ulisse::states::ID::pathfollow) {        
        feedbackGuiMsg.goal_position.latitude = statePathFollowing_->GetNextPoint().latitude;
        feedbackGuiMsg.goal_position.longitude = statePathFollowing_->GetNextPoint().longitude;
        feedbackGuiMsg.current_track_point.latitude = statePathFollowing_->GetCurrentTrackPoint().latitude;
        feedbackGuiMsg.current_track_point.longitude = statePathFollowing_->GetCurrentTrackPoint().longitude;
        feedbackGuiMsg.goal_distance = statePathFollowing_->GetDistanceToEnd();

        ulisse_msgs::msg::PathFollow pathFollowMsg;
        pathFollowMsg.stamp.sec = now_stamp_secs;
        pathFollowMsg.stamp.nanosec = now_stamp_nanosecs;
        pathFollowMsg.gain = 0;
        pathFollowMsg.delta = statePathFollowing_->GetDeltaY();
        pathFollowMsg.y = statePathFollowing_->GetY();
        pathFollowMsg.x = 0;
        pathFollowMsg.x_dot = 0;
        pathFollowMsg.psi = 0;

        pathFollowMsg.heading2closest_point = 0;
        pathFollowMsg.goal_heading = statePathFollowing_->GetGoalHeading();
        pathFollowMsg.heading_error = statePathFollowing_->GetHeadingError();
        pathFollowMsg.y_real = statePathFollowing_->GetYReal();

        pathFollowPub_->publish(pathFollowMsg);

    } else if (uFsm_.GetCurrentStateName() == ulisse::states::ID::pathfollow_ilos){                 // ILOS for the whole path
        feedbackGuiMsg.goal_position.latitude = statePathFollowingILOS_->GetNextPoint().latitude;
        feedbackGuiMsg.goal_position.longitude = statePathFollowingILOS_->GetNextPoint().longitude;
        feedbackGuiMsg.current_track_point.latitude = statePathFollowingILOS_->GetCurrentTrackPoint().latitude;
        feedbackGuiMsg.current_track_point.longitude = statePathFollowingILOS_->GetCurrentTrackPoint().longitude;
        feedbackGuiMsg.goal_distance = statePathFollowingILOS_->GetDistanceToEnd();
        ulisse_msgs::msg::PathFollow pathFollowIlosMsg;
        pathFollowIlosMsg.stamp.sec = now_stamp_secs;
        pathFollowIlosMsg.stamp.nanosec = now_stamp_nanosecs;
        pathFollowIlosMsg.gain = statePathFollowingILOS_->GetSigmaY();
        pathFollowIlosMsg.delta = statePathFollowingILOS_->GetDeltaY();
        pathFollowIlosMsg.y = statePathFollowingILOS_->GetY();
        pathFollowIlosMsg.x = statePathFollowingILOS_->GetYint();
        pathFollowIlosMsg.x_dot = statePathFollowingILOS_->GetYintDot();
        pathFollowIlosMsg.psi = statePathFollowingILOS_->GetPsi();

        pathFollowIlosMsg.heading2closest_point = statePathFollowingILOS_->GetHeading2ClosetPoint();
        pathFollowIlosMsg.goal_heading = statePathFollowingILOS_->GetGoalHeading();
        pathFollowIlosMsg.heading_error = statePathFollowingILOS_->GetHeadingError();
        pathFollowIlosMsg.y_real = statePathFollowingILOS_->GetYReal();
        //pathFollowIlosMsg.y_real = simulatedData_.n_p;
        pathFollowPub_->publish(pathFollowIlosMsg);
    } else if (uFsm_.GetCurrentStateName() == ulisse::states::ID::pathfollow_current){                 // Current estimator for the whole path
        feedbackGuiMsg.goal_position.latitude = statePathFollowingCurrent_->GetNextPoint().latitude;
        feedbackGuiMsg.goal_position.longitude = statePathFollowingCurrent_->GetNextPoint().longitude;
        feedbackGuiMsg.current_track_point.latitude = statePathFollowingCurrent_->GetCurrentTrackPoint().latitude;
        feedbackGuiMsg.current_track_point.longitude = statePathFollowingCurrent_->GetCurrentTrackPoint().longitude;
        feedbackGuiMsg.goal_distance = statePathFollowingCurrent_->GetDistanceToEnd();

        ulisse_msgs::msg::PathFollow pathFollowMsg;
        pathFollowMsg.stamp.sec = now_stamp_secs;
        pathFollowMsg.stamp.nanosec = now_stamp_nanosecs;
        pathFollowMsg.gain = 0;
        pathFollowMsg.delta = statePathFollowingCurrent_->GetDeltaY();
        pathFollowMsg.y = statePathFollowingCurrent_->GetY();
        pathFollowMsg.x = 0;
        pathFollowMsg.x_dot = 0;
        pathFollowMsg.psi = 0;

        pathFollowMsg.heading2closest_point = 0;
        pathFollowMsg.goal_heading = statePathFollowingCurrent_->GetGoalHeading();
        pathFollowMsg.heading_error = statePathFollowingCurrent_->GetHeadingError();
        pathFollowMsg.y_real = statePathFollowingCurrent_->GetYReal();

        pathFollowPub_->publish(pathFollowMsg);
    } else if (uFsm_.GetCurrentStateName() == ulisse::states::ID::pathfollow_iloscurrent){          // ILOS on straight lines and Current estimator in the curves
        feedbackGuiMsg.goal_position.latitude = statePathFollowingILOSCurrent_->GetNextPoint().latitude;
        feedbackGuiMsg.goal_position.longitude = statePathFollowingILOSCurrent_->GetNextPoint().longitude;
        feedbackGuiMsg.current_track_point.latitude = statePathFollowingILOSCurrent_->GetCurrentTrackPoint().latitude;
        feedbackGuiMsg.current_track_point.longitude = statePathFollowingILOSCurrent_->GetCurrentTrackPoint().longitude;
        feedbackGuiMsg.goal_distance = statePathFollowingILOSCurrent_->GetDistanceToEnd();

        ulisse_msgs::msg::PathFollow pathFollowMsg;
        pathFollowMsg.stamp.sec = now_stamp_secs;
        pathFollowMsg.stamp.nanosec = now_stamp_nanosecs;
        pathFollowMsg.gain = statePathFollowingILOSCurrent_->GetSigmaY();
        pathFollowMsg.delta = statePathFollowingILOSCurrent_->GetDeltaY();
        pathFollowMsg.y = statePathFollowingILOSCurrent_->GetY();
        pathFollowMsg.x = statePathFollowingILOSCurrent_->GetYint();
        pathFollowMsg.x_dot = statePathFollowingILOSCurrent_->GetYintDot();
        pathFollowMsg.psi = statePathFollowingILOSCurrent_->GetPsi();

        pathFollowMsg.heading2closest_point = statePathFollowingILOSCurrent_->GetHeading2ClosetPoint();
        pathFollowMsg.goal_heading = statePathFollowingILOSCurrent_->GetGoalHeading();
        pathFollowMsg.heading_error = statePathFollowingILOSCurrent_->GetHeadingError();
        pathFollowMsg.y_real = statePathFollowingILOSCurrent_->GetYReal();

        pathFollowPub_->publish(pathFollowMsg);
    } else if (uFsm_.GetCurrentStateName() == ulisse::states::ID::pathfollow_alos){                 // ALOS for the whole path
        feedbackGuiMsg.goal_position.latitude = statePathFollowingALOS_->GetNextPoint().latitude;
        feedbackGuiMsg.goal_position.longitude = statePathFollowingALOS_->GetNextPoint().longitude;
        feedbackGuiMsg.current_track_point.latitude = statePathFollowingALOS_->GetCurrentTrackPoint().latitude;
        feedbackGuiMsg.current_track_point.longitude = statePathFollowingALOS_->GetCurrentTrackPoint().longitude;
        
        //RCLCPP_INFO_STREAM(this->get_logger(), "trackPoint: " << statePathFollowingALOS_->GetCurrentTrackPoint());

        feedbackGuiMsg.goal_distance = statePathFollowingALOS_->GetDistanceToEnd();
        ulisse_msgs::msg::PathFollow pathFollowAlosMsg;
        pathFollowAlosMsg.stamp.sec = now_stamp_secs;
        pathFollowAlosMsg.stamp.nanosec = now_stamp_nanosecs;

        pathFollowAlosMsg.gain = statePathFollowingALOS_->GetGammaY(); // gamma in ALOS case
        pathFollowAlosMsg.delta = statePathFollowingALOS_->GetDeltaY();
        pathFollowAlosMsg.y = statePathFollowingALOS_->GetY();
        pathFollowAlosMsg.x = statePathFollowingALOS_->GetBeta(); // Beta_hat in ALOS case
        pathFollowAlosMsg.x_dot = statePathFollowingALOS_->GetBetaDot(); // Beta_hat_dot in ALOS case
        pathFollowAlosMsg.psi = 0.0;
        pathFollowAlosMsg.heading2closest_point = statePathFollowingALOS_->GetHeading2ClosetPoint();
        pathFollowAlosMsg.goal_heading = statePathFollowingALOS_->GetGoalHeading();
        pathFollowAlosMsg.heading_error = statePathFollowingALOS_->GetHeadingError();
        pathFollowAlosMsg.y_real = statePathFollowingALOS_->GetYReal();
        pathFollowPub_->publish(pathFollowAlosMsg);
    }

    feedbackGuiPub_->publish(feedbackGuiMsg);
}
} // namespace ulisse
