#include "ulisse_ctrl/kinematic_vehicle_controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <jsoncpp/json/json.h>

#include "ulisse_ctrl/configuration.hpp"
#include "ulisse_ctrl/states/generic_state.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"

#include "ulisse_msgs/terminal_utils.hpp"
#include "ulisse_msgs/topicnames.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace ulisse {

VehicleController::VehicleController(std::string conf_filename)
    : Node("kinematic_control_node")
    , boundariesSet_(false) // changed to true!!
{
    conf_ = std::make_shared<KCLConfiguration>();

    ctrlData_ = std::make_shared<ControlData>();
    rovData_ = std::make_shared<ControlData>();

    fileName_ = conf_filename;

    stateHalt_ = std::make_shared<states::StateHalt>();
    stateHold_ = std::make_shared<states::StateHold>();
    statePathFollowing_ = std::make_shared<states::StatePathFollow>();
    stateRovFollowing_ = std::make_shared<states::StateRovFollow>(); // ROV-ASV
    stateLatLong_ = std::make_shared<states::StateLatLong>();
    stateSurgeHeading_ = std::make_shared<states::StateSurgeHeading>();
    stateSurgeYawRate_ = std::make_shared<states::StateSurgeYawRate>();

    // Sensor Subscriptions
    navFilterSub_ = this->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10, std::bind(&VehicleController::NavFilterCB, this, _1));
    navFilterROVSub_ = this->create_subscription<rov_msgs::msg::NavFilterData>("/rov/nav_filter/data", 10, std::bind(&VehicleController::NavFilterRovCB, this, _1)); //ROV
    llcStatusSub_ = this->create_subscription<ulisse_msgs::msg::LLCStatus>(ulisse_msgs::topicnames::llc_status, 10, std::bind(&VehicleController::LLCStatusCB, this, _1));
    cableROVSub_ = this->create_subscription<rov_msgs::msg::CableData>("/winch/cable_data", 10, std::bind(&VehicleController::CableDataRovCB, this, _1)); //ROV
    obstacleSub_ = this->create_subscription<detav_msgs::msg::ObstacleList>(ulisse_msgs::topicnames::obstacle, 10, std::bind(&VehicleController::ObstacleCB, this, _1)); // ASV-ROV

    // Data Subscriptions
    surgeHeadingSub_ = this->create_subscription<ulisse_msgs::msg::SurgeHeading>(ulisse_msgs::topicnames::surge_heading, 10, std::bind(&VehicleController::SurgeHeadingCB, this, _1));
    surgeYawRateSub_ = this->create_subscription<ulisse_msgs::msg::SurgeYawRate>(ulisse_msgs::topicnames::surge_yawrate, 10, std::bind(&VehicleController::SurgeYawRateCB, this, _1));

    // Avoidance Path Subscription
    avoidancePathSub_ = this->create_subscription<ulisse_msgs::msg::CommandPathFollow>("ulisse/avoidance/current_path", 10, std::bind(&VehicleController::AvoidancePathCB, this, _1)); // ASV-ROV

    // Control Publishers
    genericLogPub_ = this->create_publisher<std_msgs::msg::String>("/ulisse/log/generic", 10);
    vehicleStatusPub_ = this->create_publisher<ulisse_msgs::msg::VehicleStatus>(ulisse_msgs::topicnames::vehicle_status, 10);
    referenceVelocitiesPub_ = this->create_publisher<ulisse_msgs::msg::ReferenceVelocities>(ulisse_msgs::topicnames::reference_velocities, 10);
    feedbackGuiPub_ = this->create_publisher<ulisse_msgs::msg::FeedbackGui>(ulisse_msgs::topicnames::feedback_gui, 10);
    tpikActionPub_ = this->create_publisher<ulisse_msgs::msg::TPIKAction>(ulisse_msgs::topicnames::tpik_action, 10);
    referenceCablePub_ = this->create_publisher<rov_msgs::msg::CableReference>("/winch/reference_cable", 10);
    referenceWinchMotorPub_ = this->create_publisher<rov_msgs::msg::WinchMotorReference>("/winch/reference_motor", 10); // using the tether model
    plotVarPub_ = this->create_publisher<ulisse_msgs::msg::PlotVariables>(ulisse_msgs::topicnames::plot_variables, 10); // ASV-ROV plot

    safetyBoundarySetPub_ = this->create_publisher<std_msgs::msg::Bool>(ulisse_msgs::topicnames::safety_boundary_set, 10);

    /// TPIK Manager
    actionManager_ = std::make_shared<tpik::ActionManager>(tpik::ActionManager());

    /// ROBOT MODEL
    Eigen::TransformationMatrix world_T_vehicle;

    /// OBSTACLE MODELS
    Eigen::TransformationMatrix world_T_obstacle1;
    Eigen::TransformationMatrix world_T_obstacle2;
    Eigen::TransformationMatrix world_T_obstacle3;

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

    // AUV CONTROL ANGULAR POSITION ASV-ROV
    asvAngularPositionRovFollowing_ = std::make_shared<ikcl::AlignToTarget>(ikcl::AlignToTarget(ulisse::task::asvAngularPositionRovFollow, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvAngularPositionRovFollowing_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_angular_position_rov_follow, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvAngularPositionRovFollow, taskInfo_));

    // ASV CONTROL OBSTACLE AVOIDANCE
    asvAngularPositionObstacle_ = std::make_shared<ikcl::AlignToTarget>(ikcl::AlignToTarget(ulisse::task::asvAngularPositionObstacle, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvAngularPositionObstacle_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_angular_position_obstacle, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvAngularPositionObstacle, taskInfo_));

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

    // ASV CONTROL DISTANCE PATH FOLLOWING ASV-ROV
    asvCartesianDistanceRovFollowing_ = std::make_shared<ikcl::CartesianDistance>(ikcl::CartesianDistance(ulisse::task::asvCartesianDistanceRovFollowing, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvCartesianDistanceRovFollowing_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_cartesian_distance_rov_follow, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvCartesianDistanceRovFollowing, taskInfo_));

    // ASV CONTROL DISTANCE Obstacle ASV-ROV
    asvCartesianDistanceObstacle_ = std::make_shared<ikcl::CartesianDistance>(ikcl::CartesianDistance(ulisse::task::asvCartesianDistanceObstacle, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvCartesianDistanceObstacle_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_cartesian_distance_obstacle, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvCartesianDistanceObstacle, taskInfo_));

    // ASV SAFETY BOUNDARIES (INEQUALITY TASK) // Safety
    asvSafetyBoundaries_ = std::make_shared<ikcl::SafetyBoundaries>(ikcl::SafetyBoundaries(ulisse::task::asvSafetyBoundaries, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvSafetyBoundaries_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_safety_boundaries, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvSafetyBoundaries, taskInfo_));

    // ASV absolute axis alignment task
    asvAbsoluteAxisAlignment_ = std::make_shared<ikcl::AbsoluteAxisAlignment>(ikcl::AbsoluteAxisAlignment(ulisse::task::asvAbsoluteAxisAlignment, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvAbsoluteAxisAlignment_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvAbsoluteAxisAlignment, taskInfo_));

    // ASV absolute axis alignment task // Safety
    asvAbsoluteAxisAlignmentSafety_ = std::make_shared<ikcl::AbsoluteAxisAlignment>(ikcl::AbsoluteAxisAlignment(ulisse::task::asvAbsoluteAxisAlignmentSafety, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvAbsoluteAxisAlignmentSafety_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment_safety, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvAbsoluteAxisAlignmentSafety, taskInfo_));

    // ASV absolute axis alignment task hold
    asvAbsoluteAxisAlignmentHold_ = std::make_shared<ikcl::AbsoluteAxisAlignment>(ikcl::AbsoluteAxisAlignment(ulisse::task::asvAbsoluteAxisAlignmentHold, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvAbsoluteAxisAlignmentHold_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment_hold, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvAbsoluteAxisAlignmentHold, taskInfo_));

    // ASV absolute axis alignment task obstacle avoidance  
/*    asvAbsoluteAxisAlignmentObstacle_ = std::make_shared<ikcl::AbsoluteAxisAlignment>(ikcl::AbsoluteAxisAlignment(ulisse::task::asvAbsoluteAxisAlignmentObstacle, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvAbsoluteAxisAlignmentObstacle_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment_obstacle, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvAbsoluteAxisAlignmentObstacle, taskInfo_)); */

    // ASV CONTROL VELOCITY LINEAR HOLD
    asvLinearVelocityHold_ = std::make_shared<ikcl::LinearVelocity>(ikcl::LinearVelocity(ulisse::task::asvLinearVelocityHold, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvLinearVelocityHold_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_linear_velocity_hold, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvLinearVelocityHold, taskInfo_));

    // ASV CONTROL VELOCITY LINEAR Obstacle
/*    asvLinearVelocityObstacle_ = std::make_shared<ikcl::LinearVelocity>(ikcl::LinearVelocity(ulisse::task::asvLinearVelocityObstacle, robotModel_, ulisse::robotModelID::ASV));
    taskInfo_.task = asvLinearVelocityObstacle_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_linear_velocity_obstacle, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvLinearVelocityObstacle, taskInfo_)); */

    // ASV Obstacle Avoidance task

    /*const Eigen::Vector3d W_position;
    W_position.Zero();
    obs_distance = obs1_->ComputeDistance(W_position);*/

    //const std::string obs1_id = "obstacle1_frameID";
    //obs1_ = std::make_shared<ikcl::SphereObstacle>(world_T_obstacle, obs1_id, 3.0);
    //obs1_ = std::make_shared<ikcl::SphereObstacle>();

    //std::vector<std::string> obstacleFrames(1,ulisse::robotModelID::ASV);
    std::vector<std::string> obstacleFrames = {ulisse::robotModelID::ASV};
    //obstacleFrames.push_back(ulisse::robotModelID::ASV);

    //Eigen::TransformationMatrix world_T_obstacle2;
    Eigen::TransformationMatrix world_T_obstacle;
    world_T_obstacle.setZero();
    Eigen::Vector3d vec(0.0, 5.0, 0.0);
    world_T_obstacle.TranslationVector(vec);
    obs1_ = std::make_shared<ikcl::SphereObstacle>(world_T_obstacle, rml::FrameID::WorldFrame, 1.5);
    //obstaclePointers_.push_back(obs1_); // no initialization

    asvObstacleAvoidance_ = std::make_shared<ikcl::ObstacleAvoidance>(ikcl::ObstacleAvoidance(ulisse::task::asvObstacleAvoidance, robotModel_, obstacleFrames, obstaclePointers_));
    taskInfo_.task = asvObstacleAvoidance_;
    taskInfo_.taskPub = this->create_publisher<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_obstacle_avoidance, 1);
    tasksMap_.insert(std::make_pair(ulisse::task::asvObstacleAvoidance, taskInfo_));    

    obstaclePointers_.clear();

    /*
    world_T_obstacle2.setZero();
    Eigen::Vector3d vec2(0.0, 10.0, 0.0);
    world_T_obstacle2.TranslationVector(vec2);
    obs2_ = std::make_shared<ikcl::SphereObstacle>(world_T_obstacle2, rml::FrameID::WorldFrame, 1.5);
*/
    //obstaclePointers_.push_back(obs2_); // no initialization
    //asvObstacleAvoidance_->Obstacles() = obstaclePointers_; // no initialization

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
    std::cout << "Configuration Done: " << std::endl;

    // solver_ definition
    solver_ = std::make_shared<tpik::Solver>(tpik::Solver(actionManager_, iCat_));
    std::cout << "Define Solver Done: " << std::endl;

    // FSM Initialization
    SetUpFSM();
    std::cout << "SetUpFSM: " << std::endl;

    srvCommand_ = this->create_service<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service,
        std::bind(&VehicleController::CommandsHandler, this, _1, _2, _3));

    srvSetBoundaries_ = this->create_service<ulisse_msgs::srv::SetBoundaries>(ulisse_msgs::topicnames::set_boundaries_service,
        std::bind(&VehicleController::SetBoundariesHandler, this, _1, _2, _3));

    srvGetBoundaries_ = this->create_service<ulisse_msgs::srv::GetBoundaries>(ulisse_msgs::topicnames::get_boundaries_service,
        std::bind(&VehicleController::GetBoundariesHandler, this, _1, _2, _3));

    srvResetConf_ = this->create_service<ulisse_msgs::srv::ResetConfiguration>(ulisse_msgs::topicnames::reset_kcl_conf_service,
        std::bind(&VehicleController::ResetConfHandler, this, _1, _2, _3));

    srvAvoidancePath_ = this->create_client<ulisse_msgs::srv::ComputeAvoidancePath>(ulisse_msgs::topicnames::control_avoidance_cmd_service);
    //srvAvoidancePath_.reset();

    // variable initialization
    dist_now_ = 0.0;
    dist_last_ = 0.0;
    t_last_ = t_now_ = std::chrono::system_clock::now();
    ctrlData_->avoidancePathEnabled = false;
    sentReq = false;

    // Main function timer
    int msRunPeriod = 1.0 / (conf_->controlLoopRate) * 1000;
    std::cout << "Controller Rate: " << conf_->controlLoopRate << "Hz" << std::endl;
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
    std::cout<< "PLs configured!" <<std::endl <<std::endl;

    if (!ConfigureActionsFromFile(actionManager_, confObj)) {
        std::cerr << "Failed to load  Actions from file" << std::endl;
        return false;
    };
    std::cout<< "Actions configured!" <<std::endl << std::endl;

    //insert states in the map
    statesMap_.insert({ ulisse::states::ID::halt, stateHalt_ });
    statesMap_.insert({ ulisse::states::ID::hold, stateHold_ });
    statesMap_.insert({ ulisse::states::ID::latlong, stateLatLong_ });
    statesMap_.insert({ ulisse::states::ID::pathfollow, statePathFollowing_ });
    statesMap_.insert({ ulisse::states::ID::surgeheading, stateSurgeHeading_ });
    statesMap_.insert({ ulisse::states::ID::surgeyawrate, stateSurgeYawRate_ });
    statesMap_.insert({ ulisse::states::ID::rovfollow, stateRovFollowing_ });

    if (!ConfigureSatesFromFile(statesMap_, confObj)) {
        std::cerr << "Failed to load States from file" << std::endl;
        return false;
    };
    std::cout<< "Map configured!" <<std::endl << std::endl;


    //insert command in the map
    commandsMap_.insert({ ulisse::commands::ID::halt, commandHalt_ });
    commandsMap_.insert({ ulisse::commands::ID::hold, commandHold_ });
    commandsMap_.insert({ ulisse::commands::ID::latlong, commandLatLong_ });
    commandsMap_.insert({ ulisse::commands::ID::pathfollow, commandPathFollowing_ });
    commandsMap_.insert({ ulisse::commands::ID::surgeheading, commandSurgeHeading_ });
    commandsMap_.insert({ ulisse::commands::ID::surgeyawrate, commandSurgeYawRate_ });
    commandsMap_.insert({ ulisse::commands::ID::rovfollow, commandRovFollowing_ });

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

    commandRovFollowing_.SetFSM(&uFsm_);
    commandRovFollowing_.SetState(stateRovFollowing_);

    std::cout << "Command and States are set" << std::endl;

    // ***** STATES ***** //
    //Set the fsm and the structure that the states need.
    for (auto& state : statesMap_) {
        state.second->actionManager = actionManager_;
        state.second->robotModel = robotModel_;
        state.second->tasksMap = tasksMap_;
        state.second->ctrlData = ctrlData_;
        //state.second->obstaclesVector = obstaclesVector_; // Obstacle Avoidance
        state.second->SetFSM(&uFsm_);
    }
    std::cout << "State.second are set" << std::endl;

    // ***** EVENTS ***** //
    eventRcEnabled_.SetFSM(&uFsm_);
    eventNearGoalPosition_.SetFSM(&uFsm_);
    eventNearGoalPosition_.ControlData() = ctrlData_;
    eventNearGoalPosition_.GoToHoldAfterMove(conf_->goToHoldAfterMove);
    eventNearGoalPosition_.StateHold() = std::dynamic_pointer_cast<ulisse::states::StateHold>(statesMap_.find(ulisse::states::ID::hold)->second);
    eventNearGoalPosition_.StateRovFollow() = std::dynamic_pointer_cast<ulisse::states::StateRovFollow>(statesMap_.find(ulisse::states::ID::rovfollow)->second);
    // Add here the same for the eventLongCable_
    eventLongTether_.SetFSM(&uFsm_);
    eventLongTether_.ControlData() = ctrlData_;
    eventLongTether_.StatePathFollow() = std::dynamic_pointer_cast<ulisse::states::StatePathFollow>(statesMap_.find(ulisse::states::ID::pathfollow)->second);

    eventFarAlignmentPosition_.SetFSM(&uFsm_);
    eventFarAlignmentPosition_.ControlData() = ctrlData_;
    eventFarAlignmentPosition_.StatePathFollow() = std::dynamic_pointer_cast<ulisse::states::StatePathFollow>(statesMap_.find(ulisse::states::ID::pathfollow)->second);

    std::cout << "Events are configured" << std::endl;

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
    uFsm_.AddEvent(ulisse::events::names::longtether, &eventLongTether_);
    uFsm_.AddEvent(ulisse::events::names::faralignmentposition, &eventFarAlignmentPosition_);

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
                    //request->latlong_cmd.acceptance_radius)){
                    request->latlong_cmd.acceptance_radius, request->latlong_cmd.ref_speed)){
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

            if (!statePathFollowing_->LoadPath(request->path_cmd.path)) {
                response->res = "CommandAnswer::fail - Malformed Path Message.";
                ret = fsm::retval::fail;
            }
            log << "Received Command PathFollowing (id: " << request->path_cmd.path.id << " )";
            PublishLog(log.str().c_str());

        } else if (request->command_type == ulisse::commands::ID::rovfollow) {

            std::cout << "Received Command ROV Following" << std::endl;
            if(!commandRovFollowing_.SetGoTo(LatLong(rovData_->inertialF_linearPosition.latitude, rovData_->inertialF_linearPosition.longitude))){
                response->res = "CommandAnswer::fail - Malformed ROV following Message.";
                ret = fsm::retval::fail;
            }

            //log << "Received Command ROV Follow (lat: " << request->rov_cmd.goal.latitude << " , long: " << request->rov_cmd.goal.longitude << " )";
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
                    std::cerr << "Command Handler" << std::endl;
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
    ctrlData_->inertialF_waterCurrent[2] = 0.0;

    ctrlData_->bodyF_angularPosition.Pitch(0.0);
    ctrlData_->bodyF_angularPosition.Roll(0.0);
    ctrlData_->bodyF_angularPosition.Yaw(msg->bodyframe_angular_position.yaw);

    ctrlData_->bodyF_linearVelocity[0] = msg->bodyframe_linear_velocity[0];
    ctrlData_->bodyF_linearVelocity[1] = msg->bodyframe_linear_velocity[1];
    ctrlData_->bodyF_linearVelocity[2] = msg->bodyframe_linear_velocity[2];

    // Linear position in world frame
    Eigen::Vector3d worldF_vehicleLinearPosition(ctrlData_->inertialF_linearPosition.latitude, ctrlData_->inertialF_linearPosition.longitude, 0.0);
    // Updating the robot model
    Eigen::TransformationMatrix worldF_T_vehicleF;
    Eigen::TransformationMatrix worldCentroidF_T_vehicleF; //juri

    Eigen::Vector3d worldF_vehicleLinearPositionUTM;
    ctb::LatLong2LocalUTM(LatLong(ctrlData_->inertialF_linearPosition.latitude, ctrlData_->inertialF_linearPosition.longitude), 0.0, centroidLocation_, worldF_vehicleLinearPositionUTM);
    worldCentroidF_T_vehicleF.TranslationVector(worldF_vehicleLinearPositionUTM);
    worldCentroidF_T_vehicleF.RotationMatrix(ctrlData_->bodyF_angularPosition.ToRotationMatrix());

    worldF_T_vehicleF.TranslationVector(worldF_vehicleLinearPosition);
    worldF_T_vehicleF.RotationMatrix(ctrlData_->bodyF_angularPosition.ToRotationMatrix());

    Eigen::Vector6d velocity_fbk = Eigen::Vector6d::Zero();
    velocity_fbk(0) = ctrlData_->bodyF_linearVelocity[0];
    velocity_fbk(1) = ctrlData_->bodyF_linearVelocity[1];

    //robotModel_->PositionOnInertialFrame(worldF_T_vehicleF); // original
    robotModel_->PositionOnInertialFrame(worldCentroidF_T_vehicleF); // juri
    robotModel_->VelocityVector(ulisse::robotModelID::ASV, velocity_fbk);
}

void VehicleController::NavFilterRovCB(const rov_msgs::msg::NavFilterData::SharedPtr msg){ //ROV
    rovData_->inertialF_linearPosition.latitude = msg->inertialframe_linear_position.latlong.latitude;
    rovData_->inertialF_linearPosition.longitude = msg->inertialframe_linear_position.latlong.longitude;
    rovData_->inertialF_altitude = msg->inertialframe_linear_position.altitude;

    // Get the water current for hold state
    rovData_->inertialF_waterCurrent[0] = msg->inertialframe_water_current[0];
    rovData_->inertialF_waterCurrent[1] = msg->inertialframe_water_current[1];
    rovData_->inertialF_waterCurrent[2] = msg->inertialframe_water_current[2];

    rovData_->bodyF_angularPosition.Pitch(msg->bodyframe_angular_position.pitch);
    rovData_->bodyF_angularPosition.Roll(msg->bodyframe_angular_position.roll);
    rovData_->bodyF_angularPosition.Yaw(msg->bodyframe_angular_position.yaw);

    rovData_->bodyF_linearVelocity[0] = msg->bodyframe_linear_velocity[0];
    rovData_->bodyF_linearVelocity[1] = msg->bodyframe_linear_velocity[1];
    rovData_->bodyF_linearVelocity[2] = msg->bodyframe_linear_velocity[2];
}

void VehicleController::CableDataRovCB(const rov_msgs::msg::CableData::SharedPtr msg){ //ROV
    cableData_ = *msg;
    ctrlData_->cable_length =  cableData_.released_cable_length;
}

//void VehicleController::ObstacleCB(const ulisse_msgs::msg::Obstacle::SharedPtr msg){ //ROV
//    //obstacleData_.
//    ulisse_msgs::msg::Obstacle obs;
//    obs = *msg;
//    /*obs.id = msg->id;
//    obs.center.latitude = msg->center.latitude;
//    obs.center.longitude = msg->center.longitude;
//    obs.b_box_dim_x = msg->b_box_dim_x;
//    obs.b_box_dim_y = msg->b_box_dim_y;*/
//    //obstacleMsg = true;
//    bool ExistsObs = false;
//    for(unsigned long i=0; i < ctrlData_->obstacleMsgVector.size(); i++){
//        if(obs.id == ctrlData_->obstacleMsgVector[i].id){
//            //std::shared_ptr<ikcl::SphereObstacle> obs;
//            ctrlData_->obstacleMsgVector[i].center.latitude = obs.center.latitude;
//            ctrlData_->obstacleMsgVector[i].center.longitude = obs.center.longitude;
//            ExistsObs = true;
//        }
//    }
//    if(!ExistsObs){
//        ctrlData_->obstacleMsgVector.push_back(obs);
//    }
//}

void VehicleController::ObstacleCB(const detav_msgs::msg::ObstacleList::SharedPtr msg){ //ROV
    //obstacleData_.
    detav_msgs::msg::Obstacle obs;
    detav_msgs::msg::ObstacleList obs_list;

    unsigned long n;
    //n = msg->size();
    obs_list = *msg;
    n = obs_list.obstacles.size();
    //std::cout << "size = " << n << std::endl;
    /*obs.id = msg->id;
    obs.center.latitude = msg->center.latitude;
    obs.center.longitude = msg->center.longitude;
    obs.b_box_dim_x = msg->b_box_dim_x;
    obs.b_box_dim_y = msg->b_box_dim_y;*/
    //obstacleMsg = true;

    ctrlData_->obstacleMsgVector.clear();
    for(unsigned long i=0; i < n; i++){
        //std::shared_ptr<ikcl::SphereObstacle> obs;
        obs = obs_list.obstacles[i];
        ctrlData_->obstacleMsgVector.push_back(obs);
    }
    /*bool ExistsObs = false;
    for(unsigned long i=0; i < ctrlData_->obstacleMsgVector.size(); i++){
        if(obs.id == ctrlData_->obstacleMsgVector[i].id){
            //std::shared_ptr<ikcl::SphereObstacle> obs;
            ctrlData_->obstacleMsgVector[i].center.latitude = obs.center.latitude;
            ctrlData_->obstacleMsgVector[i].center.longitude = obs.center.longitude;
            ExistsObs = true;
        }
    }
    if(!ExistsObs){
        ctrlData_->obstacleMsgVector.push_back(obs);
    }*/
}

void VehicleController::AvoidancePathCB(const ulisse_msgs::msg::CommandPathFollow::SharedPtr msg){
    //ctrlData_->avoidancePath = *msg;
    aPath_ = *msg;
}

void VehicleController::UpdateObstacles(){
    /*for(unsigned long i= obstaclePointers_.size(); i < ctrlData_->obstacleMsgVector.size(); i++){
        Eigen::Vector3d centerUTM;
        ctb::LatLong2LocalUTM(LatLong(ctrlData_->obstacleMsgVector[i].center.latitude, ctrlData_->obstacleMsgVector[i].center.longitude), 0.0, centroidLocation_, centerUTM);
        Eigen::TransformationMatrix world_T_obstacle;
        world_T_obstacle.setZero();
        world_T_obstacle.TranslationVector(centerUTM);
        std::shared_ptr<ikcl::SphereObstacle> obs;
        obs = std::make_shared<ikcl::SphereObstacle>(world_T_obstacle, rml::FrameID::WorldFrame, ctrlData_->obstacleMsgVector[i].b_box_dim_x/2);
        obstaclePointers_.push_back(obs);
        asvObstacleAvoidance_->Obstacles() = obstaclePointers_; // new line code --needed!!!--
    }*/
    obstaclePointers_.clear();
    for(unsigned long i= 0; i < ctrlData_->obstacleMsgVector.size(); i++){
        Eigen::Vector3d centerUTM;
        ctb::LatLong2LocalUTM(LatLong(ctrlData_->obstacleMsgVector[i].pose.position.position.latitude, ctrlData_->obstacleMsgVector[i].pose.position.position.longitude), 0.0, centroidLocation_, centerUTM);
        Eigen::TransformationMatrix world_T_obstacle;
        world_T_obstacle.setZero();
        world_T_obstacle.TranslationVector(centerUTM);
        std::shared_ptr<ikcl::SphereObstacle> obs;
        obs = std::make_shared<ikcl::SphereObstacle>(world_T_obstacle, rml::FrameID::WorldFrame, ctrlData_->obstacleMsgVector[i].size.size.length);
        obstaclePointers_.push_back(obs);
        asvObstacleAvoidance_->Obstacles() = obstaclePointers_; // new line code --needed!!!--
    }
}

void VehicleController::PrintObstacles(std::vector<std::shared_ptr<ikcl::Obstacle>> obstaclePointers){
    for(unsigned long i=0; i < obstaclePointers.size(); i++) {
        std::shared_ptr<ikcl::SphereObstacle> obs_i = std::dynamic_pointer_cast<ikcl::SphereObstacle>(obstaclePointers[i]);
        Eigen::TransformationMatrix world_T_obstacle = obs_i->CenterFrame();
        std::cout << " obstaclePointers_" << i+1 << std::endl;
        std::cout << world_T_obstacle  << std::endl;
        std::cout << "radius = " << obs_i->Radius() << std::endl;
    }
    std::cout << std::endl;
}

void VehicleController::ComputeCableLength(){

    double goalDistance, goalHeading;
    ctb::DistanceAndAzimuthRad(ctrlData_->inertialF_linearPosition, rovData_->inertialF_linearPosition, goalDistance, goalHeading);
    goalDistance = sqrt(pow(goalDistance,2) + pow(rovData_->inertialF_altitude,2));
    float alfa_, beta_, gamma_; // cable params
    alfa_ = conf_->alfa;
    beta_ = conf_->beta;
    gamma_ = 100;

    controlledCable_.length = (beta_ + alfa_ * goalHeading ) * goalDistance + gamma_;
}

void VehicleController::ComputeAsvRovDistance(float &distance){
    double goalDistance, goalHeading;
    ctb::DistanceAndAzimuthRad(ctrlData_->inertialF_linearPosition, rovData_->inertialF_linearPosition, goalDistance, goalHeading);
    distance = sqrt(pow(goalDistance,2) + pow(rovData_->inertialF_altitude,2));
}

void VehicleController::ComputeAsvRovDistanceVelocity(float &vel){
    ComputeAsvRovDistance(dist_now_);
    t_now_ = std::chrono::system_clock::now();
    iter_elapsed_ = std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_ - t_last_);
    Ts_ = iter_elapsed_.count() / 1E9;
    vel = (dist_now_ - dist_last_)/Ts_;
    t_last_ = t_now_;
    dist_last_ = dist_now_;
}

void VehicleController::ControlCableLength(const float &rpm_percentage, float &reference_length){
    float precision = 1; //float w;

    if(reference_length > cableData_.max_length)
        reference_length = cableData_.max_length;
    else if(reference_length < cableData_.min_length)
        reference_length = cableData_.min_length;

    if(abs(cableData_.released_cable_length - reference_length) > precision){
        winchMotorRef_.on_off = 1;
        if(cableData_.released_cable_length < reference_length){
            //w = M_PI/30 * rpm;
            winchMotorRef_.rpm_percentage = rpm_percentage;
        }
        else{
            //w = - M_PI/30 * rpm;
            winchMotorRef_.rpm_percentage = - rpm_percentage;
        }
        //float v = w * cableData_.winding_radius / 1000;
        //UpdateCableLength(v,dt);
    }
    else winchMotorRef_.on_off =0;
    //winchMotorRef_.stamp =
    //referenceWinchMotorPub_->publish(winchMotorRef_);
}

void VehicleController::ComputeCableVelocity(const float &l_cable, float &vel_cable){
    float distance, dist_vel;
    ComputeAsvRovDistance(distance);
    ComputeAsvRovDistanceVelocity(dist_vel);

    float a1, b1, c1, a2, b2, c2; // ax + by + c = 0; x is distnace and y is cable length
    a1 = conf_->Eq1_a; b1=-1; c1= conf_->Eq1_b;
    a2 = conf_->Eq2_a;; b2=-1; c2= conf_->Eq2_b;
    float x ,y; x = distance; y = l_cable;
    float d1 = abs(a1*x + b1*y + c1)/sqrt(pow(a1,2)+pow(b1,2)); // distance of the point from the line1
    float d2 = abs(a2*x + b2*y + c2)/sqrt(pow(a2,2)+pow(b2,2)); // distance of the point from the line2

    //std::cout << "l_cable = "<< l_cable<< std::endl;
    //std::cout << "distanceASV-ROV = "<< distance<< std::endl;
    //std::cout << "dist_velASV-ROV = "<< dist_vel<< std::endl;
    //std::cout << "d1 = "<< d1<< std::endl;
    //std::cout << "d2 = "<< d2<< std::endl;
    //std::cout << "Eq_delta = "<< conf_->Eq_delta << std::endl;
    //std::cout << "Eq_delta = "<< conf_->Eq_delta << std::endl;
    //std::cout << "Eq1_a = "<< conf_->Eq1_a << std::endl;

//    if(d1 < conf_->Eq_delta){
//        if (abs(dist_vel) < 1)
//            vel_cable = 1;
//        else
//            vel_cable = abs(a1 * dist_vel);
//    }
//    else if(d2 < conf_->Eq_delta){
//        if (abs(dist_vel) < 1)
//            vel_cable = -1;
//        else
//            vel_cable = -abs(a2 * dist_vel);
//    }
//    else
//        vel_cable = 0.0;

    //if(d1 < conf_->Eq_delta){
    if(d1 < distance/10){
        double gain = rml::DecreasingBellShapedFunction(conf_->Eq_delta - 0.5, conf_->Eq_delta, 0, 1.0, d1);
        vel_cable = gain * 1.0;
    }
    //else if(d2 < conf_->Eq_delta){
    else if(d2 < distance/10){
        double gain = rml::DecreasingBellShapedFunction(conf_->Eq_delta - 0.5, conf_->Eq_delta, 0, 1.0, d2);
        vel_cable = - gain * 1.0;
    }
    else
        vel_cable = 0.0;

    //std::cout << "vel_cable = "<< vel_cable << std::endl;
    //std::cout << "-----------" << std::endl;
}

void VehicleController::LLCStatusCB(const ulisse_msgs::msg::LLCStatus::SharedPtr msg)
{
    ctrlData_->radioControllerEnabled = msg->flags.ppm_remote_enabled;
}

bool VehicleController::sendLatLongAvoidanceCommand(const ctb::LatLong & goal, double radius, double ref_speed, bool COLREGS)
{
  auto serviceReq = std::make_shared<ulisse_msgs::srv::ComputeAvoidancePath::Request>();

  serviceReq->latlong_cmd.goal.latitude = goal.latitude;
  serviceReq->latlong_cmd.goal.longitude = goal.longitude;
  serviceReq->latlong_cmd.acceptance_radius = radius;
  serviceReq->latlong_cmd.ref_speed = ref_speed;
  serviceReq->colregs_compliant = COLREGS;

  // Chiama servizio di avoidance
  static std::string result_msg;
  bool serviceAvailable;

  if (srvAvoidancePath_->service_is_ready()) {
    auto result_future = srvAvoidancePath_->async_send_request(serviceReq);
    std::cout << "Sent Request to Avoidance" << std::endl;
    //ctrlData_->avoidancePath.path = result_future.get()->path;

    ctrlData_->avoidancePath = aPath_;
    ctrlData_->avoidancePathGenerated = true;
    std::cout << "avoidancePath.path ---> assigned" << std::endl;
    // Set the timeout for 0.5 seconds (500 milliseconds)
//    auto timeout = std::chrono::milliseconds(500);

//    // Wait for the result or timeout
//    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, timeout) != rclcpp::FutureReturnCode::SUCCESS) {
//      result_msg = "Avoidance service call failed or timed out :(";
//      RCLCPP_ERROR_STREAM(this->get_logger(), result_msg.c_str());
//    } else {
//      auto result = result_future.get();
//      result_msg = "Avoidance service returned: " + std::to_string(result->res);
//      RCLCPP_INFO_STREAM(this->get_logger(), result_msg);
//      ctrlData_->avoidancePathGenerated = true;
//    }

    serviceAvailable = true;
    //StopOngoingTimers();
  } else {
    result_msg = "The Avoidance Node doesn't seem to be active.";
    serviceAvailable = false;
  }

  //ShowToast(result_msg.c_str(), 4000);
  //ctrlData_->avoidancePathGenerated = true;
  //std::cout << "End of sendLatLongAvoidanceCommand" << std::endl;
  return serviceAvailable;
}

void VehicleController::Run()
{
    if (boundariesSet_) {

        if (uFsm_.GetCurrentStateName() == ulisse::states::ID::rovfollow){
            commandRovFollowing_.SetGoTo(LatLong(rovData_->inertialF_linearPosition.latitude, rovData_->inertialF_linearPosition.longitude));
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
                std::cerr << "RUN" << std::endl;
                std::cerr << "UPDATE TASK EXCEPTION" << std::endl;
                std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
            }
        }

        // Computing Kinematic Control via TPIK
        yTpik_ = solver_->ComputeVelocities();


        for (int i = 0; i < yTpik_.size(); i++) {
            if (std::isnan(yTpik_(i))) {
                yTpik_(i) = 0.0;
                RCLCPP_INFO(this->get_logger(), "NaN requested velocity");
            }
        }
    }

    if(ctrlData_->avoidancePathEnabled ){//&& !sentReq){
        if(!ctrlData_->avoidancePathGenerated){
            std::cerr << "sentReq " << sentReq << std::endl;
            //ctb::LatLong inertialF_linearPositionCurrentGoal; double altitude;
            //Eigen::Vector3d pos;
            //pos.x()=-5.0;
            //pos.y()=-25.0;
            //pos.z()=0.0;
            //ctb::LocalUTM2LatLong(pos, centroidLocation_, inertialF_linearPositionCurrentGoal, altitude);
            bool request_result = sendLatLongAvoidanceCommand(ctrlData_->inertialF_linearPositionCurrentGoal, 4.0, 1.5, false);
            //bool request_result = sendLatLongAvoidanceCommand(inertialF_linearPositionCurrentGoal, 4.0, 1.5, false);
            std::cerr << "request_result = " << request_result << std::endl;
            ctrlData_->avoidancePathGenerated = true;
            sentReq = true;
        }
        ctrlData_->avoidancePath = aPath_;
        //sendLatLongAvoidanceCommand(ctrlData_->inertialF_linearPositionCurrentGoal, 4.0, 1.5, true);
    }
    else
        sentReq = false;

    // reference cable length
    //ComputeCableLength();

    //double rpm_percentage = 1;
    //CableWinchControl(rpm_percentage, controlledCable_.length);

    float cable_vel;
    ComputeCableVelocity(cableData_.released_cable_length, cable_vel);
    // reference cable velocity
    controlledCable_.velocity = cable_vel;

    tNow_ = std::chrono::system_clock::now();

    UpdateObstacles();
    //std::cout << "-- orig obs --"<< std::endl;
    //PrintObstacles(obstaclePointers_);
    //std::cout << "--task obs--"<< std::endl;
    //PrintObstacles(asvObstacleAvoidance_->Obstacles());
    //std::cout<< std::endl;

    //compute the heading error
    std::vector<Eigen::Vector3d> ObsDistance;
    asvObstacleAvoidance_->Update();
    //ObsDistance = asvObstacleAvoidance_->GetDistance(ulisse::robotModelID::ASV);
    //ObsDistance = asvObstacleAvoidance_->GetDistance(rml::FrameID::WorldFrame);

    //Eigen::Vector3d ObsDistance1 = ObsDistance[0];
    //double DistanceFromObs1 = ObsDistance1.norm();

    //double obstacleError = asvObstacleAvoidance_->ControlVariable().norm();
    //compute the gain of the cartesian distance
    //double maxObstacleError_ = asvObstacleAvoidance_->GreaterThanParams().xmax[0];
    //double minObstacleError_ = asvObstacleAvoidance_->GreaterThanParams().xmin[0];
    //double taskGain = rml::DecreasingBellShapedFunction(minObstacleError_, maxObstacleError_ , 0, 1.0, obstacleError);
    double taskGain = 1;
    asvObstacleAvoidance_->TaskParameter().gain = taskGain * asvObstacleAvoidance_->TaskParameter().conf_gain;

    //std::cout << "ObsDistance1_vector = "<< ObsDistance1<< std::endl;
    //std::cout << "DistanceFromObs1 = "<< DistanceFromObs1<< std::endl;
    //std::cout << "obstacleError = "<< obstacleError<< std::endl;
    //std::cout << "maxObstacleError_ = "<< maxObstacleError_<< std::endl;
    //std::cout << "minObstacleError_ = "<< minObstacleError_<< std::endl;
    //std::cout << "taskGain = "<< taskGain<< std::endl;
    //std::cout << "gain = "<< asvObstacleAvoidance_->TaskParameter().gain<< std::endl;

    //asvObstacleAvoidance_->Update();
    //Eigen::Vector3d vec(0.0, 5.0, 0.0);
    //std::cout << "vec= "<< vec<< std::endl;

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

        //if (uFsm_.GetCurrentStateName() == ulisse::states::ID::rovfollow) {
            controlledCable_.stamp.sec = now_stamp_secs;
            controlledCable_.stamp.nanosec = now_stamp_nanosecs;
            referenceCablePub_->publish(controlledCable_);

            winchMotorRef_.stamp.sec = now_stamp_secs;
            winchMotorRef_.stamp.nanosec = now_stamp_nanosecs;
            referenceWinchMotorPub_->publish(winchMotorRef_);


            plotVar_.heading_error_rov = stateRovFollowing_->headingError;
            plotVar_.goal_distance_rov = stateRovFollowing_->goalDistance;

            plotVar_.heading_error_obstacle = stateRovFollowing_->headingErrorAP;
            plotVar_.goal_distance_obstacle = stateRovFollowing_->goalDistanceAP;

            plotVar_.distance_rov_obstacle = stateRovFollowing_->ROV2obstacleDistance;


        //}
        plotVar_.stamp.sec = now_stamp_secs;
        plotVar_.stamp.nanosec = now_stamp_nanosecs;
        float dis;
        ComputeAsvRovDistance(dis);
        plotVar_.distance_rov_asv = dis;
        //plotVar_.distance_rov_asv = stateRovFollowing_->goalDistance;
        plotVarPub_->publish(plotVar_);
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
    } else if (uFsm_.GetCurrentStateName() == ulisse::states::ID::rovfollow) {
        feedbackGuiMsg.goal_position.latitude = stateRovFollowing_->goalPosition.latitude;
        feedbackGuiMsg.goal_position.longitude = stateRovFollowing_->goalPosition.longitude;
        feedbackGuiMsg.goal_heading = stateRovFollowing_->goalHeading;
        feedbackGuiMsg.acceptance_radius = stateRovFollowing_->acceptanceRadius;
        feedbackGuiMsg.goal_distance = stateRovFollowing_->goalDistance;
    }
    feedbackGuiPub_->publish(feedbackGuiMsg);
}
} // namespace ulisse
