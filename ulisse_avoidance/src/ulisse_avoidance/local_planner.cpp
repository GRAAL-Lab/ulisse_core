#include "ulisse_avoidance/local_planner.hpp"

void LocalPlannerNode::handlePlanRequest(
    [[maybe_unused]] std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<ulisse_msgs::srv::ComputeAvoidancePath::Request> request,
    std::shared_ptr<ulisse_msgs::srv::ComputeAvoidancePath::Response> response)
{

    response->res = false;
    if (lastKnownVhStatus_ == ulisse::states::ID::halt) {
        RCLCPP_INFO(this->get_logger(), " VEHICLE NOT READY (HALT STATE) ");
        return;
    }
    std::cout << "---------------------------------" << std::endl;
    RCLCPP_INFO(this->get_logger(), "New request.");
    if (followingPlan_) {
        StopCheckingProgress(); // stop tracking old path
    }

    Plan::vhData.velocities = generateRange(conf_.vhData.speedMin, request->latlong_cmd.ref_speed, conf_.vhData.speedStep);
    // std::cerr << "Speeds: ";
    // for (const auto& v : Plan::vhData.velocities) {
    //     std::cerr << v << " ";
    // }
    // std::cerr << std::endl;

    currentGoal_ = ctb::LatLong(request->latlong_cmd.goal.latitude, request->latlong_cmd.goal.longitude);
    bool colregs = request->colregs_compliant;

    // auto syncedObs = obstacles_;
    // SyncObsToLastVhUpdate(syncedObs);
    SyncObsToLastVhUpdate();

    plan = std::make_shared<Plan>(vh_position_, vh_heading_, currentGoal_, conf_.centroid, obstacles_, colregs,
        this->now(), debugSettings->logDirectory, debugSettings);

    if (plan->Report().result == oal::SearchResult::FAIL) {
        RCLCPP_WARN(this->get_logger(), "Path computation failed: %s", plan->Report().failMsg.c_str());
        HoldCmd();
        return;
    }

    if (plan->Report().result == oal::SearchResult::PARTIAL)
        RCLCPP_WARN(this->get_logger(), "Cannot reach goal, reaching one closer (%s)", plan->Report().failMsg.c_str());

    if (!PathCmd(plan->GetPathMsg()))
        return;

    response->res = true;
    // last_check_progress_ = last_pos_update_;
    return;
}

void LocalPlannerNode::CheckProgress()
{
    if (!followingPlan_)
        return;

    if (lastKnownVhStatus_ != ulisse::states::ID::pathfollow) {
        // RCLCPP_INFO(this->get_logger(), " VEHICLE NOT READY (HALT STATE) ");
        StopCheckingProgress();
    } else {
        // If new pos update exists, check progress
        if (lastPosUpdate_ > lastCheckProgress_) {
            // Check if it reached a new waypoint and check if path is still doable
            lastCheckProgress_ = lastPosUpdate_;

            // auto syncedObs = obstacles_;
            // SyncObsToLastVhUpdate(syncedObs);
            SyncObsToLastVhUpdate();

            auto out = plan->HandleEnvironmentDifferences(vh_position_, vh_heading_, obstacles_, conf_.betterPathDistancePerc, this->now());
            if (out == pathProgress::interruptedPlan) {
                StopCheckingProgress();
                HoldCmd();
                RCLCPP_WARN(this->get_logger(), "Old path is not safe and no new path can be found.");
                return;
            } else if (out == pathProgress::switchingToBetterPlan) {
                RCLCPP_INFO(this->get_logger(), "Switching to better path.");
                PathCmd(plan->GetPathMsg());
                return;
            } else if (out == pathProgress::switchingToSaferPlan) {
                RCLCPP_INFO(this->get_logger(), "Switching to safer path.");
                PathCmd(plan->GetPathMsg());
                return;
            } else if (out == pathProgress::keepingSamePath) {
                return;
            }
        }
    }
}

void LocalPlannerNode::VehicleStatusCB(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg)
{
    lastKnownVhStatus_ = msg->vehicle_state;
}

void LocalPlannerNode::ObstaclesDetectionCB(const detav_msgs::msg::ObstacleList::SharedPtr msg)
{
    if (msg->obstacles.size() != obstacles_.size()) {
        RCLCPP_WARN(this->get_logger(), "Now tracking %ld instead of %ld obstacles!", msg->obstacles.size(), obstacles_.size());
    }

    rclcpp::Time newObsMsgTime = msg->header.stamp;
    // if (newObsMsgTime.nanoseconds() < lastObsUpdate_.nanoseconds()) {
    //     RCLCPP_WARN(this->get_logger(), "Received an old obstacle message: Last prediction is more recent than last detection.");
    // }

    lastObsUpdate_ = newObsMsgTime;
    obstacles_.clear();
    for (const auto& obs : msg->obstacles) {
        // TODO Delete once Luca fixes the obsId
        std::string obsId = obs.id;
        // if ( obsId.empty() || !std::all_of(obsId.begin(), obsId.end(), ::isalnum)) {
        //     obsId = "unknown" + std::to_string(obstacles_.size());
        // }

        Eigen::Vector2d position = GetLocal(ctb::LatLong(obs.pose.position.position.latitude, obs.pose.position.position.longitude));
        double heading = M_PI_2 - obs.pose.orientation.yaw; // Heading so far refers to the north axis, but in obstacle ENU is used
        Eigen::Vector2d velocity = { obs.twist.twist.linear.y, -obs.twist.twist.linear.x }; // ENU to NED
        if(velocity.norm() <= conf_.staticObsMaxSpeed){
            velocity = {0, 0};
            RCLCPP_WARN(this->get_logger(), "Static obstacle with id %s have velocity norm of %.2f m/s, setting it to 0.",
                obsId.c_str(), velocity.norm());
        }

        oal::BoundingBoxData bbData = conf_.bbDataDefault;
        bbData.dim_x = obs.size.size.length;
        bbData.dim_y = obs.size.size.width;

        bbData.Set(
            { obs.twist.twist.linear.x, obs.twist.twist.linear.y }, // obs velocity
            obs.obs_class,
            obs.size.covariance[0], // size_x_sigma
            obs.size.covariance[1], // size_y_sigma
            obs.pose.position.north_cov, // pose_x_sigma
            obs.pose.position.east_cov, // pose_y_sigma
            obs.pose.orientation.covariance, // pose_yaw_sigma
            obs.twist.covariance[0], // vel_x_sigma
            obs.twist.covariance[1] // vel_y_sigma
        );

        auto obstacle = std::make_shared<oal::Obstacle>(obsId, obs.obs_class, oal::Pose(position, heading), velocity, bbData, GetLocal(vh_position_));
        obstacles_.push_back(obstacle);
    }
}

void LocalPlannerNode::GuiObsPub()
{
    for (const auto& obs : obstacles_) {
        auto gui_msg = ulisse_msgs::msg::Obstacle();
        SetObsMsg(obs, gui_msg);
        guiObsPub_->publish(gui_msg);
    }
}

void LocalPlannerNode::NavFilterCB(ulisse_msgs::msg::NavFilterData::SharedPtr msg)
{
    rclcpp::Time msg_time(msg->stamp.sec, msg->stamp.nanosec);
    lastPosUpdate_ = msg_time;
    vh_position_ = ctb::LatLong(msg->inertialframe_linear_position.latlong.latitude,
        msg->inertialframe_linear_position.latlong.longitude);
    // assuming given yaw is in NED coordinates
    vh_heading_ = M_PI / 2 - msg->bodyframe_angular_position.yaw;
    if (vh_heading_ > M_PI)
        vh_heading_ -= 2 * M_PI;
    if (vh_heading_ < -M_PI)
        vh_heading_ += 2 * M_PI;

    if (followingPlan_) {
        if (plan->UpdateStatus(vh_position_, conf_.waypointAcceptanceRadius)) {
            RCLCPP_INFO(this->get_logger(), "Reached end of path!");
            StopCheckingProgress();
            // HoldCmd();
        }
    }
}

Eigen::Vector2d LocalPlannerNode::GetLocal(ctb::LatLong LatLong, bool NED) const
{
    Eigen::Vector3d pos;
    if (NED) {
        ctb::LatLong2LocalNED(LatLong, 0, conf_.centroid, pos);
    } else {
        ctb::LatLong2LocalUTM(LatLong, 0.0, conf_.centroid, pos);
    }
    return { pos.x(), pos.y() };
}

ctb::LatLong LocalPlannerNode::GetLatLong(Eigen::Vector2d pos_2d, bool NED) const
{
    Eigen::Vector3d pos(pos_2d.x(), pos_2d.y(), 0);
    double alt;
    ctb::LatLong LatLong;
    if (NED) {
        ctb::LocalNED2LatLong(pos, conf_.centroid, LatLong, alt);
    } else {
        ctb::LocalUTM2LatLong(pos, conf_.centroid, LatLong, alt);
    }
    return LatLong;
}

void LocalPlannerNode::AvoidanceStatusPub()
{
    auto msg = ulisse_msgs::msg::AvoidanceStatus();

    long now_stamp = (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_stamp / static_cast<int>(1E9));
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_stamp % static_cast<int>(1E9));
    msg.stamp.sec = now_stamp_secs;
    msg.stamp.nanosec = now_stamp_nanosecs;
    msg.n_known_obs = obstacles_.size();

    if (!followingPlan_) {
        msg.status = "Not active";
    } else {
        msg.status = "Active";
        msg.path_change = lastPathChangeReason_;
        lastPathChangeReason_ = "";
        msg.colregs_compliant = plan->colregs;
        msg.desired_speed = plan->path.Data().front()->data.approachingSpeed;
        msg.current_path_creation.sec = plan->When().seconds();
        msg.current_path_creation.nanosec = plan->When().nanoseconds();
    }
    avoidanceStatusPub_->publish(msg);
}

void LocalPlannerNode::GuiPathPub()
{
    auto msg = ulisse_msgs::msg::CoordinateList();
    Path path_temp = plan->path;
    msg.id = "Path";

    auto latlong = ulisse_msgs::msg::LatLong();
    latlong.latitude = vh_position_.latitude;
    latlong.longitude = vh_position_.longitude;
    msg.coordinates.push_back(latlong);

    while (!path_temp.Data().empty()) {
        ctb::LatLong pos = GetLatLong(path_temp.Data().front()->data.position);
        path_temp.Data().pop_front();

        latlong.longitude = pos.longitude;
        latlong.latitude = pos.latitude;
        msg.coordinates.push_back(latlong);
    }
    guiPathPub_->publish(msg);
}

bool LocalPlannerNode::HaltCmd()
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    RCLCPP_INFO(this->get_logger(), "--> Sending HALT command to KCL. ");
    serviceReq->command_type = ulisse::commands::ID::halt;
    return WaitForResponse(serviceReq);
}

bool LocalPlannerNode::HoldCmd()
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    RCLCPP_INFO(this->get_logger(), "--> Sending HOLD command to KCL. ");
    serviceReq->command_type = ulisse::commands::ID::hold;
    // serviceReq->hold_cmd.acceptance_radius = conf_.waypointAcceptanceRadius;
    return WaitForResponse(serviceReq);
}

bool LocalPlannerNode::PathCmd(const ulisse_msgs::msg::PathData& path)
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    auto commandPathFollow = ulisse_msgs::msg::CommandPathFollow();
    commandPathFollow.path = path;
    serviceReq->command_type = ulisse::commands::ID::pathfollow;
    serviceReq->path_cmd = commandPathFollow;
    GuiPathPub();
    StartCheckingProgress();
    return WaitForResponse(serviceReq);
}

// void LocalPlannerNode::Gui

bool LocalPlannerNode::WaitForResponse(std::shared_ptr<ulisse_msgs::srv::ControlCommand::Request>& serviceReq)
{
    // static std::string result_msg;
    // RCLCPP_INFO(this->get_logger(), "sending ctrl cmd: %s", serviceReq->command_type.c_str());
    if (kclCmdSrv_->service_is_ready()) {
        auto result_future = kclCmdSrv_->async_send_request(serviceReq);
        std::future_status status = result_future.wait_for(std::chrono::seconds(2)); // timeout to guarantee a graceful finish
        if (status != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Failed ctrl cmd (%s)", serviceReq->command_type.c_str());
            StopCheckingProgress();
            return false;
        }
        auto result = result_future.get();
        // result_msg = " ---> service returned: " + result->res;
        // RCLCPP_INFO(this->get_logger(), "%s", result_msg.c_str());
        return true;
    }
    RCLCPP_WARN(this->get_logger(), "The controller doesn't seem to be active.");
    StopCheckingProgress();
    return false;
}

void LocalPlannerNode::SyncObsToLastVhUpdate()
{
    try {

        auto lastPosTime = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(lastPosUpdate_.nanoseconds()));
        auto lastObsTime = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(lastObsUpdate_.nanoseconds()));

        auto diff = lastPosTime - lastObsTime;
        auto diffSecondsDouble = std::chrono::duration<double>(diff);

        Eigen::Vector3d vhPositionUTM;
        ctb::LatLong2LocalUTM(vh_position_, 0.0, conf_.centroid, vhPositionUTM);

        // WATCH OUT! I'M MODIFYING OBSTACLES_
        for (auto& obs : obstacles_) {
            obs = std::make_shared<oal::Obstacle>(obs, diffSecondsDouble, vhPositionUTM.head(2));
        }

        lastObsUpdate_ = lastPosUpdate_;

    } catch (const std::exception& e) {
        std::cerr << "Caught exception: " << e.what() << "\n";
    }
}

// Mind that timer starting and stopping are placed only in callbacks (single thread node)
void LocalPlannerNode::StartCheckingProgress()
{
    followingPlan_ = true;
    checkProgressTimer_->reset();
}

void LocalPlannerNode::StopCheckingProgress()
{
    checkProgressTimer_->cancel();
    followingPlan_ = false;
    actual_path_.push_back(vh_position_);
    // 	if(debugSettings.printActualPath){
    // 		RCLCPP_INFO(this->get_logger(), "Last path actual waypoints:");
    // 		for (const ctb::LatLong& pos : actual_path_) {
    // 			RCLCPP_INFO(this->get_logger(), "    - %.8f,   %.8f", pos.latitude, pos.longitude);
    // 		}
    // 	}
    actual_path_.clear();
}

void LocalPlannerNode::SetObsMsg(const std::shared_ptr<oal::Obstacle>& obs, ulisse_msgs::msg::Obstacle& msg)
{
    ctb::LatLong pos = GetLatLong(obs->InitialPose().Position());
    msg.id = obs->Id();
    msg.center.latitude = pos.latitude;
    msg.center.longitude = pos.longitude;
    msg.heading = M_PI_2 - obs->InitialPose().Heading();
    msg.b_box_dim_x = obs->BBData().dim_x;
    msg.b_box_dim_y = obs->BBData().dim_y;
    msg.show_bbs = debugSettings->obsBbsInGui;
    msg.show_id = true;
    msg.bb_max.x_bow = obs->BBData().max_x_bow;
    msg.bb_max.x_stern = obs->BBData().max_x_stern;
    msg.bb_max.y_starboard = obs->BBData().max_y_starboard;
    msg.bb_max.y_port = obs->BBData().max_y_port;
    msg.bb_safe.x_bow = obs->BBData().safety_x_bow;
    msg.bb_safe.x_stern = obs->BBData().safety_x_stern;
    msg.bb_safe.y_starboard = obs->BBData().safety_y_starboard;
    msg.bb_safe.y_port = obs->BBData().safety_y_port;
    msg.uncertainty_gap = obs->BBData().reductionWhileCheckingPath;
    msg.vel_x = -1;
    msg.vel_y = -1;
}

void LocalPlannerNode::TopicSetup()
{
    nestedCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // KCL cmd service client
    kclCmdSrv_ = create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service, rmw_qos_profile_services_default,
        nestedCbGroup_);
    // Avoidance cmd service server
    computePathSrv_ = create_service<ulisse_msgs::srv::ComputeAvoidancePath>(ulisse_msgs::topicnames::control_avoidance_cmd_service,
        std::bind(&LocalPlannerNode::handlePlanRequest, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
        rmw_qos_profile_services_default);
    // SUBSCRIBING
    //  Nav Filter sub
    navFilterSub_ = create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, qos_sensor,
        std::bind(&LocalPlannerNode::NavFilterCB, this, std::placeholders::_1));
    //  Obs sub
    obsDtcSub_ = create_subscription<detav_msgs::msg::ObstacleList>(
        detav_msgs::topicnames::obstacles, qos_sensor,
        std::bind(&LocalPlannerNode::ObstaclesDetectionCB, this, std::placeholders::_1));

    //  Ownship status sub
    vhStatusSub_ = create_subscription<ulisse_msgs::msg::VehicleStatus>(
        ulisse_msgs::topicnames::vehicle_status, qos_sensor,
        std::bind(&LocalPlannerNode::VehicleStatusCB, this, std::placeholders::_1));

    // PUBLISHING
    //  Avoidance status pub
    avoidanceStatusPub_ = create_publisher<ulisse_msgs::msg::AvoidanceStatus>(
        ulisse_msgs::topicnames::avoidance_status, qos_sensor);
    //  Coordinates pub
    guiPathPub_ = create_publisher<ulisse_msgs::msg::CoordinateList>(ulisse_msgs::topicnames::avoidance_path,
        qos_sensor);
    avoidancePathPub_ = create_publisher<ulisse_msgs::msg::AvoidancePath>(
        ulisse_msgs::topicnames::avoidance_path_oal, qos_sensor);

    guiObsPub_ = create_publisher<ulisse_msgs::msg::Obstacle>(
        ulisse_msgs::topicnames::obstacle, qos_sensor);
}

void LocalPlannerNode::LoadConf()
{
    libconfig::Config cfg_;
    std::string ulisse_avoidance_dir = ament_index_cpp::get_package_share_directory("ulisse_avoidance");
    std::string ulisse_avoidance_dir_conf = ulisse_avoidance_dir + "/conf/configuration.cfg";
    cfg_.readFile(ulisse_avoidance_dir_conf.c_str());
    RCLCPP_INFO(this->get_logger(), "Loading configuration file:\n      %s", ulisse_avoidance_dir_conf.c_str());

    // Load vehicle data
    const libconfig::Setting& vehicle_data = cfg_.getRoot()["vehicle_data"];
    vehicle_data.lookupValue("rotational_speed", conf_.vhData.maxYawRate);
    vehicle_data.lookupValue("starting_velocity", conf_.vhData.speedMin);
    vehicle_data.lookupValue("velocity_step", conf_.vhData.speedStep);
    Plan::vhData.max_yaw_rate = conf_.vhData.maxYawRate;

    // Load bounding box data
    const libconfig::Setting& bounding_box = cfg_.getRoot()["bounding_box"];
    bounding_box.lookupValue("gain", conf_.bbDataDefault.gain);
    bounding_box.lookupValue("min_distance_from_obs", conf_.bbDataDefault.minDistFromObs);
    bounding_box.lookupValue("reduction_while_checking_path", conf_.bbDataDefault.reductionWhileCheckingPath);
    bounding_box.lookupValue("safe_max_gap", conf_.bbDataDefault.safeMaxGap);
    bounding_box.lookupValue("look_ahead_safety_span", conf_.bbDataDefault.lookAheadSafetySpan);

    // Load planning parameters
    const libconfig::Setting& planning = cfg_.getRoot()["planning"];
    planning.lookupValue("only_once_on_same_vx", conf_.searchTreePruningParams.onlyOnceOnSameVx);
    planning.lookupValue("stop_search_if_goal_in_bb", conf_.searchTreePruningParams.stopSearchIfGoalInBB);
    planning.lookupValue("same_position_threshold", conf_.searchTreePruningParams.samePositionThreshold);
    planning.lookupValue("same_time_threshold", conf_.searchTreePruningParams.sameTimeThreshold);
    planning.lookupValue("waypoint_acceptance_radius", conf_.waypointAcceptanceRadius);
    planning.lookupValue("check_progress_rate", conf_.checkProgressRate);
    planning.lookupValue("status_pub_rate", conf_.avoidanceStatusPubRate);
    planning.lookupValue("gui_obs_pub_rate", conf_.guiObsPubRate);
    planning.lookupValue("better_path_distance_perc", conf_.betterPathDistancePerc);
    planning.lookupValue("search_time_out", conf_.searchTreePruningParams.timeOut);
    planning.lookupValue("static_obs_max_speed", conf_.staticObsMaxSpeed);
    Plan::pParams = conf_.searchTreePruningParams;


    // Load centroid
    const libconfig::Setting& centroid = cfg_.getRoot()["centroid"];
    centroid.lookupValue("latitude", conf_.centroid.latitude);
    centroid.lookupValue("longitude", conf_.centroid.longitude);

    // Load debug
    const libconfig::Setting& debug = cfg_.getRoot()["debug"];
    debugSettings = std::make_shared<DebugUA>();
    debugSettings->oal = std::make_shared<oal::DebugSettings>();
    bool print;
    debug.lookupValue("print_conf", print);
    bool logNodes = false;
    std::string logNodesPathDir, logObsPathDir;
    debug.lookupValue("log_nodes", logNodes);
    debug.lookupValue("log_nodes_path_dir", logNodesPathDir);
    debug.lookupValue("log_obstacles_path_dir", logObsPathDir);
    if (logNodes) {
        debugSettings->oal->completePathNodesLog.log = true;
        debugSettings->oal->completePathNodesLog.dirPath = logNodesPathDir;
        debugSettings->oal->completePathNodesLog.fileName = "completePathNodes.txt";
        debugSettings->oal->failedPathNodesLog.log = true;
        debugSettings->oal->failedPathNodesLog.dirPath = logNodesPathDir;
        debugSettings->oal->failedPathNodesLog.fileName = "failedPathNodes.txt";
        debugSettings->oal->validPathNodesLog.log = true;
        debugSettings->oal->validPathNodesLog.dirPath = logNodesPathDir;
        debugSettings->oal->validPathNodesLog.fileName = "validPathNodes.txt";
        debugSettings->oal->notValidPathNodesLog.log = true;
        debugSettings->oal->notValidPathNodesLog.dirPath = logNodesPathDir;
        debugSettings->oal->notValidPathNodesLog.fileName = "notValidPathNodes.txt";

        debugSettings->oal->obstaclesLog.log = true;
        debugSettings->oal->obstaclesLog.dirPath = logObsPathDir;
        debugSettings->oal->obstaclesLog.fileName = "obstacles.txt";
    }

    debug.lookupValue("log_obs", debugSettings->oal->logObstacles);
    debug.lookupValue("log_obs_path_file", debugSettings->oal->logObstaclesPathFile);
    debug.lookupValue("obs_bbs_in_gui", debugSettings->obsBbsInGui);

    
    

    if (print) {
        RCLCPP_INFO(this->get_logger(), "Loaded configuration:");
        RCLCPP_INFO(this->get_logger(), "  - Centroid: { %.8f, %.8f }", conf_.centroid.latitude, conf_.centroid.longitude);

        RCLCPP_INFO(
            this->get_logger(),
            "  - Considering the vehicle turns %s",
            conf_.vhData.maxYawRate == 0 ? "instantaneously" : ("at " + std::to_string(conf_.vhData.maxYawRate) + " rad/s").c_str());

        RCLCPP_INFO(this->get_logger(),
            "  - Path progress check every %.2fs, Waypoint acceptance radius: %.2fm",
            conf_.checkProgressRate, conf_.waypointAcceptanceRadius);
        RCLCPP_INFO(this->get_logger(),
            "  - New path if shorter than %.2f%% of previous one, Pub avoidance update every %.2fs",
            conf_.betterPathDistancePerc * 100, conf_.avoidanceStatusPubRate);
        RCLCPP_INFO(this->get_logger(),
            "  - Min speed: %.2f, Speed step: %.2f, BB uncertainty multiplier: %.2f",
            conf_.vhData.speedMin, conf_.vhData.speedStep, conf_.bbDataDefault.gain);

        RCLCPP_INFO(this->get_logger(),
            "  - Pruning Parameters: Only once on same Vx: %s, Stop search if goal in BB: %s",
            conf_.searchTreePruningParams.onlyOnceOnSameVx ? "true" : "false",
            conf_.searchTreePruningParams.stopSearchIfGoalInBB ? "true" : "false");
        RCLCPP_INFO(this->get_logger(),
            "  - Same position threshold: %.2f, Same time threshold: %.2f",
            conf_.searchTreePruningParams.samePositionThreshold,
            conf_.searchTreePruningParams.sameTimeThreshold);
    }
}

json LocalPlannerNode::LogSearchStatus()
{
    // Data to Log
    // auto syncedObs = obstacles_;
    // SyncObsToLastVhUpdate(syncedObs);
    Eigen::Vector3d vhPositionUTM, goalUTM;
    ctb::LatLong2LocalUTM(vh_position_, 0.0, conf_.centroid, vhPositionUTM);
    ctb::LatLong2LocalUTM(currentGoal_, 0.0, conf_.centroid, goalUTM);

    json statusLog; // Create a JSON object for the log

    // Basic Plan information
    statusLog["Goal"] = { { "x", goalUTM.x() }, { "y", goalUTM.y() } };
    statusLog["VehiclePose"] = { { "x", vhPositionUTM.x() },
        { "y", vhPositionUTM.y() },
        { "heading", vh_heading_ },
        { "timestamp", lastPosUpdate_.nanoseconds() } };

    statusLog["VehicleData"] = { { "Velocities", Plan::vhData.velocities },
        { "MaxYawRate", conf_.vhData.maxYawRate } };

    statusLog["PruningParams"] = { { "OnlyOnceOnSameVx", conf_.searchTreePruningParams.onlyOnceOnSameVx },
        { "StopSearchIfGoalInBB", conf_.searchTreePruningParams.stopSearchIfGoalInBB },
        { "SamePositionThreshold", conf_.searchTreePruningParams.samePositionThreshold },
        { "SameTimeThreshold", conf_.searchTreePruningParams.sameTimeThreshold } };

    // Obstacles
    json obstaclesLog = json::array();
    for (const auto& obs : obstacles_) {
        obstaclesLog.push_back(obs->Log());
    }
    statusLog["Obstacles"] = obstaclesLog;

    // Colregs information
    // statusLog["Colregs"] = colregs;

    return statusLog;
}