#include "ulisse_avoidance/local_planner.hpp"

void LocalPlannerNode::handleComputePathRequest(
        [[maybe_unused]] std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<ulisse_msgs::srv::ComputeAvoidancePath::Request> request,
        std::shared_ptr<ulisse_msgs::srv::ComputeAvoidancePath::Response> response) {

    response->res = false;
    if (lastKnownVhStatus_ == ulisse::states::ID::halt) {
        RCLCPP_INFO(this->get_logger(), " VEHICLE NOT READY (HALT STATE) ");
        return;
    } 
    std::cout << "---------------------------------" << std::endl;
    RCLCPP_INFO(this->get_logger(), "New request.");
    if (followingPlan_) StopCheckingProgress(); // stop tracking old path

    Plan::vhData.velocities = generateRange(conf_.vhData.speedMin, request->latlong_cmd.ref_speed, conf_.vhData.speedStep);

    auto goal = ctb::LatLong(request->latlong_cmd.goal.latitude, request->latlong_cmd.goal.longitude);
    bool colregs = request->colregs_compliant;

    auto syncedObs = obstacles_;
    SyncObsToLastVhUpdate(syncedObs);

    plan = std::make_shared<Plan>(vh_position_, vh_heading_, goal, conf_.centroid, syncedObs, colregs, this->now(), debugSettings);
    if(plan->Report().result == oal::SearchResult::FAIL){
        RCLCPP_WARN(this->get_logger(), "Path computation failed: %s", plan->Report().failMsg.c_str());
        return;
    }

    if(plan->Report().result == oal::SearchResult::PARTIAL){
        RCLCPP_WARN(this->get_logger(), "Cannot reach goal, reaching one closer (%s)", plan->Report().failMsg.c_str());
    }

    if(!PathCmd(plan->GetPathMsg())) return;
    
    response->res = true;
    //last_check_progress_ = last_pos_update_;
    return;
}

void LocalPlannerNode::CheckProgress() {
    if(!followingPlan_) return;

    if (lastKnownVhStatus_ != ulisse::states::ID::pathfollow) {
        //RCLCPP_INFO(this->get_logger(), " VEHICLE NOT READY (HALT STATE) ");
        StopCheckingProgress();
    } else {
        // If new pos update exists, check progress
        if (lastPosUpdate_ > lastCheckProgress_) {
            // Check if it reached a new waypoint and check if path is still doable
            lastCheckProgress_ = lastPosUpdate_;

            auto syncedObs = obstacles_;
            SyncObsToLastVhUpdate(syncedObs);

            Eigen::Vector2d unreachableWaypoint;
            bool isOldSafe = plan->IsValid(vh_position_, vh_heading_, syncedObs, unreachableWaypoint);

            bool existBetterPlan = plan->ExistBetterPlan(conf_.betterPathDistancePerc, vh_position_, vh_heading_, syncedObs, plan->colregs, this->now());

            if(!isOldSafe && !existBetterPlan){
                RCLCPP_INFO(this->get_logger(), "Old path is not safe and no new path can be found.");
                StopCheckingProgress();
                HoldCmd();
                return;
            }

            if(existBetterPlan){
                PathCmd(plan->GetPathMsg());
                RCLCPP_INFO(this->get_logger(), "Switching to better path.");
            }
        }
    }
}

void LocalPlannerNode::VehicleStatusCB(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg) {
    lastKnownVhStatus_ = msg->vehicle_state;
}


void LocalPlannerNode::ObstacleCB(const detav_msgs::msg::ObstacleList::SharedPtr msg) {
    if(msg->obstacles.size() != obstacles_.size()) {
        RCLCPP_WARN(this->get_logger(), "Now tracking %ld instead of %ld obstacles!", msg->obstacles.size(), obstacles_.size());
    }

    lastObsUpdate_ = msg->header.stamp;
    obstacles_.clear(); 
    for( const auto& obs : msg->obstacles){
        // TODO Delete once Luca fixes the obsId
        std::string obsId = obs.id;
        if ( obsId.empty() || !std::all_of(obsId.begin(), obsId.end(), ::isalnum)) {
            obsId = "unknown" + std::to_string(obstacles_.size());
        }

        Eigen::Vector2d position = GetLocal(ctb::LatLong(obs.pose.position.position.latitude, obs.pose.position.position.longitude));
        double heading = M_PI_2 - obs.pose.orientation.yaw; //Heading so far refers to the north axis, but in obstacle ENU is used
        Eigen::Vector2d velocity = {obs.twist.twist.linear.x, obs.twist.twist.linear.y};

        oal::BoundingBoxData bbData = conf_.bbDataDefault;
        bbData.dim_x = obs.size.size.length;
        bbData.dim_y = obs.size.size.width;

        bbData.Set(
            {obs.twist.twist.linear.x, obs.twist.twist.linear.y}, //obs velocity
            obs.obs_class,
            obs.size.covariance[0], //size_x_sigma
            obs.size.covariance[1], //size_y_sigma
            obs.pose.position.north_cov, //pose_x_sigma
            obs.pose.position.east_cov, //pose_y_sigma
            obs.pose.orientation.covariance, //pose_yaw_sigma
            obs.twist.covariance[0], //vel_x_sigma
            obs.twist.covariance[1] //vel_y_sigma
        );

        auto obstacle = std::make_shared<oal::Obstacle>(obsId, obs.obs_class, oal::Pose(position, heading), velocity, bbData, GetLocal(vh_position_));
        obstacles_.push_back(obstacle);

        auto gui_msg = ulisse_msgs::msg::Obstacle();
        SetObsMsg(obstacle, gui_msg);
        obsGuiPub_->publish(gui_msg);
    }
}

void LocalPlannerNode::NavFilterCB(ulisse_msgs::msg::NavFilterData::SharedPtr msg) {
    rclcpp::Time msg_time(msg->stamp.sec, msg->stamp.nanosec);
    lastPosUpdate_ = msg_time;
    vh_position_ = ctb::LatLong(msg->inertialframe_linear_position.latlong.latitude,
                                msg->inertialframe_linear_position.latlong.longitude);
    // assuming given yaw is in NED coordinates
    vh_heading_ = M_PI / 2 - msg->bodyframe_angular_position.yaw;
    if(vh_heading_>M_PI) vh_heading_ -= 2*M_PI;
    if(vh_heading_<-M_PI) vh_heading_ += 2*M_PI;

    if(followingPlan_){
        if(plan->UpdateStatus(vh_position_, conf_.waypointAcceptanceRadius)){
            RCLCPP_INFO(this->get_logger(), "Reached end of path!");
            StopCheckingProgress();
            //HoldCmd();
        }
    }
}

Eigen::Vector2d LocalPlannerNode::GetLocal(ctb::LatLong LatLong, bool NED) const {
    Eigen::Vector3d pos;
    if (NED) {
        ctb::LatLong2LocalNED(LatLong, 0, conf_.centroid, pos);
    } else {
        ctb::LatLong2LocalUTM(LatLong, 0.0, conf_.centroid, pos);
    }
    return {pos.x(), pos.y()};
}

ctb::LatLong LocalPlannerNode::GetLatLong(Eigen::Vector2d pos_2d, bool NED) const {
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

void LocalPlannerNode::StatusPub() {
    auto msg = ulisse_msgs::msg::AvoidanceStatus();

    long now_stamp = (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_stamp / static_cast<int>(1E9));
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_stamp % static_cast<int>(1E9));
    msg.stamp.sec = now_stamp_secs;
    msg.stamp.nanosec = now_stamp_nanosecs;
    msg.n_known_obs = obstacles_.size();  

    if(!followingPlan_){
        msg.status = "Not active";
    }else{
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

void LocalPlannerNode::CoordinatesPub() {
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
    coordinatesPub_->publish(msg);
}

bool LocalPlannerNode::HaltCmd(){
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    RCLCPP_INFO(this->get_logger(), "--> Sending HALT command to KCL. ");
    serviceReq->command_type = ulisse::commands::ID::halt;
    return WaitForResponse(serviceReq);
}

bool LocalPlannerNode::HoldCmd(){
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    RCLCPP_INFO(this->get_logger(), "--> Sending HOLD command to KCL. ");
    serviceReq->command_type = ulisse::commands::ID::hold;
    //serviceReq->hold_cmd.acceptance_radius = conf_.waypointAcceptanceRadius;
    return WaitForResponse(serviceReq);
}

bool LocalPlannerNode::PathCmd(const ulisse_msgs::msg::PathData& path){
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    auto commandPathFollow = ulisse_msgs::msg::CommandPathFollow();
    commandPathFollow.path = path;
    serviceReq->command_type = ulisse::commands::ID::pathfollow;
    serviceReq->path_cmd = commandPathFollow;
    CoordinatesPub();
    StartCheckingProgress();
    return WaitForResponse(serviceReq);
}

bool LocalPlannerNode::WaitForResponse(std::shared_ptr<ulisse_msgs::srv::ControlCommand::Request>& serviceReq){
    static std::string result_msg;
    RCLCPP_INFO(this->get_logger(), "sending ctrl cmd: %s", serviceReq->command_type.c_str());
    if (command_srv_->service_is_ready()) {
        auto result_future = command_srv_->async_send_request(serviceReq);
        std::future_status status = result_future.wait_for(std::chrono::seconds(2));  // timeout to guarantee a graceful finish
        if (status != std::future_status::ready) {
            result_msg = " ---> service call failed.";
            RCLCPP_ERROR_STREAM(this->get_logger(), result_msg.c_str());
            StopCheckingProgress();
            return false;
        }
        auto result = result_future.get();
        result_msg = " ---> service returned: " + result->res;
        RCLCPP_INFO(this->get_logger(), "%s", result_msg.c_str());
        return true;
    } 
    RCLCPP_WARN(this->get_logger(), "The controller doesn't seem to be active (No CommandServer available)");
    StopCheckingProgress();
    return false;
}

void LocalPlannerNode::SyncObsToLastVhUpdate(std::vector<ObsPtr>& obstacles){
    try{

        auto lastPosTime = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(lastPosUpdate_.nanoseconds()));
        auto lastObsTime = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(lastObsUpdate_.nanoseconds()));

        auto diff = lastPosTime - lastObsTime; // This is now safe!
        auto diffSecondsDouble = std::chrono::duration<double>(diff);

        Eigen::Vector3d vhPositionUTM;
        ctb::LatLong2LocalUTM(vh_position_, 0.0, conf_.centroid, vhPositionUTM);

        for(auto& obs : obstacles){
            obs = std::make_shared<oal::Obstacle>(obs, diffSecondsDouble, vhPositionUTM.head(2));
        }
    } catch (const std::exception& e) {
        std::cerr << "Caught exception: " << e.what() << "\n";
    }
}

// Mind that timer starting and stopping are placed only in callbacks (single thread node)
void LocalPlannerNode::StartCheckingProgress() {
    followingPlan_ = true;
    checkProgressTimer_->reset();
}

void LocalPlannerNode::StopCheckingProgress() {
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


void LocalPlannerNode::SetObsMsg(const std::shared_ptr<oal::Obstacle>& obs, ulisse_msgs::msg::Obstacle &msg){
    ctb::LatLong pos = GetLatLong(obs->InitialPose().Position());
    msg.id = obs->Id();
    msg.center.latitude = pos.latitude;
    msg.center.longitude = pos.longitude;
    msg.heading = M_PI_2 - obs->InitialPose().Heading();
    msg.b_box_dim_x = obs->BBData().dim_x;
    msg.b_box_dim_y = obs->BBData().dim_y;
    msg.show_bbs = debugSettings->obsBbsInGui;
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

void LocalPlannerNode::TopicSetup(){
    nested_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // KCL cmd service client
    command_srv_ = create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service, rmw_qos_profile_services_default,
                    nested_cb_group_);
    // Avoidance cmd service server
    compute_path_service_ = create_service<ulisse_msgs::srv::ComputeAvoidancePath>
                            (ulisse_msgs::topicnames::control_avoidance_cmd_service,
                            std::bind(&LocalPlannerNode::handleComputePathRequest, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                            rmw_qos_profile_services_default);
    // SUBSCRIBING
    //  Nav Filter sub
    navFilter_subscription_ = create_subscription<ulisse_msgs::msg::NavFilterData>
                            (ulisse_msgs::topicnames::nav_filter_data, qos_sensor,
                            std::bind(&LocalPlannerNode::NavFilterCB, this, std::placeholders::_1));
    //  Obs sub
    obs_subscription_ = create_subscription<detav_msgs::msg::ObstacleList>(
                        detav_msgs::topicnames::obstacles, qos_sensor,
                        std::bind(&LocalPlannerNode::ObstacleCB, this, std::placeholders::_1));

    //  Ownship status sub
    vhStatus_subscription_ = create_subscription<ulisse_msgs::msg::VehicleStatus>(
                            ulisse_msgs::topicnames::vehicle_status, qos_sensor,
                            std::bind(&LocalPlannerNode::VehicleStatusCB, this, std::placeholders::_1));

    // PUBLISHING
    //  Avoidance status pub
    avoidanceStatusPub_ = create_publisher<ulisse_msgs::msg::AvoidanceStatus>(
                            ulisse_msgs::topicnames::avoidance_status, qos_sensor);
    //  Coordinates pub
    coordinatesPub_ = create_publisher<ulisse_msgs::msg::CoordinateList>(ulisse_msgs::topicnames::avoidance_path,
                        qos_sensor);
    compPathPub_ = create_publisher<ulisse_msgs::msg::AvoidancePath>(
                        ulisse_msgs::topicnames::avoidance_path_oal, qos_sensor);

    obsGuiPub_ = create_publisher<ulisse_msgs::msg::Obstacle>(
                ulisse_msgs::topicnames::obstacle, qos_sensor);

}

void LocalPlannerNode::LoadConf() {
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
    planning.lookupValue("status_pub_rate", conf_.statusPubRate);
    planning.lookupValue("better_path_distance_perc", conf_.betterPathDistancePerc);

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
    std::string logNodesPathDir;
    debug.lookupValue("log_nodes", logNodes);
    debug.lookupValue("log_nodes_path_dir", logNodesPathDir);
    if(logNodes){
        debugSettings->oal->completePath.log = true;
        debugSettings->oal->completePath.dirPath = logNodesPathDir;
        debugSettings->oal->completePath.fileName = "completePathNodes.txt";
        debugSettings->oal->failedPath.log = true;
        debugSettings->oal->failedPath.dirPath = logNodesPathDir;
        debugSettings->oal->failedPath.fileName = "failedPathNodes.txt";
        debugSettings->oal->validPath.log = true;
        debugSettings->oal->validPath.dirPath = logNodesPathDir;
        debugSettings->oal->validPath.fileName = "validPathNodes.txt";
        debugSettings->oal->notValidPath.log = true;
        debugSettings->oal->notValidPath.dirPath = logNodesPathDir;
        debugSettings->oal->notValidPath.fileName = "notValidPathNodes.txt";
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
            conf_.vhData.maxYawRate == 0 ? "instantaneously" : ("at " + std::to_string(conf_.vhData.maxYawRate) + " rad/s").c_str()
        );

        RCLCPP_INFO(this->get_logger(),
                    "  - Path progress check every %.2fs, Waypoint acceptance radius: %.2fm",
                    conf_.checkProgressRate, conf_.waypointAcceptanceRadius);
        RCLCPP_INFO(this->get_logger(),
                    "  - New path if %.2f%% shorter, Pub avoidance update every %.2fs",
                    conf_.betterPathDistancePerc * 100, conf_.statusPubRate);
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


