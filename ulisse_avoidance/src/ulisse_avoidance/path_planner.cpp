// #include "ulisse_avoidance/path_planner.hpp"
// #include "oal/path_planner.hpp"

// void PathPlannerNode::handleComputePathRequest(
//         [[maybe_unused]] std::shared_ptr<rmw_request_id_t> request_header,
//         std::shared_ptr<ulisse_msgs::srv::ComputeAvoidancePath::Request> request,
//         std::shared_ptr<ulisse_msgs::srv::ComputeAvoidancePath::Response> response) {

//     if (last_known_vhStatus_ == ulisse::states::ID::halt) {
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " VEHICLE NOT READY (HALT STATE) ");
//         return;
//     } 

//     std::cout << "---------------------------------" << std::endl;
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New request.");
//     if (active_) stopTracking(); // stop tracking old path


//     velocities_ = generateRange(conf_.speed_min, request->latlong_cmd.ref_speed, conf_.speed_step);
//     goal_ = ctb::LatLong(request->latlong_cmd.goal.latitude, request->latlong_cmd.goal.longitude);
//     colregs_ = request->colregs_compliant;
//     radius_ = request->latlong_cmd.acceptance_radius; 
    
//     try {
//         if (ComputePath(path_, last_path_creation_time)) {
//             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Path found:");
//             printPath();
//             last_path_computation_ = rclcpp::Clock().now();
//             CoordinatesPub();
//             sendNew = true;
//             response->res = true;
//             last_check_progress_ = last_pos_update_;
//             last_path_change_reason_ = "new_request";
//             startTracking();
//             return;
//         }
//     } catch (const std::bad_alloc& e) {
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Memory allocation failed in ComputePath: %s", e.what());
//     } catch (const std::length_error& e) {
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Length error in ComputePath: %s", e.what());
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception occurred in ComputePath: %s", e.what());
//     } catch (...) {
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown exception occurred in ComputePath");
//     }
//     response->res = false;

// }

// bool PathPlannerNode::CallKCL(const std::string &cmd_type) {
//     auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
//     if (cmd_type == ulisse::commands::ID::halt) {
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Sending HALT command to KCL. ");
//         serviceReq->command_type = ulisse::commands::ID::halt;
//     } else {
//         if (cmd_type == ulisse::commands::ID::hold) {
//             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Sending HOLD command to KCL. ");
//             serviceReq->command_type = ulisse::commands::ID::hold;
//             serviceReq->latlong_cmd.acceptance_radius = radius_;
//         } else {
//             // Get point
//             Path path = path_;
//             path.waypoints.pop();
//             ctb::LatLong goal = GetLatLong(path.waypoints.top().position);
//             serviceReq->command_type = ulisse::commands::ID::latlong;
//             serviceReq->latlong_cmd.goal.latitude = goal.latitude;
//             serviceReq->latlong_cmd.goal.longitude = goal.longitude;
//             serviceReq->latlong_cmd.ref_speed = path.waypoints.top().speed_to_it;
//             std::cerr<<" sending cmd with speed: "<< serviceReq->latlong_cmd.ref_speed<<"\n";
//             last_cmd_speed = path.waypoints.top().speed_to_it;
            
//             if (path.size() == 1) {
//                 // Last wp
//                 serviceReq->latlong_cmd.acceptance_radius = radius_;
//             } else {
//                 serviceReq->latlong_cmd.acceptance_radius = 0.001;  // Let this node decide when inner waypoints have been reached and not KCL
//             }
//         }
//     }
//     static std::string result_msg;
//     if (command_srv_->service_is_ready()) {
//         auto result_future = command_srv_->async_send_request(serviceReq);
//         std::future_status status = result_future.wait_for(std::chrono::seconds(2));  // timeout to guarantee a graceful finish
//         if (status == std::future_status::ready) {
//             auto result = result_future.get();
//             result_msg = "Service returned: " + result->res;
//             //std::cout << result_msg << std::endl;
//           RCLCPP_INFO(this->get_logger(), "%s", result_msg.c_str());
//         } else {
//             result_msg = "Service call failed :(";
//             RCLCPP_ERROR_STREAM(this->get_logger(), result_msg.c_str());
//             stopTracking();
//             return false;
//         }
//         return true;
//     } else {
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
//                     "The controller doesn't seem to be active.\n(No CommandServer available)");
//         return false;
//     }
// }

// bool PathPlannerNode::ComputePath(Path &path, rclcpp::Time& creation_time) {
//     VehicleInfo v_info({GetLocal(vh_position_), velocities_, vh_heading_, conf_.rotational_speed});
//     Eigen::Vector2d goal = GetLocal(goal_);

//     // Sync vh_data and obs data
//     oal::PathPlanner planner;
//     std::vector<Obstacle> obstacles;
//     planner.SetVhData(v_info);
//     planner.SetAccRadius(conf_.waypoint_acceptance_radius);

//     SyncObssData(last_pos_update_, obstacles);
//     planner.SetObssData(obstacles);
//     path = Path();

//     std::chrono::milliseconds time_limit = std::chrono::milliseconds(30);
    
//     oal::PathReport report;

//     planner.ComputePath(goal, colregs_, path, report, time_limit);
    
//     if(report.result == oal::SearchResult::TIMEOUT){
//         RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Path computation timeout");
//         return false;
//     }else if(report.result == oal::SearchResult::NOWAYOUT){
//         RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "No way out of first position, %s", report.failMsg.c_str());
//         return false;
//     }else if(report.result == oal::SearchResult::PARTIAL){
//         RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Partial path found, %s", report.failMsg.c_str());
//     }

    
//     path.UpdateMetrics(GetLocal(vh_position_), vh_heading_, conf_.rotational_speed);
//     auto message = ulisse_msgs::msg::AvoidancePath();

//     creation_time = rclcpp::Clock().now();
//     message.stamp.sec = creation_time.seconds();
//     message.stamp.nanosec = creation_time.nanoseconds();

//     message.colregs_compliant = colregs_;
//     message.vh_position.latitude = vh_position_.latitude;
//     message.vh_position.longitude = vh_position_.longitude;
//     message.vh_heading = vh_heading_;
//     message.vh_rot_speed = conf_.rotational_speed;
//     for(double vel : velocities_) message.velocities.push_back(vel);
//     message.goal.latitude = goal_.latitude;
//     message.goal.longitude = goal_.longitude;
//     message.max_heading_change = path.metrics.maxHeadingChange;
//     message.tot_heading_change = path.metrics.totHeadingChange;
//     message.tot_distance = path.metrics.totDistance;
//     message.estimated_time_to_goal = path.metrics.estimatedTime;

//     // Waypoints
//     Path path_temp = path;
//     while(!path_temp.empty()){
//         auto wp = ulisse_msgs::msg::Waypoint();
//         auto abs = GetLatLong(path_temp.top().position);
//         wp.position.latitude = abs.latitude;
//         wp.position.longitude = abs.longitude;
//         wp.speed = path_temp.top().speed_to_it;
//         //std::cerr<<"---"<<wp.speed<<"\n";
//         if(path_temp.top().obs_ptr != nullptr){
//             wp.obs_id = path_temp.top().obs_ptr->id;
//             switch(path_temp.top().vx){
//             case FR:
//                 wp.vx = "FR";
//                 break;
//             case FL:
//                 wp.vx = "FL";
//                 break;
//             case RR:
//                 wp.vx = "RR";
//                 break;
//             case RL:
//                 wp.vx = "RL";
//                 break;
//             case NA:
//                 wp.vx = "NA";
//                 break;
//             }
//         }
//         message.wps.push_back(wp);
//         path_temp.pop();
//     }   


//     //SetPathMsg(path, obstacles, message);
//     compPathPub_->publish(message);
 
//     // If close to next wp, ignore it
//     path_temp = path;
//     path_temp.pop();
//     if ((GetLocal(vh_position_) - path_temp.top().position).norm() < conf_.waypoint_acceptance_radius) {
//         //std::cerr<<"close to next wp, skipping it\n"; 
//         path.pop();
//     }

//     return true;

// }

// bool PathPlannerNode::CheckPath(Path &path) {
//     // Update environment info
//     oal::PathPlanner planner;
//     Eigen::Vector2d vh_pos = GetLocal(vh_position_);
//     VehicleInfo v_info({vh_pos, velocities_, vh_heading_, conf_.rotational_speed});
//     planner.SetVhData(v_info);
//     planner.SetAccRadius(conf_.waypoint_acceptance_radius);
//     std::vector<Obstacle> obstacles;
//     SyncObssData(last_pos_update_, obstacles);
//     planner.SetObssData(obstacles);
//     Eigen::Vector2d unreachable_wp_local;
//     bool ret = planner.CheckPath(vh_pos, path, unreachable_wp_local);
//     if(!ret) std::cout<<" Collision before reaching wp ( "<<GetLatLong(unreachable_wp_local).latitude<<", "<<GetLatLong(unreachable_wp_local).longitude<<" )"<<std::endl;
//     return ret;
// }

// void PathPlannerNode::CheckProgress() {

//     if (last_known_vhStatus_ == ulisse::states::ID::halt) {
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " VEHICLE NOT READY (HALT STATE) ");
//         active_ = false;
//         stopTracking();
//         //CallKCL(ulisse::commands::ID::halt);
//     } else {

//         //bool sendNew = false;
//         // If new pos update exists, check progress
//         if (last_pos_update_ > last_check_progress_ || sendNew) {
//             // Check if it reached a new waypoint and check if path is still doable
//             last_check_progress_ = last_pos_update_;

//             // Start by removing first wp as it is the starting one
//             Path path_temp = path_;
//             if (path_temp.waypoints.empty()) {
//                 std::cout << " ERROR! " << std::endl;
//             }
//             path_temp.pop(); // removing current starting point

//             // Check if ownship reached next wp / goal
//             //  note there are 2 different acceptance radius, user choose goal one in GUI
//             if (path_temp.size() == 1) {
//                 if ((GetLocal(vh_position_) - path_temp.top().position).norm() < radius_) {
//                     // Vehicle reached goal
//                     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " Reached end after %f seconds",
//                                 rclcpp::Clock().now().seconds() - last_path_computation_.seconds());
//                     // Reached goal, make vh hold its position
//                     stopTracking();
//                     CallKCL(ulisse::commands::ID::hold);
//                     return;
//                 }
//             } else {
//                 if ((GetLocal(vh_position_) - path_temp.top().position).norm() < conf_.waypoint_acceptance_radius) {
//                     // Vehicle reached inner waypoint
//                     path_.pop();
//                     sendNew = true;
//                     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " Reached a waypoint after %f seconds",
//                                 rclcpp::Clock().now().seconds() - last_path_computation_.seconds());
//                 }
//             }

//             // Check if path is still safe and is there is one better
//             bool isOldSafe = CheckPath(path_);

//             Path new_path;
//             rclcpp::Time creation_time;
//             if (ComputePath(new_path, creation_time)) {
//                 path_temp = path_;
//                 path_temp.UpdateMetrics(GetLocal(vh_position_), vh_heading_, conf_.rotational_speed);
//                 //new_path.UpdateMetrics(GetLocal(vh_position_), vh_heading_); done when computing it

//                 bool isNewBetter =
//                         new_path.metrics.totDistance / path_temp.metrics.totDistance < 1 - conf_.better_path_distance_perc;
//                 if (!isOldSafe || isNewBetter) {
//                     if (isNewBetter) {
//                         double perc = new_path.metrics.totDistance / path_temp.metrics.totDistance * 100;
//                         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - changing path to one [%f]%% shorter", 100 - perc);
//                         last_path_change_reason_ = "found_one_better"+std::to_string(100-perc);
//                     } else {
//                         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - path is NOT safe, found new one.");
//                         last_path_change_reason_ = "risk_of_collision";
//                     }

//                     last_path_computation_ = rclcpp::Clock().now();
//                     last_path_creation_time = creation_time;
//                     path_ = new_path;
//                     CoordinatesPub();
//                     sendNew = true;
//                     std::cout << "-----------------------------" << std::endl;
//                     std::cout << "Here is the new path:" << std::endl;
//                     printPath();
//                     std::cout << "------" << std::endl;
//                 }
//             } else {
//                 if (!isOldSafe) {
//                     // old sucks and there is no new, fuck
//                     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - path is NOT safe and no other exists, stop.");
//                     last_path_change_reason_ = "risk_of_collision_without_alternatives";
//                     stopTracking();
//                     CallKCL(ulisse::commands::ID::hold);
//                     return;
//                 }
//             }

//             if (sendNew) {
//                 actual_path_.push_back(vh_position_);
//                 CallKCL();
//                 sendNew = false;
//             }

//         } else {
//             // Check if position is very old
//             // if (!isPosUpToDate()) {
//             //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Own Ship position unknown.");
//             //     stopTracking();
//             //     CallKCL(ulisse::commands::ID::hold);
//             // }
//         }
//     }
// }

// void PathPlannerNode::VehicleStatusCB(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg) {
//     last_known_vhStatus_ = msg->vehicle_state;
// }


// void PathPlannerNode::ObstacleCB(const detav_msgs::msg::ObstacleList::SharedPtr msg) {
//     lastObsUpdate_ = msg->header.stamp;
     
//     if(msg->obstacles.size() != obstacles_.size()) {
//         RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Now tracking %ld instead of %ld obstacles!", msg->obstacles.size(), obstacles_.size());
//     }

//     obstacles_.clear(); //TODO just update existing ones + new
//     for( const auto& obs : msg->obstacles){

//         // TODO Delete once Luca fixes the obs_id
//         std::string obs_id = obs.id;
//         if ( obs_id.empty() || !std::all_of(obs_id.begin(), obs_id.end(), ::isalnum)) {
//             obs_id = "unknown" + std::to_string(obstacles_.size());
//         }

//         bool higherPriority = false;
//         if(obs.obs_class == detav_msgs::obstypes::higher_priority) higherPriority = true;

//         Eigen::Vector2d position = GetLocal(ctb::LatLong(obs.pose.position.position.latitude, obs.pose.position.position.longitude));
//         double heading = M_PI_2 - obs.pose.orientation.yaw; //Heading so far refers to the north axis, but in obstacle ENU is used
//         Eigen::Vector2d velocity = {obs.twist.twist.linear.x, obs.twist.twist.linear.y};

//         auto bb_dimension = bb_dimension_default_;
//         bb_dimension.dim_x = obs.size.size.length;
//         bb_dimension.dim_y = obs.size.size.width;

//         bb_dimension.Set(conf_.bb_gain,
//             {obs.twist.twist.linear.x, obs.twist.twist.linear.y}, //obs velocity
//             obs.obs_class,
//             obs.size.covariance[0], //size_x_sigma
//             obs.size.covariance[1], //size_y_sigma
//             obs.pose.position.north_cov, //pose_x_sigma
//             obs.pose.position.east_cov, //pose_y_sigma
//             obs.pose.orientation.covariance, //pose_yaw_sigma
//             obs.twist.covariance[0], //vel_x_sigma
//             obs.twist.covariance[1] //vel_y_sigma
//         );

//         Obstacle obstacle(obs_id, position, heading, velocity.norm(), atan2(velocity.y(), velocity.x()), bb_dimension, higherPriority);
        
//         obstacles_.push_back(obstacle);

//         auto gui_msg = ulisse_msgs::msg::Obstacle();
//         SetObsMsg(obstacle, gui_msg);
//         obsGuiPub_->publish(gui_msg);
//     }
// }

// void PathPlannerNode::NavFilterCB(ulisse_msgs::msg::NavFilterData::SharedPtr msg) {
//     rclcpp::Time msg_time(msg->stamp.sec, msg->stamp.nanosec);
//     last_pos_update_ = msg_time;
//     vh_position_ = ctb::LatLong(msg->inertialframe_linear_position.latlong.latitude,
//                                 msg->inertialframe_linear_position.latlong.longitude);
//     // assuming given yaw is in NED coordinates
//     vh_heading_ = M_PI / 2 - msg->bodyframe_angular_position.yaw;
//     if(vh_heading_>M_PI) vh_heading_ -= 2*M_PI;
//     if(vh_heading_<-M_PI) vh_heading_ += 2*M_PI;
// }

// Eigen::Vector2d PathPlannerNode::GetLocal(ctb::LatLong LatLong, bool NED) const {
//     Eigen::Vector3d pos;
//     if (NED) {
//         ctb::LatLong2LocalNED(LatLong, 0, conf_.centroid, pos);
//     } else {
//         ctb::LatLong2LocalUTM(LatLong, 0.0, conf_.centroid, pos);
//     }
//     return {pos.x(), pos.y()};
// }

// ctb::LatLong PathPlannerNode::GetLatLong(Eigen::Vector2d pos_2d, bool NED) const {
//     Eigen::Vector3d pos(pos_2d.x(), pos_2d.y(), 0);
//     double alt;
//     ctb::LatLong LatLong;
//     if (NED) {
//         ctb::LocalNED2LatLong(pos, conf_.centroid, LatLong, alt);
//     } else {
//         ctb::LocalUTM2LatLong(pos, conf_.centroid, LatLong, alt);
//     }
//     return LatLong;
// }

// void PathPlannerNode::StatusPub() {
//     auto message = ulisse_msgs::msg::AvoidanceStatus();

//     long now_stamp = (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())).count();
//     auto now_stamp_secs = static_cast<unsigned int>(now_stamp / static_cast<int>(1E9));
//     auto now_stamp_nanosecs = static_cast<unsigned int>(now_stamp % static_cast<int>(1E9));
//     message.stamp.sec = now_stamp_secs;
//     message.stamp.nanosec = now_stamp_nanosecs;
//     message.n_known_obs = obstacles_.size();  

//     if(!active_){
//         message.status = "Not active";
//     }else{
//         message.status = "Active";
//         message.path_change = last_path_change_reason_;
//         last_path_change_reason_ = "";
//         message.colregs_compliant = colregs_;
//         message.desired_speed = last_cmd_speed;
//         message.current_path_creation.sec = last_path_creation_time.seconds();
//         message.current_path_creation.nanosec = last_path_creation_time.nanoseconds();
//     }
//     avoidanceStatusPub_->publish(message);
// }

// void PathPlannerNode::CoordinatesPub() {
//     auto message = ulisse_msgs::msg::CoordinateList();
//     Path path_temp = path_;
//     message.id = "Path";
//     while (!path_temp.empty()) {
//         ctb::LatLong pos = GetLatLong(path_temp.top().position);
//         path_temp.pop();
//         auto latlong = ulisse_msgs::msg::LatLong();
//         latlong.longitude = pos.longitude;
//         latlong.latitude = pos.latitude;
//         message.coordinates.push_back(latlong);
//     }
//     coordinatesPub_->publish(message);
// }

// void PathPlannerNode::LoadConf(bool print){
//     libconfig::Config cfg_;
//     std::string ulisse_avoidance_dir = ament_index_cpp::get_package_share_directory("ulisse_avoidance");
//     std::string ulisse_avoidance_dir_conf = ulisse_avoidance_dir;
//     ulisse_avoidance_dir_conf.append("/conf/configuration.cfg");
//     cfg_.readFile(ulisse_avoidance_dir_conf.c_str());
//     //conf_.colregs = cfg_.lookup("colregs");
//     conf_.bb_gain = cfg_.lookup("bb_gain");
//     conf_.centroid.latitude = cfg_.lookup("centroid.latitude");
//     conf_.centroid.longitude = cfg_.lookup("centroid.longitude");
//     conf_.rotational_speed = cfg_.lookup("rotational_speed");
//     conf_.speed_min = cfg_.lookup("starting_velocity");
//     conf_.speed_step = cfg_.lookup("velocity_step");
//     conf_.better_path_distance_perc = cfg_.lookup("better_path_distance_perc");
//     conf_.max_pos_delay_time = cfg_.lookup("max_pos_delay_time");
//     conf_.check_progress_rate = cfg_.lookup("check_progress_rate");
//     conf_.status_pub_rate = cfg_.lookup("status_pub_rate");
//     conf_.obs_expired_time = cfg_.lookup("obs_expired_time");
//     conf_.waypoint_acceptance_radius = cfg_.lookup("waypoint_acceptance_radius");

//     bb_dimension_default_.minDistFromObs = cfg_.lookup("min_distance_from_obs");
//     bb_dimension_default_.reductionWhileCheckingPath = cfg_.lookup("reduction_while_checking_path");
//     bb_dimension_default_.safeMaxGap = cfg_.lookup("safe_max_gap");
//     bb_dimension_default_.lookAheadSafetySpan = cfg_.lookup("look_ahead_safety_span");

//     if (print) {
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loaded configuration:");
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  - Centroid: { %.8f, %.8f }", conf_.centroid.latitude, conf_.centroid.longitude);

//     RCLCPP_INFO(
//         rclcpp::get_logger("rclcpp"), 
//         "  - Considering the vehicle turns %s", 
//         conf_.rotational_speed == 0 ? "instantaneously" : ("at " + std::to_string(conf_.rotational_speed) + " rad/s").c_str()
//     );

//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
//                 "  - Path progress check every %.2fs, Waypoint acceptance radius: %.2fm", 
//                 conf_.check_progress_rate, conf_.waypoint_acceptance_radius);
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
//                 "  - New path if %.2f%% shorter, Pub avoidance update every %.2fs", 
//                 conf_.better_path_distance_perc * 100, conf_.status_pub_rate);
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
//                 "  - Obstacle ignore time after %.2fs without updates, Max delay in receiving position: %.2fs", 
//                 conf_.obs_expired_time, conf_.max_pos_delay_time);
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
//                 "  - Min speed: %.2f, Speed step: %.2f, BB uncertainty multiplier: %.2f", 
//                 conf_.speed_min, conf_.speed_step, conf_.bb_gain);  
//     }

// }