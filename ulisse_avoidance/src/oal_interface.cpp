#include "ulisse_avoidance/oal_interface.hpp"

 void OalInterfaceNode::handleComputePathRequest(
    const std::shared_ptr<rmw_request_id_t>& request_header,
    const std::shared_ptr<ulisse_msgs::srv::ComputeAvoidancePath::Request>& request,
    const std::shared_ptr<ulisse_msgs::srv::ComputeAvoidancePath::Response>& response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New request.");
  if(active_) stopTracking();
  if(!isPosUpToDate()){
    response->res = false;
    return;
  }

  velocities_ = request->velocities;
  goal_ = ctb::LatLong(request->latlong_cmd.goal.latitude, request->latlong_cmd.goal.longitude);
  colregs_ = request->colregs_compliant;
  radius_ = request->latlong_cmd.acceptance_radius; // used also for waypoint acceptance

  if(ComputePath()){
    response->res = true;
    CallKCL();
    last_check_progress_ = last_pos_update_;
    startTracking();  // starts CheckProgress() timer
  }else{
    response->res = false;
  }
}

void OalInterfaceNode::CallKCL(bool hold){
  auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
  if(hold){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Stopping vehicle! ");
    serviceReq->command_type = ulisse::commands::ID::hold;
    serviceReq->latlong_cmd.acceptance_radius = radius_;
  }else{
    // Get point
    Path path = path_;
    //double speed = path.waypoints.top().vh_speed;
    // HOW DO I SEND SPEED COMMAND TO ULISSE?
    path.waypoints.pop();
    ctb::LatLong goal = GetAbsolute(path.waypoints.top().position);
    serviceReq->command_type = ulisse::commands::ID::latlong;
    serviceReq->latlong_cmd.goal.latitude = goal.latitude;
    serviceReq->latlong_cmd.goal.longitude = goal.longitude;
    // TODO too small radius makes state go to hold and so stop tracking
    if(path.size() == 1){
      serviceReq->latlong_cmd.acceptance_radius = radius_;
    }else{
      serviceReq->latlong_cmd.acceptance_radius = WAYPOINT_ACC_RADIUS;
    }
  }
  static std::string result_msg;

  if (command_srv_->service_is_ready()) {
    auto result_future = command_srv_->async_send_request(serviceReq);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - sending cmd to KCL");
    /*if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
      result_msg = "Service call failed :(";
      std::cout << result_msg << std::endl;
      RCLCPP_ERROR_STREAM(this->get_logger(), result_msg.c_str());
    } else {
      auto result = result_future.get();
      result_msg = "Service returned: " + result->res;
      std::cout << result_msg << std::endl;
      RCLCPP_INFO_STREAM(this->get_logger(), result_msg);
    }*/
  } else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The controller doesn't seem to be active.\n(No CommandServer available)");
    stopTracking();
  }
  // CHECK PROGRESS = serviceAvailable;
}

bool OalInterfaceNode::ComputePath(){
  VehicleInfo v_info({GetLocal2d(vh_position_), velocities_});
  Eigen::Vector2d goal = GetLocal2d(goal_);

  // Sync vh_data and obs data
  planner = path_planner();
  planner.SetVhData(v_info);
  SetObssData(last_pos_update_);
  path_ = Path();
  if (planner.ComputePath(goal, colregs_, path_)) {



    last_path_computation_ = rclcpp::Clock().now();
    coordinates_pub();
    return true;
  }

  return false;
}

bool OalInterfaceNode::CheckPath()
{
  // Update environment info
  planner = path_planner();
  Eigen::Vector2d vh_pos = GetLocal2d(vh_position_);
  SetObssData(last_pos_update_);
  return planner.CheckPath(vh_pos, path_);
}

void OalInterfaceNode::CheckProgress(){
  if (!active_) return;

  bool sendNew = false;
  // If new pos update exists, check progress
  if(last_pos_update_ > last_check_progress_){
    last_check_progress_ = last_pos_update_;

    // check if it reached a new waypoint and check if path is still doable
    Path path_temp = path_;
    if(path_temp.waypoints.empty()){
      std::cout << " ERROR! " << std::endl;
    }
    path_temp.pop(); // removing current starting point

    if(path_temp.size()==1){
      // Reaching final wp
      if((GetLocal2d(vh_position_) - path_temp.top().position).norm() < radius_ ){
        std::cout << " reached end!!! " << std::endl;
        // Reached goal, make vh hold its position
        CallKCL(true);
        stopTracking();
        return;
      }
    }else if((GetLocal2d(vh_position_) - path_temp.top().position).norm() < WAYPOINT_ACC_RADIUS ){
        // Vehicle reached next waypoint
        path_.pop();
        sendNew = true;
        std::cout << " reached wp! " << std::endl;
    }

    /*if((GetLocal2d(vh_position_) - path_temp.top().position).norm() < radius_ ){
      // Vehicle reached next waypoint
      path_.pop();
      std::cout << " reached wp! " << std::endl;
      if(path_.size()<=1){
        std::cout << " reached end!!! " << std::endl;
        // Reached goal, make vh hold its position
        CallKCL(true);
        stopTracking();
        return;
      }
      sendNew = true;
    }*/

    if(CheckPath() && (last_path_computation_.seconds() - rclcpp::Clock().now().seconds()) <= TIME_EXPIRED_PATH){
      //std::cout << " - path is safe, keep it going (tracking " << obstacles_.size()<<" obstacles)"<< std::endl;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - path is safe, keep it going (tracking %zu obstacles)", obstacles_.size());
      if(sendNew) CallKCL();
    }else{
      //overtakingObsList !!!!
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - path is NOT safe, searching a new one");
      if(ComputePath()){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "    - found");
        CallKCL();
      }else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "    - NOT found, stop");
        CallKCL(true);
        stopTracking();
      }
    }
  }else{
    // Check if position is very old
    if(!isPosUpToDate()){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Own Ship position unknown.");
      CallKCL(true);
      stopTracking();
    }
  }
}

void OalInterfaceNode::VehicleStatusCB(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg) {
  auto vehicleStatus = *msg;
  if(active_ && vehicleStatus.vehicle_state != last_known_vhStatus_) {
    last_known_vhStatus_ = vehicleStatus.vehicle_state;
    if (vehicleStatus.vehicle_state != ulisse::states::ID::latlong) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " STATUS HAS CHANGED: STOP TRACKING");
      stopTracking();
    }
  }
}

void OalInterfaceNode::addObstacle(const Obstacle& obs){
  auto it = std::find_if(obstacles_.begin(), obstacles_.end(),
                         [&obs](const ObstacleWithTime& entry) {
                             return entry.data.id == obs.id;
                         });
  if (it != obstacles_.end()) {
    // Update existing entry
    it->data = obs;
    it->timestamp = rclcpp::Clock().now();
  } else {
    // Add new entry
    ObstacleWithTime entry;
    entry.data = obs;
    entry.timestamp = rclcpp::Clock().now();
    obstacles_.push_back(entry);
  }
}

void OalInterfaceNode::ObstacleCB(const ulisse_msgs::msg::Obstacle::SharedPtr msg){
  Eigen::Vector2d pos = GetLocal2d(ctb::LatLong(msg->center.latitude, msg->center.longitude));

  // HERE CONVERSION OF DIRECTION FROM NED TO UTM
  // (v_x' = v_y, v_y'= v_x, heading' = M_PI/2-heading)
  // Handle topic message
  Eigen::Vector2d velocity(msg->vel_y, msg->vel_x);
  bb_data bb_dimension(msg->b_box_dim_x, msg->b_box_dim_y,
                       msg->bb_max.x_bow_ratio , msg->bb_max.x_stern_ratio,
                       msg->bb_max.y_starboard_ratio, msg->bb_max.y_port_ratio,
                       msg->bb_safe.x_bow_ratio , msg->bb_safe.x_stern_ratio,
                       msg->bb_safe.y_starboard_ratio, msg->bb_safe.y_port_ratio,
                       msg->uncertainty_gap);
  Obstacle obs(msg->id, pos,
               M_PI/2-msg->heading, velocity.norm(), atan2(velocity.y(), velocity.x()),
               bb_dimension,
               msg->high_priority);
  addObstacle(obs);

  //std::cout << "NEW OBS IN LIBRARY: "<<msg->id<<" in localUTM pos: ("<<(int)pos.x()<<", "<<(int)pos.y()<<"), heading: "<< M_PI/2-msg->heading<<" with speed vector: ("<<velocity.x()<<", "<<velocity.y()<<") "<<std::endl;
  /*Eigen::Vector3d pos_test;
  ctb::LatLong2LocalNED(ctb::LatLong(msg->center.latitude, msg->center.longitude), 0, centroid_, pos_test);
  std::cout<<"NED: "<<pos_test.x()<<", "<< pos_test.y()<<std::endl;*/

}

void OalInterfaceNode::PosCB(const ulisse_msgs::msg::GPSData::SharedPtr msg){
  // Handle topic message
  last_pos_update_ = rclcpp::Clock().now();
  vh_position_ = ctb::LatLong(msg->latitude, msg->longitude);
}

Eigen::Vector2d OalInterfaceNode::GetLocal2d(ctb::LatLong point){
  Eigen::Vector3d pointLocalUTM;
  ctb::LatLong2LocalUTM(point, 0.0, centroid_, pointLocalUTM);
  Eigen::Vector2d pointLocalUTM_2d(pointLocalUTM.x(), pointLocalUTM.y());
  return pointLocalUTM_2d;
}

ctb::LatLong OalInterfaceNode::GetAbsolute(Eigen::Vector2d pointLocalUTM_2d){
  Eigen::Vector3d pointLocalUTM(pointLocalUTM_2d.x(), pointLocalUTM_2d.y(), 0);
  double alt;
  ctb::LatLong point;
  ctb::LocalUTM2LatLong(pointLocalUTM, centroid_, point, alt);
  return point;
}

void OalInterfaceNode::status_pub_callback(){
  auto message = ulisse_msgs::msg::AvoidanceStatus();
  if (active_){
    message.status = "Active";
    message.colregs_compliant = colregs_;
    message.goal.longitude = goal_.longitude;
    message.goal.latitude = goal_.latitude;
    //message.speed = path_.top().vh_speed;
    Path path_temp = path_;
    path_temp.pop();
    message.speed = path_.top().speed_to_it;
    ctb::LatLong next_wp = GetAbsolute(path_temp.top().position);
    message.next_wp.longitude = next_wp.longitude;
    message.next_wp.latitude = next_wp.latitude;
  }else{
    message.status = "Not active";
  }

  // DELETING OLD OBSTACLE
  //  older_than 10s;
  auto old = [](const ObstacleWithTime& obs_t) -> bool {
      return (10 <= (obs_t.timestamp.seconds() - rclcpp::Clock().now().seconds()));
  };
  obstacles_.erase(std::remove_if(obstacles_.begin(), obstacles_.end(), old), obstacles_.end());
  message.n_known_obs = (double)obstacles_.size();  // TODO fix msg
  avoidanceStatusPub_->publish(message);
}

void OalInterfaceNode::coordinates_pub(){
  auto message = ulisse_msgs::msg::CoordinateList();
  Path path_temp = path_;
  //path_temp.pop(); why?
  message.id = "Path";

  while(!path_temp.empty()){
    ctb::LatLong pos = GetAbsolute(path_temp.top().position);
    path_temp.pop();
    auto latlong = ulisse_msgs::msg::LatLong();
    latlong.longitude = pos.longitude;
    latlong.latitude = pos.latitude;
    message.coordinates.push_back(latlong);
  }
  coordinatesPub_->publish(message);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  //auto node = std::make_shared<OalInterfaceNode>();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready.");
  rclcpp::spin(std::make_shared<OalInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}