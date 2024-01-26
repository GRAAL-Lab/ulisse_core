#include "ulisse_avoidance/oal_interface.hpp"

 void OalInterfaceNode::handleComputePathRequest(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ulisse_msgs::srv::ComputeAvoidancePath::Request> request,
    const std::shared_ptr<ulisse_msgs::srv::ComputeAvoidancePath::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New request.");
  stopTracking();
  if(!available_ || !isPosUpToDate()){
    // MAYBE SHUT SERVICE DOWN IN THIS CASE
    response->res = false;
    available_ = false;
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

void OalInterfaceNode::CallKCL(bool halt){
  auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
  if(halt){
    serviceReq->command_type = ulisse::commands::ID::halt;
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
    serviceReq->latlong_cmd.acceptance_radius = radius_;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "...");
  static std::string result_msg;
  bool serviceAvailable;
  if (command_srv_->service_is_ready()) {
    auto result_future = command_srv_->async_send_request(serviceReq);
    std::cout << "Sent Request to controller" << std::endl;
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
    serviceAvailable = true;
  } else {
    result_msg = "The controller doesn't seem to be active.\n(No CommandServer available)";
    serviceAvailable = false;
  }
  // CHECK PROGRESS = serviceAvailable;
}

bool OalInterfaceNode::ComputePath(){
  VehicleInfo v_info({GetLocal2d(vh_position_), velocities_});
  Eigen::Vector2d goal = GetLocal2d(goal_);

  // Sync vh_data and obs data
  planner.SetVhData(v_info);
  SetObssData(last_pos_update_);  // TODO LOSE OLD OBS ??????????

  if (!planner.ComputePath(goal, colregs_, path_)) {
    std::cout <<std::endl << "Not found." << std::endl;
    return false;
  }else{
    //std::cout <<std::endl << "Found." << std::endl;
    coordinates_pub();
    return true;
  }
}

bool OalInterfaceNode::CheckPath()
{
  // Update environment info
  Eigen::Vector2d vh_pos = GetLocal2d(vh_position_);
  SetObssData(last_pos_update_);
  return planner.CheckPath(vh_pos, path_);
}

void OalInterfaceNode::CheckProgress(){
  std::cout << " . " << std::endl;
  std::cout << " tracking " << obstacles_.size()<< " obstacles."<< std::endl;
  bool sendNew = false;
  // If new pos update exists when timerCB
  // CHECK > MEANS AFTER
  if(last_pos_update_ > last_check_progress_){
    last_check_progress_ = last_pos_update_;

    // I know where it is: look if it reached a new waypoint and check if path is still doable
    Path path_temp = path_;
    path_temp.pop(); // removing current starting point

    // TODO < radius_
    if((GetLocal2d(vh_position_) - path_temp.top().position).norm() < radius_ ){
      // Vehicle reached next waypoint
      path_.pop();
      std::cout << " reached wp " << std::endl;
      if(path_.waypoints.size()<=1){
        std::cout << " reached end " << std::endl;
        // Reached goal
        CallKCL(true);
        stopTracking();
        return;
      }
      sendNew = true;
    }

    if(CheckPath()){
      std::cout << " - path still good " << std::endl;
      if(sendNew) CallKCL();
    }else{
      //overtakingObsList !!!!
      if(ComputePath()){
        CallKCL();
      }else{
        CallKCL(true);
        stopTracking();
      }
    }
  }else{
    // Check if position is very old
    if(!isPosUpToDate()){
      std::cout <<std::endl << "Own Ship position unknown." << std::endl;
      available_ = false;
      CallKCL(true);
      stopTracking();
    }
  }
}

void OalInterfaceNode::VehicleStatusCB(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg) {
  auto vehicleStatus = *msg;
  if(vehicleStatus.vehicle_state != ulisse::states::ID::latlong){
    stopTracking();
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
  // Handle topic message
  Eigen::Vector2d velocity(msg->vel_x, msg->vel_y);
  bb_data bb_dimension(20, 5,
                       1.6, 1.4,
                       2, 2,
                       1.5, 1.2,
                       1.5, 1.5,
                       0);
  Obstacle obs(msg->id, GetLocal2d(ctb::LatLong(msg->center.latitude, msg->center.longitude)),
               msg->heading, velocity.norm(), atan2(velocity.y(), velocity.x()),
               bb_dimension,
               msg->high_priority);
  addObstacle(obs);
}

void OalInterfaceNode::PosCB(const ulisse_msgs::msg::GPSData::SharedPtr msg){
  // Handle topic message
  last_pos_update_ = rclcpp::Clock().now();
  vh_position_ = ctb::LatLong(msg->latitude, msg->longitude);
  available_ = true;
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
  auto old = [](ObstacleWithTime obs_t) -> bool {
      return (10 <= (obs_t.timestamp.seconds() - rclcpp::Clock().now().seconds()));
  };
  obstacles_.erase(std::remove_if(obstacles_.begin(), obstacles_.end(), old), obstacles_.end());
  message.n_known_obs = obstacles_.size();
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