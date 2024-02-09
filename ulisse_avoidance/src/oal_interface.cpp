#include "ulisse_avoidance/oal_interface.hpp"

void OalInterfaceNode::handleComputePathRequest(
        const std::shared_ptr<rmw_request_id_t> &request_header,
        const std::shared_ptr<ulisse_msgs::srv::ComputeAvoidancePath::Request> &request,
        const std::shared_ptr<ulisse_msgs::srv::ComputeAvoidancePath::Response> &response) {
    if (last_known_vhStatus_ == ulisse::states::ID::halt) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " VEHICLE NOT READY (HALT STATE) ");
    } else {
        std::cout << "---------------------------------" << std::endl;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New request.");
        if (active_) stopTracking(); // stop tracking old path
        if (!isPosUpToDate()) {
            response->res = false;
            return;
        }

        velocities_ = request->velocities;
        goal_ = ctb::LatLong(request->latlong_cmd.goal.latitude, request->latlong_cmd.goal.longitude);
        //colregs_ = request->colregs_compliant;    TODO uncomment after making gui able to choose
        radius_ = request->latlong_cmd.acceptance_radius; // used also for waypoint acceptance

        if (ComputePath(path_)) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " Path found:");
            printPath();
            last_path_computation_ = rclcpp::Clock().now();
            coordinates_pub();
            sendNew = true;
            startTracking();
            //if(CallKCL()){
            response->res = true;
            last_check_progress_ = last_pos_update_;
            //startTracking();  // starts CheckProgress() timer WAIT IT IS IN CALLKCL ALREADY
            return;
            //}
        }
        response->res = false;
    }
}

bool OalInterfaceNode::CallKCL(const std::string &cmd_type) {
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    if (cmd_type == ulisse::commands::ID::halt) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Sending HALT command to KCL. ");
        serviceReq->command_type = ulisse::commands::ID::halt;
    } else {
        if (cmd_type == ulisse::commands::ID::hold) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Sending HOLD command to KCL. ");
            serviceReq->command_type = ulisse::commands::ID::hold;
            serviceReq->latlong_cmd.acceptance_radius = radius_;
        } else {
            // Get point
            Path path = path_;
            //double speed = path.waypoints.top().vh_speed;
            // HOW DO I SEND SPEED COMMAND TO ULISSE?
            path.waypoints.pop();
            ctb::LatLong goal = GetLatLong(path.waypoints.top().position);
            serviceReq->command_type = ulisse::commands::ID::latlong;
            serviceReq->latlong_cmd.goal.latitude = goal.latitude;
            serviceReq->latlong_cmd.goal.longitude = goal.longitude;
            if (path.size() == 1) {
                // Last wp
                serviceReq->latlong_cmd.acceptance_radius = radius_;
            } else {
                serviceReq->latlong_cmd.acceptance_radius = 0.001;   // TODO  this is a shitty thing, but move_to goes to hold from ulisse_map and makes avoidance stop working
            }
        }
    }
    static std::string result_msg;

    if (command_srv_->service_is_ready()) {
        auto result_future = command_srv_->async_send_request(serviceReq);
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - sending cmd to KCL");
        // TODO error if not commented
        std::future_status status = result_future.wait_for(std::chrono::seconds(2));  // timeout to guarantee a graceful finish
        if (status == std::future_status::ready) {
            auto result = result_future.get();
            //result_msg = "Service returned: " + result->res;
            //std::cout << result_msg << std::endl;
            RCLCPP_INFO_STREAM(this->get_logger(), result_msg);
        } else {
            result_msg = "Service call failed :(";
            RCLCPP_ERROR_STREAM(this->get_logger(), result_msg.c_str());
            stopTracking();
            return false;
        }
        return true;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "The controller doesn't seem to be active.\n(No CommandServer available)");
        return false;
    }
}

bool OalInterfaceNode::ComputePath(Path &path) {
    VehicleInfo v_info({GetLocal(vh_position_), velocities_, vh_heading_, conf_.rotational_speed});
    Eigen::Vector2d goal = GetLocal(goal_);

    // Sync vh_data and obs data
    planner = path_planner();
    planner.SetVhData(v_info);
    planner.SetAccRadius(conf_.waypoint_acceptance_radius);
    SetObssData(last_pos_update_);
    path = Path();
    if (planner.ComputePath(goal, conf_.colregs, path)) {

        // If close to next wp, ignore it TODO check this is right
        if (path.size() > 1) {
            Path path_temp = path;
            path_temp.pop();
            if ((GetLocal(vh_position_) - path_temp.top().position).norm() < conf_.waypoint_acceptance_radius) path.pop();
        } else {
            std::cout << "!!!!!!!!! execution should never get here" << std::endl;
        }

        // Only if it is used straightaway
        /*last_path_computation_ = rclcpp::Clock().now();
    coordinates_pub();*/
        return true;
    }

    return false;
}

bool OalInterfaceNode::CheckPath(Path &path) {
    // Update environment info
    planner = path_planner();
    Eigen::Vector2d vh_pos = GetLocal(vh_position_);
    VehicleInfo v_info({vh_pos, velocities_, vh_heading_, conf_.rotational_speed});
    planner.SetVhData(v_info);
    planner.SetAccRadius(conf_.waypoint_acceptance_radius);
    SetObssData(last_pos_update_);

    return planner.CheckPath(vh_pos, path);
}

void OalInterfaceNode::CheckProgress() {

    if (last_known_vhStatus_ == ulisse::states::ID::halt) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " VEHICLE NOT READY (HALT STATE) ");
        active_ = false;
        stopTracking();
        //CallKCL(ulisse::commands::ID::halt);
    } else {

        //bool sendNew = false;
        // If new pos update exists, check progress
        if (last_pos_update_ > last_check_progress_ || sendNew) {
            // Check if it reached a new waypoint and check if path is still doable
            last_check_progress_ = last_pos_update_;

            // Start by removing first wp as it is the starting one
            Path path_temp = path_;
            if (path_temp.waypoints.empty()) {
                std::cout << " ERROR! " << std::endl;
            }
            path_temp.pop(); // removing current starting point

            // Check if ownship reached next wp / goal
            //  note there are 2 different acceptance radius, user choose goal one in GUI
            if (path_temp.size() == 1) {
                if ((GetLocal(vh_position_) - path_temp.top().position).norm() < radius_) {
                    // Vehicle reached goal
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " Reached end after %f seconds",
                                rclcpp::Clock().now().seconds() - last_path_computation_.seconds());
                    // Reached goal, make vh hold its position
                    stopTracking();
                    CallKCL(ulisse::commands::ID::hold);
                    return;
                }
            } else {
                if ((GetLocal(vh_position_) - path_temp.top().position).norm() < conf_.waypoint_acceptance_radius) {
                    // Vehicle reached inner waypoint
                    path_.pop();
                    sendNew = true;
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " Reached a waypoint after %f seconds",
                                rclcpp::Clock().now().seconds() - last_path_computation_.seconds());
                }
            }

            // Check if path is still safe and is there is one better
            bool isOldSafe = CheckPath(path_);

            Path new_path;
            if (ComputePath(new_path)) {
                path_temp = path_;
                path_temp.UpdateMetrics(GetLocal(vh_position_), vh_heading_);
                new_path.UpdateMetrics(GetLocal(vh_position_), vh_heading_);

                bool isNewBetter =
                        new_path.metrics.totDistance < conf_.better_path_distance_perc * path_temp.metrics.totDistance;
                if (!isOldSafe || isNewBetter) {
                    if (isNewBetter) {
                        double perc = new_path.metrics.totDistance / path_temp.metrics.totDistance * 100;
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - changing path to one [%f]%% shorter", 100 - perc);
                    } else {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - path is NOT safe, found new one.");
                    }

                    last_path_computation_ = rclcpp::Clock().now();
                    coordinates_pub();
                    path_ = new_path;
                    sendNew = true;
                    std::cout << "-----------------------------" << std::endl;
                    std::cout << "Here is the new path:" << std::endl;
                    printPath();
                    std::cout << "------" << std::endl;
                }
            } else {
                if (!isOldSafe) {
                    // old sucks and there is no new, fuck
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - path is NOT safe and no other exists, stop.");
                    stopTracking();
                    CallKCL(ulisse::commands::ID::hold);
                    return;
                }
            }

            if (sendNew) {
                CallKCL();
                sendNew = false;
            }

        } else {
            // Check if position is very old
            if (!isPosUpToDate()) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Own Ship position unknown.");
                stopTracking();
                CallKCL(ulisse::commands::ID::hold);
            }
        }
    }
}

void OalInterfaceNode::VehicleStatusCB(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg) {
    last_known_vhStatus_ = msg->vehicle_state;
}

void OalInterfaceNode::addObstacle(const Obstacle &obs) {
    auto it = std::find_if(obstacles_.begin(), obstacles_.end(),
                           [&obs](const ObstacleWithTime &entry) {
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

void OalInterfaceNode::ObstacleCB(const ulisse_msgs::msg::Obstacle::SharedPtr msg) {
    Eigen::Vector2d pos = GetLocal(ctb::LatLong(msg->center.latitude, msg->center.longitude));

    // HERE CONVERSION OF DIRECTION FROM NED TO UTM
    // (v_x' = v_y, v_y'= v_x, heading' = M_PI/2-heading)
    // Handle topic message
    Eigen::Vector2d velocity(msg->vel_y, msg->vel_x);
    bb_data bb_dimension(msg->b_box_dim_x, msg->b_box_dim_y,
                         msg->bb_max.x_bow_ratio, msg->bb_max.x_stern_ratio,
                         msg->bb_max.y_starboard_ratio, msg->bb_max.y_port_ratio,
                         msg->bb_safe.x_bow_ratio, msg->bb_safe.x_stern_ratio,
                         msg->bb_safe.y_starboard_ratio, msg->bb_safe.y_port_ratio,
                         msg->uncertainty_gap);
    Obstacle obs(msg->id, pos,
                 M_PI / 2 - msg->heading, velocity.norm(), atan2(velocity.y(), velocity.x()),
                 bb_dimension,
                 msg->high_priority);
    addObstacle(obs);

    //std::cout << "NEW OBS IN LIBRARY: "<<msg->id<<" in localUTM pos: ("<<(int)pos.x()<<", "<<(int)pos.y()<<"), heading: "<< M_PI/2-msg->heading<<" with speed vector: ("<<velocity.x()<<", "<<velocity.y()<<") "<<std::endl;
    /*Eigen::Vector3d pos_test;
  ctb::LatLong2LocalNED(ctb::LatLong(msg->center.latitude, msg->center.longitude), 0, centroid_, pos_test);
  std::cout<<"NED: "<<pos_test.x()<<", "<< pos_test.y()<<std::endl;*/

}

void OalInterfaceNode::NavFilterCB(ulisse_msgs::msg::NavFilterData::SharedPtr msg) {
    last_pos_update_ = rclcpp::Clock().now();
    vh_position_ = ctb::LatLong(msg->inertialframe_linear_position.latlong.latitude,
                                msg->inertialframe_linear_position.latlong.longitude);
    // assuming given yam is in NED coordinates
    vh_heading_ = M_PI / 2 - msg->bodyframe_angular_position.yaw;
}

Eigen::Vector2d OalInterfaceNode::GetLocal(ctb::LatLong LatLong, bool NED) const {
    Eigen::Vector3d pos;
    if (NED) {
        ctb::LatLong2LocalNED(LatLong, 0, conf_.centroid, pos);
    } else {
        ctb::LatLong2LocalUTM(LatLong, 0.0, conf_.centroid, pos);
    }
    return {pos.x(), pos.y()};
}

ctb::LatLong OalInterfaceNode::GetLatLong(Eigen::Vector2d pos_2d, bool NED) const {
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

void OalInterfaceNode::status_pub_callback() {
    auto message = ulisse_msgs::msg::AvoidanceStatus();
    if (active_) {
        Path path_temp = path_;
        path_temp.pop();
        ctb::LatLong next_wp = GetLatLong(path_temp.top().position);
        message.status = "Active";
        message.colregs_compliant = conf_.colregs;
        message.goal.longitude = goal_.longitude;
        message.goal.latitude = goal_.latitude;
        message.next_wp.longitude = next_wp.longitude;
        message.next_wp.latitude = next_wp.latitude;
        message.speed = path_.top().speed_to_it;
    } else {
        message.status = "Not active";
    }
    // DELETING OLD OBSTACLE
    auto old = [this](const ObstacleWithTime &obs_t) -> bool {
        return (conf_.obs_expired_time <= (rclcpp::Clock().now().seconds() - obs_t.timestamp.seconds()));
    };
    obstacles_.erase(std::remove_if(obstacles_.begin(), obstacles_.end(), old), obstacles_.end());
    message.n_known_obs = (double) obstacles_.size();  // TODO fix msg
    avoidanceStatusPub_->publish(message);
}

void OalInterfaceNode::coordinates_pub() {
    auto message = ulisse_msgs::msg::CoordinateList();
    Path path_temp = path_;
    message.id = "Path";
    while (!path_temp.empty()) {
        ctb::LatLong pos = GetLatLong(path_temp.top().position);
        path_temp.pop();
        auto latlong = ulisse_msgs::msg::LatLong();
        latlong.longitude = pos.longitude;
        latlong.latitude = pos.latitude;
        message.coordinates.push_back(latlong);
    }
    coordinatesPub_->publish(message);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto oalInterface = std::make_shared<OalInterfaceNode>();

    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(oalInterface);
    exe.spin();

    rclcpp::shutdown();

    return 0;
}
