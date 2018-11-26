/*
 * taskOM2Console.cc
 *
 *  Created on: Sep 13, 2016
 *      Author: wonder
 */

#include <cstdio>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "ulisse_ctrl/data_structs.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"

#include "ulisse_msgs/srv/control_command.hpp"
#include "ulisse_msgs/topicnames.hpp"

using namespace ulisse;
using namespace std::chrono_literals;

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("controller_console_node");

    int choice;
    bool send;

    auto serviceClient = node->create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service);
    while (!serviceClient->wait_for_service(2s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "waiting for Controller service to appear...");
    }

    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();

    while (rclcpp::ok()) {
        //std::cout << "0 speed jog" << std::endl;
        std::cout << "1)  Move to LatLong" << std::endl;
        std::cout << "2)  Halt" << std::endl;
        //std::cout << "3 hold" << std::endl;
        //std::cout << "4 arm" << std::endl;
        //std::cout << "5 disarm" << std::endl;
        //std::cout << "6 enqueue waypoints" << std::endl;
        //std::cout << "7 set current waypoint" << std::endl;
        //std::cout << "8 follow waypoints" << std::endl;
        //std::cout << "9 reload config" << std::endl;
        std::cout << "10) speed heading command" << std::endl;
        std::cin >> choice;

        if (std::cin.fail()) {
            std::cout << "Flushing bad input!" << std::endl;
            std::cin.clear(); // unset failbit
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }

        send = true;

        switch (choice) {
            /*case 0: {
			std::cout << "speed ";
			std::cin >> command.d.speedJog.desiredSpeed;

			std::cout << "jog ";
			std::cin >> command.d.speedJog.desiredJog;

			std::cout << "timeout [s] ";
			std::cin >> command.d.speedJog.desiredTimeout;
			command.d.speedJog.desiredTimeout *= ortos::constants::oneSecond;

			command.commandType = (uint16_t) CommandType::speedJog;
		}
            break;*/
        case 1: {
            serviceReq->command_type = ulisse::commands::ID::latlong;
            std::cout << "latitude ";
            std::cin >> serviceReq->latlong_cmd.latitude;
            std::cout << "longitude ";
            std::cin >> serviceReq->latlong_cmd.longitude;
            std::cout << "acceptanceRadius ";
            std::cin >> serviceReq->latlong_cmd.acceptance_radius;
        } break;
        case 2: {
            serviceReq->command_type = ulisse::commands::ID::halt;
        } break;
            /*case 3: {
			std::cout << "acceptanceRadius ";
			std::cin >> command.d.hold.acceptanceRadius;

			command.commandType = (uint16_t) CommandType::hold;
		}
			break;
		case 4: {
			command.d.armDisarm.arm = true;

			command.commandType = (uint16_t) CommandType::armDisarm;
		}
			break;
		case 5: {
			command.d.armDisarm.arm = false;

			command.commandType = (uint16_t) CommandType::armDisarm;
		}
			break;
		case 6: {
			std::cout << "enqueue? (1 to enqueue 0 to replace) ";
			int enqueue;
			std::cin >> enqueue;

			if (enqueue)
				command.d.enqueueWaypoints.eraseExistingWaypoints = false;
			else
				command.d.enqueueWaypoints.eraseExistingWaypoints = true;

			std::cout << "waypoints to enqueue ";
			int number;
			std::cin >> number;

			command.d.enqueueWaypoints.waypointsToEnqueue = number;

			if (command.d.enqueueWaypoints.waypointsToEnqueue
					> constants::commandEnqueueWaypointsDataMaxWaypoints) {
				command.d.enqueueWaypoints.waypointsToEnqueue =
						constants::commandEnqueueWaypointsDataMaxWaypoints;
			}

			for (int i = 0; i < command.d.enqueueWaypoints.waypointsToEnqueue;
					++i) {
				std::cout << "waypoint " << i + 1 << "/"
						<< (int) command.d.enqueueWaypoints.waypointsToEnqueue
						<< std::endl;
				command.d.enqueueWaypoints.waypoints[i].seq = i + 1;

				std::cout << "latitude ";
				std::cin >> command.d.enqueueWaypoints.waypoints[i].latitude;

				std::cout << "longitude ";
				std::cin >> command.d.enqueueWaypoints.waypoints[i].longitude;

				std::cout << "holdTime ";
				std::cin >> command.d.enqueueWaypoints.waypoints[i].holdTime;

				std::cout << "acceptanceRadius ";
				std::cin
						>> command.d.enqueueWaypoints.waypoints[i].acceptanceRadius;

				std::cout << "autocontinue? (1 yes 0 no) ";
				int autocontinue;
				std::cin >> autocontinue;

				if (autocontinue)
					command.d.enqueueWaypoints.waypoints[i].autocontinue = true;
				else
					command.d.enqueueWaypoints.waypoints[i].autocontinue =
							false;
			}

			std::cout << "protocol completed? (1 to complete 0 otherwise) ";
			int complete;
			std::cin >> complete;

			if (complete)
				command.d.enqueueWaypoints.waypointProtocolCompleted = true;
			else
				command.d.enqueueWaypoints.waypointProtocolCompleted = false;

			command.commandType = (uint16_t) CommandType::enqueueWaypoints;
		}
			break;
		case 7: {
			std::cout << "current waypoint? ";
			int currentWaypoint;
			std::cin >> currentWaypoint;

			command.d.setCurrentWaypoint.currentWaypoint = currentWaypoint;

			command.commandType = (uint16_t) CommandType::setCurrentWaypoint;
		}
			break;
		case 8: {
			std::cout << "from beginning? (1 yes 0 no) ";
			int beginning;
			std::cin >> beginning;

			if (beginning)
				command.d.followWaypoints.fromBeginning = true;
			else
				command.d.followWaypoints.fromBeginning = false;

			command.commandType = (uint16_t) CommandType::followWaypoints;
		}
			break;
		case 9: {
			command.commandType = (uint16_t) CommandType::reloadConfig;
		}
            break;*/
        case 10: {
            serviceReq->command_type = ulisse::commands::ID::speedheading;

			std::cout << "speed ";
            std::cin >> serviceReq->sh_cmd.speed;

			std::cout << "heading ";
            std::cin >> serviceReq->sh_cmd.heading;

			std::cout << "timeout [s] ";
            std::cin >> serviceReq->sh_cmd.timeout.sec;
            serviceReq->sh_cmd.timeout.nanosec = 0;

            //shref.d.type = om2ctrl::SpeedHeadingType::absolute;
        }
            break;
        default:
            std::cout << "Unsupported choice! " << choice << std::endl;
            send = false;
            continue;
            //break;
        }

        if (send) {
            auto result_future = serviceClient->async_send_request(serviceReq);
            std::cout << "Sent Request to controller" << std::endl;
            if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(node->get_logger(), "service call failed :(");
            } else {
                auto result = result_future.get();
                RCLCPP_INFO(node->get_logger(), "Service returned: %s", (result->res).c_str());
            }
        }
    }
}
