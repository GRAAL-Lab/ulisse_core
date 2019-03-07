#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <rml/RML.h>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/control_context.hpp"
//#include "ulisse_msgs/msg/llc_motors.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
//#include "ulisse_msgs/msg/position_context.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/srv/nav_filter_command.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ulisse_driver/GPSDHelperDataStructs.h"

//#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include "nav_filter/nav_data_structs.hpp"
#include "nav_filter/pos_vel_observer.hpp"

//using namespace ulisse;
using namespace ulisse::nav;
using namespace std::chrono_literals;

static PosVelObserver obs;
static rclcpp::Node::SharedPtr node = nullptr;
static std::shared_ptr<rclcpp::SyncParametersClient> par_client;
static ulisse_msgs::msg::Compass compass;
static ulisse_msgs::msg::GPSData gpsData;
//static ulisse_msgs::msg::PositionContext positionData;
static ulisse_msgs::msg::ControlContext controlData;
static int rate = 10;

void ReloadConfig();
void handle_navfilter_commands(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Request> request,
    std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Response> response);
void controlcontext_cb(const ulisse_msgs::msg::ControlContext::SharedPtr msg);
//void positioncontext_cb(const ulisse_msgs::msg::PositionContext::SharedPtr msg);
void compass_cb(const ulisse_msgs::msg::Compass::SharedPtr msg);
void gpsdata_cb(const ulisse_msgs::msg::GPSData::SharedPtr msg);

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("navigation_filter_node");

    rclcpp::WallRate loop_rate(rate);

    auto srv_ = node->create_service<ulisse_msgs::srv::NavFilterCommand>(
        ulisse_msgs::topicnames::navfilter_cmd_service, handle_navfilter_commands);

    par_client = std::make_shared<rclcpp::SyncParametersClient>(node);
    while (!par_client->wait_for_service(1ms)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(0);
        }
        RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }

    ReloadConfig();

    auto navfilter_pub = node->create_publisher<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data);

    auto ctrlcxt_sub = node->create_subscription<ulisse_msgs::msg::ControlContext>(ulisse_msgs::topicnames::control_context, controlcontext_cb);
    //auto poscxt_sub = node->create_subscription<ulisse_msgs::msg::PositionContext>(ulisse_msgs::topicnames::position_context, positioncontext_cb);
    auto compass_sub = node->create_subscription<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass, compass_cb);
    auto gpsdata_sub = node->create_subscription<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data, gpsdata_cb);

    double lastValidGPSTime = 0;
    ulisse_msgs::msg::NavFilterData filterData;

    gpsData.latitude = 44.4;
    gpsData.longitude = 8.94;

    bool filterEnable(true);

    while (rclcpp::ok()) {

        if (gpsData.time > lastValidGPSTime) {
            if (gpsData.gpsfixmode >= (int)ulisse::gpsd::GpsFixMode::mode_2d) {

                float64_t speedRef;

                speedRef = controlData.desired_speed;

                int zone;
                bool northp;
                double x_utm, y_utm;

                try {
                    GeographicLib::UTMUPS::Forward(gpsData.latitude, gpsData.longitude, zone, northp, x_utm, y_utm);

                    double x_ned = y_utm;
                    double y_ned = x_utm;

                    if (filterEnable) {
                        // The geographic lib conversion outputs UTM coordinates but
                        //
                        obs.Update(speedRef, compass.yaw, x_ned, y_ned);
                        obs.GetCurrent(filterData.current[0], filterData.current[1]);
                        obs.GetSpeed(filterData.speed[0], filterData.speed[1]);
                        obs.GetPosition(x_ned, y_ned);

                        x_utm = y_ned;
                        y_utm = x_ned;

                        GeographicLib::UTMUPS::Reverse(zone, northp, x_utm, y_utm, filterData.latitude,
                            filterData.longitude);
                    } else {
                        filterData.latitude = gpsData.latitude;
                        filterData.longitude = gpsData.longitude;
                    }
                    navfilter_pub->publish(filterData);

                } catch (const GeographicLib::GeographicErr& e) {
                    RCLCPP_ERROR(node->get_logger(), "GeographicLib exception: what = %s", e.what());
                    obs.Reset();
                }

                lastValidGPSTime = gpsData.time;
            }
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    node = nullptr;
    return 0;
}

void ReloadConfig()
{
    static NavFilterConfigData navFilterConfig;
    //useThrusterMap = par_client->get_parameter("UseThrusterMapping", false);
    std::vector<double> gains = par_client->get_parameter("Gains", std::vector<double>(4, 0.0));
    rate = par_client->get_parameter("Rate", 10);

    for (size_t i = 0; i < 4; ++i) {
        navFilterConfig.k[i] = gains.at(i);
    }

    obs.SetConfig(navFilterConfig);
}

void handle_navfilter_commands(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Request> request,
    std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Response> response)
{
    (void)request_header;
    RCLCPP_INFO(node->get_logger(), "Incoming request: %s", CommandTypeToString((CommandType)(request->command_type)).c_str());

    CommandAnswer ret = CommandAnswer::ok;

    switch (request->command_type) {
    case (uint16_t)CommandType::undefined:
        RCLCPP_WARN(node->get_logger(), "CommandType undefined");
        ret = CommandAnswer::fail;
        break;
    case (uint16_t)CommandType::reset:
        obs.Reset();
        break;
    case (uint16_t)CommandType::reloadconfig:
        ReloadConfig();
        break;
    default:
        RCLCPP_WARN(node->get_logger(), "Unsupported Command Code");
        break;
    }
    if (ret != CommandAnswer::ok) {
        response->res = (int16_t)(CommandAnswer::fail);
    } else {
        response->res = (int16_t)(CommandAnswer::ok);
    }
}

void controlcontext_cb(const ulisse_msgs::msg::ControlContext::SharedPtr msg)
{
    controlData = *msg;
}

/*void positioncontext_cb(const ulisse_msgs::msg::PositionContext::SharedPtr msg)
{
    positionData = *msg;
}*/

void compass_cb(const ulisse_msgs::msg::Compass::SharedPtr msg)
{
    compass = *msg;
}

void gpsdata_cb(const ulisse_msgs::msg::GPSData::SharedPtr msg)
{
    gpsData = *msg;
}
