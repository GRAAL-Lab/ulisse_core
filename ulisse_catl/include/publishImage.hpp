#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>

#include <json_utils/json_utils.hpp>
#include <mqttt/mqtt_publisher.hpp>
#include <mqttt/paho_publisher.hpp>
#include <stng/stanag4609_conversion.hpp>

#include <ulisse_msgs/msg/nav_filter_data.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace ctljsn;

bool enableDebugPrint = false;

#define OUTPUT_ENCODING_PATH "/home/graal/Documents/leonardo_nausycaa/sw/catkin_stanag/testUlisseCore/" // TODO put as config param