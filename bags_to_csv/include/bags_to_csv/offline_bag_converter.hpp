#ifndef OFFLINEBAGCONVERTER_HPP
#define OFFLINEBAGCONVERTER_HPP

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/clock.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"

class OfflineBagConverter
{
public:
    OfflineBagConverter();

    void ConvertToCSV(const std::string& bag_folder, const std::string& csv_folder);
};

#endif // OFFLINEBAGCONVERTER_HPP
