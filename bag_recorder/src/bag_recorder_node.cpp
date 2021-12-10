#include <rclcpp/rclcpp.hpp>

#include "bag_recorder/rosbag_recorder.hpp"



int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);

    auto rosbag_recorder = std::make_shared<RosbagRecorder>();

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(rosbag_recorder);
    exe.spin();

    rclcpp::shutdown();

    return 0;
}
