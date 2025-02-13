#include "ulisse_avoidance/path_planner.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto plannerNode = std::make_shared<PathPlannerNode>();

    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(plannerNode);
    exe.spin();

    rclcpp::shutdown();

    return 0;
}