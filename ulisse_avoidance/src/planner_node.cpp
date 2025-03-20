#include "ulisse_avoidance/local_planner.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto plannerNode = std::make_shared<LocalPlannerNode>();

    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(plannerNode);
    exe.spin();

    rclcpp::shutdown();

    return 0;
}