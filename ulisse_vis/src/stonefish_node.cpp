#include "rclcpp/rclcpp.hpp"
#include "ulisse_vis/ulisse_stonefish.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    std::string filename = "visualizer_rov.conf";

    auto stoneFishVisualizer = std::make_shared<ulisse::StoneFishVisualizer>(filename);

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(stoneFishVisualizer);
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
