#include "rclcpp/rclcpp.hpp"
#include "ulisse_vis/ulisse_visualizer.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    std::string filename = "visualizer_rov.conf";

    auto vehicleVisualizer = std::make_shared<ulisse::VehicleVisualizer>(filename);

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(vehicleVisualizer);
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
