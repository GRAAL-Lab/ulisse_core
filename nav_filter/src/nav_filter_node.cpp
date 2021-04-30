#include <rclcpp/rclcpp.hpp>
#include <nav_filter/navigation_filter.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("navigation_filter_node");

    int rate = 10;
    rclcpp::WallRate loop_rate(rate);
    std::string confPath = ament_index_cpp::get_package_share_directory("nav_filter").append("/conf/navigation_filter.conf");
    ulisse::nav::NavigationFilter myNavFilter(node, confPath);

    while (rclcpp::ok()) {

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
