#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <bags_to_csv/offline_bag_converter.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    if(argc < 2){
        std::cerr << "No bag provided" << std::endl;
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }


    std::string bagPath(argv[1]);
    std::cout << "PATH TO BAG FOLDER: " << bagPath << std::endl;

    rclcpp::executors::SingleThreadedExecutor exe;
    auto bag2csv_node = std::make_shared<OfflineBagConverter>(bagPath);
    exe.add_node(bag2csv_node);
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
