#include <experimental/filesystem>
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "bags_to_csv/offline_bag_converter.hpp"
#include "ulisse_msgs/futils.hpp"

namespace fs = std::experimental::filesystem;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::string homePath = futils::get_homepath();

    if(argc < 2){
        std::cerr << "argv[1] missing: No input bag provided" << std::endl;
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }

    if(argc < 3){
        std::cerr << "argv[2] missing: Provide csv folder name" << std::endl;
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }

    std::string bagPath(argv[1]);
    //std::cout << "Path to BAG folder: " << bagPath << std::endl;

    if(!futils::does_file_exists(bagPath)){
        std::cerr << "Bag file does not exists!" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Extract Bag Name to create destination csv folder
    std::string bagName = bagPath;
    const size_t last_slash_idx = bagName.find_last_of("/");
    if((bagName.length() - 1) == last_slash_idx)
    {
        bagName.erase(last_slash_idx, last_slash_idx + 1);
    }
    bagName = bagName.substr(bagName.find_last_of("/") + 1);

    std::string csvPath(argv[2]);
    std::string csvSaveFolder = csvPath + "/" + bagName;
    //std::cout << "Save Folder: " << csvSaveFolder << std::endl;
    fs::create_directories(csvSaveFolder);

    if(fs::exists(bagPath + "/conf")){
        fs::copy(bagPath + "/conf", csvSaveFolder + "/conf");
    }

    if(fs::exists(bagPath + "/bag_info.txt")){
        fs::copy(bagPath + "/bag_info.txt", csvSaveFolder);
    }

    rclcpp::executors::SingleThreadedExecutor exe;
    auto bag2csv_node = std::make_shared<OfflineBagConverter>(bagPath, csvSaveFolder);
    exe.add_node(bag2csv_node);
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
