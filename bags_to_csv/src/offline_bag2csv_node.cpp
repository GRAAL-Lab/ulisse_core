#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "bags_to_csv/offline_bag_converter.hpp"
#include "bags_to_csv/futils.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::string homePath = futils::get_homepath();
    std::string csvPath = homePath + "/logs/csv";
    futils::MakeDir(csvPath.c_str());

    if(argc < 2){
        std::cerr << "argv[1] missing: No input bag provided" << std::endl;
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }

    if(argc < 3){
        std::cerr << "argv[2] missing: Provide Experiment name" << std::endl;
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }

    std::string bagPath(argv[1]);
    std::cout << "Path to BAG folder: " << bagPath << std::endl;
    std::string bagName = bagPath;
    const size_t last_slash_idx = bagName.find_last_of("\\/");
    if (std::string::npos == last_slash_idx)
    {
        bagName.erase(last_slash_idx, last_slash_idx + 1);
    }
    bagName.substr(bagName.find_last_of("/") + 1);

    /*if(!futils::does_file_exists(bagPath)){
        std::cerr << "Bag file does not exists!" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::string experimentName(argv[2]);
    std::string saveFolder = csvPath + "/" + experimentName + "/" + bagName;
    std::cout << "Save Folder: " << saveFolder << std::endl;
    futils::MakeDir(saveFolder.c_str());


    rclcpp::executors::SingleThreadedExecutor exe;
    auto bag2csv_node = std::make_shared<OfflineBagConverter>(bagPath, saveFolder);
    exe.add_node(bag2csv_node);
    exe.spin();
*/
    rclcpp::shutdown();
    return 0;
}
