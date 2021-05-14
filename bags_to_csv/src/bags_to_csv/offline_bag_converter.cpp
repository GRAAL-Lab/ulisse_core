#include <iostream>
#include <cstdio>

#include "std_msgs/msg/string.hpp"

#include "bags_to_csv/futils.h"
#include "bags_to_csv/offline_bag_converter.hpp"

OfflineBagConverter::OfflineBagConverter(const std::string& bagPath)
    : Node("offline_bag2csv_node"), bagPath_(bagPath)
{
    std::string homePath = futils::get_homepath();
    std::string csvPath = homePath + "/logs/csv";
    futils::MakeDir(csvPath.c_str());

    ConvertToCSV(bagPath, csvPath);

    rclcpp::shutdown();
}

OfflineBagConverter::~OfflineBagConverter(){

}

bool OfflineBagConverter::ConvertToCSV(const std::string& bag_folder, const std::string& csv_folder)
{
    //auto rosbag_directory = rcpputils::fs::path(bag_folder);
    if(!futils::does_file_exists(bag_folder)){
        std::cerr << "Bag file does not exists" << std::endl;
        return false;
    }

    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());

    rosbag2_cpp::StorageOptions storage_options{};
    storage_options.uri = bag_folder;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options{};
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    reader.open(storage_options, converter_options);

    auto topics = reader.get_all_topics_and_types();

    for (auto&& t : topics){
        std::cout << "meta name: " << t.name << std::endl;
        std::cout << "meta type: " << t.type << std::endl;
        std::cout << "meta serialization_format: " << t.serialization_format << std::endl;
    }

    // read and deserialize "serialized data"
    /*while (reader.has_next()) {
        auto bag_message = reader.read_next();

        std::cout<<"Found topic name " << bag_message->topic_name << std::endl;

        using TopicMsgT = std_msgs::msg::String;
        if (bag_message->topic_name == "/topic") {

            TopicMsgT extracted_test_msg;
            rclcpp::Serialization<TopicMsgT> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(
                &extracted_serialized_msg, &extracted_test_msg);

            std::cout<<"Found data in topic " << bag_message->topic_name << ": " << extracted_test_msg.data << std::endl;

        }

    }*/

    return true;
}
