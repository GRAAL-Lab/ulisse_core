#include <iomanip>

#include "ulisse_msgs/topicnames.hpp"
#include "bags_to_csv/offline_bag_converter.hpp"


OfflineBagConverter::OfflineBagConverter(const std::string& bagPath, const std::string& saveFolder)
    : Node("offline_bag2csv_node"), bagPath_(bagPath), saveFolder_(saveFolder)
{

    OpenFiles();
    if(ConvertToCSV()){
        std::cout << "Conversion Successful." << std::endl;
    } else {
        std::cout << "Conversion Failed." << std::endl;
    }
    CloseFiles();

    rclcpp::shutdown();
}

OfflineBagConverter::~OfflineBagConverter(){

}

bool OfflineBagConverter::ConvertToCSV()//, const std::string& csv_folder)
{


    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());

    rosbag2_cpp::StorageOptions storage_options{};
    storage_options.uri = bagPath_;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options{};
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    reader.open(storage_options, converter_options);

    auto topics = reader.get_all_topics_and_types();

    for (auto&& t : topics){
        std::cout << "meta name: " << t.name << std::endl;
        std::cout << "meta type: " << t.type << std::endl;
        //std::cout << "meta serialization_format: " << t.serialization_format << std::endl;
    }

    // Read and deserialize "serialized data"
    while (reader.has_next()) {
        auto bag_message = reader.read_next();

        if (bag_message->topic_name == ulisse_msgs::topicnames::sensor_gps_data) {

            rclcpp::Serialization<ulisse_msgs::msg::GPSData> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(
                &extracted_serialized_msg, &gpsData_);

            /*std::cout << "GPS pos: "  // bag_message->topic_name << ": " <<
                      <<  gpsData_.time << ", " << gpsData_.latitude << ", " << gpsData_.latitude << std::endl;*/
            gpsFile_ << std::fixed << std::setprecision(8) << gpsData_.time << ", " << gpsData_.latitude << ", " << gpsData_.latitude << "\n";

        }

    }

    return true;
}

bool OfflineBagConverter::OpenFiles()
{
    gpsFile_          .open(std::string(saveFolder_ + "/gps.txt"));
    gpsFile_ << "time, lat, long\n";
    sensorsFile_      .open(std::string(saveFolder_ + "/sensors.txt"));
    motorsFile_       .open(std::string(saveFolder_ + "/motors.txt"));
    navFilterFile_    .open(std::string(saveFolder_ + "/nav_filter.txt"));
    controlFile_      .open(std::string(saveFolder_ + "/control.txt"));
    vehicleStatusFile_.open(std::string(saveFolder_ + "/vehicle_status.txt"));

    return true;
}

void OfflineBagConverter::CloseFiles()
{
    gpsFile_          .close();
    sensorsFile_      .close();
    motorsFile_       .close();
    navFilterFile_    .close();
    controlFile_      .close();
    vehicleStatusFile_.close();

}
