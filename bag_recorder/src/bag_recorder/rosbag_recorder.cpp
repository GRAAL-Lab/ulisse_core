#include <ament_index_cpp/get_package_share_directory.hpp>

#include "bag_recorder/rosbag_recorder.hpp"
#include "ulisse_msgs/futils.hpp"
#include "ulisse_msgs/topicnames.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

RosbagRecorder::RosbagRecorder() :
    Node("rosbag_recorder"),
    recording_(false)
{

    std::string ulisse_ctrl_dir = ament_index_cpp::get_package_share_directory("ulisse_ctrl");
    ulisse_ctrl_kcl_conf_ = ulisse_ctrl_dir;
    ulisse_ctrl_kcl_conf_.append("/conf/kcl_ulisse.conf");
    ulisse_ctrl_dcl_conf_ = ulisse_ctrl_dir;
    ulisse_ctrl_dcl_conf_.append("/conf/dcl_ulisse.conf");

    std::string nav_filter_dir = ament_index_cpp::get_package_share_directory("nav_filter");
    nav_filter_conf_ = nav_filter_dir;
    nav_filter_conf_.append("/conf/navigation_filter.conf");

    // TODO!!!! check result of get_package_share... for when simulator is not compiled! (like on the catamaran)
    std::string ulisse_sim_dir = ament_index_cpp::get_package_share_directory("ulisse_sim");
    ulisse_sim_conf_ = ulisse_sim_dir;
    ulisse_sim_conf_.append("/conf/simulator_ulisse.conf");

    trigger_srv_ = this->create_service<ulisse_msgs::srv::RosbagCmd>(ulisse_msgs::topicnames::rosbag_service,
        std::bind(&RosbagRecorder::ServiceHandler, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "%sReady to record Bag.%s", tc::cyan, tc::none);
}

RosbagRecorder::~RosbagRecorder()
{
    // If the process is killed while a recording is in progress we need to cleanup
    if (recording_){
        delete[] argv_new_;
        kill(PID_, 15);  //Sends the SIGINT Signal to the process, telling it to stop.
        RCLCPP_INFO(this->get_logger(), "Recording stopped.");
    }
}

void RosbagRecorder::ServiceHandler(const std::shared_ptr<ulisse_msgs::srv::RosbagCmd::Request> request,
    std::shared_ptr<ulisse_msgs::srv::RosbagCmd::Response> response)
{

    // We start recording only if there is no actual recording
    if (request->record_cmd == 1) {
        if(recording_ == false) {
            std::string home_path = futils::get_homepath();
            std::string current_date = futils::GetCurrentDateFormatted();
            std::string sys_folder;

            if (request->bag_info.size() > 2){
                bag_info_ = request->bag_info;
            } else {
                bag_info_.clear();
            }

            if (request->save_folder.size() > 2){
                sys_folder = request->save_folder;
                if(sys_folder.back() != '/'){
                    sys_folder.append("/");
                }
            } else {
                sys_folder = home_path + "/logs/bag_recorder/";
                futils::MakeDir(home_path + "/logs/bag_recorder");
                RCLCPP_INFO(this->get_logger(), "No save_folder provided, defaulting to: %s", sys_folder.c_str());
            }
            bag_folder_ = sys_folder + "rosbag2_" + current_date;

            if (futils::does_file_exists(sys_folder)) {
                recording_ = true;
                response->res = "Recording Started.";

                PID_ = fork();
                if (PID_ == -1) {
                    perror("[rosbag_recorder] fork error");
                    exit(EXIT_FAILURE);
                } else if (PID_ == 0) {

                    std::vector<std::string> args;
                    args.push_back("bag");
                    args.push_back("record");
                    args.push_back("-a");
                    args.push_back("-o");
                    args.push_back(bag_folder_);
                    argv_new_ = new char*[args.size() + 2];

                    argv_new_[0] = (char*)"ros2";
                    argv_new_[args.size() + 1] = NULL;

                    for(unsigned int c=0; c<args.size(); c++)
                        argv_new_[c+1] = (char*)args[c].c_str();

                    execvp(argv_new_[0], argv_new_);
                    // execvp only returns on error
                    perror("[rosbag_recorder] execvp error");
                    exit(1);
                }

                RCLCPP_INFO(this->get_logger(), "Bag Folder: %s", bag_folder_.c_str());
                RCLCPP_INFO(this->get_logger(), "Recording started.");
            } else {
                response->res = "Save folder does not exist. No bag will be recorded.";
            }
        } else {
            response->res = "Recording already in progress.";
        }

    } else if (request->record_cmd == 0) {
        if (recording_ == true) {
            recording_ = false;
            response->res = "Recording Stopped.";

            futils::MakeDir(bag_folder_ + "/conf");
            futils::CopyFile(ulisse_ctrl_kcl_conf_, bag_folder_ + "/conf/kcl_ulisse.conf");
            futils::CopyFile(ulisse_ctrl_dcl_conf_, bag_folder_ + "/conf/dcl_ulisse.conf");
            futils::CopyFile(nav_filter_conf_, bag_folder_ + "/conf/navigation_filter.conf");
            futils::CopyFile(ulisse_sim_conf_, bag_folder_ + "/conf/simulator_ulisse.conf");

            if (!bag_info_.empty()){
                std::ofstream out(bag_folder_ + "/bag_info.txt");
                out << bag_info_;
            }

            delete[] argv_new_;
            kill(PID_, 15);  //Sends the SIGINT Signal to the process, telling it to stop.
        } else {
            response->res = "No active recording to stop. NOP.";
        }
    }

    response->record_status = recording_;
    RCLCPP_INFO(this->get_logger(), "sending back response: [%s]", response->res.c_str());
}
