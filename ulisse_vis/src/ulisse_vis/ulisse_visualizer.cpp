//#include <cmath>
//#include <iomanip>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>

//#include "GeographicLib/UTMUPS.hpp"

#include "ulisse_vis/ulisse_visualizer.hpp"
//#include "rov_sim/simulator_defines.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "ulisse_msgs/topicnames.hpp"

#include "ctrl_toolbox/DataStructs.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace ulisse {
using namespace std::chrono_literals;
using std::placeholders::_1;

VehicleVisualizer::VehicleVisualizer(const std::string file_name)
    : Node("simulator_node1")
    
{
    config_ = std::make_shared<ulisse::VisualizerConfiguration>();

    if (!LoadConfiguration(file_name)) {
        exit(EXIT_FAILURE);
    }

    std::cout << "centroid" << centroidLocation_ << std::endl;

    t_start_ = t_last_ = t_now_ = std::chrono::system_clock::now();

    navDataSub_ = this->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data,1, std::bind(&VehicleVisualizer::NavDataCB, this, _1));
    visualizationPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray> ("visualization_marker_array", 0 );

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_broadcaster_ROV = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    ObstacleSub_ = this->create_subscription<ulisse_msgs::msg::Obstacle>(ulisse_msgs::topicnames::obstacle, 10,
                                                                                   std::bind(&VehicleVisualizer::ObstacleCB, this, _1));

    obstaclesVector_.resize(0);
    obstacleMsg = false;

    // Main function timer
    int msRunPeriod = 1.0 / (config_->rate) * 1000;
    // std::cout << "Controller Rate: " << rate << "Hz" << std::endl;
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&VehicleVisualizer::Run, this));
    std::cout << "set time " << "\n";

}

bool VehicleVisualizer::LoadConfiguration(const std::string file_name)
{
    libconfig::Config confObj;


    ///////////////////////////////////////////////////////////////////////////////
    /////       LOAD CONFIGURATION FROM NAV FILTER TO READ CENTROID
    ///
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nav_filter_rov");
    std::string confPath = package_share_directory;
    confPath.append("/conf/navigation_filter_rov.conf");

    std::cout << "PATH TO NAV_FILTER CONF FILE : " << confPath << std::endl;

    // Read the file. If there is an error, report it and exit.
    try {
        confObj.readFile(confPath.c_str());
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        return -1;
    } catch (const libconfig::ParseException& pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
        return -1;
    }

    //acquired the centroid location
    Eigen::VectorXd centroidLocationTmp;
    if (!ctb::GetParamVector(confObj, centroidLocationTmp, "centroidLocation")) {
        std::cerr << "Failed to load centroidLocation from file" << std::endl;
        return false;
    };

    centroidLocation_ = ctb::LatLong(centroidLocationTmp[0], centroidLocationTmp[1]);
    ctb::LatLong2LocalUTM(centroidLocation_, 0.0, centroidLocation_, centerUTM_);

    ///////////////////////////////////////////////////////////////////////////////
    /////       LOAD SIMULATOR CONFIGURATION
    ///
    libconfig::Config confObjSim;
    confPath = (ament_index_cpp::get_package_share_directory("rov_vis")).append("/conf/").append(file_name);

    std::cout << "PATH TO SIMULATOR CONF FILE : " << confPath << std::endl;

    try {
        confObjSim.readFile(confPath.c_str());
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        return -1;
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return -1;
    }


    if (!config_->ConfigureFromFile(confObjSim)){
        std::cerr << "Simulator node: Failed to load config params from files" << std::endl;
        return false;
    }

    return true;
}

void VehicleVisualizer::Run()
{
    //getchar();
    ExecuteStep();
    //SimulateSensors();
    //PublishSensors();
    UpdateFrames();
    PublishTf();
    VisualizeObstacles();
}

void VehicleVisualizer::ExecuteStep()
{
    // We reset the motor reference in case we don't receive any message for more than one second
    /*
    if (motorTimeout_.Elapsed() > 1.0) {
        hp_ = hs_ = 0.0;
    }

    std::clamp(hp_, -100.0, 100.0);
    std::clamp(hs_, -100.0, 100.0);
    */

    if (realTime_) {
        t_now_ = std::chrono::system_clock::now();
    } else {
        t_now_ = t_now_ + std::chrono::milliseconds(static_cast<long>(Ts_ * 1000.0));
    }

    iter_elapsed_ = std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_ - t_last_);
    total_elapsed_ = std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_ - t_start_);

    if (realTime_) {
        Ts_ = iter_elapsed_.count() / 1E9;
    } else {
        Ts_ = Ts_fixed_;
    }

    //SimulateActuation();

    t_last_ = t_now_;
    //previous_bodyF_orientation_ = bodyF_orientation_;
    //vehiclePreviousPos_ = vehiclePos_;
    //Pre_altitude_ = altitude_;
    //ROVprepose_ = ROVpose_;
}

void VehicleVisualizer::AssignMessage(std::array<double,6>& msg,const Eigen::Vector6d& vector){
    for(unsigned long i=0; i < msg.size(); i++){
        msg[i] = vector(i);
    }
}

double VehicleVisualizer::GetCurrentTimeStamp() const
{
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
    return static_cast<double>(now_nanosecs / 1E9);
}

void VehicleVisualizer::PublishTf(){
/*
    t_stamp.header.stamp = this->get_clock()->now();
    t_stamp.header.frame_id = "world";
    t_stamp.child_frame_id = "centroid";
    t_stamp.transform.translation.x = centerUTM_(0);
    t_stamp.transform.translation.y = centerUTM_(1);
    t_stamp.transform.translation.z = centerUTM_(2);
    t_stamp.transform.rotation.x = 1.0;
    t_stamp.transform.rotation.y = 0.0;
    t_stamp.transform.rotation.z = 0.0;
    t_stamp.transform.rotation.w = 0.0;
    tf_broadcaster_->sendTransform(t_stamp);

*/
    //Eigen::Vector3d ASVpos;
    //ctb::LatLong2LocalUTM(vehiclePos_, altitude_, centroidLocation_, ASVpos);

    Eigen::Vector3d ASVpos;
    ctb::LatLong ASVposLatLong(navData_.inertialframe_linear_position.latlong.latitude, navData_.inertialframe_linear_position.latlong.longitude);
    double altitude = navData_.inertialframe_linear_position.altitude;
    ctb::LatLong2LocalUTM(ASVposLatLong, altitude, centroidLocation_, ASVpos);


    tf2::Quaternion q1;
    //q1.setRPY(bodyF_orientation_.Roll(),bodyF_orientation_.Pitch(),bodyF_orientation_.Yaw());
    q1.setRPY(navData_.bodyframe_angular_position.roll, navData_.bodyframe_angular_position.pitch, navData_.bodyframe_angular_position.yaw);

    t_stamp.header.stamp = this->get_clock()->now();
    t_stamp.header.frame_id = "world";
    t_stamp.child_frame_id = "ASV";
    t_stamp.transform.translation.x = ASVpos.y(); // inverted
    t_stamp.transform.translation.y = ASVpos.x(); // inverted
    t_stamp.transform.translation.z = ASVpos.z();
    t_stamp.transform.rotation.x = q1.x();
    t_stamp.transform.rotation.y = q1.y();
    t_stamp.transform.rotation.z = q1.z();
    t_stamp.transform.rotation.w = q1.w();
    tf_broadcaster_->sendTransform(t_stamp);

    //Eigen::Quaterniond eq1(worldF_ROV_bodyF_);
    //tf2::Quaternion ASVq;
    //ASVq.setRPY(navData_.bodyframe_angular_position.roll, navData_.bodyframe_angular_position.pitch, navData_.bodyframe_angular_position.yaw);


/*
    t_stamp_ROV.header.stamp = this->get_clock()->now();
    t_stamp_ROV.header.frame_id = "world";
    t_stamp_ROV.child_frame_id = "ROV";
    t_stamp_ROV.transform.translation.x = ROVpose.x();
    t_stamp_ROV.transform.translation.y = ROVpose.y();
    t_stamp_ROV.transform.translation.z = altitude;
    t_stamp_ROV.transform.rotation.x = ROVq.x();
    t_stamp_ROV.transform.rotation.y = ROVq.y();
    t_stamp_ROV.transform.rotation.z = ROVq.z();
    t_stamp_ROV.transform.rotation.w = ROVq.w();
    tf_broadcaster_ROV->sendTransform(t_stamp_ROV);
*/

    //tf2::Quaternion asv_q;
    //rov2_q.setRPY(bodyF_orientation_.Roll(),bodyF_orientation_.Pitch(),bodyF_orientation_.Yaw() + M_PI/2);
    //asv_q.setEuler(navData_.bodyframe_angular_position.pitch, -navData_.bodyframe_angular_position.roll, navData_.bodyframe_angular_position.yaw - M_PI/2);

    visualization_msgs::msg::Marker marker;
    visualization_msgs::msg::MarkerArray markerArray;

    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "asv_link";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;    


    tf2::Quaternion asv_q;
    //rov2_q.setRPY(bodyF_orientation_.Roll(),bodyF_orientation_.Pitch(),bodyF_orientation_.Yaw() + M_PI/2);
    asv_q.setEuler(bodyF_ASVmesh_.Yaw(),bodyF_ASVmesh_.Pitch(),bodyF_ASVmesh_.Roll());
    marker.pose.position.x = ASVpos.y(); // inverted
    marker.pose.position.y = ASVpos.x(); // inverted
    marker.pose.position.z = ASVpos.z();

    marker.pose.orientation.x = asv_q.x();
    marker.pose.orientation.y = asv_q.y();
    marker.pose.orientation.z = asv_q.z();
    marker.pose.orientation.w = asv_q.w();
    marker.scale.x = 0.0005;
    marker.scale.y = 0.0005;
    marker.scale.z = 0.0005;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0; //0
    marker.color.g = 0.678;
    marker.color.b = 0.184; //0

    marker.mesh_resource = "package://ulisse_sim/meshes/ulisse2_simplified.dae";
    markerArray.markers.push_back(marker);

    visualizationPub_->publish(markerArray);

}

void VehicleVisualizer::UpdateFrames(){
    //Compute the worldF_R_bodyF
    Eigen::RotationMatrix Rz, Ry, Rx;
    double roll, pitch, yaw;
    roll = navData_.bodyframe_angular_position.roll;
    pitch = navData_.bodyframe_angular_position.roll;
    yaw = navData_.bodyframe_angular_position.yaw;
    Rz << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;

    Ry << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);

    Rx << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);

    worldF_R_bodyF_ = Rz * Ry * Rx;

    Eigen::RotationMatrix Rx_n, Ry_n, Rz_n;
    Rx_n << 1, 0, 0,
            0, cos(-M_PI/2), -sin(-M_PI/2),
            0, sin(-M_PI/2), cos(-M_PI/2);

    Ry_n << cos(-M_PI/2), 0, sin(-M_PI/2),
            0, 1, 0,
            -sin(-M_PI/2), 0, cos(-M_PI/2);

    Rz_n << cos(-M_PI/2), -sin(-M_PI/2), 0,
        sin(-M_PI/2), cos(-M_PI/2), 0,
        0, 0, 1;

    worldF_ASV_meshF_ = Ry_n *Rz_n* worldF_R_bodyF_   ;
    bodyF_ASVmesh_ = worldF_ASV_meshF_.eulerAngles(2, 1, 0);
}

void VehicleVisualizer::VisualizeObstacles(){
    visualization_msgs::msg::Marker marker;
    visualization_msgs::msg::MarkerArray markerArray;

    for(unsigned long i=0; i< obstaclesVector_.size(); i++){
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = obstaclesVector_[i].id;
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        //marker.action = visualization_msgs::Marker::ADD;
        Eigen::Vector3d Pos_UTM;
        ctb::LatLong Pos_LatLong;
        Pos_LatLong.latitude = obstaclesVector_[i].center.latitude;
        Pos_LatLong.longitude = obstaclesVector_[i].center.longitude;
        ctb::LatLong2LocalUTM(Pos_LatLong, 0.0, centroidLocation_, Pos_UTM);
        marker.pose.position.x = Pos_UTM.y();
        marker.pose.position.y = Pos_UTM.x(); // inverted
        marker.pose.position.z = 0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = obstaclesVector_[i].b_box_dim_x;
        marker.scale.y = obstaclesVector_[i].b_box_dim_y;
        marker.scale.z = 11.0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0; //0
        marker.color.g = 1.0;
        marker.color.b = 1.0; //0

        markerArray.markers.push_back(marker);

    //    obstaclesVector_[i].center.latitude = obstacle_.center.latitude;
    //    obstaclesVector_[i].center.longitude = obstacle_.center.longitude;
    }
    visualizationPub_->publish( markerArray );
}

void VehicleVisualizer::NavDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg) { navData_ = *msg; }

void VehicleVisualizer::ObstacleCB(const ulisse_msgs::msg::Obstacle::SharedPtr msg){
    obstacle_.id = msg->id;
    obstacle_.center.latitude = msg->center.latitude;
    obstacle_.center.longitude = msg->center.longitude;
    obstacle_.b_box_dim_x = msg->b_box_dim_x;
    obstacle_.b_box_dim_y = msg->b_box_dim_y;
    obstacleMsg = true;
    bool ExistsObs = false;
    for(unsigned long i=0; i < obstaclesVector_.size(); i++){
        if(obstacle_.id == obstaclesVector_[i].id){
            obstaclesVector_[i].center.latitude = obstacle_.center.latitude;
            obstaclesVector_[i].center.longitude = obstacle_.center.longitude;
            ExistsObs = true;
        }
    }
    if(!ExistsObs){
        obstaclesVector_.push_back(obstacle_);
    }


    //std::cout << "obstacle_ : " << obstacle_.id <<std::endl;
}


}
