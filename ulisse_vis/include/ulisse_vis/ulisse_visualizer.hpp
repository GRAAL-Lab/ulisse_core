#ifndef VEHICLEVISUALIZER_H
#define VEHICLEVISUALIZER_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <random>

#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
//#include "rov_msgs/msg/pressure_data.hpp"
//#include "rov_msgs/msg/cable_data.hpp"
//#include "rov_msgs/msg/cable_length_reference.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"

//#include "rov_sim/simulator_defines.hpp"
#include "ulisse_vis/visualizer_defines.hpp"
#include "surface_vehicle_model/surfacevehiclemodel.hpp"

#include "ulisse_msgs/msg/simulated_system.hpp"
//#include "ulisse_msgs/msg/simulated_system.hpp"

#include "ulisse_msgs/msg/micro_loop_count.hpp"
//#include "ulisse_msgs/msg/forces.hpp"
//#include "rov_msgs/msg/thrusters_reference.hpp"
#include "ulisse_msgs/msg/obstacle.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

//#include "GeographicLib/Geodesic.hpp"
//#include "eigen3/Eigen/Dense"
//#include "rml/RML.h"

//#include "rov_msgs/srv/user_input.hpp"

#include "ulisse_msgs/topicnames.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "ctrl_toolbox/DataStructs.h"

//#include "visualization_msgs/InteractiveMarker.h"

namespace ulisse {

class VehicleVisualizer : public rclcpp::Node {

    rclcpp::TimerBase::SharedPtr runTimer_;
    double Ts_, Ts_fixed_;
    bool realTime_;
    std::chrono::system_clock::time_point t_start_, t_now_, t_last_;
    std::chrono::nanoseconds iter_elapsed_, total_elapsed_;

    ctb::LatLong vehiclePos_, vehiclePreviousPos_, centroidLocation_;
    Eigen::Vector3d centerUTM_;

    std::shared_ptr<VisualizerConfiguration> config_;

    geometry_msgs::msg::TransformStamped t_stamp_;
    geometry_msgs::msg::TransformStamped t_stamp_ASV_;

    rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr navDataSub_;
    rclcpp::Subscription<ulisse_msgs::msg::SimulatedSystem>::SharedPtr simulatedSysSub_;
    ulisse_msgs::msg::NavFilterData navData_;
    ulisse_msgs::msg::SimulatedSystem simData_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualizationPub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualizationArrayPub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ASV;

    visualization_msgs::msg::Marker ulisseMarker_;
    visualization_msgs::msg::Marker obstacleMarker_;
    visualization_msgs::msg::MarkerArray MarkerArray_;
    Eigen::Vector3d obsUTM_;
    ctb::LatLong obsLatLong_;

    Eigen::Vector3d asvSimUTM_;
    ctb::LatLong asvSimLatLong_;
    tf2::Quaternion asvSimQ_;

    Eigen::Vector3d asvNavUTM_;
    ctb::LatLong asvNavLatLong_;
    tf2::Quaternion asvNavQ_;

    rclcpp::Subscription<ulisse_msgs::msg::Obstacle>::SharedPtr ObstacleSub_;

    Eigen::RotationMatrix worldF_R_bodyF_;
    Eigen::RotationMatrix worldF_ASV_meshF_;
    rml::EulerRPY bodyF_ASVmesh_;

    bool LoadConfiguration(const std::string file_name);

    void NavDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg);
    void SimSystemCB(const ulisse_msgs::msg::SimulatedSystem::SharedPtr msg);
    void ObstacleCB(const ulisse_msgs::msg::Obstacle::SharedPtr msg);

    ulisse_msgs::msg::Obstacle obstacle_; // Obstacle Avoidance
    //std::vector<std::shared_ptr<ulisse_msgs::msg::Obstacle>> obstaclesPointerVector_; // Obstacle Avoidance
    std::vector<ulisse_msgs::msg::Obstacle> obstaclesVector_;
    bool obstacleMsg;


public:
    VehicleVisualizer(const std::string file_name);

    void SetSampleTime(double ts);
    void Run();
    void ExecuteStep();
    void AssignMessage(std::array<double,6>& msg, const Eigen::Vector6d& vector);
    void UpdateFrames();
    void VisualizeASV();
    void VisualizeObstacles(); // Obstacle Avoidance
    void PublishTf();
    void PublishMarkerArray();

    double GetCurrentTimeStamp() const;



};
}

#endif // VehicleVisualizer_H
