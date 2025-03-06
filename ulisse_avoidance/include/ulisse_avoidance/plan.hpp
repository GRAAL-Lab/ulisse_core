#ifndef SEARCH_HPP
#define SEARCH_HPP

#include "rclcpp/rclcpp.hpp"
#include "ulisse_avoidance/data_structs.hpp"
#include "ulisse_msgs/msg/path_data.hpp"
#include "ctrl_toolbox/HelperFunctions.h"

using oal::VehicleData, oal::PruningParams;

class Plan {
private:

    Eigen::Vector2d goal_;
    ctb::LatLong centroid_;

    std::unique_ptr<oal::Generator> gen_;

    rclcpp::Time pathCreationTime_;
    oal::PathReport searchReport_;

public:

    static VehicleData vhData;
    static PruningParams pParams;

    oal::Pose vhPose;
    std::vector<ObsPtr> obstacles;
    bool colregs;

    Path path;
    
   

    Plan(const ctb::LatLong& vhPos, const double& vhHeading, const ctb::LatLong& goal, const ctb::LatLong& centroid, const std::vector<ObsPtr>& obs, bool rulesCompliant, rclcpp::Time now)
        : centroid_(centroid), obstacles(obs), colregs(rulesCompliant) {

        //TODO colregs
        
        Eigen::Vector3d cartesian;
        ctb::LatLong2LocalUTM(vhPos, 0.0, centroid, cartesian);
        vhPose = oal::Pose(cartesian.head(2), vhHeading);

        ctb::LatLong2LocalUTM(goal, 0.0, centroid, cartesian);
        goal_ = cartesian.head(2);
        
        // std::cerr<<" Planning with vel:\n   ";
        // for(const auto& v : vhData.velocities) std::cerr<<v<<", ";
        // std::cerr<<"\n";

        gen_ = std::make_unique<oal::Generator>(vhData, pParams);
        
        searchReport_ = gen_->FindPath(vhPose, goal_, obstacles, path);
        if (searchReport_.result != oal::SearchResult::FAIL) {
            pathCreationTime_ = now;
        }
    }

    auto When() const -> const rclcpp::Time& {return pathCreationTime_;};
    auto Report() const -> const oal::PathReport& {return searchReport_;};

    bool IsValid(const ctb::LatLong& vhPos, const double& vhHeading, const std::vector<ObsPtr>& new_obs, Eigen::Vector2d& unreachableWp) {
        Eigen::Vector3d cartesian;
        ctb::LatLong2LocalUTM(vhPos, 0.0, centroid_, cartesian);
        auto vhPoseNew = oal::Pose(cartesian.head(2), vhHeading);
        return gen_->IsPathValid(path, vhPoseNew, new_obs, unreachableWp);
    }

    bool ExistBetterPlan(double betterPlanPercThreshold, const ctb::LatLong& vhPos, const double& vhHeading, const std::vector<ObsPtr>& obs, bool rulesCompliant, rclcpp::Time now){
        Eigen::Vector3d cartesian;
        ctb::LatLong2LocalUTM(vhPos, 0.0, centroid_, cartesian);
        auto vhPoseNew = oal::Pose(cartesian.head(2), vhHeading);
        
        Path tentative;
        auto out = gen_->FindPath(vhPose, goal_, obs, tentative);
        if (out.result == oal::SearchResult::FAIL) return false;

        if(path.Length(vhPoseNew.Position())*betterPlanPercThreshold >= tentative.Length(vhPoseNew.Position())){
            vhPose = vhPoseNew;
            obstacles = obs;
            searchReport_ = out;
            path = tentative;
            pathCreationTime_ = now;
            return true;
        }
        return false;
    }

    ulisse_msgs::msg::PathData GetPathMsg() {
        auto msg = ulisse_msgs::msg::PathData();
        auto coordinate = ulisse_msgs::msg::LatLong();
        msg.id = "AvoidancePath";
        msg.type = "PointPath";
        msg.centroid.latitude = centroid_.latitude;
        msg.centroid.longitude = centroid_.longitude;

        //Adding starting point
        ctb::LatLong startingPos;
        double alt;
        ctb::LocalUTM2LatLong(Eigen::Vector3d(vhPose.Position().x(), vhPose.Position().y(), 0), centroid_, startingPos, alt);
        coordinate.latitude = startingPos.latitude;
        coordinate.longitude = startingPos.longitude;
        msg.coordinates.push_back(coordinate);

        double path_abscissa = 0;
        Eigen::Vector2d vh_pos = vhPose.Position();
        for(const auto& wp : path.Data()){
            Eigen::Vector3d cartesian(wp->data.position.x(), wp->data.position.y(), 0);
            double alt;
            ctb::LatLong position;
            ctb::LocalUTM2LatLong(cartesian, centroid_, position, alt);
            
            coordinate.latitude = position.latitude;
            coordinate.longitude = position.longitude;
            msg.coordinates.push_back(coordinate);

            path_abscissa += (wp->data.position - vh_pos).norm();
            vh_pos = wp->data.position;
            msg.velocities.push_back(wp->data.approachingSpeed);
            std::cerr<<"\n speed ref: "<<wp->data.approachingSpeed<<"\n";
            msg.velocities_abscissas.push_back(path_abscissa);            
        }
        return msg;
    }

    bool UpdateStatus(const ctb::LatLong& vhPos, const double& wpAcceptanceRadius){
        Eigen::Vector3d cartesian;
        ctb::LatLong2LocalUTM(vhPos, 0.0, centroid_, cartesian);
        if((cartesian.head(2) - path.Data().front()->data.position).norm() < wpAcceptanceRadius){
            path.Data().pop_front();
            if(path.Data().empty()) return true; //Reached end
        }
        return false;
    }

    // std::shared_ptr<sisl::Path> GetPath(){
    //         std::vector<Eigen::Vector3d> polyVerticesUTM(path.Data().size());
    //         int i{0};
    //         for (const auto &coord : path.Data()) {
    //                 polyVerticesUTM.at(i) = coord->data.position;
    //                 polyVerticesUTM.at(i)(2) = 0.0;
    //                 i++;
    //         }
            
    //         return sisl::PathFactory::NewPolygonalChain(sisl::Path::Direction::Forward, polyVerticesUTM);
    // }
};



// string id
// string type             # PolyPath, PolyLine
// LatLong[] coordinates   # Coordinates of path
// LatLong centroid
// float64[] velocities 
// float64[] velocities_abscissas
// string polypath_type    # Serpentine, RaceTrack
// float64 size_1
// float64 size_2
// float64 angle
// bool direction


#endif