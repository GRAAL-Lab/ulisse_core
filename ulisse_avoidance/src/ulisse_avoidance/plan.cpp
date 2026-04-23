#include "ulisse_avoidance/plan.hpp"

oal::VehicleData Plan::vhData;
oal::PruningParams Plan::pParams;

const rclcpp::Time& Plan::When() const { return lastStatusUpdate_; }
const oal::PathReport& Plan::Report() const { return searchReport_; }
const Eigen::Vector2d& Plan::Goal() const { return goal_; }
const ctb::LatLong& Plan::Centroid() const { return centroid_; }

Eigen::Vector2d Plan::GetLocal2d(const ctb::LatLong& position) const
{
    Eigen::Vector3d cartesian;
    ctb::LatLong2LocalUTM(position, 0.0, centroid_, cartesian);
    return cartesian.head(2);
}

std::string Plan::HandleEnvironmentDifferences(const ctb::LatLong& vhPos,
    const double& vhHeading,
    const std::vector<ObsPtr>& currentObs,
    const double& betterPlanPercThreshold,
    rclcpp::Time now)
{
    vhPose = oal::Pose(GetLocal2d(vhPos), vhHeading);

    // if (currentObs.size() != obstacles.size()) {
    //     std::cerr << "[WARNING] Checking path on different number of obstacles." << std::endl;
    // }
    obstacles = currentObs;

    // TODO still not getting vx
    //  ObsPtr trespasseObs = nullptr;
    //  std::vector<ObsPtr> surrounding_obs;
    //  gen_->IsInBB(oal::TimeDouble(0), vhPose.Position(), obstacles, surrounding_obs);
    //  if(!surrounding_obs.empty()) trespasseObs = surrounding_obs[0];
    //  if(surrounding_obs.size() > 2){
    //      std::cerr << "[FATAL] Starting in more than one bb" << std::endl;
    //      response.failMsg = "Start position is in more than one BB";
    //      response.result = SearchResult::FAIL;
    //      return false;
    //  }

    Eigen::Vector2d unreachableWaypoint;
    bool isOldSafe = gen_->IsPathValid(path, vhPose, obstacles, unreachableWaypoint, currentSupportObs_, currentSupportVx_);
    // if (!isOldSafe) {
    //     // TEMP
    //     // for (const auto& wp : path.Data()) {
    //     //     wp->Print();
    //     // }
    // }

    Path possiblyBetterPath;
    oal::PathReport newSearchReport;
    newSearchReport = gen_->FindPath(vhPose, goal_, obstacles, possiblyBetterPath);
    bool newPlanFound = (newSearchReport.result != oal::SearchResult::FAIL);

    if (newPlanFound) {
        bool isNewBetter = possiblyBetterPath.Length(vhPose.Position()) <= betterPlanPercThreshold * path.Length(vhPose.Position());
        // std::cerr << "Remaining length: " << path.Length(vhPose.Position()) << std::endl;
        // std::cerr << "New path length: " << possiblyBetterPath.Length(vhPose.Position()) << std::endl;

        if (!isOldSafe || isNewBetter) {
            std::ofstream* outFile = currentRequestDir_->openFile(isOldSafe ? "switchToBetterPlan" : "switchToSaferPlan");

            json switchLog;
            switchLog["Type"] = "switch";
            switchLog["SearchStatus"] = LogCurrentSearchData(now);
            switchLog["PreviousPath"] = { { "Path", path.LogTrace() }, { "SearchResult", searchReport_.result }, { "FailMsg", searchReport_.failMsg } };
            switchLog["NewPath"] = { { "Path", possiblyBetterPath.LogTrace() }, { "SearchResult", newSearchReport.result }, { "FailMsg", newSearchReport.failMsg } };

            currentRequestDir_->saveToFile(outFile, switchLog.dump(4));

            path = possiblyBetterPath;
            searchReport_ = newSearchReport;
            lastStatusUpdate_ = now;

            return isOldSafe ? pathProgress::switchingToBetterPlan : pathProgress::switchingToSaferPlan;
        }
    }

    if (!isOldSafe) {
        std::ofstream* outFile = currentRequestDir_->openFile("interruptedPlan");

        json interruptedPlanLog;
        interruptedPlanLog["Type"] = "fail";
        interruptedPlanLog["SearchStatus"] = LogCurrentSearchData(now);
        interruptedPlanLog["PreviousPath"] = { { "Path", path.LogTrace() }, { "SearchResult", searchReport_.result }, { "FailMsg", searchReport_.failMsg } };

        currentRequestDir_->saveToFile(outFile, interruptedPlanLog.dump(4));

        return pathProgress::interruptedPlan;
    }

    return pathProgress::keepingSamePath;
}

json Plan::LogCurrentSearchData(rclcpp::Time now)
{
    json statusLog;
    statusLog["Goal"] = { { "x", goal_.x() }, { "y", goal_.y() } };
    statusLog["VehiclePose"] = { { "x", vhPose.Position().x() }, { "y", vhPose.Position().y() }, { "heading", vhPose.Heading() }, { "timestamp", now.nanoseconds() } };
    statusLog["VehicleData"] = { { "Velocities", Plan::vhData.velocities }, { "MaxYawRate", Plan::vhData.max_yaw_rate } };
    statusLog["PruningParams"] = { "TODO" };

    json obstaclesLog = json::array();
    for (const auto& obs : obstacles) {
        obstaclesLog.push_back(obs->Log());
    }
    statusLog["Obstacles"] = obstaclesLog;
    return statusLog;
}

ulisse_msgs::msg::PathData Plan::GetPathMsg()
{
    auto msg = ulisse_msgs::msg::PathData();
    auto coordinate = ulisse_msgs::msg::LatLong();
    msg.id = "AvoidancePath";
    msg.type = "PointPath";
    msg.centroid.latitude = centroid_.latitude;
    msg.centroid.longitude = centroid_.longitude;

    // Adding starting point
    ctb::LatLong startingPos;
    double alt;
    ctb::LocalUTM2LatLong(Eigen::Vector3d(vhPose.Position().x(), vhPose.Position().y(), 0), centroid_, startingPos, alt);
    coordinate.latitude = startingPos.latitude;
    coordinate.longitude = startingPos.longitude;
    msg.coordinates.push_back(coordinate);

    double path_abscissa = 0;
    Eigen::Vector2d vh_pos = vhPose.Position();
    for (const auto& wp : path.Data()) {
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
        msg.velocities_abscissas.push_back(path_abscissa);
    }
    return msg;
}

bool Plan::UpdateStatus(const ctb::LatLong& vhPos, const double& wpAcceptanceRadius)
{
    // TODO should I update vhPose? prob unncessary
    // Remove reached waypoints and stop saving them as nodes parents
    //vhPose = oal::Pose(GetLocal2d(vhPos), vhHeading);
    if ((GetLocal2d(vhPos) - path.Data().front()->data.position).norm() < wpAcceptanceRadius) {
        std::cerr << "Reached waypoint" << std::endl;
        currentSupportObs_ = path.Data().front()->data.obs_ptr;
        currentSupportVx_ = path.Data().front()->data.vx;

        // path.Data().front()->reached = true; //TODO before uncomment check every 'front' call
        path.Data().pop_front();
        if (path.Data().empty()) {
            return true; // Reached end
        } else {
            path.Data().front()->Parent() = nullptr;
        }
    }
    return false;
}
