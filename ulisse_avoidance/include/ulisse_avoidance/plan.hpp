#ifndef PLAN_HPP
#define PLAN_HPP

#include "ctrl_toolbox/HelperFunctions.h"
#include "rclcpp/rclcpp.hpp"
#include "ulisse_avoidance/data_structs.hpp"
#include "ulisse_avoidance/directory_manager.hpp"
#include "ulisse_msgs/msg/path_data.hpp"

#include <memory>
#include <nlohmann/json.hpp>
#include <vector>

using json = nlohmann::ordered_json;

using oal::VehicleData, oal::PruningParams;

class Plan {
private:
    Eigen::Vector2d goal_;
    ctb::LatLong centroid_;

    ObsPtr currentSupportObs_ = nullptr;
    oal::VxId currentSupportVx_ = oal::NA;

    std::unique_ptr<oal::Generator> gen_;
    std::shared_ptr<DebugUA> debug_;

    rclcpp::Time lastStatusUpdate_;
    oal::PathReport searchReport_;
    std::unique_ptr<DirectoryManager> currentRequestDir_;

    Eigen::Vector2d GetLocal2d(const ctb::LatLong& position) const;

    json LogCurrentSearchData(rclcpp::Time now);

public:
    static VehicleData vhData;
    static PruningParams pParams;

    oal::Pose vhPose;
    std::vector<ObsPtr> obstacles;

    // bool colregs;
    Path path;

    /* Create a new plan */
    Plan(const ctb::LatLong& vhPos,
        const double& vhHeading,
        const ctb::LatLong& goal,
        const ctb::LatLong& centroid,
        const std::vector<ObsPtr>& obs,
        // bool rulesCompliant,
        rclcpp::Time now,
        const std::string& logDirectory,
        std::shared_ptr<DebugUA> debug = nullptr)
        : centroid_(centroid)
        , obstacles(obs)
    //, colregs(rulesCompliant)
    {
        vhPose = oal::Pose(GetLocal2d(vhPos), vhHeading);
        goal_ = GetLocal2d(goal);

        debug_ = debug ? debug : std::make_shared<DebugUA>();
        gen_ = std::make_unique<oal::Generator>(vhData, pParams, debug_->oal);

        std::cerr << "[WARNING] Creating plan with COLREGS: " << pParams.colregsCompliant << std::endl;

        searchReport_ = gen_->FindPath(vhPose, goal_, obstacles, path);

        if (searchReport_.result != oal::SearchResult::FAIL) {
            lastStatusUpdate_ = now;

            std::vector<ObsPtr> surrounding_obs;
            gen_->IsInBB(oal::TimeDouble(0), vhPose.Position(), obstacles, surrounding_obs);
            if (!surrounding_obs.empty())
                currentSupportObs_ = surrounding_obs[0];
        }

        currentRequestDir_ = std::make_unique<DirectoryManager>(logDirectory);
        std::ofstream* outFile = currentRequestDir_->openFile("newPath");
        json pathLog;
        pathLog["Type"] = "new";
        pathLog["SearchStatus"] = LogCurrentSearchData(now);
        pathLog["NewPath"] = { { "Path", path.LogTrace() }, { "SearchResult", searchReport_.result }, { "FailMsg", searchReport_.failMsg } };
        currentRequestDir_->saveToFile(outFile, pathLog.dump(4));
    }

    /* Get msg for kcl */
    ulisse_msgs::msg::PathData GetPathMsg();

    /* Pop reached waypoints from path */
    bool UpdateStatus(const ctb::LatLong& vhPos, const double& wpAcceptanceRadius);

    /* Check if previous path is still safe */
    std::string HandleEnvironmentDifferences(const ctb::LatLong& vhPos,
        const double& vhHeading,
        const std::vector<ObsPtr>& currentObs,
        const double& betterPlanPercThreshold,
        rclcpp::Time now);

    const rclcpp::Time& When() const;
    const oal::PathReport& Report() const;
    const Eigen::Vector2d& Goal() const;
    const ctb::LatLong& Centroid() const;
};

#endif // PLAN_HPP
