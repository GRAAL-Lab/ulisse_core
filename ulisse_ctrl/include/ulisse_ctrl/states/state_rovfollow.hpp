#ifndef ULISSE_CTRL_STATEROVFOLLOW_HPP
#define ULISSE_CTRL_STATEROVFOLLOW_HPP

#include "ulisse_ctrl/states/generic_state.hpp"
#include <ulisse_ctrl/path_manager.hpp>

namespace ulisse {

namespace states {

    class StateRovFollow : public GenericState {

    protected:
        std::shared_ptr<ikcl::AlignToTarget> alignToTargetTask_;
        std::shared_ptr<ikcl::CartesianDistance> cartesianDistanceTask_;
        std::shared_ptr<ikcl::AlignToTarget> alignToTargetObstacleTask_; // Obstacle
        std::shared_ptr<ikcl::CartesianDistance> cartesianDistanceObstacleTask_; // Obstacle
        std::shared_ptr<ikcl::ObstacleAvoidance> obstacleAvoidanceTask_;
        //std::shared_ptr<ikcl::AbsoluteAxisAlignment> obstacleAlignmentTask_;
        //std::shared_ptr<ikcl::LinearVelocity> obstacleVelocityTask_;
        //std::shared_ptr<ikcl::AlignToTarget> obstacleAlignTask_;

        bool normalZone, normalZoneObs;

        PathManager pathManager_;

    public:
        StateRovFollow();
        ~StateRovFollow() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;

        LatLong goalPosition;
        double goalHeading;
        double goalDistance;
        double goalHeadingObstacle;
        double goalDistanceObstacle;
        double acceptanceRadius;

        LatLong centroidLocation;
        Eigen::Vector3d worldF_obstacle;
        LatLong obstaclePosition;
        double ASV2obstacleHeading;
        double ASV2obstacleDistance;
        double ROV2obstacleHeading;
        double ROV2obstacleDistance;
        //int closestObstacle2ROV_id;

        double obsAltitude;

        double headingError;
        double headingErrorObstacle;
        double cartesianErrorObstacle;

        double minCartesianObstacleError_, maxCartesianObstacleError_, minObstacleZoneRadius, maxObstacleZoneRadius, obstacleGoalAcceptanceRadius;


        bool ConfigureStateFromFile(libconfig::Config& confObj) override;

        void UpdateObstacles();
    };
}
}

#endif // ULISSE_CTRL_STATEROVFOLLOW_HPP
