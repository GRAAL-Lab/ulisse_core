#ifndef ULISSE_CTRL_STATE_PATHFOLLOW_ALOS_HPP
#define ULISSE_CTRL_STATE_PATHFOLLOW_ALOSHPP

#include "sisl_toolbox/sisl_toolbox.hpp"
#include "ulisse_ctrl/states/generic_state.hpp"
#include "ulisse_msgs/msg/path_data.hpp"
#include <ulisse_ctrl/path_manager_alos.hpp>

namespace ulisse {

namespace states {

    class StatePathFollowALOS : public GenericState {

    protected:
        // Tasks of the state
        std::shared_ptr<ikcl::AlignToTarget> alignToTargetTask_;
        std::shared_ptr<ikcl::CartesianDistance> cartesianDistanceTask_;
        std::shared_ptr<ikcl::CartesianDistance> cartesianDistancePathFollowingTask_;

        bool isCurveSet_;               // Flag for checking if a curve has been loaded
        bool vehicleOnTrack_;           // Flag for checking is the robot at the path start
        bool loopPath_;                 // Flag for setting the infinite loop for the path
        //ctb::LatLong startP_, endP_;    // Starting and ending point
        ctb::LatLong nextP_;            // Next point of the path
        double tolleranceStartingPoint_; // Tolerance on the starting point
        double tolleranceEndingPoint_;  // Tolerance on the ending point
        bool logPathOnFile_;

        PathManager pathManager_;       // Object to handle the path

        double delta_y_;
        double y_;
        double ALOS_goalHeading; // ALOS
        double ALOS_headingError; // ALOS
        double yReal_;

    public:
        StatePathFollowALOS();
        ~StatePathFollowALOS() override;
        fsm::retval OnEntry() override;
        fsm::retval OnExit() override;
        fsm::retval Execute() override;

        bool ConfigureStateFromFile(libconfig::Config& confObj) override;

        bool LoadPath(const ulisse_msgs::msg::PathData& path);
        const ctb::LatLong& GetNextPoint() const { return nextP_; }
        const ctb::LatLong& GetCurrentTrackPoint() const { return pathManager_.CurrentTrackPoint(); }
        double GetDistanceToEnd() const { return pathManager_.DistanceToEnd(); }

        double GetDeltaY() const { return delta_y_; } // LOS
        double GetY() const { return y_; }
        double GetGoalHeading() const { return ALOS_goalHeading; }
        double GetHeadingError() const { return ALOS_headingError; }
        double GetYReal() const { return yReal_; }


    };
}
}

#endif // ULISSE_CTRL_STATE_PATHFOLLOW_ALOS_HPP
