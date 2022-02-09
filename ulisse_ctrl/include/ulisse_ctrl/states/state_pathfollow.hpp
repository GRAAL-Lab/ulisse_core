#ifndef ULISSE_CTRL_STATE_PATHFOLLOW_HPP
#define ULISSE_CTRL_STATE_PATHFOLLOW_HPP

#include "sisl_toolbox/sisl_toolbox.hpp"
#include "ulisse_ctrl/states/generic_state.hpp"
#include "ulisse_msgs/msg/path_data.hpp"
#include <ulisse_ctrl/path_manager.hpp>

namespace ulisse {

namespace states {

    class StatePathFollow : public GenericState {

    protected:
        // Tasks of the state
        std::shared_ptr<ikcl::AlignToTarget> alignToTargetTask_;
        std::shared_ptr<ikcl::CartesianDistance> cartesianDistanceTask_;
        std::shared_ptr<ikcl::CartesianDistance> cartesianDistancePathFollowingTask_;

        bool isCurveSet_;               // Flag for checking if a curve has been loaded
        bool vehicleOnTrack_;           // Flag for checking is the robot at the path start
        ctb::LatLong startP_, endP_;    // Starting and ending point
        ctb::LatLong nextP_;            // Next point of the path
        double tolleranceStartingPoint_; // Tolerance on the starting point
        double tolleranceEndingPoint_;  // Tolerance on the ending point
        bool logPathOnFile_;

        PathManager pathManager_;       // Object to handle the path

    public:
        StatePathFollow();
        ~StatePathFollow() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;

        bool ConfigureStateFromFile(libconfig::Config& confObj) override;

        bool LoadPath(const ulisse_msgs::msg::PathData& path);
        const ctb::LatLong &GetNextPoint() const { return nextP_; }
    };
}
}

#endif // ULISSE_CTRL_STATE_PATHFOLLOW_HPP
