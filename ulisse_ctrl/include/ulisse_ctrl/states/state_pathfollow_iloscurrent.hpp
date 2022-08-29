#ifndef ULISSE_CTRL_STATE_PATHFOLLOWILOSCURRENT_HPP
#define ULISSE_CTRL_STATE_PATHFOLLOWILOSCURRENT_HPP

#include "sisl_toolbox/sisl_toolbox.hpp"
#include "ulisse_ctrl/states/generic_state.hpp"
#include "ulisse_msgs/msg/path_data.hpp"
#include <ulisse_ctrl/path_manager_iloscurrent.hpp>

namespace ulisse {

namespace states {

    class StatePathFollowILOSCurrent : public GenericState {

    protected:
        // Tasks of the state
        std::shared_ptr<ikcl::AlignToTarget> alignToTargetTask_;
        std::shared_ptr<ikcl::CartesianDistance> cartesianDistanceTask_;
        std::shared_ptr<ikcl::CartesianDistance> cartesianDistancePathFollowingTask_;
        std::shared_ptr<ikcl::LinearVelocity> linearVelocityPathFollowingCurrentTask_;
        std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentTask_;
        std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentILOSTask_;

        bool isCurveSet_;               // Flag for checking if a curve has been loaded
        bool vehicleOnTrack_;           // Flag for checking is the robot at the path start
        //ctb::LatLong startP_, endP_;    // Starting and ending point
        ctb::LatLong nextP_;            // Next point of the path
        ctb::LatLong closestP_;            // Closest point from the path ILOS
        double tolleranceStartingPoint_; // Tolerance on the starting point
        double tolleranceEndingPoint_;  // Tolerance on the ending point
        bool logPathOnFile_;
        double minWaterCurrent_, maxWaterCurrent_;

        //ILOS
        //double sigma_y_;
        //double delta_y_;
        double ILOS_Heading2ClosetPoint;
        double ILOS_goalHeading;
        double ILOS_headingError;
        double yReal_;
        bool variableDelta_;

        struct info_{
            double y_;
            double y_int;
            double y_int_dot_;
            double psi_;
            double sigma_y_;
            double delta_y_;
            //double goal_heading_;
        };
        info_ INFO;
        //double info_[5];

        PathManagerILOSCurrent pathManager_;       // Object to handle the path

        double delta_y_;
        double y_;
        double LOS_goalHeading;
        double LOS_headingError;
        //double yReal_;

    public:
        StatePathFollowILOSCurrent();
        ~StatePathFollowILOSCurrent() override;
        fsm::retval OnEntry() override;
        fsm::retval OnExit() override;
        fsm::retval Execute() override;

        bool ConfigureStateFromFile(libconfig::Config& confObj) override;

        bool LoadPath(const ulisse_msgs::msg::PathData& path);
        const ctb::LatLong& GetNextPoint() const { return nextP_; }
        const ctb::LatLong& GetCurrentTrackPoint() const { return pathManager_.CurrentTrackPoint(); }
        double GetDistanceToEnd() const { return pathManager_.DistanceToEnd(); }

        double GetDeltaY() const { return delta_y_; } // ILOS current
        double GetSigmaY() const { return INFO.sigma_y_; } // ILOS current

        double GetY() const { return y_; }
        double GetYint() const { return INFO.y_int; }
        double GetYintDot() const { return INFO.y_int_dot_; }
        double GetPsi() const { return INFO.psi_; }
        double GetHeading2ClosetPoint() const { return ILOS_Heading2ClosetPoint; }
        double GetGoalHeading() const { return LOS_goalHeading; }
        double GetHeadingError() const { return LOS_headingError; }
        double GetYReal() const { return yReal_; }

        bool SetInformation(const double information[], info_& F) const {
            F.y_ = information[0];
            F.y_int = information[1];
            F.y_int_dot_ = information[2];
            F.psi_ = information[3];
            F.delta_y_ = information[4];
            F.sigma_y_ = information[5];
            return true;
        }

    };
}
}

#endif // ULISSE_CTRL_STATE_PATHFOLLOWILOSCURRENT_HPP
