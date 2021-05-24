#ifndef ULISSE_CTRL_STATEpathfollow_HPP
#define ULISSE_CTRL_STATEpathfollow_HPP

#include "sisl.h"
#include "ulisse_ctrl/states/generic_state.hpp"
#include "ulisse_msgs/msg/path.hpp"
#include <ulisse_ctrl/nurbs.h>

namespace ulisse {

namespace states {

    class StatePathFollow : public GenericState {

    protected:
        //tasks of the state
        std::shared_ptr<ikcl::AlignToTarget> alignToTargetTask_;
        std::shared_ptr<ikcl::CartesianDistance> cartesianDistanceTask_;
        std::shared_ptr<ikcl::CartesianDistance> cartesianDistancePathFollowingTask_;

        bool isCurveSet_; // flag for checking if a curve has been loaded
        bool vehicleOnTrack_; // flag for checking is the robot at the path start
        ctb::LatLong startP_, endP_; // starting and ending point
        ctb::LatLong nextP_; // next point of the path
        double tolleranceStartingPoint_; // tollerance on the starting point
        double tolleranceEndingPoint_; //tollerance on the ending point
        bool logPathOnFile_;

        Nurbs nurbsObj_; //objet to handle the path

    public:
        StatePathFollow();
        ~StatePathFollow() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;

        bool ConfigureStateFromFile(libconfig::Config& confObj) override;

        bool LoadNurbs(const ulisse_msgs::msg::Path& path);
    };
}
}

#endif // ULISSE_CTRL_STATEMOVE_HPP
