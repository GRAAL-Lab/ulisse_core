#ifndef ULISSE_CTRL_STATENAVIGATE_HPP
#define ULISSE_CTRL_STATENAVIGATE_HPP

#include "sisl.h"
#include "ulisse_ctrl/states/genericstate.hpp"
#include <ulisse_ctrl/Nurbs.h>

namespace ulisse {

namespace states {

    class StateNavigate : public GenericState {

    protected:
        std::shared_ptr<ikcl::AlignToTarget> alignToTargetTask_;
        std::shared_ptr<ikcl::CartesianDistance> cartesianDistanceTask_;

        ctb::LatLong centroid_;
        bool isCurveSet;
        bool start;
        ctb::LatLong startP, endP;
        double tolleranceStartingPoint;
        ctb::LatLong lookAheadPoint;
        Nurbs nurbsObj_;
        int count;

    public:
        StateNavigate();
        ~StateNavigate() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;

        void ConfigureStateFromFile(libconfig::Config& confObj) override;

        bool LoadNurbs(const std::string& nurbs);
    };
}
}

#endif // ULISSE_CTRL_STATEMOVE_HPP
