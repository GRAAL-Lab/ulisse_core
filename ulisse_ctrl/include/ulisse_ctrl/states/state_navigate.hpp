#ifndef ULISSE_CTRL_STATENAVIGATE_HPP
#define ULISSE_CTRL_STATENAVIGATE_HPP

#include "sisl.h"
#include "ulisse_ctrl/states/genericstate.hpp"

namespace ulisse {

namespace states {

    class StateNavigate : public GenericState {

    protected:
        std::shared_ptr<ikcl::AlignToTarget> alignToTargetTask_;
        std::shared_ptr<ikcl::ControlCartesianDistance> cartesianDistanceTask_;

        ctb::LatLong centroid_;

        unsigned int numberCurves_;

        double curvilinearAbscissa;

        double currentCurvilinearAbscissa;
        double delta_;
        unsigned int currentCurve;

        bool isCurveSet;
        bool start;
        bool oriented;

        int count;

        ctb::LatLong startingPoint;
        ctb::LatLong endPoint;
        double startingAngle;

        std::vector<SISLCurve*> nurbs_;

        SISLCurve* curve;
        int stat;
        int leftKnot;
        double aepsco = 0.01;
        double aepsge = 0.01;
        double gpar = 0;
        double gpar2 = 0;
        double dist = 0;
        double dist2 = 0;
        double maximumLookupAbscissa;
        double tolleranceEndingPoint;
        double tolleranceStartingAngle;
        double tolleranceStartingPoint;
        double curLength;

        bool useLineOfSight;

        SISLCurve* newCurve_;
        SISLCurve* curve2;
        SISLCurve* newCurve2;
        SISLCurve* resultCurve;

        ctb::LatLong lookAheadPoint;

        double getCurvilinearAbscissa();

    public:
        StateNavigate();
        virtual ~StateNavigate();
        fsm::retval OnEntry() override;
        virtual fsm::retval Execute();
        virtual fsm::retval OnExit();

        void ConfigureStateFromFile(libconfig::Config& confObj) override;

        bool LoadSpur(std::string json_nurbs);
    };
}
}

#endif // ULISSE_CTRL_STATEMOVE_HPP
