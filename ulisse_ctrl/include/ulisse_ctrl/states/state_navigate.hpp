#ifndef ULISSE_CTRL_STATENAVIGATE_HPP
#define ULISSE_CTRL_STATENAVIGATE_HPP

#include "sisl.h"
#include "ulisse_ctrl/states/genericstate.hpp"

namespace ulisse {

namespace states {

    class StateNavigate : public GenericState {

    protected:
        std::shared_ptr<ikcl::AlignToTarget> angularPositionTask_;
        std::shared_ptr<ikcl::ControlCartesianDistance> distanceTask_;

        ctb::LatLong centroid_;

        unsigned int number_of_curves_;

        double curvilinear_abscissa;

        double current_curvilinear_abscissa;
        double delta_;
        unsigned int current_curve;

        bool isCurveSet;
        bool start;
        bool oriented;

        int count;

        ctb::LatLong starting_point;
        ctb::LatLong end_point;
        double starting_angle;

        std::vector<SISLCurve*> nurbs_;

        SISLCurve* curve;
        int stat;
        int leftknot;
        double aepsco = 0.01;
        double aepsge = 0.01;
        double gpar = 0;
        double gpar2 = 0;
        double dist = 0;
        double dist2 = 0;
        double max_range_abscissa;
        double cruise;
        double tollerance_start_point;
        double tollerance_start_angle;
        double tollerance_end_point;
        double cur_length;

        bool use_line_of_sight;

        SISLCurve* newcurve;
        SISLCurve* curve2;
        SISLCurve* newcurve2;
        SISLCurve* result_curve;

        ctb::LatLong lookAheadPoint;

        double getCurvilinearAbscissa();

    public:
        StateNavigate();
        virtual ~StateNavigate();
        fsm::retval OnEntry() override;
        virtual fsm::retval Execute();
        virtual fsm::retval OnExit();

        void SetAngularPositionTask(std::shared_ptr<ikcl::AlignToTarget> angularPositionTask);
        void SetDistanceTask(std::shared_ptr<ikcl::ControlCartesianDistance> distanceTask);

        void SetLineOfSightMethod(bool status);
        bool GetLineOfSightMethod();

        void SetMaxRangeAbscissa(double max_range);
        void SetDelta(double delta);
        void SetCruiseControl(double cruise_control);
        void SetTolleranceStartingPoint(double toll_start_point);
        void SetTolleranceEndingPoint(double toll_end_point);
        void SetTolleranceStartingAngle(double toll_start_angle);

        double GetMaxRangeAbscissa();
        double GetDelta();
        double GetCruiseControl();
        double GetTolleranceStartingPoint();
        double GetTolleranceEndingPoint();
        double GetTolleranceStartingAngle();

        bool LoadSpur(std::string json_nurbs);
    };
}
}

#endif // ULISSE_CTRL_STATEMOVE_HPP
