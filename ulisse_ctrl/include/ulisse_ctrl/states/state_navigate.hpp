#ifndef ULISSE_CTRL_STATENAVIGATE_HPP
#define ULISSE_CTRL_STATENAVIGATE_HPP

#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_ctrl/states/genericstate.hpp"
#include <memory>

#include "sisl.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>

namespace ulisse {

namespace states {

    class StateNavigate : public GenericState {

    protected:
        std::shared_ptr<ikcl::AngularPosition> angularPositionTask_;
        std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask_;
        std::shared_ptr<ikcl::AngularVelocity> angularVelocityTask_;
        std::shared_ptr<ikcl::Hold> asvHoldTask_;
        std::shared_ptr<ikcl::MakeCurve> asvMakeCurveTask_;
        std::shared_ptr<ikcl::ControlDistance> distanceTask_;

        ctb::LatLong centroid_;

        int number_of_curves_;

        double curvilinear_abscissa;

        double current_curvilinear_abscissa;
        double delta_;
        int current_curve;

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
        double* current_point;
        double max_range_abscissa;
        double cruise;
        double tollerance_start_point;
        double tollerance_start_angle;
        double tollerance_end_point;
        double cur_length;
        SISLCurve* newcurve;
        SISLCurve* curve2;
        SISLCurve* newcurve2;
        SISLCurve* result_curve;

        ctb::LatLong lookAheadPoint;

        double getCurvilinearAbscissa();
        ctb::LatLong to_lat_long(double x, double y);
        double* to_meters(double latitude, double longitude);

    public:
        StateNavigate();
        virtual ~StateNavigate();
        fsm::retval OnEntry() override;
        virtual fsm::retval Execute();
        virtual fsm::retval OnExit();

        void SetAngularPositionTask(std::shared_ptr<ikcl::AngularPosition> angularPositionTask);
        void SetLinearVelocityTask(std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask);
        void SetASVHoldTask(std::shared_ptr<ikcl::Hold> asvHoldTask);
        void SetASVMakeCurveTask(std::shared_ptr<ikcl::MakeCurve> asvMakeCurveTask);
        void SetAngularVelocityTask(std::shared_ptr<ikcl::AngularVelocity> angularVelocityTask);
        void SetDistanceTask(std::shared_ptr<ikcl::ControlDistance> distanceTask);

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
