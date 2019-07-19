#include "ulisse_ctrl/states/state_navigate.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/helper_functions.hpp"

#include <jsoncpp/json/json.h>

#include <openNURBS/opennurbs.h>

#include <math.h>

namespace ulisse {

namespace states {

    StateNavigate::StateNavigate()
    {
        curvilinear_abscissa = 0;
        number_of_curves_ = 0;
        centroid_lat_ = 0.0;
        centroid_long_ = 0.0;

        //TODO: Aumenta quando passiamo a metri
        delta_ = 0.0001;
        delta_abscissa = 0.0001;
        isCurveSet = false;
    }

    StateNavigate::~StateNavigate()
    {
}

    void StateNavigate::SetAngularVelocityTask(std::shared_ptr<ikcl::AngularVelocity> angularVelocityTask)
    {
        angularVelocityTask_ = angularVelocityTask;
    }

    void StateNavigate::SetAngularPositionTask(std::shared_ptr<ikcl::AngularPosition> angularPositionTask)
    {
        angularPositionTask_ = angularPositionTask;
    }

    void StateNavigate::SetLinearVelocityTask(std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask)
    {
        linearVelocityTask_ = linearVelocityTask;
    }


    void StateNavigate::SetDistanceTask(std::shared_ptr<ikcl::ControlDistance> distanceTask)
    {
        distanceTask_ = distanceTask;
    }

    void StateNavigate::SetASVHoldTask(std::shared_ptr<ikcl::Hold> asvHoldTask)
    {
        asvHoldTask_ = asvHoldTask;
    }

    void StateNavigate::SetASVMakeCurveTask(std::shared_ptr<ikcl::MakeCurve> asvMakeCurveTask)
    {
        asvMakeCurveTask_ = asvMakeCurveTask;
    }

    void StateNavigate::LoadSpur(float latitude, float longitude, int num_curves, std::vector<std::string> curves){

        nurbs_.clear();

        centroid_lat_ = latitude;
        centroid_long_ = longitude;
        number_of_curves_ = num_curves;

        Json::Reader reader;
        Json::Value obj;

        int dimension = 3;
        ON_BOOL32 bIsRational = true;
        int degree;
        int cv_count;
        int knot_count;

        double x, y;

        int count = 0;

        for(std::string c : curves) {

            reader.parse(c, obj);

            degree = obj["degree"].asInt();
            std::cout << "DEGREE: " << degree << std::endl;

            cv_count = 0;
            for (Json::ArrayIndex i = 0; i<obj["points"].size(); i++) {
                cv_count++;
            }
            std::cout << "CV_COUNT: " << cv_count << std::endl;

            knot_count = cv_count + degree - 1;
            ON_SimpleArray<ON_3dPoint> control_points(cv_count);
            ON_SimpleArray<double> weights(cv_count);
            ON_SimpleArray<double> knots(knot_count);

            count = 0;
            for (Json::ArrayIndex i = 0; i<obj["points"].size(); i++) {
                x = obj["points"][i][0].asDouble();
                y = obj["points"][i][1].asDouble();

                control_points[count] = ON_3dPoint(x, y, 0.000);
                std::cout << "POINT " << count << ": " << control_points[count].x << " , "  << control_points[count].y << std::endl;
                count++;
            }

            count = 0;
            for (Json::ArrayIndex i = 0; i<obj["weigths"].size(); i++) {
                weights[count] = obj["weigths"][i].asDouble();
                std::cout << "WEIGHT " << count << ": " << weights[count] << std::endl;
                count++;
            }

            count = 0;
            for (Json::ArrayIndex i = 0; i<obj["knots"].size(); i++) {
                knots[count] = obj["knots"][i].asDouble();
                std::cout << "KNOT " << count << ": " << knots[count] << std::endl;
                count++;
            }

            curve.Create(dimension, bIsRational, degree + 1, cv_count);
            curve.ReserveCVCapacity( cv_count );
            curve.ReserveKnotCapacity( degree+cv_count-1 );
            curve.MakeRational();


            for (int ci = 0; ci < curve.CVCount(); ci++)
            {
                curve.SetCV(ci, control_points[ci]);
                curve.SetWeight(ci, weights[ci]);
            }

            for (int ki = 0; ki < knot_count; ki++)
                curve.m_knot[ki] = knots[ki];

            curve.SetDomain(0.0, 1.0);

            if (curve.IsValid()) {
                curve.SetStartPoint(control_points[0]);
                nurbs_.push_back(curve);
                isCurveSet = true;
            }
            else{
                std::cout << "ERROR" << std::endl;
            }
        }

        ON_SimpleArray<ON_3dPoint> control_points(4);
        ON_SimpleArray<double> weights(4);
        ON_SimpleArray<double> knots(6);

        control_points[0] = ON_3dPoint(44.3935, 8.9462, 0.000);
        control_points[1] = ON_3dPoint(44.3935, 8.9468, 0.000);
        control_points[2] = ON_3dPoint(44.3930, 8.9468, 0.000);
        control_points[3] = ON_3dPoint(44.3930, 8.9462, 0.000);

        weights[0] = 1.0;
        weights[1] = 0.33;
        weights[2] = 0.33;
        weights[3] = 1.0;

        knots[0] = 0.000;
        knots[1] = 0.000;
        knots[2] = 0.000;
        knots[3] = 1.000;
        knots[4] = 1.000;
        knots[5] = 1.000;

        curve.Create(3, bIsRational, 4, 4);
        curve.ReserveCVCapacity( 4 );
        curve.ReserveKnotCapacity( 6 );
        curve.MakeRational();

        for (int ci = 0; ci < 4; ci++)
        {
            curve.SetCV(ci, control_points[ci]);
            curve.SetWeight(ci, weights[ci]);
        }

        for (int ki = 0; ki < 6; ki++)
            curve.m_knot[ki] = knots[ki];


        curve.SetDomain(0.0, 1.0);

        if (curve.IsValid()) {
            curve.SetStartPoint(control_points[0]);
            nurbs_.push_back(curve);
            isCurveSet = true;

            std::cout << "*************  ("  << nurbs_[1].PointAt(0.0).x << " , " <<  nurbs_[1].PointAt(0.0).y << "  ) " << std::endl;
            number_of_curves_++;
        }

    }

    fsm::retval StateNavigate::OnEntry()
    {
        actionManager_->SetAction(ulisse::action::navigate, true);
        curvilinear_abscissa = 0.0;
        current_curvilinear_abscissa = 0.0;
        current_curve = 0;
        start = false;
        oriented = false;
        count = 0;

        ON_3dPoint pt = nurbs_[0].PointAt(0.0);

        starting_point.latitude = pt.x;
        starting_point.longitude = pt.y;

        pt = nurbs_[number_of_curves_ - 1].PointAt(1.0);

        end_point.latitude = pt.x;
        end_point.longitude = pt.y;

        pt = nurbs_[0].PointAt(0.01);
        starting_angle = atan2(pt.y - starting_point.longitude, pt.x - starting_point.latitude);

        return fsm::ok;
    }

    fsm::retval StateNavigate::Execute()
    {
        for (auto& task : unifiedHierarchy_) {
            try {
                task->Update();
            } catch (tpik::ExceptionWithHow& e) {
                std::cerr << "UPDATE TASK EXCEPTION" << std::endl;
                std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
            }
        }

        if (isCurveSet) {
            if(!start){
                std::cout << "*** GOING TO INITIAL POINT! ***" << std::endl;
                std::cout << "*** INIT LAT: " << starting_point.latitude << " LONG: " << starting_point.longitude << std::endl;
                ctb::DistanceAndAzimuthRad(statusCxt_->vehiclePos, starting_point, goalCxt_->goalDistance, goalCxt_->goalHeading);

                if (goalCxt_->goalDistance < 2) {
                    std::cout << "*** START MISSION! ***" << std::endl;
                    count++;
                    if(count > 20){
                        count = 0;
                        start = true;
                    }
                }
                else {
                    angularPositionTask_->SetAngle(Eigen::Vector3d(0, 0, goalCxt_->goalHeading));
                    distanceTask_->SetDistance(Eigen::Vector3d(goalCxt_->goalDistance, 0, 0));
                    asvMakeCurveTask_->Reset();
                }
            }
            if (start && !oriented) {
                std::cout << "*** ORIENTING! ***" << std::endl;
                std::cout << "START ANGLE: " << starting_angle << std::endl;
                if(abs(statusCxt_->vehicleHeading - starting_angle) < 0.05)
                {
                    count++;
                    if(count > 20){
                        count = 0;
                        oriented = true;
                    }
                }
                else
                    angularPositionTask_->SetAngle(Eigen::Vector3d(0, 0, starting_angle));
            }

            else if (start && oriented){
                std::cout << "*** PLAYING MISSION! with curv abs: " << curvilinear_abscissa << std::endl;
                std::cout << "*** END:: LAT: " << end_point.latitude << " LONG: " << end_point.longitude << std::endl;

                ctb::DistanceAndAzimuthRad(statusCxt_->vehiclePos, end_point, goalCxt_->goalDistance,
                                           goalCxt_->goalHeading);

                std::cout << "*** DISTANCE TO GOAL: " << goalCxt_->goalDistance << std::endl;

                next_curvilinear_abscissa = getCurvilinearAbscissa();
                if( (next_curvilinear_abscissa - curvilinear_abscissa) < delta_abscissa){
                    curvilinear_abscissa = curvilinear_abscissa + delta_abscissa;
                }
                else{
                    curvilinear_abscissa = next_curvilinear_abscissa;
                }

                curvilinear_abscissa = next_curvilinear_abscissa + delta_;

                if (goalCxt_->goalDistance < 2 || curvilinear_abscissa >= number_of_curves_) {
                    std::cout << "*** MISSION FINISHED! ***" << std::endl;
                    isCurveSet = false;
                    asvHoldTask_->SetGoalHold(end_point);
                    fsm_->ExecuteCommand(ulisse::commands::ID::hold);
                }
                else {
                    /*
                    current_curve = floor(curvilinear_abscissa);
                    curve = nurbs_[current_curve];
                    current_curvilinear_abscissa = modf(curvilinear_abscissa , NULL);

                    ON_3dPoint pt = curve.PointAt(current_curvilinear_abscissa);

                    ON_3dVector tgt = curve.TangentAt(current_curvilinear_abscissa);
                    lookAheadPoint = to_lat_long(pt.x + tgt.x * delta_, pt.y + tgt.y * delta_);
                    */

                    current_curve = floor(curvilinear_abscissa);
                    curve = nurbs_[current_curve];

                    current_curvilinear_abscissa = curvilinear_abscissa;
                    if(current_curvilinear_abscissa > 1){
                        current_curvilinear_abscissa -= current_curve;
                    }

                    ON_3dPoint pt = curve.PointAt(current_curvilinear_abscissa);
                    lookAheadPoint = to_lat_long(pt.x, pt.y);

                    std::cout << "*** POINTING TO LAT: " << lookAheadPoint.latitude << " , LONG: "
                              << lookAheadPoint.longitude << "   ;" << std::endl;

                    std::cout << "CURVILINEAR ABSCISSA: " << current_curvilinear_abscissa << std::endl;
                    ctb::DistanceAndAzimuthRad(statusCxt_->vehiclePos, lookAheadPoint, goalCxt_->goalDistance,
                                               goalCxt_->goalHeading);

                    angularPositionTask_->SetAngle(Eigen::Vector3d(0, 0, goalCxt_->goalHeading));
                    distanceTask_->SetDistance(Eigen::Vector3d(goalCxt_->goalDistance, 0, 0));
                }
            }
        }

        /*
         * calculate radius and clock/counterclock #@ ascissa curvilinea x
         * asvMakeCurveTask_->SetCurve(radius, ulisse::curves::circle_arc, clockwise);
         * "wait for raggiungere ascissa x
         * calcola prossima
         *
         * parte intera di x = nurbs corrente
         * parte decimale = avanzamento sulla nurb
         */
        /*
        std::cout << "STATE NAVIGATE " << std::endl;

        std::cout << "SPLINES: " << std::endl;

        std::cout << splines_[0].knots[0] << std::endl;

        std::cout << splines_[0].weights[1] << std::endl;

        std::cout << splines_[0].points[1] << std::endl;
         */

        return fsm::ok;
    }

    double StateNavigate::getCurvilinearAbscissa() {
        ON_3dPoint pt = ON_3dPoint(statusCxt_->vehiclePos.latitude, statusCxt_->vehiclePos.longitude, 0);
        double min_abs = curvilinear_abscissa + 0.5/10000;
        double min_dist = 1000;

        double cur_abs = curvilinear_abscissa;
        double index;
        for (int i = 0; i < 10000; i++){
            cur_abs += 0.5/10000;

            if(cur_abs > number_of_curves_)
                break;

            index = cur_abs;
            if(index > 1){
                index -= floor(cur_abs);
            }
            ON_3dPoint cur_pt = nurbs_[floor(cur_abs)].PointAt(index);
            if(cur_pt.DistanceTo(pt) < min_dist){
                min_dist = cur_pt.DistanceTo(pt);
                min_abs = cur_abs;
            }
        }
        return min_abs;
    }

    ctb::LatLong StateNavigate::to_lat_long(double x, double y)  {
        ctb::LatLong latLong;
        latLong.latitude = x;
        latLong.longitude = y;

        return latLong;
    }

    fsm::retval StateNavigate::OnExit()
    {
        curvilinear_abscissa = 0.0;
        current_curve = 0;
        start = false;
        oriented = false;
        isCurveSet = false;
        return fsm::ok;
    }

}
}
