#include "ulisse_ctrl/states/state_navigate.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/helper_functions.hpp"

#include <jsoncpp/json/json.h>

#include <math.h>

#include "sisl.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>

namespace ulisse {

namespace states {

    StateNavigate::StateNavigate()
    {
        curvilinear_abscissa = 0;
        number_of_curves_ = 0;
        centroid_lat_ = 0.0;
        centroid_long_ = 0.0;
        max_range_abscissa = 0.5;

        //TODO: Aumenta quando passiamo a metri
        delta_ = 0.0001;
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
        isCurveSet = true;

        centroid_lat_ = latitude;
        centroid_long_ = longitude;
        number_of_curves_ = num_curves;

        Json::Reader reader;
        Json::Value obj;

        int dimension = 3;
        int degree;
        int cv_count;
        int knot_count = 0;

        double x, y;
        int count = 0;

        double *knots;
        double* control_points;
        double* weights;

        for(std::string c : curves) {

            std::cout << "STARTINGGGGGGGGGGGG" << std::endl;
            reader.parse(c, obj);

            degree = obj["degree"].asInt();
            std::cout << "DEGREE: " << degree << std::endl;

            cv_count = 0;
            for (Json::ArrayIndex i = 0; i<obj["points"].size(); i++) {
                cv_count++;
            }
            std::cout << "CV_COUNT: " << cv_count << std::endl;

            knot_count = 0;
            for (Json::ArrayIndex i = 0; i<obj["knots"].size(); i++) {
                knot_count++;
            }
            std::cout << "KNOT_COUNT: " << knot_count << std::endl;

            weights = new double[cv_count];
            count = 0;
            for (Json::ArrayIndex i = 0; i<obj["weigths"].size(); i++) {
                weights[count] = obj["weigths"][i].asDouble();
                std::cout << "WEIGHT " << count << ": " << weights[count] << std::endl;
                count++;
            }

            control_points = new double[cv_count * 4];
            std::cout << "CONTROL POINTS:  " << cv_count * 4 << std::endl;
            count = 0;
            int weight_index = 0;
            for (Json::ArrayIndex i = 0; i<obj["points"].size(); i++) {
                x = obj["points"][i][0].asDouble();
                y = obj["points"][i][1].asDouble();

                control_points[count] = x * weights[weight_index];
                control_points[count + 1] = y * weights[weight_index];
                control_points[count + 2] = 0;
                control_points[count + 3] = weights[weight_index];
                std::cout << "POINT " << weight_index << "  : " << control_points[count] << " , "  << control_points[count + 1] << " , " << control_points[count + 2] << " , " << control_points[count + 3] << std::endl;
                count+=4;
                weight_index++;
            }
            std::cout << "COUNT A FINE:  " << count << std::endl;

            count = 0;
            knots = new double[knot_count];
            for (Json::ArrayIndex i = 0; i<obj["knots"].size(); i++) {
                knots[count] = obj["knots"][i].asDouble();
                std::cout << "KNOT " << count << ": " << knots[count] << std::endl;
                count++;
            }

            curve = newCurve(           cv_count,                 // number of control points
                                        degree + 1,               // order of spline curve (degree + 1)
                                        knots,                    // pointer to knot vector (parametrization)
                                        control_points,           // pointer to coefficient vector (control points)
                                        2,                        // kind => 2 : NURBS curve
                                        dimension,                // dimension
                                        2);                       // no copying of information, 'borrow' array

            if( obj["reverse"].asInt()){
                // Turn the direction of a curve by reversing the ordering of the coefficients
                s1706(curve);
            }
            if (!curve) {
                std::cout << "MA DIO" << std::endl;
            }
            else{
                std::cout << "YEEEEEEEEEE" << std::endl;
            }

            free(knots);
            free(control_points);
            free(weights);

            nurbs_.push_back(curve);

            std::cout << "FINITO CURVA " << std::endl;
        }
        std::cout << "FINITO TUTTO " << std::endl;

    }

    fsm::retval StateNavigate::OnEntry(){
        actionManager_->SetAction(ulisse::action::navigate, true);
        curvilinear_abscissa = 0.0;
        current_curvilinear_abscissa = 0.0;
        current_curve = 0;
        start = false;
        oriented = false;
        count = 0;

        curve = nurbs_[0];
        double point_at[4];
        // Compute the point of the first curve at 0.0.
        s1227(curve, 0, 0.0, &leftknot, point_at, &stat);
        std::cout << "STARTING POINT : ( " << point_at[0] << " , " << point_at[1] << " ) " << std::endl;

        starting_point.latitude = point_at[0];
        starting_point.longitude = point_at[1];

        curve = nurbs_[number_of_curves_ - 1];
        // Compute the point of the last curve at 1.0.
        s1227(curve, 0, 1.0, &leftknot, point_at, &stat);
        std::cout << "ENDING POINT : ( " << point_at[0] << " , " << point_at[1] << " ) " << std::endl;

        end_point.latitude = point_at[0];
        end_point.longitude = point_at[1];

        curve = nurbs_[0];
        // Compute the point of the first curve at 0.1.
        s1227(curve, 0, 0.1, &leftknot, point_at, &stat);
        starting_angle = atan2(point_at[1] - starting_point.longitude, point_at[0] - starting_point.latitude);

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
                    if(count > 100){
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
                    if(count > 100){
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

                curvilinear_abscissa = getCurvilinearAbscissa() + delta_;

                if (goalCxt_->goalDistance < 2 || curvilinear_abscissa >= number_of_curves_) {
                    std::cout << "*** MISSION FINISHED! ***" << std::endl;
                    isCurveSet = false;
                    asvHoldTask_->SetGoalHold(end_point);
                    fsm_->ExecuteCommand(ulisse::commands::ID::hold);
                }
                else {

                    current_curve = floor(curvilinear_abscissa);
                    curve = nurbs_[current_curve];

                    current_curvilinear_abscissa = curvilinear_abscissa;
                    if(current_curvilinear_abscissa > 1){
                        current_curvilinear_abscissa -= current_curve;
                    }

                    double point_at[4];
                    // Compute the point of the first curve at current_curvilinear_abscissa.
                    s1227(curve, 0, current_curvilinear_abscissa, &leftknot, point_at, &stat);
                    lookAheadPoint = to_lat_long(point_at[0], point_at[1]);

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
        else{

            std::cout << "PERCHE STRACAZZO SEI QUI????" << std::endl;
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
        double min_abscissa = curvilinear_abscissa;
        double max_abscissa = curvilinear_abscissa + max_range_abscissa;
        if(max_abscissa > number_of_curves_)
            max_abscissa = number_of_curves_;

        curve = nurbs_[current_curve];

        current_point[0] = statusCxt_->vehiclePos.latitude;
        current_point[1] = statusCxt_->vehiclePos.longitude;
        current_point[2] = 0;

        if(floor(max_abscissa) == floor(min_abscissa))
        {
            // To select the window part of curv, from min_abscissa to max_abscissa
            s1713(curve, frexp(min_abscissa, &n), frexp(max_abscissa, &n), &newcurve, &stat);

            // Find the closest point between a curve and a point
            s1957(newcurve, current_point, 3, aepsco, aepsge, &gpar, &dist, &stat);

        }
        else{
            // To select the last part of curv, from min_abscissa to 1.0
            s1713(curve, frexp(min_abscissa, &n), 1.0, &newcurve, &stat);
            curve2 = nurbs_[current_curve + 1];
            // To select the first part of curv2, from 0.0 to max_abscissa
            s1713(curve2, 0.0, frexp(max_abscissa, &n), &newcurve2, &stat);

            // To join newcurve and newcurve2 into result_curve
            s1716(newcurve, newcurve2, -1, &result_curve, &stat);

            // Find the closest point between a curve and a point
            s1957(result_curve, current_point, 3, aepsco, aepsge, &gpar, &dist, &stat);
        }
        return gpar;
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
