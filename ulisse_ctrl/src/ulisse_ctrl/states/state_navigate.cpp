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
        max_range_abscissa = 0.3;

        //TODO: Aumenta quando passiamo a metri
        delta_ = 0.001;
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

    bool StateNavigate::LoadSpur(std::string json_nurbs){
        nurbs_.clear();

        Json::Reader reader;
        Json::Value obj, obj_master;

        bool reverse = false;
        std::cout << "ARRIVED :" << json_nurbs << std::endl;

        reader.parse(json_nurbs, obj_master);
        centroid_.latitude = obj_master["centroid"][0].asDouble();
        centroid_.longitude = obj_master["centroid"][1].asDouble();

        if( obj["direction"].asInt()){
            std::cout << "DIRECTION TRUE" << std::endl;
            reverse = true;
        }
        else{
            std::cout << "DIRECTION FALSE" << std::endl;
            reverse = false;
        }

        number_of_curves_ = 0;

        int dimension = 3;
        int degree = 0;
        int cv_count = 0;
        int knot_count = 0;

        double x, y;
        int count = 0;

        double *knots;
        double* control_points;
        double* weights;

        try {
            for (Json::Value c : obj_master["curves"]) {

                reader.parse(c.toStyledString(), obj);

                degree = obj["degree"].asInt();

                cv_count = 0;
                for (Json::ArrayIndex i = 0; i < obj["points"].size(); i++) {
                    cv_count++;
                }

                knot_count = 0;
                for (Json::ArrayIndex i = 0; i < obj["knots"].size(); i++) {
                    knot_count++;
                }

                weights = new double[cv_count];
                count = 0;
                for (Json::ArrayIndex i = 0; i < obj["weigths"].size(); i++) {
                    weights[count] = obj["weigths"][i].asDouble();
                    count++;
                }

                control_points = new double[cv_count * 4];
                count = 0;
                int weight_index = 0;
                for (Json::ArrayIndex i = 0; i < obj["points"].size(); i++) {
                    x = obj["points"][i][0].asDouble();
                    y = obj["points"][i][1].asDouble();

                    control_points[count] = x * weights[weight_index];
                    control_points[count + 1] = y * weights[weight_index];
                    control_points[count + 2] = 0;
                    control_points[count + 3] = weights[weight_index];

                    count += 4;
                    weight_index++;
                }

                count = 0;
                knots = new double[knot_count];
                for (Json::ArrayIndex i = 0; i < obj["knots"].size(); i++) {
                    knots[count] = obj["knots"][i].asDouble();
                    count++;
                }

                SISLCurve *insert_curve = newCurve(
                        cv_count,                 // number of control points
                        degree + 1,               // order of spline curve (degree + 1)
                        knots,                    // pointer to knot vector (parametrization)
                        control_points,           // pointer to coefficient vector (control points)
                        2,                        // kind => 2 : NURBS curve
                        dimension,                // dimension
                        1);                       // no copying of information, 'borrow' array


                if (!insert_curve) {
                    std::cout << "SOMETHING GOES WRONG" << std::endl;
                    return false;
                }

                if (reverse) {
                    // Turn the direction of a curve by reversing the ordering of the coefficients
                    s1706(insert_curve);
                }

                nurbs_.push_back(insert_curve);

                delete (knots);
                delete (control_points);
                delete (weights);

                number_of_curves_++;
            }
        }
        catch (Json::Exception& e)
        {
            // output exception information
            isCurveSet = false;
            return false;
            std::cout << "NURBS Descriptor Error";
        }

        if(reverse){
            // Revert the nurbs_ curve
            std::reverse(nurbs_.begin(), nurbs_.end());
        }

        isCurveSet = true;
        return true;
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

        starting_point = to_lat_long(point_at[0], point_at[1]);

        curve = nurbs_[number_of_curves_ - 1];
        // Compute the point of the last curve at 1.0.
        s1227(curve, 0, 1.0, &leftknot, point_at, &stat);

        end_point = to_lat_long(point_at[0], point_at[1]);

        curve = nurbs_[0];
        // Compute the point of the first curve at 0.1.
        s1227(curve, 0, 0.1, &leftknot, point_at, &stat);

        ctb::LatLong next_point = to_lat_long(point_at[0], point_at[1]);

        double dist;
        ctb::DistanceAndAzimuthRad(starting_point, next_point, dist, starting_angle);

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
                    count++;
                    if(count > 50){
                        count = 0;
                        start = true;
                        std::cout << "*** START MISSION! ***" << std::endl;
                    }
                }
                angularPositionTask_->SetAngle(Eigen::Vector3d(0, 0, goalCxt_->goalHeading));
                distanceTask_->SetDistance(Eigen::Vector3d(goalCxt_->goalDistance, 0, 0));
            }
            if (start && !oriented) {
                std::cout << "*** ORIENTING! ***" << std::endl;
                std::cout << "START ANGLE: " << starting_angle << std::endl;
                if(abs(statusCxt_->vehicleHeading - starting_angle) < 0.05)
                {
                    count++;
                    if(count > 50){
                        count = 0;
                        oriented = true;
                        std::cout << "*** ORIENTED! ***" << std::endl;
                    }
                }
                distanceTask_->SetDistance(Eigen::Vector3d(0, 0, 0));
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

                    /*
                    s1240(curve, aepsge, &cur_length, &stat);
                    delta_ = 1.0 / cur_length;
                     */

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


                    /*
                    if(goalCxt_->goalDistance < 0.5){
                        std::cout << "RADDOPPIA DELTA" << std::endl;
                        delta_ *= 2;
                    }

                    if(goalCxt_->goalDistance > 2.0){
                        std::cout << "DIMEZZA DELTA" << std::endl;
                        delta_ /= 2;
                    }
                     */

                    angularPositionTask_->SetAngle(Eigen::Vector3d(0, 0, goalCxt_->goalHeading));
                    distanceTask_->SetDistance(Eigen::Vector3d(goalCxt_->goalDistance, 0, 0));

                    std::cout << std::endl << "************* DISTA: " << goalCxt_->goalDistance << std::endl;
                    std::cout << "************* DELTA: " << delta_<< std::endl;

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
        double min_abscissa = curvilinear_abscissa;
        double max_abscissa = curvilinear_abscissa + max_range_abscissa;
        if(max_abscissa > number_of_curves_) {
            max_abscissa = number_of_curves_;
        }

        current_curve = floor(curvilinear_abscissa);

        curve = nurbs_[current_curve];

        current_point = to_meters(statusCxt_->vehiclePos.latitude, statusCxt_->vehiclePos.longitude);

        if ( (floor(max_abscissa) == floor(min_abscissa)) || ((max_abscissa - floor(max_abscissa)) == 0) )
        {
            // To select the window part of curv, from min_abscissa to max_abscissa
            s1713(curve, DecimalPart(min_abscissa), DecimalPart(max_abscissa), &newcurve, &stat);

            // Find the closest point between a curve and a point
            s1957(newcurve, current_point, 3, aepsco, aepsge, &gpar, &dist, &stat);

            gpar = gpar + floor(min_abscissa);

        }
        else{
            // To select the last part of first curve, from min_abscissa to 1.0
            s1713(curve, DecimalPart(min_abscissa), 1.0, &newcurve, &stat);

            // Select the second curve
            curve2 = nurbs_[current_curve + 1];
            // To select the first part of the second curve, from 0.0 to max_abscissa
            s1713(curve2, 0.0, DecimalPart(max_abscissa), &newcurve2, &stat);

            // Find the closest point between the first curve and the point
            s1957(curve, current_point, 3, aepsco, aepsge, &gpar, &dist, &stat);

            // Find the closest point between the second curve and the point
            s1957(curve2, current_point, 3, aepsco, aepsge, &gpar2, &dist2, &stat);

            if (dist < dist2){
                gpar = gpar + floor(min_abscissa);
            }
            else{
                gpar = gpar2 + floor(max_abscissa);
            }
        }
        return gpar;
    }

    ctb::LatLong StateNavigate::to_lat_long(double x, double y)  {
        double lam = lat_to_m_coeff(centroid_.latitude);
        double lom = lon_to_m_coeff(centroid_.longitude);
        ctb::LatLong p = point_euclidean2map(x, y, centroid_, lam, lom);

        return p;
    }


    double* StateNavigate::to_meters(double latitude, double longitude) {
        double lam = lat_to_m_coeff(centroid_.latitude);
        double lom = lon_to_m_coeff(centroid_.longitude);
        double* p = point_map2euclidean(latitude, longitude, centroid_, lam, lom);

        return p;
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
