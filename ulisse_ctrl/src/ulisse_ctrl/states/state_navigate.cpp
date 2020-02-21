#include "ulisse_ctrl/states/state_navigate.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/helper_functions.hpp"

#include <jsoncpp/json/json.h>

#include <cmath>

#include "sisl.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace ulisse {

namespace states {

    StateNavigate::StateNavigate()
    {
        curvilinear_abscissa = 0;
        number_of_curves_ = 0;
        isCurveSet = false;

        max_range_abscissa = 0.2;
        delta_ = 2.0;
        cruise = 1.0;
        tollerance_start_point = 2.0;
        tollerance_start_angle = 0.05;
        tollerance_end_point = 1.0;

        use_line_of_sight = true;
    }

    StateNavigate::~StateNavigate() {}

    void StateNavigate::SetAngularPositionTask(std::shared_ptr<ikcl::AlignToTarget> angularPositionTask)
    {
        angularPositionTask_ = angularPositionTask;
    }

    void StateNavigate::SetDistanceTask(std::shared_ptr<ikcl::ControlCartesianDistance> distanceTask)
    {
        distanceTask_ = distanceTask;
    }

    void StateNavigate::SetMaxRangeAbscissa(double max_range)
    {
        max_range_abscissa = max_range;
    }

    void StateNavigate::SetDelta(double delta)
    {
        delta_ = delta;
    }

    void StateNavigate::SetCruiseControl(double cruise_control)
    {
        cruise = cruise_control;
    }

    void StateNavigate::SetTolleranceStartingPoint(double toll_start_point)
    {
        tollerance_start_point = toll_start_point;
    }

    void StateNavigate::SetTolleranceEndingPoint(double toll_end_point)
    {
        tollerance_end_point = toll_end_point;
    }

    void StateNavigate::SetTolleranceStartingAngle(double toll_start_angle)
    {
        tollerance_start_angle = toll_start_angle;
    }

    void StateNavigate::SetLineOfSightMethod(bool status)
    {
        use_line_of_sight = status;
    }

    bool StateNavigate::GetLineOfSightMethod()
    {
        return use_line_of_sight;
    }

    double StateNavigate::GetMaxRangeAbscissa()
    {
        return max_range_abscissa;
    }

    double StateNavigate::GetDelta()
    {
        return delta_;
    }

    double StateNavigate::GetCruiseControl()
    {
        return cruise;
    }

    double StateNavigate::GetTolleranceStartingPoint()
    {
        return tollerance_start_point;
    }

    double StateNavigate::GetTolleranceEndingPoint()
    {
        return tollerance_end_point;
    }

    double StateNavigate::GetTolleranceStartingAngle()
    {
        return tollerance_start_angle;
    }

    bool StateNavigate::LoadSpur(std::string json_nurbs)
    {
        nurbs_.clear();

        Json::Reader reader;
        Json::Value obj, obj_master;

        bool reverse = false;

        reader.parse(json_nurbs, obj_master);
        centroid_.latitude = obj_master["centroid"][0].asDouble();
        centroid_.longitude = obj_master["centroid"][1].asDouble();

        if (obj_master["direction"].asInt()) {
            reverse = true;
        } else {
            reverse = false;
        }

        number_of_curves_ = 0;

        int dimension = 3;
        int degree = 0;
        unsigned int cv_count = 0;
        unsigned int knot_count = 0;

        double x, y;
        int count = 0;

        double* knots;
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

                SISLCurve* insert_curve = newCurve(
                    static_cast<int>(cv_count), // number of control points
                    degree + 1, // order of spline curve (degree + 1)
                    knots, // pointer to knot vector (parametrization)
                    control_points, // pointer to coefficient vector (control points)
                    2, // kind => 2 : NURBS curve
                    dimension, // dimension
                    1); // no copying of information, 'borrow' array

                if (!insert_curve) {
                    std::cout << "Something Goes Wrong in NURBS Parsing" << std::endl;
                    return false;
                }

                if (reverse) {
                    // Turn the direction of a curve by reversing the ordering of the
                    // coefficients
                    s1706(insert_curve);
                }

                nurbs_.push_back(insert_curve);

                delete (knots);
                delete (control_points);
                delete (weights);

                number_of_curves_++;
            }
        } catch (Json::Exception& e) {
            // output exception information
            std::cout << "NURBS Descriptor Error";
            isCurveSet = false;
            return false;
        }

        if (reverse) {
            // Revert the nurbs_ curve
            std::reverse(nurbs_.begin(), nurbs_.end());
        }

        isCurveSet = true;
        return true;
    }

    fsm::retval StateNavigate::OnEntry()
    {
        curvilinear_abscissa = 0.0;
        current_curvilinear_abscissa = 0.0;
        current_curve = 0;
        start = false;
        oriented = false;
        count = 0;

        curve = nurbs_[0];
        double point_at[3];
        // Compute the point of the first curve at 0.0.
        s1227(curve, 0, 0.0, &leftknot, point_at, &stat);

        starting_point = to_lat_long(point_at[0], point_at[1]);

        curve = nurbs_[number_of_curves_ - 1];
        // Compute the point of the last curve at 1.0.
        s1227(curve, 0, 1.0, &leftknot, point_at, &stat);

        end_point = to_lat_long(point_at[0], point_at[1]);

        curve = nurbs_[0];
        // Compute the point of the first curve at 0.1.
        s1227(curve, 0, 0.5, &leftknot, point_at, &stat);

        ctb::LatLong next_point = to_lat_long(point_at[0], point_at[1]);

        double dist;
        ctb::DistanceAndAzimuthRad(starting_point, next_point, dist, starting_angle);

        for (current_curve = 0; current_curve < nurbs_.size(); current_curve++) {
            curve = nurbs_[current_curve];

            // Estimate curve length
            s1240(curve, aepsge, &cur_length, &stat);

            if (cur_length < delta_) {
                std::cout << "Delta too high!" << std::endl;
                return fsm::fail;
            }
        }

        actionManager_->SetAction(ulisse::action::navigate, true);
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
            //Going to the starting point
            if (!start) {
                std::cout << "*** GOING TO INITIAL POINT! ***" << std::endl;
                ctb::DistanceAndAzimuthRad(statusCxt_->vehiclePos, starting_point, goalCxt_->goalDistance, goalCxt_->goalHeading);

                if (goalCxt_->goalDistance < tollerance_start_point) {
                    count++;
                    if (count > 50) {
                        count = 0;
                        start = true;
                    }
                } else {
                    angularPositionTask_->SetAlignmentAxis(Eigen::Vector3d(1, 0, 0));
                    angularPositionTask_->SetDistanceToTarget(Eigen::Vector3d(goalCxt_->goalDistance * cos(goalCxt_->goalHeading), goalCxt_->goalDistance * sin(goalCxt_->goalHeading), 0), rml::FrameID::WorldFrame);
                    distanceTask_->SetDistance(Eigen::Vector3d(goalCxt_->goalDistance * cos(goalCxt_->goalHeading), goalCxt_->goalDistance * sin(goalCxt_->goalHeading), 0), rml::FrameID::WorldFrame);
                }
            }

/*            if (start && !oriented) {
                std::cout << "*** ORIENTING! ***" << std::endl;
                if (abs(statusCxt_->vehicleHeading - starting_angle) < tollerance_start_angle) {
                    count++;
                    if (count > 50) {
                        count = 0;
                        oriented = true;
                    }
                } else {
                    angularPositionTask_->SetAlignmentAxis(Eigen::Vector3d(1, 0, 0));
                    angularPositionTask_->SetDistanceToTarget(Eigen::Vector3d(cos(starting_angle), sin(starting_angle), 0), rml::FrameID::WorldFrame);

                    std::cout << "Starting angle " << abs(statusCxt_->vehicleHeading - starting_angle) << std::endl;
                }
            } else*/ if (start/* && oriented*/) {
                ctb::DistanceAndAzimuthRad(statusCxt_->vehiclePos, end_point, goalCxt_->goalDistance, goalCxt_->goalHeading);

                curvilinear_abscissa = getCurvilinearAbscissa();

                if (curvilinear_abscissa >= number_of_curves_) {
                    std::cout << "*** MISSION FINISHED! ***" << std::endl;
                    isCurveSet = false;
                    fsm_->ExecuteCommand(ulisse::commands::ID::hold);
                } else {
                    current_curve = static_cast<unsigned int>(floor(curvilinear_abscissa));
                    curve = nurbs_[current_curve];

                    current_curvilinear_abscissa = curvilinear_abscissa;
                    if (current_curvilinear_abscissa > 1) {
                        current_curvilinear_abscissa -= current_curve;
                    }

                    if (use_line_of_sight) {
                        double point_at[6];
                        // Compute the point of the first curve at current_curvilinear_abscissa.
                        s1227(curve, 1, current_curvilinear_abscissa, &leftknot, point_at, &stat);

                        double tan_angle = atan2(point_at[4], point_at[3]);

                        lookAheadPoint = to_lat_long(point_at[0] + delta_ * cos(tan_angle), point_at[1] + delta_ * sin(tan_angle));
                    } else {
                        // Estimate curve length
                        s1240(curve, aepsge, &cur_length, &stat);

                        double delta_increment = (delta_ / cur_length);
                        double next_curvilinear_abscissa = current_curvilinear_abscissa + delta_increment;

                        unsigned int next_curve_index = current_curve;
                        SISLCurve* next_curve;

                        if (next_curvilinear_abscissa > 1) {
                            if (current_curve == number_of_curves_ - 1) {
                                next_curvilinear_abscissa = 1;
                            } else {
                                next_curvilinear_abscissa = next_curvilinear_abscissa - 1;
                                next_curve_index++;
                            }
                        }

                        next_curve = nurbs_[next_curve_index];
                        double point_at[6];
                        // Compute the point of the first curve at current_curvilinear_abscissa.
                        s1227(next_curve, 1, next_curvilinear_abscissa, &leftknot, point_at,
                            &stat);

                        lookAheadPoint = to_lat_long(point_at[0], point_at[1]);
                    }
                    ctb::DistanceAndAzimuthRad(statusCxt_->vehiclePos, lookAheadPoint, goalCxt_->goalDistance, goalCxt_->goalHeading);

                    angularPositionTask_->SetAlignmentAxis(Eigen::Vector3d(1, 0, 0));
                    angularPositionTask_->SetDistanceToTarget(Eigen::Vector3d(goalCxt_->goalDistance * cos(goalCxt_->goalHeading), goalCxt_->goalDistance * sin(goalCxt_->goalHeading), 0), rml::FrameID::WorldFrame);
                    distanceTask_->SetDistance(Eigen::Vector3d(goalCxt_->goalDistance * cos(goalCxt_->goalHeading), goalCxt_->goalDistance * sin(goalCxt_->goalHeading), 0), rml::FrameID::WorldFrame);
                }
            }
        }
        std::cout << "STATE PATH FOLLOWING" << std::endl;
        std::cout << "Curvilinear Abscissa: " << current_curvilinear_abscissa << std::endl;
        std::cout << "Delta: " << delta_ << std::endl;
        std::cout << "Cruise Control: " << cruise << std::endl;

        return fsm::ok;
    }

    double StateNavigate::getCurvilinearAbscissa()
    {
        double min_abscissa = curvilinear_abscissa;
        double max_abscissa = curvilinear_abscissa + max_range_abscissa;
        if (max_abscissa > number_of_curves_) {
            max_abscissa = number_of_curves_;
        }

        current_curve = static_cast<unsigned int>(floor(curvilinear_abscissa));

        curve = nurbs_[current_curve];

        current_point = to_meters(statusCxt_->vehiclePos.latitude,
            statusCxt_->vehiclePos.longitude);

        if (floor(max_abscissa) == floor(min_abscissa) || (floor(max_abscissa) == number_of_curves_)) {
            // To select the window part of curv, from min_abscissa to max_abscissa
            double decMinAbscissa, decMaxAbscissa;
            std::modf(min_abscissa, &decMinAbscissa);
            std::modf(min_abscissa, &decMaxAbscissa);
            s1713(curve, decMinAbscissa, (decMaxAbscissa), &newcurve, &stat);

            // Find the closest point between a curve and a point
            s1957(newcurve, current_point, 3, aepsco, aepsge, &gpar, &dist, &stat);

            gpar = gpar + floor(min_abscissa);

        } else {
            // To select the last part of first curve, from min_abscissa to 1.
            double decMinAbscissa, decMaxAbscissa;
            std::modf(min_abscissa, &decMinAbscissa);
            std::modf(min_abscissa, &decMaxAbscissa);
            s1713(curve, decMinAbscissa, 1.0, &newcurve, &stat);

            // Select the second curve
            curve2 = nurbs_[current_curve + 1];
            // To select the first part of the second curve, from 0.0 to max_abscissa
            s1713(curve2, 0.0, decMaxAbscissa, &newcurve2, &stat);

            // Find the closest point between the first curve and the point
            s1957(curve, current_point, 3, aepsco, aepsge, &gpar, &dist, &stat);

            // Find the closest point between the second curve and the point
            s1957(curve2, current_point, 3, aepsco, aepsge, &gpar2, &dist2, &stat);

            if (dist < dist2) {
                gpar = gpar + floor(min_abscissa);
            } else {
                gpar = gpar2 + floor(max_abscissa);
            }
        }

        return gpar;
    }

    ctb::LatLong StateNavigate::to_lat_long(double x, double y)
    {
        double lam = lat_to_m_coeff(centroid_.latitude);
        double lom = lon_to_m_coeff(centroid_.longitude);
        ctb::LatLong p = point_euclidean2map(x, y, centroid_, lam, lom);

        return p;
    }

    double* StateNavigate::to_meters(double latitude, double longitude)
    {
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
        count = 0;
        isCurveSet = false;
        return fsm::ok;
    }

} // namespace states
} // namespace ulisse
