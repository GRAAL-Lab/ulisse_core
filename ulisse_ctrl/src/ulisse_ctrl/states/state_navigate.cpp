#include "ulisse_ctrl/states/state_navigate.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include <jsoncpp/json/json.h>
#include <ulisse_ctrl/geometry_defines.h>
#include <ulisse_ctrl/ulisse_definitions.h>

namespace ulisse {

namespace states {

    StateNavigate::StateNavigate()
    {
        curvilinearAbscissa = 0;
        numberCurves_ = 0;
        isCurveSet = false;
        maximumLookupAbscissa = 0.2;
        delta_ = 2.0;
        tolleranceStartingPoint = 2.0;
        tolleranceStartingAngle = 0.05;
        tolleranceEndingPoint = 1.0;
        useLineOfSight = true;
    }

    StateNavigate::~StateNavigate() {}

    void StateNavigate::ConfigureStateFromFile(libconfig::Config& confObj)
    {

        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& states = root["states"];

        const libconfig::Setting& state = states.lookup(ulisse::states::ID::navigate);
        ctb::SetParam(state, maxHeadingError_, "maxHeadingError");
        ctb::SetParam(state, minHeadingError_, "minHeadingError");
        ctb::SetParam(state, useLineOfSight, "useLineOfSight");
        ctb::SetParam(state, maximumLookupAbscissa, "maximumLookupAbscissa");
        ctb::SetParam(state, delta_, "delta");
        ctb::SetParam(state, tolleranceEndingPoint, "tolleranceEndPoint");
        ctb::SetParam(state, tolleranceStartingAngle, "tolleranceStartingAngle");
        ctb::SetParam(state, tolleranceStartingPoint, "tolleranceStartingPoint");

        //find the max gain for safty task.
        const libconfig::Setting& tasks = root["tasks"];
        const libconfig::Setting& task = tasks.lookup(task::asvSafetyBoundaries);
        ctb::SetParam(task, maxGainSafety_, "gain");
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

        numberCurves_ = 0;

        int dimension = 3;
        int degree = 0;
        unsigned int cv_count = 0;
        unsigned int knot_count = 0;

        double x, y;
        int count = 0;

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

                std::shared_ptr<double[]> weights(new double[cv_count]);

                count = 0;
                for (Json::ArrayIndex i = 0; i < obj["weigths"].size(); i++) {
                    weights[count] = obj["weigths"][i].asDouble();
                    count++;
                }

                std::shared_ptr<double[]> control_points(new double[cv_count * 4]);

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

                std::shared_ptr<double[]> knots(new double[obj["knots"].size()]);

                for (Json::ArrayIndex i = 0; i < obj["knots"].size(); i++) {
                    knots[i] = obj["knots"][i].asDouble();
                    count++;
                }

                SISLCurve* insert_curve = newCurve(
                    static_cast<int>(cv_count), // number of control points
                    degree + 1, // order of spline curve (degree + 1)
                    knots.get(), // pointer to knot vector (parametrization)
                    control_points.get(), // pointer to coefficient vector (control points)
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

                numberCurves_++;
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
        curvilinearAbscissa = 0.0;
        currentCurvilinearAbscissa = 0.0;
        currentCurve = 0;
        start = false;
        oriented = false;
        count = 0;

        curve = nurbs_[0];
        std::shared_ptr<double[]> point_at(new double[3]);
        // Compute the point of the first curve at 0.0.
        s1227(curve, 0, 0.0, &leftKnot, point_at.get(), &stat);

        ctb::Euclidian2MapPoint(point_at, centroid_, startingPoint);
        //        starting_point = ToLatLong(point_at[0], point_at[1]);

        curve = nurbs_[numberCurves_ - 1];
        // Compute the point of the last curve at 1.0.
        s1227(curve, 0, 1.0, &leftKnot, point_at.get(), &stat);

        ctb::Euclidian2MapPoint(point_at, centroid_, endPoint);

        curve = nurbs_[0];
        // Compute the point of the first curve at 0.1.
        s1227(curve, 0, 0.5, &leftKnot, point_at.get(), &stat);

        ctb::LatLong next_point;
        ctb::Euclidian2MapPoint(point_at, centroid_, next_point);

        double dist;
        ctb::DistanceAndAzimuthRad(startingPoint, next_point, dist, startingAngle);

        for (currentCurve = 0; currentCurve < nurbs_.size(); currentCurve++) {
            curve = nurbs_[currentCurve];

            // Estimate curve length
            s1240(curve, aepsge, &curLength, &stat);

            if (curLength < delta_) {
                std::cout << "Delta too high!" << std::endl;
                return fsm::fail;
            }
        }

        //set tasks
        safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(stateCtx_.tasksMap.find(ulisse::task::asvSafetyBoundaries)->second.task);
        absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(stateCtx_.tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentSafety)->second.task);
        cartesianDistanceTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(stateCtx_.tasksMap.find(ulisse::task::asvCartesianDistance)->second.task);
        alignToTargetTask_ = std::dynamic_pointer_cast<ikcl::AlignToTarget>(stateCtx_.tasksMap.find(ulisse::task::asvAngularPosition)->second.task);

        stateCtx_.actionManager->SetAction(ulisse::action::navigate, true);
        return fsm::ok;
    }

    fsm::retval StateNavigate::Execute()
    {
        //SafetyBoundaries task: it's a velocity task base on the distance from the boundaries. The behaviour that has to achive is align to
        //a desired escape directon and to generate a desired velocity. To do this we use the task AbsoluteAxisAlignment to cope with
        //the align behavior activated in function of the internal actiovation function of the safety task.

        safetyBoundariesTask_->VehiclePosition() = stateCtx_.statusCxt->vehiclePos;

        Eigen::MatrixXd Aexternal;

        Aexternal = safetyBoundariesTask_->InternalActivationFunction().maxCoeff() * Aexternal.setIdentity(absoluteAxisAlignmentSafetyTask_->TaskSpace(), absoluteAxisAlignmentSafetyTask_->TaskSpace());

        absoluteAxisAlignmentSafetyTask_->ExternalActivationFunction() = Aexternal;

        absoluteAxisAlignmentSafetyTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
        absoluteAxisAlignmentSafetyTask_->SetDirectionAlignment(safetyBoundariesTask_->AlignVector(), rml::FrameID::WorldFrame);

        //To avoid the case in which the error between the goal heading and the current heading is too big
        //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

        //compute the heading error
        double headingErrorsafety = absoluteAxisAlignmentSafetyTask_->ControlVariable().norm();
        std::cout << "headingErrorsafety: " << headingErrorsafety << std::endl;

        //compute the gain of the cartesian distance
        double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, maxGainSafety_, headingErrorsafety);

        // Set the gain of the cartesian distance task
        safetyBoundariesTask_->TaskParameterGain(taskGainSafety);

        //navigate action
        if (isCurveSet) {
            //Going to the starting point
            if (!start) {
                std::cout << "*** GOING TO INITIAL POINT! ***" << std::endl;
                ctb::DistanceAndAzimuthRad(stateCtx_.statusCxt->vehiclePos, startingPoint, stateCtx_.goalCxt->goalDistance, stateCtx_.goalCxt->goalHeading);

                if (stateCtx_.goalCxt->goalDistance < tolleranceStartingPoint) {
                    count++;
                    if (count > 50) {
                        count = 0;
                        start = true;
                    }
                } else {
                    alignToTargetTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
                    alignToTargetTask_->SetTargetDistance(Eigen::Vector3d(stateCtx_.goalCxt->goalDistance * cos(stateCtx_.goalCxt->goalHeading), stateCtx_.goalCxt->goalDistance * sin(stateCtx_.goalCxt->goalHeading), 0), rml::FrameID::WorldFrame);
                    cartesianDistanceTask_->SetTargetDistance(Eigen::Vector3d(stateCtx_.goalCxt->goalDistance * cos(stateCtx_.goalCxt->goalHeading), stateCtx_.goalCxt->goalDistance * sin(stateCtx_.goalCxt->goalHeading), 0), rml::FrameID::WorldFrame);
                }
            }

            /*            if (start && !oriented) {
                std::cout << "*** ORIENTING! ***" << std::endl;
                if (abs(stateCtx_.statusCxt->vehicleHeading - starting_angle) < tollerance_start_angle) {
                    count++;
                    if (count > 50) {
                        count = 0;
                        oriented = true;
                    }
                } else {
                    angularPositionTask_->SetAlignmentAxis(Eigen::Vector3d(1, 0, 0));
                    angularPositionTask_->SetDistanceToTarget(Eigen::Vector3d(cos(starting_angle), sin(starting_angle), 0), rml::FrameID::WorldFrame);

                    std::cout << "Starting angle " << abs(stateCtx_.statusCxt->vehicleHeading - starting_angle) << std::endl;
                }
            } else*/
            if (start /* && oriented*/) {

                std::cout << "*** STARTING POINT! ***" << std::endl;
                ctb::DistanceAndAzimuthRad(stateCtx_.statusCxt->vehiclePos, endPoint, stateCtx_.goalCxt->goalDistance, stateCtx_.goalCxt->goalHeading);

                curvilinearAbscissa = getCurvilinearAbscissa();

                if (curvilinearAbscissa >= numberCurves_) {
                    std::cout << "*** MISSION FINISHED! ***" << std::endl;
                    isCurveSet = false;
                    fsm_->ExecuteCommand(ulisse::commands::ID::hold);
                } else {
                    currentCurve = static_cast<unsigned int>(floor(curvilinearAbscissa));
                    curve = nurbs_[currentCurve];

                    currentCurvilinearAbscissa = curvilinearAbscissa;
                    if (currentCurvilinearAbscissa > 1) {
                        currentCurvilinearAbscissa -= currentCurve;
                    }

                    if (useLineOfSight) {
                        std::shared_ptr<double[]> point_at(new double[6]);
                        // Compute the point of the first curve at current_curvilinear_abscissa.
                        s1227(curve, 1, currentCurvilinearAbscissa, &leftKnot, point_at.get(), &stat);

                        double tan_angle = atan2(point_at[4], point_at[3]);

                        point_at[0] = point_at[0] + delta_ * cos(tan_angle);
                        point_at[1] = point_at[1] + delta_ * sin(tan_angle);

                        ctb::Euclidian2MapPoint(point_at, centroid_, lookAheadPoint);

                    } else {
                        // Estimate curve length
                        s1240(curve, aepsge, &curLength, &stat);

                        double delta_increment = (delta_ / curLength);
                        double next_curvilinear_abscissa = currentCurvilinearAbscissa + delta_increment;

                        unsigned int next_curve_index = currentCurve;
                        SISLCurve* next_curve;

                        if (next_curvilinear_abscissa > 1) {
                            if (currentCurve == numberCurves_ - 1) {
                                next_curvilinear_abscissa = 1;
                            } else {
                                next_curvilinear_abscissa = next_curvilinear_abscissa - 1;
                                next_curve_index++;
                            }
                        }

                        next_curve = nurbs_[next_curve_index];
                        std::shared_ptr<double[]> point_at(new double[6]);
                        // Compute the point of the first curve at current_curvilinear_abscissa.
                        s1227(next_curve, 1, next_curvilinear_abscissa, &leftKnot, point_at.get(),
                            &stat);

                        Euclidian2MapPoint(point_at, centroid_, lookAheadPoint);
                    }
                    ctb::DistanceAndAzimuthRad(stateCtx_.statusCxt->vehiclePos, lookAheadPoint, stateCtx_.goalCxt->goalDistance, stateCtx_.goalCxt->goalHeading);

                    alignToTargetTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
                    alignToTargetTask_->SetTargetDistance(Eigen::Vector3d(stateCtx_.goalCxt->goalDistance * cos(stateCtx_.goalCxt->goalHeading), stateCtx_.goalCxt->goalDistance * sin(stateCtx_.goalCxt->goalHeading), 0), rml::FrameID::WorldFrame);
                    cartesianDistanceTask_->SetTargetDistance(Eigen::Vector3d(stateCtx_.goalCxt->goalDistance * cos(stateCtx_.goalCxt->goalHeading), stateCtx_.goalCxt->goalDistance * sin(stateCtx_.goalCxt->goalHeading), 0), rml::FrameID::WorldFrame);
                }
            }
        }
        std::cout << "STATE PATH FOLLOWING" << std::endl;
        std::cout << "Curvilinear Abscissa: " << currentCurvilinearAbscissa << std::endl;
        std::cout << "Delta: " << delta_ << std::endl;

        return fsm::ok;
    }

    double StateNavigate::getCurvilinearAbscissa()
    {
        double min_abscissa = curvilinearAbscissa;
        double max_abscissa = numberCurves_ + maximumLookupAbscissa;
        if (max_abscissa > numberCurves_) {
            max_abscissa = numberCurves_;
        }

        std::shared_ptr<double[]> current_point(new double[3]);

        currentCurve = static_cast<unsigned int>(floor(numberCurves_));

        curve = nurbs_[currentCurve];

        ctb::Map2EuclidianPoint(stateCtx_.statusCxt->vehiclePos, centroid_, current_point);

        if (floor(max_abscissa) == floor(min_abscissa) || (floor(max_abscissa) == numberCurves_)) {
            // To select the window part of curv, from min_abscissa to max_abscissa
            double decMinAbscissa, decMaxAbscissa;
            std::modf(min_abscissa, &decMinAbscissa);
            std::modf(min_abscissa, &decMaxAbscissa);
            s1713(curve, decMinAbscissa, (decMaxAbscissa), &newCurve_, &stat);

            // Find the closest point between a curve and a point
            s1957(newCurve_, current_point.get(), 3, aepsco, aepsge, &gpar, &dist, &stat);

            gpar = gpar + floor(min_abscissa);

        } else {
            // To select the last part of first curve, from min_abscissa to 1.
            double decMinAbscissa, decMaxAbscissa;
            std::modf(min_abscissa, &decMinAbscissa);
            std::modf(min_abscissa, &decMaxAbscissa);
            s1713(curve, decMinAbscissa, 1.0, &newCurve_, &stat);

            // Select the second curve
            curve2 = nurbs_[currentCurve + 1];
            // To select the first part of the second curve, from 0.0 to max_abscissa
            s1713(curve2, 0.0, decMaxAbscissa, &newCurve2, &stat);

            // Find the closest point between the first curve and the point
            s1957(curve, current_point.get(), 3, aepsco, aepsge, &gpar, &dist, &stat);

            // Find the closest point between the second curve and the point
            s1957(curve2, current_point.get(), 3, aepsco, aepsge, &gpar2, &dist2, &stat);

            if (dist < dist2) {
                gpar = gpar + floor(min_abscissa);
            } else {
                gpar = gpar2 + floor(max_abscissa);
            }
        }

        return gpar;
    }

    fsm::retval StateNavigate::OnExit()
    {
        curvilinearAbscissa = 0.0;
        currentCurve = 0;
        start = false;
        oriented = false;
        count = 0;
        isCurveSet = false;
        return fsm::ok;
    }

} // namespace states
} // namespace ulisse
