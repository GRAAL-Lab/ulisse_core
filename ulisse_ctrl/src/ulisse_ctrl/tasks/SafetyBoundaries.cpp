#include "ulisse_ctrl/tasks/SafetyBoundaries.h"
#include "ulisse_ctrl/helper_functions.hpp"

using namespace ctb;
namespace ikcl {

SafetyBoundaries::SafetyBoundaries(std::string taskID, std::shared_ptr<rml::RobotModel> robotModel, std::string frameID)
    : tpik::InequalityTask(taskID, 6, robotModel->GetTotalDOFs(), tpik::InequalityTaskType::None)
    , robotModel_(robotModel)
    , frameID_(frameID)

{
    MAX_THRESHOLD = 5.0;
    MIN_THRESHOLD = 3.0;

    desired_speed_on_turn = 2.5;
}

void SafetyBoundaries::SetTolleranceBellShape(double tollerance)
{
    tollerance_ = tollerance;
}

void SafetyBoundaries::SetPose(std::shared_ptr<Eigen::Vector6d> pose)
{
    pose_shared = pose;
}

void SafetyBoundaries::SetBoundaries(double bound_min, double bound_max)
{
    MAX_THRESHOLD = bound_max;
    MIN_THRESHOLD = bound_min;
}

void SafetyBoundaries::SetDesiredSpeedOnTurning(double des_speed)
{
    desired_speed_on_turn = des_speed;
}

double SafetyBoundaries::GetDesiredSpeedOnTurning()
{
    return desired_speed_on_turn;
}

void SafetyBoundaries::SetGoalContext(const std::shared_ptr<ulisse::GoalContext>& goalCxt)
{
    goalCxt_ = goalCxt;
}

void SafetyBoundaries::Update() throw(tpik::ExceptionWithHow)
{
    CheckInitialization();
    if (!isBoundariesInitialized) {
        tpik::NotInitialziedTaskParameterException jointsLimitException;
        std::string how = "[SafetyBoundaries] The boundaries have not been set, use SetBoundaries() for task "
            + ID_;
        jointsLimitException.SetHow(how);
        throw(jointsLimitException);
    }

    double goalDistance, goalHeading = 0.0;
    double desired_speed, desired_jog;
    ctb::LatLong desired_pose;
    std::shared_ptr<double[]> p(new double[3]);
    LatLong pose;
    pose.latitude = (*pose_shared)(0);
    pose.longitude = (*pose_shared)(1);

    ctb::Map2EuclidianPoint(pose, centroid, p);

    target = DistanceCheck(point_type(p[0], p[1]));

    if (target.gain > 0) {

        LatLong current_pose;

        current_pose.latitude = (*pose_shared)(0);
        current_pose.longitude = (*pose_shared)(1);

        std::shared_ptr<double[]> targetEuclidian(new double[3]);

        targetEuclidian[0] = target.x;
        targetEuclidian[1] = target.y;
        targetEuclidian[2] = 0.0;

        Euclidian2MapPoint(targetEuclidian, centroid, desired_pose);

        ctb::DistanceAndAzimuthRad(current_pose, desired_pose, goalDistance, goalHeading);

        std::cout << "latitutine: " << desired_pose.latitude << "\nlongitude: " << desired_pose.longitude << std::endl;

        desired_speed = desired_speed_on_turn;

        goalHeading = ctb::FilterAngularJump((*pose_shared)(5), goalHeading);
        desired_jog = ulisse::MinimumAngleBetween((*pose_shared)(5), goalHeading);

        desiredVelocity_(3) = desired_speed;
        desiredVelocity_(2) = desired_jog;

    } else {
        target.gain = 0;
    }

    goalCxt_->goalHeadingWithSafety = target.gain * goalHeading + (1 - target.gain) * goalCxt_->goalHeading;

    UpdateInternalActivationFunction();
    UpdateJacobian();
    UpdateReference();
    SaturateReference();
    SaturateReferenceComponentWise();
}

void SafetyBoundaries::UpdateInternalActivationFunction()
{
    Ai_.setIdentity();
    Ai_ = target.gain * Ai_;

    std::cout << Ai_ << std::endl;
    std::cout << "Max Thresh" << MAX_THRESHOLD << std::endl;
    std::cout << "Min Thresh" << MIN_THRESHOLD << std::endl;
}

void SafetyBoundaries::UpdateJacobian() { J_ = robotModel_->GetCartesianJacobian(frameID_).block(0, 0, 6, DoF_); }

void SafetyBoundaries::UpdateReference()
{
    x_dot_ = taskParameter_.gain * target.gain * (desiredVelocity_);
}

bool SafetyBoundaries::InitializePolygon(ctb::LatLong startingPosition, std::string polygonString)
{
    segments.clear();
    centroid = startingPosition;

    point_type p(centroid.latitude, centroid.longitude);
    segment_type seg;

    boost::geometry::read_wkt(polygonString, poly);

    //Generate a list with all the segments
    double x, y;

    for (auto it = boost::begin(boost::geometry::exterior_ring(poly)); (it + 1) != boost::end(boost::geometry::exterior_ring(poly)); ++it) {

        x = (*it).x();

        if (x > coord_max) {
            coord_max = x;
        }
        if (x < coord_min) {
            coord_min = x;
        }

        y = (*it).y();

        if (y > coord_max) {
            coord_max = y;
        }
        if (y < coord_min) {
            coord_min = y;
        }

        point_type p(x, y);

        x = (*(it + 1)).x();
        y = (*(it + 1)).y();
        point_type next(x, y);

        MakeSegments(p, next, seg);

        segments.push_front(seg);
    }

    coord_max = coord_max + abs(coord_min);

    isBoundariesInitialized = true;

    return true;
}

desired_target SafetyBoundaries::DistanceCheck(point_type const& currentPosition)
{

    desired_target target_value = { 0.0, 0.0, 0.0 };
    double d, d_p1, d_p2;
    double m, x_2, y_2, theta;
    std::deque<point_type> output;
    linestring_type l1, l2;

    double x_max, x_min, y_max, y_min;

    point_type nearest_p;

    nearest_p.set<0>(0.0);
    nearest_p.set<1>(0.0);

    double min_d = static_cast<double>(INFINITY);
    bool first = true;

    double count = 0.0;

    //if the current robot position is not in the polygon
    if (!boost::geometry::covered_by(currentPosition, poly)) {
        for (auto i : segments) {

            //distance from the current potion to the i-th segment
            d = boost::geometry::distance(currentPosition, i);

            //
            if (d < min_d) {

                //starting point of the segment
                point_type p1{ boost::geometry::get<0, 0>(i), boost::geometry::get<0, 1>(i) };
                //ending point of the segment
                point_type p2{ boost::geometry::get<1, 0>(i), boost::geometry::get<1, 1>(i) };

                //distance between the current position and the starting point of the segment
                d_p1 = boost::geometry::distance(currentPosition, p1);
                //distance between the current position and the ending point of the segment
                d_p2 = boost::geometry::distance(currentPosition, p2);

                if (d_p1 <= d_p2) {
                    //if
                    nearest_p.set<0>(p1.x());
                    nearest_p.set<1>(p1.y());
                    d = d_p1;
                } else {
                    nearest_p.set<0>(p2.x());
                    nearest_p.set<1>(p2.y());
                    d = d_p2;
                }

                x_min = p1.x() < p2.x() ? p1.x() : p2.x();
                x_max = p1.x() > p2.x() ? p1.x() : p2.x();
                y_min = p1.y() < p2.y() ? p1.y() : p2.y();
                y_max = p1.y() > p2.y() ? p1.y() : p2.y();

                if ((currentPosition.x() < x_max && currentPosition.x() > x_min) || (currentPosition.y() < y_max && currentPosition.y() > y_min)) {

                    d = boost::geometry::distance(currentPosition, i, boost::geometry::strategy::distance::projected_point<>{});

                    m = (p2.y() - p1.y()) / (p2.x() - p1.x());
                    m = -1 / (m);

                    l1.push_back(p1);
                    l1.push_back(p2);

                    l2.push_back(currentPosition);
                    x_2 = coord_max + currentPosition.x();
                    y_2 = m + currentPosition.y();
                    l2.push_back(point_type(x_2, y_2));

                    x_2 = -coord_max + currentPosition.x();
                    y_2 = m + currentPosition.y();
                    l2.push_back(point_type(x_2, y_2));

                    boost::geometry::intersection(l1, l2, output);
                    nearest_p.set<0>((output.front()).x());
                    nearest_p.set<1>((output.front()).y());
                }

                min_d = d;
                target_value.gain = 1.0;
                count = 1.0;
            }
        }
    } else {
        for (auto i : segments) {
            d = boost::geometry::distance(currentPosition, i);

            // Detect dangerous situation , remember to give back any time the nearest.
            if (d < MIN_THRESHOLD) {

                //starting point of the segment
                point_type p1{ boost::geometry::get<0, 0>(i), boost::geometry::get<0, 1>(i) };
                //ending point of the segment
                point_type p2{ boost::geometry::get<1, 0>(i), boost::geometry::get<1, 1>(i) };

                //distance between the current position and the starting point of the segment
                d_p1 = boost::geometry::distance(currentPosition, p1);
                //distance between the current position and the ending point of the segment
                d_p2 = boost::geometry::distance(currentPosition, p2);

                if (d_p1 <= d_p2) {
                    //if
                    nearest_p.set<0>(p1.x());
                    nearest_p.set<1>(p1.y());
                    d = d_p1;
                } else {
                    nearest_p.set<0>(p2.x());
                    nearest_p.set<1>(p2.y());
                    d = d_p2;
                }

                x_min = p1.x() < p2.x() ? p1.x() : p2.x();
                x_max = p1.x() > p2.x() ? p1.x() : p2.x();
                y_min = p1.y() < p2.y() ? p1.y() : p2.y();
                y_max = p1.y() > p2.y() ? p1.y() : p2.y();

                if ((currentPosition.x() < x_max && currentPosition.x() > x_min) || (currentPosition.y() < y_max && currentPosition.y() > y_min)) {

                    d = boost::geometry::distance(currentPosition, i, boost::geometry::strategy::distance::projected_point<>{});

                    m = (p2.y() - p1.y()) / (p2.x() - p1.x());
                    m = -1 / (m);

                    l1.push_back(p1);
                    l1.push_back(p2);

                    l2.push_back(currentPosition);
                    x_2 = coord_max + currentPosition.x();
                    y_2 = m + currentPosition.y();
                    l2.push_back(point_type(x_2, y_2));

                    x_2 = -coord_max + currentPosition.x();
                    y_2 = m + currentPosition.y();
                    l2.push_back(point_type(x_2, y_2));

                    boost::geometry::intersection(l1, l2, output);
                    nearest_p.set<0>((output.front()).x());
                    nearest_p.set<1>((output.front()).y());

                    if (first) {
                        first = false;
                        double gain = (d < MAX_THRESHOLD ? 1.0 : (MIN_THRESHOLD - d) / (MIN_THRESHOLD - MAX_THRESHOLD));
                        nearest_p.set<0>((gain * (output.front()).x()));
                        nearest_p.set<1>((gain * (output.front()).y()));
                        count += gain;

                    } else {
                        double gain = (d < MAX_THRESHOLD ? 1.0 : (MIN_THRESHOLD - d) / (MIN_THRESHOLD - MAX_THRESHOLD));
                        nearest_p.set<0>(nearest_p.x() + (gain * (output.front()).x()));
                        nearest_p.set<1>(nearest_p.y() + (gain * (output.front()).y()));
                        count += gain;
                    }
                }

                if (d < min_d) {
                    min_d = d;
                    d < MAX_THRESHOLD ? target_value.gain = 1.0 : target_value.gain = (MIN_THRESHOLD - d) / (MIN_THRESHOLD - MAX_THRESHOLD);
                }
            }
        }
    }

    if (min_d < static_cast<double>(INFINITY)) {
        // return these values: target(x,y) and "gain"
        theta = atan2(currentPosition.y() - (nearest_p.y() / count), currentPosition.x() - (nearest_p.x() / count));

        if (!boost::geometry::covered_by(currentPosition, poly)) {
            theta = theta + M_PI;
        } else {

            theta = theta + ulisse::MinimumAngleBetween((*pose_shared)(5), goalCxt_->goalHeading);
        }

        target_value.x = currentPosition.x() + min_d * cos(theta);
        target_value.y = currentPosition.y() + min_d * sin(theta);
    }

    std::cout << "DEBUG:: MIN_D:" << min_d << std::endl;

    return target_value;
}

void SafetyBoundaries::MakeSegments(point_type const& p, point_type const& next, segment_type& seg)
{
    boost::geometry::set<0, 0>(seg, p.x());
    boost::geometry::set<0, 1>(seg, p.y());
    boost::geometry::set<1, 0>(seg, next.x());
    boost::geometry::set<1, 1>(seg, next.y());
}
}
