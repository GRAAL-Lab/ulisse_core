#include "ulisse_ctrl/tasks/SafetyBoundaries.h"

#include "ctrl_toolbox/DataStructs.h"
#include <ikcl/ikcl.h>
#include <rml/RML.h>
#include <tpik/TPIKlib.h>

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include "ulisse_ctrl/helper_functions.hpp"

#include <algorithm>

using namespace ctb;
namespace ikcl {

SafetyBoundaries::SafetyBoundaries(std::string taskID, std::shared_ptr<rml::RobotModel> robotModel, std::string frameID)
    : tpik::InequalityTask(taskID, 6, robotModel->GetTotalDOFs(), tpik::InequalityTaskType::None)
    , robotModel_(robotModel)
    , frameID_(frameID)

{
    MAX_THRESHOLD = 5.0;
    MIN_THRESHOLD = 3.0;

    alpha_min_on_turn = 0.5;
    desired_speed_on_turn = 2.5;
}

SafetyBoundaries::~SafetyBoundaries() {}

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

void SafetyBoundaries::SetAlphaMinOnTurning(double alpha){
    alpha_min_on_turn = alpha;
}

void SafetyBoundaries::SetDesiredSpeedOnTurning(double des_speed){
    desired_speed_on_turn = des_speed;
}

double SafetyBoundaries::GetAlphaMinOnTurning(){
    return alpha_min_on_turn;
}

double SafetyBoundaries::GetDesiredSpeedOnTurning(){
    return desired_speed_on_turn;
}

void SafetyBoundaries::SetConf(const std::shared_ptr<ulisse::ControllerConfiguration>& conf)
{
    conf_ = conf;
}

void SafetyBoundaries::SetControlContext(const std::shared_ptr<ulisse::ControlContext>& ctrlCxt)
{
    ctrlCxt_ = ctrlCxt;
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

    double lam = ulisse::lat_to_m_coeff(centroid.latitude);
    double lom = ulisse::lon_to_m_coeff(centroid.longitude);
    double* p = ulisse::point_map2euclidean((*pose_shared)(0), (*pose_shared)(1), centroid, lam, lom);
    target = distance_check(point_type(p[0], p[1]));

    if (target.gain > 0) {
        current_pose.latitude = (*pose_shared)(0);
        current_pose.longitude = (*pose_shared)(1);

        desired_pose = ulisse::point_euclidean2map(target.x, target.y, centroid, lam, lom);

        ctb::DistanceAndAzimuthRad(current_pose, desired_pose, goalDistance, goalHeading);

        double alph = conf_->slowOnTurns.alphaMin;
        conf_->slowOnTurns.alphaMin = alpha_min_on_turn;
        goalDistance = SlowDownWhenTurning(ctrlCxt_->desiredJog, desired_speed_on_turn, *conf_);
        conf_->slowOnTurns.alphaMin = alph;
        desired_speed = goalDistance;
        desired_jog = ulisse::MinimumAngleBetween((*pose_shared)(5), goalHeading);

        desiredVelocity_(2) = desired_jog;
        desiredVelocity_(3) = desired_speed;

    } else {
        target.gain = 0;
    }

    UpdateInternalActivationFunction();
    UpdateJacobian();
    UpdateReference();
    SaturateReference();
    ;
    SaturateReferenceComponentWise();
}

void SafetyBoundaries::UpdateInternalActivationFunction()
{
    Ai_.setIdentity();
    Ai_ = target.gain * Ai_;

    std::cout << Ai_ << std::endl;

}

void SafetyBoundaries::UpdateJacobian() { J_ = robotModel_->GetCartesianJacobian(frameID_).block(0, 0, 6, DoF_); }

void SafetyBoundaries::UpdateReference()
{
    x_dot_ = taskParameter_.gain * target.gain * (desiredVelocity_);
}

template <typename Point>
void SafetyBoundaries::make_segments(Point const& p, Point const& next)
{
    segment_type seg;

    boost::geometry::set<0, 0>(seg, boost::geometry::get<0>(p));
    boost::geometry::set<0, 1>(seg, boost::geometry::get<1>(p));
    boost::geometry::set<1, 0>(seg, boost::geometry::get<0>(next));
    boost::geometry::set<1, 1>(seg, boost::geometry::get<1>(next));

    segments.push_front(seg);
}

bool SafetyBoundaries::InitializePoly(ctb::LatLong current_position, std::string polygon_to_string, std::string polygon_lat_long)
{
    centroid = current_position;

    lam = ulisse::lat_to_m_coeff(centroid.latitude);
    lom = ulisse::lon_to_m_coeff(centroid.longitude);

    point_type p(centroid.latitude, centroid.longitude);

    boost::geometry::read_wkt(polygon_to_string, poly);
    boost::geometry::read_wkt(polygon_lat_long, poly_lat_long);
    if (!boost::geometry::covered_by(p, poly_lat_long))
        return false;

    //Generate a list with all the segments
    double x, y;
    for (auto it = boost::begin(boost::geometry::exterior_ring(poly)); (it + 1) != boost::end(boost::geometry::exterior_ring(poly)); ++it) {
        x = boost::geometry::get<0>(*it);
        if (x > coord_max) {
            coord_max = x;
        }
        if (x < coord_min) {
            coord_min = x;
        }
        y = boost::geometry::get<1>(*it);
        if (y > coord_max) {
            coord_max = y;
        }
        if (y < coord_min) {
            coord_min = y;
        }

        point_type p(x, y);

        x = boost::geometry::get<0>(*(it + 1));
        y = boost::geometry::get<1>(*(it + 1));
        point_type next(x, y);

        make_segments(p, next);
    }

    coord_max = coord_max + abs(coord_min);
    isBoundariesInitialized = true;
    return true;
}

template <typename Point>
desired_target SafetyBoundaries::distance_check(Point const& p)
{
    desired_target target_value = { 0.0, 0.0, 0.0 };
    double d, d_p1, d_p2, tmp_d;
    double m, x_2, y_2, theta;
    std::deque<point_type> output;
    linestring_type l1, l2;

    double x_max, x_min, y_max, y_min;

    nearest_p.set<0>(0.0);
    nearest_p.set<1>(0.0);

    min_d = INFINITY;
    first = true;

    double count = 0.0;

    for (auto i : segments) {
        d = boost::geometry::distance(p, i);

        // Detect dangerous situation , remember to give back anyn time the nearest.
        if (d < MIN_THRESHOLD) {

            point_type p1{ boost::geometry::get<0, 0>(i), boost::geometry::get<0, 1>(i) };
            point_type p2{ boost::geometry::get<1, 0>(i), boost::geometry::get<1, 1>(i) };

            d_p1 = boost::geometry::distance(p, p1);
            d_p2 = boost::geometry::distance(p, p2);

            if (d_p1 <= d_p2) {
                nearest_p.set<0>(boost::geometry::get<0, 0>(i));
                nearest_p.set<1>(boost::geometry::get<0, 1>(i));
                d = d_p1;
            } else {
                nearest_p.set<0>(boost::geometry::get<1, 0>(i));
                nearest_p.set<1>(boost::geometry::get<1, 1>(i));
                d = d_p2;
            }

            x_min = boost::geometry::get<0>(p1) < boost::geometry::get<0>(p2) ? boost::geometry::get<0>(p1) : boost::geometry::get<0>(p2);
            x_max = boost::geometry::get<0>(p1) > boost::geometry::get<0>(p2) ? boost::geometry::get<0>(p1) : boost::geometry::get<0>(p2);
            y_min = boost::geometry::get<1>(p1) < boost::geometry::get<1>(p2) ? boost::geometry::get<1>(p1) : boost::geometry::get<1>(p2);
            y_max = boost::geometry::get<1>(p1) > boost::geometry::get<1>(p2) ? boost::geometry::get<1>(p1) : boost::geometry::get<1>(p2);

            if (boost::geometry::get<0>(p) < x_max && boost::geometry::get<0>(p) > x_min || boost::geometry::get<0>(p) < y_max && boost::geometry::get<0>(p) > y_min) {
                d = boost::geometry::distance(p, i, boost::geometry::strategy::distance::projected_point<>{});

                m = (boost::geometry::get<1>(p2) - boost::geometry::get<1>(p1)) / (boost::geometry::get<0>(p2) - boost::geometry::get<0>(p1));
                m = -1 / (m);

                l1.push_back(point_type(boost::geometry::get<0>(p1), boost::geometry::get<1>(p1)));
                l1.push_back(point_type(boost::geometry::get<0>(p2), boost::geometry::get<1>(p2)));

                l2.push_back(point_type(boost::geometry::get<0>(p), boost::geometry::get<1>(p)));
                x_2 = coord_max + boost::geometry::get<0>(p);
                y_2 = m + boost::geometry::get<1>(p);
                l2.push_back(point_type(x_2, y_2));

                x_2 = -coord_max + boost::geometry::get<0>(p);
                y_2 = m + boost::geometry::get<1>(p);
                l2.push_back(point_type(x_2, y_2));

                boost::geometry::intersection(l1, l2, output);

                if (first) {
                    first = false;
                    double gain = (d < MAX_THRESHOLD ? 1.0 : (MIN_THRESHOLD - d) / (MIN_THRESHOLD - MAX_THRESHOLD));
                    nearest_p.set<0>((gain * boost::geometry::get<0>(output.front())));
                    nearest_p.set<1>((gain * boost::geometry::get<1>(output.front())));
                    count += gain;

                } else {
                    double gain = (d < MAX_THRESHOLD ? 1.0 : (MIN_THRESHOLD - d) / (MIN_THRESHOLD - MAX_THRESHOLD));
                    nearest_p.set<0>(boost::geometry::get<0>(nearest_p) + (gain * boost::geometry::get<0>(output.front())));
                    nearest_p.set<1>(boost::geometry::get<1>(nearest_p) + (gain * boost::geometry::get<1>(output.front())));
                    count += gain;
                }
            }

            if (d < min_d) {
                min_d = d;
                d < MAX_THRESHOLD ? target_value.gain = 1.0 : target_value.gain = (MIN_THRESHOLD - d) / (MIN_THRESHOLD - MAX_THRESHOLD);
            }
        }
    }

    // return these values: target(x,y) and "gain"
    theta = atan2(boost::geometry::get<1>(p) - (boost::geometry::get<1>(nearest_p) / count), boost::geometry::get<0>(p) - (boost::geometry::get<0>(nearest_p) / count));

    target_value.x = boost::geometry::get<0>(p) + min_d * cos(theta);
    target_value.y = boost::geometry::get<1>(p) + min_d * sin(theta);

    return target_value;
}

}
