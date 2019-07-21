#include "ulisse_ctrl/tasks/SafetyBoundaries.h"

#include <ikcl/ikcl.h>
#include <rml/RML.h>
#include <tpik/TPIKlib.h>
#include "ctrl_toolbox/DataStructs.h"

#include "ulisse_ctrl/helper_functions.hpp"
#include "ulisse_ctrl/ctrl_data_structs.hpp"

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
    }

    SafetyBoundaries::~SafetyBoundaries() {}


    void SafetyBoundaries::SetTolleranceBellShape(double tollerance){
        tollerance_ = tollerance;
    }

    void SafetyBoundaries::SetPose(std::shared_ptr<Eigen::Vector6d> pose){
        pose_shared = pose;
    }

    void SafetyBoundaries::SetBoundaries(double bound_min, double bound_max){
        MAX_THRESHOLD = bound_max;
        MIN_THRESHOLD = bound_min;
    }

    void SafetyBoundaries::SetConf(const std::shared_ptr<ulisse::ControllerConfiguration>& conf){
        conf_ = conf;
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

        std::cout << " TARGET GAIN : " << target.gain << std::endl;

        if(target.gain > 0) {
            current_pose.latitude = (*pose_shared)(0);
            current_pose.longitude = (*pose_shared)(1);

            double lam = ulisse::lat_to_m_coeff(centroid.latitude);
            double lom = ulisse::lon_to_m_coeff(centroid.longitude);
            desired_pose = ulisse::point_euclidean2map(target.x, target.y, centroid, lam, lom);

            std::cout << " TARGET LAT : " << desired_pose.latitude << std::endl;
            std::cout << " TARGET LONG : " << desired_pose.longitude << std::endl;

            ctb::DistanceAndAzimuthRad(current_pose, desired_pose, goalDistance, goalHeading);

            double headingError = ctb::HeadingErrorRad(goalHeading, (*pose_shared)(5));
            goalDistance = SlowDownWhenTurning(headingError, goalDistance, *conf_);
            desired_speed = goalDistance;
            desired_jog = ulisse::MinimumAngleBetween( (*pose_shared)(5), goalHeading);

            desiredVelocity_(2) = desired_jog;
            desiredVelocity_(3) = desired_speed;

            UpdateInternalActivationFunction();
            UpdateJacobian();
            UpdateReference();
            SaturateReference();;
            SaturateReferenceComponentWise();
        }
        else{
            target.gain = 0;
            J_.setZero();
        }
    }

    void SafetyBoundaries::UpdateInternalActivationFunction()
    {
        Ai_.setIdentity();
        Ai_ = target.gain * Ai_;

        /*
            Ai_ = rml::IncreasingBellShapedFunction(1, target.gain, 0, 1, 0);
            Ai_ += rml::DecreasingBellShapedFunction(0, target.gain, 0, 1, 0);
        */
    }

    void SafetyBoundaries::UpdateJacobian() { J_ = robotModel_->GetCartesianJacobian(frameID_).block(0, 0, 6, DoF_); }

    void SafetyBoundaries::UpdateReference()
    {
        for (int i = 0; i < 6; i++) {
            x_dot_(i) = taskParameter_.gain * target.gain * desiredVelocity_(i);
        }
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

    bool SafetyBoundaries::InitializePoly(ctb::LatLong current_position, std::string polygon_to_string)
    {
        centroid = current_position;

        point_type p(0.0, 0.0);

        boost::geometry::read_wkt(polygon_to_string, poly);
        if (!boost::geometry::covered_by(p, poly))
            return false;

        //Generate a list with all the segments
        double x, y;
        for(auto it = boost::begin(boost::geometry::exterior_ring(poly)); (it+1) != boost::end(boost::geometry::exterior_ring(poly)); ++it)
        {
            x = boost::geometry::get<0>(*it);
            if (x > coord_max) {coord_max = x;}
            if (x < coord_min) {coord_min = x;}
            y = boost::geometry::get<1>(*it);
            if (y > coord_max) {coord_max = y;}
            if (y < coord_min) {coord_min = y;}

            point_type p(x, y);

            x = boost::geometry::get<0>(*(it + 1));
            y = boost::geometry::get<1>(*(it + 1));
            point_type next(x, y);

            make_segments(p, next);
        }
        segments.pop_back();

        coord_max = coord_max + abs(coord_min);
        isBoundariesInitialized = true;
        return true;
    }


    template <typename Point>
    desired_target SafetyBoundaries::distance_check(Point const& p)
    {
        desired_target target_value = {0.0 , 0.0 , 0.0};
        double d, d_p1, d_p2, tmp_d;
        double m, x_2, y_2, theta;
        std::deque<point_type> output;
        linestring_type l1, l2;

        std::cout << "P is at coordinates " << boost::geometry::get<0>(p) << " , " << boost::geometry::get<1>(p) << std::endl;

        for (auto i : segments)
        {
            std::cout << "Points coordinates: ("
                      << boost::geometry::get<0, 0>(i) << "," << boost::geometry::get<0, 1>(i) << ")\t("
                      << boost::geometry::get<1, 0>(i) << "," << boost::geometry::get<1, 1>(i) << ")" << std::endl;
            std::cout << "Point-Segments: " << boost::geometry::distance(p, i) << std::endl;

            d = boost::geometry::distance(p, i);
            //TODO: convert latlong to meters

            // Detect dangerous situation , remember to give back anyn time the nearest.
            if ( d < MIN_THRESHOLD )
            {

                std::cout << "The segment is closer than MIN_THRESHOLD (= " << MIN_THRESHOLD << std::endl;
                point_type p1{boost::geometry::get<0, 0>(i), boost::geometry::get<0, 1>(i)};
                point_type p2{boost::geometry::get<1, 0>(i), boost::geometry::get<1, 1>(i)};

                d_p1 = boost::geometry::distance(p, p1);
                d_p2 = boost::geometry::distance(p, p2);

                // d_p1 = ulisse::from_lat_long_to_measure(boost::geometry::get<0>(p), boost::geometry::get<1>(p), boost::geometry::get<0>(p1), boost::geometry::get<1>(p1));
                // d_p2 = ulisse::from_lat_long_to_measure(boost::geometry::get<0>(p), boost::geometry::get<1>(p), boost::geometry::get<0>(p2), boost::geometry::get<1>(p2));

                if (d_p1 <= d_p2)
                {
                    nearest_p.set<0>(boost::geometry::get<0, 0>(i));
                    nearest_p.set<1>(boost::geometry::get<0, 1>(i));
                    min_d = d_p1;
                }
                else
                {
                    nearest_p.set<0>(boost::geometry::get<1, 0>(i));
                    nearest_p.set<1>(boost::geometry::get<1, 1>(i));
                    min_d = d_p2;
                }

                double x_min = boost::geometry::get<0>(p1) < boost::geometry::get<0>(p2) ? boost::geometry::get<0>(p1) : boost::geometry::get<0>(p2);
                double x_max = boost::geometry::get<0>(p1) > boost::geometry::get<0>(p2) ? boost::geometry::get<0>(p1) : boost::geometry::get<0>(p2);
                double y_min = boost::geometry::get<1>(p1) < boost::geometry::get<1>(p2) ? boost::geometry::get<1>(p1) : boost::geometry::get<1>(p2);
                double y_max = boost::geometry::get<1>(p1) > boost::geometry::get<1>(p2) ? boost::geometry::get<1>(p1) : boost::geometry::get<1>(p2);

                if ( boost::geometry::get<0>(p) < x_max && boost::geometry::get<0>(p) > x_min ||
                     boost::geometry::get<0>(p) < y_max && boost::geometry::get<0>(p) > y_min )
                {
                    min_d = boost::geometry::distance(p, i, boost::geometry::strategy::distance::projected_point<>{});

                    m = (boost::geometry::get<1>(p2) - boost::geometry::get<1>(p1)) / (boost::geometry::get<0>(p2) - boost::geometry::get<0>(p1));
                    m = -1/(m);

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

                    nearest_p.set<0>(boost::geometry::get<0>(output.front()));
                    nearest_p.set<1>(boost::geometry::get<1>(output.front()));
                }

                d < MAX_THRESHOLD ? target_value.gain = 1.0 : target_value.gain = (d - MAX_THRESHOLD)/(MIN_THRESHOLD - MAX_THRESHOLD);
            }

        }

        // return these values: target(x,y) and "gain"
        theta = atan2(boost::geometry::get<1>(p) - boost::geometry::get<1>(nearest_p), boost::geometry::get<0>(p) - boost::geometry::get<0>(nearest_p));
        //if (theta < 0) theta = theta + M_2_PI;

        target_value.x = boost::geometry::get<0>(p) - 100*(MIN_THRESHOLD - min_d)*cos(theta);
        target_value.y = boost::geometry::get<1>(p) - 100*(MIN_THRESHOLD - min_d)*sin(theta);


        std::cout << "Closest Point : (" << boost::geometry::get<0>(nearest_p) << "," << boost::geometry::get<1>(nearest_p) << ") \t"
                  << "At a distance: " << min_d << std::endl;

        std::cout << "Point target: (" << target_value.x << "," << target_value.y << ")" <<
                  " with a value for the activation function: " << target_value.gain << std::endl;


        return target_value;
    }


}
