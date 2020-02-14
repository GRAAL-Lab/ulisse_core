#ifndef __CONTROLSAFETYBOUNDARIES_H__
#define __CONTROLSAFETYBOUNDARIES_H__

#include "ctrl_toolbox/DataStructs.h"
#include "ctrl_toolbox/DigitalPID.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <list>
#include <rml/RML.h>
#include <tpik/TPIKlib.h>

#include "ctrl_toolbox/DataStructs.h"
#include <ikcl/ikcl.h>
#include <rml/RML.h>
#include <tpik/TPIKlib.h>

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include "ulisse_ctrl/helper_functions.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/strategies/cartesian/distance_projected_point.hpp>
#include <math.h>

#include <boost/foreach.hpp>
#include <boost/numeric/conversion/bounds.hpp>

using namespace ctb;

namespace ikcl {

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
typedef boost::geometry::model::segment<point_type> segment_type;
typedef boost::geometry::model::linestring<point_type> linestring_type;

typedef struct desired_target {
    double x;
    double y;
    double gain;
} desired_target;

/**
 * @brief The LinearVelocity class implementing the linear velocity task.
 * @details ikcl class aimed to implement the linear velocity control task for the constructor input frame.
 * The class uses the rml::RobotModel in order to compute the needed jacobians and parameters.
 * The class derives from tpik::EqualityTask and allows to control the linear velocity of the desired frame in a task priority
 * framework.\n
 * \f$ \dot{x}= \gamma \cdot DesiredVelocity \f$\n
 * \f$ J=J_{f, lin} \f$\n
 */

class SafetyBoundaries : public tpik::InequalityTask {
public:
    /**
     * @brief ControlLinearVelocity class constructor
     * @param taskID task id
     * @param robotModel shared ptr to the rml::RobotModel
     * @param frameID id of the frame to control
     */
    SafetyBoundaries(std::string taskID, std::shared_ptr<rml::RobotModel> robotModel, std::string frameID);
    /**@brief ~ControlLinearVelocity default deconstructor
    */
    ~SafetyBoundaries();

    void SetPose(std::shared_ptr<Eigen::Vector6d> pose);

    void SetTolleranceBellShape(double tollerance);
    /**
	 * @brief Method updating the task Jacobian, reference, internal activation function.
     * Implementation of the pure virtual method of the base class tpik::InequalityTask.
	 * @note An exception is throw if deltaJL has not been initialized yet.
	 */
    void Update() throw(tpik::ExceptionWithHow) override;

    bool InitializePoly(ctb::LatLong current_position, std::string polygon_to_string, std::string polygon_lat_long);

    void SetBoundaries(double bound_min, double bound_max);

    void SetControlContext(const std::shared_ptr<ulisse::ControlContext>& ctrlCxt);

    void SetGoalContext(const std::shared_ptr<ulisse::GoalContext>& goalCxt);

    void SetConf(const std::shared_ptr<ulisse::ControllerConfiguration>& conf);
    /**
	 * @brief Overloading of the cout operator
	 */
    friend std::ostream& operator<<(std::ostream& os, SafetyBoundaries const& safetyBoundaries)
    {
        os << "\033[1;37m"
           << "SAFETY BOUNDARIES " << (tpik::InequalityTask&)safetyBoundaries << std::setprecision(4);
        os << "\033[1;37m"
           << "\033[1;37m"
           << "frameID \n"
           << "\033[0m" << safetyBoundaries.frameID_ << "\n";
        return os;
    }

    void SetAlphaMinOnTurning(double alpha);
    void SetDesiredSpeedOnTurning(double des_speed);
    double GetAlphaMinOnTurning();
    double GetDesiredSpeedOnTurning();

protected:
    /**
	 * @brief Method updating the internal activation function.
     * Implementation of the pure virtual method of the base class tpik::InequalityTask.
	 */
    void UpdateInternalActivationFunction() override;
    /**
	 * @brief Method updating the Jacobian.
     * Implementation of the pure virtual method of the base class tpik::InequalityTask.
	 */
    void UpdateJacobian() override;
    /**
	 * @brief Method updating the reference.
     * Implementation of the pure virtual method of the base class tpik::InequalityTask.
	 */
    void UpdateReference() override;

    template <typename Point>
    void make_segments(Point const& p, Point const& next);

    template <typename Point>
    desired_target distance_check(Point const& p);

    std::shared_ptr<rml::RobotModel> robotModel_; //!< The shared ptr to the robot model
    std::string frameID_; //!< The id of the frame to be controlled

    Eigen::VectorXd aMin_; //!< Eigen::Vector containing the diagonal of the Activation Matrix for the min bound
    Eigen::VectorXd aMax_; //!< Eigen::Vector containing the diagonal of the Activation Matrix for the max bound

    bool isBoundariesInitialized{ false }; //!< Boolean used to state whehther deltaJL has been setted

    std::shared_ptr<Eigen::Vector6d> pose_shared;
    Eigen::VectorXd pose_;
    double tollerance_;

    std::list<segment_type> segments;

    polygon_type poly;
    polygon_type poly_lat_long;
    point_type nearest_p;
    double coord_max, coord_min;
    double min_d;

    ctb::LatLong centroid;
    desired_target target;
    ctb::LatLong current_pose;
    ctb::LatLong desired_pose;
    double goalDistance, goalHeading;
    double desired_speed, desired_jog;
    Eigen::Vector6d desiredVelocity_;
    double lam, lom;
    bool first;

    double MAX_THRESHOLD, MIN_THRESHOLD;
    double alpha_min_on_turn;
    double desired_speed_on_turn;

    std::shared_ptr<ulisse::ControllerConfiguration> conf_;
    std::shared_ptr<ulisse::ControlContext> ctrlCxt_;
    std::shared_ptr<ulisse::GoalContext> goalCxt_;
};
}
#endif
