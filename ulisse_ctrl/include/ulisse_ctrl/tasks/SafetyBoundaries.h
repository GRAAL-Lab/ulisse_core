#ifndef __CONTROLSAFETYBOUNDARIES_H__
#define __CONTROLSAFETYBOUNDARIES_H__

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <tpik/TPIKlib.h>

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

    void SetPose(std::shared_ptr<Eigen::Vector6d> pose);

    void SetTolleranceBellShape(double tollerance);
    /**
	 * @brief Method updating the task Jacobian, reference, internal activation function.
     * Implementation of the pure virtual method of the base class tpik::InequalityTask.
	 * @note An exception is throw if deltaJL has not been initialized yet.
	 */
    void Update() throw(tpik::ExceptionWithHow) override;

    bool InitializePolygon(ctb::LatLong startingPosition, std::string polygonString);

    void SetBoundaries(double bound_min, double bound_max);

    void SetGoalContext(const std::shared_ptr<ulisse::GoalContext>& goalCxt);

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

    void SetDesiredSpeedOnTurning(double des_speed);

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

    void MakeSegments(point_type const& p, point_type const& next, segment_type& seg);

    desired_target DistanceCheck(point_type const& currentPosition);

    std::shared_ptr<rml::RobotModel> robotModel_; //!< The shared ptr to the robot model
    std::string frameID_; //!< The id of the frame to be controlled
    bool isBoundariesInitialized{ false }; //!< Boolean used to state whehther deltaJL has been setted
    std::shared_ptr<Eigen::Vector6d> pose_shared;
    double tollerance_;
    std::list<segment_type> segments;
    double coord_max, coord_min;
    ctb::LatLong centroid;
    desired_target target;
    Eigen::Vector6d desiredVelocity_;
    polygon_type poly;

    double MAX_THRESHOLD, MIN_THRESHOLD;

    double desired_speed_on_turn;

    std::shared_ptr<ulisse::GoalContext> goalCxt_;
};
}
#endif
