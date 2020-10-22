#ifndef __CONTROLSAFETYBOUNDARIES_H__
#define __CONTROLSAFETYBOUNDARIES_H__

#include "tpik/TPIKDefines.h"
#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include "ulisse_msgs/msg/boundaries.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <tpik/TPIKlib.h>

using namespace ctb;

namespace ikcl {

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
typedef boost::geometry::model::segment<point_type> segment_type;

/**
 * @brief The LinearVelocity class implementing the linear velocity task.
 * @details ikcl class aimed to implement the linear velocity control task for
 * the constructor input frame. The class uses the rml::RobotModel in order to
 * compute the needed jacobians and parameters. The class derives from
 * tpik::EqualityTask and allows to control the linear velocity of the desired
 * frame in a task priority framework.\f$ \dot{x}= \gamma \cdot
 * DesiredVelocity  J=J_{f, lin} \f$
 */

class SafetyBoundaries : public tpik::ReactiveTask {
public:
    /**
   * @brief ControlLinearVelocity class constructor
   * @param taskID task id
   * @param robotModel shared ptr to the rml::RobotModel
   * @param frameID id of the frame to control
   */
    SafetyBoundaries(std::string taskID, std::shared_ptr<rml::RobotModel> robotModel, std::string frameID);

    /**
   * @brief Method updating the task Jacobian, reference, internal activation
   * function. Implementation of the pure virtual method of the base class
   * tpik::InequalityTask.
   * @note An exception is throw if deltaJL has not been initialized yet.
   */
    void Update() noexcept(false) override;

    bool InitializePolygon(const ulisse_msgs::msg::Boundaries& boundaries);

    //  void SetBoundaries(double bound_min, double bound_max);

    /**
   * @brief Overloading of the cout operator
   */
    friend std::ostream& operator<<(std::ostream& os, SafetyBoundaries const& safetyBoundaries)
    {
        os << "\033[1;37m"
           << "SAFETY BOUNDARIES " << (tpik::ReactiveTask&)safetyBoundaries
           << std::setprecision(4);
        os << "\033[1;37m"
           << "\033[1;37m"
           << "frameID \n"
           << "\033[0m" << safetyBoundaries.frameID_ << "\n";
        return os;
    }

    auto VehiclePosition() -> LatLong& { return vehiclePositionLatLong_; }

    auto AlignVector() const -> const Eigen::Vector3d& { return alignVector_; }

    bool ConfigFromFile(libconfig::Config& confObj) override;

protected:
    /**
   * @brief Method updating the internal activation function.
   * Implementation of the pure virtual method of the base class
   * tpik::InequalityTask.
   */
    void UpdateInternalActivationFunction() override;
    /**
   * @brief Method updating the Jacobian.
   * Implementation of the pure virtual method of the base class
   * tpik::InequalityTask.
   */
    void UpdateJacobian() override;

    void MakeSegments(point_type const& p, point_type const& next, segment_type& seg);

    void ExtractMinDistanceSegments(std::list<segment_type> segments, point_type currentPosition, std::list<segment_type>& segment);

    void DistanceCheck(point_type const& currentPositionr, Eigen::Vector3d& UTM_alignVecotr);

    bool IsConvex(std::list<segment_type> segments);

    void ComputeAlignVectorConcave(segment_type segment, point_type currentPosition, Eigen::Vector3d& UTM_alignVecotr);

    void ComputeAlignVectorConvex(std::list<segment_type> segment, point_type currentPosition, Eigen::Vector3d& UTM_alignVecotr);

    void ComputeNormalVector2Segment(segment_type segment, point_type& alignVector);

    void ComputeNormalVector2Segment(segment_type segment, point_type& alignVector, point_type& u);

    bool ComputeIntersectionPointMiddleZone(std::list<segment_type> segments, std::list<point_type>& points);

    std::shared_ptr<rml::RobotModel> robotModel_; //!< The shared ptr to the robot model
    std::string frameID_; //!< The id of the frame to be controlled
    bool isBoundariesInitialized_; //!< Boolean used to state whehther deltaJL has been setted
    std::list<segment_type> segments_;
    ctb::LatLong centroid_;
    polygon_type poly_;
    Eigen::Vector3d alignVector_;
    Eigen::Vector3d vehiclePosition_;
    LatLong vehiclePositionLatLong_;
    double d_;
};
} // namespace ikcl
#endif
