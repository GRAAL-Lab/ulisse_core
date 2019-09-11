#ifndef __CONTROLDISTANCE_H__
#define __CONTROLDISTANCE_H__

#include <eigen3/Eigen/Dense>
#include <rml/RML.h>
#include <tpik/TPIKlib.h>
#include "ctrl_toolbox/DataStructs.h"
#include "ctrl_toolbox/DigitalPID.h"

#include <ikcl/ikcl.h>
#include <rml/RML.h>
#include <tpik/TPIKlib.h>

#include "ulisse_ctrl/helper_functions.hpp"
#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include "ulisse_ctrl/tasks/ActionTask.h"

namespace ikcl {
/**
 * @brief The LinearVelocity class implementing the linear velocity task.
 * @details ikcl class aimed to implement the linear velocity control task for the constructor input frame.
 * The class uses the rml::RobotModel in order to compute the needed jacobians and parameters.
 * The class derives from tpik::EqualityTask and allows to control the linear velocity of the desired frame in a task priority
 * framework.\n
 * \f$ \dot{x}= \gamma \cdot DesiredVelocity \f$\n
 * \f$ J=J_{f, lin} \f$\n
 */

class ControlDistance : public tpik::CartesianTask {
public:
    /**
     * @brief ControlLinearVelocity class constructor
     * @param taskID task id
     * @param robotModel shared ptr to the rml::RobotModel
     * @param frameID id of the frame to control
     */
    ControlDistance(std::string taskID, std::shared_ptr<rml::RobotModel> robotModel, std::string frameID, tpik::CartesianTaskType taskType );
    /**@brief ~ControlLinearVelocity default deconstructor
    */
    ~ControlDistance();
    /**
     * @brief SetAngle Method to set the desired angle
     * @param desiredAngle desired angular velocity of the frame
     */
    void SetDistance(Eigen::Vector3d distance);

    void SetStatusContext(const std::shared_ptr<ulisse::StatusContext>& posCxt);

    void SetGoalContext(const std::shared_ptr<ulisse::GoalContext>& goalCxt);

    void Reset();

    /**
     * @brief Update Method to update the task status.
     * @details Implememntation of the pure virtual method of the base class tpik::Equality task, used to update the task parameters
     * i.e. the jacobian, activation function and reference. Exception are thrown if the task parameters have not been initialized.
     */
    void Update() throw(tpik::ExceptionWithHow) override;

    friend std::ostream& operator<<(std::ostream& os, ControlDistance const& controlDistance)
    {
        os << "\033[1;37m"
           << "CONTROL DISTANCE " << (tpik::CartesianTask&)controlDistance;
        os << std::setprecision(4) << "\n";
        return os;
    }

protected:

    std::shared_ptr<rml::RobotModel> robotModel_; //!< The shared ptr to the robot model
    std::string frameID_; //!< The id of the frame to be controlled
    Eigen::Vector3d distance_; //!< The desired velocity
    bool isDistanceInitialized_{ false }; //!< Boolean stating whether the velocity have been initialized

    std::shared_ptr<ulisse::StatusContext> statusCxt_;
    std::shared_ptr<ulisse::GoalContext> goalCxt_;

    std::shared_ptr<ulisse::ControllerConfiguration> conf_;

    double desired_speed;
    double headingError;
    double goalDistance;

    /**
     * @brief Method updating the task Jacobian.
     * @details Implementation of the pure virtual method of the base class tpik::CartesianTask  that updates the task jacobians.
     */
    void UpdateJacobian() override;

    Eigen::Vector3d distanceBodyFrame_;

    Eigen::Matrix3d P_;
    Eigen::Vector3d normalOnBodyFrame;
};
}
#endif
