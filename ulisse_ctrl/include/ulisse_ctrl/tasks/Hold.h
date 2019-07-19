#ifndef __CONTROLHOLD_H__
#define __CONTROLHOLD_H__

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
#include "ulisse_ctrl/geometry_defines.h"
#include "ulisse_ctrl/tasks/ActionTask.h"
#include "ulisse_ctrl/tasks/AngularPosition.h"
#include "ulisse_ctrl/tasks/ControlDistance.h"

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

class Hold : public tpik::EqualityTask {
public:
    /**
     * @brief ControlLinearVelocity class constructor
     * @param taskID task id
     * @param robotModel shared ptr to the rml::RobotModel
     * @param frameID id of the frame to control
     */
    Hold(std::string taskID, std::shared_ptr<rml::RobotModel> robotModel, std::string frameID);
    /**@brief ~ControlLinearVelocity default deconstructor
    */
    ~Hold();

    void SetGoalContext(const std::shared_ptr<ulisse::GoalContext>& goalCxt);

    void SetStatusContext(const std::shared_ptr<ulisse::StatusContext>& statusCxt);

    void SetConf(const std::shared_ptr<ulisse::ControllerConfiguration>& conf);

    void SetGoalHold(ctb::LatLong goal);

    void ResetGoal();

    bool IsGoalSet();

    /**
     * @brief Update Method to update the task status.
     * @details Implememntation of the pure virtual method of the base class tpik::Equality task, used to update the task parameters
     * i.e. the jacobian, activation function and reference. Exception are thrown if the task parameters have not been initialized.
     */
    void Update() throw(tpik::ExceptionWithHow) override;

protected:
    std::shared_ptr<rml::RobotModel> robotModel_; //!< The shared ptr to the robot model
    std::string frameID_; //!< The id of the frame to be controlled

    std::shared_ptr<ulisse::GoalContext> goalCxt_;
    std::shared_ptr<ulisse::StatusContext> statusCxt_;
    std::shared_ptr<ulisse::ControllerConfiguration> conf_;

    ctb::LatLong goal_;
    bool is_goal_set;

    bool goalReached_;
    double desired_speed, desired_jog;
    double surgeRef;
    double currentDirection;
    double desiredHeading;
    double currentNorm;
    double hrefA;
    double goalDistance;
    double headingError;

    Eigen::Vector6d desiredVelocity_;

    void UpdateReference();
    void UpdateJacobian();

};
}
#endif
