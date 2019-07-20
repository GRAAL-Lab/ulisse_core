#ifndef __CONTROLMAKECURVE_H__
#define __CONTROLMAKECURVE_H__

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

class MakeCurve : public tpik::EqualityTask {
public:
    /**
     * @brief ControlLinearVelocity class constructor
     * @param taskID task id
     * @param robotModel shared ptr to the rml::RobotModel
     * @param frameID id of the frame to control
     */
    MakeCurve(std::string taskID, std::shared_ptr<rml::RobotModel> robotModel, std::string frameID);
    /**@brief ~ControlLinearVelocity default deconstructor
    */
    ~MakeCurve();

    /**
     * @brief SetAngularVelocityTask Method to set the angular velocity task
     * @param angularVelocityTask angular velocity task to pilot
     */
     void SetAngularVelocityTask(std::shared_ptr<ikcl::AngularVelocity> angularVelocityTask);

    /**
     * @brief SetLinearVelocityTask Method to set the linear velocity task
     * @param linearVelocityTask linear velocity task to pilot
     */
    void SetLinearVelocityTask(std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask);

    void SetStatusContext(const std::shared_ptr<ulisse::StatusContext>& statusCxt);

    void SetControlContext(const std::shared_ptr<ulisse::ControlContext>& ctrlCxt);

    void SetSaturationLimits(double max_surge, double max_jog);

    void SetCurve(double radius, std::string curve_type, bool clockwise);

    void Reset();

    /**
     * @brief Update Method to update the task status.
     * @details Implememntation of the pure virtual method of the base class tpik::Equality task, used to update the task parameters
     * i.e. the jacobian, activation function and reference. Exception are thrown if the task parameters have not been initialized.
     */
    void Update() throw(tpik::ExceptionWithHow) override;

protected:
    std::shared_ptr<rml::RobotModel> robotModel_; //!< The shared ptr to the robot model
    std::string frameID_; //!< The id of the frame to be controlled

    std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask_;
    std::shared_ptr<ikcl::AngularVelocity> angularVelocityTask_;

    std::shared_ptr<ulisse::ControlContext> ctrlCxt_;
    std::shared_ptr<ulisse::StatusContext> statusCxt_;
    double max_surge_, max_jog_;
    std::string curve_type_;
    bool clockwise_;

    double radius_curve_;
    double goal_heading_;
    double desired_jog_, desired_speed_;

    double tollerance_;

    bool isCurveInitialized_;

    Eigen::Vector6d desiredVelocity_;

    void UpdateReference();
    void UpdateJacobian();
};
}
#endif
