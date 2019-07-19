#include "ulisse_ctrl/tasks/ActionTask.h"
#include "rml/RML.h"
#include <tpik/TPIKlib.h>

namespace tpik {

    ActionTask::ActionTask(const std::string ID, int taskSpace, int DoF)
            : Task(ID, taskSpace, DoF)
            , initializedTaskParameter_{ false }
    {
        UpdateInternalActivationFunction();
    }

    ActionTask::~ActionTask() {}

    void ActionTask::SetTaskParameter(TaskParameter taskParameter)
    {
        taskParameter_ = taskParameter;
        isActive_ = taskParameter_.taskEnable;
        initializedTaskParameter_ = true;
    }

    TaskParameter ActionTask::GetTaskParameter() { return taskParameter_; }

    void ActionTask::SaturateReference() { ; }

    void ActionTask::SaturateReferenceComponentWise()
    {
        ;
    }

    void ActionTask::CheckInitialization() throw(ExceptionWithHow)
    {
        if (!initializedTaskParameter_) {
            NotInitialziedTaskParameterException notInitializedTaskParameter;
            std::string how = "[ActionTask] Not initialized taskParameter struct, use SetTaskParameter() for task " + ID_;
            notInitializedTaskParameter.SetHow(how);
            throw(notInitializedTaskParameter);
        }
    }

    void ActionTask::UpdateInternalActivationFunction() { Ai_.setIdentity(); }

    void ActionTask::UpdateReference() { x_dot_.setZero(taskSpace_); }

    void ActionTask::UpdateJacobian() { J_.setZero(taskSpace_, DoF_);  }
}
