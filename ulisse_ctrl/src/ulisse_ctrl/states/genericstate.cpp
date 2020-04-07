#include "ulisse_ctrl/states/genericstate.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_driver/LLCHelperDefines.h"

namespace ulisse {

namespace states {

    GenericState::GenericState()
    {
    }

    GenericState::~GenericState()
    {
    }

    void GenericState::CheckRadioController()
    {

        if ((statusCxt_->llcStatus & EMB_STSMASK_PPM_ENABLED) != 0) {
            fsm_->EmitEvent(ulisse::events::names::rcenabled, ulisse::events::priority::high);
        }
    }

    void GenericState::SetStatusContext(const std::shared_ptr<StatusContext>& statusCxt)
    {
        statusCxt_ = statusCxt;
    }

    void GenericState::SetGoalContext(const std::shared_ptr<GoalContext>& goalCxt)
    {
        goalCxt_ = goalCxt;
    }

    void GenericState::SetCtrlContext(const std::shared_ptr<ControlContext>& ctrlCxt)
    {
        ctrlCxt_ = ctrlCxt;
    }

    void GenericState::SetConf(const std::shared_ptr<ControllerConfiguration>& conf)
    {
        conf_ = conf;
    }

    void GenericState::SetActionManager(std::shared_ptr<tpik::ActionManager> actionManager)
    {
        actionManager_ = actionManager;
    }
    void GenericState::SetUnifiedHierarchy(std::vector<std::shared_ptr<tpik::Task>> unifiedHierarchy)
    {
        unifiedHierarchy_ = unifiedHierarchy;
    }
    void GenericState::SetRobotModel(std::shared_ptr<rml::RobotModel> robotModel)
    {
        robotModel_ = robotModel;
    }

    void GenericState::SetSafetyBoundariesTask(std::shared_ptr<ikcl::SafetyBoundaries> safetyBoundariesTask)
    {
        safetyBoundariesTask_ = safetyBoundariesTask;
    }

    void GenericState::SetAngularPositionSafetyTask(std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentSafetyTask)
    {
        absoluteAxisAlignmentSafetyTask_ = absoluteAxisAlignmentSafetyTask;
    }
    void GenericState::SetMinMaxHeadingErrorSafety(double min, double max)
    {
        minHeadingErrorSafety_ = min;
        maxHeadingErrorSafety_ = max;
    }

    void GenericState::SetMaxGainSafety(double maxGainSafety)
    {
        maxGainSafety_ = maxGainSafety;
    }
}
}
