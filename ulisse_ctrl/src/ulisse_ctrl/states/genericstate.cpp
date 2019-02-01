#include "ulisse_ctrl/states/genericstate.hpp"
#include "ulisse_driver/EESHelperDefines.h"

namespace  ulisse {

namespace states {

GenericState::GenericState()
{

}

GenericState::~GenericState()
{

}

void GenericState::CheckRadioController() {

    if ((statusCxt_->eesStatus & EMB_STSMASK_PPM_ENABLED) != 0) {
        fsm_->EmitEvent(ulisse::events::names::rcenabled, ulisse::events::priority::high);
    }
}

void GenericState::SetStatusContext(const std::shared_ptr<StatusContext>& statusCxt)
{
    statusCxt_ = statusCxt;
}

void GenericState::SetGoalContext(const std::shared_ptr<GoalContext>& goalCxt){
    goalCxt_ = goalCxt;
}

void GenericState::SetCtrlContext(const std::shared_ptr<ControlContext>& ctrlCxt)
{
    ctrlCxt_ = ctrlCxt;
}

void GenericState::SetConf(const std::shared_ptr<ConfigurationData>& conf)
{
    conf_ = conf;
}

}

}
