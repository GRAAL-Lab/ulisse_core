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

        if ((stateCtx_.statusCxt->llcStatus & EMB_STSMASK_PPM_ENABLED) != 0) {
            fsm_->EmitEvent(ulisse::events::names::rcenabled, ulisse::events::priority::high);
        }
    }

    void GenericState::SetStateCtx(StateCtx stateCtx)
    {
        stateCtx_.statusCxt = stateCtx.statusCxt;
        stateCtx_.goalCxt = stateCtx.goalCxt;
        stateCtx_.ctrlCxt = stateCtx.ctrlCxt;
        stateCtx_.actionManager = stateCtx.actionManager;
        stateCtx_.robotModel = stateCtx.robotModel;
        stateCtx_.tasksMap = stateCtx.tasksMap;
    }

}
}
