#include "ulisse_ctrl/states/generic_state.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"

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
        if (ctrlData->radioControllerEnabled) {
            fsm_->EmitEvent(ulisse::events::names::rcenabled, ulisse::events::priority::high);
        }
    }
}
}
