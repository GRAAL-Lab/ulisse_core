#include "ulisse_ctrl/states/state_halt.hpp"

namespace ulisse {

namespace states {

    StateHalt::StateHalt()
    {
    }

    StateHalt::~StateHalt()
    {
    }

    fsm::retval StateHalt::OnEntry()
    {
        actionManager_->SetAction(ulisse::action::idle, true);

        return fsm::ok;
    }

    fsm::retval StateHalt::Execute()
    {
        for (auto& task : unifiedHierarchy_) {
            try {
                task->Update();
            } catch (tpik::ExceptionWithHow& e) {
                std::cerr << "UPDATE TASK EXCEPTION" << std::endl;
                std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
            }
        }

        std::cout << "STATE HALT" << std::endl;
        return fsm::ok;
    }
}
}
