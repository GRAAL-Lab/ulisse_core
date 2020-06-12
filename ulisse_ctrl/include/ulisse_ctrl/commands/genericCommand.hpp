#ifndef ULISSE_CTRL_GENERICCOMMAND_HPP
#define ULISSE_CTRL_GENERICCOMMAND_HPP

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_ctrl/states/genericstate.hpp"
#include <fsm/fsm.h>

namespace ulisse {

namespace commands {

    class GenericCommand : public fsm::BaseCommand {

    public:
        GenericCommand();
        virtual ~GenericCommand();

        virtual void SetState(std::shared_ptr<states::GenericState> state) = 0;
    };
}
}
#endif // ULISSE_CTRL_GENERICCOMMAND_HPP
