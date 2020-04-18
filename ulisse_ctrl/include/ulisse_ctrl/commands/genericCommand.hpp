#ifndef ULISSE_CTRL_GENERICCOMMAND_HPP
#define ULISSE_CTRL_GENERICCOMMAND_HPP

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include <fsm/fsm.h>

namespace ulisse {

namespace commands {

    class GenericCommand : public fsm::BaseCommand {

    protected:
        std::shared_ptr<GoalContext> goalCxt_;

    public:
        GenericCommand();
        virtual ~GenericCommand();

        void SetGoalCtx(const std::shared_ptr<GoalContext>& goalCtx);
    };
}
}
#endif // ULISSE_CTRL_GENERICCOMMAND_HPP
