#include "ulisse_ctrl/commands/genericCommand.hpp"

namespace ulisse {

namespace commands {

    GenericCommand::GenericCommand() {}

    GenericCommand::~GenericCommand() {}

    void GenericCommand::SetGoalCtx(const std::shared_ptr<GoalContext>& goalCtx)
    {
        goalCxt_ = goalCtx;
    }
}
}
