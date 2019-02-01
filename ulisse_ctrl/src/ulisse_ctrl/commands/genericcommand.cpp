#include "ulisse_ctrl/commands/genericcommand.hpp"

namespace ulisse {

namespace commands {

    GenericCommand::GenericCommand()
    {
    }

    GenericCommand::~GenericCommand()
    {
    }

    void GenericCommand::SetStatusContext(const std::shared_ptr<StatusContext>& statusCxt)
    {
        statusCxt_ = statusCxt;
    }

    void GenericCommand::SetGoalContext(const std::shared_ptr<GoalContext>& ctrlCxt)
    {
        goalCxt_ = ctrlCxt;
    }

    void GenericCommand::SetControlContext(const std::shared_ptr<ControlContext>& ctrlCxt)
    {
        ctrlCxt_ = ctrlCxt;
    }
}
}
