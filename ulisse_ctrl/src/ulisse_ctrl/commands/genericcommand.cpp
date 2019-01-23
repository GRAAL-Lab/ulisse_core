#include "ulisse_ctrl/commands/genericcommand.hpp"

namespace ulisse {

namespace commands {

    GenericCommand::GenericCommand()
    {
    }

    GenericCommand::~GenericCommand()
    {
    }

//    void GenericCommand::SetPosContext(const std::shared_ptr<StatusContext>& posCxt)
//    {
//        posCxt_ = posCxt;
//    }

    void GenericCommand::SetGoalContext(const std::shared_ptr<GoalContext>& ctrlCxt)
    {
        goalCxt_ = ctrlCxt;
    }

}
}
