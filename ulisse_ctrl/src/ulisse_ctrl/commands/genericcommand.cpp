#include "ulisse_ctrl/commands/genericcommand.hpp"

namespace ulisse {

namespace commands {

    GenericCommand::GenericCommand()
    {
    }

    GenericCommand::~GenericCommand()
    {
    }

    void GenericCommand::SetPosContext(const std::shared_ptr<PositionContext>& posCxt)
    {
        posCxt_ = posCxt;
    }

    void GenericCommand::SetCtrlContext(const std::shared_ptr<ControlContext>& ctrlCxt)
    {
        ctrlCxt_ = ctrlCxt;
    }

}
}
