#ifndef ULISSE_CTRL_GENERICCOMMAND_HPP
#define ULISSE_CTRL_GENERICCOMMAND_HPP

#include <rclcpp/logger.hpp>
#include <fsm/fsm.h>
#include "ulisse_ctrl/fsm_defines.hpp"


namespace ulisse {

namespace commands {

    class GenericCommand : public fsm::BaseCommand {
    protected:
        std::shared_ptr<PositionContext> posCxt_;
        std::shared_ptr<ControlContext> ctrlCxt_;

    public:
        GenericCommand();
        virtual ~GenericCommand();
        void SetPosContext(const std::shared_ptr<PositionContext>& posCxt);
        void SetCtrlContext(const std::shared_ptr<ControlContext>& ctrlCxt);
    };
}
}

#endif // ULISSE_CTRL_GENERICCOMMAND_HPP
