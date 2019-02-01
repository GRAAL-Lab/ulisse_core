#ifndef ULISSE_CTRL_GENERICCOMMAND_HPP
#define ULISSE_CTRL_GENERICCOMMAND_HPP

#include <rclcpp/logger.hpp>
#include <fsm/fsm.h>
#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_ctrl/ctrl_data_structs.hpp"


namespace ulisse {

namespace commands {

    class GenericCommand : public fsm::BaseCommand {
    protected:
        std::shared_ptr<StatusContext> statusCxt_;
        std::shared_ptr<GoalContext> goalCxt_;
        std::shared_ptr<ControlContext> ctrlCxt_;

    public:
        GenericCommand();
        virtual ~GenericCommand();
        void SetStatusContext(const std::shared_ptr<StatusContext>& statusCxt);
        void SetGoalContext(const std::shared_ptr<GoalContext>& goalCxt);
        void SetControlContext(const std::shared_ptr<ControlContext>& ctrlCxt);
    };
}
}

#endif // ULISSE_CTRL_GENERICCOMMAND_HPP
