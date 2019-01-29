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
        //std::shared_ptr<StatusContext> posCxt_;
        std::shared_ptr<GoalContext> goalCxt_;

    public:
        GenericCommand();
        virtual ~GenericCommand();
        //void SetPosContext(const std::shared_ptr<StatusContext>& posCxt);
        void SetGoalContext(const std::shared_ptr<GoalContext>& goalCxt);
    };
}
}

#endif // ULISSE_CTRL_GENERICCOMMAND_HPP
