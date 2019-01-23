#ifndef ULISSE_CTRL_GENERICSTATE_HPP
#define ULISSE_CTRL_GENERICSTATE_HPP

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include <fsm/fsm.h>
#include <rclcpp/logger.hpp>

namespace ulisse {

namespace states {

    class GenericState : public fsm::BaseState {
    protected:
        std::shared_ptr<ConfigurationData> conf_;
        std::shared_ptr<StatusContext> statusCxt_;
        std::shared_ptr<GoalContext> goalCxt_;
        std::shared_ptr<ControlContext> ctrlCxt_;

    public:
        GenericState(void);
        virtual ~GenericState(void);

        void CheckRadioController();
        void SetConf(const std::shared_ptr<ConfigurationData>& conf);
        void SetStatusContext(const std::shared_ptr<StatusContext>& posCxt);
        void SetGoalContext(const std::shared_ptr<GoalContext>& goalCxt);
        void SetCtrlContext(const std::shared_ptr<ControlContext>& ctrlCxt);
    };
}
}

#endif // ULISSE_CTRL_GENERICSTATE_HPP
