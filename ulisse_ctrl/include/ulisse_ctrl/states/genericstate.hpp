#ifndef ULISSE_CTRL_GENERICSTATE_HPP
#define ULISSE_CTRL_GENERICSTATE_HPP

#include <fsm/fsm.h>
#include <rclcpp/logger.hpp>
#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_ctrl/ctrl_data_structs.hpp"

namespace ulisse {

namespace states {

    class GenericState : public fsm::BaseState {
    protected:
        std::shared_ptr<PositionContext> posCxt_;
        std::shared_ptr<ControlContext> ctrlCxt_;
        std::shared_ptr<ConfigurationData> conf_;

    public:
        GenericState(void);
        virtual ~GenericState(void);

        void CheckRadioController();
        void SetPosContext(const std::shared_ptr<PositionContext>& posCxt);
        void SetCtrlContext(const std::shared_ptr<ControlContext>& ctrlCxt);
        void SetConf(const std::shared_ptr<ConfigurationData>& conf);

    };
}
}

#endif // ULISSE_CTRL_GENERICSTATE_HPP
