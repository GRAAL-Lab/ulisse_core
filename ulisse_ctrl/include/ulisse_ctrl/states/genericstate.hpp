#ifndef ULISSE_CTRL_GENERICSTATE_HPP
#define ULISSE_CTRL_GENERICSTATE_HPP

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include <fsm/fsm.h>
#include <ikcl/ikcl.h>
#include <libconfig.h++>

//to be delete once safetyB in ickc
#include "ulisse_ctrl/tasks/SafetyBoundaries.h"

namespace ulisse {

namespace states {

    class GenericState : public fsm::BaseState {
    protected:
        std::shared_ptr<ikcl::SafetyBoundaries> safetyBoundariesTask_;
        std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentSafetyTask_;

        double minHeadingError_, maxHeadingError_;

    public:
        struct StateCtx {
            std::shared_ptr<StatusContext> statusCxt;
            std::shared_ptr<GoalContext> goalCxt;
            std::shared_ptr<ControlContext> ctrlCxt;
            std::shared_ptr<tpik::ActionManager> actionManager;
            std::shared_ptr<rml::RobotModel> robotModel;
            std::unordered_map<std::string, TasksInfo> tasksMap;
        } stateCtx_;

        GenericState();
        virtual ~GenericState(void);

        void CheckRadioController();
        void SetStateCtx(StateCtx stateCtx);
        virtual void ConfigureStateFromFile(libconfig::Config& confObj) = 0;
    };
}
}

#endif // ULISSE_CTRL_GENERICSTATE_HPP
