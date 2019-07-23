#ifndef ULISSE_CTRL_GENERICSTATE_HPP
#define ULISSE_CTRL_GENERICSTATE_HPP

#include <fsm/fsm.h>
#include <rclcpp/logger.hpp>

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"

#include "ulisse_ctrl/tasks/ActionTask.h"
#include "ulisse_ctrl/tasks/AngularPosition.h"
#include "ulisse_ctrl/tasks/ControlDistance.h"
#include "ulisse_ctrl/tasks/Hold.h"
#include "ulisse_ctrl/tasks/MakeCurve.h"

#include <ikcl/ikcl.h>
#include <rml/RML.h>
#include <tpik/TPIKlib.h>

#include <ulisse_ctrl/geometry_defines.h>
#include <ulisse_ctrl/ulisse_definitions.h>

namespace ulisse {

namespace states {

    class GenericState : public fsm::BaseState {
    protected:
        std::shared_ptr<ControllerConfiguration> conf_;
        std::shared_ptr<StatusContext> statusCxt_;
        std::shared_ptr<GoalContext> goalCxt_;
        std::shared_ptr<ControlContext> ctrlCxt_;

        std::shared_ptr<tpik::ActionManager> actionManager_;
        std::vector<std::shared_ptr<tpik::Task>> unifiedHierarchy_;
        std::shared_ptr<rml::RobotModel> robotModel_;

    public:
        GenericState(void);
        virtual ~GenericState(void);

        void CheckRadioController();
        void SetConf(const std::shared_ptr<ControllerConfiguration>& conf);
        void SetStatusContext(const std::shared_ptr<StatusContext>& posCxt);
        void SetGoalContext(const std::shared_ptr<GoalContext>& goalCxt);
        void SetCtrlContext(const std::shared_ptr<ControlContext>& ctrlCxt);

        void SetActionManager(std::shared_ptr<tpik::ActionManager> actionManager);
        void SetUnifiedHierarchy(std::vector<std::shared_ptr<tpik::Task>> unifiedHierarchy);
        void SetRobotModel(std::shared_ptr<rml::RobotModel> robotModel);
    };
}
}

#endif // ULISSE_CTRL_GENERICSTATE_HPP
