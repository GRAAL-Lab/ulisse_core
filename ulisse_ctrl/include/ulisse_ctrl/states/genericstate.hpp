#ifndef ULISSE_CTRL_GENERICSTATE_HPP
#define ULISSE_CTRL_GENERICSTATE_HPP

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include <fsm/fsm.h>
#include <ikcl/ikcl.h>

//to be delete once safetyB in ickc
#include "ulisse_ctrl/tasks/SafetyBoundaries.h"

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

        std::shared_ptr<ikcl::SafetyBoundaries> safetyBoundariesTask_;
        std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentSafetyTask_;

        double minHeadingErrorSafety_, maxHeadingErrorSafety_, maxGainSafety_;
        Eigen::Vector3d desiredVelocitySafety_;

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

        void SetSafetyBoundariesTask(std::shared_ptr<ikcl::SafetyBoundaries> safetyBoundariesTask);
        void SetAngularPositionSafetyTask(std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentSafetyTask);
        void SetMinMaxHeadingErrorSafety(double min, double max);
        void SetDesiredVelocitySafety(Eigen::Vector3d desiredVelocity);
        void SetMaxGainSafety(double maxGainSafety);
    };
}
}

#endif // ULISSE_CTRL_GENERICSTATE_HPP
