
#ifndef FAR_ALIGNMENT_POSITION_H_
#define FAR_ALIGNMENT_POSITION_H_

#include "fsm/fsm.h"
#include "ulisse_ctrl/states/state_pathfollow.hpp"
#include "ulisse_ctrl/states/state_rovfollow.hpp"

namespace ulisse {

namespace events {

    class EventFarAlignmentPosition : public fsm::BaseEvent {

        //bool goToHold_;
        std::shared_ptr<ulisse::ControlData> ctrlData_;
        std::shared_ptr<ulisse::states::StatePathFollow> statePathFollowing_; // juri
        //std::shared_ptr<ulisse::states::StateRovFollow> stateRovFollow_; // juri

    public:
        EventFarAlignmentPosition(void) {}
        virtual ~EventFarAlignmentPosition(void) {}
        virtual fsm::retval Execute(void);
        virtual fsm::retval Propagate(void);

        auto ControlData() -> std::shared_ptr<ulisse::ControlData>& { return ctrlData_; }
        //auto StateHold() -> std::shared_ptr<ulisse::states::StateHold>& { return stateHold_; }
        //auto StateRovFollow() -> std::shared_ptr<ulisse::states::StateRovFollow>& { return stateRovFollow_; } // juri
        auto StatePathFollow() -> std::shared_ptr<ulisse::states::StatePathFollow>& { return statePathFollowing_; } // juri

        //auto GoToHoldAfterMove(bool flag) { goToHold_ = flag; }
    };

} // namespace events

} // namespace om2ctrl

#endif /* SRC_CTRL_EVENTS_EVENTNEARPOSITION_H_ */
