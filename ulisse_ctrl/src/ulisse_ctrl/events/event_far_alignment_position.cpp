#include "ulisse_ctrl/events/event_far_alignment_position.hpp"

namespace ulisse {

namespace events {

    fsm::retval EventFarAlignmentPosition::Execute()
    {
        //std::cout << "Executing: Event Far-Alignment-Point" << std::endl;
        //if (!ctrlData_->cableCurrentAligned && ctrlData_->avoidancePathGenerated) {
        if (ctrlData_->avoidancePathGenerated) {
            if (ctrlData_->avoidancePath.path.type == "PointPath"){
                ctrlData_->preStateRovFollow = true;
                statePathFollowing_->LoadPath(ctrlData_->avoidancePath.path);
                fsm_->SetNextState("Path_Following");
                std::cout << "Executing: Event Far-Alignment-Point" << std::endl;
                //ctrlData_->pathFollowRequested = true;
                ctrlData_->preApReached = false;
            }

            //ctrlData_->avoidancePathGenerated = false;
        }
//        else {
//            //fsm_->SetNextState("Halt");
//        }

        return fsm::retval::ok;
    }

    fsm::retval EventFarAlignmentPosition::Propagate(void)
    {
        //std::cout << "Executing Event" << std::endl;
        return fsm::retval::ok;
    }

} // namespace events

} // namespace ulisse
