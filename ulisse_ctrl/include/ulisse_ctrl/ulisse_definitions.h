#ifndef ULISSEDEFINES_H
#define ULISSEDEFINES_H

#include <rml/RMLDefines.h>
#include <vector>

namespace ulisse {
namespace task {

    const std::string asvLinearVelocity = "ASV_Linear_Velocity";
    const std::string asvAngularPosition = "ASV_Angular_Position";
    const std::string asvAngularPositionHold = "ASV_Angular_Position_Hold";
    const std::string asvAbsoluteAxisAlignment = "ASV_Absolute_Axis_Alignment";
    const std::string asvCartesianDistance = "ASV_Cartesian_Distance";
    const std::string asvCartesianDistanceHold = "ASV_Cartesian_Distance_Hold";
    const std::string asvCartesianDistancePathFollowing = "ASV_Cartesian_Distance_Path_Following";
    const std::string asvSafetyBoundaries = "ASV_Safety_Boundaries";
    const std::string asvAbsoluteAxisAlignmentSafety = "ASV_Absolute_Axis_Alignment_Safety";
    const std::string asvAbsoluteAxisAlignmentHold = "ASV_Absolute_Axis_Alignment_Hold";
    const std::string asvLinearVelocityHold = "ASV_Linear_Velocity_Hold";

}

namespace action {

    const std::string goTo = "Go_To";
    const std::string idle = "Idle";
    const std::string hold = "Hold";
    const std::string speed_heading = "Speed_Heading";
    const std::string navigate = "Path_Following";
}
}
#endif
