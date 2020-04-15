#ifndef ULISSEDEFINES_H
#define ULISSEDEFINES_H

#include <rml/RMLDefines.h>
#include <vector>

namespace ulisse {
namespace task {

    const std::string asv_control_velocity_linear = "ASV_Linear_Velocity";
    const std::string asv_angular_position = "ASV_Angular_Position";
    const std::string asv_absolute_axis_alignment = "ASV_Absolute_Axis_Alignment";
    const std::string asv_control_distance = "ASV_Cartesian_Distance";
    const std::string asv_safety_boundaries = "ASV_Safety_Boundaries";
    const std::string asv_absolute_axis_alignment_safety = "ASV_Absolute_Axis_Alignment_Safety";
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
