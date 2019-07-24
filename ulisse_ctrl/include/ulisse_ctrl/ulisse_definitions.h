#ifndef ULISSEDEFINES_H
#define ULISSEDEFINES_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <rml/RMLDefines.h>
#include <vector>

namespace ulisse {
    namespace task {
        const std::string task = "task";
        const std::string asv_control_velocity_linear = "ASV_control_velocity_linear";
        const std::string asv_control_velocity_angular = "ASV_control_velocity_angular";
        const std::string asv_hold_position = "ASV_hold_position";
        const std::string asv_angular_position = "ASV_angular_position";
        const std::string asv_control_distance = "ASV_control_distance";
        const std::string asv_make_curve = "ASV_make_curve";
        const std::string asv_safety_boundaries = "ASV_safety_boundaries";

        const std::vector<std::string> unified_hierarchy = { asv_safety_boundaries, asv_hold_position, asv_make_curve,
                                                             asv_angular_position, asv_control_velocity_angular,
                                                             asv_control_distance, asv_control_velocity_linear
        };
    }

    namespace taskParameter {
        const std::string gain = "gain";
        const std::string saturation = "saturation";
        const std::string taskEnable = "taskEnable";
        const std::string controlReference = "controlReference";
        const std::string virtualFrameGain = "virtualFrameGain";
        const std::string onTrackThreshold = "onTrackThreshold";
        const std::string crossTrackThrehsodl = "crossTrackThrehsold";
        const std::string axis = "axis";
        const std::string deltaJL = "deltaJL";
        const std::string muReference = "muReferenceValue";
        const std::string tollerance = "tollerance";
    }

    namespace priorityLevelID {
        const std::string asv_safety_boundaries = "ASV_safety_boundaries_PL";
        const std::string asv_make_curve = "ASV_make_curve_PL";
        const std::string asv_control_velocity_linear = "ASV_control_velocity_linear_PL";
        const std::string asv_control_velocity_angular = "ASV_control_velocity_angular_PL";
        const std::string asv_control_position_angular = "ASV_control_position_angular_PL";
        const std::string asv_control_distance = "ASV_control_distance_PL";
        const std::string asv_hold_position = "ASV_hold_position_PL";
        const std::vector<std::string> unified_hierarchy = { asv_safety_boundaries, asv_hold_position, asv_make_curve,
                                                             asv_control_position_angular, asv_control_velocity_angular,
                                                             asv_control_distance, asv_control_velocity_linear
        }; //auv descendent movement lowest priority
    }

    namespace priorityLevelParameter {
        const std::string priorityLevel = "priorityLevel";
        const std::string regularizationParameter = "regularizationParameter";
        const std::string lambda = "lambda";
        const std::string threshold = "threshold";
        const std::string saturationMax = "saturationMax";
        const std::string saturationMin = "saturationMin";
    }

    namespace bellShapeParameter {
        const std::string increasingBellShape = "increasingBellShape";
        const std::string decreasingBellShape = "decreasingBellShape";
        const std::string xmin = "xmin";
        const std::string xmax = "xmax";
    }

    namespace action {

        const std::string goTo = "goTo";
        const std::vector<std::string> goToPriorityLevels = { priorityLevelID::asv_safety_boundaries, priorityLevelID::asv_control_position_angular,
                                                              priorityLevelID::asv_control_velocity_angular ,
                                                              priorityLevelID::asv_control_distance ,
                                                              priorityLevelID::asv_control_velocity_linear };

        const std::string orient = "orient";
        const std::vector<std::string> orientPriorityLevels = {};

        const std::string idle = "idle";
        const std::vector<std::string> idlePriorityLevels = { priorityLevelID::asv_safety_boundaries,
                                                              priorityLevelID::asv_control_velocity_angular, priorityLevelID::asv_control_velocity_linear };

        const std::string hold = "hold";
        const std::vector<std::string> holdPriorityLevels = { priorityLevelID::asv_safety_boundaries,
                                                              priorityLevelID::asv_hold_position, priorityLevelID::asv_control_position_angular,
                                                              priorityLevelID::asv_control_velocity_angular,
                                                              priorityLevelID::asv_control_distance , priorityLevelID::asv_control_velocity_linear  };

        const std::string speed_heading = "speed_heading";
        const std::vector<std::string> speed_headingPriorityLevels = { priorityLevelID::asv_safety_boundaries,
                                                                       priorityLevelID::asv_control_position_angular, priorityLevelID::asv_control_velocity_angular,
                                                                       priorityLevelID::asv_control_velocity_linear };

        const std::string navigate = "navigate";
        const std::vector<std::string> navigatePriorityLevels = { priorityLevelID::asv_safety_boundaries,
                                                                  priorityLevelID::asv_control_position_angular,
                                                                  priorityLevelID::asv_control_velocity_angular,
                                                                  priorityLevelID::asv_control_distance , priorityLevelID::asv_control_velocity_linear
                                                                  };



    }
}
#endif