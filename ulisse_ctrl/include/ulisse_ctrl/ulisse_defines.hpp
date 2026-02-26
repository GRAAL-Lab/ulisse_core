#ifndef ULISSEDEFINES_H
#define ULISSEDEFINES_H

/*#include <rml/RMLDefines.h>
#include <vector>*/
#include <string>

namespace ulisse {

namespace robotModelID {
const std::string ASV = "Ulisse";
}

namespace task {

const std::string asvLinearVelocity = "ASV_Linear_Velocity";
const std::string asvAngularPosition = "ASV_Angular_Position";
const std::string asvAngularPositionHold = "ASV_Angular_Position_Hold";
const std::string asvAngularPositionRovFollow = "ASV_Angular_Position_ROV_Follow"; // ROV
const std::string asvAngularPositionObstacle = "ASV_Angular_Position_Obstacle"; // Obstacle
const std::string asvAbsoluteAxisAlignment = "ASV_Absolute_Axis_Alignment";
const std::string asvCartesianDistance = "ASV_Cartesian_Distance";
const std::string asvCartesianDistanceHold = "ASV_Cartesian_Distance_Hold";
const std::string asvCartesianDistancePathFollowing = "ASV_Cartesian_Distance_Path_Follow";
const std::string asvCartesianDistanceRovFollowing = "ASV_Cartesian_Distance_ROV_Follow"; // ROV
const std::string asvCartesianDistanceObstacle = "ASV_Cartesian_Distance_Obstacle"; // Obstacle
const std::string asvSafetyBoundaries = "ASV_Safety_Boundaries";
const std::string asvAbsoluteAxisAlignmentSafety = "ASV_Absolute_Axis_Alignment_Safety";
const std::string asvAbsoluteAxisAlignmentHold = "ASV_Absolute_Axis_Alignment_Hold";
const std::string asvAbsoluteAxisAlignmentObstacle = "ASV_Absolute_Axis_Alignment_Obstacle"; // ASV-ROV obstacle avoidance
const std::string asvLinearVelocityHold = "ASV_Linear_Velocity_Hold";
const std::string asvObstacleAvoidance = "ASV_Obstacle_Avoidance"; // ASV-ROV obstacle avoidance
const std::string asvLinearVelocityObstacle = "ASV_Linear_Velocity_Obstacle"; // ASV-ROV obstacle

}

namespace action {

const std::string goTo = "Move_To";
const std::string halt = "Halt";
const std::string hold = "Hold";
const std::string surge_heading = "Surge_Heading";
const std::string surge_yawrate = "Surge_YawRate";
const std::string pathfollow = "Path_Following";
const std::string rovfollow = "ROV_Following";
}

namespace obstacle {

const std::string obstacle1 = "Obstacle_1";
const std::string obstacle2 = "Obstacle_2";
const std::string obstacle3 = "Obstacle_3";

}

namespace commands {

namespace ID {

const std::string halt = "halt_command";
const std::string latlong = "latlong_command";
const std::string hold = "hold_command";
const std::string surgeheading = "surgeheading_command";
const std::string surgeyawrate = "surgeyawrate_command";
const std::string pathfollow = "pathfollow_command";
const std::string rovfollow = "rovfollow_command";
}
}

namespace states {

namespace ID {

const std::string latlong = "Move_To";
const std::string halt = "Halt";
const std::string hold = "Hold";
const std::string surgeheading = "Surge_Heading";
const std::string surgeyawrate = "Surge_YawRate";
const std::string pathfollow = "Path_Following";
const std::string rovfollow = "ROV_Following";
}
}

namespace events {

namespace names {
const char* const neargoalposition = "NEARGOALPOSITION";
const char* const longtether = "LONGTETHER";
const char* const faralignmentposition = "FARALIGNMENTPOSITION";
const char* const switchstate = "SWITCHSTATE";
//const char* const surgeheadingtimeout = "SURGEHEADINGTIMEOUT";
//const char* const surgeyawratetimeout = "SURGEYAWRATETIMEOUT";
const char* const rcenabled = "RCENABLED";
}

namespace topicnames {
const char* const events = "/ctrl/out/events";
}

namespace priority {
const uint8_t high = 10;
const uint8_t medium = 5;
const uint8_t low = 1;
}
}

}
#endif
