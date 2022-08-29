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
//const std::string asvAngularPositionILOS = "ASV_Angular_Position_ILOS"; //ILOS
const std::string asvAngularPositionHold = "ASV_Angular_Position_Hold";
const std::string asvAbsoluteAxisAlignment = "ASV_Absolute_Axis_Alignment";
const std::string asvCartesianDistance = "ASV_Cartesian_Distance";
const std::string asvCartesianDistanceHold = "ASV_Cartesian_Distance_Hold";
const std::string asvCartesianDistancePathFollowing = "ASV_Cartesian_Distance_Path_Follow";
const std::string asvSafetyBoundaries = "ASV_Safety_Boundaries";
const std::string asvAbsoluteAxisAlignmentSafety = "ASV_Absolute_Axis_Alignment_Safety";
const std::string asvAbsoluteAxisAlignmentHold = "ASV_Absolute_Axis_Alignment_Hold";
const std::string asvAbsoluteAxisAlignmentILOS = "ASV_Absolute_Axis_Alignment_ILOS"; //ILOS
const std::string asvLinearVelocityHold = "ASV_Linear_Velocity_Hold";
const std::string asvLinearVelocityCurrentEst = "ASV_Linear_Velocity_CurrentEst";
const std::string asvAbsoluteAxisAlignmentCurrentEst = "ASV_Absolute_Axis_Alignment_CurrentEst";

}

namespace action {

const std::string goTo = "Move_To";
const std::string halt = "Halt";
const std::string hold = "Hold";
const std::string surge_heading = "Surge_Heading";
const std::string surge_yawrate = "Surge_YawRate";
const std::string pathfollow = "Path_Following";
const std::string pathfollow_ilos = "Path_FollowingILOS"; //ILOS
const std::string pathfollow_current = "Path_FollowingCurrentEst";
const std::string pathfollow_iloscurrent = "Path_FollowingILOSCurrentEst"; // ILOS current
}

namespace commands {

namespace ID {

const std::string halt = "halt_command";
const std::string latlong = "latlong_command";
const std::string hold = "hold_command";
const std::string surgeheading = "surgeheading_command";
const std::string surgeyawrate = "surgeyawrate_command";
const std::string pathfollow = "pathfollow_command";
const std::string pathfollow_ilos = "pathfollow_ilos_command";
const std::string pathfollow_current = "pathfollow_current_command";
const std::string pathfollow_iloscurrent = "pathfollow_iloscurrent_command";
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
const std::string pathfollow_ilos = "Path_FollowingILOS";
const std::string pathfollow_current = "Path_FollowingCurrentEst";
const std::string pathfollow_iloscurrent = "Path_FollowingILOSCurrentEst";
}
}

namespace events {

namespace names {
const char* const neargoalposition = "NEARGOALPOSITION";
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

namespace pathFollowModes {
enum { LOS = 0, ILOS = 1 ,LOSandCurrentEst = 2, ILOS_LOSandCurrentEst = 3};
}

}
#endif
