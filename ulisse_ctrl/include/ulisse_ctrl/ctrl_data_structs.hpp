#ifndef ULISSE_CTRL_DATA_STRUCTS_HPP
#define ULISSE_CTRL_DATA_STRUCTS_HPP

#include "ctrl_toolbox/DigitalPID.h"
#include "ctrl_toolbox/HelperFunctions.h"
#include "rclcpp/rclcpp.hpp"
#include "surface_vehicle_model/surfacevehiclemodel.hpp"
#include "ulisse_msgs/msg/task_status.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>
#include <tpik/TPIK.h>

namespace ulisse {

struct TasksInfo {

    std::shared_ptr<tpik::Task> task;
    rclcpp::Publisher<ulisse_msgs::msg::TaskStatus>::SharedPtr taskPub;
};

enum class ControlMode : int {
    ThrusterMapping,
    ClassicPIDControl,
    SlidingMode
};

struct MotorReference {
    double left;
    double right;
    MotorReference()
        : left(0.0)
        , right(0.0)
    {
    }
};

struct ThrusterControlData {
    double desiredSurge;
    double desiredJog;
    MotorReference mapOut, ctrlRef;
};

struct Waypoint {
    ctb::LatLong pos;
    double acceptRadius;
    Waypoint()
        : acceptRadius(1.0)
    {
    }
};

struct StatusContext {
    ctb::LatLong vehiclePos;
    double gpsTrack, gpsSpeed;
    double vehicleHeading;

    double seacurrent[2];

    uint16_t llcStatus;
    uint16_t sw485Status;

    std::string vehicleState;

    StatusContext()
        : vehicleHeading(0.0)
        , llcStatus(0)
    {
    }
};

struct ControlContext {
    ctb::DigitalPID pidSurge;
    ctb::DigitalPID pidPosition;
    ctb::DigitalPID pidHeading;
    double desiredSurge;
    double desiredJog;
};

struct GoalContext {
    Waypoint currentGoal, nextGoal;
    double goalDistance, goalHeading, goalSurge;
    uint cmdTimeout;
    GoalContext()
        : goalDistance(0.0)
        , goalHeading(0.0)
    {
    }
};

struct ControllerConfiguration {

    bool goToHoldAfterMove;
    double posAcceptanceRadius;
    double controlLoopPeriod;
    Eigen::VectorXd saturationMin, saturationMax;

    ControllerConfiguration()
        : goToHoldAfterMove(false)
    {
    }

    void ConfigureFromFile(libconfig::Config& confObj)
    {
        ctb::SetParam(confObj, goToHoldAfterMove, "goToHoldAfterMove");
        ctb::SetParam(confObj, controlLoopPeriod, "controlLoopPeriod");
        ctb::SetParam(confObj, posAcceptanceRadius, "posAcceptanceRadius");
        ctb::SetParamVector(confObj, saturationMax, "saturationMax");
        ctb::SetParamVector(confObj, saturationMin, "saturationMin");
    }

    friend std::ostream& operator<<(std::ostream& os, ControllerConfiguration const& a)
    {
        return os << "======= CONTROLLER CONF =======\n"
                  << "ControlLoopPeriod: " << a.controlLoopPeriod << "\n"
                  << "PosAcceptanceRadius: " << a.posAcceptanceRadius << "\n"
                  << "GoToHoldAfterMove: " << a.goToHoldAfterMove << "\n"
                  << "SaturationMin: " << a.saturationMin.transpose() << "\n"
                  << "SaturationMax: " << a.saturationMax.transpose() << "\n"
                  << "===============================\n";
    }
};

struct SlidingSurface {

    std::vector<double> cX;
    std::vector<double> cN;
    std::vector<double> inertia;
    double k;
    double k1;
};

struct SlidingParameter {
    double gain1, gain2, surgeGain, headingGain;
    double forceLimiter, torqueLimiter;
};

struct SlidingMode {
    SlidingParameter sp;
};

struct ThrusterMapping {
    ctb::PIDGains pidGainsSurge;
    double pidSatSurge;
};

struct ClassicPidControl {
    ctb::PIDGains pidGainsSurge;
    double pidSatSurge;
    ctb::PIDGains pidGainsYawRate;
    double pidSatYawRate;
};

struct DCLConfiguration {

    bool enableThrusters;
    double thrusterPercLimit;
    ControlMode ctrlMode;
    double surgeMin, surgeMax;
    double yawRateMin, yawRateMax;

    UlisseModelParameters ulisseConfig;
    ThrusterMapping thrusterMapping;
    ClassicPidControl classicPidControl;
    SlidingMode slidingMode;

    Eigen::Vector2d filterParameter;

    friend std::ostream& operator<<(std::ostream& os, DCLConfiguration const& a)
    {
        return os << "======= DCL CONF =======\n"
                  << "CtrlMode: " << static_cast<int>(a.ctrlMode) << "\n"
                  << "EnableThrusters: " << a.enableThrusters << "\n"
                  << "ThrusterPercLimit: " << a.thrusterPercLimit << "\n"
                  << "----------------------\n"
                  << a.ulisseConfig
                  << "----------------------\n"
                  << "==============================\n";
    }
};
}

#endif //  ULISSE_CTRL_DATA_STRUCTS_HPP
