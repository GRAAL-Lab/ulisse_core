#ifndef ULISSE_CTRL_DATA_STRUCTS_HPP
#define ULISSE_CTRL_DATA_STRUCTS_HPP

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>
#include <tpik/TPIK.h>

#include "rclcpp/rclcpp.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ctrl_toolbox/pid/DigitalPID.h"
#include "surface_vehicle_model/surfacevehiclemodel.hpp"
#include "ulisse_msgs/msg/task_status.hpp"


namespace ulisse {

struct ControlData {
    ctb::LatLong inertialF_linearPosition;
    rml::EulerRPY bodyF_angularPosition;
    Eigen::Vector3d bodyF_linearVelocity;
    Eigen::Vector3d bodyF_angularVelocity;
    Eigen::Vector2d inertialF_waterCurrent;
    bool radioControllerEnabled;

    ControlData() : radioControllerEnabled(false) {}
};

struct TasksInfo {

    std::shared_ptr<tpik::Task> task;
    rclcpp::Publisher<ulisse_msgs::msg::TaskStatus>::SharedPtr taskPub;
};

enum class ControlMode : int {
    ThrusterMapping,
    ClassicPIDControl,
    ComputedTorque
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

    bool ConfigureFromFile(libconfig::Config& confObj)
    {

        if (!ctb::GetParam(confObj, goToHoldAfterMove, "goToHoldAfterMove"))
            return false;
        if (!ctb::GetParam(confObj, controlLoopPeriod, "controlLoopPeriod"))
            return false;
        if (!ctb::GetParam(confObj, posAcceptanceRadius, "posAcceptanceRadius"))
            return false;
        if (!ctb::GetParamVector(confObj, saturationMax, "saturationMax"))
            return false;
        if (!ctb::GetParamVector(confObj, saturationMin, "saturationMin"))
            return false;

        return true;
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

struct ThrusterMapping {
    ctb::PIDGains pidGainsSurge;
    double pidSatSurge;

    bool ConfigureFromFile(const libconfig::Setting& confObj) noexcept(false)
    {
        const libconfig::Setting& pidSurge = confObj["pidSurge"];

        if (!ctb::GetParam(pidSurge, pidGainsSurge.Kd, "kd"))
            return false;
        if (!ctb::GetParam(pidSurge, pidGainsSurge.Kp, "kp"))
            return false;
        if (!ctb::GetParam(pidSurge, pidGainsSurge.Ki, "ki"))
            return false;
        if (!ctb::GetParam(pidSurge, pidGainsSurge.Kff, "kff"))
            return false;
        if (!ctb::GetParam(pidSurge, pidGainsSurge.N, "n"))
            return false;
        if (!ctb::GetParam(pidSurge, pidGainsSurge.Tr, "tr"))
            return false;
        if (!ctb::GetParam(pidSurge, pidSatSurge, "sat"))
            return false;

        return true;
    }

    friend std::ostream& operator<<(std::ostream& os, ThrusterMapping const& a)
    {
        return os << "======= Thruster Mapping CONF =======\n"
                  << "Pid Surge: "
                  << "\n"
                  << "Kd: " << a.pidGainsSurge.Kd << "\n"
                  << "Kp: " << a.pidGainsSurge.Kp << "\n"
                  << "Ki: " << a.pidGainsSurge.Ki << "\n"
                  << "Kff: " << a.pidGainsSurge.Kff << "\n"
                  << "N: " << a.pidGainsSurge.N << "\n"
                  << "Tr: " << a.pidGainsSurge.Tr << "\n"
                  << "Saturation: " << a.pidSatSurge << "\n"
                  << "==============================\n";
    }
};

struct DynamicPid {
    ctb::PIDGains pidGainsSurge;
    double pidSatSurge;
    ctb::PIDGains pidGainsYawRate;
    double pidSatYawRate;

    bool ConfigureFromFile(const libconfig::Setting& confObj) noexcept(false)
    {
        const libconfig::Setting& pidSurge = confObj["pidSurge"];

        if (!ctb::GetParam(pidSurge, pidGainsSurge.Kd, "kd"))
            return false;
        if (!ctb::GetParam(pidSurge, pidGainsSurge.Kp, "kp"))
            return false;
        if (!ctb::GetParam(pidSurge, pidGainsSurge.Ki, "ki"))
            return false;
        if (!ctb::GetParam(pidSurge, pidGainsSurge.Kff, "kff"))
            return false;
        if (!ctb::GetParam(pidSurge, pidGainsSurge.N, "n"))
            return false;
        if (!ctb::GetParam(pidSurge, pidGainsSurge.Tr, "tr"))
            return false;
        if (!ctb::GetParam(pidSurge, pidSatSurge, "sat"))
            return false;

        const libconfig::Setting& pidYawRate = confObj["pidYawRate"];

        if (!ctb::GetParam(pidYawRate, pidGainsYawRate.Kd, "kd"))
            return false;
        if (!ctb::GetParam(pidYawRate, pidGainsYawRate.Kp, "kp"))
            return false;
        if (!ctb::GetParam(pidYawRate, pidGainsYawRate.Ki, "ki"))
            return false;
        if (!ctb::GetParam(pidYawRate, pidGainsYawRate.Kff, "kff"))
            return false;
        if (!ctb::GetParam(pidYawRate, pidGainsYawRate.N, "n"))
            return false;
        if (!ctb::GetParam(pidYawRate, pidGainsYawRate.Tr, "tr"))
            return false;
        if (!ctb::GetParam(pidYawRate, pidSatYawRate, "sat"))
            return false;

        return true;
    }

    friend std::ostream& operator<<(std::ostream& os, DynamicPid const& a)
    {
        return os << "======= Classic Pid Control CONF =======\n"
                  << "Pid Surge: "
                  << "\n"
                  << "Kd: " << a.pidGainsSurge.Kd << "\n"
                  << "Kp: " << a.pidGainsSurge.Kp << "\n"
                  << "Ki: " << a.pidGainsSurge.Ki << "\n"
                  << "Kff: " << a.pidGainsSurge.Kff << "\n"
                  << "N: " << a.pidGainsSurge.N << "\n"
                  << "Tr: " << a.pidGainsSurge.Tr << "\n"
                  << "Saturation: " << a.pidSatSurge << "\n"
                  << "----------------------\n"
                  << "Pid Yaw Rate: "
                  << "\n"
                  << "Kd: " << a.pidGainsYawRate.Kd << "\n"
                  << "Kp: " << a.pidGainsYawRate.Kp << "\n"
                  << "Ki: " << a.pidGainsYawRate.Ki << "\n"
                  << "Kff: " << a.pidGainsYawRate.Kff << "\n"
                  << "N: " << a.pidGainsYawRate.N << "\n"
                  << "Tr: " << a.pidGainsYawRate.Tr << "\n"
                  << "Saturation: " << a.pidSatYawRate << "\n"
                  << "==============================\n";
    }
};

struct DCLConfiguration {

    bool enableThrusters;
    double thrusterPercLimit;
    ControlMode ctrlMode;
    double surgeMin, surgeMax;
    double yawRateMin, yawRateMax;

    UlisseModelParameters ulisseModel;
    ThrusterMapping thrusterMapping;
    DynamicPid classicPidControl;
    DynamicPid computedTorqueControl;

    friend std::ostream& operator<<(std::ostream& os, DCLConfiguration const& a)
    {
        os << "======= DCL CONF =======\n"
           << "CtrlMode: " << static_cast<int>(a.ctrlMode) << "\n"
           << "EnableThrusters: " << a.enableThrusters << "\n"
           << "ThrusterPercLimit: " << a.thrusterPercLimit << "\n"
           << "----------------------\n"
           << a.ulisseModel
           << "----------------------\n";
        if (a.ctrlMode == ControlMode::ThrusterMapping) {
            os << a.thrusterMapping;
        } else if (a.ctrlMode == ControlMode::ClassicPIDControl) {
            os << a.classicPidControl;
        } else {
            os << a.computedTorqueControl;
        }

        os << "==============================\n";
        return os;
    }

    bool ConfigureFromFile(libconfig::Config& confObj) noexcept(false)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& dcl = root["dcl_ulisse"];
        // Load DCL Config
        int tmpCtrlMode;
        if (!ctb::GetParam(dcl, tmpCtrlMode, "ctrlMode"))
            return false;
        ctrlMode = static_cast<ControlMode>(tmpCtrlMode);
        if (!ctb::GetParam(dcl, enableThrusters, "enableThrusters"))
            return false;
        if (!ctb::GetParam(dcl, thrusterPercLimit, "thrusterPercLimit"))
            return false;
        if (!ctb::GetParam(dcl, surgeMin, "surgeMin"))
            return false;
        if (!ctb::GetParam(dcl, surgeMax, "surgeMax"))
            return false;
        if (!ctb::GetParam(dcl, yawRateMin, "yawRateMin"))
            return false;
        if (!ctb::GetParam(dcl, yawRateMax, "yawRateMax"))
            return false;

        const libconfig::Setting& ulisseModelParams = dcl["ulisseModel"];
        if (!ulisseModel.ConfigureFromFile(ulisseModelParams))
            return false;

        std::cerr << "ulisseModelParams configured" << std::endl;

        if (ctrlMode == ControlMode::ThrusterMapping) {
            const libconfig::Setting& thrusterMap = dcl["thrusterMapping"];
            if (!thrusterMapping.ConfigureFromFile(thrusterMap))
                return false;
            std::cerr << "ThrusterMapping configured" << std::endl;
        } else if (ctrlMode == ControlMode::ClassicPIDControl) {
            const libconfig::Setting& classicPidCtr = dcl["classicPidControl"];
            if (!classicPidControl.ConfigureFromFile(classicPidCtr))
                return false;
            std::cerr << "ClassicPIDControl configured" << std::endl;
        } else if (ctrlMode == ControlMode::ComputedTorque) {
            const libconfig::Setting& computedTorqueCtr = dcl["computedTorqueControl"];
            if (!computedTorqueControl.ConfigureFromFile(computedTorqueCtr))
                return false;

        } else {
            std::cerr << "Type of control not recognized" << std::endl;
        }

        return true;
    }
};
}

#endif //  ULISSE_CTRL_DATA_STRUCTS_HPP
