#ifndef ULISSE_CTRL_DATA_STRUCTS_HPP
#define ULISSE_CTRL_DATA_STRUCTS_HPP

#include "ctrl_toolbox/HelperFunctions.h"
#include "ctrl_toolbox/pid/DigitalPID.h"
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

struct ThrusterMapping {
    ctb::PIDGains pidGainsSurge;
    double pidSatSurge;

    void ConfigureFromFile(const libconfig::Setting& confObj) noexcept(false)
    {
        const libconfig::Setting& pidSurge = confObj["pidSurge"];

        ctb::SetParam(pidSurge, pidGainsSurge.Kd, "kd");
        ctb::SetParam(pidSurge, pidGainsSurge.Kp, "kp");
        ctb::SetParam(pidSurge, pidGainsSurge.Ki, "ki");
        ctb::SetParam(pidSurge, pidGainsSurge.Kff, "kff");
        ctb::SetParam(pidSurge, pidGainsSurge.N, "n");
        ctb::SetParam(pidSurge, pidGainsSurge.Tr, "tr");
        ctb::SetParam(pidSurge, pidSatSurge, "sat");
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

struct ClassicPidControl {
    ctb::PIDGains pidGainsSurge;
    double pidSatSurge;
    ctb::PIDGains pidGainsYawRate;
    double pidSatYawRate;

    void ConfigureFromFile(const libconfig::Setting& confObj) noexcept(false)
    {
        const libconfig::Setting& pidSurge = confObj["pidSurge"];

        ctb::SetParam(pidSurge, pidGainsSurge.Kd, "kd");
        ctb::SetParam(pidSurge, pidGainsSurge.Kp, "kp");
        ctb::SetParam(pidSurge, pidGainsSurge.Ki, "ki");
        ctb::SetParam(pidSurge, pidGainsSurge.Kff, "kff");
        ctb::SetParam(pidSurge, pidGainsSurge.N, "n");
        ctb::SetParam(pidSurge, pidGainsSurge.Tr, "tr");
        ctb::SetParam(pidSurge, pidSatSurge, "sat");

        const libconfig::Setting& pidYawRate = confObj["pidYawRate"];
        ctb::SetParam(pidYawRate, pidGainsYawRate.Kd, "kd");
        ctb::SetParam(pidYawRate, pidGainsYawRate.Kp, "kp");
        ctb::SetParam(pidYawRate, pidGainsYawRate.Ki, "ki");
        ctb::SetParam(pidYawRate, pidGainsYawRate.Kff, "kff");
        ctb::SetParam(pidYawRate, pidGainsYawRate.N, "n");
        ctb::SetParam(pidYawRate, pidGainsYawRate.Tr, "tr");
        ctb::SetParam(pidYawRate, pidSatYawRate, "sat");
    }

    friend std::ostream& operator<<(std::ostream& os, ClassicPidControl const& a)
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
    ClassicPidControl classicPidControl;

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
        } else {
            return os << a.classicPidControl;
        }

        os << "==============================\n";
        return os;
    }

    void ConfigureFromFile(libconfig::Config& confObj) noexcept(false)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& dcl = root["dcl_ulisse"];
        // Load DCL Config
        int tmpCtrlMode;
        ctb::SetParam(dcl, tmpCtrlMode, "ctrlMode");
        ctrlMode = static_cast<ControlMode>(tmpCtrlMode);
        ctb::SetParam(dcl, enableThrusters, "enableThrusters");
        ctb::SetParam(dcl, thrusterPercLimit, "thrusterPercLimit");
        ctb::SetParam(dcl, surgeMin, "surgeMin");
        ctb::SetParam(dcl, surgeMax, "surgeMax");
        ctb::SetParam(dcl, yawRateMin, "yawRateMin");
        ctb::SetParam(dcl, yawRateMax, "yawRateMax");

        const libconfig::Setting& ulisseModelParams = dcl["ulisseModel"];
        ulisseModel.ConfigureFormFile(ulisseModelParams);

        if (ctrlMode == ControlMode::ThrusterMapping) {
            const libconfig::Setting& thrusterMap = dcl["thrusterMapping"];
            thrusterMapping.ConfigureFromFile(thrusterMap);
        } else if (ctrlMode == ControlMode::ClassicPIDControl) {
            const libconfig::Setting& classicPidCtr = dcl["classicPidControl"];
            classicPidControl.ConfigureFromFile(classicPidCtr);
        } else {
            std::cerr << "Type of control not recognized" << std::endl;
        }
    }
};
}

#endif //  ULISSE_CTRL_DATA_STRUCTS_HPP
