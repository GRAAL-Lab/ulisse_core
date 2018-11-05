#ifndef ULISSE_CTRL_DATA_STRUCTS_HPP
#define ULISSE_CTRL_DATA_STRUCTS_HPP

#include "ctrl_toolbox/DataStructs.h"
#include "surface_vehicle_model/surfacevehiclemodel.hpp"

namespace ulisse {

struct MotorReference {
    double left;
    double right;
    MotorReference()
        : left(0.0)
        , right(0.0)
    {
    }
};

enum class ControlMode : int {
    ThrusterMapping,
    DynamicModel
};

struct SlowDownOnTurnsData {
    double headingErrorMin; //10.0
    double headingErrorMax; //25.0
    double alphaMin; //0.1
    double alphaMax; // 1.0

    SlowDownOnTurnsData()
        : headingErrorMin(0.0)
        , headingErrorMax(0.0)
        , alphaMin(0.0)
        , alphaMax(0.0)
    {
    }
};

struct ConfigurationData {
    ControlMode ctrlMode;

    bool enableThrusters;
    ThrusterMappingParameters thrusterMap;
    double thrusterPercLimit;

    double posAcceptanceRadius;
    bool enableSlowDownOnTurns;
    SlowDownOnTurnsData sdtData;

    ctb::PIDGains pidgains_speed;
    ctb::PIDGains pidgains_position;
    ctb::PIDGains pidgains_heading;

    double pidsat_speed;
    double pidsat_position;
    double pidsat_heading;

    //double thrusterUpperSat, thrusterLowerSat;

    ConfigurationData()
        : ctrlMode(ControlMode::ThrusterMapping)
        , enableThrusters(false)
        , thrusterPercLimit(0.0)
        , enableSlowDownOnTurns(false)
        , pidsat_speed(0.0)
        , pidsat_position(0.0)
        , pidsat_heading(0.0)
    {
    }

    friend std::ostream& operator<<(std::ostream& os, ConfigurationData const& a)
    {
        return os << "======= CONFIGURATION =======\n"
                  << "CtrlMode: " << (int)a.ctrlMode << "\n"
                  << "EnableThrusters: " << a.enableThrusters << "\n"
                  << "PosAcceptanceRadius: " << a.posAcceptanceRadius << "\n"
                  << "EnableSlowDownOnTurns: " << a.enableSlowDownOnTurns << "\n"
                  << "ThrusterPercLimit: " << a.thrusterPercLimit << "\n"
                  << "----------------------\n"
                  << "Thruster Mapping:\n"
                  << a.thrusterMap << "\n"
                  << "----------------------\n"
                  << "=============================\n";
    }
};

struct ThrusterControlData {
    double desiredSpeed;
    double desiredJog;

    MotorReference mapOut, ctrlRef;
};

struct Spinner {
    Spinner(int frequency)
        : freq(frequency)
        , spinIndex(0)
        , spin_chars("/-\\|")
    {
        clock_gettime(CLOCK_MONOTONIC, &last);
        period = 1 / static_cast<double>(freq + 1E-6);
        //std::cout << "period: " << period << "s" << std::endl;
    }

    void operator()(void)
    {

        clock_gettime(CLOCK_MONOTONIC, &now);
        double timeElapsed = (now.tv_sec - last.tv_sec) + (now.tv_nsec - last.tv_nsec) / 1E9;

        //std::cout << "timeElapsed: " << timeElapsed << std::endl;
        if (period - timeElapsed < 1E-3) {
            //std::cout << "fabs(freq - timeElapsed): " << fabs(freq - timeElapsed) << std::endl;
            //printf("\e[?25l"); /* hide the cursor */
            putchar(' ');
            putchar(spin_chars[spinIndex % spin_chars.length()]);
            putchar(' ');
            fflush(stdout);
            putchar('\r');
            spinIndex++;
            last = now;
        }
    }

private:
    struct timespec last, now;
    int freq;
    double period;
    unsigned long spinIndex;
    std::string spin_chars;
};
}

#endif // ULISSE_CTRL_DATA_STRUCTS_HPP
