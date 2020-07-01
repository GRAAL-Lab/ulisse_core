#include <cmath>
#include <rml/RML.h>

#include "nav_filter/luenberger_observer/pos_vel_observer.hpp"

namespace ulisse {

namespace nav {

    PosVelObserver::PosVelObserver()
    {
        k = Eigen::Vector4d{ 2, 2, 0.2, 0.2 };

        estimateState_ = Eigen::VectorXd::Zero(8);
        prevTime_ = duration_cast<nanoseconds>(system_clock::now().time_since_epoch());
        initialized_ = false;
    }

    void PosVelObserver::Update(Eigen::VectorXd measurements)
    {
        nanoseconds now = duration_cast<nanoseconds>(system_clock::now().time_since_epoch());
        if (!initialized_) {
            estimateState_.segment(0, 2) = measurements.segment(0, 2);

            initialized_ = true;
            prevTime_ = now;
        }

        estimateState_.segment(4, 2) = measurements[3] * Eigen::Vector2d{ cos(measurements[2]), sin(measurements[2]) }.asDiagonal() * estimateState_.segment(2, 2) + k.segment(0, 2).asDiagonal() * (measurements.segment(0, 2) - estimateState_.segment(0, 2));
        estimateState_.segment(6, 2) = k.segment(2, 2).asDiagonal() * (measurements.segment(0, 2) - estimateState_.segment(0, 2));

        double dt = (now - prevTime_).count() / 1E9;

        std::cout << "delta T:" << dt << std::endl;

        estimateState_.segment(0, 4) += dt * Eigen::Matrix4d::Identity() * estimateState_.segment(4, 4);

        prevTime_ = now;
    }

    void PosVelObserver::Reset()
    {
        initialized_ = false;
        estimateState_.setZero();
    }
}
}
