#ifndef SRC_NAV_POSVELOBSERVER_H_
#define SRC_NAV_POSVELOBSERVER_H_

#include "nav_filter/nav_data_structs.hpp"
#include <chrono>

using namespace std::chrono;

namespace ulisse {

namespace nav {

    class PosVelObserver {
    public:
        PosVelObserver();

        void Update(Eigen::VectorXd measurements); //[x y theta x_dot]

        auto LinearVelocity() const -> Eigen::Vector2d { return estimateState_.segment(4, 2); }
        auto LinearPosition() const -> Eigen::Vector2d { return estimateState_.segment(0, 2); }
        auto WaterCurrent() const -> Eigen::Vector2d { return estimateState_.segment(6, 2); }

        void Reset();

        Eigen::VectorXd k;

    private:
        Eigen::VectorXd estimateState_; // [x y cx cy x_dot y_dot cx_dot cy_dot]
        std::chrono::nanoseconds prevTime_;
        bool initialized_;
    };
}
}

#endif /* SRC_NAV_POSVELOBSERVER_H_ */
