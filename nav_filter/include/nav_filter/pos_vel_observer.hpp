#ifndef SRC_NAV_POSVELOBSERVER_H_
#define SRC_NAV_POSVELOBSERVER_H_

#include <chrono>
#include "nav_filter/nav_data_structs.hpp"

using namespace std::chrono;

namespace ulisse {

namespace nav {

class PosVelObserver {
public:
	PosVelObserver() {
		for (int i = 0; i < 4; i++) {
			k_[i] = 0.2;
		}
		k_[0] = 5;
		k_[1] = 5;
		xhat_ = 0;
		yhat_ = 0;
		xhatdot_ = 0;
		yhatdot_ = 0;
		Cxhat_ = 0;
		Cyhat_ = 0;
		Cxhatdot_ = 0;
		Cyhatdot_ = 0;
        prevTime_ = duration_cast<nanoseconds>(system_clock::now().time_since_epoch());
		initialized_ = false;
	}
    void SetConfig(const NavFilterConfigData& config) {
		for (int i = 0; i < 4; i++) {
            k_[i] = config.k[i];
		}
	}

    void Update(double v, double theta, double x, double y);
    void GetCurrent(double& cx, double& cy) {
		cx = Cxhat_;
		cy = Cyhat_;
	}
    void GetPosition(double& x, double& y) {
		x = xhat_;
		y = yhat_;
	}
    void GetSpeed(double& xdot, double& ydot) {
		xdot = xhatdot_;
		ydot = yhatdot_;
	}
	void Reset() {
		initialized_ = false;
		xhatdot_ = 0;
		yhatdot_ = 0;
		Cxhat_ = 0;
		Cyhat_ = 0;
	}

private:
    double Cxhat_, Cyhat_, Cxhatdot_, Cyhatdot_;
    double xhat_, yhat_, xhatdot_, yhatdot_;
    double k_[4];
    std::chrono::nanoseconds prevTime_;
	bool initialized_;
};

}

}

#endif /* SRC_NAV_POSVELOBSERVER_H_ */
