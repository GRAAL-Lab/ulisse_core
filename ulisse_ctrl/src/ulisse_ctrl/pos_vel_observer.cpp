#include <cmath>
#include <rml/RML.h>

#include "ulisse_ctrl/pos_vel_observer.hpp"

namespace ulisse {

namespace nav {

void PosVelObserver::Update(double v, double theta, double x, double y) {
    nanoseconds now = duration_cast<nanoseconds>(system_clock::now().time_since_epoch());
	if (!initialized_) {
		xhat_ = x;
		yhat_ = y;
		Cxhat_ = 0;
		Cyhat_ = 0;
		initialized_ = true;
		prevTime_ = now;
	}
    xhatdot_ = v * cos(theta) + Cxhat_ + k_[0] * (x - xhat_);
    yhatdot_ = v * sin(theta) + Cyhat_ + k_[1] * (y - yhat_);
	Cxhatdot_ = k_[2] * (x - xhat_);
	Cyhatdot_ = k_[3] * (y - yhat_);


    double dt = (now - prevTime_).count() / 1E9;

	xhat_ +=  xhatdot_*dt;
	yhat_ +=  yhatdot_*dt;
	Cxhat_ +=  Cxhatdot_*dt;
	Cyhat_ +=  Cyhatdot_*dt;

	prevTime_ = now;

	/*ortos::DebugConsole::Write(ortos::LogLevel::info, "PosVelObserver", "xhat = %lf yhat = %lf xhatdot = %lf yhatdot = %lf",
			xhat_, yhat_, xhatdot_, yhatdot_);
	ortos::DebugConsole::Write(ortos::LogLevel::info, "PosVelObserver", "Cxhat = %lf Cyhat = %lf Cxhatdot = %lf Cyhatdot = %lf",
				Cxhat_, Cyhat_, Cxhatdot_, Cyhatdot_);*/
}

}
}
