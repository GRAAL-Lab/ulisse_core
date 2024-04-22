#ifndef ULISSE_VEHICLE_MODEL
#define ULISSE_VEHICLE_MODEL

#include "ctrl_toolbox/kalman_filter/ModelKalmanFilter.h"
#include "surface_vehicle_model/surfacevehiclemodel.hpp"
#include <memory>

namespace ulisse {
namespace nav {

    class UlisseVehicleModel : public ctb::ModelKalmanFilter {

    public:
        enum class Version {
            SimplifiedCoMFrame, CompleteBodyFrame
        };
        /*
         * @brief Constructor
         */
        UlisseVehicleModel(const Version& v);
        /*
         * @brief Destructor
         */
        ~UlisseVehicleModel() override;
        /*
         * @brief Method for computing the state transition jacobian which is the derivative of the state model w.r.t the state
         * @param [in] state - the state vector
         * @param [in] inpunt - the input vector
         * @return The state transition jacobian
         */
        Eigen::MatrixXd ComputeJacobian(const Eigen::VectorXd& state, const Eigen::VectorXd& input) override;
        /*
         * @brief Method for computing the state transition in function of the state vector
         * @param [in] state - the state vector
         * @param [in] inpunt - the input vector
         * @return The state transition model
         */
        Eigen::VectorXd ComputeStateTransitionModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input) override;
        /*
         * @brief Method for setting the model parameters
         */
        //auto ModelParameters() -> UlisseModelParameters& { return params_; }
        auto ModelParameters() -> SurfaceVehicleModelParameters& { return params_; }

        double sign(double x);

    private:
        Version version_;
        //UlisseModelParameters params_;
        SurfaceVehicleModelParameters params_;
    };
}
}

#endif
