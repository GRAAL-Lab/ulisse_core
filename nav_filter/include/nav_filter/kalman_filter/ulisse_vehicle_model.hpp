#ifndef ULISSE_VEHICLE_MODEL
#define ULISSE_VEHICLE_MODEL

#include "ctrl_toolbox/kalman_filter/ModelKalmanFilter.h"
#include "surface_vehicle_model/surfacevehiclemodel.hpp"
#include <memory>

namespace ulisse {
namespace nav {

    enum ASVModelVersion {
        SimplifiedCoMFrame, CompleteBodyFrame, CompleteBodyFrameBaseline, num_versions
    };

    const std::map<ASVModelVersion, std::string> ASVModelVersion2String = {
        { SimplifiedCoMFrame, "SimplifiedCoMFrame" },
        { CompleteBodyFrame, "CompleteBodyFrame_M2" },
        { CompleteBodyFrameBaseline, "CompleteBodyFrame_BASELINE" }
    };

    class UlisseVehicleModel : public ctb::ModelKalmanFilter {

    public:
        /*
         * @brief Constructor
         */
        UlisseVehicleModel(const ASVModelVersion& v);
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
        auto ModelParameters() -> SurfaceVehicleModelParameters& { return params_; }

        double sign(double x);

    private:
        ASVModelVersion version_;
        SurfaceVehicleModelParameters params_;
    };
}
}

#endif
