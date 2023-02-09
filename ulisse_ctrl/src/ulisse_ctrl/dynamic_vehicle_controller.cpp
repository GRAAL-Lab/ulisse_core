#include "ulisse_ctrl/dynamic_vehicle_controller.hpp"

#include <jsoncpp/json/json.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "ulisse_ctrl/ulisse_defines.hpp"
#include "ulisse_ctrl/configuration.hpp"

#include "ulisse_msgs/terminal_utils.hpp"
#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_msgs/futils.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace ulisse {

DynamicVehicleController::DynamicVehicleController(std::string file_name)
    : Node("dynamic_control_node")
{

    confFileName_ = file_name;

         //Subscribers
    filterSub_ = this->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10,
        std::bind(&DynamicVehicleController::FilterDataCB, this, _1));
    vehicleStatusSub_ = this->create_subscription<ulisse_msgs::msg::VehicleStatus>(ulisse_msgs::topicnames::vehicle_status, 10,
        std::bind(&DynamicVehicleController::VehicleStatusCB, this, _1));
    referenceVelocitiesSub_ = this->create_subscription<ulisse_msgs::msg::ReferenceVelocities>(ulisse_msgs::topicnames::reference_velocities, 10,
        std::bind(&DynamicVehicleController::ReferenceVelocitiesCB, this, _1));

         //Publishers
    thrusterDataPub_ = this->create_publisher<ulisse_msgs::msg::ThrustersReference>(ulisse_msgs::topicnames::llc_thrusters_reference_perc, 1);
    thrusterMappigPub_ = this->create_publisher<ulisse_msgs::msg::ThrusterMappingControl>(ulisse_msgs::topicnames::thruster_mapping_control, 1);
    simulatedVelocitySensorPub_ = this->create_publisher<ulisse_msgs::msg::SimulatedVelocitySensor>(ulisse_msgs::topicnames::simulated_velocity_sensor, 1);
    classicPidControlPub_ = this->create_publisher<ulisse_msgs::msg::DynamicPidControl>(ulisse_msgs::topicnames::classic_pid_control, 1);
    computedTorqueControlPub_ = this->create_publisher<ulisse_msgs::msg::DynamicPidControl>(ulisse_msgs::topicnames::computed_torque_control, 1);


    dcl_conf = std::make_shared<DCLConfiguration>();

         //Ulisse params configuration
    if (!LoadDclConfiguration(dcl_conf, confFileName_)) {
        std::cerr << "Failed to laod DCL Configuration. Check the parameters in the conf file" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << tc::brown << *dcl_conf << tc::none << std::endl;

    ulisseModel.params = dcl_conf->ulisseModel;

         //Controller inizialization
    if (dcl_conf->ctrlMode == ControlMode::ThrusterMapping) {
        ThrusterMappingInizialization(dcl_conf, sampleTime_, pidSurgeTM);
    } else if (dcl_conf->ctrlMode == ControlMode::ClassicPIDControl) {
        ClassicPidControlInizialization(dcl_conf, sampleTime_, pidSurgeCP, pidYawRateCP);
    } else if (dcl_conf->ctrlMode == ControlMode::ComputedTorque){
        ComputedTorqueControlInizialization(dcl_conf, sampleTime_, pidSurgeCT, pidYawRateCT);
    } else if (dcl_conf->ctrlMode == ControlMode::STSM) {
        STSMControlInizialization(dcl_conf, sampleTime_);// pidSurgeCP, pidYawRateCP);
    }
    srvResetConf_ = this->create_service<ulisse_msgs::srv::ResetConfiguration>(ulisse_msgs::topicnames::reset_dcl_conf_service,
        std::bind(&DynamicVehicleController::ResetConfHandler, this, _1, _2, _3));


         // Main function timer
    int msRunPeriod = 1.0/(dcl_conf->controlLoopRate) * 1000;
    //std::cout << "Controller Rate: " << rate << "Hz" << std::endl;
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&DynamicVehicleController::Run, this));
}

DynamicVehicleController::~DynamicVehicleController() {

}

void DynamicVehicleController::ResetConfHandler(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Request> request,
    std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Response> response){

    (void)request_header;
    (void)request;
    RCLCPP_INFO(this->get_logger(), "Incoming request for reset conf");

    auto previousConf = dcl_conf;
    //Ulisse params configuration
    if (!LoadDclConfiguration(dcl_conf, confFileName_)) {

             //If changing the file at runtime the configuration should fail, reload the previous configuration parameters
        std::cerr << "Failed to laod new config file. Reload the last configuration" << std::endl;
        LoadDclConfiguration(previousConf, confFileName_);
    }

    ulisseModel.params = dcl_conf->ulisseModel;

         //Controller inizialization
    if (dcl_conf->ctrlMode == ControlMode::ThrusterMapping) {
        ThrusterMappingInizialization(dcl_conf, sampleTime_, pidSurgeTM);
    } else if (dcl_conf->ctrlMode == ControlMode::ClassicPIDControl) {
        ClassicPidControlInizialization(dcl_conf, sampleTime_, pidSurgeCP, pidYawRateCP);
    } else if (dcl_conf->ctrlMode == ControlMode::ComputedTorque){
        ComputedTorqueControlInizialization(dcl_conf, sampleTime_, pidSurgeCT, pidYawRateCT);
    } else if (dcl_conf->ctrlMode == ControlMode::STSM){
        STSMControlInizialization(dcl_conf, sampleTime_);
    }

    response->res = "[DCL] ReloadConfiguration::ok";
}

void DynamicVehicleController::Run()
{

         // Evaluating the total surge, taking into account the water current
    Eigen::Rotation2D<double> wRb = Eigen::Rotation2D<double>(filterData.bodyframe_angular_position.yaw);
    Eigen::Vector2d water_current_b = wRb.inverse() * Eigen::Vector2d(filterData.inertialframe_water_current.data());

    double absSurgeFbk = filterData.bodyframe_linear_velocity[0] + water_current_b[0];   // ?! è la velocità relativa all'acqua?
    double relSurgeFbk = filterData.bodyframe_linear_velocity[0];
    double yawRateFbk = filterData.bodyframe_angular_velocity[2];

    if (vehicleStatus.vehicle_state != ulisse::states::ID::halt) {
        //ThrusterMapping mode
        if (dcl_conf->ctrlMode == ControlMode::ThrusterMapping) {

            Eigen::Vector6d requestedVel;
            requestedVel.setZero();

            requestedVel(0) = pidSurgeTM.Compute(referenceVelocities.desired_surge, absSurgeFbk);
            requestedVel(5) = referenceVelocities.desired_yaw_rate;

            Eigen::Vector3d tauDrag = ulisseModel.ComputeCoriolisAndDragForces(requestedVel);
            tau = Eigen::Vector2d(tauDrag[0], tauDrag[2]);
            Eigen::Vector2d forces = ulisseModel.ThusterAllocation(tau);

                 //saturation
            requestedVel(0) = ctb::clamp(requestedVel(0), dcl_conf->surgeMin, dcl_conf->surgeMax);
            requestedVel(5) = ctb::clamp(requestedVel(5), dcl_conf->yawRateMin, dcl_conf->yawRateMax);

            ulisseModel.InverseMotorsEquations(requestedVel, forces, motorLeft, motorRight);

            ulisseModel.ThrustersSaturation(motorLeft, motorRight, -dcl_conf->thrusterPercLimit, dcl_conf->thrusterPercLimit, thrustersReference.left_percentage, thrustersReference.right_percentage);

                 //Fill the Thruster Mapping msg
            auto t_now_ = std::chrono::system_clock::now();
            long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
            thrusterMappingMsg.stamp.sec = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
            thrusterMappingMsg.stamp.nanosec = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

            thrusterMappingMsg.desired_surge = referenceVelocities.desired_surge;
            thrusterMappingMsg.feedback_surge = absSurgeFbk;
            thrusterMappingMsg.out_pid_surge = pidSurgeTM.GetOutput();
            thrusterMappingMsg.desired_yaw_rate = referenceVelocities.desired_yaw_rate;
            thrusterMappingMsg.feedback_yaw_rate = yawRateFbk;
            thrusterMappingMsg.motor_percentage.left_percentage = motorLeft;
            thrusterMappingMsg.motor_percentage.right_percentage = motorRight;

            thrusterMappigPub_->publish(thrusterMappingMsg);

                 //fill the feedback for the nav filter
            simulatedVelocitySensor.water_relative_surge = pidSurgeTM.GetOutput();
            simulatedVelocitySensorPub_->publish(simulatedVelocitySensor);

        } else if (dcl_conf->ctrlMode == ControlMode::ClassicPIDControl) {
            //Dynamic Pids
            Eigen::Vector6d feedbackVel = Eigen::Vector6d::Zero();

            tau = { pidSurgeCP.Compute(referenceVelocities.desired_surge, absSurgeFbk), pidYawRateCP.Compute(referenceVelocities.desired_yaw_rate, yawRateFbk) };

            feedbackVel(0) = absSurgeFbk;
            feedbackVel(5) = yawRateFbk;
            double outleft, outright;

            Eigen::Vector2d forces = ulisseModel.ThusterAllocation(tau);
            ulisseModel.InverseMotorsEquations(feedbackVel, forces, outleft, outright);
            ulisseModel.ThrustersSaturation(outleft, outright, -dcl_conf->thrusterPercLimit, dcl_conf->thrusterPercLimit, thrustersReference.left_percentage, thrustersReference.right_percentage);

                 //Fill the classic dynamic pid contol msg
            auto t_now_ = std::chrono::system_clock::now();
            long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
            classicPidControlMsg.stamp.sec = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
            classicPidControlMsg.stamp.nanosec = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
            classicPidControlMsg.desired_surge = referenceVelocities.desired_surge;
            classicPidControlMsg.feedback_surge = absSurgeFbk;
            classicPidControlMsg.out_pid_surge = pidSurgeCP.GetOutput();
            classicPidControlMsg.desired_yaw_rate = referenceVelocities.desired_yaw_rate;
            classicPidControlMsg.feedback_yaw_rate = yawRateFbk;
            classicPidControlMsg.out_pid_yaw_rate = pidYawRateCP.GetOutput();
            classicPidControlMsg.forces = { forces[0], forces[1] };
            classicPidControlMsg.tau = { tau[0], tau[1] };
            classicPidControlMsg.motor_percentage.left_percentage = outleft;
            classicPidControlMsg.motor_percentage.right_percentage = outright;

            classicPidControlPub_->publish(classicPidControlMsg);

                 //fill the feedback for the nav filter    <----------- CHECK TODO
            simulatedVelocitySensor.water_relative_surge = referenceVelocities.desired_surge;
            simulatedVelocitySensorPub_->publish(simulatedVelocitySensor);

        } else if (dcl_conf->ctrlMode == ControlMode::ComputedTorque) {


            tau = { pidSurgeCT.Compute(referenceVelocities.desired_surge, absSurgeFbk), pidYawRateCT.Compute(referenceVelocities.desired_yaw_rate, yawRateFbk) };

                 // using relative surge velocity for the feedforward term
            Eigen::Vector6d feedbackVel = Eigen::Vector6d::Zero();
            feedbackVel(0) = relSurgeFbk;
            feedbackVel(5) = yawRateFbk;

            Eigen::Vector3d tauDrag = ulisseModel.ComputeCoriolisAndDragForces(feedbackVel);
            
            //std::cerr << "tau PID:  F = " << tau[0] << " | N = " << tau[1] << std::endl;
            //std::cerr << "tau CT :  F = " << tauDrag[0] << " | N = " << tauDrag[2] << std::endl;

            tau += Eigen::Vector2d(tauDrag[0], tauDrag[2]);
            double outLeft, outRight; /*riferimenti per i thrusters*/
            // utilizzare queste 3 funzioni per passare dalla tau alle percentuali dei motori
            Eigen::Vector2d forces = ulisseModel.ThusterAllocation(tau);
            ulisseModel.InverseMotorsEquations(feedbackVel, forces, outLeft, outRight);
            ulisseModel.ThrustersSaturation(outLeft, outRight, -dcl_conf->thrusterPercLimit, dcl_conf->thrusterPercLimit, thrustersReference.left_percentage, thrustersReference.right_percentage);


                 //Fill the classic dynamic pid contol msg
            auto t_now_ = std::chrono::system_clock::now();
            long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
            computedTorqueMsg.stamp.sec = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
            computedTorqueMsg.stamp.nanosec = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
            computedTorqueMsg.desired_surge = referenceVelocities.desired_surge;
            computedTorqueMsg.feedback_surge = absSurgeFbk;
            computedTorqueMsg.out_pid_surge = pidSurgeCP.GetOutput();
            computedTorqueMsg.desired_yaw_rate = referenceVelocities.desired_yaw_rate; //velocità desiderate
            computedTorqueMsg.feedback_yaw_rate = yawRateFbk;
            computedTorqueMsg.out_pid_yaw_rate = pidYawRateCP.GetOutput();
            computedTorqueMsg.forces = { forces[0], forces[1] };
            computedTorqueMsg.tau = { tau[0], tau[1] };
            computedTorqueMsg.motor_percentage.left_percentage = outLeft;
            computedTorqueMsg.motor_percentage.right_percentage = outRight;
            computedTorqueControlPub_->publish(computedTorqueMsg);

                 //fill the feedback for the nav filter
            simulatedVelocitySensor.water_relative_surge = referenceVelocities.desired_surge;//velocità desiderate
            simulatedVelocitySensorPub_->publish(simulatedVelocitySensor);
        } else if (dcl_conf->ctrlMode == ControlMode::STSM){

            double Ts = sampleTime_;
            double alfa_1 = 5;
            double alfa_2 = 180;

            /// C e D sono le matrici del paper di Coriolis e Drag (correggere quella presa come diagonal dal conf)
            /// v_r velocità istantea del veicolo (vel relativa) feedback   (---> filterData.bodyframe_linear_velocity)
            /// sigma (sliding variable è l'errore di velocità)

            //Disturbance observer signal

            Eigen::Map<Eigen::Vector3d> v_r(filterData.bodyframe_linear_velocity.data(), 3);
            Eigen::Vector3d d_hat = compute_d_hat(z_stsm, L, ulisseModel.params.Inertia, v_r); //inizializzare z = 0; segnale osservatore



            //tau_controllo

            Eigen::Vector3d tau_eq = compute_tau_eq(C, D, v_r, d_hat);

            Eigen::Vector3d tau_stsm_1 = compute_tau_stsm_1(alfa_1, sigma_stsm);
            tau_stsm_2 = compute_tau_stsm_2(alfa_2, sigma_stsm, tau_stsm_2, Ts); //inizializzare tau_stsm_2 = 0*

            Eigen::Vector3d tau_controllo = tau_eq + tau_stsm_1 + tau_stsm_2;

            //Auxiliary variable z

            //Eigen::Vector3d z, Eigen::Matrix3d L, Eigen::Matrix3d C, Eigen::Matrix3d D, Eigen::Matrix3d M, Eigen::Vector3d v_r, Eigen::Vector3d tau_controllo

            // L Observer gain

            z_stsm = compute_z(z_stsm, L, C, D, ulisseModel.params.Inertia, v_r, tau_controllo);

            Eigen::Vector6d vehicleVel;
            vehicleVel.LinearVector(Eigen::Map<Eigen::Vector3d>(filterData.bodyframe_linear_velocity.data(), 3));
            vehicleVel.AngularVector(Eigen::Map<Eigen::Vector3d>(filterData.bodyframe_angular_velocity.data(), 3));
            ulisseModel.ComputeCoriolisAndDragForces(vehicleVel);
            //std::cout<<"K1"<<std::endl<<K1; //stampo la matrice K1 per verificare che sia tutto ok

            //Eigen::Matrix3d M; // Matrice d'inerzia (considera anche effetti di massa aggiunta)
            //M << ulisseModel.params.m11, 0, 0,
            //    0, ulisseModel.params.m22, ulisseModel.params.m23,
            //    0, ulisseModel.params.m32, ulisseModel.params.m33;

            //Eigen::Matrix3d D; //Matrice Drag (dove prendo gli elementi?)


            //Eigen::Matrix3d Co; //Matrice Coriolis (dove prendo gli elementi?)

            //error(0) = referenceVelocities.desired_surge;
            //error(2) = referenceVelocities.desired_yaw_rate;

            // using relative surge velocity for the feedforward term
            Eigen::Vector6d feedbackVel = Eigen::Vector6d::Zero();
            feedbackVel(0) = relSurgeFbk;
            feedbackVel(5) = yawRateFbk;
            double outLeft, outRight; 

                 tau = tau_controllo;

                 //Utilizzare queste 3 funzioni per passare dalla tau alle percentuali dei motori
            Eigen::Vector2d forces = ulisseModel.ThusterAllocation(tau);
            ulisseModel.InverseMotorsEquations(feedbackVel, forces, outLeft, outRight);
            ulisseModel.ThrustersSaturation(outLeft, outRight, -dcl_conf->thrusterPercLimit, dcl_conf->thrusterPercLimit,thrustersReference.left_percentage, thrustersReference.right_percentage);
        }
    } else {
        thrustersReference.left_percentage = 0.0;
        thrustersReference.right_percentage = 0.0;

        if (dcl_conf->ctrlMode == ControlMode::ThrusterMapping) {
            pidSurgeTM.Reset();
        } else if (dcl_conf->ctrlMode == ControlMode::ClassicPIDControl) {
            pidSurgeCP.Reset();
            pidYawRateCP.Reset();
        } else {
            pidSurgeCT.Reset();
            pidYawRateCT.Reset();
        }
    }

    PublishControl();

}

void DynamicVehicleController::PublishControl()
{
    auto tNow = std::chrono::system_clock::now();
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow.time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
    thrustersReference.stamp.sec = now_stamp_secs;
    thrustersReference.stamp.nanosec = now_stamp_nanosecs;
    thrusterDataPub_->publish(thrustersReference);
}

bool DynamicVehicleController::LoadDclConfiguration(std::shared_ptr<DCLConfiguration> dcl_conf, std::string filename)
{
    libconfig::Config confObj;

         // Read the ULISSE_CTRL config file
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("ulisse_ctrl");
    std::string confPath = package_share_directory + "/conf/" + filename;
    std::cout << "PATH TO ULISSE_CTRL CONF FILE (DCL): " << confPath << std::endl;

    try {
        confObj.readFile(confPath.c_str());
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return false;
    }
    if (!dcl_conf->LoadConfiguration(confObj))
        return false;

         // Read the ULISSE_MODEL config file
    package_share_directory = ament_index_cpp::get_package_share_directory("surface_vehicle_model");
    confPath = package_share_directory + "/conf/ulisse_model.conf";
    std::cout << "PATH TO ULISSE_MODEL CONF FILE (DCL): " << confPath << std::endl;

    try {
        confObj.readFile(confPath.c_str());
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return false;
    }

    if (!dcl_conf->ConfigureUlisseModel(confObj))
        return false;

    return true;
}

void DynamicVehicleController::ThrusterMappingInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pid)
{
    pid.Initialize(conf->thrusterMapping.pidGainsSurge, sampleTime, conf->thrusterMapping.pidSatSurge);
}

void DynamicVehicleController::ClassicPidControlInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pidSurge, ctb::DigitalPID& pidYawRate)
{
    pidSurge.Initialize(conf->classicPidControl.pidGainsSurge, sampleTime, conf->classicPidControl.pidSatSurge);
    pidYawRate.Initialize(conf->classicPidControl.pidGainsYawRate, sampleTime, conf->classicPidControl.pidSatYawRate);
}

void DynamicVehicleController::ComputedTorqueControlInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pidSurge, ctb::DigitalPID& pidYawRate)
{
    pidSurge.Initialize(conf->computedTorqueControl.pidGainsSurge, sampleTime, conf->computedTorqueControl.pidSatSurge);
    pidYawRate.Initialize(conf->computedTorqueControl.pidGainsYawRate, sampleTime, conf->computedTorqueControl.pidSatYawRate);
}
/*STSM Inizialization <-- TO DO*/
void DynamicVehicleController::STSMControlInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime)
{
    sigma_stsm = Eigen::Vector3d::Zero();
    z_stsm = Eigen::Vector3d::Zero();
    tau_stsm_2 = Eigen::Vector3d::Zero();
    //m11 = m - Xup;
    //m22 = m - Yvp;
    //m23 = -Yrp
    //m32 = -Nvp
    //m33 = Iz - Nrp;
    //L.diagonal() = dcl_conf->stsmControl.L;
    //C.diagonal() = dcl_conf->stsmControl.C;

}

Eigen::Vector3d DynamicVehicleController::compute_z(Eigen::Vector3d z, Eigen::Matrix3d L, Eigen::Matrix3d C, Eigen::Matrix3d D, Eigen::Matrix3d M, Eigen::Vector3d v_r, Eigen::Vector3d tau_controllo){
    return z + L*(C*v_r + D*v_r - tau_controllo - z - L*M*v_r); //z inizializzato a vettore nullo
}

Eigen::Vector3d DynamicVehicleController::compute_d_hat(Eigen::Vector3d z, Eigen::Matrix3d L, Eigen::Matrix3d M, Eigen::Vector3d v_r){
    return z + sampleTime_*L*M*v_r; // z = compute_z();
}

// Di seguito definisco le funzioni per calcolare tau_controllo = tau_eq + tau_stsm_1 + tau_stsm_2;

Eigen::Vector3d DynamicVehicleController::compute_tau_eq(Eigen::Matrix3d C, Eigen::Matrix3d D, Eigen::Vector3d v_r, Eigen::Vector3d d_hat){
    return C*v_r + D*v_r -d_hat; //d_hat = compute_d_hat();

}

Eigen::Vector3d DynamicVehicleController::compute_tau_stsm_1(double alfa_1, Eigen::Vector3d sigma){
    for (int i = 0; i < 3; ++i) {
        sigma(i) = -(alfa_1*(pow(abs(sigma(i)),0.5))*(futils::sign(sigma(i))));
    }

    return sigma; // sigma è la sliding variable, definita come errore tra le velocità: sigma = v_r - v_desiderata
}

Eigen::Vector3d DynamicVehicleController::compute_tau_stsm_2(double alfa_2, Eigen::Vector3d sigma, Eigen::Vector3d tau_stsm_2, double Ts){

    for (int i = 0; i < 3; ++i) {
        sigma(i) = tau_stsm_2(i) - (alfa_2*Ts*futils::sign(sigma(i)));
    }
    return sigma; //tau_stsm_2 inizializzato a vettore nullo
}

void DynamicVehicleController::ReferenceVelocitiesCB(const ulisse_msgs::msg::ReferenceVelocities::SharedPtr msg) { referenceVelocities = *msg; }

void DynamicVehicleController::VehicleStatusCB(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg) { vehicleStatus = *msg; }

void DynamicVehicleController::FilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg) { filterData = *msg; }

} // namespace ulisse
