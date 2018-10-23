#include "ulisse_sim/vehiclesimulator.h"

VehicleSimulator::VehicleSimulator()
    : geod_(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f())
{
    vehPos_now_.setZero();
    vehAtt_now_.SetRPY(0.0, 0.0, 0.0);

    vehPos_last_ = vehPos_now_;
    vehAtt_last_ = vehAtt_now_;

    start_ = last_ = now_ = std::chrono::system_clock::now();
}

Eigen::Vector3d VehicleSimulator::VehPos() const
{
    return vehPos_now_;
}

rml::EulerRPY VehicleSimulator::VehAtt() const
{
    return vehAtt_now_;
}

Eigen::Vector6d VehicleSimulator::VehVel_world() const
{
    return vehVel_world;
}

void VehicleSimulator::SetParameters(double Ts, const ThrusterMappingParameters& thmapparams)
{
    Ts_ = Ts;
    ulisseModel_.SetMappingParams(thmapparams);
}

void VehicleSimulator::SimulateActuation(double h_s, double h_p)
{
    // Computing vehicle acceleration
    ulisseModel_.DirectDynamics(h_s, h_p, vehRelVel_body_, vehRelAcc_body_);

    /*std::cout << "vehRelVel_body: " << vehRelVel_body_.transpose() << std::endl;
    std::cout << "vehRelAcc_body: " << vehRelAcc_body_.transpose() << std::endl;*/

    // Integrating the acceleration to get the vehicle velocity
    vehRelVel_body_ = vehRelVel_body_ + vehRelAcc_body_ * Ts_;

    // Projecting the acceleration and velocity on the world frame
    vehRelAcc_world_ = vehAtt_now_.ToRotMatrix().GetCartesianRotationMatrix() * vehRelAcc_body_;
    vehRelVel_world_ = vehAtt_now_.ToRotMatrix().GetCartesianRotationMatrix() * vehRelVel_body_;

    // Get the vehicle absolute velocity by adding the water current velocity
    vehVel_world = vehRelVel_world_ + waterVel_world_;

    // Passing from angular vehicle acceleration to Euler rates
    Eigen::Matrix3d S;
    S << cos(vehAtt_now_.GetYaw()) * cos(vehAtt_now_.GetPitch()), -sin(vehAtt_now_.GetYaw()), 0,
        sin(vehAtt_now_.GetYaw()) * cos(vehAtt_now_.GetPitch()), cos(vehAtt_now_.GetYaw()), 0,
        -sin(vehAtt_now_.GetPitch()), 0, 1;
    rml::RegularizationData mySvd;
    Eigen::Vector3d rpyEulerRates = rml::RegularizedPseudoInverse(S, mySvd) * vehVel_world.GetSecondVect3();

    // Integrating the linear velocity to get the new position
    vehPos_now_ = vehPos_last_ + vehVel_world.GetFirstVect3() * Ts_;

    // Integrating the Euler rates to get the new Euler angles
    vehAtt_now_.SetRoll(vehAtt_last_.GetRoll() + rpyEulerRates(0) * Ts_);
    vehAtt_now_.SetPitch(vehAtt_last_.GetPitch() + rpyEulerRates(1) * Ts_);
    vehAtt_now_.SetYaw(vehAtt_last_.GetYaw() + rpyEulerRates(2) * Ts_);
}

void VehicleSimulator::SimulateSensors()
{

    now_ = std::chrono::system_clock::now();
    now_time_t_ = std::chrono::system_clock::to_time_t(now_);

    elapsed_ = std::chrono::duration_cast<std::chrono::microseconds>(now_ - last_);
    total_ = std::chrono::duration_cast<std::chrono::microseconds>(now_ - start_);

    std::cout << "Time difference (sec) = " << (total_.count() / 1E6) << std::endl;
    //std::cout << "Time now: " << now_time_t_ << "\r" << std::flush;

    direction_ = vehPos_now_ - vehPos_last_;

//gpsdata_msg_.timestamp = now_time_t_; // = now
    gpsdata_msg_.track = std::atan2(direction_(1), direction_(0));

    //geod.Direct(lat_, long_, azimuth, distance, lat_, long_);




    /*
    gpsdata_.d.latitude = state_.position.latitude;
    gpsdata_.d.longitude = state_.position.longitude;
    gpsdata_.d.speed = state_.speed;
    gpsdata_.d.track = state_.track;
    gpsdata_.d.mode = om2ctrl::gpsd::GpsFixMode::mode_3d;


    eessensors_.d.compassHeading = (state_.heading) * PI / 180.0 ;

    eessensors_.d.accelerometer[0] = state_.acceleration;
    eessensors_.d.accelerometer[2] = 9.81;

    eessensors_.d.gyro[2] = state_.omegaz;

    eessensors_.d.temperatureCtrlBox = 23.1 + (rand()/(double)RAND_MAX) * 2 ;

    eessensors_.d.leftReference = state_.leftThruster;
    eessensors_.d.rightReference = state_.rightThruster;

    */
}

void VehicleSimulator::ExecuteStep(double h_s, double h_p)
{

    SimulateActuation(h_s, h_p);
    SimulateSensors();

    last_ = now_;
    vehPos_last_ = vehPos_now_;
    vehAtt_last_ = vehAtt_now_;

    //state_.track = atan2(y3, x3)*180.0/PI; //c.o.g.
    //state_.speed = sqrt(pow(x3,2) + pow(y3,2)); //
    //ArrivalPoint(state_.position, distance, state_.track, state_.position);
}
