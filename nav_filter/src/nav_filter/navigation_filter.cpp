#include "nav_filter/navigation_filter.hpp"

using std::placeholders::_1;

namespace ulisse {

namespace nav {
    NavigationFilter::NavigationFilter(const rclcpp::Node::SharedPtr& nh)
        : nh_(nh)
    {

    }

    NavigationFilter::~NavigationFilter(){ }




    void NavigationFilter::SensorsCheckCB(){
        std::cout << "Hello, world!" << std::endl;
    }

    bool NavigationFilter::LoadConfiguration(NavigationFilterParams& filterParameters)
    {
        //read conf file
        libconfig::Config confObj;

        //Inizialization
        std::string confPath = ament_index_cpp::get_package_share_directory("nav_filter").append("/conf/navigation_filter.conf");

        std::cout << "PATH TO CONF FILE : " << confPath << std::endl;

        try {
            confObj.readFile(confPath.c_str());
        } catch (const libconfig::FileIOException& fioex) {
            std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
            return false;
        } catch (libconfig::ParseException& e) {
            std::cerr << "Parse exception when reading:" << confPath << std::endl;
            std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
            return false;
        }

        //Configure the filter node params
        if (!filterParameters.ConfigureFromFile(confObj)) {
            std::cerr << "Failed to load navigation mode/rate prams" << std::endl;
            return false;
        };

        if (filterParameters.mode == FilterMode::LuenbergerObserver) {
            if (!LuenbergerObserverConfiguration(confObj)) {
                std::cerr << "Failed to load Luenberger Observer configuration" << std::endl;
                return false;
            };

        } else if (filterParameters.mode == FilterMode::KalmanFilter) {
            if (!KalmanFilterConfiguration(confObj)) {
                std::cerr << "Failed to load Kalman Filter configuration" << std::endl;
                return false;
            }
        } else if (filterParameters.mode == FilterMode::GroundTruth) {
        } else {
            std::cerr << "Type of filter not recognized" << std::endl;
            return false;
        }

        return true;
    }

    bool NavigationFilter::KalmanFilterConfiguration(libconfig::Config& confObj) noexcept(false)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& ekf = root["extendedKalmanFilter"];

        //Load the ulisse params
        const libconfig::Setting& ulisseModel = ekf["ulisseModel"];
        UlisseModelParameters ulisseModelParams;

        if (!ulisseModelParams.ConfigureFormFile(ulisseModel)) {
            std::cerr << "Kalman Filter: Failed to load ulisse model" << std::endl;
            return false;
        }

        std::cout << ulisseModelParams << std::endl;

        ulisseModelEKF->ModelParameters() = ulisseModelParams;

        //Load the measures covariance
        const libconfig::Setting& measure = ekf["measures"];

        Eigen::VectorXd covarianceDiag;
        bool isActive;
        Eigen::Vector3d bodyF_gps_position;

        const libconfig::Setting& gps = measure["gps"];
        if (!ctb::GetParam(gps, isActive, "enable"))
            return false;
        measuresActive.insert(std::make_pair("gps", isActive));
        if (!ctb::GetParamVector(gps, covarianceDiag, "covariance"))
            return false;
        gpsMeasurement->Covariance().diagonal() = covarianceDiag;

        if (!ctb::GetParamVector(gps, bodyF_gps_position, "bodyF_gps_position"))
            return false;
        gpsMeasurement->bodyF_gps_position_ = bodyF_gps_position;

        const libconfig::Setting& compass = measure["compass"];
        if (!ctb::GetParam(compass, isActive, "enable"))
            return false;

        measuresActive.insert(std::make_pair("compass", isActive));
        if (!ctb::GetParamVector(compass, covarianceDiag, "covariance"))
            return false;

        compassMeasurement->Covariance().diagonal() = covarianceDiag;

        const libconfig::Setting& gyro = measure["gyro"];
        if (!ctb::GetParam(gyro, isActive, "enable"))
            return false;

        measuresActive.insert(std::make_pair("gyro", isActive));
        if (!ctb::GetParamVector(gyro, covarianceDiag, "covariance"))
            return false;

        gyroMeasurement->Covariance().diagonal() = covarianceDiag;

        const libconfig::Setting& accelerometer = measure["accelerometer"];
        if (!ctb::GetParam(accelerometer, isActive, "enable"))
            return false;

        measuresActive.insert(std::make_pair("accelerometer", isActive));
        if (!ctb::GetParamVector(accelerometer, covarianceDiag, "covariance"))
            return false;

        accelerometerMeasurement->Covariance().diagonal() = covarianceDiag;

        const libconfig::Setting& magnetometer = measure["magnetometer"];
        if (!ctb::GetParam(magnetometer, isActive, "enable"))
            return false;

        measuresActive.insert(std::make_pair("magnetometer", isActive));
        if (!ctb::GetParamVector(magnetometer, covarianceDiag, "covariance"))
            return false;

        magnetometerMeasurement->Covariance().diagonal() = covarianceDiag;

        const libconfig::Setting& zMeterSetting = measure["z_meter"];
        if (!ctb::GetParam(zMeterSetting, isActive, "enable"))
            return false;

        measuresActive.insert(std::make_pair("zMeter", isActive));
        if (!ctb::GetParamVector(zMeterSetting, covarianceDiag, "covariance"))
            return false;

        zMeterMeasurement->Covariance().diagonal() = covarianceDiag;

        //Load the initial state and covariance and the model covariance

        //state dimention
        const libconfig::Setting& state = ekf["state"];

        if (!ctb::GetParam(state, stateDim, "dim"))
            return false;

        if (!ctb::GetParamVector(state, covarianceDiag, "modelCovariance"))
            return false;
        ulisseModelEKF->Covariance().diagonal() = covarianceDiag;

        Eigen::VectorXd initialState;
        if (!ctb::GetParamVector(state, initialState, "initialization"))
            return false;

        Eigen::Vector3d cartesian_p;
        ctb::LatLong2LocalNED(ctb::LatLong(initialState[0], initialState[1]), initialState[2], centroidLocation, cartesian_p);
        initialState.segment(0, 2) = cartesian_p.segment(0, 2);

        if (!ctb::GetParamVector(state, covarianceDiag, "initializationCovariance"))
            return false;

        Eigen::MatrixXd initialCovariance = Eigen::MatrixXd::Zero(stateDim, stateDim);

        initialCovariance.diagonal() = covarianceDiag;

        extendedKalmanFilter->Init(initialState, initialCovariance);

        return true;
    }

    bool NavigationFilter::LuenbergerObserverConfiguration(libconfig::Config& confObj) noexcept(false)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& luenbergerObs = root["luenbergerObserver"];

        Eigen::VectorXd gain;

        if (!ctb::GetParamVector(luenbergerObs, gain, "gain"))
            return false;
        obs.k = gain;

        //yaw rate digital filter gains. This is not used by the filter but is needed to filter the yaw rate
        if (!ctb::GetParamVector(luenbergerObs, yawRateFilterGains, "yawRateFilterGains"))
            return false;

        return true;
    }

    void NavigationFilter::CommandHandler(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Request> request, std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Response> response)
    {
        (void)request_header;
        RCLCPP_INFO(node->get_logger(), "Incoming request: %s", CommandTypeToString(static_cast<CommandType>(request->command_type)).c_str());

        CommandAnswer ret = CommandAnswer::ok;

        switch (request->command_type) {
        case static_cast<uint16_t>(CommandType::undefined):
            RCLCPP_WARN(node->get_logger(), "CommandType undefined");
            ret = CommandAnswer::fail;
            break;
        case static_cast<uint16_t>(CommandType::reset):
            if (filterParams.mode == FilterMode::LuenbergerObserver) {
                obs.Reset();
                RCLCPP_INFO(node->get_logger(), "Reset Luenberger observer");
            } else {
                extendedKalmanFilter->Reset();
                //sample the current gps data anfd yaw
                Eigen::VectorXd initialState = Eigen::VectorXd::Zero(stateDim);
                Eigen::Vector3d NED_currentPosition;
                ctb::LatLong2LocalNED(ctb::LatLong(gpsData.latitude, gpsData.longitude), gpsData.altitude, centroidLocation, NED_currentPosition);
                initialState.segment(0, 2) = NED_currentPosition.segment(0, 2);

                initialState[5] = -atan2(magnetometerData.orthogonalstrength[1] * cos(state[3]) - magnetometerData.orthogonalstrength[2] * sin(state[3]), magnetometerData.orthogonalstrength[0] * cos(state[4]) + magnetometerData.orthogonalstrength[2] * cos(state[3]) * sin(state[4]) + magnetometerData.orthogonalstrength[1] * sin(state[4]) * sin(state[3]));

                extendedKalmanFilter->Init(initialState);
                RCLCPP_INFO(node->get_logger(), "Reset EKF");
            }
            break;
        case static_cast<uint16_t>(CommandType::reloadconfig): {
            auto previousFilterParams = filterParams;
            LoadConfiguration(previousFilterParams);
            break;
        }
        default:
            RCLCPP_WARN(node->get_logger(), "Unsupported Command Code");
            break;
        }
        if (ret != CommandAnswer::ok) {
            response->res = static_cast<int16_t>(CommandAnswer::fail);
        } else {
            response->res = static_cast<int16_t>(CommandAnswer::ok);
        }
    }

    void NavigationFilter::CompassDataCB(const ulisse_msgs::msg::Compass::SharedPtr msg) { compassData = *msg; }

    void NavigationFilter::GPSDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg) { gpsData = *msg; }

    void NavigationFilter::IMUDataCB(const ulisse_msgs::msg::IMUData::SharedPtr msg) { imuData = *msg; }

    void NavigationFilter::MagnetometerDataCB(const ulisse_msgs::msg::Magnetometer::SharedPtr msg) { magnetometerData = *msg; }

    void NavigationFilter::SimulatedVelocitySensorCB(const ulisse_msgs::msg::SimulatedVelocitySensor::SharedPtr msg) { simulatedVelocitySensor = *msg; }

    void NavigationFilter::ThrustersDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg) { thrustersFbk = *msg; }

    void NavigationFilter::GroundTruthDataCB(const ulisse_msgs::msg::RealSystem::SharedPtr msg) { groundTruthData = *msg; }


}
}
