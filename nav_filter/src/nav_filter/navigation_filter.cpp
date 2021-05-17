#include "nav_filter/navigation_filter.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace ulisse {

namespace nav {
    NavigationFilter::NavigationFilter(const std::string& confPath)
        : Node("navigation_filter_node"), confPath_(confPath)
    {
        stateDim_ = 0;
        centroidLocation_ = ctb::LatLong(44.4, 8.94);
        ulisseModelEKF_ = std::make_shared<UlisseVehicleModel>(UlisseVehicleModel());

        std::vector<int> indexAngles = { 3, 4, 5 }; //rpy
        extendedKalmanFilter_ = std::make_shared<ctb::ExtendedKalmanFilter>(ctb::ExtendedKalmanFilter(stateDim_, indexAngles, ulisseModelEKF_));

        gyroMeasurement_ = std::make_shared<ulisse::nav::GyroMeasurement>(ulisse::nav::GyroMeasurement());
        compassMeasurement_ = std::make_shared<ulisse::nav::CompassMeasurement>(ulisse::nav::CompassMeasurement());
        accelerometerMeasurement_ = std::make_shared<ulisse::nav::AccelerometerMeasurement>(ulisse::nav::AccelerometerMeasurement());
        gpsMeasurement_ = std::make_shared<ulisse::nav::GpsMeasurement>(ulisse::nav::GpsMeasurement());
        magnetometerMeasurement_ = std::make_shared<ulisse::nav::MagnetometerMeasurement>(ulisse::nav::MagnetometerMeasurement());
        zMeterMeasurement_ = std::make_shared<ulisse::nav::zMeter>(ulisse::nav::zMeter());

        //Load filter params
        if (!LoadConfiguration(filterParams_)) {
            std::cerr << "Failed to load navigation filter configuration" << std::endl;
            exit(EXIT_FAILURE);
        }

        //Publisher of nav data structure
        navDataPub_ = this->create_publisher<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10);

        //Subscribes to data sensors
        compassSub_ = this->create_subscription<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass,
            10, std::bind(&NavigationFilter::CompassDataCB, this, _1));
        gpsdataSub_ = this->create_subscription<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data,
            10, std::bind(&NavigationFilter::GPSDataCB, this, _1));
        imudataSub_ = this->create_subscription<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu,
            10, std::bind(&NavigationFilter::IMUDataCB, this, _1));
        magnetometerSub_ = this->create_subscription<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer,
            10, std::bind(&NavigationFilter::MagnetometerDataCB, this, _1));
        simulatedSystemSub_ = this->create_subscription<ulisse_msgs::msg::SimulatedSystem>(ulisse_msgs::topicnames::simulated_system,
            10, std::bind(&NavigationFilter::GroundTruthDataCB, this, _1));
        thrustersFkbSub_ = this->create_subscription<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data,
            10, std::bind(&NavigationFilter::ThrustersDataCB, this, _1));
        simulatedVelocitySub_ = this->create_subscription<ulisse_msgs::msg::SimulatedVelocitySensor>(ulisse_msgs::topicnames::simulated_velocity_sensor,
            10, std::bind(&NavigationFilter::SimulatedVelocitySensorCB, this, _1));

        lastValidGPSTime_ = 0.0;
        lastValidImuTime_ = 0.0;
        lastValidCompassTime_ = 0.0;
        lastValidMagnetomerTime_ = 0.0;

        filterData_.gps_received = filterData_.imu_received = filterData_.compass_received = filterData_.magnetometer_received = false;

        previousYaw_ = 0.0;
        sampleTime_ = 0.0;

        if (filterParams_.mode == FilterMode::LuenbergerObserver) {
            //init position
            gpsData_.latitude = 44.095693;
            gpsData_.longitude = 9.862684;

            sampleTime_ = 1.0 / filterParams_.rate;
        }

        filterEnable_ = true;
        isFirst_ = true;
        state_ = Eigen::VectorXd::Zero(stateDim_);

        last_comp_time_ = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        sensorsCheckInterval_ = 5;
        sensorsCheckTimer_ = this->create_wall_timer(std::chrono::seconds(sensorsCheckInterval_),
            std::bind(&NavigationFilter::SensorsOnlineCB, this));

        int msRunPeriod = 1.0/(filterParams_.rate) * 1000;
        std::cout << "NavFilter Rate: " << filterParams_.rate << "Hz" << std::endl;
        //std::cout << "-> msRunPeriod: " << msRunPeriod << "ms" << std::endl;
        runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&NavigationFilter::Run, this));

        // Service
        navFilterCmdService_ = this->create_service<ulisse_msgs::srv::NavFilterCommand>(ulisse_msgs::topicnames::navfilter_cmd_service,
            std::bind(&NavigationFilter::CommandHandler, this, _1, _2, _3));

    }

    NavigationFilter::~NavigationFilter(){ }


    void NavigationFilter::Run(){

        SensorsValidityCheck();

        if (filterParams_.mode == FilterMode::LuenbergerObserver) {
            LuenbergerObserverFilter();
        } else if (filterParams_.mode == FilterMode::KalmanFilter) {
            ExtendedKalmanFilter();
        } else if (filterParams_.mode == FilterMode::GroundTruth) {
            GroundThruthFilter();
        }

        navDataPub_->publish(filterData_);
    }

    void NavigationFilter::LuenbergerObserverFilter(){
        if (gpsValid_) {

            int zone;
            bool northp;
            Eigen::Vector2d p_utm;

            try {
                GeographicLib::UTMUPS::Forward(gpsData_.latitude, gpsData_.longitude, zone, northp, p_utm.x(), p_utm.y());

                // The geographic lib conversion outputs UTM coordinates but the filter uses NED.
                Eigen::Vector2d p_ned = { p_utm.y(), p_utm.x() };

                if (filterEnable_) {

                    obs_.Update(Eigen::Vector4d { p_ned.x(), p_ned.y(), compassData_.orientation.yaw, simulatedVelocitySensor_.water_relative_surge });
                    filterData_.inertialframe_water_current.fill(0.0);

                    //Construct the inertial to body frame rotation
                    rml::EulerRPY rpy { 0.0, 0.0, compassData_.orientation.yaw };
                    Eigen::Vector3d bodyF_linearVelocity = rpy.ToRotationMatrix().transpose() * Eigen::Vector3d { obs_.LinearVelocity().x(), obs_.LinearVelocity().y(), 0.0 };

                    filterData_.bodyframe_linear_velocity[0] = bodyF_linearVelocity.x();
                    filterData_.bodyframe_linear_velocity[1] = bodyF_linearVelocity.y();

                    p_ned = obs_.LinearPosition();

                    p_utm = { p_ned.y(), p_ned.x() };

                    GeographicLib::UTMUPS::Reverse(zone, northp, p_utm.x(), p_utm.y(), filterData_.inertialframe_linear_position.latlong.latitude, filterData_.inertialframe_linear_position.latlong.longitude);
                }

                /// FILL THE MSG WITH ALL THE REST OF UNMANAGED DATA
                filterData_.bodyframe_linear_velocity[2] = 0.0;
                filterData_.inertialframe_linear_position.altitude = 0.0;
                filterData_.bodyframe_angular_position = compassData_.orientation;

                filterData_.bodyframe_angular_velocity[0] = imuData_.gyro[0];
                filterData_.bodyframe_angular_velocity[1] = imuData_.gyro[1];

                //Yaw rate estimation with a digital filter
                double omega_dot_dot = ctb::AngleDifference(compassData_.orientation.yaw, previousYaw_) / sampleTime_;
                previousYaw_ = compassData_.orientation.yaw;

                filterData_.bodyframe_angular_velocity[2] = yawRateFilterGains_[0] * filterData_.bodyframe_angular_velocity[2] + yawRateFilterGains_[1] * omega_dot_dot;

            } catch (const GeographicLib::GeographicErr& e) {
                RCLCPP_ERROR(this->get_logger(), "GeographicLib exception: what = %s", e.what());
                obs_.Reset();
            }
        }
    }

    void NavigationFilter::ExtendedKalmanFilter(){
        if (gpsValid_) {
            if (measuresActive_.find("gps")->second) {

                if (isFirst_) {
                    Eigen::VectorXd initialState = Eigen::VectorXd::Zero(stateDim_);
                    centroidLocation_ = { gpsData_.latitude, gpsData_.longitude };
                    ctb::LatLong2LocalNED(ctb::LatLong(gpsData_.latitude, gpsData_.longitude), gpsData_.altitude, centroidLocation_, NED_gps_cartesian_);

                    initialState.segment(0, 2) << NED_gps_cartesian_.x(), NED_gps_cartesian_.y();
                    isFirst_ = false;
                }

                //The filter use the cartesian coordinates
                ctb::LatLong2LocalNED(ctb::LatLong(gpsData_.latitude, gpsData_.longitude), gpsData_.altitude, centroidLocation_, NED_gps_cartesian_);
                gpsMeasurement_->MeasureVector() = Eigen::Vector2d { NED_gps_cartesian_.x(), NED_gps_cartesian_.y() };
                extendedKalmanFilter_->AddMeasurement(gpsMeasurement_);
            }
        }

        if (imuValid_) {
            if (measuresActive_.find("gyro")->second) {
                gyroMeasurement_->MeasureVector() = Eigen::Vector3d { imuData_.gyro[0], imuData_.gyro[1], imuData_.gyro[2] };
                extendedKalmanFilter_->AddMeasurement(gyroMeasurement_);
            }
            if (measuresActive_.find("accelerometer")->second) {
                accelerometerMeasurement_->MeasureVector() = Eigen::Vector3d { imuData_.accelerometer[0], imuData_.accelerometer[1], imuData_.accelerometer[2] };
                extendedKalmanFilter_->AddMeasurement(accelerometerMeasurement_);
            }

        }

        if (compassValid_) {
            if (measuresActive_.find("compass")->second) {
                compassMeasurement_->MeasureVector() = Eigen::Vector3d { compassData_.orientation.roll, compassData_.orientation.pitch, compassData_.orientation.yaw };
                extendedKalmanFilter_->AddMeasurement(compassMeasurement_);
            }
        }

        if (magnetometerValid_) {
            if (measuresActive_.find("magnetometer")->second) {
                //preprocessing: I compensate for the roll and the pitch and then I pretend to have a sensor that measures the yaw
                magnetometerMeasurement_->MeasureVector() << -atan2(magnetometerData_.orthogonalstrength[1] * cos(state_[3]) - magnetometerData_.orthogonalstrength[2] * sin(state_[3]), magnetometerData_.orthogonalstrength[0] * cos(state_[4]) + magnetometerData_.orthogonalstrength[2] * cos(state_[3]) * sin(state_[4]) + magnetometerData_.orthogonalstrength[1] * sin(state_[4]) * sin(state_[3]));
                extendedKalmanFilter_->AddMeasurement(magnetometerMeasurement_);
            }
        }

        //Added a perfect com altitude meter to be coherent with the real data that recod 0.0 as altitude

        if (measuresActive_.find("zMeter")->second) {
            Eigen::Vector3d NED_p;
            zMeterMeasurement_->MeasureVector() << 0.0;
            extendedKalmanFilter_->AddMeasurement(zMeterMeasurement_);
        }

        //Filter Update
        extendedKalmanFilter_->Update(Eigen::Vector2d { thrustersFbk_.motor_percentage.left, thrustersFbk_.motor_percentage.right });

        state_ = extendedKalmanFilter_->StateVector();

        ctb::LatLong map_p;
        double altitude;

        ctb::LocalNED2LatLong(Eigen::Vector3d { state_.x(), state_.y(), state_.z() }, centroidLocation_, map_p, altitude);

        auto tNow = std::chrono::system_clock::now();
        long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow.time_since_epoch())).count();
        auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
        auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

        filterData_.stamp.sec = now_stamp_secs;
        filterData_.stamp.nanosec = now_stamp_nanosecs;

        filterData_.inertialframe_linear_position.latlong.latitude = map_p.latitude;
        filterData_.inertialframe_linear_position.latlong.longitude = map_p.longitude;
        filterData_.inertialframe_linear_position.altitude = altitude;
        filterData_.bodyframe_angular_position.roll = state_[3];
        filterData_.bodyframe_angular_position.pitch = state_[4];
        filterData_.bodyframe_angular_position.yaw = state_[5];
        filterData_.bodyframe_linear_velocity[0] = state_[6];
        filterData_.bodyframe_linear_velocity[1] = state_[7];
        filterData_.bodyframe_linear_velocity[2] = state_[8];
        filterData_.bodyframe_angular_velocity[0] = state_[9];
        filterData_.bodyframe_angular_velocity[1] = state_[10];
        filterData_.bodyframe_angular_velocity[2] = state_[11];
        filterData_.inertialframe_water_current[0] = state_[12];
        filterData_.inertialframe_water_current[1] = state_[13];
        filterData_.gyro_bias[0] = state_[14];
        filterData_.gyro_bias[1] = state_[15];
        filterData_.gyro_bias[2] = state_[16];

        std::vector<double> P;
        for (unsigned int i = 0; i < extendedKalmanFilter_->PropagationError().rows(); i++) {
            P.push_back(extendedKalmanFilter_->PropagationError().at(i, i));
        }

        filterData_.covariance_estimation_diag = P;
    }

    void NavigationFilter::GroundThruthFilter(){
        auto tNow = std::chrono::system_clock::now();
        long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow.time_since_epoch())).count();
        auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
        auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

        filterData_.stamp.sec = now_stamp_secs;
        filterData_.stamp.nanosec = now_stamp_nanosecs;

        filterData_.inertialframe_linear_position.latlong.latitude = simulatedData_.inertialframe_linear_position.latlong.latitude;
        filterData_.inertialframe_linear_position.latlong.longitude = simulatedData_.inertialframe_linear_position.latlong.longitude;
        filterData_.inertialframe_linear_position.altitude = simulatedData_.inertialframe_linear_position.altitude;
        filterData_.bodyframe_angular_position.roll = simulatedData_.bodyframe_angular_position.roll;
        filterData_.bodyframe_angular_position.pitch = simulatedData_.bodyframe_angular_position.pitch;
        filterData_.bodyframe_angular_position.yaw = simulatedData_.bodyframe_angular_position.yaw;
        filterData_.bodyframe_linear_velocity[0] = simulatedData_.bodyframe_linear_velocity[0];
        filterData_.bodyframe_linear_velocity[1] = simulatedData_.bodyframe_linear_velocity[1];
        filterData_.bodyframe_linear_velocity[2] = simulatedData_.bodyframe_linear_velocity[2];

        filterData_.bodyframe_angular_velocity[0] = simulatedData_.bodyframe_angular_velocity[0];
        filterData_.bodyframe_angular_velocity[1] = simulatedData_.bodyframe_angular_velocity[1];
        filterData_.bodyframe_angular_velocity[2] = simulatedData_.bodyframe_angular_velocity[2];
        filterData_.inertialframe_water_current[0] = simulatedData_.inertialframe_water_current[0];
        filterData_.inertialframe_water_current[1] = simulatedData_.inertialframe_water_current[1];
        filterData_.gyro_bias[0] = simulatedData_.gyro_bias[0];
        filterData_.gyro_bias[1] = simulatedData_.gyro_bias[1];
        filterData_.gyro_bias[2] = simulatedData_.gyro_bias[2];
    }

    void NavigationFilter::SensorsValidityCheck(){

        gpsValid_ = imuValid_ = compassValid_ = magnetometerValid_ = false;

        if (gpsData_.time > lastValidGPSTime_) {
            if (gpsData_.gpsfixmode >= static_cast<int>(ulisse::gpsd::GpsFixMode::mode_2d)) {
                gpsValid_ = true;
                filterData_.gps_received = true;
                lastValidGPSTime_ = gpsData_.time;

            } else {
                gpsValid_ = false;
            }
        }

        if (imuData_.stamp.sec + (imuData_.stamp.nanosec * 1e-9) > lastValidImuTime_) {
            imuValid_ = true;
            filterData_.imu_received = true;
            lastValidImuTime_ = imuData_.stamp.sec + (imuData_.stamp.nanosec * 1e-9);
        } else {
            imuValid_ = false;
        }

        if (compassData_.stamp.sec + (compassData_.stamp.nanosec * 1e-9) > lastValidCompassTime_) {
            compassValid_ = true;
            filterData_.compass_received = true;
            lastValidCompassTime_ = compassData_.stamp.sec + (compassData_.stamp.nanosec * 1e-9);
        } else {
            compassValid_ = false;
        }

        if (magnetometerData_.stamp.sec + (magnetometerData_.stamp.nanosec * 1e-9) > lastValidMagnetomerTime_) {
            magnetometerValid_ = true;
            filterData_.magnetometer_received = true;
            lastValidMagnetomerTime_ = magnetometerData_.stamp.sec + (magnetometerData_.stamp.nanosec * 1e-9);
        } else {
            magnetometerValid_ = false;
        }
    }

    void NavigationFilter::SensorsOnlineCB(){

        auto t_now = std::chrono::system_clock::now();
        auto timeNowSecs = (std::chrono::duration_cast<std::chrono::seconds>(t_now.time_since_epoch())).count();
        auto lastValidGPSSecs = static_cast<std::time_t>(lastValidGPSTime_);

        if (std::abs(lastValidGPSSecs - timeNowSecs) > sensorsCheckInterval_){
            RCLCPP_WARN(this->get_logger(), "GPS Data unavailable for more than %i seconds.", sensorsCheckInterval_);
            filterData_.gps_received = false;
        }

        if (std::abs(imuData_.stamp.sec - timeNowSecs) > sensorsCheckInterval_){
            RCLCPP_WARN(this->get_logger(), "IMU Data unavailable for more than %i seconds.", sensorsCheckInterval_);
            filterData_.imu_received = false;
        }

        if (std::abs(compassData_.stamp.sec - timeNowSecs) > sensorsCheckInterval_){
            RCLCPP_WARN(this->get_logger(), "Compass Data unavailable for more than %i seconds.", sensorsCheckInterval_);
            filterData_.compass_received = false;
        }

        if (std::abs(magnetometerData_.stamp.sec - timeNowSecs) > sensorsCheckInterval_){
            RCLCPP_WARN(this->get_logger(), "Magnetometer Data unavailable for more than %i seconds.", sensorsCheckInterval_);
            filterData_.magnetometer_received = false;
        }

        // Utility Print
        if(!filterData_.gps_received){
            if (std::ctime(&timeNowSecs) != nullptr) {
                std::string timedate_cpu = std::ctime(&timeNowSecs);
                timedate_cpu.erase(std::remove(timedate_cpu.begin(), timedate_cpu.end(), '\n'), timedate_cpu.end());
                std::cout << "CPU Time now = " << timedate_cpu << std::endl;
            }  else {
                std::cerr << "CPU Time now = nullptr" << std::endl;
            }

            if (std::ctime(&lastValidGPSSecs) != nullptr) {
                std::string timedate_gps = std::ctime(&lastValidGPSSecs);
                timedate_gps.erase(std::remove(timedate_gps.begin(), timedate_gps.end(), '\n'), timedate_gps.end());
                std::cout << "GPS Time now = " << timedate_gps << std::endl;
            }  else {
                std::cerr << "GPS Time now = nullptr" << std::endl;
            }
        }
    }

    bool NavigationFilter::LoadConfiguration(NavigationFilterParams& filterParameters)
    {
        // Read conf file
        libconfig::Config confObj;

        //I nizialization
        try {
            confObj.readFile(confPath_.c_str());
        } catch (const libconfig::FileIOException& fioex) {
            std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
            return false;
        } catch (libconfig::ParseException& e) {
            std::cerr << "Parse exception when reading:" << confPath_ << std::endl;
            std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
            return false;
        }

        // Configure the filter node params
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

        // Load the ulisse params
        const libconfig::Setting& ulisseModel = ekf["ulisseModel"];
        UlisseModelParameters ulisseModelParams;

        if (!ulisseModelParams.ConfigureFormFile(ulisseModel)) {
            std::cerr << "Kalman Filter: Failed to load ulisse model" << std::endl;
            return false;
        }

        std::cout << ulisseModelParams << std::endl;

        ulisseModelEKF_->ModelParameters() = ulisseModelParams;

        // Load the measures covariance
        const libconfig::Setting& measure = ekf["measures"];

        Eigen::VectorXd covarianceDiag;
        bool isActive;
        Eigen::Vector3d bodyF_gps_position;

        const libconfig::Setting& gps = measure["gps"];
        if (!ctb::GetParam(gps, isActive, "enable"))
            return false;
        measuresActive_.insert(std::make_pair("gps", isActive));
        if (!ctb::GetParamVector(gps, covarianceDiag, "covariance"))
            return false;
        gpsMeasurement_->Covariance().diagonal() = covarianceDiag;

        if (!ctb::GetParamVector(gps, bodyF_gps_position, "bodyF_gps_position"))
            return false;
        gpsMeasurement_->bodyF_gps_position_ = bodyF_gps_position;

        const libconfig::Setting& compass = measure["compass"];
        if (!ctb::GetParam(compass, isActive, "enable"))
            return false;

        measuresActive_.insert(std::make_pair("compass", isActive));
        if (!ctb::GetParamVector(compass, covarianceDiag, "covariance"))
            return false;

        compassMeasurement_->Covariance().diagonal() = covarianceDiag;

        const libconfig::Setting& gyro = measure["gyro"];
        if (!ctb::GetParam(gyro, isActive, "enable"))
            return false;

        measuresActive_.insert(std::make_pair("gyro", isActive));
        if (!ctb::GetParamVector(gyro, covarianceDiag, "covariance"))
            return false;

        gyroMeasurement_->Covariance().diagonal() = covarianceDiag;

        const libconfig::Setting& accelerometer = measure["accelerometer"];
        if (!ctb::GetParam(accelerometer, isActive, "enable"))
            return false;

        measuresActive_.insert(std::make_pair("accelerometer", isActive));
        if (!ctb::GetParamVector(accelerometer, covarianceDiag, "covariance"))
            return false;

        accelerometerMeasurement_->Covariance().diagonal() = covarianceDiag;

        const libconfig::Setting& magnetometer = measure["magnetometer"];
        if (!ctb::GetParam(magnetometer, isActive, "enable"))
            return false;

        measuresActive_.insert(std::make_pair("magnetometer", isActive));
        if (!ctb::GetParamVector(magnetometer, covarianceDiag, "covariance"))
            return false;

        magnetometerMeasurement_->Covariance().diagonal() = covarianceDiag;

        const libconfig::Setting& zMeterSetting = measure["z_meter"];
        if (!ctb::GetParam(zMeterSetting, isActive, "enable"))
            return false;

        measuresActive_.insert(std::make_pair("zMeter", isActive));
        if (!ctb::GetParamVector(zMeterSetting, covarianceDiag, "covariance"))
            return false;

        zMeterMeasurement_->Covariance().diagonal() = covarianceDiag;

        // Load the initial state and covariance and the model covariance

        // State dimention
        const libconfig::Setting& state = ekf["state"];

        if (!ctb::GetParam(state, stateDim_, "dim"))
            return false;

        if (!ctb::GetParamVector(state, covarianceDiag, "modelCovariance"))
            return false;
        ulisseModelEKF_->Covariance().diagonal() = covarianceDiag;

        Eigen::VectorXd initialState;
        if (!ctb::GetParamVector(state, initialState, "initialization"))
            return false;

        Eigen::Vector3d cartesian_p;
        ctb::LatLong2LocalNED(ctb::LatLong(initialState[0], initialState[1]), initialState[2], centroidLocation_, cartesian_p);
        initialState.segment(0, 2) = cartesian_p.segment(0, 2);

        if (!ctb::GetParamVector(state, covarianceDiag, "initializationCovariance"))
            return false;

        Eigen::MatrixXd initialCovariance = Eigen::MatrixXd::Zero(stateDim_, stateDim_);

        initialCovariance.diagonal() = covarianceDiag;

        extendedKalmanFilter_->Init(initialState, initialCovariance);

        return true;
    }

    bool NavigationFilter::LuenbergerObserverConfiguration(libconfig::Config& confObj) noexcept(false)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& luenbergerObs = root["luenbergerObserver"];

        Eigen::VectorXd gain;

        if (!ctb::GetParamVector(luenbergerObs, gain, "gain"))
            return false;
        obs_.k = gain;

        // Yaw rate digital filter gains. This is not used by the filter but is needed to filter the yaw rate
        if (!ctb::GetParamVector(luenbergerObs, yawRateFilterGains_, "yawRateFilterGains"))
            return false;

        return true;
    }

    void NavigationFilter::CommandHandler(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Request> request,
        std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Response> response)
    {
        (void)request_header;
        RCLCPP_INFO(this->get_logger(), "Incoming request: %s", CommandTypeToString(static_cast<CommandType>(request->command_type)).c_str());

        CommandAnswer ret = CommandAnswer::ok;

        switch (request->command_type) {
        case static_cast<uint16_t>(CommandType::undefined):
            RCLCPP_WARN(this->get_logger(), "CommandType undefined");
            ret = CommandAnswer::fail;
            break;
        case static_cast<uint16_t>(CommandType::reset):
            if (filterParams_.mode == FilterMode::LuenbergerObserver) {
                obs_.Reset();
                RCLCPP_INFO(this->get_logger(), "Reset Luenberger observer");
            } else {
                extendedKalmanFilter_->Reset();
                //sample the current gps data anfd yaw
                Eigen::VectorXd initialState = Eigen::VectorXd::Zero(stateDim_);
                Eigen::Vector3d NED_currentPosition;
                ctb::LatLong2LocalNED(ctb::LatLong(gpsData_.latitude, gpsData_.longitude), gpsData_.altitude, centroidLocation_, NED_currentPosition);
                initialState.segment(0, 2) = NED_currentPosition.segment(0, 2);

                initialState[5] = -atan2(magnetometerData_.orthogonalstrength[1] * cos(state_[3]) - magnetometerData_.orthogonalstrength[2] * sin(state_[3]), magnetometerData_.orthogonalstrength[0] * cos(state_[4]) + magnetometerData_.orthogonalstrength[2] * cos(state_[3]) * sin(state_[4]) + magnetometerData_.orthogonalstrength[1] * sin(state_[4]) * sin(state_[3]));

                extendedKalmanFilter_->Init(initialState);
                RCLCPP_INFO(this->get_logger(), "Reset EKF");
            }
            break;
        case static_cast<uint16_t>(CommandType::reloadconfig): {
            auto previousFilterParams = filterParams_;
            LoadConfiguration(previousFilterParams);
            break;
        }
        default:
            RCLCPP_WARN(this->get_logger(), "Unsupported Command Code");
            break;
        }
        if (ret != CommandAnswer::ok) {
            response->res = static_cast<int16_t>(CommandAnswer::fail);
        } else {
            response->res = static_cast<int16_t>(CommandAnswer::ok);
        }
    }

    void NavigationFilter::CompassDataCB(const ulisse_msgs::msg::Compass::SharedPtr msg) { compassData_ = *msg; }

    void NavigationFilter::GPSDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg) { gpsData_ = *msg; }

    void NavigationFilter::IMUDataCB(const ulisse_msgs::msg::IMUData::SharedPtr msg) { imuData_ = *msg; }

    void NavigationFilter::MagnetometerDataCB(const ulisse_msgs::msg::Magnetometer::SharedPtr msg) { magnetometerData_ = *msg; }

    void NavigationFilter::SimulatedVelocitySensorCB(const ulisse_msgs::msg::SimulatedVelocitySensor::SharedPtr msg) { simulatedVelocitySensor_ = *msg; }

    void NavigationFilter::ThrustersDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg) { thrustersFbk_ = *msg; }

    void NavigationFilter::GroundTruthDataCB(const ulisse_msgs::msg::SimulatedSystem::SharedPtr msg) { simulatedData_ = *msg; }


}
}
