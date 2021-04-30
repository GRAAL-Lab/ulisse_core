#include "nav_filter/navigation_filter.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace ulisse {

namespace nav {
    NavigationFilter::NavigationFilter(const rclcpp::Node::SharedPtr& nh, const std::string& confPath)
        : nh_(nh), confPath_(confPath)
    {
        ulisseModelEKF = std::make_shared<UlisseVehicleModel>(UlisseVehicleModel());

        //Load filter params
        if (!LoadConfiguration(filterParams)) {
            std::cerr << "Failed to load navigation filter configuration" << std::endl;
            exit(EXIT_FAILURE);
        }


        std::vector<int> indexAngles = { 3, 4, 5 }; //rpy
        extendedKalmanFilter = std::make_shared<ctb::ExtendedKalmanFilter>(ctb::ExtendedKalmanFilter(stateDim, indexAngles, ulisseModelEKF));

        gyroMeasurement = std::make_shared<ulisse::nav::GyroMeasurement>(ulisse::nav::GyroMeasurement());
        compassMeasurement = std::make_shared<ulisse::nav::CompassMeasurement>(ulisse::nav::CompassMeasurement());
        accelerometerMeasurement = std::make_shared<ulisse::nav::AccelerometerMeasurement>(ulisse::nav::AccelerometerMeasurement());
        gpsMeasurement = std::make_shared<ulisse::nav::GpsMeasurement>(ulisse::nav::GpsMeasurement());
        magnetometerMeasurement = std::make_shared<ulisse::nav::MagnetometerMeasurement>(ulisse::nav::MagnetometerMeasurement());
        zMeterMeasurement = std::make_shared<ulisse::nav::zMeter>(ulisse::nav::zMeter());


        //rclcpp::WallRate loop_rate(filterParams.rate);

        //Publisher of nav data structure
        navDataPub_ = nh_->create_publisher<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10);

        //Subscribes to data sensors
        compassSub_ = nh_->create_subscription<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass,
            10, std::bind(&NavigationFilter::CompassDataCB, this, _1));
        gpsdataSub_ = nh_->create_subscription<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data,
            10, std::bind(&NavigationFilter::GPSDataCB, this, _1));
        imudataSub_ = nh_->create_subscription<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu,
            10, std::bind(&NavigationFilter::IMUDataCB, this, _1));
        magnetometerSub_ = nh_->create_subscription<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer,
            10, std::bind(&NavigationFilter::MagnetometerDataCB, this, _1));
        groundTruthSub_ = nh_->create_subscription<ulisse_msgs::msg::RealSystem>(ulisse_msgs::topicnames::real_system,
            10, std::bind(&NavigationFilter::GroundTruthDataCB, this, _1));
        thrustersFkbSub_ = nh_->create_subscription<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data,
            10, std::bind(&NavigationFilter::ThrustersDataCB, this, _1));
        simulatedVelocitySub_ = nh_->create_subscription<ulisse_msgs::msg::SimulatedVelocitySensor>(ulisse_msgs::topicnames::simulated_velocity_sensor,
            10, std::bind(&NavigationFilter::SimulatedVelocitySensorCB, this, _1));

        lastValidGPSTime_ = 0.0;
        lastValidImuTime_ = 0.0;
        lastValidCompassTime_ = 0.0;
        lastValidMagnetomerTime_ = 0.0;

        gpsOnline_ = imuOnline_ = compassOnline_ = magnetometerOnline_ = false;

        previousYaw_ = 0.0;
        sampleTime_ = 0.0;

        if (filterParams.mode == FilterMode::LuenbergerObserver) {
            //init position
            gpsData.latitude = 44.095693;
            gpsData.longitude = 9.862684;

            sampleTime_ = 1.0 / filterParams.rate;
        }

        filterEnable_ = true;
        isFirst_ = true;
        state = Eigen::VectorXd::Zero(stateDim);

        last_comp_time_ = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        sensorsCheckTimer_ = nh_->create_wall_timer(5s, std::bind(&NavigationFilter::SensorsCheckCB, this));

        int msRunPeriod = 1.0/(filterParams.rate)*1000;
        std::cout << "Rate: " << filterParams.rate << "Hz" << std::endl;
        std::cout << "-> msRunPeriod: " << msRunPeriod << std::endl;
        runTimer_ = nh_->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&NavigationFilter::Run, this));

        //service
        navFilterCmdService_ = nh_->create_service<ulisse_msgs::srv::NavFilterCommand>(ulisse_msgs::topicnames::navfilter_cmd_service,
            std::bind(&NavigationFilter::CommandHandler, this, _1, _2, _3));

    }

    NavigationFilter::~NavigationFilter(){ }


    void NavigationFilter::Run(){

        bool gpsValid(false), imuValid(false), compassValid(false), magnetometerValid(false);

        ///// Sensors Check /////
        if (gpsData.time > lastValidGPSTime_) {
            if (gpsData.gpsfixmode >= static_cast<int>(ulisse::gpsd::GpsFixMode::mode_2d)) {
                gpsValid = true;
                gpsOnline_ = true;
                lastValidGPSTime_ = gpsData.time;
            } else {
                gpsValid = false;
            }
        }
        long now_secs = static_cast<long>(lastValidGPSTime_);
        std::time_t current = now_secs;

        if (imuData.stamp.sec + (imuData.stamp.nanosec * 1e-9) > lastValidImuTime_) {
            imuValid = true;
            imuOnline_ = true;
            lastValidImuTime_ = imuData.stamp.sec + (imuData.stamp.nanosec * 1e-9);
        } else {
            imuValid = false;
        }

        if (compassData.stamp.sec + (compassData.stamp.nanosec * 1e-9) > lastValidCompassTime_) {
            compassValid = true;
            compassOnline_ = true;
            lastValidCompassTime_ = compassData.stamp.sec + (compassData.stamp.nanosec * 1e-9);
        } else {
            compassValid = false;
        }

        if (magnetometerData.stamp.sec + (magnetometerData.stamp.nanosec * 1e-9) > lastValidMagnetomerTime_) {
            magnetometerValid = true;
            magnetometerOnline_ = true;
            lastValidMagnetomerTime_ = magnetometerData.stamp.sec + (magnetometerData.stamp.nanosec * 1e-9);
        } else {
            magnetometerValid = false;
        }

        /////////////////////////

        if (filterParams.mode == FilterMode::LuenbergerObserver) {
            if (gpsValid) {

                int zone;
                bool northp;
                Eigen::Vector2d p_utm;

                try {
                    GeographicLib::UTMUPS::Forward(gpsData.latitude, gpsData.longitude, zone, northp, p_utm.x(), p_utm.y());

                    // The geographic lib conversion outputs UTM coordinates but the filter uses NED.
                    Eigen::Vector2d p_ned = { p_utm.y(), p_utm.x() };

                    if (filterEnable_) {

                        obs.Update(Eigen::Vector4d { p_ned.x(), p_ned.y(), compassData.orientation.yaw, simulatedVelocitySensor.water_relative_surge });
                        filterData_.inertialframe_water_current.fill(0.0);

                        //Construct the inertial to body frame rotation
                        rml::EulerRPY rpy { 0.0, 0.0, compassData.orientation.yaw };
                        Eigen::Vector3d bodyF_linearVelocity = rpy.ToRotationMatrix().transpose() * Eigen::Vector3d { obs.LinearVelocity().x(), obs.LinearVelocity().y(), 0.0 };

                        filterData_.bodyframe_linear_velocity[0] = bodyF_linearVelocity.x();
                        filterData_.bodyframe_linear_velocity[1] = bodyF_linearVelocity.y();

                        p_ned = obs.LinearPosition();

                        p_utm = { p_ned.y(), p_ned.x() };

                        GeographicLib::UTMUPS::Reverse(zone, northp, p_utm.x(), p_utm.y(), filterData_.inertialframe_linear_position.latlong.latitude, filterData_.inertialframe_linear_position.latlong.longitude);
                    }

                    /// FILL THE MSG WITH ALL THE REST OF UNMANAGED DATA
                    filterData_.bodyframe_linear_velocity[2] = 0.0;
                    filterData_.inertialframe_linear_position.altitude = 0.0;
                    filterData_.bodyframe_angular_position = compassData.orientation;

                    filterData_.bodyframe_angular_velocity[0] = imuData.gyro[0];
                    filterData_.bodyframe_angular_velocity[1] = imuData.gyro[1];

                    //Yaw rate estimation with a digital filter
                    double omega_dot_dot = ctb::AngleDifference(compassData.orientation.yaw, previousYaw_) / sampleTime_;
                    previousYaw_ = compassData.orientation.yaw;

                    filterData_.bodyframe_angular_velocity[2] = yawRateFilterGains[0] * filterData_.bodyframe_angular_velocity[2] + yawRateFilterGains[1] * omega_dot_dot;

                } catch (const GeographicLib::GeographicErr& e) {
                    RCLCPP_ERROR(nh_->get_logger(), "GeographicLib exception: what = %s", e.what());
                    obs.Reset();
                }
            }
        } else if (filterParams.mode == FilterMode::KalmanFilter) {
            if (gpsValid) {
                if (measuresActive.find("gps")->second) {

                    if (isFirst_) {
                        Eigen::VectorXd initialState = Eigen::VectorXd::Zero(stateDim);
                        centroidLocation = { gpsData.latitude, gpsData.longitude };
                        ctb::LatLong2LocalNED(ctb::LatLong(gpsData.latitude, gpsData.longitude), gpsData.altitude, centroidLocation, NED_gps_cartesian_);

                        initialState.segment(0, 2) << NED_gps_cartesian_.x(), NED_gps_cartesian_.y();
                        isFirst_ = false;
                    }

                    //The filter use the cartesian coordinates
                    ctb::LatLong2LocalNED(ctb::LatLong(gpsData.latitude, gpsData.longitude), gpsData.altitude, centroidLocation, NED_gps_cartesian_);
                    gpsMeasurement->MeasureVector() = Eigen::Vector2d { NED_gps_cartesian_.x(), NED_gps_cartesian_.y() };
                    extendedKalmanFilter->AddMeasurement(gpsMeasurement);
                }
            }

            if (imuValid) {
                if (measuresActive.find("gyro")->second) {
                    gyroMeasurement->MeasureVector() = Eigen::Vector3d { imuData.gyro[0], imuData.gyro[1], imuData.gyro[2] };
                    extendedKalmanFilter->AddMeasurement(gyroMeasurement);
                }
                if (measuresActive.find("accelerometer")->second) {
                    accelerometerMeasurement->MeasureVector() = Eigen::Vector3d { imuData.accelerometer[0], imuData.accelerometer[1], imuData.accelerometer[2] };
                    extendedKalmanFilter->AddMeasurement(accelerometerMeasurement);
                }

            }

            if (compassValid) {
                if (measuresActive.find("compass")->second) {
                    compassMeasurement->MeasureVector() = Eigen::Vector3d { compassData.orientation.roll, compassData.orientation.pitch, compassData.orientation.yaw };
                    extendedKalmanFilter->AddMeasurement(compassMeasurement);
                }
            }

            if (magnetometerValid) {
                if (measuresActive.find("magnetometer")->second) {
                    //preprocessing: I compensate for the roll and the pitch and then I pretend to have a sensor that measures the yaw
                    magnetometerMeasurement->MeasureVector() << -atan2(magnetometerData.orthogonalstrength[1] * cos(state[3]) - magnetometerData.orthogonalstrength[2] * sin(state[3]), magnetometerData.orthogonalstrength[0] * cos(state[4]) + magnetometerData.orthogonalstrength[2] * cos(state[3]) * sin(state[4]) + magnetometerData.orthogonalstrength[1] * sin(state[4]) * sin(state[3]));
                    extendedKalmanFilter->AddMeasurement(magnetometerMeasurement);
                }
            }

            //Added a perfect com altitude meter to be coherent with the real data that recod 0.0 as altitude

            if (measuresActive.find("zMeter")->second) {
                Eigen::Vector3d NED_p;
                zMeterMeasurement->MeasureVector() << 0.0;
                extendedKalmanFilter->AddMeasurement(zMeterMeasurement);
            }

            //Filter Update
            extendedKalmanFilter->Update(Eigen::Vector2d { thrustersFbk.motor_percentage.left, thrustersFbk.motor_percentage.right });

            state = extendedKalmanFilter->StateVector();

            ctb::LatLong map_p;
            double altitude;

            ctb::LocalNED2LatLong(Eigen::Vector3d { state.x(), state.y(), state.z() }, centroidLocation, map_p, altitude);

            auto tNow = std::chrono::system_clock::now();
            long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow.time_since_epoch())).count();
            auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
            auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

            filterData_.stamp.sec = now_stamp_secs;
            filterData_.stamp.nanosec = now_stamp_nanosecs;

            filterData_.inertialframe_linear_position.latlong.latitude = map_p.latitude;
            filterData_.inertialframe_linear_position.latlong.longitude = map_p.longitude;
            filterData_.inertialframe_linear_position.altitude = altitude;
            filterData_.bodyframe_angular_position.roll = state[3];
            filterData_.bodyframe_angular_position.pitch = state[4];
            filterData_.bodyframe_angular_position.yaw = state[5];
            filterData_.bodyframe_linear_velocity[0] = state[6];
            filterData_.bodyframe_linear_velocity[1] = state[7];
            filterData_.bodyframe_linear_velocity[2] = state[8];
            filterData_.bodyframe_angular_velocity[0] = state[9];
            filterData_.bodyframe_angular_velocity[1] = state[10];
            filterData_.bodyframe_angular_velocity[2] = state[11];
            filterData_.inertialframe_water_current[0] = state[12];
            filterData_.inertialframe_water_current[1] = state[13];
            filterData_.gyro_bias[0] = state[14];
            filterData_.gyro_bias[1] = state[15];
            filterData_.gyro_bias[2] = state[16];

            std::vector<double> P;
            for (unsigned int i = 0; i < extendedKalmanFilter->PropagationError().rows(); i++) {
                P.push_back(extendedKalmanFilter->PropagationError().at(i, i));
            }

            filterData_.covariance_estimation_diag = P;
        } else if (filterParams.mode == FilterMode::GroundTruth) {

            auto tNow = std::chrono::system_clock::now();
            long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow.time_since_epoch())).count();
            auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
            auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

            filterData_.stamp.sec = now_stamp_secs;
            filterData_.stamp.nanosec = now_stamp_nanosecs;

            filterData_.inertialframe_linear_position.latlong.latitude = groundTruthData.inertialframe_linear_position.latlong.latitude;
            filterData_.inertialframe_linear_position.latlong.longitude = groundTruthData.inertialframe_linear_position.latlong.longitude;
            filterData_.inertialframe_linear_position.altitude = groundTruthData.inertialframe_linear_position.altitude;
            filterData_.bodyframe_angular_position.roll = groundTruthData.bodyframe_angular_position.roll;
            filterData_.bodyframe_angular_position.pitch = groundTruthData.bodyframe_angular_position.pitch;
            filterData_.bodyframe_angular_position.yaw = groundTruthData.bodyframe_angular_position.yaw;
            filterData_.bodyframe_linear_velocity[0] = groundTruthData.bodyframe_linear_velocity[0];
            filterData_.bodyframe_linear_velocity[1] = groundTruthData.bodyframe_linear_velocity[1];
            filterData_.bodyframe_linear_velocity[2] = groundTruthData.bodyframe_linear_velocity[2];

            filterData_.bodyframe_angular_velocity[0] = groundTruthData.bodyframe_angular_velocity[0];
            filterData_.bodyframe_angular_velocity[1] = groundTruthData.bodyframe_angular_velocity[1];
            filterData_.bodyframe_angular_velocity[2] = groundTruthData.bodyframe_angular_velocity[2];
            filterData_.inertialframe_water_current[0] = groundTruthData.inertialframe_water_current[0];
            filterData_.inertialframe_water_current[1] = groundTruthData.inertialframe_water_current[1];
            filterData_.gyro_bias[0] = groundTruthData.gyro_bias[0];
            filterData_.gyro_bias[1] = groundTruthData.gyro_bias[1];
            filterData_.gyro_bias[2] = groundTruthData.gyro_bias[2];
        }

        navDataPub_->publish(filterData_);

    }

    void NavigationFilter::SensorsCheckCB(){
        std::cout << "Hello, world!" << std::endl;
    }

    bool NavigationFilter::LoadConfiguration(NavigationFilterParams& filterParameters)
    {
        //read conf file
        libconfig::Config confObj;

        //Inizialization
        //std::string confPath = ament_index_cpp::get_package_share_directory("nav_filter").append("/conf/navigation_filter.conf");
        //std::cout << "PATH TO CONF FILE : " << confPath << std::endl;

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

    void NavigationFilter::CommandHandler(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Request> request,
        std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Response> response)
    {
        (void)request_header;
        RCLCPP_INFO(nh_->get_logger(), "Incoming request: %s", CommandTypeToString(static_cast<CommandType>(request->command_type)).c_str());

        CommandAnswer ret = CommandAnswer::ok;

        switch (request->command_type) {
        case static_cast<uint16_t>(CommandType::undefined):
            RCLCPP_WARN(nh_->get_logger(), "CommandType undefined");
            ret = CommandAnswer::fail;
            break;
        case static_cast<uint16_t>(CommandType::reset):
            if (filterParams.mode == FilterMode::LuenbergerObserver) {
                obs.Reset();
                RCLCPP_INFO(nh_->get_logger(), "Reset Luenberger observer");
            } else {
                extendedKalmanFilter->Reset();
                //sample the current gps data anfd yaw
                Eigen::VectorXd initialState = Eigen::VectorXd::Zero(stateDim);
                Eigen::Vector3d NED_currentPosition;
                ctb::LatLong2LocalNED(ctb::LatLong(gpsData.latitude, gpsData.longitude), gpsData.altitude, centroidLocation, NED_currentPosition);
                initialState.segment(0, 2) = NED_currentPosition.segment(0, 2);

                initialState[5] = -atan2(magnetometerData.orthogonalstrength[1] * cos(state[3]) - magnetometerData.orthogonalstrength[2] * sin(state[3]), magnetometerData.orthogonalstrength[0] * cos(state[4]) + magnetometerData.orthogonalstrength[2] * cos(state[3]) * sin(state[4]) + magnetometerData.orthogonalstrength[1] * sin(state[4]) * sin(state[3]));

                extendedKalmanFilter->Init(initialState);
                RCLCPP_INFO(nh_->get_logger(), "Reset EKF");
            }
            break;
        case static_cast<uint16_t>(CommandType::reloadconfig): {
            auto previousFilterParams = filterParams;
            LoadConfiguration(previousFilterParams);
            break;
        }
        default:
            RCLCPP_WARN(nh_->get_logger(), "Unsupported Command Code");
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
