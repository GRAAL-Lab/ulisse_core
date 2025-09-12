#include "nav_filter/navigation_filter.hpp"
#include "ulisse_msgs/futils.hpp"
#include <unistd.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


namespace ulisse {

namespace nav {
    NavigationFilter::NavigationFilter(const std::string& confPath)
        : Node("navigation_filter_node")
        , confPath_(confPath)
    {
        stateDim_ = 0;
        centroidLocation_ = ctb::LatLong(44.0956, 9.8631);

        gyroMeasurement_ = std::make_shared<ulisse::nav::GyroMeasurement>(ulisse::nav::GyroMeasurement());
        compassMeasurement_ = std::make_shared<ulisse::nav::CompassMeasurement>(ulisse::nav::CompassMeasurement());
        accelerometerMeasurement_ = std::make_shared<ulisse::nav::AccelerometerMeasurement>(ulisse::nav::AccelerometerMeasurement());
        gpsMeasurement_ = std::make_shared<ulisse::nav::GpsMeasurement>(ulisse::nav::GpsMeasurement());
        magnetometerMeasurement_ = std::make_shared<ulisse::nav::MagnetometerMeasurement>(ulisse::nav::MagnetometerMeasurement());
        zMeterMeasurement_ = std::make_shared<ulisse::nav::zMeter>(ulisse::nav::zMeter());
        portRPMMeasurement_ = std::make_shared<ulisse::nav::RPMMeasurement>(ulisse::nav::RPMMeasurement());
        stbdRPMMeasurement_ = std::make_shared<ulisse::nav::RPMMeasurement>(ulisse::nav::RPMMeasurement());

        portRPMMeasurement_->SetPortStarboard(ulisse::nav::Side::Port);
        stbdRPMMeasurement_->SetPortStarboard(ulisse::nav::Side::Starboard);

             //Load filter params
        if (!LoadConfiguration()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load navigation filter configuration");
            exit(EXIT_FAILURE);
        }

             //Publisher of nav data structure
        navDataPub_ = this->create_publisher<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 1);
        rqtAbsSurgePub_ = this->create_publisher<std_msgs::msg::Float64>("/rqt/abs_surge", 1);
        rqtRelSurgePub_ = this->create_publisher<std_msgs::msg::Float64>("/rqt/rel_surge", 1);
        rqtWaterCurrentXPub_ = this->create_publisher<std_msgs::msg::Float64>("/rqt/water_current_x", 1);
        rqtWaterCurrentYPub_ = this->create_publisher<std_msgs::msg::Float64>("/rqt/water_current_y", 1);
        rqtBiasXPub_ = this->create_publisher<std_msgs::msg::Float64>("/rqt/gyro_bias_x", 1);
        rqtBiasYPub_ = this->create_publisher<std_msgs::msg::Float64>("/rqt/gyro_bias_y", 1);
        rqtBiasZPub_ = this->create_publisher<std_msgs::msg::Float64>("/rqt/gyro_bias_z", 1);
        rqtOmegaXPub_ = this->create_publisher<std_msgs::msg::Float64>("/rqt/omega_x", 1);
        rqtOmegaYPub_ = this->create_publisher<std_msgs::msg::Float64>("/rqt/omega_y", 1);
        rqtOmegaZPub_ = this->create_publisher<std_msgs::msg::Float64>("/rqt/omega_z", 1);
        rqtGyroXPub_ = this->create_publisher<std_msgs::msg::Float64>("/rqt/gyro_x", 1);
        rqtGyroYPub_ = this->create_publisher<std_msgs::msg::Float64>("/rqt/gyro_y", 1);
        rqtGyroZPub_ = this->create_publisher<std_msgs::msg::Float64>("/rqt/gyro_z", 1);
        rqtRPMPortPub_ = this->create_publisher<std_msgs::msg::Float64>("/rqt/n_p", 1);
        rqtRPMStbdPub_ = this->create_publisher<std_msgs::msg::Float64>("/rqt/n_s", 1);

        //Subscribes to data sensors

        //compassSub_ = this->create_subscription<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass,
        //    1, std::bind(&NavigationFilter::CompassDataCB, this, _1));
        imuPoseSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(ulisse_msgs::topicnames::sensor_imu_pose,
            1, std::bind(&NavigationFilter::ImuPoseCB, this, _1));
        gpsdataSub_ = this->create_subscription<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data,
            1, std::bind(&NavigationFilter::GPSDataCB, this, _1));
        //imudataSub_ = this->create_subscription<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu,
        //    1, std::bind(&NavigationFilter::IMUDataCB, this, _1));
        imuSensorSub_ = this->create_subscription<sensor_msgs::msg::Imu>(ulisse_msgs::topicnames::sensor_imu,
            1, std::bind(&NavigationFilter::IMUDataCB, this, _1));
        
        //magnetometerSub_ = this->create_subscription<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer,
        //    1, std::bind(&NavigationFilter::MagnetometerDataCB, this, _1));
        imuMagnetometerSub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(ulisse_msgs::topicnames::sensor_magnetometer,
            1, std::bind(&NavigationFilter::ImuMagnetometerCB, this, _1));
            
        simulatedSystemSub_ = this->create_subscription<ulisse_msgs::msg::SimulatedSystem>(ulisse_msgs::topicnames::simulated_system,
            1, std::bind(&NavigationFilter::GroundTruthDataCB, this, _1));
        thrustersAppliedRefSub_ = this->create_subscription<ulisse_msgs::msg::ThrustersReference>(ulisse_msgs::topicnames::llc_thrusters_applied_perc,
            1, std::bind(&NavigationFilter::ThrustersAppliedReferenceCB, this, _1));
        llcThrustersSub_ = this->create_subscription<ulisse_msgs::msg::LLCThrusters>(ulisse_msgs::topicnames::llc_thrusters,
            1, std::bind(&NavigationFilter::LLCThrustersCB, this, _1));

        //simulatedVelocitySub_ = this->create_subscription<ulisse_msgs::msg::SimulatedVelocitySensor>(ulisse_msgs::topicnames::simulated_velocity_sensor,
        //    1, std::bind(&NavigationFilter::SimulatedVelocitySensorCB, this, _1));

        USE_GPS.data = true;
        USE_GPS_Sub = this->create_subscription<std_msgs::msg::Bool>("/ulisse/USE_GPS",
            1, std::bind(&NavigationFilter::USE_GPS_CB, this, _1));

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
        // Check for first measurement
        isFirst_ = true;
        state_ = Eigen::VectorXd::Zero(stateDim_);

        last_comp_time_ = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        sensorsCheckInterval_ = 5;
        sensorsCheckTimer_ = this->create_wall_timer(std::chrono::seconds(sensorsCheckInterval_),
            std::bind(&NavigationFilter::SensorsOnlineCB, this));

        int msRunPeriod = 1.0 / (filterParams_.rate) * 1000;
        RCLCPP_INFO(this->get_logger(), "NavFilter Rate: %d Hz", filterParams_.rate);
        runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&NavigationFilter::Run, this));

             // Service
        navFilterCmdService_ = this->create_service<ulisse_msgs::srv::NavFilterCommand>(ulisse_msgs::topicnames::navfilter_cmd_service,
            std::bind(&NavigationFilter::CommandHandler, this, _1, _2, _3));

        RCLCPP_INFO(this->get_logger(), "Thrusters references FIFO length: %d", filterParams_.thrusterFIFOdelayLength);
        for (unsigned int i = 0; i < filterParams_.thrusterFIFOdelayLength; i++) {
            fifo_h_p_.push(0.0);
            fifo_h_s_.push(0.0);
        }
    }

    NavigationFilter::~NavigationFilter() { }

    void NavigationFilter::Run()
    {

        SensorsValidityCheck();

        if (filterParams_.mode == FilterMode::LuenbergerObserver) {
            LuenbergerObserverFilter();
        } else if (filterParams_.mode == FilterMode::KalmanFilter) {
            ExtendedKalmanFilter();
        } else if (filterParams_.mode == FilterMode::GroundTruth) {
            GroundThruthFilter();
        }

        navDataPub_->publish(filterData_);

             //auto tNow = std::chrono::system_clock::now();
             //long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow.time_since_epoch())).count();
             //auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
             //auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
        std_msgs::msg::Float64 msg;
        //msg.stamp.sec = now_stamp_secs;
        //msg.stamp.nanosec = now_stamp_nanosecs;

        Eigen::Vector3d bodyframe_velocity = { filterData_.bodyframe_linear_velocity[0], filterData_.bodyframe_linear_velocity[1], filterData_.bodyframe_linear_velocity[2] };
        Eigen::Vector3d inertialframe_current = { filterData_.inertialframe_water_current[0], filterData_.inertialframe_water_current[1], 0 };
        //rml::EulerRPY rpy { filterData_.bodyframe_angular_position.roll, filterData_.bodyframe_angular_position.pitch, filterData_.bodyframe_angular_position.yaw };


        rml::EulerRPY rpy;

        if (ulisseModelVersion_ == ASVModelVersion::SimplifiedCoMFrame){
            rpy.RPY(0, 0, filterData_.bodyframe_angular_position.yaw);
        } else {
            rpy.RPY(filterData_.bodyframe_angular_position.roll, filterData_.bodyframe_angular_position.pitch, filterData_.bodyframe_angular_position.yaw);
        }

        Eigen::Vector3d bodyF_absVelocity = rpy.ToRotationMatrix().transpose() * inertialframe_current + bodyframe_velocity;

        msg.data = bodyF_absVelocity[0];
        rqtAbsSurgePub_->publish(msg);

        msg.data = filterData_.bodyframe_linear_velocity[0];
        rqtRelSurgePub_->publish(msg);

        msg.data = filterData_.inertialframe_water_current[0];
        rqtWaterCurrentXPub_->publish(msg);
        msg.data = filterData_.inertialframe_water_current[1];
        rqtWaterCurrentYPub_->publish(msg);

        msg.data = filterData_.gyro_bias[0];
        rqtBiasXPub_->publish(msg);
        msg.data = filterData_.gyro_bias[1];
        rqtBiasYPub_->publish(msg);
        msg.data = filterData_.gyro_bias[2];
        rqtBiasZPub_->publish(msg);

        msg.data = filterData_.bodyframe_angular_velocity[0];
        rqtOmegaXPub_->publish(msg);
        msg.data = filterData_.bodyframe_angular_velocity[1];
        rqtOmegaYPub_->publish(msg);
        msg.data = filterData_.bodyframe_angular_velocity[2];
        rqtOmegaZPub_->publish(msg);

        msg.data = imuData_.angular_velocity.x - filterData_.gyro_bias[0];
        rqtGyroXPub_->publish(msg);
        msg.data = imuData_.angular_velocity.y - filterData_.gyro_bias[1];
        rqtGyroYPub_->publish(msg);
        msg.data = imuData_.angular_velocity.z - filterData_.gyro_bias[2];
        rqtGyroZPub_->publish(msg);

        msg.data = state_[17];
        rqtRPMPortPub_->publish(msg);
        msg.data = state_[18];
        rqtRPMStbdPub_->publish(msg);
    }

    /*void NavigationFilter::LuenbergerObserverFilter()
    {
        if (gpsValid_) {

            int zone;
            bool northp;
            Eigen::Vector2d p_utm;

            try {
                GeographicLib::UTMUPS::Forward(gpsData_.latitude, gpsData_.longitude, zone, northp, p_utm.x(), p_utm.y());

                // The geographic lib conversion outputs UTM coordinates but the filter uses NED.
                Eigen::Vector2d p_ned = { p_utm.y(), p_utm.x() };

                Eigen::Quaterniond poseQ(imuPose_.pose.orientation.w, imuPose_.pose.orientation.x, imuPose_.pose.orientation.y, imuPose_.pose.orientation.z);
                auto compassRPY = rml::EulerRPY(poseQ);

                if (filterEnable_) {

                    
                    obs_.Update(Eigen::Vector4d { p_ned.x(), p_ned.y(), compassRPY.Yaw(), simulatedVelocitySensor_.water_relative_surge });
                    filterData_.inertialframe_water_current.fill(0.0);

                         //Construct the inertial to body frame rotation
                    rml::EulerRPY rpy { 0.0, 0.0, compassRPY.Yaw() };
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
                

                filterData_.bodyframe_angular_position.roll = compassRPY.Roll();
                filterData_.bodyframe_angular_position.roll = compassRPY.Pitch();
                filterData_.bodyframe_angular_position.roll = compassRPY.Yaw();


                filterData_.bodyframe_angular_velocity[0] = imuData_.angular_velocity.x;
                filterData_.bodyframe_angular_velocity[1] = imuData_.angular_velocity.y;

                     //Yaw rate estimation with a digital filter
                double omega_dot_dot = ctb::AngleDifference(compassRPY.Yaw(), previousYaw_) / sampleTime_;
                previousYaw_ = compassRPY.Yaw();

                filterData_.bodyframe_angular_velocity[2] = yawRateFilterGains_[0] * filterData_.bodyframe_angular_velocity[2] + yawRateFilterGains_[1] * omega_dot_dot;

            } catch (const GeographicLib::GeographicErr& e) {
                RCLCPP_ERROR(this->get_logger(), "GeographicLib exception: what = %s", e.what());
                obs_.Reset();
            }
        }
    }*/

    void NavigationFilter::ExtendedKalmanFilter()
    {
        // Added USE_GPS boolean to deliberately ignore the GPS
        if (gpsValid_ && USE_GPS.data) {
            if (measuresActive_.find("gps")->second) {

                if (isFirst_) {
                    // Eigen::VectorXd initialState = Eigen::VectorXd::Zero(stateDim_);

                    ctb::LatLong2LocalNED(ctb::LatLong(gpsData_.latitude, gpsData_.longitude), gpsData_.altitude, centroidLocation_, NED_gps_cartesian_);

                    auto initialState = extendedKalmanFilter_->StateVector();
                    initialState.segment(0, 2) << NED_gps_cartesian_.x(), NED_gps_cartesian_.y();
                    extendedKalmanFilter_->Init(initialState);
                    isFirst_ = false;
                }

                     // The filter use the cartesian coordinates
                ctb::LatLong2LocalNED(ctb::LatLong(gpsData_.latitude, gpsData_.longitude), gpsData_.altitude, centroidLocation_, NED_gps_cartesian_);
                gpsMeasurement_->MeasureVector() = Eigen::Vector2d { NED_gps_cartesian_.x(), NED_gps_cartesian_.y() };
                extendedKalmanFilter_->AddMeasurement(gpsMeasurement_);
            }
        }

        if (imuValid_) {
            if (measuresActive_.find("gyro")->second) {
                // Verify !TODO!
                gyroMeasurement_->MeasureVector() = Eigen::Vector3d { imuData_.angular_velocity.x, imuData_.angular_velocity.y, imuData_.angular_velocity.z };
                extendedKalmanFilter_->AddMeasurement(gyroMeasurement_);
            }
            if (measuresActive_.find("accelerometer")->second) {
                accelerometerMeasurement_->MeasureVector() = Eigen::Vector3d { imuData_.linear_acceleration.x, imuData_.linear_acceleration.y, imuData_.linear_acceleration.z };
                extendedKalmanFilter_->AddMeasurement(accelerometerMeasurement_);
            }
        }

        if (compassValid_) {
            if (measuresActive_.find("compass")->second) {
                Eigen::Quaterniond poseQ(imuPose_.pose.orientation.w, imuPose_.pose.orientation.x, imuPose_.pose.orientation.y, imuPose_.pose.orientation.z);
                rml::EulerRPY orientationRPY = rml::EulerRPY(poseQ);
                compassMeasurement_->MeasureVector() = Eigen::Vector3d { orientationRPY.Roll(), orientationRPY.Pitch(), orientationRPY.Yaw() };
                extendedKalmanFilter_->AddMeasurement(compassMeasurement_);
            }
        }

        if (magnetometerValid_) {
            if (measuresActive_.find("magnetometer")->second) {
                // preprocessing: I compensate for the roll and the pitch and then I pretend to have a sensor that measures the yaw
                magnetometerMeasurement_->MeasureVector() << -atan2(imuMagnetometer_.magnetic_field.y * cos(state_[3]) - imuMagnetometer_.magnetic_field.z * sin(state_[3]), imuMagnetometer_.magnetic_field.x * cos(state_[4]) + imuMagnetometer_.magnetic_field.z * cos(state_[3]) * sin(state_[4]) + imuMagnetometer_.magnetic_field.y * sin(state_[4]) * sin(state_[3]));
                extendedKalmanFilter_->AddMeasurement(magnetometerMeasurement_);
            }
        }

        if (leftRPMValid_) {
            if (measuresActive_.find("rpm")->second) {
                portRPMMeasurement_->MeasureVector() << llcThrustersData_.left.motor_speed;
                extendedKalmanFilter_->AddMeasurement(portRPMMeasurement_);
            }
        }

        if (rightRPMValid_) {
            if (measuresActive_.find("rpm")->second) {
                stbdRPMMeasurement_->MeasureVector() << llcThrustersData_.right.motor_speed;
                extendedKalmanFilter_->AddMeasurement(stbdRPMMeasurement_);
            }
        }

             //RCLCPP_INFO(this->get_logger(), "EFK measurement: left rpm %d - right rpm %d", leftRPMValid_, rightRPMValid_);

             //Added a perfect com altitude meter to be coherent with the real data that recod 0.0 as altitude
        if (measuresActive_.find("zMeter")->second) {
            Eigen::Vector3d NED_p;
            zMeterMeasurement_->MeasureVector() << 0.0;
            extendedKalmanFilter_->AddMeasurement(zMeterMeasurement_);
        }

             //RCLCPP_INFO(this->get_logger(), "EFK measurement: imu %d - gps %d - magnetometer %d", imuValid_, gpsValid_, magnetometerValid_);

             //Filter Update
        fifo_h_p_.push(thrustersPercReference_.left_percentage);
        fifo_h_s_.push(thrustersPercReference_.right_percentage);

             //std::cerr << "GPS="<<gpsValid_<< " IMU="<<imuValid_ << " MAG="<<magnetometerValid_<< " LRPM="<<leftRPMValid_ << " RRPM="<<rightRPMValid_ << std::endl;
             //extendedKalmanFilter_->Update(Eigen::Vector2d { thrustersFbk_.left_percentage, thrustersFbk_.right_percentage });
        extendedKalmanFilter_->Update(Eigen::Vector2d { fifo_h_p_.front(), fifo_h_s_.front() });
        fifo_h_p_.pop();
        fifo_h_s_.pop();
        //extendedKalmanFilter_->Update(Eigen::Vector2d { llcThrustersData_.left.motor_speed, llcThrustersData_.right.motor_speed });

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
        filterData_.n_p = state_[17];
        filterData_.n_s = state_[18];

        std::vector<double> P;
        for (unsigned int i = 0; i < extendedKalmanFilter_->PropagationError().rows(); i++) {
            P.push_back(extendedKalmanFilter_->PropagationError().at(i, i));
        }

        filterData_.covariance_estimation_diag = P;
    }

    void NavigationFilter::GroundThruthFilter()
    {
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

    void NavigationFilter::SensorsValidityCheck()
    {

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

        if (imuData_.header.stamp.sec + (imuData_.header.stamp.nanosec * 1e-9) > lastValidImuTime_) {
            imuValid_ = true;
            filterData_.imu_received = true;
            lastValidImuTime_ = imuData_.header.stamp.sec + (imuData_.header.stamp.nanosec * 1e-9);

        } else {
            imuValid_ = false;
        }

        /*RCLCPP_INFO(this->get_logger(), "SensorsValidityCheck(): imuValid=%d, stamp.sec:%ld, stamp.nanosec:%ld, stamp_calculated:%lf",
            imuValid_, imuData_.stamp.sec, imuData_.stamp.nanosec, imuData_.stamp.sec + (imuData_.stamp.nanosec * 1e-9));*/

        if (imuPose_.header.stamp.sec + (imuPose_.header.stamp.nanosec * 1e-9) > lastValidCompassTime_) {
            compassValid_ = true;
            filterData_.compass_received = true;
            lastValidCompassTime_ = imuPose_.header.stamp.sec + (imuPose_.header.stamp.nanosec * 1e-9);
        } else {
            compassValid_ = false;
        }

        if (imuMagnetometer_.header.stamp.sec + (imuMagnetometer_.header.stamp.nanosec * 1e-9) > lastValidMagnetomerTime_) {
            magnetometerValid_ = true;
            filterData_.magnetometer_received = true;
            lastValidMagnetomerTime_ = imuMagnetometer_.header.stamp.sec + (imuMagnetometer_.header.stamp.nanosec * 1e-9);
        } else {
            magnetometerValid_ = false;
        }

        if (llcThrustersData_.left.timestamp_485 > lastValidLeftRPMTime_) {
            leftRPMValid_ = true;
            lastValidLeftRPMTime_ = llcThrustersData_.left.timestamp_485;
        } else {
            leftRPMValid_ = false;
        }

        if (llcThrustersData_.right.timestamp_485 > lastValidRightRPMTime_) {
            rightRPMValid_ = true;
            lastValidRightRPMTime_ = llcThrustersData_.right.timestamp_485;
        } else {
            rightRPMValid_ = false;
        }
    }

    void NavigationFilter::SensorsOnlineCB()
    {
        auto t_now = std::chrono::system_clock::now();
        auto timeNowSecs = (std::chrono::duration_cast<std::chrono::seconds>(t_now.time_since_epoch())).count();
        auto lastValidGPSSecs = static_cast<std::time_t>(lastValidGPSTime_);

        if (filterData_.gps_received) {
            if (std::abs(lastValidGPSSecs - timeNowSecs) > sensorsCheckInterval_) {
                RCLCPP_WARN(this->get_logger(), "GPS Data unavailable for more than %i seconds.", sensorsCheckInterval_);
                RCLCPP_WARN(this->get_logger(), "GPS Data last valid %ld, now %ld, diff %ld", lastValidGPSSecs, timeNowSecs, std::abs(lastValidGPSSecs - timeNowSecs));
                filterData_.gps_received = false;

                if (std::ctime(&timeNowSecs) != nullptr) {
                    std::string timedate_cpu = std::ctime(&timeNowSecs);
                    timedate_cpu.erase(std::remove(timedate_cpu.begin(), timedate_cpu.end(), '\n'), timedate_cpu.end());
                    RCLCPP_INFO(this->get_logger(), "CPU Time now: %s", timedate_cpu.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "CPU Time now = nullptr");
                }

                if (std::ctime(&lastValidGPSSecs) != nullptr) {
                    std::string timedate_gps = std::ctime(&lastValidGPSSecs);
                    timedate_gps.erase(std::remove(timedate_gps.begin(), timedate_gps.end(), '\n'), timedate_gps.end());
                    RCLCPP_INFO(this->get_logger(), "GPS Time now: %s", timedate_gps.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "GPS Time now = nullptr");
                }
            }
        }

        if (filterData_.imu_received){
            if (std::abs(imuData_.header.stamp.sec - timeNowSecs) > sensorsCheckInterval_) {
                RCLCPP_WARN(this->get_logger(), "IMU Data unavailable for more than %i seconds.", sensorsCheckInterval_);
                RCLCPP_WARN(this->get_logger(), "IMU Data last valid %u, now %ld, diff %ld", imuData_.header.stamp.sec, timeNowSecs, std::abs(imuData_.header.stamp.sec - timeNowSecs));
                filterData_.imu_received = false;
            }
        }

        if (filterData_.compass_received){
            if (std::abs(imuPose_.header.stamp.sec - timeNowSecs) > sensorsCheckInterval_) {
                RCLCPP_WARN(this->get_logger(), "Compass Data unavailable for more than %i seconds.", sensorsCheckInterval_);
                RCLCPP_WARN(this->get_logger(), "Compass Data last valid %u, now %ld, diff %ld", imuPose_.header.stamp.sec, timeNowSecs, std::abs(imuPose_.header.stamp.sec - timeNowSecs));
                filterData_.compass_received = false;
            }
        }

        if (filterData_.magnetometer_received){
            if (std::abs(imuMagnetometer_.header.stamp.sec - timeNowSecs) > sensorsCheckInterval_) {
                RCLCPP_WARN(this->get_logger(), "Magnetometer Data unavailable for more than %i seconds.", sensorsCheckInterval_);
                RCLCPP_WARN(this->get_logger(), "Magnetometer Data last valid %u, now %ld, diff %ld", imuMagnetometer_.header.stamp.sec, timeNowSecs, std::abs(imuMagnetometer_.header.stamp.sec - timeNowSecs));
                filterData_.magnetometer_received = false;
            }
        }
    }

    bool NavigationFilter::LoadConfiguration()
    {
        libconfig::Config confObj;

             // Read the ULISSE MODEL config file
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("surface_vehicle_model");
        std::string modelConfPath = package_share_directory + "/conf/ulisse_model.conf";

        RCLCPP_INFO(this->get_logger(), "PATH TO ULISSE_MODEL CONF FILE (NAV): %s", modelConfPath.c_str());
        try {
            confObj.readFile(modelConfPath.c_str());
        } catch (libconfig::ParseException& e) {
            RCLCPP_ERROR(this->get_logger(), "Parse exception when reading: %s", modelConfPath.c_str());
            RCLCPP_ERROR(this->get_logger(), "Line: %d - Error: %s", e.getLine(), e.getError());
            return false;
        }

        SurfaceVehicleModelParameters ulisseModelParams;
        if (!ulisseModelParams.LoadConfiguration(confObj)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load ULISSE model");
            return false;
        }

             // Read the NAV_FILTER config file
        try {
            confObj.readFile(confPath_.c_str());
        } catch (const libconfig::FileIOException& fioex) {
            RCLCPP_ERROR(this->get_logger(), "I/O error while reading file: %s", fioex.what());
            return false;
        } catch (libconfig::ParseException& e) {
            RCLCPP_ERROR(this->get_logger(), "Parse exception when reading: %s", confPath_.c_str());
            RCLCPP_ERROR(this->get_logger(), "Line: %d - Error: %s", e.getLine(), e.getError());
            return false;
        } catch (libconfig::SettingException& e) {
            std::cerr << "ACSADASDA" << std::endl;
            RCLCPP_ERROR(this->get_logger(), "Setting error while reading %s: %s", e.getPath(), e.what());
        }

             // Configure the filter node params
        if (!filterParams_.ConfigureFromFile(confObj)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load navigation mode/rate params");
            return false;
        };

        if (filterParams_.mode == FilterMode::LuenbergerObserver) {
            if (!LuenbergerObserverConfiguration(confObj)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load Luenberger Observer configuration");
                return false;
            };

        } else if (filterParams_.mode == FilterMode::KalmanFilter) {
            if (!KalmanFilterConfiguration(confObj)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load Kalman Filter configuration");
                return false;
            }
            ulisseModelEKF_->ModelParameters() = ulisseModelParams;
        } else if (filterParams_.mode == FilterMode::GroundTruth) {
        } else {
            RCLCPP_ERROR(this->get_logger(), "Type of filter not recognized");
            return false;
        }

        return true;
    }

    bool NavigationFilter::KalmanFilterConfiguration(libconfig::Config& confObj) noexcept(false)
    {

        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& ekf = root["extendedKalmanFilter"];

        int version;
        if (!ctb::GetParam(ekf, version, "version")) {
            RCLCPP_ERROR(this->get_logger(), "Kalman Filter: Failed to load the model version");
            return false;
        }


        if (version >= 0 && version < ASVModelVersion::num_versions) {
            ulisseModelVersion_ = static_cast<ASVModelVersion>(version);
            std::cout << tc::cyanL <<  "[NavFilter] Model: " << ASVModelVersion2String.at(ulisseModelVersion_) << tc::none << std::endl;
        } else {
            std::cerr << tc::redL << "Unrecognized UlisseVehicleModel Version!" << tc::none << std::endl;
            return false;
        }
        ulisseModelEKF_ = std::make_shared<UlisseVehicleModel>(UlisseVehicleModel(ulisseModelVersion_));
        std::vector<int> indexAngles = { 3, 4, 5 }; //rpy
        extendedKalmanFilter_ = std::make_shared<ctb::ExtendedKalmanFilter>(ctb::ExtendedKalmanFilter(stateDim_, indexAngles, ulisseModelEKF_));

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

        const libconfig::Setting& rpmSetting = measure["rpm"];
        if (!ctb::GetParam(rpmSetting, isActive, "enable"))
            return false;

        measuresActive_.insert(std::make_pair("rpm", isActive));
        if (!ctb::GetParamVector(rpmSetting, covarianceDiag, "covariance"))
            return false;

        portRPMMeasurement_->Covariance().diagonal() = covarianceDiag;
        stbdRPMMeasurement_->Covariance().diagonal() = covarianceDiag;
        // Load the initial state and covariance and the model covariance

             // State dimension
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
            //ResetFilter();
            //LoadConfiguration(filterParams_);
            break;
        case static_cast<uint16_t>(CommandType::reloadconfig): {
            //auto previousFilterParams = filterParams_;
            LoadConfiguration();
            //ResetFilter();
            isFirst_ = true;
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

    void NavigationFilter::ResetFilter()
    {
        if (filterParams_.mode == FilterMode::LuenbergerObserver) {
            obs_.Reset();
            RCLCPP_INFO(this->get_logger(), "Reset Luenberger observer");
        } else {
            extendedKalmanFilter_->Reset();
            //sample the current gps data and yaw
            Eigen::VectorXd initialState = Eigen::VectorXd::Zero(stateDim_);
            Eigen::Vector3d NED_currentPosition;
            ctb::LatLong2LocalNED(ctb::LatLong(gpsData_.latitude, gpsData_.longitude), gpsData_.altitude, centroidLocation_, NED_currentPosition);
            initialState.segment(0, 2) = NED_currentPosition.segment(0, 2);
            initialState[5] = -atan2(imuMagnetometer_.magnetic_field.y * cos(state_[3]) - imuMagnetometer_.magnetic_field.z * sin(state_[3]), imuMagnetometer_.magnetic_field.x * cos(state_[4]) + imuMagnetometer_.magnetic_field.z * cos(state_[3]) * sin(state_[4]) + imuMagnetometer_.magnetic_field.y * sin(state_[4]) * sin(state_[3]));

            extendedKalmanFilter_->Init(initialState);
            RCLCPP_INFO(this->get_logger(), "Reset EKF");
        }
    }

    //void NavigationFilter::CompassDataCB(const ulisse_msgs::msg::Compass::SharedPtr msg) { compassData_ = *msg; }
    void NavigationFilter::ImuPoseCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { imuPose_ = *msg; }

    void NavigationFilter::GPSDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg) { gpsData_ = *msg; }

    void NavigationFilter::IMUDataCB(const sensor_msgs::msg::Imu::SharedPtr msg) { imuData_ = *msg; /*RCLCPP_INFO(this->get_logger(), "IMU Callback()");*/ }

    //void NavigationFilter::MagnetometerDataCB(const ulisse_msgs::msg::Magnetometer::SharedPtr msg) { magnetometerData_ = *msg; }
    void NavigationFilter::ImuMagnetometerCB(const sensor_msgs::msg::MagneticField::SharedPtr msg) {imuMagnetometer_ = *msg;};

    void NavigationFilter::SimulatedVelocitySensorCB(const ulisse_msgs::msg::SimulatedVelocitySensor::SharedPtr msg) { simulatedVelocitySensor_ = *msg; }

    void NavigationFilter::ThrustersAppliedReferenceCB(const ulisse_msgs::msg::ThrustersReference::SharedPtr msg) { thrustersPercReference_ = *msg; }

    void NavigationFilter::GroundTruthDataCB(const ulisse_msgs::msg::SimulatedSystem::SharedPtr msg) { simulatedData_ = *msg; }

    void NavigationFilter::LLCThrustersCB(const ulisse_msgs::msg::LLCThrusters::SharedPtr msg) { llcThrustersData_ = *msg; }

    void NavigationFilter::USE_GPS_CB(const std_msgs::msg::Bool::SharedPtr msg) {
        USE_GPS = *msg;
        std::cout << tc::yellow << "*** GPS Input " << (USE_GPS.data ? "Enabled" : "Disabled") << "! ***" << tc::none << std::endl;
    }
}
}
