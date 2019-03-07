/*
 * GPSDHelperDataStructs.h
 *
 *  Created on: Nov 01, 2018
 *      Author: wanderfra
 */

#ifndef ULISSE_DRIVER_GPSDHELPERDATASTRUCTS_H_
#define ULISSE_DRIVER_GPSDHELPERDATASTRUCTS_H_

#include <rclcpp/rclcpp.hpp>

#include "ulisse_driver/driver_defines.h"
#include "ulisse_driver/GPSDHelperDefines.h"

namespace ulisse {

namespace gpsd {

enum class GpsStatus : int8_t {
	no_fix = -1,   /* Unable to fix position                            */
	fix = 0,       /* Normal fix                                        */
	sbas_fix = 1,  /* Fixed using a satellite-based augmentation system */
	gbas_fix = 2,  /*          or a ground-based augmentation system    */
	dgps_fix = 28, /* Fixed with DGPS                                   */
	waas_fix = 33  /* Fixed with WAAS                                   */
};

//enum class GpsSource:int8_t {
//	none = 0,      /* No information is available                                  */
//	gps = 1,       /* Using standard GPS location [only valid for position_source] */
//	points = 2,    /* Motion/orientation fix is derived from successive points     */
//	doppler = 4,   /* Motion is derived using the Doppler effect                   */
//	altimeter = 8, /* Using an altimeter                                           */
//	magnetic = 16, /* Using magnetic sensors                                       */
//	gyre = 32,     /* Using gyroscopes                                             */
//	accel = 64     /* Using accelerometers                                         */
//};

enum class GpsFixMode : uint8_t {
	mode_not_seen = 0, /* mode update not seen yet    */
	mode_no_fix = 1,   /* none                        */
	mode_2d = 2,       /* good for latitude/longitude */
	mode_3d = 3        /* good for altitude/climb too */
};

struct GpsStatusData {
	//# Satellites used in solution
	uint16_t satellites_used;                                // Number of satellites
	int32_t satellite_used_prn[constants::maxChannels];      // PRN identifiers

	//# Satellites visible
	uint16_t satellites_visible;
	int32_t satellite_visible_prn[constants::maxChannels];     // PRN identifiers
	int32_t satellite_visible_z[constants::maxChannels];       // Elevation of satellites
	int32_t satellite_visible_azimuth[constants::maxChannels]; // Azimuth of satellites
	int32_t satellite_visible_snr[constants::maxChannels];     // Signal-to-noise ratios (dB)
	bool satellite_visible_used[constants::maxChannels];

	GpsStatus status;

//	GpsSource motion_source;      /* Source for speed, climb and track */
//	GpsSource orientation_source; /* Source for device orientation     */
//	GpsSource position_source;    /* Source for position               */

	std::string tmpString;
    void DebugPrint(rclcpp::Logger logger) {
		switch (status) {
		case GpsStatus::no_fix:
			tmpString = "No Fix";
			break;
		case GpsStatus::fix:
			tmpString = "Fix";
			break;
		case GpsStatus::sbas_fix:
			tmpString = "SBAS Fix";
			break;
		case GpsStatus::gbas_fix:
			tmpString = "GBAS Fix";
			break;
		case GpsStatus::dgps_fix:
			tmpString = "DGPS Fix";
			break;
		case GpsStatus::waas_fix:
			tmpString = "WAAS Fix";
			break;
		}
        RCLCPP_INFO(logger, "%s - %u sats in view, %u used", tmpString.c_str(), satellites_visible, satellites_used);
		for (int i=0 ; i<satellites_visible; ++i ) {
			if (satellite_visible_used[i])
				tmpString = "USED";
			else
				tmpString = "    ";
            RCLCPP_INFO(logger, "%s - prn %d snr %d z %d azimuth %d", tmpString.c_str(), satellite_visible_prn[i], satellite_visible_snr[i], satellite_visible_z[i], satellite_visible_azimuth[i]);
		}
	}
};

struct GpsData {
	float64_t time;       /* Time of update */
	GpsFixMode mode;
	uint64_t flags;
	float64_t latitude;   /* Latitude in degrllc (valid if mode >= 2) */
	float64_t longitude;  /* Longitude in degrllc (valid if mode >= 2) */
	float64_t altitude;   /* Altitude in meters (valid if mode == 3) */
	float64_t track;      /* Course made good (relative to true north) */
	float64_t speed;      /* Speed over ground, meters/sec */
	float64_t climb;      /* Vertical speed, meters/sec */

    float64_t err;           /* spherical position error, 95% confidence (meters)  */
    float64_t err_time;      /* Expected time uncertainty */
    float64_t err_latitude;  /* Latitude position uncertainty, meters */
    float64_t err_longitude; /* Longitude position uncertainty, meters */
    float64_t err_altitude;  /* Vertical position uncertainty, meters */
    float64_t err_track;     /* Track uncertainty, degrllc */
    float64_t err_speed;     /* Speed uncertainty, meters/sec */
    float64_t err_climb;     /* Vertical speed uncertainty */

    float64_t xdop;
    float64_t ydop;
    float64_t gdop; /* Total (positional-temporal) dilution of precision */
    float64_t pdop; /* Positional (3D) dilution of precision */
    float64_t hdop; /* Horizontal dilution of precision */
    float64_t vdop; /* Vertical dilution of precision */
    float64_t tdop; /* Temporal dilution of precision */

    bool CheckFlag(gps_mask_t bit) {
    	return (flags & bit);
    }

    void DebugPrint(rclcpp::Logger logger) {
        RCLCPP_INFO(logger, "-----------------------------------------------------------------------------------------------------------------------");

        RCLCPP_INFO(logger, "time %lf mode %d lat %lf long %lf altitude %lf", time, mode, latitude, longitude, altitude);
        RCLCPP_INFO(logger, "track %lf speed %lf climb %lf", track, speed, climb);
        RCLCPP_INFO(logger, "err %lf err_t %lf err_lat %lf err_lon %lf", err, err_time, err_latitude, err_longitude);
        RCLCPP_INFO(logger, "err_alt %lf err_tr %lf err_sp %lf err_cl %lf", err_altitude, err_track, err_speed, err_climb);
        RCLCPP_INFO(logger, "xdop %lf ydop %lf gdop %lf pdop %lf", xdop, ydop, gdop, pdop);
        RCLCPP_INFO(logger, "hdop %lf vdop %lf tdop %lf", hdop, vdop, tdop);

        RCLCPP_INFO(logger, "set online   %d | time     %d | timerr %d | latlon     %d | altitude %d | speed      %d | track     %d | climb    %d", CheckFlag(ONLINE_SET), CheckFlag(TIME_SET), CheckFlag(TIMERR_SET), CheckFlag(LATLON_SET), CheckFlag(ALTITUDE_SET), CheckFlag(SPEED_SET), CheckFlag(TRACK_SET), CheckFlag(CLIMB_SET));
        RCLCPP_INFO(logger, "set status   %d | mode     %d | dop    %d | herr       %d | verr     %d | attitude   %d | satellite %d | speederr %d", CheckFlag(STATUS_SET), CheckFlag(MODE_SET), CheckFlag(DOP_SET), CheckFlag(HERR_SET), CheckFlag(VERR_SET), CheckFlag(ATTITUDE_SET), CheckFlag(SATELLITE_SET), CheckFlag(SPEEDERR_SET));
        RCLCPP_INFO(logger, "set trackerr %d | climberr %d | device %d | devicelist %d | deviceid %d | rtcm2      %d | rtcm3     %d | ais      %d", CheckFlag(TRACKERR_SET), CheckFlag(CLIMBERR_SET), CheckFlag(DEVICE_SET), CheckFlag(DEVICELIST_SET), CheckFlag(DEVICEID_SET), CheckFlag(RTCM2_SET), CheckFlag(RTCM3_SET), CheckFlag(AIS_SET));
//		RCLCPP_INFO(logger, "set packet   %d | subframe %d | gst    %d | version    %d | policy   %d | logmessage %d | error     %d | toff     %d", CheckFlag(PACKET_SET), CheckFlag(SUBFRAME_SET), CheckFlag(GST_SET), CheckFlag(VERSION_SET), CheckFlag(POLICY_SET), CheckFlag(LOGMESSAGE_SET), CheckFlag(ERROR_SET), CheckFlag(TOFF_SET));
//		RCLCPP_INFO(logger, "set pps      %d | navdata  %d | other  %d", CheckFlag(PPS_SET), CheckFlag(NAVDATA_SET), (flags & (~((1llu<<35)-1)))); //(p->set & (~((1llu<<35)-1)))
        RCLCPP_INFO(logger, "-----------------------------------------------------------------------------------------------------------------------");
    }
};

} //namespace gpsd

} //namespace ulisse

#endif /* ULISSE_DRIVER_GPSDHELPERDATASTRUCTS_H_ */
