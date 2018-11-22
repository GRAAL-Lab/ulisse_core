/*
 * taskEESHelper.cc
 *
 *  Created on: Jun 27, 2016
 *      Author: wonder
 */

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <thread>

#include "ulisse_driver/GPSDHelperDataStructs.h"

#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/gps_status.hpp"
#include "ulisse_msgs/msg/time.hpp"
#include "ulisse_msgs/topicnames.hpp"

using namespace ulisse::gpsd;
using namespace std::chrono_literals;

void process_data(gpsData& my_gps_data, struct gps_data_t* p);
void process_status(gpsStatus& my_gps_status, struct gps_data_t* p);
bool step(gpsData& my_gps_data, gpsStatus& my_gps_status, gpsmm* gps);
void GPSData2ROSMsg(const gpsData& my_gps_data, ulisse_msgs::msg::GPSData& gps_data_msg);
void GPSData2ROSMsg(const gpsStatus& my_gps_data, ulisse_msgs::msg::GPSStatus& gps_data_msg);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("gps_publisher_node");

    gpsmm* gps;
    gps_data_t* resp = NULL;

    std::string host = "localhost";
    int port = 2947;

    char port_s[12];
    snprintf(port_s, 12, "%d", port);

#if GPSD_API_MAJOR_VERSION >= 5
    gps = new gpsmm(host.c_str(), port_s);
    resp = gps->stream(WATCH_ENABLE);
#elif GPSD_API_MAJOR_VERSION == 4
    gps = new gpsmm();
    gps->open(host.c_str(), port_s);
    resp = gps->stream(WATCH_ENABLE);
#else
    gps = new gpsmm();
    resp = gps->open(host.c_str(), port_s);
    gps->query("w\n");
#endif

    if (resp == NULL) {
        RCLCPP_ERROR(nh->get_logger(), "Failed to open GPSd");
        return -1;
    } else {
        RCLCPP_INFO(nh->get_logger(), "Successfully opened GPSd");
    }

    gpsData gpsdData;
    gpsStatus gpsdStatus;

    ulisse_msgs::msg::GPSData gps_data_msg;
    ulisse_msgs::msg::GPSStatus gps_status_msg;

    auto gps_data_pub = nh->create_publisher<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data);
    auto gps_status_pub = nh->create_publisher<ulisse_msgs::msg::GPSStatus>(ulisse_msgs::topicnames::sensor_gps_status);

    auto t_now = std::chrono::system_clock::now();
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now.time_since_epoch())).count();

    ulisse_msgs::msg::Time time_now_msg;
    time_now_msg.sec = static_cast<unsigned int>(now_nanosecs / (int)1E9);
    time_now_msg.nanosec = static_cast<unsigned int>(now_nanosecs % (int)1E9);

    while (rclcpp::ok()) {

        if (step(gpsdData, gpsdStatus, gps)) {

            GPSData2ROSMsg(gpsdData, gps_data_msg);
            GPSData2ROSMsg(gpsdStatus, gps_status_msg);

            gps_data_pub->publish(gps_data_msg);
            gps_status_pub->publish(gps_status_msg);
        }

        rclcpp::spin_some(nh);
    }

    rclcpp::shutdown();

    return 0;
}

void process_data(gpsData& my_gps_data, struct gps_data_t* p)
{
    if (p == NULL) {
        return;
    }

    if (!p->online) {
        return;
    }

    my_gps_data.time = p->fix.time;
    my_gps_data.mode = (GpsFixMode)p->fix.mode;
    my_gps_data.flags = p->set;

    if (my_gps_data.CheckFlag(LATLON_SET)) {
        my_gps_data.latitude = p->fix.latitude;
        my_gps_data.longitude = p->fix.longitude;
    }

    if (my_gps_data.CheckFlag(ALTITUDE_SET)) {
        my_gps_data.altitude = p->fix.altitude;
    }

    if (my_gps_data.CheckFlag(TRACK_SET)) {
        my_gps_data.track = p->fix.track;
    }

    if (my_gps_data.CheckFlag(SPEED_SET)) {
        my_gps_data.speed = p->fix.speed;
    }

    if (my_gps_data.CheckFlag(CLIMB_SET)) {
        my_gps_data.climb = p->fix.climb;
    }

    my_gps_data.err = p->epe;
    my_gps_data.err_time = p->fix.ept;
    my_gps_data.err_latitude = p->fix.epy;
    my_gps_data.err_longitude = p->fix.epx;
    my_gps_data.err_altitude = p->fix.epv;
    my_gps_data.err_track = p->fix.epd;
    my_gps_data.err_speed = p->fix.eps;
    my_gps_data.err_climb = p->fix.epc;

    my_gps_data.xdop = p->dop.xdop;
    my_gps_data.ydop = p->dop.ydop;
    my_gps_data.gdop = p->dop.gdop;
    my_gps_data.pdop = p->dop.pdop;
    my_gps_data.hdop = p->dop.hdop;
    my_gps_data.vdop = p->dop.vdop;
    my_gps_data.tdop = p->dop.tdop;
}

void process_status(gpsStatus& my_gps_status, struct gps_data_t* p)
{
    if (p == NULL) {
        return;
    }

    if (!p->online) {
        return;
    }

    my_gps_status.status = (GpsStatus)p->status;

    my_gps_status.satellites_used = p->satellites_used;

    my_gps_status.satellites_visible = SATS_VISIBLE;
    uint16_t used = 0;

    for (int i = 0; i < my_gps_status.satellites_visible; ++i) {
#if GPSD_API_MAJOR_VERSION > 5
        my_gps_status.satellite_visible_prn[i] = p->skyview[i].PRN;
        my_gps_status.satellite_visible_z[i] = p->skyview[i].elevation;
        my_gps_status.satellite_visible_azimuth[i] = p->skyview[i].azimuth;
        my_gps_status.satellite_visible_snr[i] = p->skyview[i].ss;
        my_gps_status.satellite_visible_used[i] = p->skyview[i].used;

        if (p->skyview[i].used) {
            my_gps_status.satellite_used_prn[used++] = p->skyview[i].PRN;
        }

#else
        my_gps_status.satellite_visible_prn[i] = p->PRN[i];
        my_gps_status.satellite_visible_z[i] = p->elevation[i];
        my_gps_status.satellite_visible_azimuth[i] = p->azimuth[i];
        my_gps_status.satellite_visible_snr[i] = p->ss[i];
        my_gps_status.satellite_visible_used[i] = p->used[i];

        if (p->used[i]) {
            my_gps_status.satellite_used_prn[used++] = p->PRN[i];
        }
#endif
    }
}

bool step(gpsData& my_gps_data, gpsStatus& my_gps_status, gpsmm* gps)
{
#if GPSD_API_MAJOR_VERSION >= 5
    if (!gps->waiting(1e6)) {
        return false;
    }
    gps_data_t* p = gps->read();
    if (p == NULL) {
        return false;
    }
#else
    gps_data_t* p = gps->poll();
#endif

    process_data(my_gps_data, p);
    process_status(my_gps_status, p);
    return true;
}

void GPSData2ROSMsg(const gpsData& gps_data, ulisse_msgs::msg::GPSData& gps_data_msg)
{
    gps_data_msg.time = gps_data.time;
    gps_data_msg.gpsfixmode = static_cast<uint8_t>(gps_data.mode);
    gps_data_msg.flags = gps_data.flags;
    gps_data_msg.latitude = gps_data.latitude;
    gps_data_msg.longitude = gps_data.longitude;
    gps_data_msg.altitude = gps_data.altitude;
    gps_data_msg.track = gps_data.track;
    gps_data_msg.speed = gps_data.speed;
    gps_data_msg.climb = gps_data.climb;

    gps_data_msg.err = gps_data.err;
    gps_data_msg.err_time = gps_data.err_time;
    gps_data_msg.err_latitude = gps_data.err_latitude;
    gps_data_msg.err_longitude = gps_data.err_longitude;
    gps_data_msg.err_altitude = gps_data.err_altitude;
    gps_data_msg.err_track = gps_data.err_track;
    gps_data_msg.err_speed = gps_data.err_speed;
    gps_data_msg.err_climb = gps_data.err_climb;

    gps_data_msg.xdop = gps_data.xdop;
    gps_data_msg.ydop = gps_data.ydop;
    gps_data_msg.gdop = gps_data.gdop;
    gps_data_msg.pdop = gps_data.pdop;
    gps_data_msg.hdop = gps_data.hdop;
    gps_data_msg.vdop = gps_data.vdop;
    gps_data_msg.tdop = gps_data.tdop;
}

void GPSData2ROSMsg(const gpsStatus& g_d, ulisse_msgs::msg::GPSStatus& g_m)
{
    g_m.status = static_cast<int8_t>(g_d.status);
    g_m.satellites_used = g_d.satellites_used; // Number of satellites
    g_m.satellites_visible = g_d.satellites_visible;

    std::copy(g_d.satellite_used_prn, g_d.satellite_used_prn + constants::maxChannels,
        g_m.satellite_used_prn.begin()); // PRN identifiers
    std::copy(g_d.satellite_visible_prn, g_d.satellite_visible_prn + constants::maxChannels,
        g_m.satellite_visible_prn.begin()); // PRN identifiers
    std::copy(g_d.satellite_visible_z, g_d.satellite_visible_z + constants::maxChannels,
        g_m.satellite_visible_z.begin()); // Elevation of satellites
    std::copy(g_d.satellite_visible_azimuth, g_d.satellite_visible_azimuth + constants::maxChannels,
        g_m.satellite_visible_azimuth.begin()); // Azimuth of satellites
    std::copy(g_d.satellite_visible_snr, g_d.satellite_visible_snr + constants::maxChannels,
        g_m.satellite_visible_snr.begin()); // Signal-to-noise ratios (dB)
    std::copy(g_d.satellite_visible_used, g_d.satellite_visible_used + constants::maxChannels,
        g_m.satellite_visible_used.begin());
}
