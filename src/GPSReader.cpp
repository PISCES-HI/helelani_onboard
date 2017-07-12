#include "GPSReader.h"
#include "minmea.h"
#include <ros/node_handle.h>
#include <sensor_msgs/NavSatFix.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <signal.h>

GPSReader::GPSReader(ros::NodeHandle& n, const std::string& path)
: m_gpsFixPub(n.advertise<gps_common::GPSFix>("/helelani/gps_fix", 1000)),
  m_navSatFixPub(n.advertise<sensor_msgs::NavSatFix>("/helelani/nav_sat_fix", 1000))
{
    if (path.empty())
        return;

    m_thread = std::thread([this, &path]()
    {
        // Open file descriptor
        int fd = open(path.c_str(), O_RDONLY);
        if (!fd) {
            ROS_ERROR("Unable to start GPS reader thread: %s", strerror(errno));
            return;
        }

        // Set serial parameters and wrap in FILE stream
        speed_t baud = B38400;
        struct termios settings;
        tcgetattr(fd, &settings);
        cfsetospeed(&settings, baud);
        cfsetispeed(&settings, baud);
        settings.c_cflag &= ~PARENB;
        settings.c_cflag &= ~CSTOPB;
        settings.c_cflag &= ~CSIZE;
        settings.c_cflag |= CS8 | CLOCAL;
        settings.c_iflag |= ICANON;
        settings.c_oflag &= ~OPOST;
        tcsetattr(fd, TCSANOW, &settings);
        tcflush(fd, TCOFLUSH);
        FILE* fp = fdopen(fd, "r");

        // Lists to assemble satellite objects
        gps_common::GPSStatus::_satellites_used_type satellites_used = 0;
        gps_common::GPSStatus::_satellite_used_prn_type satellite_used_prn;
        gps_common::GPSStatus::_satellites_visible_type satellites_visible = 0;
        gps_common::GPSStatus::_satellite_visible_prn_type satellite_visible_prn;
        gps_common::GPSStatus::_satellite_visible_z_type satellite_visible_z;
        gps_common::GPSStatus::_satellite_visible_azimuth_type satellite_visible_azimuth;
        gps_common::GPSStatus::_satellite_visible_snr_type satellite_visible_snr;

        // Parse NMEA sentence
        char line[MINMEA_MAX_LENGTH];
        while (m_running) {
            if (fgets(line, sizeof(line), fp) != nullptr) {
                std::lock_guard<std::mutex> lk(m_lock);
                char talker[3];
                minmea_talker_id(talker, line);
                switch (minmea_sentence_id(line, false)) {
                case MINMEA_SENTENCE_RMC: {
                    struct minmea_sentence_rmc frame;
                    if (minmea_parse_rmc(&frame, line)) {
                        m_gpsFix.latitude = minmea_tocoord(&frame.latitude);
                        m_gpsFix.longitude = minmea_tocoord(&frame.longitude);
                        m_gpsFix.speed = minmea_tofloat(&frame.speed);
                        m_gpsFix.track = minmea_tofloat(&frame.course);
                    }
                } break;
                case MINMEA_SENTENCE_GGA: {
                    struct minmea_sentence_gga frame;
                    if (minmea_parse_gga(&frame, line)) {
                        m_gpsFix.latitude = minmea_tocoord(&frame.latitude);
                        m_gpsFix.longitude = minmea_tocoord(&frame.longitude);
                        m_gpsFix.altitude = minmea_tofloat(&frame.altitude);
                        satellites_used = frame.satellites_tracked;

                        switch (frame.fix_quality) {
                        case 0: // No fix
                        default:
                            m_gpsFix.status.status = gps_common::GPSStatus::STATUS_NO_FIX;
                            break;
                        case 1: // GPS
                            m_gpsFix.status.status = gps_common::GPSStatus::STATUS_FIX;
                            break;
                        case 2: // DGPS
                            m_gpsFix.status.status = gps_common::GPSStatus::STATUS_DGPS_FIX;
                            break;
                        }

                        m_gpsFix.status.position_source = gps_common::GPSStatus::SOURCE_GPS;
                    }
                } break;
                case MINMEA_SENTENCE_GSA: {
                    struct minmea_sentence_gsa frame;
                    if (minmea_parse_gsa(&frame, line)) {
                        for (int i=0 ; i<12 ; ++i)
                            if (frame.sats[i])
                                satellite_used_prn.push_back(frame.sats[i]);
                        m_gpsFix.pdop = minmea_tofloat(&frame.pdop);
                        m_gpsFix.hdop = minmea_tofloat(&frame.hdop);
                        m_gpsFix.vdop = minmea_tofloat(&frame.vdop);
                    }
                } break;
                case MINMEA_SENTENCE_GSV: {
                    if (talker[1] == 'P') { // GPS only
                        struct minmea_sentence_gsv frame;
                        if (minmea_parse_gsv(&frame, line)) {
                            satellites_visible = frame.total_sats;
                            for (int i=0 ; i<4 ; ++i) {
                                minmea_sat_info& sat = frame.sats[i];
                                if (sat.nr) {
                                    satellite_visible_prn.push_back(sat.nr);
                                    satellite_visible_z.push_back(sat.elevation);
                                    satellite_visible_azimuth.push_back(sat.azimuth);
                                    satellite_visible_snr.push_back(sat.snr);
                                }
                            }
                        }
                    }
                } break;
                default: break;
                }

                if (satellite_used_prn.size() == satellites_used) {
                    m_gpsFix.status.satellites_used = satellites_used;
                    m_gpsFix.status.satellite_used_prn = satellite_used_prn;
                    satellite_used_prn.clear();
                }

                if (satellite_visible_prn.size() == satellites_visible) {
                    m_gpsFix.status.satellites_visible = satellites_visible;
                    m_gpsFix.status.satellite_visible_prn = satellite_visible_prn;
                    m_gpsFix.status.satellite_visible_z = satellite_visible_z;
                    m_gpsFix.status.satellite_visible_azimuth = satellite_visible_azimuth;
                    m_gpsFix.status.satellite_visible_snr = satellite_visible_snr;
                    satellite_visible_prn.clear();
                    satellite_visible_z.clear();
                    satellite_visible_azimuth.clear();
                    satellite_visible_snr.clear();
                    PublishReading();
                }
            }
        }

        // Cleanup
        fclose(fp);
        close(fd);
    });
}

GPSReader::~GPSReader()
{
    m_running = false;
    if (m_thread.joinable())
    {
        pthread_kill(m_thread.native_handle(), SIGUSR1);
        m_thread.join();
    }
}

void GPSReader::PublishReading()
{
    m_gpsFix.header.stamp = ros::Time::now();
    m_gpsFix.status.header.stamp = m_gpsFix.header.stamp;
    m_navSatFix.header.stamp = m_gpsFix.header.stamp;
    m_navSatFix.status.status = (m_gpsFix.status.status > gps_common::GPSStatus::STATUS_NO_FIX) ?
        int(sensor_msgs::NavSatStatus::STATUS_FIX) : int(sensor_msgs::NavSatStatus::STATUS_NO_FIX);
    m_navSatFix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    m_navSatFix.latitude = m_gpsFix.latitude;
    m_navSatFix.longitude = m_gpsFix.longitude;
    m_navSatFix.altitude = m_gpsFix.altitude;
    m_gpsFixPub.publish(m_gpsFix);
    m_navSatFixPub.publish(m_navSatFix);
}
