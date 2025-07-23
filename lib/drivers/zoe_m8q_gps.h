#ifndef ZOE_M8Q_GPS_H
#define ZOE_M8Q_GPS_H

#include <Arduino.h>
#include <SoftwareSerial.h>

// u-blox ZOE-M8Q GPS Module Driver
// Compact GNSS receiver with concurrent reception of up to 3 GNSS systems
// UART interface, typically 9600 baud default

// UBX Protocol Constants
#define UBX_SYNC_CHAR_1       0xB5
#define UBX_SYNC_CHAR_2       0x62

// UBX Message Classes
#define UBX_CLASS_NAV         0x01  // Navigation Results
#define UBX_CLASS_RXM         0x02  // Receiver Manager Messages
#define UBX_CLASS_INF         0x04  // Information Messages
#define UBX_CLASS_ACK         0x05  // Ack/Nak Messages
#define UBX_CLASS_CFG         0x06  // Configuration Input Messages
#define UBX_CLASS_UPD         0x09  // Firmware Update Messages
#define UBX_CLASS_MON         0x0A  // Monitoring Messages
#define UBX_CLASS_AID         0x0B  // AssistNow Aiding Messages
#define UBX_CLASS_TIM         0x0D  // Timing Messages
#define UBX_CLASS_ESF         0x10  // External Sensor Fusion Messages

// UBX Navigation Messages
#define UBX_NAV_POSECEF       0x01  // Position Solution in ECEF
#define UBX_NAV_POSLLH        0x02  // Geodetic Position Solution
#define UBX_NAV_STATUS        0x03  // Receiver Navigation Status
#define UBX_NAV_DOP           0x04  // Dilution of precision
#define UBX_NAV_ATT           0x05  // Attitude Solution
#define UBX_NAV_SOL           0x06  // Navigation Solution Information
#define UBX_NAV_PVT           0x07  // Navigation Position Velocity Time Solution
#define UBX_NAV_ODO           0x09  // Odometer Solution
#define UBX_NAV_RESETODO      0x10  // Reset odometer
#define UBX_NAV_VELECEF       0x11  // Velocity Solution in ECEF
#define UBX_NAV_VELNED        0x12  // Velocity Solution in NED
#define UBX_NAV_TIMEGPS       0x20  // GPS Time Solution
#define UBX_NAV_TIMEUTC       0x21  // UTC Time Solution
#define UBX_NAV_CLOCK         0x22  // Clock Solution
#define UBX_NAV_DGPS          0x31  // DGPS Data Used for NAV
#define UBX_NAV_SBAS          0x32  // SBAS Status Data
#define UBX_NAV_EKFSTATUS     0x40  // Extended Kalman Filter Status

// UBX Configuration Messages
#define UBX_CFG_PRT           0x00  // Port Configuration
#define UBX_CFG_MSG           0x01  // Message Configuration
#define UBX_CFG_INF           0x02  // Information Message Configuration
#define UBX_CFG_RST           0x04  // Reset Receiver / Clear Backup Data Structures
#define UBX_CFG_DAT           0x06  // Datum Setting
#define UBX_CFG_RATE          0x08  // Navigation/Measurement Rate Settings
#define UBX_CFG_CFG           0x09  // Clear, Save and Load configurations
#define UBX_CFG_RXM           0x11  // RXM configuration
#define UBX_CFG_ANT           0x13  // Antenna Control Settings
#define UBX_CFG_SBAS          0x16  // SBAS Configuration
#define UBX_CFG_NMEA          0x17  // NMEA protocol configuration
#define UBX_CFG_USB           0x1B  // USB Configuration
#define UBX_CFG_TMODE         0x1D  // Time Mode Settings
#define UBX_CFG_ODO           0x1E  // Odometer, Low-speed COG Engine Settings
#define UBX_CFG_NVS           0x22  // Clear, Save and Load non-volatile storage
#define UBX_CFG_NAVX5         0x23  // Navigation Engine Expert Settings
#define UBX_CFG_NAV5          0x24  // Navigation Engine Settings
#define UBX_CFG_TP            0x31  // Time Pulse Parameters
#define UBX_CFG_RINV          0x34  // Contents of Remote Inventory
#define UBX_CFG_ITFM          0x39  // Jamming/Interference Monitor configuration
#define UBX_CFG_PM            0x3B  // Power Management configuration
#define UBX_CFG_TMODE2        0x3D  // Time Mode Settings 2
#define UBX_CFG_GNSS          0x3E  // GNSS system configuration
#define UBX_CFG_LOGFILTER     0x47  // Data Logger Configuration
#define UBX_CFG_TXSLOT        0x53  // TX buffer time slots configuration
#define UBX_CFG_PWR           0x57  // Put receiver in a defined power state
#define UBX_CFG_HNR           0x5C  // High Navigation Rate Settings
#define UBX_CFG_ESRC          0x60  // External synchronization source configuration
#define UBX_CFG_DOSC          0x61  // Disciplined oscillator configuration
#define UBX_CFG_SMGR          0x62  // Synchronization manager configuration
#define UBX_CFG_GEOFENCE      0x69  // Geofencing configuration

// UBX Acknowledge Messages
#define UBX_ACK_NAK           0x00  // Message Not-Acknowledged
#define UBX_ACK_ACK           0x01  // Message Acknowledged

// NMEA Sentence Types
#define NMEA_GGA              0     // Global Positioning System Fix Data
#define NMEA_GLL              1     // Geographic Position - Latitude/Longitude
#define NMEA_GSA              2     // GPS DOP and active satellites
#define NMEA_GSV              3     // GPS Satellites in view
#define NMEA_RMC              4     // Recommended Minimum Navigation Information
#define NMEA_VTG              5     // Track made good and Ground speed

// GPS Fix Types
#define GPS_FIX_NONE          0     // No fix
#define GPS_FIX_DEAD_RECKONING 1    // Dead reckoning only
#define GPS_FIX_2D            2     // 2D fix
#define GPS_FIX_3D            3     // 3D fix
#define GPS_FIX_GPS_DR        4     // GPS + dead reckoning combined
#define GPS_FIX_TIME_ONLY     5     // Time only fix

// Power Save Modes
#define GPS_POWER_FULL        0     // Full power
#define GPS_POWER_BALANCED    1     // Balanced power/performance
#define GPS_POWER_INTERVAL    2     // Interval mode
#define GPS_POWER_AGGRESSIVE  3     // Aggressive power saving

// Error codes
#define GPS_SUCCESS           0
#define GPS_ERROR_INIT       -1
#define GPS_ERROR_TIMEOUT    -2
#define GPS_ERROR_CHECKSUM   -3
#define GPS_ERROR_NO_DATA    -4
#define GPS_ERROR_CONFIG     -5

// Buffer sizes
#define GPS_BUFFER_SIZE       256
#define GPS_MAX_SATELLITES    12

// Data structures
struct GPS_DateTime {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
    bool valid;
};

struct GPS_Position {
    double latitude;        // Degrees
    double longitude;       // Degrees
    float altitude;         // Meters above sea level
    float height_msl;       // Height above mean sea level
    float geoid_height;     // Geoid height
    uint8_t fix_type;       // GPS fix type
    uint8_t satellites;     // Number of satellites used
    float hdop;             // Horizontal dilution of precision
    float vdop;             // Vertical dilution of precision
    float pdop;             // Position dilution of precision
    bool valid;
};

struct GPS_Velocity {
    float speed;            // Ground speed (m/s)
    float course;           // Course over ground (degrees)
    float climb_rate;       // Vertical velocity (m/s)
    float speed_accuracy;   // Speed accuracy estimate (m/s)
    float course_accuracy;  // Course accuracy estimate (degrees)
    bool valid;
};

struct GPS_Satellite {
    uint8_t prn;            // Satellite PRN number
    uint8_t elevation;      // Elevation in degrees
    uint16_t azimuth;       // Azimuth in degrees
    uint8_t snr;            // Signal to noise ratio
    bool used;              // Used in navigation solution
};

struct GPS_SatelliteInfo {
    uint8_t satellites_in_view;
    uint8_t satellites_used;
    GPS_Satellite satellites[GPS_MAX_SATELLITES];
    bool valid;
};

struct GPS_Data {
    GPS_DateTime datetime;
    GPS_Position position;
    GPS_Velocity velocity;
    GPS_SatelliteInfo sat_info;
    uint32_t timestamp;     // System timestamp when data was received
    bool valid;
};

// UBX packet structure
struct UBX_Packet {
    uint8_t sync1;          // 0xB5
    uint8_t sync2;          // 0x62
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint8_t payload[GPS_BUFFER_SIZE];
    uint8_t checksum_a;
    uint8_t checksum_b;
};

class ZOE_M8Q_GPS {
private:
    HardwareSerial* gps_serial;
    SoftwareSerial* soft_serial;
    uint32_t baud_rate;
    bool use_hardware_serial;
    bool initialized;
    
    GPS_Data current_data;
    
    // Parsing state
    uint8_t parse_state;
    uint8_t nmea_buffer[GPS_BUFFER_SIZE];
    uint8_t nmea_index;
    UBX_Packet ubx_packet;
    uint8_t ubx_index;
    
    // Configuration
    uint16_t update_rate_ms;
    uint8_t power_mode;
    bool ubx_protocol_enabled;
    bool nmea_protocol_enabled;
    
    // Statistics
    uint32_t packets_received;
    uint32_t packets_parsed;
    uint32_t checksum_errors;
    uint32_t timeout_errors;
    
    // Timing
    uint32_t last_fix_time;
    uint32_t last_update_time;
    
public:
    ZOE_M8Q_GPS(HardwareSerial& serial, uint32_t baud = 9600);
    ZOE_M8Q_GPS(uint8_t rx_pin, uint8_t tx_pin, uint32_t baud = 9600);
    
    // Initialization and configuration
    int8_t begin();
    int8_t reset();
    bool is_initialized() const { return initialized; }
    
    // Configuration functions
    int8_t set_update_rate(uint16_t rate_ms);
    int8_t set_baud_rate(uint32_t baud);
    int8_t set_power_mode(uint8_t mode);
    int8_t enable_nmea_messages(bool enable);
    int8_t enable_ubx_messages(bool enable);
    
    // Data reading functions
    void update();
    bool has_fix() const;
    bool has_new_data() const;
    const GPS_Data& get_data() const { return current_data; }
    
    // Individual data access
    GPS_Position get_position() const { return current_data.position; }
    GPS_Velocity get_velocity() const { return current_data.velocity; }
    GPS_DateTime get_datetime() const { return current_data.datetime; }
    GPS_SatelliteInfo get_satellite_info() const { return current_data.sat_info; }
    
    // Navigation utilities
    float calculate_distance(double lat1, double lon1, double lat2, double lon2);
    float calculate_bearing(double lat1, double lon1, double lat2, double lon2);
    bool is_inside_geofence(double center_lat, double center_lon, float radius_m);
    
    // Time utilities
    uint32_t get_unix_timestamp() const;
    bool is_time_valid() const;
    uint32_t get_milliseconds_since_fix() const;
    
    // Quality assessment
    bool is_fix_valid() const;
    uint8_t get_fix_quality() const;
    float get_horizontal_accuracy() const;
    float get_vertical_accuracy() const;
    
    // Statistics and diagnostics
    uint32_t get_packets_received() const { return packets_received; }
    uint32_t get_packets_parsed() const { return packets_parsed; }
    uint32_t get_checksum_errors() const { return checksum_errors; }
    float get_parse_success_rate() const;
    void reset_statistics();
    
    // Advanced configuration
    int8_t configure_gnss_systems(bool gps, bool glonass, bool galileo, bool beidou);
    int8_t set_dynamic_model(uint8_t model); // Pedestrian, automotive, sea, airborne, etc.
    int8_t configure_jamming_detection(bool enable);
    int8_t save_configuration();
    
    // Low power functions
    int8_t enter_sleep_mode();
    int8_t wake_up();
    int8_t set_backup_mode();
    
    // Debugging and diagnostics
    void print_satellite_info();
    void print_position_info();
    void print_system_status();
    void enable_debug_output(bool enable);
    
private:
    // NMEA parsing functions
    void parse_nmea_sentence();
    bool parse_gga(const char* sentence);
    bool parse_rmc(const char* sentence);
    bool parse_gsa(const char* sentence);
    bool parse_gsv(const char* sentence);
    
    // UBX parsing functions
    void parse_ubx_packet();
    bool parse_nav_pvt(const uint8_t* payload, uint16_t length);
    bool parse_nav_posllh(const uint8_t* payload, uint16_t length);
    bool parse_nav_velned(const uint8_t* payload, uint16_t length);
    bool parse_nav_status(const uint8_t* payload, uint16_t length);
    
    // UBX packet functions
    int8_t send_ubx_packet(uint8_t msg_class, uint8_t msg_id, const uint8_t* payload, uint16_t length);
    bool wait_for_ack(uint8_t msg_class, uint8_t msg_id, uint32_t timeout_ms = 1000);
    void calculate_ubx_checksum(const UBX_Packet& packet, uint8_t& ck_a, uint8_t& ck_b);
    
    // Utility functions
    bool validate_nmea_checksum(const char* sentence);
    uint8_t hex_char_to_byte(char c);
    double nmea_to_decimal_degrees(const char* coord, char hemisphere);
    uint32_t datetime_to_unix(const GPS_DateTime& dt);
    
    // Data validation
    bool validate_position_data(const GPS_Position& pos);
    bool validate_velocity_data(const GPS_Velocity& vel);
    bool validate_datetime_data(const GPS_DateTime& dt);
    
    // Internal state management
    void reset_parse_state();
    void update_statistics();
    void mark_data_updated();
};

// Global instance helper
extern ZOE_M8Q_GPS* g_gps;

// Utility functions
namespace GPS_Utils {
    // Coordinate conversions
    void decimal_to_dms(double decimal, int& degrees, int& minutes, float& seconds);
    double dms_to_decimal(int degrees, int minutes, float seconds);
    
    // Distance and bearing calculations
    double haversine_distance(double lat1, double lon1, double lat2, double lon2);
    double great_circle_bearing(double lat1, double lon1, double lat2, double lon2);
    void destination_point(double lat, double lon, double bearing, double distance, 
                          double& dest_lat, double& dest_lon);
    
    // Coordinate system conversions
    void wgs84_to_ecef(double lat, double lon, double alt, double& x, double& y, double& z);
    void ecef_to_wgs84(double x, double y, double z, double& lat, double& lon, double& alt);
    
    // Geofencing utilities
    bool point_in_polygon(double lat, double lon, const double* poly_lats, const double* poly_lons, int vertices);
    bool point_in_circle(double lat, double lon, double center_lat, double center_lon, double radius_m);
    
    // Time utilities
    bool is_leap_year(uint16_t year);
    uint8_t days_in_month(uint8_t month, uint16_t year);
    uint32_t gps_week_seconds_to_unix(uint16_t gps_week, uint32_t gps_seconds);
    
    // Data quality assessment
    const char* fix_type_to_string(uint8_t fix_type);
    const char* quality_assessment(float hdop, uint8_t satellites);
    bool is_position_reasonable(double lat, double lon, float alt);
    
    // Navigation algorithms
    struct WaypointNavigation {
        double target_lat;
        double target_lon;
        double distance_to_target;
        double bearing_to_target;
        double cross_track_error;
        bool arrived;
    };
    
    WaypointNavigation navigate_to_waypoint(double current_lat, double current_lon, 
                                           double target_lat, double target_lon, 
                                           float arrival_radius = 10.0f);
    
    // Flight tracking utilities
    float calculate_ground_speed(const GPS_Velocity& vel);
    float calculate_track_angle(const GPS_Velocity& vel);
    float calculate_altitude_rate(float current_alt, float previous_alt, uint32_t time_diff_ms);
    
    // Data logging helpers
    void format_position_string(const GPS_Position& pos, char* buffer, size_t buffer_size);
    void format_datetime_string(const GPS_DateTime& dt, char* buffer, size_t buffer_size);
    void format_kml_coordinate(const GPS_Position& pos, char* buffer, size_t buffer_size);
}

#endif // ZOE_M8Q_GPS_H