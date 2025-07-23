#ifndef TELEMETRY_SYSTEM_H
#define TELEMETRY_SYSTEM_H

#include <Arduino.h>

// Telemetry System for Rocket Flight Computer
// Handles bidirectional communication with ground station
// Supports multiple radio protocols (LoRa, 433MHz, 2.4GHz)
// Features packet encryption, error correction, and adaptive transmission

// Communication protocols
#define TELEM_PROTOCOL_LORA       0    // LoRa long-range
#define TELEM_PROTOCOL_433MHZ     1    // 433MHz ISM band
#define TELEM_PROTOCOL_24GHZ      2    // 2.4GHz WiFi/custom
#define TELEM_PROTOCOL_BLUETOOTH  3    // Bluetooth for close range
#define TELEM_PROTOCOL_SERIAL     4    // Serial/USB for testing

// Packet types
#define TELEM_PKT_HEARTBEAT       0x01  // Regular status beacon
#define TELEM_PKT_FLIGHT_DATA     0x02  // Flight telemetry data
#define TELEM_PKT_GPS_DATA        0x03  // GPS position data
#define TELEM_PKT_SENSOR_DATA     0x04  // Sensor readings
#define TELEM_PKT_SYSTEM_STATUS   0x05  // System health status
#define TELEM_PKT_ERROR_REPORT    0x06  // Error/fault reporting
#define TELEM_PKT_COMMAND         0x10  // Ground command
#define TELEM_PKT_CONFIG          0x11  // Configuration data
#define TELEM_PKT_FILE_TRANSFER   0x12  // File upload/download
#define TELEM_PKT_ACK             0x20  // Acknowledgment
#define TELEM_PKT_NACK            0x21  // Negative acknowledgment
#define TELEM_PKT_PING            0x30  // Connection test
#define TELEM_PKT_EMERGENCY       0xFF  // Emergency broadcast

// Priority levels
#define TELEM_PRIORITY_LOW        0    // Non-critical data
#define TELEM_PRIORITY_NORMAL     1    // Regular telemetry
#define TELEM_PRIORITY_HIGH       2    // Important status
#define TELEM_PRIORITY_CRITICAL   3    // Emergency/safety data

// Transmission modes
#define TELEM_MODE_CONTINUOUS     0    // Continuous transmission
#define TELEM_MODE_ON_DEMAND      1    // Transmit on request
#define TELEM_MODE_EVENT_DRIVEN   2    // Transmit on events
#define TELEM_MODE_SCHEDULED      3    // Scheduled transmission
#define TELEM_MODE_ADAPTIVE       4    // Adaptive based on conditions

// Error correction and encryption
#define TELEM_FEC_NONE            0    // No forward error correction
#define TELEM_FEC_HAMMING         1    // Hamming code
#define TELEM_FEC_REED_SOLOMON    2    // Reed-Solomon
#define TELEM_ENCRYPTION_NONE     0    // No encryption
#define TELEM_ENCRYPTION_XOR      1    // Simple XOR encryption
#define TELEM_ENCRYPTION_AES      2    // AES encryption

// Buffer and packet sizes
#define TELEM_MAX_PACKET_SIZE     255   // Maximum packet size
#define TELEM_HEADER_SIZE         12    // Packet header size
#define TELEM_MAX_PAYLOAD_SIZE    (TELEM_MAX_PACKET_SIZE - TELEM_HEADER_SIZE)
#define TELEM_TX_BUFFER_SIZE      1024  // Transmission buffer
#define TELEM_RX_BUFFER_SIZE      1024  // Reception buffer
#define TELEM_PACKET_QUEUE_SIZE   32    // Packet queue size

// Timing constants
#define TELEM_HEARTBEAT_INTERVAL  1000  // Heartbeat every 1 second
#define TELEM_ACK_TIMEOUT         3000  // ACK timeout (3 seconds)
#define TELEM_RETRY_COUNT         3     // Number of retries
#define TELEM_CONN_TIMEOUT        30000 // Connection timeout (30 seconds)

// Error codes
#define TELEM_SUCCESS             0
#define TELEM_ERROR_INIT         -1     // Initialization failed
#define TELEM_ERROR_NO_RADIO     -2     // Radio not responding
#define TELEM_ERROR_TX_FAILED    -3     // Transmission failed
#define TELEM_ERROR_RX_TIMEOUT   -4     // Reception timeout
#define TELEM_ERROR_CHECKSUM     -5     // Checksum mismatch
#define TELEM_ERROR_DECRYPT      -6     // Decryption failed
#define TELEM_ERROR_BUFFER_FULL  -7     // Buffer overflow
#define TELEM_ERROR_INVALID_PKT  -8     // Invalid packet format
#define TELEM_ERROR_NO_ACK       -9     // No acknowledgment received
#define TELEM_ERROR_QUEUE_FULL   -10    // Packet queue full

// Data structures
struct TelemetryPacketHeader {
    uint8_t sync_bytes[2];      // Synchronization bytes (0x55, 0xAA)
    uint8_t packet_type;        // Packet type identifier
    uint8_t source_id;          // Source system identifier
    uint8_t destination_id;     // Destination identifier (0xFF = broadcast)
    uint8_t sequence_number;    // Packet sequence number
    uint8_t priority;           // Packet priority level
    uint8_t flags;              // Control flags
    uint8_t payload_length;     // Payload data length
    uint8_t encryption_type;    // Encryption method used
    uint16_t checksum;          // Header + payload checksum
};

struct TelemetryPacket {
    TelemetryPacketHeader header;
    uint8_t payload[TELEM_MAX_PAYLOAD_SIZE];
    uint32_t timestamp;         // Packet creation timestamp
    uint8_t rssi;               // Received signal strength (for RX packets)
    uint8_t retry_count;        // Number of transmission retries
    bool acknowledged;          // Acknowledgment received
};

// Flight data packet structures
struct FlightTelemetryData {
    uint32_t timestamp;         // Flight time (ms)
    float altitude;             // Altitude (m)
    float velocity;             // Velocity (m/s)
    float acceleration;         // Total acceleration (g)
    float max_altitude;         // Maximum altitude reached (m)
    float max_velocity;         // Maximum velocity reached (m/s)
    float max_acceleration;     // Maximum acceleration (g)
    uint8_t flight_state;       // Current flight state
    uint8_t parachute_status;   // Parachute deployment status
    float battery_voltage;      // Battery voltage (V)
    int8_t temperature;         // Internal temperature (°C)
    uint8_t gps_satellites;     // Number of GPS satellites
    uint16_t errors;            // Error flags bitmask
};

struct GPSTelemetryData {
    uint32_t timestamp;         // GPS timestamp
    double latitude;            // Latitude (degrees)
    double longitude;           // Longitude (degrees)
    float altitude_gps;         // GPS altitude (m)
    float speed;                // Ground speed (m/s)
    float course;               // Course over ground (degrees)
    uint8_t fix_type;           // GPS fix type
    uint8_t satellites;         // Number of satellites
    float hdop;                 // Horizontal dilution of precision
    float distance_from_launch; // Distance from launch site (m)
    float bearing_from_launch;  // Bearing from launch site (degrees)
};

struct SensorTelemetryData {
    uint32_t timestamp;         // Sensor reading timestamp
    float accel_x, accel_y, accel_z;    // Accelerometer (g)
    float gyro_x, gyro_y, gyro_z;       // Gyroscope (deg/s)
    float mag_x, mag_y, mag_z;          // Magnetometer (μT)
    float pressure;             // Barometric pressure (Pa)
    float temperature;          // Temperature (°C)
    float humidity;             // Humidity (%) if available
    uint16_t light_level;       // Light sensor reading
    float battery_current;      // Battery current (A)
    uint8_t sensor_health;      // Sensor health status bitmask
};

struct SystemStatusData {
    uint32_t timestamp;         // Status timestamp
    uint32_t uptime;            // System uptime (ms)
    uint8_t cpu_usage;          // CPU usage percentage
    uint16_t free_memory;       // Free RAM (bytes)
    uint32_t sd_free_space;     // SD card free space (KB)
    uint8_t radio_signal;       // Radio signal strength
    uint8_t system_state;       // Overall system state
    uint16_t error_count;       // Total error count
    uint8_t pyro_status;        // Pyrotechnic channel status
    uint8_t servo_status;       // Servo system status
    float internal_temp;        // Internal temperature (°C)
    uint16_t power_consumption; // Power consumption (mW)
};

// Command packet structures
struct TelemetryCommand {
    uint8_t command_id;         // Command identifier
    uint8_t parameters[16];     // Command parameters
    uint32_t execution_time;    // When to execute (0 = immediate)
    uint16_t timeout_ms;        // Command timeout
    bool requires_ack;          // Requires acknowledgment
};

// Common commands
#define TELEM_CMD_ARM_SYSTEM      0x01  // Arm flight system
#define TELEM_CMD_DISARM_SYSTEM   0x02  // Disarm flight system
#define TELEM_CMD_DEPLOY_DROGUE   0x03  // Deploy drogue parachute
#define TELEM_CMD_DEPLOY_MAIN     0x04  // Deploy main parachute
#define TELEM_CMD_ABORT_FLIGHT    0x05  // Abort flight sequence
#define TELEM_CMD_RESET_SYSTEM    0x06  // Reset flight computer
#define TELEM_CMD_SET_CONFIG      0x10  // Set configuration parameter
#define TELEM_CMD_GET_CONFIG      0x11  // Get configuration parameter
#define TELEM_CMD_START_LOGGING   0x12  // Start data logging
#define TELEM_CMD_STOP_LOGGING    0x13  // Stop data logging
#define TELEM_CMD_REQUEST_DATA    0x20  // Request specific data
#define TELEM_CMD_FILE_UPLOAD     0x30  // Upload file to ground
#define TELEM_CMD_FILE_DOWNLOAD   0x31  // Download file from ground
#define TELEM_CMD_RUN_TEST        0x40  // Run system test

// Radio configuration
struct RadioConfig {
    uint8_t protocol;           // Communication protocol
    uint32_t frequency;         // Operating frequency (Hz)
    uint8_t power_level;        // Transmission power (0-100%)
    uint16_t data_rate;         // Data rate (bps)
    uint8_t channel;            // Channel number
    bool use_encryption;        // Enable encryption
    bool use_fec;               // Enable forward error correction
    uint8_t antenna_gain;       // Antenna gain (dBi)
    char call_sign[8];          // Amateur radio call sign
};

// Telemetry configuration
struct TelemetryConfig {
    RadioConfig radio;          // Radio configuration
    uint8_t transmission_mode;  // Transmission mode
    uint16_t heartbeat_interval; // Heartbeat interval (ms)
    uint16_t data_interval;     // Data transmission interval (ms)
    uint8_t max_retries;        // Maximum retry attempts
    uint16_t ack_timeout;       // ACK timeout (ms)
    bool log_packets;           // Log all packets to SD
    bool ground_station_mode;   // Ground station mode
    uint8_t encryption_key[16]; // Encryption key
    float max_range_km;         // Maximum communication range
    int8_t rssi_threshold;      // Minimum RSSI threshold
};

// Statistics and monitoring
struct TelemetryStats {
    uint32_t packets_sent;      // Total packets transmitted
    uint32_t packets_received;  // Total packets received
    uint32_t packets_lost;      // Packets lost (no ACK)
    uint32_t checksum_errors;   // Checksum validation errors
    uint32_t decryption_errors; // Decryption failures
    uint32_t retransmissions;   // Number of retransmissions
    float packet_success_rate;  // Packet success rate (%)
    int8_t avg_rssi;            // Average RSSI
    uint32_t total_bytes_sent;  // Total bytes transmitted
    uint32_t total_bytes_received; // Total bytes received
    uint32_t uplink_time;       // Total uplink time (ms)
    uint32_t downlink_time;     // Total downlink time (ms)
};

class TelemetrySystem {
private:
    // Hardware interfaces
    Stream* radio_serial;       // Serial interface to radio
    uint8_t radio_enable_pin;   // Radio enable/power pin
    uint8_t radio_reset_pin;    // Radio reset pin
    bool initialized;
    
    // Configuration
    TelemetryConfig config;
    uint8_t local_id;           // Local system identifier
    uint8_t ground_station_id;  // Ground station identifier
    
    // Packet management
    TelemetryPacket tx_queue[TELEM_PACKET_QUEUE_SIZE];
    TelemetryPacket rx_queue[TELEM_PACKET_QUEUE_SIZE];
    uint8_t tx_queue_head, tx_queue_tail;
    uint8_t rx_queue_head, rx_queue_tail;
    uint8_t sequence_counter;
    
    // Buffers
    uint8_t tx_buffer[TELEM_TX_BUFFER_SIZE];
    uint8_t rx_buffer[TELEM_RX_BUFFER_SIZE];
    uint16_t rx_buffer_pos;
    
    // Connection state
    bool connected_to_ground;
    uint32_t last_heartbeat_time;
    uint32_t last_received_time;
    uint32_t connection_start_time;
    
    // Statistics
    TelemetryStats stats;
    
    // Acknowledgment tracking
    struct PendingAck {
        uint8_t sequence_number;
        uint32_t timestamp;
        uint8_t retry_count;
        bool active;
    };
    PendingAck pending_acks[16];
    
    // Callbacks
    void (*packet_received_callback)(const TelemetryPacket& packet);
    void (*command_received_callback)(const TelemetryCommand& command);
    void (*connection_status_callback)(bool connected);
    void (*error_callback)(int8_t error_code, const char* description);
    
public:
    TelemetrySystem(Stream& radio_interface, uint8_t enable_pin = 255, uint8_t reset_pin = 255);
    
    // Initialization and configuration
    int8_t begin(const TelemetryConfig& config, uint8_t system_id = 1);
    int8_t configure_radio(const RadioConfig& radio_config);
    bool is_initialized() const { return initialized; }
    void set_local_id(uint8_t id) { local_id = id; }
    void set_ground_station_id(uint8_t id) { ground_station_id = id; }
    
    // Connection management
    int8_t connect_to_ground_station();
    int8_t disconnect_from_ground_station();
    bool is_connected() const { return connected_to_ground; }
    int8_t send_heartbeat();
    int8_t ping_ground_station();
    
    // Packet transmission
    int8_t send_packet(uint8_t packet_type, const void* data, uint8_t length, uint8_t priority = TELEM_PRIORITY_NORMAL);
    int8_t send_flight_data(const FlightTelemetryData& data);
    int8_t send_gps_data(const GPSTelemetryData& data);
    int8_t send_sensor_data(const SensorTelemetryData& data);
    int8_t send_system_status(const SystemStatusData& data);
    int8_t send_error_report(uint16_t error_code, const char* description);
    int8_t send_emergency_packet(const void* data, uint8_t length);
    
    // Command handling
    int8_t send_command(const TelemetryCommand& command, uint8_t destination_id = 0);
    int8_t send_acknowledgment(uint8_t sequence_number, bool positive = true);
    
    // Packet reception and processing
    void update(); // Call regularly to process incoming data
    bool has_pending_packets() const;
    TelemetryPacket get_next_packet();
    TelemetryCommand get_next_command();
    
    // Configuration and parameter management
    int8_t set_transmission_mode(uint8_t mode);
    int8_t set_transmission_interval(uint16_t interval_ms);
    int8_t set_power_level(uint8_t power_percent);
    int8_t set_encryption_key(const uint8_t* key);
    
    // File transfer (for configuration and logs)
    int8_t request_file_upload(const char* filename);
    int8_t request_file_download(const char* filename);
    int8_t send_file_chunk(const char* filename, uint16_t chunk_number, const uint8_t* data, uint8_t length);
    
    // Statistics and monitoring
    const TelemetryStats& get_statistics() const { return stats; }
    void reset_statistics();
    int8_t get_rssi() const;
    float get_link_quality() const;
    uint32_t get_connection_time() const;
    
    // Advanced features
    int8_t enable_adaptive_transmission(bool enable);
    int8_t set_rssi_threshold(int8_t threshold_dbm);
    int8_t perform_range_test();
    int8_t optimize_transmission_parameters();
    
    // Emergency and safety
    int8_t send_abort_command();
    int8_t enable_emergency_beacon(bool enable);
    int8_t set_failsafe_mode(bool enable);
    
    // Callbacks
    void set_packet_received_callback(void (*callback)(const TelemetryPacket& packet));
    void set_command_received_callback(void (*callback)(const TelemetryCommand& command));
    void set_connection_status_callback(void (*callback)(bool connected));
    void set_error_callback(void (*callback)(int8_t error_code, const char* description));
    
    // Debug and diagnostics
    void print_statistics() const;
    void print_connection_status() const;
    void print_packet_queue_status() const;
    int8_t run_radio_test();
    int8_t run_loopback_test();
    
    // Power management
    int8_t set_radio_power_state(bool enable);
    int8_t enter_low_power_mode();
    int8_t exit_low_power_mode();
    
private:
    // Low-level radio communication
    int8_t radio_init();
    int8_t radio_send(const uint8_t* data, uint8_t length);
    int8_t radio_receive(uint8_t* data, uint8_t max_length, uint8_t& received_length);
    bool radio_available();
    int8_t radio_set_frequency(uint32_t frequency);
    int8_t radio_set_power(uint8_t power_level);
    
    // Packet processing
    int8_t build_packet(uint8_t packet_type, const void* data, uint8_t length, uint8_t priority, TelemetryPacket& packet);
    int8_t parse_received_data();
    bool validate_packet(const TelemetryPacket& packet);
    uint16_t calculate_checksum(const uint8_t* data, uint8_t length);
    
    // Queue management
    bool enqueue_tx_packet(const TelemetryPacket& packet);
    bool dequeue_tx_packet(TelemetryPacket& packet);
    bool enqueue_rx_packet(const TelemetryPacket& packet);
    bool dequeue_rx_packet(TelemetryPacket& packet);
    void process_tx_queue();
    
    // Acknowledgment handling
    void add_pending_ack(uint8_t sequence_number);
    void process_pending_acks();
    void handle_acknowledgment(uint8_t sequence_number, bool positive);
    
    // Encryption and error correction
    int8_t encrypt_payload(uint8_t* data, uint8_t length);
    int8_t decrypt_payload(uint8_t* data, uint8_t length);
    int8_t apply_error_correction(uint8_t* data, uint8_t length);
    int8_t check_error_correction(uint8_t* data, uint8_t length);
    
    // Connection management
    void update_connection_status();
    void handle_connection_timeout();
    void process_heartbeat();
    
    // Adaptive transmission
    void analyze_link_quality();
    void adjust_transmission_parameters();
    void update_statistics();
    
    // Utility functions
    void reset_radio();
    void clear_buffers();
    uint8_t get_next_sequence_number();
    bool is_broadcast_packet(const TelemetryPacket& packet);
    bool is_packet_for_us(const TelemetryPacket& packet);
};

// Global instance helper
extern TelemetrySystem* g_telemetry;

// Utility functions and helpers
namespace TelemetryUtils {
    // Packet formatting and parsing
    void format_flight_data_string(const FlightTelemetryData& data, char* buffer, size_t buffer_size);
    void format_gps_data_string(const GPSTelemetryData& data, char* buffer, size_t buffer_size);
    void format_sensor_data_string(const SensorTelemetryData& data, char* buffer, size_t buffer_size);
    
    // Data compression for efficient transmission
    uint8_t compress_flight_data(const FlightTelemetryData& data, uint8_t* compressed, uint8_t max_size);
    bool decompress_flight_data(const uint8_t* compressed, uint8_t length, FlightTelemetryData& data);
    
    // Range and link calculations
    float calculate_theoretical_range(float tx_power_dbm, float rx_sensitivity_dbm, float frequency_mhz, float antenna_gain_db = 0);
    float calculate_path_loss(float distance_km, float frequency_mhz);
    int8_t estimate_rssi_at_distance(float distance_km, float tx_power_dbm, float frequency_mhz);
    
    // Protocol-specific helpers
    namespace LoRa {
        struct LoRaConfig {
            uint32_t frequency;     // Frequency in Hz
            uint8_t spreading_factor; // 6-12
            uint8_t bandwidth;      // 0=125kHz, 1=250kHz, 2=500kHz
            uint8_t coding_rate;    // 1-4 (4/5, 4/6, 4/7, 4/8)
            int8_t tx_power;        // 2-20 dBm
            bool crc_enable;        // Enable CRC
            bool implicit_header;   // Implicit header mode
        };
        
        int8_t configure_lora_module(const LoRaConfig& config);
        float calculate_lora_range(const LoRaConfig& config);
        uint32_t calculate_air_time(uint8_t payload_length, const LoRaConfig& config);
    }
    
    // Ground station communication helpers
    struct GroundStationInfo {
        uint8_t station_id;
        char call_sign[8];
        float latitude, longitude;
        float altitude;
        uint16_t capabilities;
        uint32_t last_contact;
        int8_t rssi;
        float distance_km;
    };
    
    bool parse_ground_station_beacon(const TelemetryPacket& packet, GroundStationInfo& info);
    int8_t create_ground_station_beacon(const GroundStationInfo& info, TelemetryPacket& packet);
    
    // Emergency and safety protocols
    int8_t create_emergency_packet(uint8_t emergency_type, const void* data, uint8_t length, TelemetryPacket& packet);
    bool is_emergency_packet(const TelemetryPacket& packet);
    void handle_emergency_packet(const TelemetryPacket& packet);
    
    // Data logging for telemetry
    int8_t log_telemetry_packet(const TelemetryPacket& packet, const char* log_file);
    int8_t create_telemetry_log_header(const char* session_name);
    
    // Configuration templates
    namespace Templates {
        extern const TelemetryConfig LONG_RANGE_CONFIG;     // Long range LoRa configuration
        extern const TelemetryConfig HIGH_SPEED_CONFIG;     // High speed 2.4GHz configuration
        extern const TelemetryConfig LOW_POWER_CONFIG;      // Low power consumption configuration
        extern const TelemetryConfig GROUND_STATION_CONFIG; // Ground station configuration
        extern const TelemetryConfig TEST_CONFIG;           // Testing and development configuration
    }
    
    // Packet type utilities
    const char* packet_type_to_string(uint8_t packet_type);
    const char* command_id_to_string(uint8_t command_id);
    const char* error_code_to_string(int8_t error_code);
    
    // Data validation
    bool validate_flight_data(const FlightTelemetryData& data);
    bool validate_gps_data(const GPSTelemetryData& data);
    bool validate_sensor_data(const SensorTelemetryData& data);
    
    // Performance analysis
    struct LinkPerformance {
        float packet_loss_rate;
        float average_latency_ms;
        float throughput_bps;
        int8_t signal_quality;
        float range_estimate_km;
    };
    
    LinkPerformance analyze_link_performance(const TelemetryStats& stats, uint32_t analysis_window_ms);
    void generate_performance_report(const LinkPerformance& performance, char* report, size_t report_size);
    
    // Frequency management
    bool is_frequency_legal(uint32_t frequency_hz, const char* country_code = "US");
    uint32_t get_next_channel_frequency(uint32_t base_frequency, uint8_t channel, uint32_t channel_spacing);
    int8_t scan_for_clear_channel(uint32_t start_freq, uint32_t end_freq, uint32_t step_size);
    
    // Antenna and propagation
    float calculate_antenna_gain(float frequency_mhz, float antenna_length_m);
    float calculate_fresnel_zone_radius(float distance_km, float frequency_mhz);
    bool check_line_of_sight(float tx_alt_m, float rx_alt_m, float distance_km);
}

#endif // TELEMETRY_SYSTEM_H