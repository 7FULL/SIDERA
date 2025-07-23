#ifndef SAMD21_TELEMETRY_INTEGRATION_H
#define SAMD21_TELEMETRY_INTEGRATION_H

#include <Arduino.h>
#include "../../lib/telemetry/telemetry_system.h"
#include "../../lib/common/spi_protocol.h"

// SAMD21G18A-AU Telemetry Controller Integration
// Manages radio communication with ground station

// Pin definitions for SAMD21 Telemetry
#define TEL_RADIO_RX_PIN          0     // Radio UART RX (Serial1)
#define TEL_RADIO_TX_PIN          1     // Radio UART TX (Serial1)
#define TEL_RADIO_ENABLE_PIN      6     // Radio enable/power pin
#define TEL_RADIO_RESET_PIN       7     // Radio reset pin
#define TEL_RADIO_CONFIG_PIN      8     // Radio configuration pin (if needed)
#define TEL_RADIO_STATUS_PIN      9     // Radio status/ready pin

// SPI communication with master
#define TEL_SPI_MOSI_PIN          11    // SPI MOSI
#define TEL_SPI_MISO_PIN          12    // SPI MISO
#define TEL_SPI_SCK_PIN           13    // SPI SCK
#define TEL_SPI_CS_PIN            10    // SPI CS (slave select)

// Status LED pins
#define TEL_LED_RED_PIN           2     // Status LED Red
#define TEL_LED_GREEN_PIN         3     // Status LED Green
#define TEL_LED_BLUE_PIN          4     // Status LED Blue

// Additional I/O pins for radio control
#define TEL_ANTENNA_SELECT_PIN    5     // Antenna diversity select
#define TEL_PA_ENABLE_PIN         14    // Power amplifier enable
#define TEL_LNA_ENABLE_PIN        15    // Low noise amplifier enable

// Update intervals
#define TEL_RADIO_UPDATE_RATE     10    // 10ms for radio processing
#define TEL_DATA_TX_RATE          100   // 100ms for regular data transmission
#define TEL_HEARTBEAT_RATE        1000  // 1s for heartbeat
#define TEL_STATUS_UPDATE_RATE    200   // 200ms for status updates

// Communication parameters
#define TEL_MAX_PACKET_BUFFER     10    // Maximum packets to buffer
#define TEL_RETRY_TIMEOUT         3000  // 3s retry timeout
#define TEL_CONNECTION_TIMEOUT    10000 // 10s connection timeout
#define TEL_RSSI_UPDATE_RATE      500   // 500ms RSSI update

// Telemetry data structures
struct TelemetryControllerData {
    // Radio status
    struct {
        bool radio_ready;                   // Radio module ready
        bool connected_to_ground;           // Connected to ground station
        int8_t rssi;                        // Signal strength (dBm)
        uint8_t link_quality;               // Link quality (0-100%)
        uint32_t packets_sent;              // Total packets sent
        uint32_t packets_received;          // Total packets received
        uint32_t packets_lost;              // Packets lost/failed
        uint32_t last_contact_time;         // Last successful communication
        uint32_t connection_time;           // Total connection time
        uint32_t timestamp;
    } radio_status;
    
    // Ground station info
    struct {
        uint8_t ground_station_id;          // Ground station identifier
        char ground_call_sign[8];           // Ground station call sign
        float ground_distance_km;           // Distance to ground station
        int8_t ground_rssi;                 // Ground station RSSI
        bool ground_commands_enabled;       // Ground commands allowed
        uint32_t last_command_time;         // Last command received
        uint32_t timestamp;
    } ground_station;
    
    // Data transmission status
    struct {
        uint32_t flight_data_sent;          // Flight data packets sent
        uint32_t gps_data_sent;             // GPS data packets sent
        uint32_t sensor_data_sent;          // Sensor data packets sent
        uint32_t status_data_sent;          // Status data packets sent
        uint32_t emergency_data_sent;       // Emergency packets sent
        float avg_transmission_rate;        // Average transmission rate (pps)
        uint32_t data_bytes_sent;           // Total data bytes transmitted
        uint32_t timestamp;
    } transmission_stats;
};

// Radio configuration profiles
struct RadioProfile {
    const char* name;
    uint32_t frequency;         // Operating frequency (Hz)
    uint8_t power_level;        // TX power (0-100%)
    uint16_t data_rate;         // Data rate (bps)
    uint8_t modulation;         // Modulation type
    bool use_encryption;        // Enable encryption
    bool use_fec;               // Forward error correction
    float max_range_km;         // Expected maximum range
};

// Command handling
enum GroundCommand {
    CMD_ARM_SYSTEM = 0x01,
    CMD_DISARM_SYSTEM = 0x02,
    CMD_DEPLOY_DROGUE = 0x03,
    CMD_DEPLOY_MAIN = 0x04,
    CMD_ABORT_FLIGHT = 0x05,
    CMD_SET_PARAMETER = 0x10,
    CMD_REQUEST_DATA = 0x20,
    CMD_START_LOGGING = 0x30,
    CMD_STOP_LOGGING = 0x31,
    CMD_EMERGENCY_STOP = 0xFF
};

class SAMD21TelemetryIntegration {
private:
    // Hardware instances
    TelemetrySystem* telemetry;
    SPISlave* spi_slave;
    
    // Telemetry data
    TelemetryControllerData tel_data;
    
    // Communication state
    bool initialized;
    bool radio_configured;
    bool ground_station_connected;
    uint8_t current_radio_profile;
    
    // Data buffers for received master data
    struct MasterDataBuffer {
        uint8_t flight_data[64];
        uint8_t gps_data[64];
        uint8_t sensor_data[128];
        uint8_t system_status[32];
        bool flight_data_ready;
        bool gps_data_ready;
        bool sensor_data_ready;
        bool status_data_ready;
        uint32_t last_update_time;
    } master_data;
    
    // Transmission queue
    struct TransmissionQueue {
        TelemetryPacket packets[TEL_MAX_PACKET_BUFFER];
        uint8_t head, tail;
        uint8_t count;
    } tx_queue;
    
    // Timing control
    uint32_t last_radio_update;
    uint32_t last_data_transmission;
    uint32_t last_heartbeat;
    uint32_t last_status_update;
    uint32_t last_rssi_update;
    uint32_t last_master_communication;
    
    // Configuration
    RadioProfile radio_profiles[4];     // Multiple radio profiles
    TelemetryConfig current_config;
    
    // Statistics tracking
    struct TelemetryStats {
        uint32_t uptime;
        uint32_t radio_resets;
        uint32_t connection_attempts;
        uint32_t successful_connections;
        uint32_t command_executions;
        float avg_packet_rate;
        float peak_packet_rate;
    } stats;
    
    // Callbacks
    void (*ground_command_callback)(GroundCommand command, const uint8_t* parameters);
    void (*connection_status_callback)(bool connected);
    void (*data_received_callback)(const TelemetryPacket& packet);
    void (*error_callback)(const char* error_msg);
    
public:
    SAMD21TelemetryIntegration();
    ~SAMD21TelemetryIntegration();
    
    // Initialization
    int8_t begin();
    int8_t initialize_radio();
    int8_t configure_radio_profile(uint8_t profile_id);
    bool is_initialized() const { return initialized; }
    
    // Main update function
    void update();
    
    // Radio management
    int8_t connect_to_ground_station();
    int8_t disconnect_from_ground_station();
    bool is_connected_to_ground() const { return ground_station_connected; }
    int8_t scan_for_ground_stations();
    int8_t switch_radio_profile(uint8_t profile_id);
    
    // Data transmission
    int8_t transmit_flight_data(const void* data, uint16_t length);
    int8_t transmit_gps_data(const void* data, uint16_t length);
    int8_t transmit_sensor_data(const void* data, uint16_t length);
    int8_t transmit_system_status(const void* data, uint16_t length);
    int8_t transmit_emergency_data(const void* data, uint16_t length);
    
    // Ground command handling
    int8_t process_ground_commands();
    int8_t send_command_acknowledgment(uint8_t command_id, bool success);
    int8_t request_ground_station_info();
    
    // Communication with master processor
    int8_t process_master_data();
    int8_t send_telemetry_status_to_master();
    int8_t forward_ground_command_to_master(GroundCommand command, const uint8_t* parameters);
    
    // Radio control
    int8_t set_radio_power(uint8_t power_percent);
    int8_t set_radio_frequency(uint32_t frequency);
    int8_t get_radio_rssi(int8_t& rssi);
    int8_t enable_radio_power_amplifier(bool enable);
    int8_t select_antenna(uint8_t antenna_id);
    
    // Data access
    const TelemetryControllerData& get_telemetry_data() const { return tel_data; }
    bool is_radio_ready() const { return tel_data.radio_status.radio_ready; }
    int8_t get_signal_strength() const { return tel_data.radio_status.rssi; }
    uint8_t get_link_quality() const { return tel_data.radio_status.link_quality; }
    float get_ground_distance() const { return tel_data.ground_station.ground_distance_km; }
    
    // Configuration management
    int8_t load_radio_profiles();
    int8_t save_radio_profiles();
    int8_t create_custom_radio_profile(const RadioProfile& profile, uint8_t slot);
    const RadioProfile& get_radio_profile(uint8_t profile_id) const;
    
    // Emergency functions
    int8_t send_emergency_beacon();
    int8_t activate_emergency_mode();
    int8_t send_abort_signal();
    
    // Diagnostics and testing
    int8_t run_radio_test();
    int8_t run_loopback_test();
    int8_t run_range_test();
    void print_telemetry_status() const;
    void print_radio_diagnostics() const;
    void set_status_led(uint8_t red, uint8_t green, uint8_t blue);
    
    // Statistics
    const TelemetryStats& get_statistics() const { return stats; }
    void reset_statistics();
    float get_packet_success_rate() const;
    uint32_t get_connection_uptime() const;
    
    // Power management
    int8_t set_low_power_mode(bool enable);
    int8_t optimize_power_consumption();
    
    // Callbacks
    void set_ground_command_callback(void (*callback)(GroundCommand command, const uint8_t* parameters));
    void set_connection_status_callback(void (*callback)(bool connected));
    void set_data_received_callback(void (*callback)(const TelemetryPacket& packet));
    void set_error_callback(void (*callback)(const char* error_msg));
    
private:
    // Update functions
    void update_radio_communication();
    void update_master_communication();
    void update_transmission_queue();
    void update_connection_status();
    void update_statistics();
    
    // Radio communication
    int8_t process_incoming_packets();
    int8_t transmit_queued_packets();
    int8_t send_heartbeat_packet();
    int8_t handle_received_packet(const TelemetryPacket& packet);
    
    // Command processing
    void execute_ground_command(GroundCommand command, const uint8_t* parameters);
    bool validate_ground_command(GroundCommand command, const uint8_t* parameters);
    
    // Queue management
    bool enqueue_packet(const TelemetryPacket& packet);
    bool dequeue_packet(TelemetryPacket& packet);
    void clear_transmission_queue();
    
    // Data processing
    void process_flight_data_from_master(const uint8_t* data, uint16_t length);
    void process_gps_data_from_master(const uint8_t* data, uint16_t length);
    void process_sensor_data_from_master(const uint8_t* data, uint16_t length);
    void update_master_data_timestamps();
    
    // SPI communication
    void handle_spi_received_data(const uint8_t* data, uint16_t length);
    int8_t send_spi_response(uint8_t response_type, const void* data, uint16_t length);
    
    // Radio management
    int8_t reset_radio_module();
    int8_t configure_radio_parameters();
    bool check_radio_health();
    void update_radio_status();
    
    // Connection management
    void monitor_connection_quality();
    void handle_connection_loss();
    void attempt_reconnection();
    
    // Error handling
    void handle_radio_error(const char* error_msg);
    void handle_communication_timeout();
    
    // Utility functions
    void initialize_pins();
    void configure_uart();
    void initialize_radio_profiles();
    uint8_t calculate_link_quality(int8_t rssi, uint32_t packet_loss_rate);
};

// Global instance
extern SAMD21TelemetryIntegration* g_samd21_telemetry;

// Utility functions and helpers
namespace TelemetryUtils {
    // Radio profiles
    extern const RadioProfile LONG_RANGE_PROFILE;      // LoRa 433MHz long range
    extern const RadioProfile HIGH_SPEED_PROFILE;      // 2.4GHz high speed
    extern const RadioProfile LOW_POWER_PROFILE;       // Low power consumption
    extern const RadioProfile EMERGENCY_PROFILE;       // Emergency beacon
    
    // Command validation
    bool is_command_safe_during_flight(GroundCommand command);
    bool validate_command_parameters(GroundCommand command, const uint8_t* parameters, uint8_t length);
    const char* command_to_string(GroundCommand command);
    
    // Link quality analysis
    struct LinkAnalysis {
        float signal_strength_dbm;
        float noise_floor_dbm;
        float snr_db;
        uint8_t link_margin_db;
        float packet_error_rate;
        bool link_stable;
    };
    
    LinkAnalysis analyze_link_quality(int8_t rssi, uint32_t packets_sent, uint32_t packets_lost);
    uint8_t calculate_link_score(const LinkAnalysis& analysis);
    
    // Frequency management
    bool is_frequency_in_ism_band(uint32_t frequency_hz);
    uint32_t get_backup_frequency(uint32_t primary_frequency);
    int8_t scan_frequency_range(uint32_t start_freq, uint32_t end_freq, uint32_t step_size);
    
    // Data compression for telemetry
    uint16_t compress_flight_data(const void* input, uint16_t input_size, void* output, uint16_t max_output_size);
    uint16_t decompress_flight_data(const void* input, uint16_t input_size, void* output, uint16_t max_output_size);
    
    // Antenna diversity
    enum AntennaType {
        ANTENNA_OMNIDIRECTIONAL = 0,
        ANTENNA_DIRECTIONAL = 1,
        ANTENNA_DIVERSITY = 2
    };
    
    int8_t select_best_antenna(AntennaType available_antennas[], uint8_t num_antennas);
    float calculate_antenna_gain_pattern(float azimuth_deg, float elevation_deg, AntennaType antenna_type);
    
    // Ground station communication
    struct GroundStationCapabilities {
        bool supports_file_transfer;
        bool supports_real_time_control;
        bool supports_mission_upload;
        bool supports_telemetry_relay;
        uint16_t max_packet_size;
        uint8_t supported_protocols;
    };
    
    int8_t negotiate_capabilities(GroundStationCapabilities& capabilities);
    bool is_capability_supported(const GroundStationCapabilities& caps, uint8_t capability);
    
    // Performance optimization
    void optimize_transmission_timing(uint8_t current_profile, float link_quality);
    uint16_t calculate_optimal_packet_size(int8_t rssi, float packet_error_rate);
    uint8_t select_optimal_data_rate(float distance_km, int8_t rssi);
    
    // Emergency protocols
    int8_t create_emergency_packet(uint8_t emergency_type, const void* data, uint8_t length, TelemetryPacket& packet);
    void activate_emergency_beacon_mode();
    void send_distress_signal(double latitude, double longitude, uint16_t emergency_code);
    
    // Configuration helpers
    void create_default_telemetry_config();
    int8_t validate_telemetry_config(const TelemetryConfig& config);
    void print_telemetry_config(const TelemetryConfig& config);
    
    // Data logging
    int8_t log_telemetry_session(const char* session_name);
    int8_t log_ground_command(GroundCommand command, bool success);
    int8_t log_connection_event(bool connected, uint32_t duration_ms);
}

#endif // SAMD21_TELEMETRY_INTEGRATION_H