#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <Arduino.h>
#include <SD.h>
#include "flight_state_machine.h"
#include "spi_protocol.h"

// Configuration file paths
#define CONFIG_FILE_PATH "/config/flight_config.txt"
#define BACKUP_CONFIG_PATH "/config/flight_config_backup.txt"
#define LOG_CONFIG_PATH "/config/log_config.txt"
#define CALIBRATION_FILE_PATH "/config/calibration.txt"

// Configuration sections
enum ConfigSection {
    SECTION_FLIGHT_PARAMETERS,
    SECTION_SENSOR_SETTINGS,
    SECTION_COMMUNICATION,
    SECTION_RECOVERY_SYSTEM,
    SECTION_LOGGING,
    SECTION_CALIBRATION,
    SECTION_DEBUG,
    SECTION_COUNT
};

// Configuration validation results
enum ConfigValidationResult {
    CONFIG_VALID,
    CONFIG_FILE_NOT_FOUND,
    CONFIG_PARSE_ERROR,
    CONFIG_INVALID_VALUE,
    CONFIG_MISSING_REQUIRED,
    CONFIG_SD_ERROR
};

// Flight configuration structure
struct FlightConfig {
    // Flight parameters
    float launch_accel_threshold;        // m/s²
    float launch_altitude_threshold;     // m
    uint32_t launch_time_threshold;      // ms
    
    float burnout_accel_threshold;       // m/s²
    uint32_t burnout_time_threshold;     // ms
    
    float apogee_velocity_threshold;     // m/s
    float apogee_altitude_delta;         // m
    uint32_t apogee_time_threshold;      // ms
    
    uint32_t drogue_delay;               // ms
    float drogue_altitude;               // m
    
    float main_altitude;                 // m AGL
    uint32_t main_timeout;               // ms
    
    float landing_velocity_threshold;    // m/s
    float landing_altitude_delta;        // m
    uint32_t landing_time_threshold;     // ms
    
    uint32_t max_flight_time;            // ms
    uint32_t max_boost_time;             // ms
    uint32_t max_coast_time;             // ms
    
    // Sensor settings
    uint32_t sensor_update_rate;         // Hz
    float altitude_filter_alpha;         // 0.0 to 1.0
    float accel_filter_alpha;            // 0.0 to 1.0
    bool enable_kalman_filter;
    float kalman_process_noise;
    float kalman_measurement_noise;
    
    // Communication settings
    uint32_t telemetry_rate;             // Hz
    uint32_t radio_baud_rate;
    bool enable_ground_commands;
    uint32_t command_timeout;            // ms
    
    // Recovery system
    bool enable_dual_deploy;
    uint32_t pyro_fire_duration;         // ms
    float backup_timer_main;             // seconds
    float backup_timer_drogue;           // seconds
    bool enable_backup_altimeter;
    
    // Logging settings
    bool enable_sd_logging;
    uint32_t log_rate;                   // Hz
    bool log_raw_sensors;
    bool log_processed_data;
    bool log_flight_events;
    uint32_t max_log_file_size;          // bytes
    
    // Debug settings
    bool enable_serial_debug;
    uint32_t debug_baud_rate;
    bool verbose_logging;
    bool enable_test_mode;
    
    // Calibration values
    float ground_pressure_offset;        // hPa
    float accelerometer_bias[3];         // g
    float gyroscope_bias[3];             // rad/s
    float magnetometer_calibration[9];   // calibration matrix
};

// Configuration change callback type
typedef void (*config_change_callback_t)(const char* parameter, const char* old_value, const char* new_value);

class ConfigManager {
private:
    FlightConfig current_config;
    bool config_loaded;
    bool sd_available;
    uint8_t sd_cs_pin;
    
    // Configuration parsing
    char config_buffer[512];
    char* config_lines[100];
    int line_count;
    
    // Change tracking
    bool config_changed;
    uint32_t last_save_time;
    static const uint32_t AUTO_SAVE_INTERVAL = 30000; // 30 seconds
    
    // Callbacks
    config_change_callback_t change_callbacks[10];
    int callback_count;
    
public:
    ConfigManager(uint8_t sd_cs_pin);
    
    // Initialization
    bool begin();
    bool is_sd_available() const { return sd_available; }
    bool is_config_loaded() const { return config_loaded; }
    
    // Configuration loading and saving
    ConfigValidationResult load_config();
    ConfigValidationResult load_config_from_file(const char* filename);
    bool save_config();
    bool save_config_to_file(const char* filename);
    bool create_default_config();
    bool backup_config();
    bool restore_from_backup();
    
    // Configuration access
    const FlightConfig& get_config() const { return current_config; }
    bool set_config(const FlightConfig& new_config);
    
    // Individual parameter access
    bool get_float_parameter(const char* section, const char* key, float& value);
    bool get_int_parameter(const char* section, const char* key, int& value);
    bool get_bool_parameter(const char* section, const char* key, bool& value);
    bool get_string_parameter(const char* section, const char* key, char* value, size_t max_len);
    
    bool set_float_parameter(const char* section, const char* key, float value);
    bool set_int_parameter(const char* section, const char* key, int value);
    bool set_bool_parameter(const char* section, const char* key, bool value);
    bool set_string_parameter(const char* section, const char* key, const char* value);
    
    // Validation and conversion
    ConfigValidationResult validate_config(const FlightConfig& config);
    StateMachineConfig to_state_machine_config() const;
    bool from_state_machine_config(const StateMachineConfig& fsm_config);
    
    // Configuration updates
    void update();
    bool has_unsaved_changes() const { return config_changed; }
    void mark_changed() { config_changed = true; }
    
    // Callbacks
    void register_change_callback(config_change_callback_t callback);
    
    // Utility functions
    bool list_config_files(char file_list[][32], int max_files);
    bool delete_config_file(const char* filename);
    bool config_file_exists(const char* filename);
    size_t get_config_file_size(const char* filename);
    
    // Configuration templates
    bool create_test_config();
    bool create_simulation_config();
    bool create_minimal_config();
    
    // Error handling
    const char* get_validation_error_string(ConfigValidationResult result);
    void print_config_summary();
    bool verify_config_integrity();
    
private:
    // Internal parsing functions
    bool parse_config_file(File& file);
    bool parse_config_line(const char* line);
    bool parse_section_header(const char* line, char* section_name);
    bool parse_key_value_pair(const char* line, char* key, char* value);
    
    // Value conversion functions
    bool string_to_float(const char* str, float& value);
    bool string_to_int(const char* str, int& value);
    bool string_to_bool(const char* str, bool& value);
    void float_to_string(float value, char* str, int precision = 2);
    void int_to_string(int value, char* str);
    void bool_to_string(bool value, char* str);
    
    // Configuration writing functions
    bool write_config_section(File& file, const char* section_name);
    bool write_flight_parameters(File& file);
    bool write_sensor_settings(File& file);
    bool write_communication_settings(File& file);
    bool write_recovery_system(File& file);
    bool write_logging_settings(File& file);
    bool write_debug_settings(File& file);
    bool write_calibration_data(File& file);
    
    // Validation helpers
    bool validate_float_range(float value, float min_val, float max_val);
    bool validate_int_range(int value, int min_val, int max_val);
    bool validate_required_parameters();
    
    // File management
    bool ensure_config_directory();
    bool create_config_file_header(File& file);
    
    // Change notification
    void notify_parameter_changed(const char* parameter, const char* old_value, const char* new_value);
    
    // Default values
    void set_default_config();
};

// Global configuration manager instance
extern ConfigManager g_config_manager;

// Configuration constants
extern const FlightConfig DEFAULT_FLIGHT_CONFIG;

// Utility macros for configuration access
#define GET_CONFIG() (g_config_manager.get_config())
#define SET_CONFIG_FLOAT(section, key, value) (g_config_manager.set_float_parameter(section, key, value))
#define SET_CONFIG_INT(section, key, value) (g_config_manager.set_int_parameter(section, key, value))
#define SET_CONFIG_BOOL(section, key, value) (g_config_manager.set_bool_parameter(section, key, value))

#endif // CONFIG_MANAGER_H