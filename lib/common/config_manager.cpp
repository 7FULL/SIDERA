#include "config_manager.h"
#include <string.h>

ConfigManager g_config_manager(6); // SD_CARD_CS_PIN from platform_config.h

// Default configuration values
const FlightConfig DEFAULT_FLIGHT_CONFIG = {
    // Flight parameters
    .launch_accel_threshold = 20.0f,
    .launch_altitude_threshold = 10.0f,
    .launch_time_threshold = 500,
    
    .burnout_accel_threshold = 5.0f,
    .burnout_time_threshold = 1000,
    
    .apogee_velocity_threshold = 2.0f,
    .apogee_altitude_delta = 5.0f,
    .apogee_time_threshold = 1000,
    
    .drogue_delay = 1000,
    .drogue_altitude = 500.0f,
    
    .main_altitude = 150.0f,
    .main_timeout = 60000,
    
    .landing_velocity_threshold = 3.0f,
    .landing_altitude_delta = 2.0f,
    .landing_time_threshold = 5000,
    
    .max_flight_time = 300000,
    .max_boost_time = 10000,
    .max_coast_time = 120000,
    
    // Sensor settings
    .sensor_update_rate = 100,
    .altitude_filter_alpha = 0.2f,
    .accel_filter_alpha = 0.1f,
    .enable_kalman_filter = true,
    .kalman_process_noise = 0.1f,
    .kalman_measurement_noise = 2.0f,
    
    // Communication settings
    .telemetry_rate = 10,
    .radio_baud_rate = 9600,
    .enable_ground_commands = true,
    .command_timeout = 5000,
    
    // Recovery system
    .enable_dual_deploy = true,
    .pyro_fire_duration = 1000,
    .backup_timer_main = 60.0f,
    .backup_timer_drogue = 10.0f,
    .enable_backup_altimeter = false,
    
    // Logging settings
    .enable_sd_logging = true,
    .log_rate = 50,
    .log_raw_sensors = true,
    .log_processed_data = true,
    .log_flight_events = true,
    .max_log_file_size = 10485760, // 10MB
    
    // Debug settings
    .enable_serial_debug = true,
    .debug_baud_rate = 115200,
    .verbose_logging = false,
    .enable_test_mode = false,
    
    // Calibration values (defaults)
    .ground_pressure_offset = 0.0f,
    .accelerometer_bias = {0.0f, 0.0f, 0.0f},
    .gyroscope_bias = {0.0f, 0.0f, 0.0f},
    .magnetometer_calibration = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}
};

ConfigManager::ConfigManager(uint8_t cs_pin) {
    sd_cs_pin = cs_pin;
    config_loaded = false;
    sd_available = false;
    config_changed = false;
    last_save_time = 0;
    callback_count = 0;
    line_count = 0;
    
    set_default_config();
}

bool ConfigManager::begin() {
    // Initialize SD card
    if (!SD.begin(sd_cs_pin)) {
        Serial.println("SD card initialization failed!");
        sd_available = false;
        return false;
    }
    
    sd_available = true;
    
    // Ensure config directory exists
    if (!ensure_config_directory()) {
        Serial.println("Failed to create config directory");
        return false;
    }
    
    // Load configuration
    ConfigValidationResult result = load_config();
    
    if (result != CONFIG_VALID) {
        Serial.print("Config load failed: ");
        Serial.println(get_validation_error_string(result));
        
        // Create default config if file doesn't exist
        if (result == CONFIG_FILE_NOT_FOUND) {
            Serial.println("Creating default configuration file...");
            create_default_config();
        }
    }
    
    return true;
}

ConfigValidationResult ConfigManager::load_config() {
    return load_config_from_file(CONFIG_FILE_PATH);
}

ConfigValidationResult ConfigManager::load_config_from_file(const char* filename) {
    if (!sd_available) {
        return CONFIG_SD_ERROR;
    }
    
    if (!SD.exists(filename)) {
        return CONFIG_FILE_NOT_FOUND;
    }
    
    File config_file = SD.open(filename, FILE_READ);
    if (!config_file) {
        return CONFIG_SD_ERROR;
    }
    
    bool parse_success = parse_config_file(config_file);
    config_file.close();
    
    if (!parse_success) {
        return CONFIG_PARSE_ERROR;
    }
    
    ConfigValidationResult validation_result = validate_config(current_config);
    if (validation_result == CONFIG_VALID) {
        config_loaded = true;
        config_changed = false;
    }
    
    return validation_result;
}

bool ConfigManager::save_config() {
    return save_config_to_file(CONFIG_FILE_PATH);
}

bool ConfigManager::save_config_to_file(const char* filename) {
    if (!sd_available) {
        return false;
    }
    
    // Create backup of existing file
    if (SD.exists(filename)) {
        SD.remove(BACKUP_CONFIG_PATH);
        File source = SD.open(filename, FILE_READ);
        File backup = SD.open(BACKUP_CONFIG_PATH, FILE_WRITE);
        
        if (source && backup) {
            while (source.available()) {
                backup.write(source.read());
            }
        }
        
        if (source) source.close();
        if (backup) backup.close();
    }
    
    // Write new configuration
    File config_file = SD.open(filename, FILE_WRITE);
    if (!config_file) {
        return false;
    }
    
    bool write_success = true;
    write_success &= create_config_file_header(config_file);
    write_success &= write_flight_parameters(config_file);
    write_success &= write_sensor_settings(config_file);
    write_success &= write_communication_settings(config_file);
    write_success &= write_recovery_system(config_file);
    write_success &= write_logging_settings(config_file);
    write_success &= write_debug_settings(config_file);
    write_success &= write_calibration_data(config_file);
    
    config_file.close();
    
    if (write_success) {
        config_changed = false;
        last_save_time = millis();
    }
    
    return write_success;
}

bool ConfigManager::create_default_config() {
    set_default_config();
    return save_config();
}

bool ConfigManager::parse_config_file(File& file) {
    char current_section[32] = "";
    
    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        
        // Skip empty lines and comments
        if (line.length() == 0 || line.startsWith("#") || line.startsWith("//")) {
            continue;
        }
        
        // Check for section header
        if (line.startsWith("[") && line.endsWith("]")) {
            strcpy(current_section, line.substring(1, line.length() - 1).c_str());
            continue;
        }
        
        // Parse key-value pair
        int equals_pos = line.indexOf('=');
        if (equals_pos > 0) {
            String key = line.substring(0, equals_pos);
            String value = line.substring(equals_pos + 1);
            key.trim();
            value.trim();
            
            parse_parameter(current_section, key.c_str(), value.c_str());
        }
    }
    
    return true;
}

void ConfigManager::parse_parameter(const char* section, const char* key, const char* value) {
    if (strcmp(section, "flight_parameters") == 0) {
        if (strcmp(key, "launch_accel_threshold") == 0) string_to_float(value, current_config.launch_accel_threshold);
        else if (strcmp(key, "launch_altitude_threshold") == 0) string_to_float(value, current_config.launch_altitude_threshold);
        else if (strcmp(key, "launch_time_threshold") == 0) current_config.launch_time_threshold = atol(value);
        else if (strcmp(key, "burnout_accel_threshold") == 0) string_to_float(value, current_config.burnout_accel_threshold);
        else if (strcmp(key, "burnout_time_threshold") == 0) current_config.burnout_time_threshold = atol(value);
        else if (strcmp(key, "apogee_velocity_threshold") == 0) string_to_float(value, current_config.apogee_velocity_threshold);
        else if (strcmp(key, "apogee_altitude_delta") == 0) string_to_float(value, current_config.apogee_altitude_delta);
        else if (strcmp(key, "apogee_time_threshold") == 0) current_config.apogee_time_threshold = atol(value);
        else if (strcmp(key, "drogue_delay") == 0) current_config.drogue_delay = atol(value);
        else if (strcmp(key, "drogue_altitude") == 0) string_to_float(value, current_config.drogue_altitude);
        else if (strcmp(key, "main_altitude") == 0) string_to_float(value, current_config.main_altitude);
        else if (strcmp(key, "main_timeout") == 0) current_config.main_timeout = atol(value);
        else if (strcmp(key, "landing_velocity_threshold") == 0) string_to_float(value, current_config.landing_velocity_threshold);
        else if (strcmp(key, "landing_altitude_delta") == 0) string_to_float(value, current_config.landing_altitude_delta);
        else if (strcmp(key, "landing_time_threshold") == 0) current_config.landing_time_threshold = atol(value);
        else if (strcmp(key, "max_flight_time") == 0) current_config.max_flight_time = atol(value);
        else if (strcmp(key, "max_boost_time") == 0) current_config.max_boost_time = atol(value);
        else if (strcmp(key, "max_coast_time") == 0) current_config.max_coast_time = atol(value);
    }
    else if (strcmp(section, "sensor_settings") == 0) {
        if (strcmp(key, "sensor_update_rate") == 0) current_config.sensor_update_rate = atol(value);
        else if (strcmp(key, "altitude_filter_alpha") == 0) string_to_float(value, current_config.altitude_filter_alpha);
        else if (strcmp(key, "accel_filter_alpha") == 0) string_to_float(value, current_config.accel_filter_alpha);
        else if (strcmp(key, "enable_kalman_filter") == 0) string_to_bool(value, current_config.enable_kalman_filter);
        else if (strcmp(key, "kalman_process_noise") == 0) string_to_float(value, current_config.kalman_process_noise);
        else if (strcmp(key, "kalman_measurement_noise") == 0) string_to_float(value, current_config.kalman_measurement_noise);
    }
    else if (strcmp(section, "communication") == 0) {
        if (strcmp(key, "telemetry_rate") == 0) current_config.telemetry_rate = atol(value);
        else if (strcmp(key, "radio_baud_rate") == 0) current_config.radio_baud_rate = atol(value);
        else if (strcmp(key, "enable_ground_commands") == 0) string_to_bool(value, current_config.enable_ground_commands);
        else if (strcmp(key, "command_timeout") == 0) current_config.command_timeout = atol(value);
    }
    else if (strcmp(section, "recovery_system") == 0) {
        if (strcmp(key, "enable_dual_deploy") == 0) string_to_bool(value, current_config.enable_dual_deploy);
        else if (strcmp(key, "pyro_fire_duration") == 0) current_config.pyro_fire_duration = atol(value);
        else if (strcmp(key, "backup_timer_main") == 0) string_to_float(value, current_config.backup_timer_main);
        else if (strcmp(key, "backup_timer_drogue") == 0) string_to_float(value, current_config.backup_timer_drogue);
        else if (strcmp(key, "enable_backup_altimeter") == 0) string_to_bool(value, current_config.enable_backup_altimeter);
    }
    else if (strcmp(section, "logging") == 0) {
        if (strcmp(key, "enable_sd_logging") == 0) string_to_bool(value, current_config.enable_sd_logging);
        else if (strcmp(key, "log_rate") == 0) current_config.log_rate = atol(value);
        else if (strcmp(key, "log_raw_sensors") == 0) string_to_bool(value, current_config.log_raw_sensors);
        else if (strcmp(key, "log_processed_data") == 0) string_to_bool(value, current_config.log_processed_data);
        else if (strcmp(key, "log_flight_events") == 0) string_to_bool(value, current_config.log_flight_events);
        else if (strcmp(key, "max_log_file_size") == 0) current_config.max_log_file_size = atol(value);
    }
    else if (strcmp(section, "debug") == 0) {
        if (strcmp(key, "enable_serial_debug") == 0) string_to_bool(value, current_config.enable_serial_debug);
        else if (strcmp(key, "debug_baud_rate") == 0) current_config.debug_baud_rate = atol(value);
        else if (strcmp(key, "verbose_logging") == 0) string_to_bool(value, current_config.verbose_logging);
        else if (strcmp(key, "enable_test_mode") == 0) string_to_bool(value, current_config.enable_test_mode);
    }
}

bool ConfigManager::write_flight_parameters(File& file) {
    file.println("[flight_parameters]");
    file.println("# Launch detection parameters");
    file.print("launch_accel_threshold="); file.println(current_config.launch_accel_threshold);
    file.print("launch_altitude_threshold="); file.println(current_config.launch_altitude_threshold);
    file.print("launch_time_threshold="); file.println(current_config.launch_time_threshold);
    
    file.println("# Burnout detection parameters");
    file.print("burnout_accel_threshold="); file.println(current_config.burnout_accel_threshold);
    file.print("burnout_time_threshold="); file.println(current_config.burnout_time_threshold);
    
    file.println("# Apogee detection parameters");
    file.print("apogee_velocity_threshold="); file.println(current_config.apogee_velocity_threshold);
    file.print("apogee_altitude_delta="); file.println(current_config.apogee_altitude_delta);
    file.print("apogee_time_threshold="); file.println(current_config.apogee_time_threshold);
    
    file.println("# Recovery deployment parameters");
    file.print("drogue_delay="); file.println(current_config.drogue_delay);
    file.print("drogue_altitude="); file.println(current_config.drogue_altitude);
    file.print("main_altitude="); file.println(current_config.main_altitude);
    file.print("main_timeout="); file.println(current_config.main_timeout);
    
    file.println("# Landing detection parameters");
    file.print("landing_velocity_threshold="); file.println(current_config.landing_velocity_threshold);
    file.print("landing_altitude_delta="); file.println(current_config.landing_altitude_delta);
    file.print("landing_time_threshold="); file.println(current_config.landing_time_threshold);
    
    file.println("# Safety timeouts");
    file.print("max_flight_time="); file.println(current_config.max_flight_time);
    file.print("max_boost_time="); file.println(current_config.max_boost_time);
    file.print("max_coast_time="); file.println(current_config.max_coast_time);
    file.println();
    
    return true;
}

bool ConfigManager::write_sensor_settings(File& file) {
    file.println("[sensor_settings]");
    file.print("sensor_update_rate="); file.println(current_config.sensor_update_rate);
    file.print("altitude_filter_alpha="); file.println(current_config.altitude_filter_alpha);
    file.print("accel_filter_alpha="); file.println(current_config.accel_filter_alpha);
    file.print("enable_kalman_filter="); file.println(current_config.enable_kalman_filter ? "true" : "false");
    file.print("kalman_process_noise="); file.println(current_config.kalman_process_noise);
    file.print("kalman_measurement_noise="); file.println(current_config.kalman_measurement_noise);
    file.println();
    
    return true;
}

bool ConfigManager::write_communication_settings(File& file) {
    file.println("[communication]");
    file.print("telemetry_rate="); file.println(current_config.telemetry_rate);
    file.print("radio_baud_rate="); file.println(current_config.radio_baud_rate);
    file.print("enable_ground_commands="); file.println(current_config.enable_ground_commands ? "true" : "false");
    file.print("command_timeout="); file.println(current_config.command_timeout);
    file.println();
    
    return true;
}

bool ConfigManager::write_recovery_system(File& file) {
    file.println("[recovery_system]");
    file.print("enable_dual_deploy="); file.println(current_config.enable_dual_deploy ? "true" : "false");
    file.print("pyro_fire_duration="); file.println(current_config.pyro_fire_duration);
    file.print("backup_timer_main="); file.println(current_config.backup_timer_main);
    file.print("backup_timer_drogue="); file.println(current_config.backup_timer_drogue);
    file.print("enable_backup_altimeter="); file.println(current_config.enable_backup_altimeter ? "true" : "false");
    file.println();
    
    return true;
}

bool ConfigManager::write_logging_settings(File& file) {
    file.println("[logging]");
    file.print("enable_sd_logging="); file.println(current_config.enable_sd_logging ? "true" : "false");
    file.print("log_rate="); file.println(current_config.log_rate);
    file.print("log_raw_sensors="); file.println(current_config.log_raw_sensors ? "true" : "false");
    file.print("log_processed_data="); file.println(current_config.log_processed_data ? "true" : "false");
    file.print("log_flight_events="); file.println(current_config.log_flight_events ? "true" : "false");
    file.print("max_log_file_size="); file.println(current_config.max_log_file_size);
    file.println();
    
    return true;
}

bool ConfigManager::write_debug_settings(File& file) {
    file.println("[debug]");
    file.print("enable_serial_debug="); file.println(current_config.enable_serial_debug ? "true" : "false");
    file.print("debug_baud_rate="); file.println(current_config.debug_baud_rate);
    file.print("verbose_logging="); file.println(current_config.verbose_logging ? "true" : "false");
    file.print("enable_test_mode="); file.println(current_config.enable_test_mode ? "true" : "false");
    file.println();
    
    return true;
}

bool ConfigManager::write_calibration_data(File& file) {
    file.println("[calibration]");
    file.print("ground_pressure_offset="); file.println(current_config.ground_pressure_offset);
    file.print("accelerometer_bias_x="); file.println(current_config.accelerometer_bias[0]);
    file.print("accelerometer_bias_y="); file.println(current_config.accelerometer_bias[1]);
    file.print("accelerometer_bias_z="); file.println(current_config.accelerometer_bias[2]);
    file.print("gyroscope_bias_x="); file.println(current_config.gyroscope_bias[0]);
    file.print("gyroscope_bias_y="); file.println(current_config.gyroscope_bias[1]);
    file.print("gyroscope_bias_z="); file.println(current_config.gyroscope_bias[2]);
    file.println();
    
    return true;
}

ConfigValidationResult ConfigManager::validate_config(const FlightConfig& config) {
    // Validate flight parameters
    if (!validate_float_range(config.launch_accel_threshold, 5.0f, 100.0f)) return CONFIG_INVALID_VALUE;
    if (!validate_float_range(config.burnout_accel_threshold, 0.0f, 50.0f)) return CONFIG_INVALID_VALUE;
    if (!validate_float_range(config.main_altitude, 50.0f, 1000.0f)) return CONFIG_INVALID_VALUE;
    
    // Validate sensor settings
    if (!validate_int_range(config.sensor_update_rate, 1, 1000)) return CONFIG_INVALID_VALUE;
    if (!validate_float_range(config.altitude_filter_alpha, 0.0f, 1.0f)) return CONFIG_INVALID_VALUE;
    
    // Validate communication settings
    if (!validate_int_range(config.telemetry_rate, 1, 100)) return CONFIG_INVALID_VALUE;
    if (!validate_int_range(config.radio_baud_rate, 1200, 115200)) return CONFIG_INVALID_VALUE;
    
    return CONFIG_VALID;
}

StateMachineConfig ConfigManager::to_state_machine_config() const {
    StateMachineConfig fsm_config;
    
    fsm_config.launch_accel_threshold = current_config.launch_accel_threshold;
    fsm_config.launch_altitude_threshold = current_config.launch_altitude_threshold;
    fsm_config.launch_time_threshold = current_config.launch_time_threshold;
    
    fsm_config.burnout_accel_threshold = current_config.burnout_accel_threshold;
    fsm_config.burnout_time_threshold = current_config.burnout_time_threshold;
    
    fsm_config.apogee_velocity_threshold = current_config.apogee_velocity_threshold;
    fsm_config.apogee_altitude_delta = current_config.apogee_altitude_delta;
    fsm_config.apogee_time_threshold = current_config.apogee_time_threshold;
    
    fsm_config.drogue_delay = current_config.drogue_delay;
    fsm_config.drogue_altitude = current_config.drogue_altitude;
    
    fsm_config.main_altitude = current_config.main_altitude;
    fsm_config.main_timeout = current_config.main_timeout;
    
    fsm_config.landing_velocity_threshold = current_config.landing_velocity_threshold;
    fsm_config.landing_altitude_delta = current_config.landing_altitude_delta;
    fsm_config.landing_time_threshold = current_config.landing_time_threshold;
    
    fsm_config.max_flight_time = current_config.max_flight_time;
    fsm_config.max_boost_time = current_config.max_boost_time;
    fsm_config.max_coast_time = current_config.max_coast_time;
    
    return fsm_config;
}

void ConfigManager::update() {
    // Auto-save if changes pending and enough time has passed
    if (config_changed && (millis() - last_save_time) > AUTO_SAVE_INTERVAL) {
        save_config();
    }
}

// Utility functions
bool ConfigManager::string_to_float(const char* str, float& value) {
    char* endptr;
    float result = strtof(str, &endptr);
    if (endptr != str && *endptr == '\0') {
        value = result;
        return true;
    }
    return false;
}

bool ConfigManager::string_to_bool(const char* str, bool& value) {
    if (strcasecmp(str, "true") == 0 || strcmp(str, "1") == 0) {
        value = true;
        return true;
    } else if (strcasecmp(str, "false") == 0 || strcmp(str, "0") == 0) {
        value = false;
        return true;
    }
    return false;
}

bool ConfigManager::validate_float_range(float value, float min_val, float max_val) {
    return (value >= min_val && value <= max_val);
}

bool ConfigManager::validate_int_range(int value, int min_val, int max_val) {
    return (value >= min_val && value <= max_val);
}

bool ConfigManager::ensure_config_directory() {
    if (!SD.exists("/config")) {
        return SD.mkdir("/config");
    }
    return true;
}

bool ConfigManager::create_config_file_header(File& file) {
    file.println("# Rocket Flight Computer Configuration File");
    file.println("# Generated automatically - Edit with caution");
    file.print("# Generated on: ");
    file.println(millis());
    file.println("#");
    file.println("# Format: key=value");
    file.println("# Comments start with # or //");
    file.println("#");
    file.println();
    
    return true;
}

const char* ConfigManager::get_validation_error_string(ConfigValidationResult result) {
    switch (result) {
        case CONFIG_VALID: return "Configuration is valid";
        case CONFIG_FILE_NOT_FOUND: return "Configuration file not found";
        case CONFIG_PARSE_ERROR: return "Failed to parse configuration file";
        case CONFIG_INVALID_VALUE: return "Invalid parameter value";
        case CONFIG_MISSING_REQUIRED: return "Missing required parameter";
        case CONFIG_SD_ERROR: return "SD card error";
        default: return "Unknown error";
    }
}

void ConfigManager::set_default_config() {
    current_config = DEFAULT_FLIGHT_CONFIG;
    config_changed = true;
}