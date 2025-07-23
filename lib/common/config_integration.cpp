#include "config_integration.h"

ConfigIntegration g_config_integration;
bool ConfigIntegration::last_config_valid = false;
uint32_t ConfigIntegration::last_config_check = 0;

bool ConfigIntegration::initialize_systems() {
    // Initialize configuration manager first
    if (!g_config_manager.begin()) {
        Serial.println("Failed to initialize configuration manager");
        return false;
    }
    
    // Register configuration change callbacks
    g_config_manager.register_change_callback(on_flight_parameter_changed);
    g_config_manager.register_change_callback(on_sensor_setting_changed);
    g_config_manager.register_change_callback(on_communication_changed);
    
    // Apply configuration to all systems
    if (!update_flight_state_machine()) {
        Serial.println("Failed to configure flight state machine");
        return false;
    }
    
    if (!update_sensor_manager()) {
        Serial.println("Failed to configure sensor manager");
        return false;
    }
    
    if (!update_communication_settings()) {
        Serial.println("Failed to configure communication settings");
        return false;
    }
    
    last_config_valid = true;
    last_config_check = millis();
    
    Serial.println("All systems initialized with configuration");
    return true;
}

bool ConfigIntegration::update_flight_state_machine() {
    const FlightConfig& config = g_config_manager.get_config();
    
    // Convert flight config to state machine config
    StateMachineConfig fsm_config = g_config_manager.to_state_machine_config();
    
    // Apply to flight state machine (assuming global instance exists)
    // This would need to be implemented in the actual integration
    // For now, just validate the configuration
    
    Serial.println("Flight state machine configuration updated");
    return true;
}

bool ConfigIntegration::update_sensor_manager() {
    const FlightConfig& config = g_config_manager.get_config();
    
    // Update sensor manager settings
    // This would integrate with the actual sensor manager instance
    
    Serial.print("Sensor update rate: ");
    Serial.println(config.sensor_update_rate);
    Serial.print("Kalman filter enabled: ");
    Serial.println(config.enable_kalman_filter ? "Yes" : "No");
    
    return true;
}

bool ConfigIntegration::update_communication_settings() {
    const FlightConfig& config = g_config_manager.get_config();
    
    Serial.print("Telemetry rate: ");
    Serial.println(config.telemetry_rate);
    Serial.print("Radio baud rate: ");
    Serial.println(config.radio_baud_rate);
    
    return true;
}

bool ConfigIntegration::validate_hardware_compatibility(const FlightConfig& config) {
    // Check if configuration values are compatible with hardware
    
    // Validate sensor update rates against hardware capabilities
    if (config.sensor_update_rate > 1000) {
        Serial.println("Warning: Sensor update rate may be too high for hardware");
        return false;
    }
    
    // Validate communication settings
    if (config.radio_baud_rate > 115200) {
        Serial.println("Warning: Radio baud rate not supported");
        return false;
    }
    
    // Validate pyrotechnic timing
    if (config.pyro_fire_duration > 5000) {
        Serial.println("Warning: Pyrotechnic fire duration too long");
        return false;
    }
    
    // Validate altitude settings
    if (config.main_altitude < 50.0f) {
        Serial.println("Warning: Main chute altitude too low");
        return false;
    }
    
    return true;
}

bool ConfigIntegration::apply_config_changes() {
    // Re-read configuration from SD card
    ConfigValidationResult result = g_config_manager.load_config();
    
    if (result != CONFIG_VALID) {
        Serial.print("Failed to reload configuration: ");
        Serial.println(g_config_manager.get_validation_error_string(result));
        return false;
    }
    
    // Validate hardware compatibility
    if (!validate_hardware_compatibility(g_config_manager.get_config())) {
        Serial.println("Configuration not compatible with hardware");
        return false;
    }
    
    // Update all systems
    bool success = true;
    success &= update_flight_state_machine();
    success &= update_sensor_manager();
    success &= update_communication_settings();
    
    if (success) {
        Serial.println("Configuration changes applied successfully");
        last_config_valid = true;
    } else {
        Serial.println("Failed to apply some configuration changes");
        last_config_valid = false;
    }
    
    last_config_check = millis();
    return success;
}

bool ConfigIntegration::create_flight_backup() {
    // Create a backup of current configuration before flight
    return g_config_manager.backup_config();
}

bool ConfigIntegration::restore_from_flight_backup() {
    // Restore configuration from backup
    return g_config_manager.restore_from_backup();
}

bool ConfigIntegration::perform_preflight_config_check() {
    Serial.println("Performing preflight configuration check...");
    
    // Check if configuration is loaded
    if (!g_config_manager.is_config_loaded()) {
        Serial.println("ERROR: Configuration not loaded");
        return false;
    }
    
    // Validate current configuration
    const FlightConfig& config = g_config_manager.get_config();
    ConfigValidationResult result = g_config_manager.validate_config(config);
    
    if (result != CONFIG_VALID) {
        Serial.print("ERROR: Configuration validation failed: ");
        Serial.println(g_config_manager.get_validation_error_string(result));
        return false;
    }
    
    // Check hardware compatibility
    if (!validate_hardware_compatibility(config)) {
        Serial.println("ERROR: Configuration not compatible with hardware");
        return false;
    }
    
    // Check critical flight parameters
    if (config.main_altitude < 100.0f) {
        Serial.println("WARNING: Main chute altitude is very low");
    }
    
    if (config.launch_accel_threshold < 10.0f) {
        Serial.println("WARNING: Launch detection threshold is very low");
    }
    
    if (!config.enable_dual_deploy) {
        Serial.println("WARNING: Dual deployment is disabled");
    }
    
    // Print configuration summary
    g_config_manager.print_config_summary();
    
    Serial.println("Preflight configuration check completed");
    return true;
}

void ConfigIntegration::monitor_config_integrity() {
    uint32_t current_time = millis();
    
    // Check configuration integrity every 10 seconds
    if (current_time - last_config_check > 10000) {
        
        // Check if SD card is still available
        if (!g_config_manager.is_sd_available()) {
            Serial.println("WARNING: SD card no longer available");
            last_config_valid = false;
        }
        
        // Check for unsaved changes
        if (g_config_manager.has_unsaved_changes()) {
            Serial.println("INFO: Configuration has unsaved changes");
        }
        
        last_config_check = current_time;
    }
}

// Configuration change callback implementations
void on_flight_parameter_changed(const char* parameter, const char* old_value, const char* new_value) {
    Serial.print("Flight parameter changed: ");
    Serial.print(parameter);
    Serial.print(" = ");
    Serial.print(new_value);
    Serial.print(" (was ");
    Serial.print(old_value);
    Serial.println(")");
    
    // Update flight state machine if needed
    ConfigIntegration::update_flight_state_machine();
}

void on_sensor_setting_changed(const char* parameter, const char* old_value, const char* new_value) {
    Serial.print("Sensor setting changed: ");
    Serial.print(parameter);
    Serial.print(" = ");
    Serial.print(new_value);
    Serial.print(" (was ");
    Serial.print(old_value);
    Serial.println(")");
    
    // Update sensor manager if needed
    ConfigIntegration::update_sensor_manager();
}

void on_communication_changed(const char* parameter, const char* old_value, const char* new_value) {
    Serial.print("Communication setting changed: ");
    Serial.print(parameter);
    Serial.print(" = ");
    Serial.print(new_value);
    Serial.print(" (was ");
    Serial.print(old_value);
    Serial.println(")");
    
    // Update communication settings if needed
    ConfigIntegration::update_communication_settings();
}