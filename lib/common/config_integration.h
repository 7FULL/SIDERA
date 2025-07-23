#ifndef CONFIG_INTEGRATION_H
#define CONFIG_INTEGRATION_H

#include "config_manager.h"
#include "flight_state_machine.h"
#include "sensor_manager.h"

// Configuration integration utilities
class ConfigIntegration {
public:
    // Initialize all systems with configuration
    static bool initialize_systems();
    
    // Update systems when configuration changes
    static bool update_flight_state_machine();
    static bool update_sensor_manager();
    static bool update_communication_settings();
    
    // Configuration validation specific to hardware
    static bool validate_hardware_compatibility(const FlightConfig& config);
    
    // Apply runtime configuration changes
    static bool apply_config_changes();
    
    // Configuration backup and recovery
    static bool create_flight_backup();
    static bool restore_from_flight_backup();
    
    // Pre-flight configuration check
    static bool perform_preflight_config_check();
    
    // Configuration monitoring during flight
    static void monitor_config_integrity();
    
private:
    static bool last_config_valid;
    static uint32_t last_config_check;
};

// Configuration change handlers
void on_flight_parameter_changed(const char* parameter, const char* old_value, const char* new_value);
void on_sensor_setting_changed(const char* parameter, const char* old_value, const char* new_value);
void on_communication_changed(const char* parameter, const char* old_value, const char* new_value);

// Global configuration integration instance
extern ConfigIntegration g_config_integration;

#endif // CONFIG_INTEGRATION_H