#include <Arduino.h>
#include "mk20_sensor_integration.h"
#include "../../../lib/common/config_manager.h"

// Global instances
MK20SensorIntegration* mk20_sensors = nullptr;
ConfigManager* config_manager = nullptr;

// Timing variables
uint32_t last_status_print = 0;
uint32_t last_led_update = 0;
uint32_t system_start_time = 0;

// System status
bool system_ready = false;
bool emergency_mode = false;
uint8_t led_pattern = 0;

// Function prototypes
void setup();
void loop();
void initialize_system();
void update_system();
void update_status_leds();
void print_system_status();
void handle_flight_state_change(FlightState old_state, FlightState new_state);
void handle_sensor_error(const char* sensor_name, int8_t error_code);
void handle_data_ready(const MK20SensorData& data);
void emergency_shutdown();

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait for serial connection or timeout after 3 seconds
    }
    
    Serial.println("=== MK20DX256VLH7 Rocket Flight Computer Master ===");
    Serial.println("Initializing system...");
    
    system_start_time = millis();
    
    // Initialize system components
    initialize_system();
    
    Serial.println("System initialization complete.");
    Serial.print("Total initialization time: ");
    Serial.print(millis() - system_start_time);
    Serial.println(" ms");
}

void loop() {
    // Main system update
    update_system();
    
    // Update status LEDs
    update_status_leds();
    
    // Print system status every 5 seconds
    if (millis() - last_status_print >= 5000) {
        print_system_status();
        last_status_print = millis();
    }
    
    // Handle serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "status") {
            print_system_status();
        } else if (command == "arm") {
            if (mk20_sensors) {
                mk20_sensors->arm_flight_system();
                Serial.println("Flight system armed");
            }
        } else if (command == "disarm") {
            if (mk20_sensors) {
                mk20_sensors->disarm_flight_system();
                Serial.println("Flight system disarmed");
            }
        } else if (command == "test") {
            if (mk20_sensors) {
                Serial.println("Running sensor test...");
                mk20_sensors->run_sensor_test();
            }
        } else if (command == "emergency") {
            emergency_shutdown();
        } else if (command == "reset") {
            Serial.println("Resetting system...");
            delay(100);
            // Software reset (platform specific)
            SCB_AIRCR = 0x05FA0004; // ARM Cortex-M4 reset
        } else {
            Serial.println("Unknown command. Available commands:");
            Serial.println("  status  - Print system status");
            Serial.println("  arm     - Arm flight system");
            Serial.println("  disarm  - Disarm flight system");
            Serial.println("  test    - Run sensor test");
            Serial.println("  emergency - Emergency shutdown");
            Serial.println("  reset   - Reset system");
        }
    }
    
    // Check for emergency conditions
    if (!emergency_mode && mk20_sensors) {
        const MK20SensorData& data = mk20_sensors->get_sensor_data();
        
        // Check battery voltage
        if (data.system.battery_voltage < 6.0f) {
            Serial.println("WARNING: Low battery voltage detected!");
        }
        
        // Check for system errors
        if (!mk20_sensors->is_initialized()) {
            Serial.println("ERROR: Sensor system not initialized!");
            emergency_mode = true;
        }
    }
    
    // Small delay to prevent overwhelming the system
    delay(1);
}

void initialize_system() {
    // Initialize configuration manager first (with SD card CS pin)
    config_manager = new ConfigManager(4); // Using pin 4 for SD card CS
    if (!config_manager->begin()) {
        Serial.println("WARNING: Configuration manager initialization failed");
    } else {
        // Load configuration from SD card
        if (config_manager->load_config_from_file("flight_config.txt") == 0) {
            Serial.println("Configuration loaded from SD card");
        } else {
            Serial.println("Using default configuration");
        }
    }
    
    // Initialize sensor integration
    mk20_sensors = new MK20SensorIntegration();
    g_mk20_sensors = mk20_sensors; // Set global pointer
    
    if (mk20_sensors->begin() != 0) {
        Serial.println("ERROR: Sensor integration initialization failed!");
        emergency_mode = true;
        return;
    }
    
    // Set up callbacks
    mk20_sensors->set_flight_state_callback(handle_flight_state_change);
    mk20_sensors->set_sensor_error_callback(handle_sensor_error);
    mk20_sensors->set_data_ready_callback(handle_data_ready);
    
    // Load flight configuration if available
    if (config_manager && config_manager->is_config_loaded()) {
        // Apply configuration parameters
        float main_deploy_alt = 300.0f;
        float launch_threshold = 3.0f;
        
        // Get parameters with correct function signature
        config_manager->get_float_parameter("FLIGHT", "MAIN_DEPLOY_ALTITUDE", main_deploy_alt);
        config_manager->get_float_parameter("FLIGHT", "LAUNCH_THRESHOLD_G", launch_threshold);
        
        mk20_sensors->set_main_deploy_altitude(main_deploy_alt);
        mk20_sensors->set_launch_threshold(launch_threshold);
        
        Serial.print("Main deploy altitude: ");
        Serial.print(main_deploy_alt);
        Serial.println(" m");
        Serial.print("Launch threshold: ");
        Serial.print(launch_threshold);
        Serial.println(" g");
    }
    
    // Calibrate ground altitude
    Serial.println("Calibrating ground altitude...");
    mk20_sensors->calibrate_ground_altitude();
    
    system_ready = true;
    Serial.println("MK20 Master system ready");
}

void update_system() {
    if (!system_ready || emergency_mode) {
        return;
    }
    
    // Update sensor integration (this handles all sensors and state machine)
    if (mk20_sensors) {
        mk20_sensors->update();
    }
}

void update_status_leds() {
    if (millis() - last_led_update < 100) return; // Update at 10Hz
    
    if (emergency_mode) {
        // Fast red blink in emergency mode
        bool led_state = (millis() / 100) % 2;
        if (mk20_sensors) {
            mk20_sensors->set_status_led(led_state ? 255 : 0, 0, 0);
        }
    } else if (!system_ready) {
        // Orange pulse during initialization
        uint8_t brightness = (sin((millis() / 500.0f) * 2 * PI) + 1) * 127.5f;
        if (mk20_sensors) {
            mk20_sensors->set_status_led(brightness, brightness / 2, 0);
        }
    } else if (mk20_sensors) {
        FlightState state = mk20_sensors->get_flight_state();
        
        switch (state) {
            case FLIGHT_STATE_IDLE:
                // Slow green pulse - system ready
                {
                    uint8_t brightness = (sin((millis() / 1000.0f) * 2 * PI) + 1) * 127.5f;
                    mk20_sensors->set_status_led(0, brightness, 0);
                }
                break;
                
            case FLIGHT_STATE_ARMED:
                // Alternating green/blue - armed
                {
                    bool toggle = (millis() / 500) % 2;
                    mk20_sensors->set_status_led(0, toggle ? 255 : 0, toggle ? 0 : 255);
                }
                break;
                
            case FLIGHT_STATE_BOOST:
                // Bright white - boost phase
                mk20_sensors->set_status_led(255, 255, 255);
                break;
                
            case FLIGHT_STATE_COAST:
                // Steady blue - coast phase
                mk20_sensors->set_status_led(0, 0, 255);
                break;
                
            case FLIGHT_STATE_APOGEE:
                // Purple pulse - apogee
                {
                    uint8_t brightness = (sin((millis() / 200.0f) * 2 * PI) + 1) * 127.5f;
                    mk20_sensors->set_status_led(brightness, 0, brightness);
                }
                break;
                
            case FLIGHT_STATE_DROGUE:
                // Cyan - drogue descent
                mk20_sensors->set_status_led(0, 255, 255);
                break;
                
            case FLIGHT_STATE_MAIN:
                // Yellow - main descent
                mk20_sensors->set_status_led(255, 255, 0);
                break;
                
            case FLIGHT_STATE_LANDED:
                // Slow white pulse - landed
                {
                    uint8_t brightness = (sin((millis() / 2000.0f) * 2 * PI) + 1) * 127.5f;
                    mk20_sensors->set_status_led(brightness, brightness, brightness);
                }
                break;
                
            case FLIGHT_STATE_ERROR:
                // Fast red blink - error
                {
                    bool led_state = (millis() / 200) % 2;
                    mk20_sensors->set_status_led(led_state ? 255 : 0, 0, 0);
                }
                break;
        }
    }
    
    last_led_update = millis();
}

void print_system_status() {
    if (!mk20_sensors) return;
    
    Serial.println("\n=== SYSTEM STATUS ===");
    Serial.print("Uptime: ");
    Serial.print((millis() - system_start_time) / 1000);
    Serial.println(" seconds");
    
    Serial.print("System Ready: ");
    Serial.println(system_ready ? "YES" : "NO");
    
    Serial.print("Emergency Mode: ");
    Serial.println(emergency_mode ? "YES" : "NO");
    
    const MK20SensorData& data = mk20_sensors->get_sensor_data();
    
    // Flight state
    Serial.print("Flight State: ");
    switch (mk20_sensors->get_flight_state()) {
        case FLIGHT_STATE_IDLE: Serial.println("IDLE"); break;
        case FLIGHT_STATE_ARMED: Serial.println("ARMED"); break;
        case FLIGHT_STATE_BOOST: Serial.println("BOOST"); break;
        case FLIGHT_STATE_COAST: Serial.println("COAST"); break;
        case FLIGHT_STATE_APOGEE: Serial.println("APOGEE"); break;
        case FLIGHT_STATE_DROGUE: Serial.println("DROGUE_DESCENT"); break;
        case FLIGHT_STATE_MAIN: Serial.println("MAIN_DESCENT"); break;
        case FLIGHT_STATE_LANDED: Serial.println("LANDED"); break;
        case FLIGHT_STATE_ERROR: Serial.println("ERROR"); break;
    }
    
    // Sensors
    Serial.println("\n--- SENSOR DATA ---");
    if (data.high_g_accel.data_valid) {
        Serial.print("High-G Accel: ");
        Serial.print(data.high_g_accel.accel_x, 2);
        Serial.print(", ");
        Serial.print(data.high_g_accel.accel_y, 2);
        Serial.print(", ");
        Serial.print(data.high_g_accel.accel_z, 2);
        Serial.println(" g");
    } else {
        Serial.println("High-G Accel: INVALID");
    }
    
    if (data.barometer.data_valid) {
        Serial.print("Altitude: ");
        Serial.print(data.barometer.altitude, 2);
        Serial.print(" m, Pressure: ");
        Serial.print(data.barometer.pressure, 1);
        Serial.print(" Pa, Temp: ");
        Serial.print(data.barometer.temperature, 1);
        Serial.println(" °C");
    } else {
        Serial.println("Barometer: INVALID");
    }
    
    if (data.gps.data_valid) {
        Serial.print("GPS: ");
        Serial.print(data.gps.latitude, 6);
        Serial.print(", ");
        Serial.print(data.gps.longitude, 6);
        Serial.print(", Alt: ");
        Serial.print(data.gps.altitude_gps, 1);
        Serial.print(" m, Sats: ");
        Serial.println(data.gps.satellites);
    } else {
        Serial.println("GPS: NO FIX");
    }
    
    // System status
    Serial.println("\n--- SYSTEM STATUS ---");
    Serial.print("Battery: ");
    Serial.print(data.system.battery_voltage, 2);
    Serial.print(" V (");
    Serial.print(data.system.battery_percentage);
    Serial.println("%)");
    
    Serial.print("Free Memory: ");
    Serial.print(data.system.free_memory);
    Serial.println(" bytes");
    
    Serial.print("SD Card: ");
    Serial.println(data.system.sd_card_ready ? "READY" : "NOT READY");
    
    Serial.print("SPI Flash: ");
    Serial.println(data.system.flash_ready ? "READY" : "NOT READY");
    
    Serial.print("Pyro Armed: ");
    Serial.println(data.system.pyro_armed ? "YES" : "NO");
    
    Serial.print("Logging: ");
    Serial.println(mk20_sensors->is_logging_active() ? "ACTIVE" : "INACTIVE");
    
    Serial.println("========================\n");
}

void handle_flight_state_change(FlightState old_state, FlightState new_state) {
    Serial.print("Flight state changed: ");
    Serial.print((int)old_state);
    Serial.print(" -> ");
    Serial.println((int)new_state);
    
    // Handle specific state transitions
    switch (new_state) {
        case FLIGHT_STATE_ARMED:
            Serial.println("*** FLIGHT SYSTEM ARMED ***");
            break;
            
        case FLIGHT_STATE_BOOST:
            Serial.println("*** LAUNCH DETECTED ***");
            break;
            
        case FLIGHT_STATE_APOGEE:
            Serial.println("*** APOGEE REACHED - DEPLOYING DROGUE ***");
            break;
            
        case FLIGHT_STATE_DROGUE:
            Serial.println("*** DEPLOYING MAIN PARACHUTE ***");
            break;
            
        case FLIGHT_STATE_LANDED:
            Serial.println("*** LANDING DETECTED ***");
            break;
            
        case FLIGHT_STATE_ERROR:
            Serial.println("*** ERROR STATE ENTERED ***");
            emergency_mode = true;
            break;
    }
}

void handle_sensor_error(const char* sensor_name, int8_t error_code) {
    Serial.print("SENSOR ERROR: ");
    Serial.print(sensor_name);
    Serial.print(" failed with code ");
    Serial.println(error_code);
    
    // Log error to SD card if available
    // (This would be handled by the sensor integration class)
}

void handle_data_ready(const MK20SensorData& data) {
    // This callback is called when new sensor data is available
    // Can be used for real-time data processing or logging
    
    // Example: Check for anomalous readings
    if (data.high_g_accel.data_valid) {
        float total_g = sqrt(data.high_g_accel.accel_x * data.high_g_accel.accel_x +
                           data.high_g_accel.accel_y * data.high_g_accel.accel_y +
                           data.high_g_accel.accel_z * data.high_g_accel.accel_z);
        
        if (total_g > 50.0f) { // Very high G detected
            Serial.print("HIGH G WARNING: ");
            Serial.print(total_g, 1);
            Serial.println(" g");
        }
    }
}

void emergency_shutdown() {
    Serial.println("*** EMERGENCY SHUTDOWN INITIATED ***");
    
    emergency_mode = true;
    
    // Disarm flight system
    if (mk20_sensors) {
        mk20_sensors->emergency_shutdown();
        mk20_sensors->set_status_led(255, 0, 0); // Red LED
    }
    
    // Stop data logging
    if (mk20_sensors && mk20_sensors->is_logging_active()) {
        mk20_sensors->stop_logging();
    }
    
    Serial.println("Emergency shutdown complete. System in safe mode.");
}