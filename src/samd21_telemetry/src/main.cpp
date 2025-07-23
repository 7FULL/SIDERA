#include <Arduino.h>
#include "samd21_telemetry_integration.h"

// Global instances
SAMD21TelemetryIntegration* telemetry_system = nullptr;

// Timing variables
uint32_t last_status_print = 0;
uint32_t last_led_update = 0;
uint32_t system_start_time = 0;

// System status
bool system_ready = false;
bool emergency_mode = false;
bool radio_connected = false;

// Function prototypes
void setup();
void loop();
void initialize_system();
void update_system();
void update_status_leds();
void print_system_status();
void handle_ground_command(GroundCommand command, const uint8_t* parameters);
void handle_connection_status(bool connected);
void handle_data_received(const TelemetryPacket& packet);
void handle_telemetry_error(const char* error_msg);
void emergency_shutdown();

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait for serial connection or timeout after 3 seconds
    }
    
    Serial.println("=== SAMD21G18A-AU Rocket Flight Computer Telemetry ===");
    Serial.println("Initializing telemetry system...");
    
    system_start_time = millis();
    
    // Initialize system components
    initialize_system();
    
    Serial.println("Telemetry system initialization complete.");
    Serial.print("Total initialization time: ");
    Serial.print(millis() - system_start_time);
    Serial.println(" ms");
}

void loop() {
    // Main system update
    update_system();
    
    // Update status LEDs
    update_status_leds();
    
    // Print system status every 15 seconds
    if (millis() - last_status_print >= 15000) {
        print_system_status();
        last_status_print = millis();
    }
    
    // Handle serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "status") {
            print_system_status();
        } else if (command == "connect") {
            if (telemetry_system) {
                Serial.println("Attempting to connect to ground station...");
                telemetry_system->connect_to_ground_station();
            }
        } else if (command == "disconnect") {
            if (telemetry_system) {
                telemetry_system->disconnect_from_ground_station();
                Serial.println("Disconnected from ground station");
            }
        } else if (command == "scan") {
            if (telemetry_system) {
                Serial.println("Scanning for ground stations...");
                telemetry_system->scan_for_ground_stations();
            }
        } else if (command == "test") {
            if (telemetry_system) {
                Serial.println("Running telemetry test...");
                telemetry_system->run_radio_test();
                telemetry_system->run_loopback_test();
            }
        } else if (command == "emergency") {
            emergency_shutdown();
        } else if (command.startsWith("profile ")) {
            String profile_str = command.substring(8);
            uint8_t profile_id = profile_str.toInt();
            if (telemetry_system && profile_id < 4) {
                telemetry_system->switch_radio_profile(profile_id);
                Serial.print("Switched to radio profile ");
                Serial.println(profile_id);
            } else {
                Serial.println("Invalid profile. Available: 0-3");
            }
        } else if (command.startsWith("power ")) {
            String power_str = command.substring(6);
            uint8_t power_level = power_str.toInt();
            if (telemetry_system && power_level <= 100) {
                telemetry_system->set_radio_power(power_level);
                Serial.print("Radio power set to ");
                Serial.print(power_level);
                Serial.println("%");
            } else {
                Serial.println("Invalid power level. Range: 0-100");
            }
        } else if (command == "beacon") {
            if (telemetry_system) {
                telemetry_system->send_emergency_beacon();
                Serial.println("Emergency beacon sent");
            }
        } else if (command == "stats") {
            if (telemetry_system) {
                telemetry_system->print_telemetry_status();
            }
        } else {
            Serial.println("Unknown command. Available commands:");
            Serial.println("  status           - Print system status");
            Serial.println("  connect          - Connect to ground station");
            Serial.println("  disconnect       - Disconnect from ground station");
            Serial.println("  scan             - Scan for ground stations");
            Serial.println("  test             - Run system test");
            Serial.println("  emergency        - Emergency shutdown");
            Serial.println("  profile <0-3>    - Switch radio profile");
            Serial.println("  power <0-100>    - Set radio power level");
            Serial.println("  beacon           - Send emergency beacon");
            Serial.println("  stats            - Print telemetry statistics");
        }
    }
    
    // Check for emergency conditions
    if (!emergency_mode && telemetry_system) {
        // Check radio health
        if (!telemetry_system->is_radio_ready()) {
            Serial.println("WARNING: Radio not ready!");
        }
        
        // Check for system errors
        if (!telemetry_system->is_initialized()) {
            Serial.println("ERROR: Telemetry system not initialized!");
            emergency_mode = true;
        }
    }
    
    // Small delay to prevent overwhelming the system
    delay(1);
}

void initialize_system() {
    // Initialize telemetry integration
    telemetry_system = new SAMD21TelemetryIntegration();
    g_samd21_telemetry = telemetry_system; // Set global pointer
    
    if (telemetry_system->begin() != 0) {
        Serial.println("ERROR: Telemetry system initialization failed!");
        emergency_mode = true;
        return;
    }
    
    // Set up callbacks
    telemetry_system->set_ground_command_callback(handle_ground_command);
    telemetry_system->set_connection_status_callback(handle_connection_status);
    telemetry_system->set_data_received_callback(handle_data_received);
    telemetry_system->set_error_callback(handle_telemetry_error);
    
    // Attempt to connect to ground station
    Serial.println("Attempting to connect to ground station...");
    telemetry_system->connect_to_ground_station();
    
    system_ready = true;
    Serial.println("SAMD21 Telemetry system ready");
}

void update_system() {
    if (!system_ready || emergency_mode) {
        return;
    }
    
    // Update telemetry system (this handles radio communication and data processing)
    if (telemetry_system) {
        telemetry_system->update();
        
        // Update connection status
        radio_connected = telemetry_system->is_connected_to_ground();
    }
}

void update_status_leds() {
    if (millis() - last_led_update < 100) return; // Update at 10Hz
    
    if (emergency_mode) {
        // Fast red blink in emergency mode
        bool led_state = (millis() / 100) % 2;
        if (telemetry_system) {
            telemetry_system->set_status_led(led_state ? 255 : 0, 0, 0);
        }
    } else if (!system_ready) {
        // Orange pulse during initialization
        uint8_t brightness = (sin((millis() / 500.0f) * 2 * PI) + 1) * 127.5f;
        if (telemetry_system) {
            telemetry_system->set_status_led(brightness, brightness / 2, 0);
        }
    } else if (telemetry_system) {
        bool radio_ready = telemetry_system->is_radio_ready();
        int8_t signal_strength = telemetry_system->get_signal_strength();
        uint8_t link_quality = telemetry_system->get_link_quality();
        
        if (!radio_ready) {
            // Red pulse - radio not ready
            uint8_t brightness = (sin((millis() / 500.0f) * 2 * PI) + 1) * 127.5f;
            telemetry_system->set_status_led(brightness, 0, 0);
        } else if (radio_connected) {
            // Green intensity based on signal strength and link quality
            uint8_t green_intensity = 0;
            uint8_t red_intensity = 0;
            
            if (link_quality > 80) {
                // Excellent link - bright green
                green_intensity = 255;
            } else if (link_quality > 60) {
                // Good link - medium green
                green_intensity = 180;
            } else if (link_quality > 40) {
                // Fair link - yellow (green + red)
                green_intensity = 150;
                red_intensity = 100;
            } else if (link_quality > 20) {
                // Poor link - orange
                green_intensity = 100;
                red_intensity = 200;
            } else {
                // Very poor link - red
                red_intensity = 255;
            }
            
            telemetry_system->set_status_led(red_intensity, green_intensity, 0);
        } else {
            // Blue pulse - radio ready but not connected
            uint8_t brightness = (sin((millis() / 1000.0f) * 2 * PI) + 1) * 127.5f;
            telemetry_system->set_status_led(0, 0, brightness);
        }
    }
    
    last_led_update = millis();
}

void print_system_status() {
    if (!telemetry_system) return;
    
    Serial.println("\n=== TELEMETRY SYSTEM STATUS ===");
    Serial.print("Uptime: ");
    Serial.print((millis() - system_start_time) / 1000);
    Serial.println(" seconds");
    
    Serial.print("System Ready: ");
    Serial.println(system_ready ? "YES" : "NO");
    
    Serial.print("Emergency Mode: ");
    Serial.println(emergency_mode ? "YES" : "NO");
    
    const TelemetryControllerData& data = telemetry_system->get_telemetry_data();
    
    // Radio status
    Serial.println("\n--- RADIO STATUS ---");
    Serial.print("Radio Ready: ");
    Serial.println(data.radio_status.radio_ready ? "YES" : "NO");
    
    Serial.print("Connected to Ground: ");
    Serial.println(data.radio_status.connected_to_ground ? "YES" : "NO");
    
    Serial.print("Signal Strength (RSSI): ");
    Serial.print(data.radio_status.rssi);
    Serial.println(" dBm");
    
    Serial.print("Link Quality: ");
    Serial.print(data.radio_status.link_quality);
    Serial.println("%");
    
    Serial.print("Packets Sent: ");
    Serial.println(data.radio_status.packets_sent);
    
    Serial.print("Packets Received: ");
    Serial.println(data.radio_status.packets_received);
    
    Serial.print("Packets Lost: ");
    Serial.println(data.radio_status.packets_lost);
    
    if (data.radio_status.packets_sent > 0) {
        float success_rate = ((float)(data.radio_status.packets_sent - data.radio_status.packets_lost) / 
                             data.radio_status.packets_sent) * 100.0f;
        Serial.print("Success Rate: ");
        Serial.print(success_rate, 1);
        Serial.println("%");
    }
    
    // Ground station info
    if (data.radio_status.connected_to_ground) {
        Serial.println("\n--- GROUND STATION ---");
        Serial.print("Station ID: ");
        Serial.println(data.ground_station.ground_station_id);
        
        Serial.print("Call Sign: ");
        Serial.println(data.ground_station.ground_call_sign);
        
        Serial.print("Distance: ");
        Serial.print(data.ground_station.ground_distance_km, 2);
        Serial.println(" km");
        
        Serial.print("Ground RSSI: ");
        Serial.print(data.ground_station.ground_rssi);
        Serial.println(" dBm");
        
        Serial.print("Commands Enabled: ");
        Serial.println(data.ground_station.ground_commands_enabled ? "YES" : "NO");
        
        uint32_t time_since_contact = millis() - data.radio_status.last_contact_time;
        Serial.print("Last Contact: ");
        Serial.print(time_since_contact / 1000);
        Serial.println(" seconds ago");
    }
    
    // Transmission statistics
    Serial.println("\n--- TRANSMISSION STATS ---");
    Serial.print("Flight Data Sent: ");
    Serial.println(data.transmission_stats.flight_data_sent);
    
    Serial.print("GPS Data Sent: ");
    Serial.println(data.transmission_stats.gps_data_sent);
    
    Serial.print("Sensor Data Sent: ");
    Serial.println(data.transmission_stats.sensor_data_sent);
    
    Serial.print("Status Data Sent: ");
    Serial.println(data.transmission_stats.status_data_sent);
    
    Serial.print("Emergency Data Sent: ");
    Serial.println(data.transmission_stats.emergency_data_sent);
    
    Serial.print("Total Data Bytes: ");
    Serial.println(data.transmission_stats.data_bytes_sent);
    
    Serial.print("Avg Transmission Rate: ");
    Serial.print(data.transmission_stats.avg_transmission_rate, 2);
    Serial.println(" pps");
    
    Serial.println("================================\n");
}

void handle_ground_command(GroundCommand command, const uint8_t* parameters) {
    Serial.print("Ground command received: ");
    Serial.print(TelemetryUtils::command_to_string(command));
    Serial.println();
    
    // Commands are automatically forwarded to the master processor
    // This callback is for logging and monitoring purposes
    
    switch (command) {
        case CMD_ARM_SYSTEM:
            Serial.println("*** GROUND COMMAND: ARM SYSTEM ***");
            break;
            
        case CMD_DISARM_SYSTEM:
            Serial.println("*** GROUND COMMAND: DISARM SYSTEM ***");
            break;
            
        case CMD_DEPLOY_DROGUE:
            Serial.println("*** GROUND COMMAND: DEPLOY DROGUE ***");
            break;
            
        case CMD_DEPLOY_MAIN:
            Serial.println("*** GROUND COMMAND: DEPLOY MAIN ***");
            break;
            
        case CMD_ABORT_FLIGHT:
            Serial.println("*** GROUND COMMAND: ABORT FLIGHT ***");
            break;
            
        case CMD_EMERGENCY_STOP:
            Serial.println("*** GROUND COMMAND: EMERGENCY STOP ***");
            emergency_shutdown();
            break;
            
        default:
            Serial.print("*** GROUND COMMAND: UNKNOWN (");
            Serial.print((int)command);
            Serial.println(") ***");
            break;
    }
}

void handle_connection_status(bool connected) {
    if (connected) {
        Serial.println("*** CONNECTED TO GROUND STATION ***");
        radio_connected = true;
    } else {
        Serial.println("*** DISCONNECTED FROM GROUND STATION ***");
        radio_connected = false;
    }
}

void handle_data_received(const TelemetryPacket& packet) {
    // This callback is called when data is received from ground station
    // Can be used for packet monitoring or logging
    
    Serial.print("Packet received: Type=");
    Serial.print(packet.header.packet_type, HEX);
    Serial.print(", Length=");
    Serial.print(packet.header.payload_length);
    Serial.print(", RSSI=");
    Serial.print((int8_t)packet.rssi);
    Serial.println(" dBm");
}

void handle_telemetry_error(const char* error_msg) {
    Serial.print("TELEMETRY ERROR: ");
    Serial.println(error_msg);
    
    // Could trigger emergency mode or other safety measures
    if (strstr(error_msg, "radio") != nullptr || 
        strstr(error_msg, "critical") != nullptr) {
        // Don't immediately emergency shutdown for radio errors
        Serial.println("Radio error detected - monitoring...");
    }
}

void emergency_shutdown() {
    Serial.println("*** EMERGENCY SHUTDOWN INITIATED ***");
    
    emergency_mode = true;
    
    // Activate emergency mode
    if (telemetry_system) {
        telemetry_system->activate_emergency_mode();
        telemetry_system->send_emergency_beacon();
        telemetry_system->set_status_led(255, 0, 0); // Red LED
    }
    
    Serial.println("Emergency shutdown complete. Telemetry system in emergency mode.");
}