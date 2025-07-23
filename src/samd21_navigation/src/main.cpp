#include <Arduino.h>
#include "samd21_navigation_integration.h"

// Global instances
SAMD21NavigationIntegration* navigation_system = nullptr;

// Timing variables
uint32_t last_status_print = 0;
uint32_t last_led_update = 0;
uint32_t system_start_time = 0;

// System status
bool system_ready = false;
bool emergency_mode = false;
bool imu_calibration_complete = false;

// Function prototypes
void setup();
void loop();
void initialize_system();
void update_system();
void update_status_leds();
void print_system_status();
void handle_attitude_update(float roll, float pitch, float yaw);
void handle_servo_update(float* servo_angles);
void handle_navigation_error(const char* error_msg);
void emergency_shutdown();

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait for serial connection or timeout after 3 seconds
    }
    
    Serial.println("=== SAMD21G18A-AU Rocket Flight Computer Navigation ===");
    Serial.println("Initializing navigation system...");
    
    system_start_time = millis();
    
    // Initialize system components
    initialize_system();
    
    Serial.println("Navigation system initialization complete.");
    Serial.print("Total initialization time: ");
    Serial.print(millis() - system_start_time);
    Serial.println(" ms");
}

void loop() {
    // Main system update
    update_system();
    
    // Update status LEDs
    update_status_leds();
    
    // Print system status every 10 seconds
    if (millis() - last_status_print >= 10000) {
        print_system_status();
        last_status_print = millis();
    }
    
    // Handle serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "status") {
            print_system_status();
        } else if (command == "calibrate") {
            if (navigation_system) {
                Serial.println("Starting IMU calibration...");
                navigation_system->start_imu_calibration();
            }
        } else if (command == "arm") {
            if (navigation_system) {
                navigation_system->arm_servos();
                Serial.println("Servos armed");
            }
        } else if (command == "disarm") {
            if (navigation_system) {
                navigation_system->disarm_servos();
                Serial.println("Servos disarmed");
            }
        } else if (command == "center") {
            if (navigation_system) {
                navigation_system->center_all_servos();
                Serial.println("Servos centered");
            }
        } else if (command == "test") {
            if (navigation_system) {
                Serial.println("Running navigation test...");
                navigation_system->run_servo_test();
                navigation_system->run_imu_test();
            }
        } else if (command == "emergency") {
            emergency_shutdown();
        } else if (command.startsWith("mode ")) {
            String mode_str = command.substring(5);
            if (navigation_system) {
                if (mode_str == "stabilize") {
                    navigation_system->set_navigation_mode(NAV_MODE_STABILIZE);
                    Serial.println("Navigation mode: STABILIZE");
                } else if (mode_str == "manual") {
                    navigation_system->set_navigation_mode(NAV_MODE_MANUAL);
                    Serial.println("Navigation mode: MANUAL");
                } else if (mode_str == "auto") {
                    navigation_system->set_navigation_mode(NAV_MODE_AUTO);
                    Serial.println("Navigation mode: AUTO");
                } else if (mode_str == "failsafe") {
                    navigation_system->set_navigation_mode(NAV_MODE_FAILSAFE);
                    Serial.println("Navigation mode: FAILSAFE");
                } else {
                    Serial.println("Invalid mode. Available: stabilize, manual, auto, failsafe");
                }
            }
        } else if (command.startsWith("target ")) {
            // Parse target attitude command: "target roll pitch yaw_rate"
            int space1 = command.indexOf(' ', 7);
            int space2 = command.indexOf(' ', space1 + 1);
            
            if (space1 > 0 && space2 > 0 && navigation_system) {
                float roll = command.substring(7, space1).toFloat();
                float pitch = command.substring(space1 + 1, space2).toFloat();
                float yaw_rate = command.substring(space2 + 1).toFloat();
                
                navigation_system->set_target_attitude(roll, pitch, yaw_rate);
                Serial.print("Target attitude set: Roll=");
                Serial.print(roll);
                Serial.print(", Pitch=");
                Serial.print(pitch);
                Serial.print(", Yaw Rate=");
                Serial.println(yaw_rate);
            } else {
                Serial.println("Usage: target <roll> <pitch> <yaw_rate>");
            }
        } else {
            Serial.println("Unknown command. Available commands:");
            Serial.println("  status           - Print system status");
            Serial.println("  calibrate        - Start IMU calibration");
            Serial.println("  arm              - Arm servos");
            Serial.println("  disarm           - Disarm servos");
            Serial.println("  center           - Center all servos");
            Serial.println("  test             - Run system test");
            Serial.println("  emergency        - Emergency shutdown");
            Serial.println("  mode <mode>      - Set navigation mode");
            Serial.println("  target <r> <p> <y> - Set target attitude");
        }
    }
    
    // Check for emergency conditions
    if (!emergency_mode && navigation_system) {
        // Check for communication timeout with master
        if (navigation_system->is_communication_timeout()) {
            Serial.println("WARNING: Master communication timeout!");
        }
        
        // Check IMU health
        if (!navigation_system->is_initialized()) {
            Serial.println("ERROR: Navigation system not initialized!");
            emergency_mode = true;
        }
    }
    
    // Small delay to prevent overwhelming the system
    delay(1);
}

void initialize_system() {
    // Initialize navigation integration
    navigation_system = new SAMD21NavigationIntegration();
    g_samd21_navigation = navigation_system; // Set global pointer
    
    if (navigation_system->begin() != 0) {
        Serial.println("ERROR: Navigation system initialization failed!");
        emergency_mode = true;
        return;
    }
    
    // Set up callbacks
    navigation_system->set_attitude_callback(handle_attitude_update);
    navigation_system->set_servo_callback(handle_servo_update);
    navigation_system->set_error_callback(handle_navigation_error);
    
    // Start IMU calibration
    Serial.println("Starting IMU calibration...");
    Serial.println("Please keep the flight computer stable during calibration.");
    navigation_system->start_imu_calibration();
    
    system_ready = true;
    Serial.println("SAMD21 Navigation system ready");
}

void update_system() {
    if (!system_ready || emergency_mode) {
        return;
    }
    
    // Update navigation system (this handles IMU, servos, and control)
    if (navigation_system) {
        navigation_system->update();
        
        // Check if IMU calibration is complete
        if (!imu_calibration_complete && navigation_system->is_imu_calibrated()) {
            imu_calibration_complete = true;
            Serial.println("IMU calibration complete!");
            Serial.print("Calibration status: ");
            Serial.println(navigation_system->get_imu_calibration_status());
        }
    }
}

void update_status_leds() {
    if (millis() - last_led_update < 100) return; // Update at 10Hz
    
    if (emergency_mode) {
        // Fast red blink in emergency mode
        bool led_state = (millis() / 100) % 2;
        if (navigation_system) {
            navigation_system->set_status_led(led_state ? 255 : 0, 0, 0);
        }
    } else if (!system_ready) {
        // Orange pulse during initialization
        uint8_t brightness = (sin((millis() / 500.0f) * 2 * PI) + 1) * 127.5f;
        if (navigation_system) {
            navigation_system->set_status_led(brightness, brightness / 2, 0);
        }
    } else if (!imu_calibration_complete) {
        // Purple pulse during IMU calibration
        uint8_t brightness = (sin((millis() / 300.0f) * 2 * PI) + 1) * 127.5f;
        if (navigation_system) {
            navigation_system->set_status_led(brightness, 0, brightness);
        }
    } else if (navigation_system) {
        NavigationMode mode = navigation_system->get_navigation_mode();
        bool servos_armed = navigation_system->are_servos_armed();
        
        switch (mode) {
            case NAV_MODE_STABILIZE:
                if (servos_armed) {
                    // Steady green - stabilize mode, armed
                    navigation_system->set_status_led(0, 255, 0);
                } else {
                    // Slow green pulse - stabilize mode, disarmed
                    uint8_t brightness = (sin((millis() / 1000.0f) * 2 * PI) + 1) * 127.5f;
                    navigation_system->set_status_led(0, brightness, 0);
                }
                break;
                
            case NAV_MODE_MANUAL:
                if (servos_armed) {
                    // Steady blue - manual mode, armed
                    navigation_system->set_status_led(0, 0, 255);
                } else {
                    // Slow blue pulse - manual mode, disarmed
                    uint8_t brightness = (sin((millis() / 1000.0f) * 2 * PI) + 1) * 127.5f;
                    navigation_system->set_status_led(0, 0, brightness);
                }
                break;
                
            case NAV_MODE_AUTO:
                if (servos_armed) {
                    // Steady cyan - auto mode, armed
                    navigation_system->set_status_led(0, 255, 255);
                } else {
                    // Slow cyan pulse - auto mode, disarmed
                    uint8_t brightness = (sin((millis() / 1000.0f) * 2 * PI) + 1) * 127.5f;
                    navigation_system->set_status_led(0, brightness, brightness);
                }
                break;
                
            case NAV_MODE_FAILSAFE:
                // Fast yellow blink - failsafe mode
                {
                    bool led_state = (millis() / 200) % 2;
                    navigation_system->set_status_led(led_state ? 255 : 0, led_state ? 255 : 0, 0);
                }
                break;
                
            case NAV_MODE_TEST:
                // Rainbow cycle - test mode
                {
                    float hue = fmod(millis() / 10.0f, 360.0f);
                    // Simple HSV to RGB conversion for rainbow effect
                    uint8_t r = 0, g = 0, b = 0;
                    if (hue < 120) {
                        r = 255 - (hue * 255 / 120);
                        g = hue * 255 / 120;
                    } else if (hue < 240) {
                        g = 255 - ((hue - 120) * 255 / 120);
                        b = (hue - 120) * 255 / 120;
                    } else {
                        b = 255 - ((hue - 240) * 255 / 120);
                        r = (hue - 240) * 255 / 120;
                    }
                    navigation_system->set_status_led(r, g, b);
                }
                break;
        }
    }
    
    last_led_update = millis();
}

void print_system_status() {
    if (!navigation_system) return;
    
    Serial.println("\n=== NAVIGATION SYSTEM STATUS ===");
    Serial.print("Uptime: ");
    Serial.print((millis() - system_start_time) / 1000);
    Serial.println(" seconds");
    
    Serial.print("System Ready: ");
    Serial.println(system_ready ? "YES" : "NO");
    
    Serial.print("Emergency Mode: ");
    Serial.println(emergency_mode ? "YES" : "NO");
    
    Serial.print("IMU Calibrated: ");
    Serial.println(imu_calibration_complete ? "YES" : "NO");
    
    const NavigationData& data = navigation_system->get_navigation_data();
    
    // Navigation mode and status
    Serial.print("Navigation Mode: ");
    switch (navigation_system->get_navigation_mode()) {
        case NAV_MODE_STABILIZE: Serial.println("STABILIZE"); break;
        case NAV_MODE_MANUAL: Serial.println("MANUAL"); break;
        case NAV_MODE_AUTO: Serial.println("AUTO"); break;
        case NAV_MODE_FAILSAFE: Serial.println("FAILSAFE"); break;
        case NAV_MODE_TEST: Serial.println("TEST"); break;
    }
    
    Serial.print("Flight Phase: ");
    switch (navigation_system->get_flight_phase()) {
        case PHASE_PRE_LAUNCH: Serial.println("PRE_LAUNCH"); break;
        case PHASE_BOOST: Serial.println("BOOST"); break;
        case PHASE_COAST: Serial.println("COAST"); break;
        case PHASE_APOGEE: Serial.println("APOGEE"); break;
        case PHASE_DESCENT: Serial.println("DESCENT"); break;
        case PHASE_RECOVERY: Serial.println("RECOVERY"); break;
    }
    
    Serial.print("Servos Armed: ");
    Serial.println(navigation_system->are_servos_armed() ? "YES" : "NO");
    
    Serial.print("Flight Stable: ");
    Serial.println(navigation_system->is_flight_stable() ? "YES" : "NO");
    
    // IMU data
    Serial.println("\n--- IMU DATA ---");
    if (data.imu.data_valid) {
        Serial.print("Attitude (Roll/Pitch/Yaw): ");
        Serial.print(data.state.roll, 1);
        Serial.print(" / ");
        Serial.print(data.state.pitch, 1);
        Serial.print(" / ");
        Serial.print(data.state.yaw, 1);
        Serial.println(" degrees");
        
        Serial.print("Angular Rates: ");
        Serial.print(data.state.roll_rate, 1);
        Serial.print(" / ");
        Serial.print(data.state.pitch_rate, 1);
        Serial.print(" / ");
        Serial.print(data.state.yaw_rate, 1);
        Serial.println(" deg/s");
        
        Serial.print("Acceleration: ");
        Serial.print(data.imu.accel_x, 2);
        Serial.print(" / ");
        Serial.print(data.imu.accel_y, 2);
        Serial.print(" / ");
        Serial.print(data.imu.accel_z, 2);
        Serial.println(" m/s²");
        
        Serial.print("Calibration Status: ");
        Serial.println(data.imu.calibration_status);
    } else {
        Serial.println("IMU: INVALID DATA");
    }
    
    // Servo positions
    Serial.println("\n--- SERVO POSITIONS ---");
    Serial.print("Aileron (Servo 1): ");
    Serial.print(data.servos.servo_1_angle, 1);
    Serial.println(" degrees");
    
    Serial.print("Elevator (Servo 2): ");
    Serial.print(data.servos.servo_2_angle, 1);
    Serial.println(" degrees");
    
    Serial.print("Rudder (Servo 3): ");
    Serial.print(data.servos.servo_3_angle, 1);
    Serial.println(" degrees");
    
    Serial.print("Canard (Servo 4): ");
    Serial.print(data.servos.servo_4_angle, 1);
    Serial.println(" degrees");
    
    Serial.print("Servos Enabled: ");
    Serial.println(data.servos.servos_enabled ? "YES" : "NO");
    
    Serial.println("==============================\n");
}

void handle_attitude_update(float roll, float pitch, float yaw) {
    // This callback is called when attitude data is updated
    // Can be used for real-time attitude monitoring or logging
    
    // Example: Check for extreme attitudes
    if (abs(roll) > 60.0f || abs(pitch) > 45.0f) {
        Serial.print("EXTREME ATTITUDE: Roll=");
        Serial.print(roll, 1);
        Serial.print(", Pitch=");
        Serial.println(pitch, 1);
    }
}

void handle_servo_update(float* servo_angles) {
    // This callback is called when servo positions are updated
    // Can be used for servo position monitoring or logging
    
    // Example: Log servo positions at lower frequency
    static uint32_t last_servo_log = 0;
    if (millis() - last_servo_log > 5000) { // Every 5 seconds
        Serial.print("Servo positions: ");
        for (int i = 0; i < 4; i++) {
            Serial.print(servo_angles[i], 1);
            if (i < 3) Serial.print(", ");
        }
        Serial.println();
        last_servo_log = millis();
    }
}

void handle_navigation_error(const char* error_msg) {
    Serial.print("NAVIGATION ERROR: ");
    Serial.println(error_msg);
    
    // Could trigger emergency mode or other safety measures
    if (strstr(error_msg, "failsafe") != nullptr) {
        emergency_mode = true;
    }
}

void emergency_shutdown() {
    Serial.println("*** EMERGENCY SHUTDOWN INITIATED ***");
    
    emergency_mode = true;
    
    // Activate failsafe mode
    if (navigation_system) {
        navigation_system->activate_failsafe();
        navigation_system->center_all_servos();
        navigation_system->disarm_servos();
        navigation_system->set_status_led(255, 0, 0); // Red LED
    }
    
    Serial.println("Emergency shutdown complete. Navigation system in safe mode.");
}