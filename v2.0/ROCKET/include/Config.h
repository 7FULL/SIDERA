/**
 * Rocket Control System - Global Configuration
 *
 * This file contains global configuration parameters for the rocket system.
 */

#ifndef CONFIG_H
#define CONFIG_H

// Debug mode - Set to 1 to enable debug output
#define DEBUG_MODE 1

// Version information
#define FIRMWARE_VERSION "1.0.0"

// Flight parameters
#define LAUNCH_ACCELERATION_THRESHOLD 2.5f  // g-force needed to detect launch
#define APOGEE_DETECTION_WINDOW 10          // Number of samples to confirm apogee
#define LANDED_ALTITUDE_THRESHOLD 10.0f     // Meters from ground level
#define LANDED_STABILITY_TIME 5000          // Milliseconds of stability to confirm landing

// Sensor fusion parameters
#define USE_SENSOR_FUSION 1                // Enable sensor fusion algorithms
#define COMPLEMENTARY_FILTER_ALPHA 0.98f   // Complementary filter parameter

// CommunicationSystems parameters
#define LORA_FREQUENCY 915E6              // LoRa frequency in Hz
#define TELEMETRY_INTERVAL 1000           // Send telemetry every X milliseconds
#define TELEMETRY_BUFFER_SIZE 10          // Number of packets to buffer

// StorageSystems parameters
#define LOG_FILE_PREFIX "FLIGHT_"         // Prefix for log files
#define LOG_INTERVAL 100                  // Log data every X milliseconds

// Power management
#define LOW_BATTERY_THRESHOLD 3.3f        // Volts
#define CRITICAL_BATTERY_THRESHOLD 3.0f   // Volts
#endif // CONFIG_H