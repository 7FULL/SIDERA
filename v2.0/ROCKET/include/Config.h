/**
 * Rocket Control System - Global Configuration
 */

#ifndef CONFIG_H
#define CONFIG_H

//#define ENABLE_BMP388_DEBUG
//#define ENABLE_MPL3115A2_DEBUG
//#define ENABLE_ATGM336H_DEBUG
//#define ENABLE_L76KB_DEBUG
//#define ENABLE_BMIO88_DEBUG
//#define ENABLE_ADXL375_DEBUG
//#define ENABLE_DS18B20_DEBUG
//#define ENABLE_LORA_DEBUG
#define ENABLE_LOGS_DEBUG

// Version information
#define FIRMWARE_VERSION "1.0.0"
#define CONFIG_VERSION 1

// Debug settings
#define DEBUG_MODE 1
#define VERBOSE_LOGGING 1

//=== FLIGHT PARAMETERS ===//
#define LAUNCH_ACCELERATION_THRESHOLD 2.5f  // g-force needed to detect launch 1.5
#define APOGEE_DETECTION_THRESHOLD 2.0f     // m below peak to confirm apogee
#define APOGEE_DETECTION_WINDOW 5          // samples to confirm apogee
#define LANDED_ALTITUDE_THRESHOLD 10.0f     // meters from ground level
#define LANDED_STABILITY_TIME 5000          // ms of stability to confirm landing
#define BURNOUT_ACCEL_THRESHOLD 2.0f        // g threshold for burnout detection 2.0
#define POWERED_FLIGHT_TIMEOUT 3000         // ms to timeout powered flight
#define LAUNCH_HEIGHT_THRESHOLD 3.0f       // meters above ground to confirm launch
#define APOGEE_SPEED_THRESHOLD -0.3f       // m/s to confirm apogee
#define APOGEE_ACCELERATION_THRESHOLD 0.5f // g to confirm apogee

//=== SENSOR PARAMETERS ===//
// Update rates by state (ms)
#define GROUND_IDLE_SENSOR_RATE 1000        // 1Hz in idle
#define READY_SENSOR_RATE 50                // 10Hz when ready
#define FLIGHT_SENSOR_RATE 10               // 100Hz during flight
#define COAST_SENSOR_RATE 10                // 100Hz during coast
#define DESCENT_SENSOR_RATE 20              // 50Hz during descent
#define LANDED_SENSOR_RATE 500              // 2Hz after landing
#define ERROR_SENSOR_RATE 1000              // 1Hz in error state

#define GPS_INIT_TIMEOUT 300000        // 60 seconds timeout for GPS initialization
#define GPS_BACKGROUND_MODE true      // Allow GPS to work in background without blocking
#define GPS_PROGRESS_INTERVAL 10000   // Show progress every 10 seconds

//=== COMMUNICATION PARAMETERS ===//
// Telemetry rates by state (ms)
#define GROUND_IDLE_TELEMETRY_RATE 1000     // 0.5Hz in idle - reduced to save power
#define READY_TELEMETRY_RATE 300           // 1Hz when ready
#define FLIGHT_TELEMETRY_RATE 200           // 5Hz during flight
#define COAST_TELEMETRY_RATE 200            // 5Hz during coast
#define DESCENT_TELEMETRY_RATE 300          // 2Hz during descent
#define LANDED_TELEMETRY_RATE 2000          // 1Hz after landing
#define ERROR_TELEMETRY_RATE 4000           // 0.5Hz in error state

// LoRa parameters
#define LORA_FREQUENCY 868E6                // Hz (EUROPE frequency)
#define LORA_BANDWIDTH 125E3                // 125 kHz bandwidth
#define LORA_SPREADING_FACTOR 9             // SF9
#define LORA_CODING_RATE 5                  // 4/5 coding rate
#define LORA_TX_POWER 20                    // dBm (5-20)
#define LORA_PREAMBLE_LENGTH 8              // Default preamble length
#define LORA_SYNC_WORD 0x34                 // Sync word for LoRa

//=== STORAGE PARAMETERS ===//
#define LOG_FILE_PREFIX "FLIGHT_"
#define LOG_INTERVAL 100                    // ms between logs
#define FLUSH_INTERVAL 5000                 // ms between forced storage flushes
#define LOG_BUFFER_SIZE 256                 // bytes for log messages

//=== POWER MANAGEMENT ===//
#define BATTERY_ADC_REF_VOLTAGE 3.3f        // Reference voltage for ADC
#define BATTERY_DIVIDER_RATIO 2.0f          // Voltage divider ratio
#define BATTERY_FILTER_ALPHA 0.2f           // Low-pass filter coefficient
#define BATTERY_FULL_VOLTAGE 4.2f           // Full LiPo voltage
#define BATTERY_EMPTY_VOLTAGE 3.0f          // Empty LiPo voltage
#define LOW_BATTERY_THRESHOLD 3.5f          // Low battery warning threshold
#define CRITICAL_BATTERY_THRESHOLD 3.2f     // Critical battery threshold
//#define ADC_RESOLUTION 4096

//=== PARACHUTE PARAMETERS ===//
#define PARACHUTE_DEPLOYMENT_DURATION 1000  // ms to activate deployment mechanism
#define PARACHUTE_SERVO_DEPLOYED_POS 180    // Servo position for deployment
#define PARACHUTE_SERVO_STOWED_POS 0        // Servo position for stowed

//=== DIAGNOSTICS ===//
#define DIAG_MEMORY_WARNING_THRESHOLD 1024  // bytes of free memory to trigger warning
#define DIAG_CPU_WARNING_THRESHOLD 90.0f    // % CPU usage to trigger warning
#define DIAG_REPORT_INTERVAL 300000         // ms between resource reports (5 min)

//=== ERROR HANDLING ===//
#define ERROR_RETRY_COUNT 3
#define ERROR_RECOVERY_TIMEOUT 5000      // ms
#define ERROR_AUTO_RESET_ENABLED 1       // 0 for disabled

//=== PERFORMANCE MONITORING ===//
#define PERF_CPU_WARNING_THRESHOLD 90.0f // %
#define PERF_MEMORY_WARNING_THRESHOLD 1024 // bytes
#define PERF_MONITOR_UPDATE_RATE 1000    // ms

#endif // CONFIG_H