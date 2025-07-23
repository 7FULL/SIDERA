#ifndef MK20_SENSOR_INTEGRATION_H
#define MK20_SENSOR_INTEGRATION_H

#include <Arduino.h>
#include "../../../lib/drivers/kx134_accelerometer.h"
#include "../../../lib/drivers/ms5611_barometer.h"
#include "../../../lib/drivers/zoe_m8q_gps.h"
#include "../../../lib/drivers/sd_card_manager.h"
#include "../../../lib/drivers/spi_flash.h"
#include "../../../lib/drivers/pyrotechnic_control.h"
#include "../../../lib/drivers/battery_monitor.h"
#include "../../../lib/common/spi_protocol.h"

// MK20DX256VLH7 Master Controller Sensor Integration
// Manages all sensors connected to the master processor

// Pin definitions for MK20DX256VLH7
#define MK20_KX134_CS_PIN         10    // KX134 High-G Accelerometer CS
#define MK20_BAROMETER_CS_PIN     9     // MS5611 Barometer CS
#define MK20_SD_CARD_CS_PIN       4     // SD Card CS
#define MK20_SPI_FLASH_CS_PIN     5     // SPI Flash CS

// GPS UART pins (Hardware Serial1)
#define MK20_GPS_RX_PIN           0     // GPS RX (Serial1)
#define MK20_GPS_TX_PIN           1     // GPS TX (Serial1)

// Pyrotechnic control pins
#define MK20_PYRO_ARM_PIN         21    // Master arm switch
#define MK20_PYRO_VOLTAGE_PIN     A9    // System voltage monitoring
#define MK20_PYRO_CH1_PIN         6     // Drogue parachute
#define MK20_PYRO_CH2_PIN         7     // Main parachute
#define MK20_PYRO_CH3_PIN         8     // Separation charge
#define MK20_PYRO_CH4_PIN         11    // Backup/custom

// Battery monitoring pins
#define MK20_BATTERY_VOLTAGE_PIN  A8    // Battery voltage divider
#define MK20_BATTERY_CURRENT_PIN  A7    // Battery current sensor (optional)

// SPI communication with slaves
#define MK20_SPI_MOSI_PIN         11    // SPI MOSI
#define MK20_SPI_MISO_PIN         12    // SPI MISO
#define MK20_SPI_SCK_PIN          13    // SPI SCK
#define MK20_SAMD21_NAV_CS_PIN    15    // Navigation SAMD21 CS
#define MK20_SAMD21_TEL_CS_PIN    16    // Telemetry SAMD21 CS

// Status LED pins
#define MK20_LED_RED_PIN          17    // Status LED Red
#define MK20_LED_GREEN_PIN        18    // Status LED Green
#define MK20_LED_BLUE_PIN         19    // Status LED Blue

// Update intervals
#define MK20_SENSOR_UPDATE_FAST   10    // 10ms for critical sensors
#define MK20_SENSOR_UPDATE_NORMAL 50    // 50ms for normal sensors
#define MK20_SENSOR_UPDATE_SLOW   200   // 200ms for slow sensors

// Data structures for sensor data
struct MK20SensorData {
    // High-G Accelerometer data
    struct {
        float accel_x, accel_y, accel_z;    // High-G acceleration (g)
        bool data_valid;
        uint32_t timestamp;
    } high_g_accel;
    
    // Barometer data
    struct {
        float pressure;                      // Pressure (Pa)
        float altitude;                      // Calculated altitude (m)
        float temperature;                   // Temperature (°C)
        bool data_valid;
        uint32_t timestamp;
    } barometer;
    
    // GPS data
    struct {
        double latitude, longitude;          // Position (degrees)
        float altitude_gps;                  // GPS altitude (m)
        float speed;                         // Ground speed (m/s)
        float course;                        // Course over ground (degrees)
        uint8_t satellites;                  // Number of satellites
        uint8_t fix_type;                    // GPS fix type
        bool data_valid;
        uint32_t timestamp;
    } gps;
    
    // System status
    struct {
        float battery_voltage;               // Battery voltage (V)
        float battery_current;               // Battery current (A)
        uint8_t battery_percentage;          // Battery level (%)
        bool pyro_armed;                     // Pyrotechnic system armed
        uint8_t pyro_status;                 // Pyrotechnic channels status
        bool sd_card_ready;                  // SD card available
        bool flash_ready;                    // SPI flash available
        uint32_t free_memory;                // Free RAM (bytes)
        uint32_t timestamp;
    } system;
};

class MK20SensorIntegration {
private:
    // Sensor instances
    KX134Accelerometer* high_g_accel;
    MS5611Barometer* barometer;
    ZOE_M8Q_GPS* gps;
    SDCardManager* sd_card;
    SPIFlash* spi_flash;
    PyrotechnicControl* pyro_control;
    BatteryMonitor* battery_monitor;
    SPIMaster* spi_master;
    
    // Sensor data
    MK20SensorData sensor_data;
    FlightState current_flight_state;
    FlightState previous_flight_state;
    
    // Timing control
    uint32_t last_fast_update;
    uint32_t last_normal_update;
    uint32_t last_slow_update;
    uint32_t last_data_log;
    
    // Configuration
    bool sensors_initialized;
    bool logging_enabled;
    bool high_rate_logging;
    
    // Flight detection
    float launch_detect_threshold;      // Acceleration threshold for launch (g)
    float apogee_detect_threshold;      // Velocity threshold for apogee (m/s)
    float main_deploy_altitude;         // Main parachute deployment altitude (m)
    float ground_altitude;              // Ground reference altitude (m)
    
    // Data logging
    uint32_t log_sequence_number;
    char current_log_filename[32];
    
    // Callbacks
    void (*flight_state_callback)(FlightState old_state, FlightState new_state);
    void (*sensor_error_callback)(const char* sensor_name, int8_t error_code);
    void (*data_ready_callback)(const MK20SensorData& data);
    
public:
    MK20SensorIntegration();
    ~MK20SensorIntegration();
    
    // Initialization
    int8_t begin();
    int8_t initialize_sensors();
    int8_t configure_sensors();
    bool is_initialized() const { return sensors_initialized; }
    
    // Main update function - call this regularly
    void update();
    
    // Sensor data access
    const MK20SensorData& get_sensor_data() const { return sensor_data; }
    FlightState get_flight_state() const { return current_flight_state; }
    
    // Individual sensor access
    float get_altitude() const { return sensor_data.barometer.altitude; }
    float get_velocity() const; // Calculated from altitude derivative
    float get_acceleration() const; // Combined acceleration magnitude
    float get_battery_voltage() const { return sensor_data.system.battery_voltage; }
    bool is_gps_valid() const { return sensor_data.gps.data_valid; }
    
    // Flight control
    int8_t arm_flight_system();
    int8_t disarm_flight_system();
    int8_t deploy_drogue_parachute();
    int8_t deploy_main_parachute();
    int8_t abort_flight();
    
    // Configuration
    int8_t set_main_deploy_altitude(float altitude);
    int8_t set_launch_threshold(float acceleration_g);
    int8_t calibrate_ground_altitude();
    
    // Data logging
    int8_t start_logging(const char* flight_name = nullptr);
    int8_t stop_logging();
    int8_t log_sensor_data();
    bool is_logging_active() const { return logging_enabled; }
    
    // Communication with slave processors
    int8_t send_data_to_navigation(const void* data, uint16_t length);
    int8_t send_data_to_telemetry(const void* data, uint16_t length);
    int8_t request_navigation_data();
    int8_t request_telemetry_status();
    
    // Emergency functions
    int8_t emergency_shutdown();
    int8_t safe_mode();
    bool is_safe_to_arm() const;
    
    // Callbacks
    void set_flight_state_callback(void (*callback)(FlightState old_state, FlightState new_state));
    void set_sensor_error_callback(void (*callback)(const char* sensor_name, int8_t error_code));
    void set_data_ready_callback(void (*callback)(const MK20SensorData& data));
    
    // Diagnostics
    void print_sensor_status() const;
    void print_flight_data() const;
    int8_t run_sensor_test();
    void set_status_led(uint8_t red, uint8_t green, uint8_t blue);
    
    // Configuration save/load
    int8_t save_flight_config();
    int8_t load_flight_config();
    
private:
    // Update functions
    void update_fast_sensors();   // High-G accel, critical systems
    void update_normal_sensors(); // Barometer, GPS
    void update_slow_sensors();   // Battery, system status
    void update_flight_state();   // Flight state machine
    
    // Flight state detection
    void detect_launch();
    void detect_apogee();
    void detect_main_deployment();
    void detect_landing();
    
    // Data processing
    void process_sensor_data();
    void calculate_derived_values();
    void apply_sensor_fusion();
    
    // Communication helpers
    int8_t send_spi_packet(uint8_t slave_cs_pin, uint8_t packet_type, const void* data, uint16_t length);
    
    // Error handling
    void handle_sensor_error(const char* sensor_name, int8_t error_code);
    void handle_critical_error();
    
    // Utility functions
    void initialize_pins();
    void configure_interrupts();
    bool check_system_health();
    uint32_t get_free_memory();
};

// Global instance
extern MK20SensorIntegration* g_mk20_sensors;

// Utility functions
namespace MK20Utils {
    // Flight calculations
    float calculate_velocity_from_altitude(float current_alt, float previous_alt, uint32_t time_diff_ms);
    float calculate_vertical_acceleration(float current_vel, float previous_vel, uint32_t time_diff_ms);
    float calculate_air_density(float pressure, float temperature);
    float calculate_dynamic_pressure(float air_density, float velocity);
    
    // Coordinate transformations
    void body_to_earth_frame(float accel_x, float accel_y, float accel_z, 
                            float roll, float pitch, float yaw,
                            float& earth_x, float& earth_y, float& earth_z);
    
    // Data validation
    bool validate_sensor_data(const MK20SensorData& data);
    bool is_acceleration_reasonable(float accel_g);
    bool is_altitude_reasonable(float altitude);
    bool is_gps_data_reasonable(double lat, double lon, float alt);
    
    // Flight analysis
    struct FlightAnalysis {
        float max_altitude;
        float max_velocity;
        float max_acceleration;
        float flight_time;
        float apogee_time;
        float descent_rate;
        bool successful_recovery;
    };
    
    FlightAnalysis analyze_flight_data(const char* log_filename);
    
    // Configuration templates
    struct FlightConfig {
        float main_deploy_altitude;
        float launch_threshold_g;
        float apogee_threshold_ms;
        uint16_t sensor_update_rate_ms;
        bool enable_high_rate_logging;
        bool enable_backup_systems;
    };
    
    extern const FlightConfig DEFAULT_FLIGHT_CONFIG;
    extern const FlightConfig HIGH_ALTITUDE_CONFIG;
    extern const FlightConfig TEST_FLIGHT_CONFIG;
}

#endif // MK20_SENSOR_INTEGRATION_H