#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "flight_state_machine.h"
#include <Arduino.h>

// Sensor status flags
#define SENSOR_OK 0x00
#define SENSOR_ERROR 0x01
#define SENSOR_NOT_READY 0x02
#define SENSOR_TIMEOUT 0x03

// Sensor types for the rocket flight computer
enum SensorType {
    SENSOR_BAROMETER,
    SENSOR_HIGH_G_ACCEL,
    SENSOR_IMU,
    SENSOR_GPS,
    SENSOR_BATTERY,
    SENSOR_COUNT
};

// Individual sensor status
struct SensorStatus {
    bool available;
    bool initialized;
    uint8_t error_code;
    uint32_t last_update;
    uint32_t error_count;
};

// Raw sensor readings structure
struct RawSensorData {
    // Barometer (MS561101BA03-50)
    float pressure;           // hPa
    float baro_temperature;   // °C
    float baro_altitude;      // m
    
    // High-G Accelerometer (KX134-1211)
    float accel_x, accel_y, accel_z;  // g
    
    // IMU (BNO055) - from navigation processor
    float imu_accel_x, imu_accel_y, imu_accel_z;  // m/s²
    float imu_gyro_x, imu_gyro_y, imu_gyro_z;     // rad/s
    float quaternion_w, quaternion_x, quaternion_y, quaternion_z;
    uint8_t imu_calibration;
    
    // GPS (ZOE-M8Q-0)
    float gps_latitude;       // degrees
    float gps_longitude;      // degrees
    float gps_altitude;       // m
    float gps_speed;          // m/s
    uint8_t gps_satellites;
    bool gps_fix_valid;
    
    // Power monitoring
    float battery_voltage;    // V
    float current_draw;       // A
    
    // Timestamps
    uint32_t timestamp;
    uint32_t sensor_timestamps[SENSOR_COUNT];
};

// Filtered and processed sensor data
struct ProcessedSensorData {
    float altitude_filtered;     // Kalman filtered altitude
    float velocity_filtered;     // Calculated vertical velocity
    float acceleration_filtered; // Filtered vertical acceleration
    float altitude_msl;         // Mean sea level altitude
    float altitude_agl;         // Above ground level altitude
    
    // Orientation (from IMU)
    float roll, pitch, yaw;     // Euler angles in radians
    float angular_velocity[3];   // Angular velocity vector
    
    // Motion state
    float vertical_velocity;    // Vertical component of velocity
    float vertical_acceleration; // Vertical component of acceleration
    float max_acceleration;     // Maximum acceleration recorded
    float max_altitude;         // Maximum altitude recorded
    
    // Quality indicators
    float altitude_confidence;  // 0.0 to 1.0
    float velocity_confidence;  // 0.0 to 1.0
    bool motion_detected;       // True if significant motion detected
    
    uint32_t timestamp;
};

// Kalman filter state for altitude/velocity estimation
struct KalmanState {
    float x[2];      // State vector [altitude, velocity]
    float P[2][2];   // Covariance matrix
    float Q[2][2];   // Process noise covariance
    float R;         // Measurement noise variance
    bool initialized;
};

class SensorManager {
private:
    SensorStatus sensor_status[SENSOR_COUNT];
    RawSensorData raw_data;
    ProcessedSensorData processed_data;
    KalmanState kalman_filter;
    
    // Baseline values for relative measurements
    float ground_pressure;
    float ground_altitude;
    float initial_battery_voltage;
    
    // Moving average filters
    static const int FILTER_SIZE = 10;
    float altitude_buffer[FILTER_SIZE];
    float accel_buffer[FILTER_SIZE];
    int buffer_index;
    bool buffer_full;
    
    // Calibration data
    bool calibration_complete;
    uint32_t calibration_start_time;
    static const uint32_t CALIBRATION_TIME = 5000; // 5 seconds
    
public:
    SensorManager();
    
    // Initialization
    void begin();
    bool calibrate_sensors();
    bool is_calibration_complete() const { return calibration_complete; }
    
    // Sensor management
    void update_all_sensors();
    bool is_sensor_available(SensorType sensor) const;
    SensorStatus get_sensor_status(SensorType sensor) const;
    void reset_sensor_errors();
    
    // Data access
    const RawSensorData& get_raw_data() const { return raw_data; }
    const ProcessedSensorData& get_processed_data() const { return processed_data; }
    
    // Convert to flight state machine format
    SensorData get_fsm_sensor_data() const;
    
    // Individual sensor updates (called by hardware-specific code)
    void update_barometer(float pressure, float temperature);
    void update_high_g_accel(float x, float y, float z);
    void update_imu_data(const imu_data_t& imu_data);
    void update_gps_data(float lat, float lon, float alt, float speed, uint8_t sats, bool valid);
    void update_battery_voltage(float voltage, float current = 0.0f);
    
    // Processing functions
    void process_altitude_data();
    void process_acceleration_data();
    void process_orientation_data();
    void apply_kalman_filter();
    
    // Utility functions
    float calculate_altitude_from_pressure(float pressure, float sea_level_pressure = 1013.25f);
    float calculate_vertical_acceleration();
    float calculate_vertical_velocity();
    void quaternion_to_euler(float qw, float qx, float qy, float qz, float& roll, float& pitch, float& yaw);
    
    // Statistics and diagnostics
    uint32_t get_total_errors() const;
    float get_data_rate(SensorType sensor) const;
    bool perform_sensor_health_check();
    
private:
    // Internal processing functions
    void init_kalman_filter();
    void update_moving_averages();
    void detect_motion();
    void update_sensor_timestamp(SensorType sensor);
    void set_sensor_error(SensorType sensor, uint8_t error_code);
    float apply_moving_average(float* buffer, float new_value, int size);
};

// Global sensor manager instance
extern SensorManager g_sensor_manager;

#endif // SENSOR_MANAGER_H