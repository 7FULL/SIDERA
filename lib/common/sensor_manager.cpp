#include "sensor_manager.h"
#include <math.h>

SensorManager g_sensor_manager;

SensorManager::SensorManager() {
    // Initialize sensor status
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensor_status[i].available = false;
        sensor_status[i].initialized = false;
        sensor_status[i].error_code = SENSOR_NOT_READY;
        sensor_status[i].last_update = 0;
        sensor_status[i].error_count = 0;
    }
    
    // Initialize data structures
    memset(&raw_data, 0, sizeof(RawSensorData));
    memset(&processed_data, 0, sizeof(ProcessedSensorData));
    
    // Initialize filters and baselines
    ground_pressure = 1013.25f;
    ground_altitude = 0.0f;
    initial_battery_voltage = 12.0f;
    
    buffer_index = 0;
    buffer_full = false;
    memset(altitude_buffer, 0, sizeof(altitude_buffer));
    memset(accel_buffer, 0, sizeof(accel_buffer));
    
    calibration_complete = false;
    calibration_start_time = 0;
    
    init_kalman_filter();
}

void SensorManager::begin() {
    calibration_start_time = millis();
    calibration_complete = false;
    
    // Mark sensors as available (hardware-specific initialization should be done elsewhere)
    sensor_status[SENSOR_BAROMETER].available = true;
    sensor_status[SENSOR_HIGH_G_ACCEL].available = true;
    sensor_status[SENSOR_IMU].available = true;
    sensor_status[SENSOR_GPS].available = true;
    sensor_status[SENSOR_BATTERY].available = true;
}

bool SensorManager::calibrate_sensors() {
    if (calibration_complete) {
        return true;
    }
    
    uint32_t calibration_time = millis() - calibration_start_time;
    
    if (calibration_time < CALIBRATION_TIME) {
        // Still calibrating - collect baseline data
        if (raw_data.pressure > 0) {
            ground_pressure = raw_data.pressure;
            ground_altitude = raw_data.baro_altitude;
        }
        
        if (raw_data.battery_voltage > 0) {
            initial_battery_voltage = raw_data.battery_voltage;
        }
        
        return false;
    }
    
    // Calibration complete
    calibration_complete = true;
    
    // Initialize processed data baseline
    processed_data.altitude_agl = 0.0f;
    processed_data.max_altitude = ground_altitude;
    processed_data.max_acceleration = 0.0f;
    
    return true;
}

void SensorManager::update_all_sensors() {
    // Process all available sensor data
    process_altitude_data();
    process_acceleration_data();
    process_orientation_data();
    
    // Apply filtering
    apply_kalman_filter();
    update_moving_averages();
    
    // Detect motion and update quality indicators
    detect_motion();
    
    // Update timestamp
    processed_data.timestamp = millis();
}

bool SensorManager::is_sensor_available(SensorType sensor) const {
    if (sensor < SENSOR_COUNT) {
        return sensor_status[sensor].available && sensor_status[sensor].initialized;
    }
    return false;
}

SensorStatus SensorManager::get_sensor_status(SensorType sensor) const {
    if (sensor < SENSOR_COUNT) {
        return sensor_status[sensor];
    }
    
    SensorStatus invalid = {false, false, SENSOR_ERROR, 0, 0};
    return invalid;
}

void SensorManager::reset_sensor_errors() {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensor_status[i].error_count = 0;
        if (sensor_status[i].error_code != SENSOR_NOT_READY) {
            sensor_status[i].error_code = SENSOR_OK;
        }
    }
}

SensorData SensorManager::get_fsm_sensor_data() const {
    SensorData fsm_data;
    
    fsm_data.altitude = processed_data.altitude_filtered;
    fsm_data.velocity = processed_data.vertical_velocity;
    fsm_data.acceleration = processed_data.vertical_acceleration;
    fsm_data.high_g_accel = sqrt(raw_data.accel_x * raw_data.accel_x + 
                                raw_data.accel_y * raw_data.accel_y + 
                                raw_data.accel_z * raw_data.accel_z);
    fsm_data.pressure = raw_data.pressure;
    fsm_data.battery_voltage = raw_data.battery_voltage;
    fsm_data.gps_valid = raw_data.gps_fix_valid;
    fsm_data.timestamp = processed_data.timestamp;
    
    return fsm_data;
}

// Sensor update functions
void SensorManager::update_barometer(float pressure, float temperature) {
    raw_data.pressure = pressure;
    raw_data.baro_temperature = temperature;
    raw_data.baro_altitude = calculate_altitude_from_pressure(pressure);
    
    update_sensor_timestamp(SENSOR_BAROMETER);
    sensor_status[SENSOR_BAROMETER].initialized = true;
    sensor_status[SENSOR_BAROMETER].error_code = SENSOR_OK;
}

void SensorManager::update_high_g_accel(float x, float y, float z) {
    raw_data.accel_x = x;
    raw_data.accel_y = y;
    raw_data.accel_z = z;
    
    update_sensor_timestamp(SENSOR_HIGH_G_ACCEL);
    sensor_status[SENSOR_HIGH_G_ACCEL].initialized = true;
    sensor_status[SENSOR_HIGH_G_ACCEL].error_code = SENSOR_OK;
}

void SensorManager::update_imu_data(const imu_data_t& imu_data) {
    raw_data.imu_accel_x = imu_data.acceleration[0];
    raw_data.imu_accel_y = imu_data.acceleration[1];
    raw_data.imu_accel_z = imu_data.acceleration[2];
    
    raw_data.imu_gyro_x = imu_data.gyroscope[0];
    raw_data.imu_gyro_y = imu_data.gyroscope[1];
    raw_data.imu_gyro_z = imu_data.gyroscope[2];
    
    raw_data.quaternion_w = imu_data.quaternion[0];
    raw_data.quaternion_x = imu_data.quaternion[1];
    raw_data.quaternion_y = imu_data.quaternion[2];
    raw_data.quaternion_z = imu_data.quaternion[3];
    
    raw_data.imu_calibration = imu_data.calibration_status;
    
    update_sensor_timestamp(SENSOR_IMU);
    sensor_status[SENSOR_IMU].initialized = true;
    sensor_status[SENSOR_IMU].error_code = SENSOR_OK;
}

void SensorManager::update_gps_data(float lat, float lon, float alt, float speed, uint8_t sats, bool valid) {
    raw_data.gps_latitude = lat;
    raw_data.gps_longitude = lon;
    raw_data.gps_altitude = alt;
    raw_data.gps_speed = speed;
    raw_data.gps_satellites = sats;
    raw_data.gps_fix_valid = valid;
    
    update_sensor_timestamp(SENSOR_GPS);
    sensor_status[SENSOR_GPS].initialized = true;
    sensor_status[SENSOR_GPS].error_code = valid ? SENSOR_OK : SENSOR_ERROR;
}

void SensorManager::update_battery_voltage(float voltage, float current) {
    raw_data.battery_voltage = voltage;
    raw_data.current_draw = current;
    
    update_sensor_timestamp(SENSOR_BATTERY);
    sensor_status[SENSOR_BATTERY].initialized = true;
    sensor_status[SENSOR_BATTERY].error_code = SENSOR_OK;
}

// Processing functions
void SensorManager::process_altitude_data() {
    if (calibration_complete && raw_data.baro_altitude > 0) {
        processed_data.altitude_msl = raw_data.baro_altitude;
        processed_data.altitude_agl = raw_data.baro_altitude - ground_altitude;
        
        // Update maximum altitude
        if (processed_data.altitude_msl > processed_data.max_altitude) {
            processed_data.max_altitude = processed_data.altitude_msl;
        }
        
        // Calculate confidence based on sensor health
        processed_data.altitude_confidence = is_sensor_available(SENSOR_BAROMETER) ? 1.0f : 0.0f;
    }
}

void SensorManager::process_acceleration_data() {
    processed_data.vertical_acceleration = calculate_vertical_acceleration();
    
    // Update maximum acceleration
    float total_accel = sqrt(raw_data.accel_x * raw_data.accel_x + 
                            raw_data.accel_y * raw_data.accel_y + 
                            raw_data.accel_z * raw_data.accel_z);
    
    if (total_accel > processed_data.max_acceleration) {
        processed_data.max_acceleration = total_accel;
    }
}

void SensorManager::process_orientation_data() {
    if (is_sensor_available(SENSOR_IMU)) {
        quaternion_to_euler(raw_data.quaternion_w, raw_data.quaternion_x, 
                           raw_data.quaternion_y, raw_data.quaternion_z,
                           processed_data.roll, processed_data.pitch, processed_data.yaw);
        
        processed_data.angular_velocity[0] = raw_data.imu_gyro_x;
        processed_data.angular_velocity[1] = raw_data.imu_gyro_y;
        processed_data.angular_velocity[2] = raw_data.imu_gyro_z;
    }
}

void SensorManager::apply_kalman_filter() {
    if (!kalman_filter.initialized) {
        init_kalman_filter();
        return;
    }
    
    // Prediction step
    float dt = 0.01f; // Assume 100Hz update rate
    
    // State transition matrix
    float F[2][2] = {{1, dt}, {0, 1}};
    
    // Predict state
    float x_pred[2];
    x_pred[0] = F[0][0] * kalman_filter.x[0] + F[0][1] * kalman_filter.x[1];
    x_pred[1] = F[1][0] * kalman_filter.x[0] + F[1][1] * kalman_filter.x[1];
    
    // Predict covariance
    float P_pred[2][2];
    P_pred[0][0] = kalman_filter.P[0][0] + dt * kalman_filter.P[0][1] + dt * kalman_filter.P[1][0] + dt * dt * kalman_filter.P[1][1] + kalman_filter.Q[0][0];
    P_pred[0][1] = kalman_filter.P[0][1] + dt * kalman_filter.P[1][1] + kalman_filter.Q[0][1];
    P_pred[1][0] = kalman_filter.P[1][0] + dt * kalman_filter.P[1][1] + kalman_filter.Q[1][0];
    P_pred[1][1] = kalman_filter.P[1][1] + kalman_filter.Q[1][1];
    
    // Update step with altitude measurement
    if (raw_data.baro_altitude > 0) {
        float y = raw_data.baro_altitude - x_pred[0]; // Innovation
        float S = P_pred[0][0] + kalman_filter.R; // Innovation covariance
        
        // Kalman gain
        float K[2];
        K[0] = P_pred[0][0] / S;
        K[1] = P_pred[1][0] / S;
        
        // Update state
        kalman_filter.x[0] = x_pred[0] + K[0] * y;
        kalman_filter.x[1] = x_pred[1] + K[1] * y;
        
        // Update covariance
        float I_KH[2][2] = {{1 - K[0], 0}, {-K[1], 1}};
        kalman_filter.P[0][0] = I_KH[0][0] * P_pred[0][0] + I_KH[0][1] * P_pred[1][0];
        kalman_filter.P[0][1] = I_KH[0][0] * P_pred[0][1] + I_KH[0][1] * P_pred[1][1];
        kalman_filter.P[1][0] = I_KH[1][0] * P_pred[0][0] + I_KH[1][1] * P_pred[1][0];
        kalman_filter.P[1][1] = I_KH[1][0] * P_pred[0][1] + I_KH[1][1] * P_pred[1][1];
    } else {
        // No measurement update
        kalman_filter.x[0] = x_pred[0];
        kalman_filter.x[1] = x_pred[1];
        memcpy(kalman_filter.P, P_pred, sizeof(P_pred));
    }
    
    // Update processed data
    processed_data.altitude_filtered = kalman_filter.x[0];
    processed_data.velocity_filtered = kalman_filter.x[1];
    processed_data.vertical_velocity = kalman_filter.x[1];
}

// Utility functions
float SensorManager::calculate_altitude_from_pressure(float pressure, float sea_level_pressure) {
    return 44330.0f * (1.0f - pow(pressure / sea_level_pressure, 0.1903f));
}

float SensorManager::calculate_vertical_acceleration() {
    // Use IMU data if available, otherwise use high-G accelerometer Z-axis
    if (is_sensor_available(SENSOR_IMU)) {
        return raw_data.imu_accel_z - 9.81f; // Remove gravity
    } else if (is_sensor_available(SENSOR_HIGH_G_ACCEL)) {
        return (raw_data.accel_z - 1.0f) * 9.81f; // Convert from g to m/s², remove 1g
    }
    return 0.0f;
}

float SensorManager::calculate_vertical_velocity() {
    return processed_data.velocity_filtered;
}

void SensorManager::quaternion_to_euler(float qw, float qx, float qy, float qz, float& roll, float& pitch, float& yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (qw * qx + qy * qz);
    float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2 * (qw * qy - qz * qx);
    if (abs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);
    else
        pitch = asin(sinp);
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (qw * qz + qx * qy);
    float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    yaw = atan2(siny_cosp, cosy_cosp);
}

// Private functions
void SensorManager::init_kalman_filter() {
    // Initialize state [altitude, velocity]
    kalman_filter.x[0] = ground_altitude;
    kalman_filter.x[1] = 0.0f;
    
    // Initialize covariance matrix
    kalman_filter.P[0][0] = 10.0f;  // Altitude variance
    kalman_filter.P[0][1] = 0.0f;
    kalman_filter.P[1][0] = 0.0f;
    kalman_filter.P[1][1] = 10.0f;  // Velocity variance
    
    // Process noise covariance
    kalman_filter.Q[0][0] = 0.1f;   // Altitude process noise
    kalman_filter.Q[0][1] = 0.0f;
    kalman_filter.Q[1][0] = 0.0f;
    kalman_filter.Q[1][1] = 1.0f;   // Velocity process noise
    
    // Measurement noise variance
    kalman_filter.R = 2.0f;         // Altitude measurement noise
    
    kalman_filter.initialized = true;
}

void SensorManager::update_moving_averages() {
    // Update altitude moving average
    processed_data.altitude_filtered = apply_moving_average(altitude_buffer, 
                                                           raw_data.baro_altitude, 
                                                           FILTER_SIZE);
    
    // Update acceleration moving average (for noise reduction)
    float total_accel = sqrt(raw_data.accel_x * raw_data.accel_x + 
                            raw_data.accel_y * raw_data.accel_y + 
                            raw_data.accel_z * raw_data.accel_z);
    apply_moving_average(accel_buffer, total_accel, FILTER_SIZE);
    
    buffer_index = (buffer_index + 1) % FILTER_SIZE;
    if (buffer_index == 0) {
        buffer_full = true;
    }
}

void SensorManager::detect_motion() {
    // Simple motion detection based on acceleration magnitude
    float accel_magnitude = sqrt(raw_data.accel_x * raw_data.accel_x + 
                                raw_data.accel_y * raw_data.accel_y + 
                                raw_data.accel_z * raw_data.accel_z);
    
    processed_data.motion_detected = (abs(accel_magnitude - 1.0f) > 0.2f); // 0.2g threshold
}

void SensorManager::update_sensor_timestamp(SensorType sensor) {
    if (sensor < SENSOR_COUNT) {
        sensor_status[sensor].last_update = millis();
        raw_data.sensor_timestamps[sensor] = millis();
    }
}

void SensorManager::set_sensor_error(SensorType sensor, uint8_t error_code) {
    if (sensor < SENSOR_COUNT) {
        sensor_status[sensor].error_code = error_code;
        sensor_status[sensor].error_count++;
    }
}

float SensorManager::apply_moving_average(float* buffer, float new_value, int size) {
    buffer[buffer_index] = new_value;
    
    float sum = 0.0f;
    int count = buffer_full ? size : (buffer_index + 1);
    
    for (int i = 0; i < count; i++) {
        sum += buffer[i];
    }
    
    return sum / count;
}

uint32_t SensorManager::get_total_errors() const {
    uint32_t total = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        total += sensor_status[i].error_count;
    }
    return total;
}

float SensorManager::get_data_rate(SensorType sensor) const {
    if (sensor < SENSOR_COUNT && sensor_status[sensor].last_update > 0) {
        uint32_t time_diff = millis() - sensor_status[sensor].last_update;
        return time_diff > 0 ? 1000.0f / time_diff : 0.0f;
    }
    return 0.0f;
}

bool SensorManager::perform_sensor_health_check() {
    bool all_healthy = true;
    uint32_t current_time = millis();
    
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (sensor_status[i].available) {
            // Check for timeout
            if (current_time - sensor_status[i].last_update > 1000) { // 1 second timeout
                set_sensor_error((SensorType)i, SENSOR_TIMEOUT);
                all_healthy = false;
            }
        }
    }
    
    return all_healthy;
}