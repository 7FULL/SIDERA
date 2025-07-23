#ifndef BNO055_IMU_H
#define BNO055_IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "spi_protocol.h"  // For imu_data_t structure

// BNO055 Intelligent 9-axis Absolute Orientation Sensor
// Combines accelerometer, gyroscope, and magnetometer with sensor fusion

// I2C Addresses
#define BNO055_I2C_ADDR_PRIMARY   0x28  // Primary address (COM3 = 0)
#define BNO055_I2C_ADDR_SECONDARY 0x29  // Secondary address (COM3 = 1)

// Page ID register
#define BNO055_PAGE_ID_ADDR       0x07

// Page 0 registers
#define BNO055_CHIP_ID_ADDR       0x00
#define BNO055_ACCEL_REV_ID_ADDR  0x01
#define BNO055_MAG_REV_ID_ADDR    0x02
#define BNO055_GYRO_REV_ID_ADDR   0x03
#define BNO055_SW_REV_ID_LSB_ADDR 0x04
#define BNO055_SW_REV_ID_MSB_ADDR 0x05
#define BNO055_BL_REV_ID_ADDR     0x06

// Accelerometer data registers
#define BNO055_ACCEL_DATA_X_LSB_ADDR 0x08
#define BNO055_ACCEL_DATA_X_MSB_ADDR 0x09
#define BNO055_ACCEL_DATA_Y_LSB_ADDR 0x0A
#define BNO055_ACCEL_DATA_Y_MSB_ADDR 0x0B
#define BNO055_ACCEL_DATA_Z_LSB_ADDR 0x0C
#define BNO055_ACCEL_DATA_Z_MSB_ADDR 0x0D

// Magnetometer data registers
#define BNO055_MAG_DATA_X_LSB_ADDR 0x0E
#define BNO055_MAG_DATA_X_MSB_ADDR 0x0F
#define BNO055_MAG_DATA_Y_LSB_ADDR 0x10
#define BNO055_MAG_DATA_Y_MSB_ADDR 0x11
#define BNO055_MAG_DATA_Z_LSB_ADDR 0x12
#define BNO055_MAG_DATA_Z_MSB_ADDR 0x13

// Gyroscope data registers
#define BNO055_GYRO_DATA_X_LSB_ADDR 0x14
#define BNO055_GYRO_DATA_X_MSB_ADDR 0x15
#define BNO055_GYRO_DATA_Y_LSB_ADDR 0x16
#define BNO055_GYRO_DATA_Y_MSB_ADDR 0x17
#define BNO055_GYRO_DATA_Z_LSB_ADDR 0x18
#define BNO055_GYRO_DATA_Z_MSB_ADDR 0x19

// Euler angle data registers
#define BNO055_EULER_H_LSB_ADDR   0x1A
#define BNO055_EULER_H_MSB_ADDR   0x1B
#define BNO055_EULER_R_LSB_ADDR   0x1C
#define BNO055_EULER_R_MSB_ADDR   0x1D
#define BNO055_EULER_P_LSB_ADDR   0x1E
#define BNO055_EULER_P_MSB_ADDR   0x1F

// Quaternion data registers
#define BNO055_QUATERNION_DATA_W_LSB_ADDR 0x20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR 0x21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR 0x22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR 0x23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR 0x24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR 0x25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR 0x26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR 0x27

// Linear acceleration data registers
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR 0x28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR 0x29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR 0x2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR 0x2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR 0x2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR 0x2D

// Gravity data registers
#define BNO055_GRAVITY_DATA_X_LSB_ADDR 0x2E
#define BNO055_GRAVITY_DATA_X_MSB_ADDR 0x2F
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR 0x30
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR 0x31
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR 0x32
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR 0x33

// Temperature register
#define BNO055_TEMP_ADDR          0x34

// Calibration status register
#define BNO055_CALIB_STAT_ADDR    0x35

// Self test result register
#define BNO055_SELFTEST_RESULT_ADDR 0x36

// Interrupt status register
#define BNO055_INTR_STAT_ADDR     0x37

// System status registers
#define BNO055_SYS_CLK_STAT_ADDR  0x38
#define BNO055_SYS_STAT_ADDR      0x39
#define BNO055_SYS_ERR_ADDR       0x3A

// Unit selection register
#define BNO055_UNIT_SEL_ADDR      0x3B

// Mode registers
#define BNO055_OPR_MODE_ADDR      0x3D
#define BNO055_PWR_MODE_ADDR      0x3E

// System trigger register
#define BNO055_SYS_TRIGGER_ADDR   0x3F

// Temperature source register
#define BNO055_TEMP_SOURCE_ADDR   0x40

// Axis remap registers
#define BNO055_AXIS_MAP_CONFIG_ADDR 0x41
#define BNO055_AXIS_MAP_SIGN_ADDR   0x42

// Chip ID value
#define BNO055_ID                 0xA0

// Operating modes
enum BNO055_OperatingMode {
    // Non-fusion modes
    BNO055_OPERATION_MODE_CONFIG        = 0x00,
    BNO055_OPERATION_MODE_ACCONLY       = 0x01,
    BNO055_OPERATION_MODE_MAGONLY       = 0x02,
    BNO055_OPERATION_MODE_GYRONLY       = 0x03,
    BNO055_OPERATION_MODE_ACCMAG        = 0x04,
    BNO055_OPERATION_MODE_ACCGYRO       = 0x05,
    BNO055_OPERATION_MODE_MAGGYRO       = 0x06,
    BNO055_OPERATION_MODE_AMG           = 0x07, // All sensors, no fusion
    
    // Fusion modes
    BNO055_OPERATION_MODE_IMUPLUS       = 0x08, // IMU mode (no magnetometer)
    BNO055_OPERATION_MODE_COMPASS       = 0x09, // Compass mode
    BNO055_OPERATION_MODE_M4G           = 0x0A, // M4G mode
    BNO055_OPERATION_MODE_NDOF_FMC_OFF  = 0x0B, // NDOF mode with fast magnetometer calibration off
    BNO055_OPERATION_MODE_NDOF          = 0x0C  // NDOF mode (9DOF sensor fusion)
};

// Power modes
enum BNO055_PowerMode {
    BNO055_POWER_MODE_NORMAL   = 0x00,
    BNO055_POWER_MODE_LOWPOWER = 0x01,
    BNO055_POWER_MODE_SUSPEND  = 0x02
};

// Unit selections
#define BNO055_UNIT_SEL_ACC_UNIT      0x01  // 0 = m/s², 1 = mg
#define BNO055_UNIT_SEL_GYR_UNIT      0x02  // 0 = °/s, 1 = rad/s
#define BNO055_UNIT_SEL_EUL_UNIT      0x04  // 0 = degrees, 1 = radians
#define BNO055_UNIT_SEL_TEMP_UNIT     0x10  // 0 = Celsius, 1 = Fahrenheit
#define BNO055_UNIT_SEL_ORI_ANDROID   0x80  // 0 = Windows, 1 = Android

// Calibration status bits
#define BNO055_CALIB_STAT_MAG_MASK    0x03
#define BNO055_CALIB_STAT_ACC_MASK    0x0C
#define BNO055_CALIB_STAT_GYR_MASK    0x30
#define BNO055_CALIB_STAT_SYS_MASK    0xC0

// System status values
#define BNO055_SYS_STAT_IDLE                    0x00
#define BNO055_SYS_STAT_SYSTEM_ERROR            0x01
#define BNO055_SYS_STAT_INIT_PERIPHERALS        0x02
#define BNO055_SYS_STAT_SYSTEM_INIT             0x03
#define BNO055_SYS_STAT_EXECUTING_SELFTEST     0x04
#define BNO055_SYS_STAT_SENSOR_FUSION_RUNNING  0x05
#define BNO055_SYS_STAT_SYSTEM_RUNNING_NO_FUSION 0x06

// Self test results
#define BNO055_SELFTEST_ACCEL   0x01
#define BNO055_SELFTEST_MAG     0x02
#define BNO055_SELFTEST_GYRO    0x04
#define BNO055_SELFTEST_MCU     0x08

// Error codes
#define BNO055_SUCCESS          0
#define BNO055_ERROR_INIT      -1
#define BNO055_ERROR_I2C       -2
#define BNO055_ERROR_ID        -3
#define BNO055_ERROR_MODE      -4
#define BNO055_ERROR_CALIB     -5
#define BNO055_ERROR_SELFTEST  -6

// Conversion factors
#define BNO055_ACCEL_LSB_PER_MS2    100.0f   // 1 m/s² = 100 LSB
#define BNO055_GYRO_LSB_PER_DPS     16.0f    // 1 °/s = 16 LSB
#define BNO055_GYRO_LSB_PER_RPS     900.0f   // 1 rad/s = 900 LSB
#define BNO055_MAG_LSB_PER_UT       16.0f    // 1 µT = 16 LSB
#define BNO055_EULER_LSB_PER_DEG    16.0f    // 1° = 16 LSB
#define BNO055_EULER_LSB_PER_RAD    900.0f   // 1 rad = 900 LSB
#define BNO055_QUAT_LSB_PER_UNIT    16384.0f // 1 unit = 16384 LSB
#define BNO055_TEMP_LSB_PER_C       1.0f     // 1°C = 1 LSB

// Data structures
struct BNO055_Vector {
    float x, y, z;
};

struct BNO055_Quaternion {
    float w, x, y, z;
};

struct BNO055_EulerAngles {
    float heading, roll, pitch;  // Degrees or radians based on unit setting
};

struct BNO055_CalibrationStatus {
    uint8_t system;      // 0-3 (3 = fully calibrated)
    uint8_t gyroscope;   // 0-3
    uint8_t accelerometer; // 0-3
    uint8_t magnetometer; // 0-3
};

struct BNO055_CalibrationData {
    uint8_t accel_offset[6];
    uint8_t mag_offset[6];
    uint8_t gyro_offset[6];
    uint8_t accel_radius[2];
    uint8_t mag_radius[2];
};

struct BNO055_SystemStatus {
    uint8_t system_status;
    uint8_t self_test_result;
    uint8_t system_error;
};

struct BNO055_SensorData {
    BNO055_Vector acceleration;      // m/s²
    BNO055_Vector gyroscope;         // rad/s
    BNO055_Vector magnetometer;      // µT
    BNO055_Vector linear_accel;      // m/s² (gravity removed)
    BNO055_Vector gravity;           // m/s²
    BNO055_EulerAngles euler;        // degrees or radians
    BNO055_Quaternion quaternion;    // normalized
    int8_t temperature;              // °C
    uint32_t timestamp;              // milliseconds
    bool valid;
};

class BNO055_IMU {
private:
    uint8_t i2c_address;
    uint8_t reset_pin;
    BNO055_OperatingMode current_mode;
    bool initialized;
    bool use_external_crystal;
    
    BNO055_SensorData last_reading;
    BNO055_CalibrationStatus calib_status;
    BNO055_SystemStatus system_status;
    
    // Statistics
    uint32_t measurement_count;
    uint32_t error_count;
    
    // Calibration data
    BNO055_CalibrationData stored_calibration;
    bool has_stored_calibration;
    
public:
    BNO055_IMU(uint8_t address = BNO055_I2C_ADDR_PRIMARY, uint8_t rst_pin = 255);
    
    // Initialization and configuration
    int8_t begin(BNO055_OperatingMode mode = BNO055_OPERATION_MODE_NDOF);
    int8_t reset();
    bool is_initialized() const { return initialized; }
    void set_external_crystal(bool use_crystal);
    
    // Mode management
    int8_t set_mode(BNO055_OperatingMode mode);
    BNO055_OperatingMode get_mode();
    int8_t set_power_mode(BNO055_PowerMode power_mode);
    
    // Unit configuration
    void set_units(bool use_radians = true, bool use_android_orientation = false);
    
    // Data reading functions
    int8_t read_sensor_data(BNO055_SensorData& data);
    const BNO055_SensorData& get_last_reading() const { return last_reading; }
    
    // Individual sensor reading
    BNO055_Vector read_acceleration();
    BNO055_Vector read_gyroscope();
    BNO055_Vector read_magnetometer();
    BNO055_Vector read_linear_acceleration();
    BNO055_Vector read_gravity();
    BNO055_EulerAngles read_euler_angles();
    BNO055_Quaternion read_quaternion();
    int8_t read_temperature();
    
    // Calibration functions
    BNO055_CalibrationStatus get_calibration_status();
    bool is_fully_calibrated();
    int8_t save_calibration_data();
    int8_t load_calibration_data();
    int8_t get_calibration_data(BNO055_CalibrationData& calib_data);
    int8_t set_calibration_data(const BNO055_CalibrationData& calib_data);
    void print_calibration_status();
    
    // System status and diagnostics
    BNO055_SystemStatus get_system_status();
    int8_t perform_self_test();
    void print_system_status();
    void print_sensor_info();
    
    // Integration functions
    imu_data_t get_imu_data_for_protocol();
    void update_sensor_manager();
    
    // Utility functions
    float get_success_rate() const;
    uint32_t get_measurement_count() const { return measurement_count; }
    uint32_t get_error_count() const { return error_count; }
    void reset_statistics();
    
    // Advanced features
    int8_t set_axis_remap(uint8_t remap_config, uint8_t remap_sign);
    void enable_external_crystal();
    void disable_external_crystal();
    
    // Interrupt configuration (if needed)
    int8_t configure_interrupts(uint8_t int_mask);
    
private:
    // Low-level I2C functions
    int8_t write_register(uint8_t reg, uint8_t value);
    int8_t read_register(uint8_t reg, uint8_t& value);
    int8_t read_registers(uint8_t reg, uint8_t* buffer, uint8_t length);
    
    // Internal utility functions
    int16_t read_16bit_register(uint8_t reg_lsb);
    int8_t set_page(uint8_t page);
    int8_t wait_for_mode_change();
    bool check_chip_id();
    void increment_error_count();
    
    // Data conversion functions
    BNO055_Vector convert_vector_data(uint8_t reg_lsb, float scale_factor);
    BNO055_Quaternion convert_quaternion_data();
    BNO055_EulerAngles convert_euler_data();
    
    // Calibration helpers
    void update_calibration_status();
    bool wait_for_calibration(uint32_t timeout_ms = 30000);
};

// Global instance helper
extern BNO055_IMU* g_imu;

// Utility functions
namespace BNO055_Utils {
    // Quaternion operations
    BNO055_Quaternion quaternion_multiply(const BNO055_Quaternion& q1, const BNO055_Quaternion& q2);
    BNO055_Quaternion quaternion_conjugate(const BNO055_Quaternion& q);
    BNO055_Quaternion quaternion_normalize(const BNO055_Quaternion& q);
    BNO055_EulerAngles quaternion_to_euler(const BNO055_Quaternion& q);
    BNO055_Quaternion euler_to_quaternion(const BNO055_EulerAngles& euler);
    
    // Vector operations
    float vector_magnitude(const BNO055_Vector& v);
    BNO055_Vector vector_normalize(const BNO055_Vector& v);
    BNO055_Vector vector_cross_product(const BNO055_Vector& a, const BNO055_Vector& b);
    float vector_dot_product(const BNO055_Vector& a, const BNO055_Vector& b);
    
    // Coordinate transformations
    BNO055_Vector rotate_vector(const BNO055_Vector& v, const BNO055_Quaternion& q);
    BNO055_Vector world_to_body_frame(const BNO055_Vector& world_vec, const BNO055_Quaternion& orientation);
    BNO055_Vector body_to_world_frame(const BNO055_Vector& body_vec, const BNO055_Quaternion& orientation);
    
    // Sensor fusion utilities
    float calculate_tilt_compensated_heading(const BNO055_Vector& mag, const BNO055_Vector& accel);
    BNO055_Vector remove_gravity(const BNO055_Vector& accel, const BNO055_Vector& gravity);
    
    // Calibration utilities
    const char* calibration_status_to_string(uint8_t status);
    const char* system_status_to_string(uint8_t status);
    bool is_calibration_acceptable(const BNO055_CalibrationStatus& status);
}

#endif // BNO055_IMU_H