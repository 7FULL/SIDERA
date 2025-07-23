#include "bno055_imu.h"
#include <math.h>

BNO055_IMU* g_imu = nullptr;

BNO055_IMU::BNO055_IMU(uint8_t address, uint8_t rst_pin) {
    i2c_address = address;
    reset_pin = rst_pin;
    current_mode = BNO055_OPERATION_MODE_CONFIG;
    initialized = false;
    use_external_crystal = false;
    
    measurement_count = 0;
    error_count = 0;
    has_stored_calibration = false;
    
    // Initialize structures
    memset(&last_reading, 0, sizeof(BNO055_SensorData));
    memset(&calib_status, 0, sizeof(BNO055_CalibrationStatus));
    memset(&system_status, 0, sizeof(BNO055_SystemStatus));
    memset(&stored_calibration, 0, sizeof(BNO055_CalibrationData));
}

int8_t BNO055_IMU::begin(BNO055_OperatingMode mode) {
    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C speed
    
    // Initialize reset pin if provided
    if (reset_pin != 255) {
        pinMode(reset_pin, OUTPUT);
        digitalWrite(reset_pin, HIGH);
    }
    
    // Reset the sensor
    if (reset() != BNO055_SUCCESS) {
        return BNO055_ERROR_INIT;
    }
    
    // Check chip ID
    if (!check_chip_id()) {
        return BNO055_ERROR_ID;
    }
    
    // Set to config mode
    if (set_mode(BNO055_OPERATION_MODE_CONFIG) != BNO055_SUCCESS) {
        return BNO055_ERROR_MODE;
    }
    
    // Set units (radians, m/s², Android orientation)
    set_units(true, false);
    
    // Use external crystal if specified
    if (use_external_crystal) {
        enable_external_crystal();
    }
    
    // Set operating mode
    if (set_mode(mode) != BNO055_SUCCESS) {
        return BNO055_ERROR_MODE;
    }
    
    // Wait for system to be ready
    delay(100);
    
    initialized = true;
    return BNO055_SUCCESS;
}

int8_t BNO055_IMU::reset() {
    // Hardware reset if pin is available
    if (reset_pin != 255) {
        digitalWrite(reset_pin, LOW);
        delay(10);
        digitalWrite(reset_pin, HIGH);
        delay(650); // Boot time
    } else {
        // Software reset
        if (write_register(BNO055_SYS_TRIGGER_ADDR, 0x20) != BNO055_SUCCESS) {
            return BNO055_ERROR_I2C;
        }
        delay(650); // Boot time
    }
    
    return BNO055_SUCCESS;
}

void BNO055_IMU::set_external_crystal(bool use_crystal) {
    use_external_crystal = use_crystal;
}

int8_t BNO055_IMU::set_mode(BNO055_OperatingMode mode) {
    if (write_register(BNO055_OPR_MODE_ADDR, (uint8_t)mode) != BNO055_SUCCESS) {
        increment_error_count();
        return BNO055_ERROR_MODE;
    }
    
    current_mode = mode;
    
    // Wait for mode change
    if (wait_for_mode_change() != BNO055_SUCCESS) {
        return BNO055_ERROR_MODE;
    }
    
    return BNO055_SUCCESS;
}

BNO055_OperatingMode BNO055_IMU::get_mode() {
    uint8_t mode;
    if (read_register(BNO055_OPR_MODE_ADDR, mode) == BNO055_SUCCESS) {
        return (BNO055_OperatingMode)mode;
    }
    return BNO055_OPERATION_MODE_CONFIG;
}

int8_t BNO055_IMU::set_power_mode(BNO055_PowerMode power_mode) {
    return write_register(BNO055_PWR_MODE_ADDR, (uint8_t)power_mode);
}

void BNO055_IMU::set_units(bool use_radians, bool use_android_orientation) {
    uint8_t unit_sel = 0;
    
    // Set acceleration units to m/s²
    unit_sel &= ~BNO055_UNIT_SEL_ACC_UNIT;
    
    // Set gyroscope units to rad/s or °/s
    if (use_radians) {
        unit_sel |= BNO055_UNIT_SEL_GYR_UNIT;
        unit_sel |= BNO055_UNIT_SEL_EUL_UNIT;
    } else {
        unit_sel &= ~BNO055_UNIT_SEL_GYR_UNIT;
        unit_sel &= ~BNO055_UNIT_SEL_EUL_UNIT;
    }
    
    // Set temperature units to Celsius
    unit_sel &= ~BNO055_UNIT_SEL_TEMP_UNIT;
    
    // Set orientation format
    if (use_android_orientation) {
        unit_sel |= BNO055_UNIT_SEL_ORI_ANDROID;
    } else {
        unit_sel &= ~BNO055_UNIT_SEL_ORI_ANDROID;
    }
    
    write_register(BNO055_UNIT_SEL_ADDR, unit_sel);
}

int8_t BNO055_IMU::read_sensor_data(BNO055_SensorData& data) {
    // Read all sensor data in burst mode for better performance
    uint8_t buffer[45]; // All data registers from 0x08 to 0x34
    
    if (read_registers(BNO055_ACCEL_DATA_X_LSB_ADDR, buffer, 45) != BNO055_SUCCESS) {
        increment_error_count();
        return BNO055_ERROR_I2C;
    }
    
    // Parse accelerometer data (registers 0x08-0x0D)
    data.acceleration.x = (int16_t)((buffer[1] << 8) | buffer[0]) / BNO055_ACCEL_LSB_PER_MS2;
    data.acceleration.y = (int16_t)((buffer[3] << 8) | buffer[2]) / BNO055_ACCEL_LSB_PER_MS2;
    data.acceleration.z = (int16_t)((buffer[5] << 8) | buffer[4]) / BNO055_ACCEL_LSB_PER_MS2;
    
    // Parse magnetometer data (registers 0x0E-0x13)
    data.magnetometer.x = (int16_t)((buffer[7] << 8) | buffer[6]) / BNO055_MAG_LSB_PER_UT;
    data.magnetometer.y = (int16_t)((buffer[9] << 8) | buffer[8]) / BNO055_MAG_LSB_PER_UT;
    data.magnetometer.z = (int16_t)((buffer[11] << 8) | buffer[10]) / BNO055_MAG_LSB_PER_UT;
    
    // Parse gyroscope data (registers 0x14-0x19)
    data.gyroscope.x = (int16_t)((buffer[13] << 8) | buffer[12]) / BNO055_GYRO_LSB_PER_RPS;
    data.gyroscope.y = (int16_t)((buffer[15] << 8) | buffer[14]) / BNO055_GYRO_LSB_PER_RPS;
    data.gyroscope.z = (int16_t)((buffer[17] << 8) | buffer[16]) / BNO055_GYRO_LSB_PER_RPS;
    
    // Parse Euler angles (registers 0x1A-0x1F)
    data.euler.heading = (int16_t)((buffer[19] << 8) | buffer[18]) / BNO055_EULER_LSB_PER_RAD;
    data.euler.roll = (int16_t)((buffer[21] << 8) | buffer[20]) / BNO055_EULER_LSB_PER_RAD;
    data.euler.pitch = (int16_t)((buffer[23] << 8) | buffer[22]) / BNO055_EULER_LSB_PER_RAD;
    
    // Parse quaternion data (registers 0x20-0x27)
    data.quaternion.w = (int16_t)((buffer[25] << 8) | buffer[24]) / BNO055_QUAT_LSB_PER_UNIT;
    data.quaternion.x = (int16_t)((buffer[27] << 8) | buffer[26]) / BNO055_QUAT_LSB_PER_UNIT;
    data.quaternion.y = (int16_t)((buffer[29] << 8) | buffer[28]) / BNO055_QUAT_LSB_PER_UNIT;
    data.quaternion.z = (int16_t)((buffer[31] << 8) | buffer[30]) / BNO055_QUAT_LSB_PER_UNIT;
    
    // Parse linear acceleration data (registers 0x28-0x2D)
    data.linear_accel.x = (int16_t)((buffer[33] << 8) | buffer[32]) / BNO055_ACCEL_LSB_PER_MS2;
    data.linear_accel.y = (int16_t)((buffer[35] << 8) | buffer[34]) / BNO055_ACCEL_LSB_PER_MS2;
    data.linear_accel.z = (int16_t)((buffer[37] << 8) | buffer[36]) / BNO055_ACCEL_LSB_PER_MS2;
    
    // Parse gravity vector data (registers 0x2E-0x33)
    data.gravity.x = (int16_t)((buffer[39] << 8) | buffer[38]) / BNO055_ACCEL_LSB_PER_MS2;
    data.gravity.y = (int16_t)((buffer[41] << 8) | buffer[40]) / BNO055_ACCEL_LSB_PER_MS2;
    data.gravity.z = (int16_t)((buffer[43] << 8) | buffer[42]) / BNO055_ACCEL_LSB_PER_MS2;
    
    // Parse temperature (register 0x34)
    data.temperature = (int8_t)buffer[44];
    
    // Set timestamp and validity
    data.timestamp = millis();
    data.valid = true;
    
    // Update last reading
    last_reading = data;
    measurement_count++;
    
    return BNO055_SUCCESS;
}

BNO055_Vector BNO055_IMU::read_acceleration() {
    BNO055_Vector accel;
    uint8_t buffer[6];
    
    if (read_registers(BNO055_ACCEL_DATA_X_LSB_ADDR, buffer, 6) == BNO055_SUCCESS) {
        accel.x = (int16_t)((buffer[1] << 8) | buffer[0]) / BNO055_ACCEL_LSB_PER_MS2;
        accel.y = (int16_t)((buffer[3] << 8) | buffer[2]) / BNO055_ACCEL_LSB_PER_MS2;
        accel.z = (int16_t)((buffer[5] << 8) | buffer[4]) / BNO055_ACCEL_LSB_PER_MS2;
    } else {
        increment_error_count();
        memset(&accel, 0, sizeof(BNO055_Vector));
    }
    
    return accel;
}

BNO055_Vector BNO055_IMU::read_gyroscope() {
    BNO055_Vector gyro;
    uint8_t buffer[6];
    
    if (read_registers(BNO055_GYRO_DATA_X_LSB_ADDR, buffer, 6) == BNO055_SUCCESS) {
        gyro.x = (int16_t)((buffer[1] << 8) | buffer[0]) / BNO055_GYRO_LSB_PER_RPS;
        gyro.y = (int16_t)((buffer[3] << 8) | buffer[2]) / BNO055_GYRO_LSB_PER_RPS;
        gyro.z = (int16_t)((buffer[5] << 8) | buffer[4]) / BNO055_GYRO_LSB_PER_RPS;
    } else {
        increment_error_count();
        memset(&gyro, 0, sizeof(BNO055_Vector));
    }
    
    return gyro;
}

BNO055_Vector BNO055_IMU::read_magnetometer() {
    BNO055_Vector mag;
    uint8_t buffer[6];
    
    if (read_registers(BNO055_MAG_DATA_X_LSB_ADDR, buffer, 6) == BNO055_SUCCESS) {
        mag.x = (int16_t)((buffer[1] << 8) | buffer[0]) / BNO055_MAG_LSB_PER_UT;
        mag.y = (int16_t)((buffer[3] << 8) | buffer[2]) / BNO055_MAG_LSB_PER_UT;
        mag.z = (int16_t)((buffer[5] << 8) | buffer[4]) / BNO055_MAG_LSB_PER_UT;
    } else {
        increment_error_count();
        memset(&mag, 0, sizeof(BNO055_Vector));
    }
    
    return mag;
}

BNO055_Vector BNO055_IMU::read_linear_acceleration() {
    BNO055_Vector linear_accel;
    uint8_t buffer[6];
    
    if (read_registers(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, buffer, 6) == BNO055_SUCCESS) {
        linear_accel.x = (int16_t)((buffer[1] << 8) | buffer[0]) / BNO055_ACCEL_LSB_PER_MS2;
        linear_accel.y = (int16_t)((buffer[3] << 8) | buffer[2]) / BNO055_ACCEL_LSB_PER_MS2;
        linear_accel.z = (int16_t)((buffer[5] << 8) | buffer[4]) / BNO055_ACCEL_LSB_PER_MS2;
    } else {
        increment_error_count();
        memset(&linear_accel, 0, sizeof(BNO055_Vector));
    }
    
    return linear_accel;
}

BNO055_Vector BNO055_IMU::read_gravity() {
    BNO055_Vector gravity;
    uint8_t buffer[6];
    
    if (read_registers(BNO055_GRAVITY_DATA_X_LSB_ADDR, buffer, 6) == BNO055_SUCCESS) {
        gravity.x = (int16_t)((buffer[1] << 8) | buffer[0]) / BNO055_ACCEL_LSB_PER_MS2;
        gravity.y = (int16_t)((buffer[3] << 8) | buffer[2]) / BNO055_ACCEL_LSB_PER_MS2;
        gravity.z = (int16_t)((buffer[5] << 8) | buffer[4]) / BNO055_ACCEL_LSB_PER_MS2;
    } else {
        increment_error_count();
        memset(&gravity, 0, sizeof(BNO055_Vector));
    }
    
    return gravity;
}

BNO055_EulerAngles BNO055_IMU::read_euler_angles() {
    BNO055_EulerAngles euler;
    uint8_t buffer[6];
    
    if (read_registers(BNO055_EULER_H_LSB_ADDR, buffer, 6) == BNO055_SUCCESS) {
        euler.heading = (int16_t)((buffer[1] << 8) | buffer[0]) / BNO055_EULER_LSB_PER_RAD;
        euler.roll = (int16_t)((buffer[3] << 8) | buffer[2]) / BNO055_EULER_LSB_PER_RAD;
        euler.pitch = (int16_t)((buffer[5] << 8) | buffer[4]) / BNO055_EULER_LSB_PER_RAD;
    } else {
        increment_error_count();
        memset(&euler, 0, sizeof(BNO055_EulerAngles));
    }
    
    return euler;
}

BNO055_Quaternion BNO055_IMU::read_quaternion() {
    BNO055_Quaternion quat;
    uint8_t buffer[8];
    
    if (read_registers(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8) == BNO055_SUCCESS) {
        quat.w = (int16_t)((buffer[1] << 8) | buffer[0]) / BNO055_QUAT_LSB_PER_UNIT;
        quat.x = (int16_t)((buffer[3] << 8) | buffer[2]) / BNO055_QUAT_LSB_PER_UNIT;
        quat.y = (int16_t)((buffer[5] << 8) | buffer[4]) / BNO055_QUAT_LSB_PER_UNIT;
        quat.z = (int16_t)((buffer[7] << 8) | buffer[6]) / BNO055_QUAT_LSB_PER_UNIT;
    } else {
        increment_error_count();
        memset(&quat, 0, sizeof(BNO055_Quaternion));
    }
    
    return quat;
}

int8_t BNO055_IMU::read_temperature() {
    uint8_t temp;
    if (read_register(BNO055_TEMP_ADDR, temp) == BNO055_SUCCESS) {
        return (int8_t)temp;
    } else {
        increment_error_count();
        return 0;
    }
}

BNO055_CalibrationStatus BNO055_IMU::get_calibration_status() {
    update_calibration_status();
    return calib_status;
}

bool BNO055_IMU::is_fully_calibrated() {
    update_calibration_status();
    return (calib_status.system == 3);
}

BNO055_SystemStatus BNO055_IMU::get_system_status() {
    read_register(BNO055_SYS_STAT_ADDR, system_status.system_status);
    read_register(BNO055_SELFTEST_RESULT_ADDR, system_status.self_test_result);
    read_register(BNO055_SYS_ERR_ADDR, system_status.system_error);
    
    return system_status;
}

int8_t BNO055_IMU::perform_self_test() {
    // Set to config mode
    BNO055_OperatingMode prev_mode = current_mode;
    if (set_mode(BNO055_OPERATION_MODE_CONFIG) != BNO055_SUCCESS) {
        return BNO055_ERROR_MODE;
    }
    
    // Trigger self test
    if (write_register(BNO055_SYS_TRIGGER_ADDR, 0x01) != BNO055_SUCCESS) {
        return BNO055_ERROR_I2C;
    }
    
    // Wait for self test to complete
    delay(1000);
    
    // Read self test result
    uint8_t self_test_result;
    if (read_register(BNO055_SELFTEST_RESULT_ADDR, self_test_result) != BNO055_SUCCESS) {
        return BNO055_ERROR_I2C;
    }
    
    // Restore previous mode
    set_mode(prev_mode);
    
    // Check if all tests passed
    if ((self_test_result & 0x0F) == 0x0F) {
        return BNO055_SUCCESS;
    } else {
        return BNO055_ERROR_SELFTEST;
    }
}

imu_data_t BNO055_IMU::get_imu_data_for_protocol() {
    imu_data_t imu_data;
    
    const BNO055_SensorData& data = last_reading;
    
    // Copy quaternion data
    imu_data.quaternion[0] = data.quaternion.w;
    imu_data.quaternion[1] = data.quaternion.x;
    imu_data.quaternion[2] = data.quaternion.y;
    imu_data.quaternion[3] = data.quaternion.z;
    
    // Copy acceleration data
    imu_data.acceleration[0] = data.acceleration.x;
    imu_data.acceleration[1] = data.acceleration.y;
    imu_data.acceleration[2] = data.acceleration.z;
    
    // Copy gyroscope data
    imu_data.gyroscope[0] = data.gyroscope.x;
    imu_data.gyroscope[1] = data.gyroscope.y;
    imu_data.gyroscope[2] = data.gyroscope.z;
    
    // Set calibration status
    update_calibration_status();
    imu_data.calibration_status = (calib_status.system << 6) | 
                                  (calib_status.gyroscope << 4) | 
                                  (calib_status.accelerometer << 2) | 
                                  calib_status.magnetometer;
    
    // Set system status
    get_system_status();
    imu_data.system_status = system_status.system_status;
    
    return imu_data;
}

float BNO055_IMU::get_success_rate() const {
    if (measurement_count == 0) return 0.0f;
    return ((float)(measurement_count - error_count) / measurement_count) * 100.0f;
}

void BNO055_IMU::reset_statistics() {
    measurement_count = 0;
    error_count = 0;
}

void BNO055_IMU::print_calibration_status() {
    update_calibration_status();
    
    Serial.println("BNO055 Calibration Status:");
    Serial.print("System: "); Serial.print(calib_status.system); Serial.println("/3");
    Serial.print("Gyroscope: "); Serial.print(calib_status.gyroscope); Serial.println("/3");
    Serial.print("Accelerometer: "); Serial.print(calib_status.accelerometer); Serial.println("/3");
    Serial.print("Magnetometer: "); Serial.print(calib_status.magnetometer); Serial.println("/3");
    Serial.print("Fully Calibrated: "); Serial.println(is_fully_calibrated() ? "YES" : "NO");
}

void BNO055_IMU::print_system_status() {
    get_system_status();
    
    Serial.println("BNO055 System Status:");
    Serial.print("System Status: "); 
    Serial.println(BNO055_Utils::system_status_to_string(system_status.system_status));
    Serial.print("Self Test Result: 0x"); Serial.println(system_status.self_test_result, HEX);
    Serial.print("System Error: 0x"); Serial.println(system_status.system_error, HEX);
}

void BNO055_IMU::print_sensor_info() {
    uint8_t chip_id, accel_id, mag_id, gyro_id;
    uint16_t sw_rev_id, bl_rev_id;
    
    read_register(BNO055_CHIP_ID_ADDR, chip_id);
    read_register(BNO055_ACCEL_REV_ID_ADDR, accel_id);
    read_register(BNO055_MAG_REV_ID_ADDR, mag_id);
    read_register(BNO055_GYRO_REV_ID_ADDR, gyro_id);
    
    uint8_t sw_lsb, sw_msb;
    read_register(BNO055_SW_REV_ID_LSB_ADDR, sw_lsb);
    read_register(BNO055_SW_REV_ID_MSB_ADDR, sw_msb);
    sw_rev_id = (sw_msb << 8) | sw_lsb;
    
    read_register(BNO055_BL_REV_ID_ADDR, (uint8_t&)bl_rev_id);
    
    Serial.println("BNO055 Sensor Information:");
    Serial.print("Chip ID: 0x"); Serial.println(chip_id, HEX);
    Serial.print("Accelerometer ID: 0x"); Serial.println(accel_id, HEX);
    Serial.print("Magnetometer ID: 0x"); Serial.println(mag_id, HEX);
    Serial.print("Gyroscope ID: 0x"); Serial.println(gyro_id, HEX);
    Serial.print("Software Revision: "); Serial.println(sw_rev_id);
    Serial.print("Bootloader Revision: "); Serial.println(bl_rev_id);
}

void BNO055_IMU::enable_external_crystal() {
    if (write_register(BNO055_SYS_TRIGGER_ADDR, 0x80) == BNO055_SUCCESS) {
        delay(10);
    }
}

// Private functions
int8_t BNO055_IMU::write_register(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(i2c_address);
    Wire.write(reg);
    Wire.write(value);
    
    if (Wire.endTransmission() != 0) {
        return BNO055_ERROR_I2C;
    }
    
    delay(2); // BNO055 requires delay between writes
    return BNO055_SUCCESS;
}

int8_t BNO055_IMU::read_register(uint8_t reg, uint8_t& value) {
    Wire.beginTransmission(i2c_address);
    Wire.write(reg);
    
    if (Wire.endTransmission() != 0) {
        return BNO055_ERROR_I2C;
    }
    
    Wire.requestFrom(i2c_address, (uint8_t)1);
    
    if (Wire.available()) {
        value = Wire.read();
        return BNO055_SUCCESS;
    }
    
    return BNO055_ERROR_I2C;
}

int8_t BNO055_IMU::read_registers(uint8_t reg, uint8_t* buffer, uint8_t length) {
    Wire.beginTransmission(i2c_address);
    Wire.write(reg);
    
    if (Wire.endTransmission() != 0) {
        return BNO055_ERROR_I2C;
    }
    
    Wire.requestFrom(i2c_address, length);
    
    uint8_t i = 0;
    while (Wire.available() && i < length) {
        buffer[i++] = Wire.read();
    }
    
    if (i == length) {
        return BNO055_SUCCESS;
    }
    
    return BNO055_ERROR_I2C;
}

bool BNO055_IMU::check_chip_id() {
    uint8_t chip_id;
    
    for (int i = 0; i < 5; i++) { // Try multiple times
        if (read_register(BNO055_CHIP_ID_ADDR, chip_id) == BNO055_SUCCESS) {
            if (chip_id == BNO055_ID) {
                return true;
            }
        }
        delay(10);
    }
    
    return false;
}

int8_t BNO055_IMU::wait_for_mode_change() {
    uint32_t start_time = millis();
    
    while (millis() - start_time < 1000) { // 1 second timeout
        uint8_t mode;
        if (read_register(BNO055_OPR_MODE_ADDR, mode) == BNO055_SUCCESS) {
            if (mode == (uint8_t)current_mode) {
                delay(19); // Additional delay for mode stabilization
                return BNO055_SUCCESS;
            }
        }
        delay(10);
    }
    
    return BNO055_ERROR_MODE;
}

void BNO055_IMU::increment_error_count() {
    error_count++;
}

void BNO055_IMU::update_calibration_status() {
    uint8_t calib_stat;
    if (read_register(BNO055_CALIB_STAT_ADDR, calib_stat) == BNO055_SUCCESS) {
        calib_status.magnetometer = calib_stat & BNO055_CALIB_STAT_MAG_MASK;
        calib_status.accelerometer = (calib_stat & BNO055_CALIB_STAT_ACC_MASK) >> 2;
        calib_status.gyroscope = (calib_stat & BNO055_CALIB_STAT_GYR_MASK) >> 4;
        calib_status.system = (calib_stat & BNO055_CALIB_STAT_SYS_MASK) >> 6;
    }
}

// Utility functions
namespace BNO055_Utils {
    BNO055_Quaternion quaternion_multiply(const BNO055_Quaternion& q1, const BNO055_Quaternion& q2) {
        BNO055_Quaternion result;
        result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
        result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
        result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
        result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
        return result;
    }
    
    BNO055_Quaternion quaternion_normalize(const BNO055_Quaternion& q) {
        float norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        BNO055_Quaternion result;
        
        if (norm > 0.0f) {
            result.w = q.w / norm;
            result.x = q.x / norm;
            result.y = q.y / norm;
            result.z = q.z / norm;
        } else {
            result = {1.0f, 0.0f, 0.0f, 0.0f};
        }
        
        return result;
    }
    
    BNO055_EulerAngles quaternion_to_euler(const BNO055_Quaternion& q) {
        BNO055_EulerAngles euler;
        
        // Roll (x-axis rotation)
        float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        euler.roll = atan2(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        float sinp = 2 * (q.w * q.y - q.z * q.x);
        if (abs(sinp) >= 1)
            euler.pitch = copysign(M_PI / 2, sinp);
        else
            euler.pitch = asin(sinp);
        
        // Yaw (z-axis rotation)
        float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        euler.heading = atan2(siny_cosp, cosy_cosp);
        
        return euler;
    }
    
    float vector_magnitude(const BNO055_Vector& v) {
        return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }
    
    BNO055_Vector vector_normalize(const BNO055_Vector& v) {
        float mag = vector_magnitude(v);
        BNO055_Vector result;
        
        if (mag > 0.0f) {
            result.x = v.x / mag;
            result.y = v.y / mag;
            result.z = v.z / mag;
        } else {
            result = {0.0f, 0.0f, 0.0f};
        }
        
        return result;
    }
    
    const char* system_status_to_string(uint8_t status) {
        switch (status) {
            case BNO055_SYS_STAT_IDLE: return "Idle";
            case BNO055_SYS_STAT_SYSTEM_ERROR: return "System Error";
            case BNO055_SYS_STAT_INIT_PERIPHERALS: return "Initializing Peripherals";
            case BNO055_SYS_STAT_SYSTEM_INIT: return "System Initialization";
            case BNO055_SYS_STAT_EXECUTING_SELFTEST: return "Executing Self Test";
            case BNO055_SYS_STAT_SENSOR_FUSION_RUNNING: return "Sensor Fusion Running";
            case BNO055_SYS_STAT_SYSTEM_RUNNING_NO_FUSION: return "System Running (No Fusion)";
            default: return "Unknown";
        }
    }
    
    const char* calibration_status_to_string(uint8_t status) {
        switch (status) {
            case 0: return "Not Calibrated";
            case 1: return "Minimally Calibrated";
            case 2: return "Mostly Calibrated";
            case 3: return "Fully Calibrated";
            default: return "Unknown";
        }
    }
    
    bool is_calibration_acceptable(const BNO055_CalibrationStatus& status) {
        // For navigation use, we want good gyro and accel calibration
        // Magnetometer can be less critical depending on application
        return (status.system >= 2 && status.gyroscope >= 2 && status.accelerometer >= 2);
    }
}