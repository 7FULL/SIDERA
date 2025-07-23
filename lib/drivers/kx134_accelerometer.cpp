#include "kx134_accelerometer.h"
#include <math.h>

KX134_Accelerometer* g_high_g_accel = nullptr;

KX134_Accelerometer::KX134_Accelerometer(uint8_t chip_select_pin) {
    cs_pin = chip_select_pin;
    spi_speed = KX134_SPI_SPEED;
    initialized = false;
    calibrated = false;
    
    measurement_count = 0;
    error_count = 0;
    scale_factor = 1.0f;
    
    // Initialize structures
    memset(&config, 0, sizeof(KX134_Config));
    memset(&last_reading, 0, sizeof(KX134_SensorData));
    
    // Default calibration (no offset, unit scale)
    offset = {0.0f, 0.0f, 0.0f};
    scale = {1.0f, 1.0f, 1.0f};
}

int8_t KX134_Accelerometer::begin(uint8_t range, uint8_t odr) {
    // Initialize CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
    
    // Initialize SPI
    SPI.begin();
    
    // Reset the device
    if (reset() != KX134_SUCCESS) {
        return KX134_ERROR_INIT;
    }
    
    // Check WHO_AM_I
    if (!check_who_am_i()) {
        return KX134_ERROR_ID;
    }
    
    // Enter standby mode for configuration
    if (enter_standby_mode() != KX134_SUCCESS) {
        return KX134_ERROR_INIT;
    }
    
    // Set range
    if (set_range(range) != KX134_SUCCESS) {
        return KX134_ERROR_RANGE;
    }
    
    // Set output data rate
    if (set_output_data_rate(odr) != KX134_SUCCESS) {
        return KX134_ERROR_ODR;
    }
    
    // Set high resolution mode
    if (set_resolution(KX134_RES_16BIT) != KX134_SUCCESS) {
        return KX134_ERROR_INIT;
    }
    
    // Enable data ready engine
    uint8_t cntl1_val;
    if (read_register(KX134_CNTL1, cntl1_val) != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    cntl1_val |= KX134_CNTL1_DRDYE;
    if (write_register(KX134_CNTL1, cntl1_val) != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    // Enter operating mode
    if (enter_operating_mode() != KX134_SUCCESS) {
        return KX134_ERROR_INIT;
    }
    
    // Store configuration
    config.range = range;
    config.odr = odr;
    config.resolution = KX134_RES_16BIT;
    config.high_performance = true;
    
    update_scale_factor();
    
    initialized = true;
    return KX134_SUCCESS;
}

int8_t KX134_Accelerometer::reset() {
    // Software reset
    if (write_register(KX134_CNTL2, KX134_CNTL2_SRST) != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    // Wait for reset to complete
    delay(50);
    
    return KX134_SUCCESS;
}

int8_t KX134_Accelerometer::set_range(uint8_t range) {
    if (range > KX134_RANGE_64G) {
        return KX134_ERROR_RANGE;
    }
    
    // Enter standby mode
    if (enter_standby_mode() != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    // Read current CNTL1 register
    uint8_t cntl1_val;
    if (read_register(KX134_CNTL1, cntl1_val) != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    // Clear range bits and set new range
    cntl1_val &= ~(KX134_CNTL1_GSEL1 | KX134_CNTL1_GSEL0);
    cntl1_val |= range;
    
    if (write_register(KX134_CNTL1, cntl1_val) != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    // Enter operating mode
    if (enter_operating_mode() != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    config.range = range;
    update_scale_factor();
    
    return KX134_SUCCESS;
}

int8_t KX134_Accelerometer::set_output_data_rate(uint8_t odr) {
    if (odr > KX134_ODR_25600HZ) {
        return KX134_ERROR_ODR;
    }
    
    // Enter standby mode
    if (enter_standby_mode() != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    // Set ODR in ODCNTL register
    if (write_register(KX134_ODCNTL, odr) != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    // Enter operating mode
    if (enter_operating_mode() != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    config.odr = odr;
    
    return KX134_SUCCESS;
}

int8_t KX134_Accelerometer::set_resolution(uint8_t resolution) {
    // Enter standby mode
    if (enter_standby_mode() != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    // Read current CNTL1 register
    uint8_t cntl1_val;
    if (read_register(KX134_CNTL1, cntl1_val) != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    // Set resolution
    if (resolution == KX134_RES_16BIT) {
        cntl1_val |= KX134_CNTL1_RES;
    } else {
        cntl1_val &= ~KX134_CNTL1_RES;
    }
    
    if (write_register(KX134_CNTL1, cntl1_val) != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    // Enter operating mode
    if (enter_operating_mode() != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    config.resolution = resolution;
    
    return KX134_SUCCESS;
}

int8_t KX134_Accelerometer::read_acceleration(KX134_Vector& accel) {
    KX134_RawData raw_data;
    
    if (read_raw_data(raw_data) != KX134_SUCCESS) {
        increment_error_count();
        return KX134_ERROR_SPI;
    }
    
    // Convert raw data to g
    convert_raw_to_g(raw_data, accel);
    
    // Apply calibration
    apply_calibration(accel);
    
    // Update last reading
    last_reading.acceleration = accel;
    last_reading.temperature = raw_data.temperature * 0.5f - 25.0f; // Temperature conversion
    last_reading.timestamp = millis();
    last_reading.valid = true;
    
    measurement_count++;
    
    return KX134_SUCCESS;
}

int8_t KX134_Accelerometer::read_raw_data(KX134_RawData& raw_data) {
    uint8_t buffer[8];
    
    // Read all data registers in burst mode
    if (read_registers(KX134_XOUT_L, buffer, 8) != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    // Parse acceleration data (16-bit, little endian)
    raw_data.x = (int16_t)((buffer[1] << 8) | buffer[0]);
    raw_data.y = (int16_t)((buffer[3] << 8) | buffer[2]);
    raw_data.z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    // Parse temperature data
    raw_data.temperature = (int16_t)((buffer[7] << 8) | buffer[6]);
    
    return KX134_SUCCESS;
}

float KX134_Accelerometer::read_temperature() {
    uint8_t temp_l, temp_h;
    
    if (read_register(KX134_TEMP_OUT_L, temp_l) != KX134_SUCCESS ||
        read_register(KX134_TEMP_OUT_H, temp_h) != KX134_SUCCESS) {
        increment_error_count();
        return 0.0f;
    }
    
    int16_t raw_temp = (int16_t)((temp_h << 8) | temp_l);
    return raw_temp * 0.5f - 25.0f; // Convert to Celsius
}

int8_t KX134_Accelerometer::calibrate_zero_g(uint16_t samples) {
    if (!initialized) {
        return KX134_ERROR_INIT;
    }
    
    Serial.println("Starting zero-G calibration...");
    Serial.println("Keep sensor stationary and level");
    
    KX134_Vector sum = {0, 0, 0};
    uint16_t valid_samples = 0;
    
    for (uint16_t i = 0; i < samples; i++) {
        KX134_Vector accel;
        
        if (read_acceleration(accel) == KX134_SUCCESS) {
            // Don't apply calibration during calibration
            KX134_Vector raw_accel = accel;
            raw_accel.x = (raw_accel.x / scale.x) + offset.x;
            raw_accel.y = (raw_accel.y / scale.y) + offset.y;
            raw_accel.z = (raw_accel.z / scale.z) + offset.z;
            
            sum.x += raw_accel.x;
            sum.y += raw_accel.y;
            sum.z += raw_accel.z;
            valid_samples++;
        }
        
        delay(10); // 100Hz sampling
        
        if (i % 100 == 0) {
            Serial.print("Calibration progress: ");
            Serial.print((i * 100) / samples);
            Serial.println("%");
        }
    }
    
    if (valid_samples < samples * 0.8) {
        Serial.println("Calibration failed - too many invalid readings");
        return KX134_ERROR_INIT;
    }
    
    // Calculate offset (assuming Z should be 1g, X and Y should be 0g)
    offset.x = sum.x / valid_samples;
    offset.y = sum.y / valid_samples;
    offset.z = (sum.z / valid_samples) - 1.0f; // Remove 1g from Z-axis
    
    // Reset scale to unity for basic calibration
    scale.x = 1.0f;
    scale.y = 1.0f;
    scale.z = 1.0f;
    
    calibrated = true;
    
    Serial.println("Zero-G calibration complete:");
    Serial.print("X offset: "); Serial.println(offset.x, 4);
    Serial.print("Y offset: "); Serial.println(offset.y, 4);
    Serial.print("Z offset: "); Serial.println(offset.z, 4);
    
    return KX134_SUCCESS;
}

int8_t KX134_Accelerometer::perform_self_test() {
    Serial.println("Performing KX134 self test...");
    
    // Check communication
    if (!verify_communication()) {
        Serial.println("Communication test failed");
        return KX134_ERROR_SPI;
    }
    
    // Read a few samples and check they're reasonable
    KX134_Vector accel;
    float total_g = 0;
    int valid_readings = 0;
    
    for (int i = 0; i < 10; i++) {
        if (read_acceleration(accel) == KX134_SUCCESS) {
            total_g = sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
            if (total_g > 0.5f && total_g < 2.0f) {
                valid_readings++;
            }
        }
        delay(10);
    }
    
    if (valid_readings >= 8) {
        Serial.println("Self test passed");
        return KX134_SUCCESS;
    } else {
        Serial.println("Self test failed - invalid readings");
        return KX134_ERROR_INIT;
    }
}

bool KX134_Accelerometer::verify_communication() {
    return check_who_am_i();
}

float KX134_Accelerometer::get_range_g() const {
    switch (config.range) {
        case KX134_RANGE_8G: return 8.0f;
        case KX134_RANGE_16G: return 16.0f;
        case KX134_RANGE_32G: return 32.0f;
        case KX134_RANGE_64G: return 64.0f;
        default: return 64.0f;
    }
}

float KX134_Accelerometer::get_success_rate() const {
    if (measurement_count == 0) return 0.0f;
    return ((float)(measurement_count - error_count) / measurement_count) * 100.0f;
}

void KX134_Accelerometer::reset_statistics() {
    measurement_count = 0;
    error_count = 0;
}

float KX134_Accelerometer::raw_to_g(int16_t raw_value) const {
    return (float)raw_value / scale_factor;
}

int16_t KX134_Accelerometer::g_to_raw(float g_value) const {
    return (int16_t)(g_value * scale_factor);
}

void KX134_Accelerometer::print_sensor_info() {
    Serial.println("KX134-1211 High-G Accelerometer");
    Serial.print("Range: ±"); Serial.print(get_range_g()); Serial.println("g");
    Serial.print("Resolution: "); Serial.println(config.resolution == KX134_RES_16BIT ? "16-bit" : "8-bit");
    Serial.print("ODR: "); Serial.println(config.odr);
    Serial.print("Scale Factor: "); Serial.println(scale_factor);
    Serial.print("Calibrated: "); Serial.println(calibrated ? "Yes" : "No");
    Serial.print("Success Rate: "); Serial.print(get_success_rate()); Serial.println("%");
}

// Private functions
void KX134_Accelerometer::spi_begin_transaction() {
    SPI.beginTransaction(SPISettings(spi_speed, KX134_SPI_BITORDER, KX134_SPI_MODE));
}

void KX134_Accelerometer::spi_end_transaction() {
    SPI.endTransaction();
}

uint8_t KX134_Accelerometer::spi_transfer(uint8_t data) {
    return SPI.transfer(data);
}

int8_t KX134_Accelerometer::write_register(uint8_t reg, uint8_t value) {
    spi_begin_transaction();
    
    digitalWrite(cs_pin, LOW);
    spi_transfer(reg & 0x7F); // Write bit (MSB = 0)
    spi_transfer(value);
    digitalWrite(cs_pin, HIGH);
    
    spi_end_transaction();
    
    delay(1); // Small delay for register write
    return KX134_SUCCESS;
}

int8_t KX134_Accelerometer::read_register(uint8_t reg, uint8_t& value) {
    spi_begin_transaction();
    
    digitalWrite(cs_pin, LOW);
    spi_transfer(reg | 0x80); // Read bit (MSB = 1)
    value = spi_transfer(0x00);
    digitalWrite(cs_pin, HIGH);
    
    spi_end_transaction();
    
    return KX134_SUCCESS;
}

int8_t KX134_Accelerometer::read_registers(uint8_t reg, uint8_t* buffer, uint8_t length) {
    spi_begin_transaction();
    
    digitalWrite(cs_pin, LOW);
    spi_transfer(reg | 0x80); // Read bit (MSB = 1)
    
    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = spi_transfer(0x00);
    }
    
    digitalWrite(cs_pin, HIGH);
    spi_end_transaction();
    
    return KX134_SUCCESS;
}

void KX134_Accelerometer::update_scale_factor() {
    scale_factor = range_to_scale_factor(config.range);
}

int8_t KX134_Accelerometer::enter_standby_mode() {
    uint8_t cntl1_val;
    if (read_register(KX134_CNTL1, cntl1_val) != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    cntl1_val &= ~KX134_CNTL1_PC1; // Clear PC1 bit
    
    if (write_register(KX134_CNTL1, cntl1_val) != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    delay(2); // Wait for mode change
    return KX134_SUCCESS;
}

int8_t KX134_Accelerometer::enter_operating_mode() {
    uint8_t cntl1_val;
    if (read_register(KX134_CNTL1, cntl1_val) != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    cntl1_val |= KX134_CNTL1_PC1; // Set PC1 bit
    
    if (write_register(KX134_CNTL1, cntl1_val) != KX134_SUCCESS) {
        return KX134_ERROR_SPI;
    }
    
    delay(2); // Wait for mode change
    return KX134_SUCCESS;
}

bool KX134_Accelerometer::check_who_am_i() {
    uint8_t who_am_i;
    
    for (int i = 0; i < 5; i++) { // Try multiple times
        if (read_register(KX134_WHO_AM_I, who_am_i) == KX134_SUCCESS) {
            if (who_am_i == KX134_WHO_AM_I_VAL) {
                return true;
            }
        }
        delay(10);
    }
    
    return false;
}

void KX134_Accelerometer::increment_error_count() {
    error_count++;
}

void KX134_Accelerometer::apply_calibration(KX134_Vector& accel) {
    if (calibrated) {
        accel.x = (accel.x - offset.x) * scale.x;
        accel.y = (accel.y - offset.y) * scale.y;
        accel.z = (accel.z - offset.z) * scale.z;
    }
}

void KX134_Accelerometer::convert_raw_to_g(const KX134_RawData& raw, KX134_Vector& accel) {
    accel.x = raw_to_g(raw.x);
    accel.y = raw_to_g(raw.y);
    accel.z = raw_to_g(raw.z);
}

float KX134_Accelerometer::range_to_scale_factor(uint8_t range) {
    // Scale factors for 16-bit mode (LSB/g)
    switch (range) {
        case KX134_RANGE_8G: return 4096.0f;   // ±8g -> 16384 LSB / 8g = 2048 LSB/g per side
        case KX134_RANGE_16G: return 2048.0f;  // ±16g -> 16384 LSB / 16g = 1024 LSB/g per side
        case KX134_RANGE_32G: return 1024.0f;  // ±32g -> 16384 LSB / 32g = 512 LSB/g per side
        case KX134_RANGE_64G: return 512.0f;   // ±64g -> 16384 LSB / 64g = 256 LSB/g per side
        default: return 512.0f;
    }
}

// Utility functions
namespace KX134_Utils {
    float calculate_total_g_force(const KX134_Vector& accel) {
        return sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    }
    
    float calculate_vertical_g(const KX134_Vector& accel, const KX134_Vector& gravity_ref) {
        // Dot product with gravity reference vector (normalized)
        float ref_mag = sqrt(gravity_ref.x * gravity_ref.x + gravity_ref.y * gravity_ref.y + gravity_ref.z * gravity_ref.z);
        if (ref_mag == 0) return 0.0f;
        
        return (accel.x * gravity_ref.x + accel.y * gravity_ref.y + accel.z * gravity_ref.z) / ref_mag;
    }
    
    KX134_Utils::PeakData find_acceleration_peak(KX134_SensorData* data, uint16_t length) {
        PeakData peak = {0, 0, 0, 0, 0};
        
        if (length == 0) return peak;
        
        float max_g = calculate_total_g_force(data[0].acceleration);
        float min_g = max_g;
        uint32_t max_time = data[0].timestamp;
        uint32_t min_time = data[0].timestamp;
        
        for (uint16_t i = 1; i < length; i++) {
            float total_g = calculate_total_g_force(data[i].acceleration);
            
            if (total_g > max_g) {
                max_g = total_g;
                max_time = data[i].timestamp;
            }
            
            if (total_g < min_g) {
                min_g = total_g;
                min_time = data[i].timestamp;
            }
        }
        
        peak.max_g = max_g;
        peak.min_g = min_g;
        peak.max_timestamp = max_time;
        peak.min_timestamp = min_time;
        peak.duration_ms = (max_time > min_time) ? (max_time - min_time) : (min_time - max_time);
        
        return peak;
    }
    
    bool detect_shock_event(const KX134_Vector& accel, float threshold_g) {
        float total_g = calculate_total_g_force(accel);
        return (total_g > threshold_g);
    }
    
    bool detect_launch_signature(KX134_SensorData* data, uint16_t length, float threshold_g) {
        if (length < 10) return false;
        
        // Look for sustained high acceleration
        uint16_t high_g_count = 0;
        
        for (uint16_t i = 0; i < length; i++) {
            float total_g = calculate_total_g_force(data[i].acceleration);
            if (total_g > threshold_g) {
                high_g_count++;
            }
        }
        
        // Launch detected if more than 50% of samples are above threshold
        return (high_g_count > length / 2);
    }
    
    KX134_Vector apply_low_pass_filter(const KX134_Vector& current, const KX134_Vector& previous, float alpha) {
        KX134_Vector filtered;
        filtered.x = alpha * current.x + (1.0f - alpha) * previous.x;
        filtered.y = alpha * current.y + (1.0f - alpha) * previous.y;
        filtered.z = alpha * current.z + (1.0f - alpha) * previous.z;
        return filtered;
    }
    
    KX134_Vector apply_high_pass_filter(const KX134_Vector& current, const KX134_Vector& previous, float alpha) {
        KX134_Vector filtered;
        filtered.x = alpha * (current.x - previous.x);
        filtered.y = alpha * (current.y - previous.y);
        filtered.z = alpha * (current.z - previous.z);
        return filtered;
    }
    
    KX134_Utils::FlightPhase analyze_flight_phase(const KX134_Vector& accel, float total_g) {
        if (total_g < 2.0f) {
            return PHASE_GROUND;
        } else if (total_g > 10.0f) {
            return PHASE_BOOST;
        } else if (total_g > 2.0f && total_g < 5.0f) {
            return PHASE_COAST;
        } else if (total_g > 5.0f && total_g < 10.0f) {
            return PHASE_DESCENT;
        } else {
            return PHASE_LANDED;
        }
    }
    
    bool is_sensor_level(const KX134_Vector& accel, float tolerance) {
        float total_g = calculate_total_g_force(accel);
        return (abs(total_g - 1.0f) < tolerance && abs(accel.x) < tolerance && abs(accel.y) < tolerance);
    }
    
    KX134_Vector calculate_gravity_reference(KX134_SensorData* data, uint16_t length) {
        KX134_Vector gravity_sum = {0, 0, 0};
        uint16_t valid_samples = 0;
        
        for (uint16_t i = 0; i < length; i++) {
            if (data[i].valid) {
                gravity_sum.x += data[i].acceleration.x;
                gravity_sum.y += data[i].acceleration.y;
                gravity_sum.z += data[i].acceleration.z;
                valid_samples++;
            }
        }
        
        if (valid_samples > 0) {
            gravity_sum.x /= valid_samples;
            gravity_sum.y /= valid_samples;
            gravity_sum.z /= valid_samples;
        }
        
        return gravity_sum;
    }
}