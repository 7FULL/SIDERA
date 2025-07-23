#include "ms5611_barometer.h"
#include <math.h>

MS5611_Barometer* g_barometer = nullptr;

MS5611_Barometer::MS5611_Barometer(uint8_t chip_select_pin, uint8_t osr) {
    cs_pin = chip_select_pin;
    osr_setting = osr;
    spi_speed = MS5611_SPI_SPEED;
    initialized = false;
    
    measurement_count = 0;
    error_count = 0;
    sea_level_pressure = MS5611_SEA_LEVEL_PRESSURE;
    
    // Initialize structures
    memset(&calibration, 0, sizeof(MS5611_Calibration));
    memset(&last_reading, 0, sizeof(MS5611_Data));
    
    dT = 0;
    OFF = 0;
    SENS = 0;
}

int8_t MS5611_Barometer::begin() {
    // Initialize CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
    
    // Initialize SPI
    SPI.begin();
    
    // Reset the device
    if (reset() != MS5611_SUCCESS) {
        return MS5611_ERROR_INIT;
    }
    
    // Read calibration data
    if (read_calibration() != MS5611_SUCCESS) {
        return MS5611_ERROR_INIT;
    }
    
    // Validate calibration CRC
    if (!validate_calibration()) {
        return MS5611_ERROR_CRC;
    }
    
    initialized = true;
    return MS5611_SUCCESS;
}

int8_t MS5611_Barometer::reset() {
    spi_begin_transaction();
    
    digitalWrite(cs_pin, LOW);
    spi_transfer(MS5611_CMD_RESET);
    digitalWrite(cs_pin, HIGH);
    
    spi_end_transaction();
    
    // Wait for reset to complete
    delay(3);
    
    return MS5611_SUCCESS;
}

void MS5611_Barometer::set_oversampling(uint8_t osr) {
    if (osr <= MS5611_OSR_4096) {
        osr_setting = osr;
    }
}

void MS5611_Barometer::set_sea_level_pressure(float pressure_hpa) {
    sea_level_pressure = pressure_hpa;
}

int8_t MS5611_Barometer::read_calibration() {
    uint16_t prom_data[8];
    
    spi_begin_transaction();
    
    // Read all PROM registers
    for (int i = 0; i < 8; i++) {
        digitalWrite(cs_pin, LOW);
        spi_transfer(MS5611_CMD_PROM_READ + (i * 2));
        prom_data[i] = spi_read_16bit();
        digitalWrite(cs_pin, HIGH);
        delayMicroseconds(10);
    }
    
    spi_end_transaction();
    
    // Parse calibration data
    calibration.manufacturer = prom_data[0];
    calibration.C1 = prom_data[1];
    calibration.C2 = prom_data[2];
    calibration.C3 = prom_data[3];
    calibration.C4 = prom_data[4];
    calibration.C5 = prom_data[5];
    calibration.C6 = prom_data[6];
    calibration.crc = prom_data[7];
    
    return MS5611_SUCCESS;
}

bool MS5611_Barometer::validate_calibration() {
    return validate_crc();
}

int8_t MS5611_Barometer::read_pressure_temperature(float& pressure, float& temperature) {
    uint32_t D1, D2;
    
    // Start pressure conversion
    if (start_pressure_conversion() != MS5611_SUCCESS) {
        increment_error_count();
        return MS5611_ERROR_SPI;
    }
    
    // Wait for conversion
    delay(get_conversion_delay());
    
    // Read pressure ADC
    D1 = read_adc();
    if (D1 == 0) {
        increment_error_count();
        return MS5611_ERROR_SPI;
    }
    
    // Start temperature conversion
    if (start_temperature_conversion() != MS5611_SUCCESS) {
        increment_error_count();
        return MS5611_ERROR_SPI;
    }
    
    // Wait for conversion
    delay(get_conversion_delay());
    
    // Read temperature ADC
    D2 = read_adc();
    if (D2 == 0) {
        increment_error_count();
        return MS5611_ERROR_SPI;
    }
    
    // Calculate compensated values
    calculate_compensated_values(D1, D2);
    
    // Convert to physical units
    pressure = (float)((OFF + (((int64_t)D1 * SENS) >> 21)) >> 15) / 100.0f;
    temperature = (float)(2000 + dT * (int64_t)calibration.C6 / 8388608) / 100.0f;
    
    // Apply second order temperature compensation
    int32_t temp_comp = (int32_t)(temperature * 100);
    int64_t off_comp = OFF;
    int64_t sens_comp = SENS;
    apply_second_order_compensation(temp_comp, off_comp, sens_comp);
    
    // Recalculate with compensation
    pressure = (float)((off_comp + (((int64_t)D1 * sens_comp) >> 21)) >> 15) / 100.0f;
    temperature = (float)temp_comp / 100.0f;
    
    // Update last reading
    last_reading.pressure = pressure;
    last_reading.temperature = temperature;
    last_reading.raw_pressure = D1;
    last_reading.raw_temperature = D2;
    last_reading.timestamp = millis();
    last_reading.valid = true;
    
    measurement_count++;
    return MS5611_SUCCESS;
}

int8_t MS5611_Barometer::read_pressure_temperature_altitude(float& pressure, float& temperature, float& altitude) {
    int8_t result = read_pressure_temperature(pressure, temperature);
    
    if (result == MS5611_SUCCESS) {
        altitude = pressure_to_altitude(pressure, sea_level_pressure);
        last_reading.altitude = altitude;
    }
    
    return result;
}

int8_t MS5611_Barometer::start_pressure_conversion() {
    spi_begin_transaction();
    
    digitalWrite(cs_pin, LOW);
    spi_transfer(MS5611_CMD_CONVERT_D1 + osr_setting);
    digitalWrite(cs_pin, HIGH);
    
    spi_end_transaction();
    
    return MS5611_SUCCESS;
}

int8_t MS5611_Barometer::start_temperature_conversion() {
    spi_begin_transaction();
    
    digitalWrite(cs_pin, LOW);
    spi_transfer(MS5611_CMD_CONVERT_D2 + osr_setting);
    digitalWrite(cs_pin, HIGH);
    
    spi_end_transaction();
    
    return MS5611_SUCCESS;
}

uint32_t MS5611_Barometer::read_adc() {
    uint32_t result;
    
    spi_begin_transaction();
    
    digitalWrite(cs_pin, LOW);
    spi_transfer(MS5611_CMD_ADC_READ);
    result = spi_read_24bit();
    digitalWrite(cs_pin, HIGH);
    
    spi_end_transaction();
    
    return result;
}

float MS5611_Barometer::pressure_to_altitude(float pressure_hpa, float sea_level_hpa) {
    // Barometric formula for altitude calculation
    return 44330.0f * (1.0f - pow(pressure_hpa / sea_level_hpa, 0.1903f));
}

float MS5611_Barometer::altitude_to_pressure(float altitude_m, float sea_level_hpa) {
    // Inverse barometric formula
    return sea_level_hpa * pow(1.0f - altitude_m / 44330.0f, 5.255f);
}

float MS5611_Barometer::calculate_sea_level_pressure(float pressure_hpa, float altitude_m) {
    // Calculate sea level pressure from current pressure and known altitude
    return pressure_hpa / pow(1.0f - altitude_m / 44330.0f, 5.255f);
}

void MS5611_Barometer::perform_self_test() {
    Serial.println("MS5611 Self Test:");
    Serial.print("Initialization: ");
    Serial.println(initialized ? "PASS" : "FAIL");
    
    print_calibration();
    
    float pressure, temperature, altitude;
    int8_t result = read_pressure_temperature_altitude(pressure, temperature, altitude);
    
    Serial.print("Reading: ");
    Serial.println(result == MS5611_SUCCESS ? "PASS" : "FAIL");
    
    if (result == MS5611_SUCCESS) {
        Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" hPa");
        Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");
        Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" m");
    }
    
    Serial.print("Success Rate: "); Serial.print(get_success_rate()); Serial.println("%");
}

float MS5611_Barometer::get_success_rate() const {
    if (measurement_count == 0) return 0.0f;
    return ((float)(measurement_count - error_count) / measurement_count) * 100.0f;
}

void MS5611_Barometer::reset_statistics() {
    measurement_count = 0;
    error_count = 0;
}

void MS5611_Barometer::print_calibration() {
    Serial.println("MS5611 Calibration Data:");
    Serial.print("Manufacturer: 0x"); Serial.println(calibration.manufacturer, HEX);
    Serial.print("C1 (SENS_T1): "); Serial.println(calibration.C1);
    Serial.print("C2 (OFF_T1): "); Serial.println(calibration.C2);
    Serial.print("C3 (TCS): "); Serial.println(calibration.C3);
    Serial.print("C4 (TCO): "); Serial.println(calibration.C4);
    Serial.print("C5 (T_REF): "); Serial.println(calibration.C5);
    Serial.print("C6 (TEMPSENS): "); Serial.println(calibration.C6);
    Serial.print("CRC: 0x"); Serial.println(calibration.crc, HEX);
    Serial.print("CRC Valid: "); Serial.println(validate_crc() ? "YES" : "NO");
}

// Private functions
void MS5611_Barometer::spi_begin_transaction() {
    SPI.beginTransaction(SPISettings(spi_speed, MS5611_SPI_BITORDER, MS5611_SPI_MODE));
}

void MS5611_Barometer::spi_end_transaction() {
    SPI.endTransaction();
}

uint8_t MS5611_Barometer::spi_transfer(uint8_t data) {
    return SPI.transfer(data);
}

uint16_t MS5611_Barometer::spi_read_16bit() {
    uint16_t result = 0;
    result = (uint16_t)spi_transfer(0x00) << 8;
    result |= spi_transfer(0x00);
    return result;
}

uint32_t MS5611_Barometer::spi_read_24bit() {
    uint32_t result = 0;
    result = (uint32_t)spi_transfer(0x00) << 16;
    result |= (uint32_t)spi_transfer(0x00) << 8;
    result |= spi_transfer(0x00);
    return result;
}

void MS5611_Barometer::calculate_compensated_values(uint32_t D1, uint32_t D2) {
    // Calculate temperature difference
    dT = (int32_t)D2 - ((int32_t)calibration.C5 << 8);
    
    // Calculate offset and sensitivity
    OFF = ((int64_t)calibration.C2 << 16) + (((int64_t)calibration.C4 * dT) >> 7);
    SENS = ((int64_t)calibration.C1 << 15) + (((int64_t)calibration.C3 * dT) >> 8);
}

void MS5611_Barometer::apply_second_order_compensation(int32_t& temp, int64_t& off, int64_t& sens) {
    int64_t T2 = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;
    
    // Second order temperature compensation
    if (temp < 2000) {
        // Low temperature
        T2 = ((int64_t)dT * dT) >> 31;
        OFF2 = (5 * (temp - 2000) * (temp - 2000)) >> 1;
        SENS2 = (5 * (temp - 2000) * (temp - 2000)) >> 2;
        
        if (temp < -1500) {
            // Very low temperature
            OFF2 += 7 * (temp + 1500) * (temp + 1500);
            SENS2 += (11 * (temp + 1500) * (temp + 1500)) >> 1;
        }
    }
    
    // Apply compensation
    temp -= T2;
    off -= OFF2;
    sens -= SENS2;
}

uint8_t MS5611_Barometer::calculate_crc(uint16_t* prom_data) {
    uint8_t crc = 0;
    uint8_t crc_read = prom_data[7] & 0x0F;
    prom_data[7] &= 0xFF00;
    
    for (int i = 0; i < 16; i++) {
        if (i % 2 == 1) {
            crc ^= ((prom_data[i >> 1]) & 0x00FF);
        } else {
            crc ^= (prom_data[i >> 1] >> 8);
        }
        
        for (int j = 8; j > 0; j--) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
    }
    
    crc = (crc >> 4) & 0x0F;
    prom_data[7] |= crc_read;
    
    return crc;
}

bool MS5611_Barometer::validate_crc() {
    uint16_t prom_data[8] = {
        calibration.manufacturer,
        calibration.C1,
        calibration.C2,
        calibration.C3,
        calibration.C4,
        calibration.C5,
        calibration.C6,
        calibration.crc
    };
    
    uint8_t calculated_crc = calculate_crc(prom_data);
    uint8_t read_crc = calibration.crc & 0x0F;
    
    return calculated_crc == read_crc;
}

void MS5611_Barometer::increment_error_count() {
    error_count++;
}

uint32_t MS5611_Barometer::get_conversion_delay() {
    switch (osr_setting) {
        case MS5611_OSR_256:  return MS5611_DELAY_OSR_256;
        case MS5611_OSR_512:  return MS5611_DELAY_OSR_512;
        case MS5611_OSR_1024: return MS5611_DELAY_OSR_1024;
        case MS5611_OSR_2048: return MS5611_DELAY_OSR_2048;
        case MS5611_OSR_4096: return MS5611_DELAY_OSR_4096;
        default: return MS5611_DELAY_OSR_4096;
    }
}

// Utility functions
namespace MS5611_Utils {
    float standard_altitude(float pressure_hpa) {
        return 44330.0f * (1.0f - pow(pressure_hpa / 1013.25f, 0.1903f));
    }
    
    float standard_pressure(float altitude_m) {
        return 1013.25f * pow(1.0f - altitude_m / 44330.0f, 5.255f);
    }
    
    float temperature_compensated_altitude(float pressure_hpa, float temperature_c, float sea_level_hpa) {
        // More accurate altitude calculation with temperature compensation
        float temp_kelvin = temperature_c + 273.15f;
        float lapse_rate = 0.0065f; // K/m
        float gas_constant = 287.053f; // J/(kg·K)
        float gravity = 9.80665f; // m/s²
        
        return (temp_kelvin / lapse_rate) * (pow(sea_level_hpa / pressure_hpa, 
               (gas_constant * lapse_rate) / gravity) - 1.0f);
    }
    
    MS5611_Utils::PressureTrend analyze_pressure_trend(float* pressure_history, int history_length, float time_span_minutes) {
        if (history_length < 2) return TREND_STEADY;
        
        float total_change = pressure_history[history_length - 1] - pressure_history[0];
        float rate_per_hour = (total_change / time_span_minutes) * 60.0f;
        
        if (abs(rate_per_hour) < 0.5f) return TREND_STEADY;
        else if (rate_per_hour > 3.0f) return TREND_RAPID_RISE;
        else if (rate_per_hour < -3.0f) return TREND_RAPID_FALL;
        else if (rate_per_hour > 0.5f) return TREND_RISING;
        else return TREND_FALLING;
    }
}