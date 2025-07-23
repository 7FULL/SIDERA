#ifndef MS5611_BAROMETER_H
#define MS5611_BAROMETER_H

#include <Arduino.h>
#include <SPI.h>

// MS5611 (MS561101BA03-50) Barometric Pressure Sensor Driver
// High-resolution pressure and temperature sensor with 24-bit ADC

// SPI Communication Settings
#define MS5611_SPI_MODE SPI_MODE0
#define MS5611_SPI_SPEED 10000000  // 10MHz max SPI speed
#define MS5611_SPI_BITORDER MSBFIRST

// MS5611 Commands
#define MS5611_CMD_RESET       0x1E  // Reset device
#define MS5611_CMD_CONVERT_D1  0x40  // Convert D1 (pressure)
#define MS5611_CMD_CONVERT_D2  0x50  // Convert D2 (temperature)
#define MS5611_CMD_ADC_READ    0x00  // Read ADC result
#define MS5611_CMD_PROM_READ   0xA0  // Read PROM (calibration data)

// Oversampling Rate (OSR) Commands
#define MS5611_OSR_256   0x00  // Conversion time: 0.6ms
#define MS5611_OSR_512   0x02  // Conversion time: 1.17ms
#define MS5611_OSR_1024  0x04  // Conversion time: 2.28ms
#define MS5611_OSR_2048  0x06  // Conversion time: 4.54ms
#define MS5611_OSR_4096  0x08  // Conversion time: 9.04ms

// PROM addresses (calibration coefficients)
#define MS5611_PROM_MANUFACTURER 0x00
#define MS5611_PROM_C1          0x02  // Pressure sensitivity
#define MS5611_PROM_C2          0x04  // Pressure offset
#define MS5611_PROM_C3          0x06  // Temperature coefficient of pressure sensitivity
#define MS5611_PROM_C4          0x08  // Temperature coefficient of pressure offset
#define MS5611_PROM_C5          0x0A  // Reference temperature
#define MS5611_PROM_C6          0x0C  // Temperature coefficient of temperature
#define MS5611_PROM_CRC         0x0E  // CRC check

// Conversion delays (in milliseconds) for different OSR settings
#define MS5611_DELAY_OSR_256   1    // 0.6ms + margin
#define MS5611_DELAY_OSR_512   2    // 1.17ms + margin
#define MS5611_DELAY_OSR_1024  3    // 2.28ms + margin
#define MS5611_DELAY_OSR_2048  5    // 4.54ms + margin
#define MS5611_DELAY_OSR_4096  10   // 9.04ms + margin

// Error codes
#define MS5611_SUCCESS          0
#define MS5611_ERROR_INIT      -1
#define MS5611_ERROR_CRC       -2
#define MS5611_ERROR_TIMEOUT   -3
#define MS5611_ERROR_SPI       -4

// Default settings
#define MS5611_DEFAULT_OSR MS5611_OSR_4096  // Highest resolution
#define MS5611_SEA_LEVEL_PRESSURE 1013.25f // Standard sea level pressure (hPa)

// Calibration structure for PROM coefficients
struct MS5611_Calibration {
    uint16_t manufacturer;
    uint16_t C1;  // Pressure sensitivity (SENS_T1)
    uint16_t C2;  // Pressure offset (OFF_T1)
    uint16_t C3;  // Temperature coefficient of pressure sensitivity (TCS)
    uint16_t C4;  // Temperature coefficient of pressure offset (TCO)
    uint16_t C5;  // Reference temperature (T_REF)
    uint16_t C6;  // Temperature coefficient of temperature (TEMPSENS)
    uint16_t crc; // CRC checksum
};

// Measurement structure
struct MS5611_Data {
    float pressure;      // Pressure in hPa (mbar)
    float temperature;   // Temperature in °C
    float altitude;      // Calculated altitude in meters
    uint32_t raw_pressure;    // Raw pressure ADC value (D1)
    uint32_t raw_temperature; // Raw temperature ADC value (D2)
    uint32_t timestamp;  // Measurement timestamp
    bool valid;          // Data validity flag
};

class MS5611_Barometer {
private:
    uint8_t cs_pin;
    uint8_t osr_setting;
    uint32_t spi_speed;
    bool initialized;
    
    MS5611_Calibration calibration;
    MS5611_Data last_reading;
    
    // Internal calculation variables
    int32_t dT;          // Temperature difference
    int64_t OFF;         // Offset at actual temperature
    int64_t SENS;        // Sensitivity at actual temperature
    
    // Statistics
    uint32_t measurement_count;
    uint32_t error_count;
    float sea_level_pressure;
    
public:
    MS5611_Barometer(uint8_t chip_select_pin, uint8_t osr = MS5611_DEFAULT_OSR);
    
    // Initialization and configuration
    int8_t begin();
    int8_t reset();
    bool is_initialized() const { return initialized; }
    void set_oversampling(uint8_t osr);
    void set_sea_level_pressure(float pressure_hpa);
    
    // Calibration data management
    int8_t read_calibration();
    bool validate_calibration();
    const MS5611_Calibration& get_calibration() const { return calibration; }
    void print_calibration();
    
    // Measurement functions
    int8_t read_pressure_temperature(float& pressure, float& temperature);
    int8_t read_pressure_temperature_altitude(float& pressure, float& temperature, float& altitude);
    const MS5611_Data& get_last_reading() const { return last_reading; }
    
    // Raw ADC reading functions
    int8_t start_pressure_conversion();
    int8_t start_temperature_conversion();
    uint32_t read_adc();
    bool is_conversion_ready();
    
    // Altitude calculation functions
    float pressure_to_altitude(float pressure_hpa, float sea_level_hpa = MS5611_SEA_LEVEL_PRESSURE);
    float altitude_to_pressure(float altitude_m, float sea_level_hpa = MS5611_SEA_LEVEL_PRESSURE);
    
    // Utility functions
    float calculate_sea_level_pressure(float pressure_hpa, float altitude_m);
    void perform_self_test();
    
    // Statistics and diagnostics
    uint32_t get_measurement_count() const { return measurement_count; }
    uint32_t get_error_count() const { return error_count; }
    float get_success_rate() const;
    void reset_statistics();
    
    // Advanced features
    int8_t read_temperature_only(float& temperature);
    int8_t read_pressure_only(float& pressure);
    void enable_continuous_mode();
    void disable_continuous_mode();
    
private:
    // Low-level SPI communication
    void spi_begin_transaction();
    void spi_end_transaction();
    uint8_t spi_transfer(uint8_t data);
    void spi_write_command(uint8_t command);
    uint16_t spi_read_16bit();
    uint32_t spi_read_24bit();
    
    // Internal calculation functions
    void calculate_compensated_values(uint32_t D1, uint32_t D2);
    void apply_second_order_compensation(int32_t& temp, int64_t& off, int64_t& sens);
    
    // CRC validation
    uint8_t calculate_crc(uint16_t* prom_data);
    bool validate_crc();
    
    // Error handling
    void increment_error_count();
    uint32_t get_conversion_delay();
};

// Global instance helper (optional)
extern MS5611_Barometer* g_barometer;

// Utility functions for common operations
namespace MS5611_Utils {
    // Standard atmosphere calculations
    float standard_altitude(float pressure_hpa);
    float standard_pressure(float altitude_m);
    
    // Temperature compensation
    float temperature_compensated_altitude(float pressure_hpa, float temperature_c, float sea_level_hpa = MS5611_SEA_LEVEL_PRESSURE);
    
    // Pressure trend analysis
    enum PressureTrend {
        TREND_STEADY,
        TREND_RISING,
        TREND_FALLING,
        TREND_RAPID_RISE,
        TREND_RAPID_FALL
    };
    
    PressureTrend analyze_pressure_trend(float* pressure_history, int history_length, float time_span_minutes);
}

#endif // MS5611_BAROMETER_H