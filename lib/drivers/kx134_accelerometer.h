#ifndef KX134_ACCELEROMETER_H
#define KX134_ACCELEROMETER_H

#include <Arduino.h>
#include <SPI.h>

// KX134-1211 High-G Digital Accelerometer Driver
// ±8g, ±16g, ±32g, ±64g ranges with 16-bit resolution
// SPI interface, up to 25.6kHz ODR

// SPI Communication Settings
#define KX134_SPI_MODE SPI_MODE0
#define KX134_SPI_SPEED 10000000  // 10MHz max SPI speed
#define KX134_SPI_BITORDER MSBFIRST

// Register addresses
#define KX134_WHO_AM_I        0x0F
#define KX134_COTR            0x12
#define KX134_TSCP            0x10
#define KX134_TSPP            0x11
#define KX134_INS1            0x16
#define KX134_INS2            0x17
#define KX134_INS3            0x18
#define KX134_STATUS_REG      0x19
#define KX134_INT_REL         0x1A
#define KX134_CNTL1           0x1B
#define KX134_CNTL2           0x1C
#define KX134_CNTL3           0x1D
#define KX134_CNTL4           0x1E
#define KX134_CNTL5           0x1F
#define KX134_CNTL6           0x20
#define KX134_ODCNTL          0x21
#define KX134_INC1            0x22
#define KX134_INC2            0x23
#define KX134_INC3            0x24
#define KX134_INC4            0x25
#define KX134_INC5            0x26
#define KX134_INC6            0x27
#define KX134_TILT_TIMER      0x29
#define KX134_TDTRC           0x2A
#define KX134_TDTC            0x2B
#define KX134_TTH             0x2C
#define KX134_TTL             0x2D
#define KX134_FTD             0x2E
#define KX134_STD             0x2F
#define KX134_TLT             0x30
#define KX134_TWS             0x31
#define KX134_FFTH            0x32
#define KX134_FFC             0x33
#define KX134_FFCNTL          0x34
#define KX134_TILT_ANGLE_LL   0x37
#define KX134_TILT_ANGLE_HL   0x38
#define KX134_HYST_SET        0x39
#define KX134_LP_CNTL1        0x3A
#define KX134_LP_CNTL2        0x3B
#define KX134_WUFTH           0x40
#define KX134_BTSWUFTH        0x41
#define KX134_BTSTH           0x42
#define KX134_BTSC            0x43
#define KX134_WUFC            0x44
#define KX134_SELF_TEST       0x5D
#define KX134_BUF_CNTL1       0x5E
#define KX134_BUF_CNTL2       0x5F
#define KX134_BUF_STATUS_1    0x60
#define KX134_BUF_STATUS_2    0x61
#define KX134_BUF_CLEAR       0x62
#define KX134_BUF_READ        0x63
#define KX134_ADP_CNTL1       0x64
#define KX134_ADP_CNTL2       0x65
#define KX134_ADP_CNTL3       0x66
#define KX134_ADP_CNTL4       0x67
#define KX134_ADP_CNTL5       0x68
#define KX134_ADP_CNTL6       0x69
#define KX134_ADP_CNTL7       0x6A
#define KX134_ADP_CNTL8       0x6B
#define KX134_ADP_CNTL9       0x6C
#define KX134_ADP_CNTL10      0x6D
#define KX134_ADP_CNTL11      0x6E
#define KX134_ADP_CNTL12      0x6F
#define KX134_ADP_CNTL13      0x70
#define KX134_ADP_CNTL14      0x71
#define KX134_ADP_CNTL15      0x72
#define KX134_ADP_CNTL16      0x73
#define KX134_ADP_CNTL17      0x74
#define KX134_ADP_CNTL18      0x75
#define KX134_ADP_CNTL19      0x76

// Data output registers
#define KX134_XOUT_L          0x08
#define KX134_XOUT_H          0x09
#define KX134_YOUT_L          0x0A
#define KX134_YOUT_H          0x0B
#define KX134_ZOUT_L          0x0C
#define KX134_ZOUT_H          0x0D
#define KX134_TEMP_OUT_L      0x0E
#define KX134_TEMP_OUT_H      0x11

// WHO_AM_I ID
#define KX134_WHO_AM_I_VAL    0x46

// Control register bits
// CNTL1 bits
#define KX134_CNTL1_PC1       0x80  // Operating mode control
#define KX134_CNTL1_RES       0x40  // Resolution selection
#define KX134_CNTL1_DRDYE     0x20  // Data ready engine enable
#define KX134_CNTL1_GSEL1     0x10  // G-range select bit 1
#define KX134_CNTL1_GSEL0     0x08  // G-range select bit 0
#define KX134_CNTL1_TDTE      0x04  // Tap/Double-tap engine enable
#define KX134_CNTL1_WUFE      0x02  // Wake-up function enable
#define KX134_CNTL1_TPE       0x01  // Tilt position engine enable

// CNTL2 bits
#define KX134_CNTL2_SRST      0x80  // Software reset
#define KX134_CNTL2_COTC      0x40  // Command test control
#define KX134_CNTL2_LEM       0x20  // Left/right state mask
#define KX134_CNTL2_RIM       0x10  // Right/left state mask
#define KX134_CNTL2_DOM       0x08  // Down/up state mask
#define KX134_CNTL2_UPM       0x04  // Up/down state mask
#define KX134_CNTL2_FDM       0x02  // Face down state mask
#define KX134_CNTL2_FUM       0x01  // Face up state mask

// CNTL3 bits
#define KX134_CNTL3_OTP1      0x80  // Tap ODR bit 1
#define KX134_CNTL3_OTP0      0x40  // Tap ODR bit 0
#define KX134_CNTL3_OTDT1     0x20  // Double-tap ODR bit 1
#define KX134_CNTL3_OTDT0     0x10  // Double-tap ODR bit 0
#define KX134_CNTL3_OWUF1     0x08  // Wake-up ODR bit 1
#define KX134_CNTL3_OWUF0     0x04  // Wake-up ODR bit 0
#define KX134_CNTL3_OTPE      0x02  // Tilt position ODR enable
#define KX134_CNTL3_DCST      0x01  // Directional count state

// G-range selections
#define KX134_RANGE_8G        0x00  // ±8g
#define KX134_RANGE_16G       0x08  // ±16g
#define KX134_RANGE_32G       0x10  // ±32g
#define KX134_RANGE_64G       0x18  // ±64g

// Output Data Rate settings (ODCNTL register)
#define KX134_ODR_0_781HZ     0x00  // 0.781 Hz
#define KX134_ODR_1_563HZ     0x01  // 1.563 Hz
#define KX134_ODR_3_125HZ     0x02  // 3.125 Hz
#define KX134_ODR_6_25HZ      0x03  // 6.25 Hz
#define KX134_ODR_12_5HZ      0x04  // 12.5 Hz
#define KX134_ODR_25HZ        0x05  // 25 Hz
#define KX134_ODR_50HZ        0x06  // 50 Hz
#define KX134_ODR_100HZ       0x07  // 100 Hz
#define KX134_ODR_200HZ       0x08  // 200 Hz
#define KX134_ODR_400HZ       0x09  // 400 Hz
#define KX134_ODR_800HZ       0x0A  // 800 Hz
#define KX134_ODR_1600HZ      0x0B  // 1600 Hz
#define KX134_ODR_3200HZ      0x0C  // 3200 Hz
#define KX134_ODR_6400HZ      0x0D  // 6400 Hz
#define KX134_ODR_12800HZ     0x0E  // 12800 Hz
#define KX134_ODR_25600HZ     0x0F  // 25600 Hz

// Resolution modes
#define KX134_RES_8BIT        0x00  // 8-bit mode
#define KX134_RES_16BIT       0x40  // 16-bit mode (high resolution)

// Error codes
#define KX134_SUCCESS         0
#define KX134_ERROR_INIT     -1
#define KX134_ERROR_SPI      -2
#define KX134_ERROR_ID       -3
#define KX134_ERROR_RANGE    -4
#define KX134_ERROR_ODR      -5

// Data structures
struct KX134_Vector {
    float x, y, z;
};

struct KX134_RawData {
    int16_t x, y, z;
    int16_t temperature;
};

struct KX134_SensorData {
    KX134_Vector acceleration;  // Acceleration in g
    float temperature;          // Temperature in °C
    uint32_t timestamp;         // Timestamp in milliseconds
    bool valid;                 // Data validity flag
};

struct KX134_Config {
    uint8_t range;              // G-range setting
    uint8_t odr;                // Output data rate
    uint8_t resolution;         // Resolution mode
    bool high_performance;      // High performance mode
    bool enable_buffer;         // Enable FIFO buffer
    uint16_t buffer_watermark;  // Buffer watermark level
};

class KX134_Accelerometer {
private:
    uint8_t cs_pin;
    uint32_t spi_speed;
    bool initialized;
    
    KX134_Config config;
    KX134_SensorData last_reading;
    
    // Calibration data
    KX134_Vector offset;        // Zero-G offset
    KX134_Vector scale;         // Scale factors
    bool calibrated;
    
    // Statistics
    uint32_t measurement_count;
    uint32_t error_count;
    
    // Scale factors for different ranges (LSB/g)
    float scale_factor;
    
public:
    KX134_Accelerometer(uint8_t chip_select_pin);
    
    // Initialization and configuration
    int8_t begin(uint8_t range = KX134_RANGE_64G, uint8_t odr = KX134_ODR_1600HZ);
    int8_t reset();
    bool is_initialized() const { return initialized; }
    
    // Configuration functions
    int8_t set_range(uint8_t range);
    int8_t set_output_data_rate(uint8_t odr);
    int8_t set_resolution(uint8_t resolution);
    int8_t set_power_mode(bool high_performance);
    
    // Data reading functions
    int8_t read_acceleration(KX134_Vector& accel);
    int8_t read_raw_data(KX134_RawData& raw_data);
    const KX134_SensorData& get_last_reading() const { return last_reading; }
    
    // Temperature reading
    float read_temperature();
    
    // Calibration functions
    int8_t calibrate_zero_g(uint16_t samples = 1000);
    int8_t set_calibration_data(const KX134_Vector& offset, const KX134_Vector& scale);
    void get_calibration_data(KX134_Vector& offset, KX134_Vector& scale);
    bool is_calibrated() const { return calibrated; }
    
    // FIFO buffer functions
    int8_t enable_buffer(uint16_t watermark = 512);
    int8_t disable_buffer();
    uint16_t get_buffer_level();
    int8_t read_buffer(KX134_SensorData* data, uint16_t max_samples);
    int8_t clear_buffer();
    
    // Interrupt configuration
    int8_t configure_data_ready_interrupt(uint8_t pin);
    int8_t configure_high_g_interrupt(float threshold_g, uint8_t duration_ms);
    int8_t disable_interrupts();
    
    // Self-test functions
    int8_t perform_self_test();
    bool verify_communication();
    
    // Utility functions
    float get_range_g() const;
    float get_max_odr_hz() const;
    uint32_t get_measurement_count() const { return measurement_count; }
    uint32_t get_error_count() const { return error_count; }
    float get_success_rate() const;
    void reset_statistics();
    
    // Advanced features
    int8_t enable_wake_up_detection(float threshold_g);
    int8_t enable_tap_detection();
    int8_t enable_free_fall_detection(float threshold_g, uint16_t time_ms);
    
    // Data conversion utilities
    float raw_to_g(int16_t raw_value) const;
    int16_t g_to_raw(float g_value) const;
    
    // Diagnostic functions
    void print_registers();
    void print_configuration();
    void print_sensor_info();
    
private:
    // Low-level SPI functions
    void spi_begin_transaction();
    void spi_end_transaction();
    uint8_t spi_transfer(uint8_t data);
    int8_t write_register(uint8_t reg, uint8_t value);
    int8_t read_register(uint8_t reg, uint8_t& value);
    int8_t read_registers(uint8_t reg, uint8_t* buffer, uint8_t length);
    
    // Internal utility functions
    void update_scale_factor();
    int8_t enter_standby_mode();
    int8_t enter_operating_mode();
    bool check_who_am_i();
    void increment_error_count();
    
    // Data processing functions
    void apply_calibration(KX134_Vector& accel);
    void convert_raw_to_g(const KX134_RawData& raw, KX134_Vector& accel);
    
    // Configuration helpers
    uint8_t odr_to_register_value(uint8_t odr);
    uint8_t range_to_register_value(uint8_t range);
    float range_to_scale_factor(uint8_t range);
};

// Global instance helper
extern KX134_Accelerometer* g_high_g_accel;

// Utility functions
namespace KX134_Utils {
    // G-force calculations
    float calculate_total_g_force(const KX134_Vector& accel);
    float calculate_vertical_g(const KX134_Vector& accel, const KX134_Vector& gravity_ref);
    
    // Peak detection
    struct PeakData {
        float max_g;
        float min_g;
        uint32_t max_timestamp;
        uint32_t min_timestamp;
        float duration_ms;
    };
    
    PeakData find_acceleration_peak(KX134_SensorData* data, uint16_t length);
    
    // Statistical analysis
    void calculate_statistics(KX134_SensorData* data, uint16_t length, 
                             KX134_Vector& mean, KX134_Vector& std_dev, 
                             KX134_Vector& min_val, KX134_Vector& max_val);
    
    // Shock detection
    bool detect_shock_event(const KX134_Vector& accel, float threshold_g = 50.0f);
    bool detect_launch_signature(KX134_SensorData* data, uint16_t length, float threshold_g = 20.0f);
    
    // Data filtering
    KX134_Vector apply_low_pass_filter(const KX134_Vector& current, const KX134_Vector& previous, float alpha = 0.1f);
    KX134_Vector apply_high_pass_filter(const KX134_Vector& current, const KX134_Vector& previous, float alpha = 0.9f);
    
    // Coordinate transformations
    KX134_Vector rotate_to_body_frame(const KX134_Vector& sensor_data, float roll, float pitch, float yaw);
    KX134_Vector align_with_rocket_axis(const KX134_Vector& accel, uint8_t mounting_orientation);
    
    // Flight analysis
    enum FlightPhase {
        PHASE_GROUND,
        PHASE_BOOST,
        PHASE_COAST,
        PHASE_DESCENT,
        PHASE_LANDED
    };
    
    FlightPhase analyze_flight_phase(const KX134_Vector& accel, float total_g);
    
    // Calibration utilities
    bool is_sensor_level(const KX134_Vector& accel, float tolerance = 0.1f);
    KX134_Vector calculate_gravity_reference(KX134_SensorData* data, uint16_t length);
    
    // Data export/import
    void export_calibration_data(const KX134_Vector& offset, const KX134_Vector& scale, const char* filename);
    bool import_calibration_data(KX134_Vector& offset, KX134_Vector& scale, const char* filename);
}

#endif // KX134_ACCELEROMETER_H