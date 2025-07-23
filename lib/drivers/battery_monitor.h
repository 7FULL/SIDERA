#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

// Battery Voltage Monitoring Driver
// Monitors battery voltage through ADC with voltage divider
// Provides battery level estimation and low voltage warnings

// ADC Configuration
#define BATTERY_ADC_RESOLUTION    1024    // 10-bit ADC (Arduino Uno/Nano)
#define BATTERY_ADC_REFERENCE     3.3f    // ADC reference voltage (V)
#define BATTERY_ADC_MAX_VALUE     1023    // Maximum ADC reading

// Battery Configuration (3S Li-Po example)
#define BATTERY_CELLS             3       // Number of cells in series
#define BATTERY_CELL_NOMINAL      3.7f    // Nominal cell voltage (V)
#define BATTERY_CELL_MAX          4.2f    // Maximum cell voltage (V)
#define BATTERY_CELL_MIN          3.0f    // Minimum safe cell voltage (V)
#define BATTERY_CELL_CRITICAL     2.8f    // Critical cell voltage (V)

// Calculated battery voltages
#define BATTERY_VOLTAGE_NOMINAL   (BATTERY_CELLS * BATTERY_CELL_NOMINAL)  // 11.1V
#define BATTERY_VOLTAGE_MAX       (BATTERY_CELLS * BATTERY_CELL_MAX)      // 12.6V
#define BATTERY_VOLTAGE_MIN       (BATTERY_CELLS * BATTERY_CELL_MIN)      // 9.0V
#define BATTERY_VOLTAGE_CRITICAL  (BATTERY_CELLS * BATTERY_CELL_CRITICAL) // 8.4V

// Voltage divider configuration (adjust based on hardware)
#define VOLTAGE_DIVIDER_R1        10000.0f   // Upper resistor (ohms)
#define VOLTAGE_DIVIDER_R2        2200.0f    // Lower resistor (ohms)
#define VOLTAGE_DIVIDER_RATIO     ((VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2) / VOLTAGE_DIVIDER_R2)

// Battery level thresholds (percentage)
#define BATTERY_LEVEL_FULL        100
#define BATTERY_LEVEL_HIGH        75
#define BATTERY_LEVEL_MEDIUM      50
#define BATTERY_LEVEL_LOW         25
#define BATTERY_LEVEL_CRITICAL    10
#define BATTERY_LEVEL_EMPTY       0

// Monitoring intervals
#define BATTERY_UPDATE_INTERVAL   1000    // Update every 1 second
#define BATTERY_FAST_UPDATE       100     // Fast update when critical (100ms)

// Moving average filter size
#define BATTERY_FILTER_SAMPLES    10

// Error codes
#define BATTERY_SUCCESS           0
#define BATTERY_ERROR_INIT       -1
#define BATTERY_ERROR_ADC        -2
#define BATTERY_ERROR_CALIBRATION -3
#define BATTERY_ERROR_CRITICAL   -4

// Battery status flags
#define BATTERY_FLAG_NORMAL       0x00
#define BATTERY_FLAG_LOW          0x01
#define BATTERY_FLAG_CRITICAL     0x02
#define BATTERY_FLAG_CHARGING     0x04
#define BATTERY_FLAG_FULL         0x08
#define BATTERY_FLAG_ERROR        0x10

// Data structures
struct BatteryReading {
    float voltage;              // Battery voltage (V)
    float current;              // Battery current (A) - if current sensor available
    float power;                // Battery power (W)
    uint8_t percentage;         // Battery level percentage (0-100)
    uint32_t timestamp;         // Reading timestamp
    bool valid;                 // Reading validity
};

struct BatteryCalibration {
    float voltage_offset;       // Voltage calibration offset
    float voltage_scale;        // Voltage calibration scale factor
    float current_offset;       // Current sensor offset (if available)
    float current_scale;        // Current sensor scale factor
    bool calibrated;            // Calibration status
};

struct BatteryStatistics {
    float min_voltage;          // Minimum voltage recorded
    float max_voltage;          // Maximum voltage recorded
    float avg_voltage;          // Average voltage
    float total_energy;         // Total energy consumed (Wh)
    uint32_t runtime;           // Total runtime (seconds)
    uint32_t cycle_count;       // Battery cycle count estimate
    uint32_t low_voltage_events; // Number of low voltage events
};

// Battery health assessment
enum BatteryHealth {
    HEALTH_UNKNOWN,
    HEALTH_EXCELLENT,           // > 90% capacity
    HEALTH_GOOD,                // 70-90% capacity
    HEALTH_FAIR,                // 50-70% capacity
    HEALTH_POOR,                // 30-50% capacity
    HEALTH_CRITICAL            // < 30% capacity
};

// Battery state
enum BatteryState {
    STATE_UNKNOWN,
    STATE_CHARGING,
    STATE_DISCHARGING,
    STATE_FULL,
    STATE_EMPTY,
    STATE_FAULT
};

class BatteryMonitor {
private:
    uint8_t voltage_pin;
    uint8_t current_pin;        // Optional current sensor pin
    bool has_current_sensor;
    bool initialized;
    
    BatteryReading current_reading;
    BatteryCalibration calibration;
    BatteryStatistics stats;
    
    // Filtering
    float voltage_samples[BATTERY_FILTER_SAMPLES];
    float current_samples[BATTERY_FILTER_SAMPLES];
    uint8_t sample_index;
    bool samples_full;
    
    // Status tracking
    uint8_t battery_flags;
    BatteryHealth health_status;
    BatteryState battery_state;
    uint32_t last_update_time;
    uint32_t state_change_time;
    
    // Thresholds and configuration
    float low_voltage_threshold;
    float critical_voltage_threshold;
    float full_voltage_threshold;
    uint16_t update_interval;
    
    // Callbacks
    void (*low_voltage_callback)(float voltage);
    void (*critical_voltage_callback)(float voltage);
    void (*state_change_callback)(BatteryState old_state, BatteryState new_state);
    
    // Energy tracking
    float last_power_reading;
    uint32_t last_energy_update;
    
public:
    BatteryMonitor(uint8_t voltage_analog_pin, uint8_t current_analog_pin = 255);
    
    // Initialization
    int8_t begin();
    bool is_initialized() const { return initialized; }
    
    // Configuration
    void set_battery_type(uint8_t cells, float cell_nominal_v, float cell_max_v, float cell_min_v);
    void set_voltage_divider(float r1_ohms, float r2_ohms);
    void set_thresholds(float low_v, float critical_v, float full_v);
    void set_update_interval(uint16_t interval_ms);
    
    // Calibration
    int8_t calibrate_voltage(float known_voltage);
    int8_t calibrate_current(float known_current);
    void set_calibration(const BatteryCalibration& cal);
    const BatteryCalibration& get_calibration() const { return calibration; }
    bool is_calibrated() const { return calibration.calibrated; }
    
    // Reading functions
    void update();
    BatteryReading get_reading() const { return current_reading; }
    float get_voltage() const { return current_reading.voltage; }
    float get_current() const { return current_reading.current; }
    uint8_t get_percentage() const { return current_reading.percentage; }
    
    // Status functions
    BatteryHealth get_health() const { return health_status; }
    BatteryState get_state() const { return battery_state; }
    uint8_t get_status_flags() const { return battery_flags; }
    bool is_voltage_low() const { return (battery_flags & BATTERY_FLAG_LOW) != 0; }
    bool is_voltage_critical() const { return (battery_flags & BATTERY_FLAG_CRITICAL) != 0; }
    bool is_charging() const { return battery_state == STATE_CHARGING; }
    
    // Time estimates
    uint32_t estimate_runtime_remaining(float load_current_a);
    uint32_t estimate_charge_time_remaining(float charge_current_a);
    float estimate_capacity_remaining_ah();
    
    // Statistics
    const BatteryStatistics& get_statistics() const { return stats; }
    void reset_statistics();
    float get_efficiency() const;
    uint32_t get_runtime_seconds() const { return stats.runtime; }
    
    // Callbacks
    void set_low_voltage_callback(void (*callback)(float voltage));
    void set_critical_voltage_callback(void (*callback)(float voltage));
    void set_state_change_callback(void (*callback)(BatteryState old_state, BatteryState new_state));
    
    // Power management
    bool should_enter_low_power_mode() const;
    bool should_shutdown() const;
    void acknowledge_low_voltage_warning();
    
    // Data logging
    void log_to_serial() const;
    void format_telemetry_string(char* buffer, size_t buffer_size) const;
    
    // Advanced features
    float calculate_internal_resistance();
    float estimate_remaining_cycles();
    void perform_battery_health_test();
    
    // Diagnostic functions
    void print_battery_info() const;
    void print_statistics() const;
    void print_health_report() const;
    
    // Energy monitoring (if current sensor available)
    float get_instantaneous_power() const { return current_reading.power; }
    float get_total_energy_consumed() const { return stats.total_energy; }
    void reset_energy_counter();
    
    // Battery protection
    bool is_overvoltage() const;
    bool is_undervoltage() const;
    bool is_overcurrent(float max_current_a) const;
    
private:
    // ADC reading functions
    float read_voltage_raw();
    float read_current_raw();
    void apply_voltage_calibration(float& voltage);
    void apply_current_calibration(float& current);
    
    // Filtering functions
    void add_voltage_sample(float voltage);
    void add_current_sample(float current);
    float get_filtered_voltage();
    float get_filtered_current();
    
    // Conversion functions
    uint8_t voltage_to_percentage(float voltage);
    float percentage_to_voltage(uint8_t percentage);
    BatteryHealth assess_battery_health(float voltage, uint8_t percentage);
    BatteryState determine_battery_state(float voltage, float current, uint8_t percentage);
    
    // State management
    void update_battery_state();
    void update_statistics();
    void update_energy_tracking();
    void check_voltage_thresholds();
    void handle_state_change(BatteryState new_state);
    
    // Utility functions
    void initialize_samples();
    void trigger_callbacks();
    bool is_time_for_update();
};

// Global instance helper
extern BatteryMonitor* g_battery_monitor;

// Utility functions
namespace BatteryUtils {
    // Voltage-percentage conversion tables for different battery types
    struct VoltageCurve {
        float voltage[11];      // Voltage points (0%, 10%, 20%, ..., 100%)
        const char* name;
    };
    
    // Pre-defined voltage curves
    extern const VoltageCurve LIPO_3S_CURVE;
    extern const VoltageCurve LIPO_4S_CURVE;
    extern const VoltageCurve LIION_3S_CURVE;
    extern const VoltageCurve NIMH_10S_CURVE;
    
    // Conversion functions using lookup tables
    uint8_t voltage_to_percentage_curve(float voltage, const VoltageCurve& curve);
    float percentage_to_voltage_curve(uint8_t percentage, const VoltageCurve& curve);
    
    // Battery capacity estimation
    float estimate_battery_capacity_ah(float voltage_drop, float current_draw, uint32_t time_ms);
    float calculate_c_rate(float current_a, float capacity_ah);
    
    // Temperature compensation (if temperature sensor available)
    float temperature_compensate_voltage(float voltage, float temperature_c, float temp_coeff = -0.003f);
    
    // Battery safety checks
    bool is_voltage_in_safe_range(float voltage, uint8_t cells, float margin = 0.1f);
    bool is_discharge_rate_safe(float current_a, float capacity_ah, float max_c_rate = 3.0f);
    
    // Power calculations
    float calculate_power(float voltage, float current);
    float calculate_energy_wh(float power_w, uint32_t time_ms);
    float calculate_charge_ah(float current_a, uint32_t time_ms);
    
    // Battery life estimation
    uint32_t estimate_cycle_life(float depth_of_discharge, float avg_temperature_c);
    float calculate_depth_of_discharge(float min_voltage, float max_voltage, const VoltageCurve& curve);
    
    // Data formatting
    void format_voltage_string(float voltage, char* buffer, size_t buffer_size);
    void format_current_string(float current, char* buffer, size_t buffer_size);
    void format_power_string(float power, char* buffer, size_t buffer_size);
    void format_energy_string(float energy, char* buffer, size_t buffer_size);
    void format_time_string(uint32_t seconds, char* buffer, size_t buffer_size);
    
    // Health assessment helpers
    const char* health_to_string(BatteryHealth health);
    const char* state_to_string(BatteryState state);
    uint8_t calculate_health_score(float current_capacity, float original_capacity);
    
    // Calibration helpers
    float calculate_voltage_divider_ratio(float r1, float r2);
    float calculate_shunt_current(float voltage_drop, float shunt_resistance);
    void generate_calibration_points(float* voltages, float* adc_readings, uint8_t num_points);
    
    // Battery testing utilities
    struct BatteryTestResult {
        float internal_resistance;
        float actual_capacity;
        float voltage_sag;
        BatteryHealth health_assessment;
        bool test_passed;
    };
    
    BatteryTestResult perform_load_test(BatteryMonitor& monitor, float load_current, uint32_t duration_ms);
    bool perform_capacity_test(BatteryMonitor& monitor, float discharge_current);
}

#endif // BATTERY_MONITOR_H