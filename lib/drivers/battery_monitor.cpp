#include "battery_monitor.h"
#include <math.h>

BatteryMonitor* g_battery_monitor = nullptr;

// Pre-defined voltage curves for different battery types
namespace BatteryUtils {
    const VoltageCurve LIPO_3S_CURVE = {
        {9.0f, 9.6f, 10.2f, 10.5f, 10.8f, 11.1f, 11.4f, 11.7f, 12.0f, 12.3f, 12.6f},
        "Li-Po 3S"
    };
    
    const VoltageCurve LIPO_4S_CURVE = {
        {12.0f, 12.8f, 13.6f, 14.0f, 14.4f, 14.8f, 15.2f, 15.6f, 16.0f, 16.4f, 16.8f},
        "Li-Po 4S"
    };
    
    const VoltageCurve LIION_3S_CURVE = {
        {9.0f, 9.9f, 10.5f, 10.8f, 11.1f, 11.4f, 11.7f, 12.0f, 12.3f, 12.6f, 12.6f},
        "Li-Ion 3S"
    };
    
    const VoltageCurve NIMH_10S_CURVE = {
        {10.0f, 10.5f, 11.0f, 11.5f, 12.0f, 12.5f, 13.0f, 13.5f, 14.0f, 14.5f, 15.0f},
        "NiMH 10S"
    };
}

BatteryMonitor::BatteryMonitor(uint8_t voltage_analog_pin, uint8_t current_analog_pin) {
    voltage_pin = voltage_analog_pin;
    current_pin = current_analog_pin;
    has_current_sensor = (current_analog_pin != 255);
    initialized = false;
    
    // Initialize structures
    memset(&current_reading, 0, sizeof(BatteryReading));
    memset(&calibration, 0, sizeof(BatteryCalibration));
    memset(&stats, 0, sizeof(BatteryStatistics));
    
    // Initialize voltage samples array
    memset(voltage_samples, 0, sizeof(voltage_samples));
    memset(current_samples, 0, sizeof(current_samples));
    sample_index = 0;
    samples_full = false;
    
    // Initialize status
    battery_flags = BATTERY_FLAG_NORMAL;
    health_status = HEALTH_UNKNOWN;
    battery_state = STATE_UNKNOWN;
    last_update_time = 0;
    state_change_time = 0;
    
    // Set default thresholds
    low_voltage_threshold = BATTERY_VOLTAGE_MIN;
    critical_voltage_threshold = BATTERY_VOLTAGE_CRITICAL;
    full_voltage_threshold = BATTERY_VOLTAGE_MAX;
    update_interval = BATTERY_UPDATE_INTERVAL;
    
    // Initialize callbacks to null
    low_voltage_callback = nullptr;
    critical_voltage_callback = nullptr;
    state_change_callback = nullptr;
    
    // Initialize energy tracking
    last_power_reading = 0.0f;
    last_energy_update = 0;
    
    // Initialize calibration with default values
    calibration.voltage_offset = 0.0f;
    calibration.voltage_scale = 1.0f;
    calibration.current_offset = 0.0f;
    calibration.current_scale = 1.0f;
    calibration.calibrated = false;
    
    // Initialize statistics
    stats.min_voltage = BATTERY_VOLTAGE_MAX;
    stats.max_voltage = 0.0f;
    stats.avg_voltage = 0.0f;
    stats.total_energy = 0.0f;
    stats.runtime = 0;
    stats.cycle_count = 0;
    stats.low_voltage_events = 0;
}

int8_t BatteryMonitor::begin() {
    // Configure ADC pins
    pinMode(voltage_pin, INPUT);
    if (has_current_sensor) {
        pinMode(current_pin, INPUT);
    }
    
    // Initialize sample arrays
    initialize_samples();
    
    // Take initial readings
    for (int i = 0; i < BATTERY_FILTER_SAMPLES; i++) {
        add_voltage_sample(read_voltage_raw());
        if (has_current_sensor) {
            add_current_sample(read_current_raw());
        }
        delay(10);
    }
    
    // Initialize readings
    current_reading.voltage = get_filtered_voltage();
    current_reading.current = has_current_sensor ? get_filtered_current() : 0.0f;
    current_reading.power = calculate_power(current_reading.voltage, current_reading.current);
    current_reading.percentage = voltage_to_percentage(current_reading.voltage);
    current_reading.timestamp = millis();
    current_reading.valid = true;
    
    // Initialize statistics
    stats.min_voltage = current_reading.voltage;
    stats.max_voltage = current_reading.voltage;
    stats.avg_voltage = current_reading.voltage;
    
    // Determine initial battery state
    update_battery_state();
    
    initialized = true;
    last_update_time = millis();
    
    return BATTERY_SUCCESS;
}

void BatteryMonitor::set_battery_type(uint8_t cells, float cell_nominal_v, float cell_max_v, float cell_min_v) {
    // Update thresholds based on battery configuration
    full_voltage_threshold = cells * cell_max_v;
    low_voltage_threshold = cells * cell_min_v;
    critical_voltage_threshold = cells * (cell_min_v - 0.2f); // 0.2V margin below minimum
}

void BatteryMonitor::set_voltage_divider(float r1_ohms, float r2_ohms) {
    // Calculate and store the voltage divider ratio
    // This affects how we convert ADC readings to actual voltage
    float ratio = (r1_ohms + r2_ohms) / r2_ohms;
    calibration.voltage_scale = ratio;
}

void BatteryMonitor::set_thresholds(float low_v, float critical_v, float full_v) {
    low_voltage_threshold = low_v;
    critical_voltage_threshold = critical_v;
    full_voltage_threshold = full_v;
}

void BatteryMonitor::set_update_interval(uint16_t interval_ms) {
    update_interval = interval_ms;
}

int8_t BatteryMonitor::calibrate_voltage(float known_voltage) {
    if (!initialized) return BATTERY_ERROR_INIT;
    
    float raw_voltage = read_voltage_raw();
    if (raw_voltage <= 0.0f) return BATTERY_ERROR_ADC;
    
    // Calculate calibration scale factor
    calibration.voltage_scale = known_voltage / raw_voltage;
    calibration.voltage_offset = 0.0f; // Reset offset when recalibrating
    calibration.calibrated = true;
    
    return BATTERY_SUCCESS;
}

int8_t BatteryMonitor::calibrate_current(float known_current) {
    if (!initialized || !has_current_sensor) return BATTERY_ERROR_INIT;
    
    float raw_current = read_current_raw();
    
    // Calculate calibration parameters
    calibration.current_scale = (known_current != 0.0f) ? (known_current / raw_current) : 1.0f;
    calibration.current_offset = known_current - (raw_current * calibration.current_scale);
    
    return BATTERY_SUCCESS;
}

void BatteryMonitor::set_calibration(const BatteryCalibration& cal) {
    calibration = cal;
}

void BatteryMonitor::update() {
    if (!initialized || !is_time_for_update()) {
        return;
    }
    
    // Read raw voltages and currents
    float raw_voltage = read_voltage_raw();
    float raw_current = has_current_sensor ? read_current_raw() : 0.0f;
    
    // Apply calibration
    apply_voltage_calibration(raw_voltage);
    apply_current_calibration(raw_current);
    
    // Add to filter
    add_voltage_sample(raw_voltage);
    if (has_current_sensor) {
        add_current_sample(raw_current);
    }
    
    // Get filtered values
    current_reading.voltage = get_filtered_voltage();
    current_reading.current = get_filtered_current();
    current_reading.power = calculate_power(current_reading.voltage, current_reading.current);
    current_reading.percentage = voltage_to_percentage(current_reading.voltage);
    current_reading.timestamp = millis();
    current_reading.valid = true;
    
    // Update battery state and statistics
    update_battery_state();
    update_statistics();
    update_energy_tracking();
    check_voltage_thresholds();
    
    last_update_time = millis();
}

uint32_t BatteryMonitor::estimate_runtime_remaining(float load_current_a) {
    if (!current_reading.valid || load_current_a <= 0.0f) return 0;
    
    // Simple estimation based on current voltage and load
    float remaining_capacity_ah = estimate_capacity_remaining_ah();
    if (remaining_capacity_ah <= 0.0f) return 0;
    
    return (uint32_t)((remaining_capacity_ah / load_current_a) * 3600.0f); // Convert to seconds
}

uint32_t BatteryMonitor::estimate_charge_time_remaining(float charge_current_a) {
    if (!current_reading.valid || charge_current_a <= 0.0f) return 0;
    
    float current_percentage = current_reading.percentage;
    if (current_percentage >= 100.0f) return 0;
    
    // Estimate based on remaining capacity needed
    float remaining_percentage = 100.0f - current_percentage;
    float nominal_capacity = 2.2f; // Assume 2.2Ah for 3S Li-Po (adjust as needed)
    float remaining_capacity_ah = (remaining_percentage / 100.0f) * nominal_capacity;
    
    return (uint32_t)((remaining_capacity_ah / charge_current_a) * 3600.0f); // Convert to seconds
}

float BatteryMonitor::estimate_capacity_remaining_ah() {
    if (!current_reading.valid) return 0.0f;
    
    float percentage = current_reading.percentage / 100.0f;
    float nominal_capacity = 2.2f; // Assume 2.2Ah nominal capacity (adjust as needed)
    
    return percentage * nominal_capacity;
}

void BatteryMonitor::reset_statistics() {
    stats.min_voltage = current_reading.voltage;
    stats.max_voltage = current_reading.voltage;
    stats.avg_voltage = current_reading.voltage;
    stats.total_energy = 0.0f;
    stats.runtime = 0;
    stats.cycle_count = 0;
    stats.low_voltage_events = 0;
}

float BatteryMonitor::get_efficiency() const {
    if (stats.runtime == 0 || stats.total_energy == 0.0f) return 0.0f;
    
    // Simple efficiency calculation (placeholder)
    float theoretical_energy = stats.avg_voltage * 2.2f * (stats.runtime / 3600.0f); // Wh
    return (theoretical_energy > 0.0f) ? (stats.total_energy / theoretical_energy) * 100.0f : 0.0f;
}

void BatteryMonitor::set_low_voltage_callback(void (*callback)(float voltage)) {
    low_voltage_callback = callback;
}

void BatteryMonitor::set_critical_voltage_callback(void (*callback)(float voltage)) {
    critical_voltage_callback = callback;
}

void BatteryMonitor::set_state_change_callback(void (*callback)(BatteryState old_state, BatteryState new_state)) {
    state_change_callback = callback;
}

bool BatteryMonitor::should_enter_low_power_mode() const {
    return (battery_flags & BATTERY_FLAG_LOW) != 0;
}

bool BatteryMonitor::should_shutdown() const {
    return (battery_flags & BATTERY_FLAG_CRITICAL) != 0;
}

void BatteryMonitor::acknowledge_low_voltage_warning() {
    battery_flags &= ~BATTERY_FLAG_LOW;
}

void BatteryMonitor::log_to_serial() const {
    Serial.println("=== Battery Monitor Status ===");
    Serial.print("Voltage: "); Serial.print(current_reading.voltage, 2); Serial.println(" V");
    Serial.print("Current: "); Serial.print(current_reading.current, 3); Serial.println(" A");
    Serial.print("Power: "); Serial.print(current_reading.power, 2); Serial.println(" W");
    Serial.print("Percentage: "); Serial.print(current_reading.percentage); Serial.println(" %");
    Serial.print("State: "); Serial.println(BatteryUtils::state_to_string(battery_state));
    Serial.print("Health: "); Serial.println(BatteryUtils::health_to_string(health_status));
    Serial.print("Runtime: "); Serial.print(stats.runtime); Serial.println(" s");
    Serial.print("Energy Used: "); Serial.print(stats.total_energy, 3); Serial.println(" Wh");
}

void BatteryMonitor::format_telemetry_string(char* buffer, size_t buffer_size) const {
    snprintf(buffer, buffer_size, "BAT:%.2fV,%.0f%%,%.2fW,%s", 
             current_reading.voltage, 
             current_reading.percentage, 
             current_reading.power,
             BatteryUtils::state_to_string(battery_state));
}

float BatteryMonitor::calculate_internal_resistance() {
    // Simplified internal resistance calculation
    // Would need load testing for accurate measurement
    if (current_reading.current <= 0.1f) return 0.0f;
    
    float no_load_voltage = full_voltage_threshold;
    float voltage_drop = no_load_voltage - current_reading.voltage;
    
    return voltage_drop / current_reading.current;
}

float BatteryMonitor::estimate_remaining_cycles() {
    // Estimate remaining cycles based on current health
    float health_factor = (float)health_status / (float)HEALTH_EXCELLENT;
    float typical_cycles = 500.0f; // Typical Li-Po cycle life
    
    return typical_cycles * health_factor - stats.cycle_count;
}

void BatteryMonitor::perform_battery_health_test() {
    // Simple health assessment based on voltage behavior
    float voltage_range = stats.max_voltage - stats.min_voltage;
    float expected_range = full_voltage_threshold - critical_voltage_threshold;
    
    if (voltage_range < expected_range * 0.3f) {
        health_status = HEALTH_CRITICAL;
    } else if (voltage_range < expected_range * 0.5f) {
        health_status = HEALTH_POOR;
    } else if (voltage_range < expected_range * 0.7f) {
        health_status = HEALTH_FAIR;
    } else if (voltage_range < expected_range * 0.9f) {
        health_status = HEALTH_GOOD;
    } else {
        health_status = HEALTH_EXCELLENT;
    }
}

void BatteryMonitor::print_battery_info() const {
    Serial.println("=== Battery Configuration ===");
    Serial.print("Voltage Pin: "); Serial.println(voltage_pin);
    Serial.print("Current Pin: "); Serial.println(has_current_sensor ? current_pin : 255);
    Serial.print("Low Threshold: "); Serial.print(low_voltage_threshold, 2); Serial.println(" V");
    Serial.print("Critical Threshold: "); Serial.print(critical_voltage_threshold, 2); Serial.println(" V");
    Serial.print("Full Threshold: "); Serial.print(full_voltage_threshold, 2); Serial.println(" V");
    Serial.print("Update Interval: "); Serial.print(update_interval); Serial.println(" ms");
    Serial.print("Calibrated: "); Serial.println(calibration.calibrated ? "Yes" : "No");
}

void BatteryMonitor::print_statistics() const {
    Serial.println("=== Battery Statistics ===");
    Serial.print("Min Voltage: "); Serial.print(stats.min_voltage, 2); Serial.println(" V");
    Serial.print("Max Voltage: "); Serial.print(stats.max_voltage, 2); Serial.println(" V");
    Serial.print("Avg Voltage: "); Serial.print(stats.avg_voltage, 2); Serial.println(" V");
    Serial.print("Runtime: "); Serial.print(stats.runtime); Serial.println(" s");
    Serial.print("Energy Used: "); Serial.print(stats.total_energy, 3); Serial.println(" Wh");
    Serial.print("Cycle Count: "); Serial.println(stats.cycle_count);
    Serial.print("Low Voltage Events: "); Serial.println(stats.low_voltage_events);
}

void BatteryMonitor::print_health_report() const {
    Serial.println("=== Battery Health Report ===");
    Serial.print("Health Status: "); Serial.println(BatteryUtils::health_to_string(health_status));
    Serial.print("Internal Resistance: "); Serial.print(calculate_internal_resistance(), 3); Serial.println(" Ω");
    Serial.print("Estimated Remaining Cycles: "); Serial.println(estimate_remaining_cycles());
    Serial.print("Efficiency: "); Serial.print(get_efficiency(), 1); Serial.println(" %");
}

void BatteryMonitor::reset_energy_counter() {
    stats.total_energy = 0.0f;
    last_energy_update = millis();
}

bool BatteryMonitor::is_overvoltage() const {
    return current_reading.voltage > (full_voltage_threshold * 1.1f);
}

bool BatteryMonitor::is_undervoltage() const {
    return current_reading.voltage < critical_voltage_threshold;
}

bool BatteryMonitor::is_overcurrent(float max_current_a) const {
    return has_current_sensor && (current_reading.current > max_current_a);
}

// Private functions
float BatteryMonitor::read_voltage_raw() {
    int adc_reading = analogRead(voltage_pin);
    float voltage = (adc_reading / (float)BATTERY_ADC_MAX_VALUE) * BATTERY_ADC_REFERENCE;
    return voltage * VOLTAGE_DIVIDER_RATIO;
}

float BatteryMonitor::read_current_raw() {
    if (!has_current_sensor) return 0.0f;
    
    int adc_reading = analogRead(current_pin);
    float voltage = (adc_reading / (float)BATTERY_ADC_MAX_VALUE) * BATTERY_ADC_REFERENCE;
    
    // Convert voltage to current based on current sensor characteristics
    // This assumes a hall effect sensor with 66mV/A sensitivity (ACS712-30A)
    return (voltage - (BATTERY_ADC_REFERENCE / 2.0f)) / 0.066f;
}

void BatteryMonitor::apply_voltage_calibration(float& voltage) {
    if (calibration.calibrated) {
        voltage = (voltage * calibration.voltage_scale) + calibration.voltage_offset;
    }
}

void BatteryMonitor::apply_current_calibration(float& current) {
    if (calibration.calibrated && has_current_sensor) {
        current = (current * calibration.current_scale) + calibration.current_offset;
    }
}

void BatteryMonitor::add_voltage_sample(float voltage) {
    voltage_samples[sample_index] = voltage;
    sample_index = (sample_index + 1) % BATTERY_FILTER_SAMPLES;
    
    if (sample_index == 0) {
        samples_full = true;
    }
}

void BatteryMonitor::add_current_sample(float current) {
    current_samples[sample_index] = current;
}

float BatteryMonitor::get_filtered_voltage() {
    float sum = 0.0f;
    int count = samples_full ? BATTERY_FILTER_SAMPLES : sample_index;
    
    for (int i = 0; i < count; i++) {
        sum += voltage_samples[i];
    }
    
    return count > 0 ? (sum / count) : 0.0f;
}

float BatteryMonitor::get_filtered_current() {
    if (!has_current_sensor) return 0.0f;
    
    float sum = 0.0f;
    int count = samples_full ? BATTERY_FILTER_SAMPLES : sample_index;
    
    for (int i = 0; i < count; i++) {
        sum += current_samples[i];
    }
    
    return count > 0 ? (sum / count) : 0.0f;
}

uint8_t BatteryMonitor::voltage_to_percentage(float voltage) {
    // Use Li-Po 3S curve by default
    return BatteryUtils::voltage_to_percentage_curve(voltage, BatteryUtils::LIPO_3S_CURVE);
}

float BatteryMonitor::percentage_to_voltage(uint8_t percentage) {
    return BatteryUtils::percentage_to_voltage_curve(percentage, BatteryUtils::LIPO_3S_CURVE);
}

BatteryHealth BatteryMonitor::assess_battery_health(float voltage, uint8_t percentage) {
    // Simple health assessment based on voltage and percentage correlation
    float expected_voltage = percentage_to_voltage(percentage);
    float voltage_error = fabs(voltage - expected_voltage);
    
    if (voltage_error < 0.1f) return HEALTH_EXCELLENT;
    if (voltage_error < 0.2f) return HEALTH_GOOD;
    if (voltage_error < 0.4f) return HEALTH_FAIR;
    if (voltage_error < 0.8f) return HEALTH_POOR;
    return HEALTH_CRITICAL;
}

BatteryState BatteryMonitor::determine_battery_state(float voltage, float current, uint8_t percentage) {
    if (voltage < critical_voltage_threshold) return STATE_EMPTY;
    if (percentage >= 95 && fabs(current) < 0.1f) return STATE_FULL;
    if (current > 0.1f) return STATE_CHARGING;
    if (current < -0.1f) return STATE_DISCHARGING;
    if (voltage < low_voltage_threshold) return STATE_EMPTY;
    
    return STATE_UNKNOWN;
}

void BatteryMonitor::update_battery_state() {
    BatteryState new_state = determine_battery_state(current_reading.voltage, current_reading.current, current_reading.percentage);
    
    if (new_state != battery_state) {
        handle_state_change(new_state);
    }
    
    // Update health assessment
    health_status = assess_battery_health(current_reading.voltage, current_reading.percentage);
}

void BatteryMonitor::update_statistics() {
    // Update min/max voltages
    if (current_reading.voltage < stats.min_voltage) {
        stats.min_voltage = current_reading.voltage;
    }
    if (current_reading.voltage > stats.max_voltage) {
        stats.max_voltage = current_reading.voltage;
    }
    
    // Update average voltage (simple moving average)
    stats.avg_voltage = (stats.avg_voltage * 0.99f) + (current_reading.voltage * 0.01f);
    
    // Update runtime
    uint32_t current_time = millis();
    if (last_update_time > 0) {
        stats.runtime += (current_time - last_update_time) / 1000;
    }
}

void BatteryMonitor::update_energy_tracking() {
    uint32_t current_time = millis();
    
    if (last_energy_update > 0 && current_reading.power > 0.0f) {
        float time_diff_hours = (current_time - last_energy_update) / 3600000.0f;
        stats.total_energy += current_reading.power * time_diff_hours;
    }
    
    last_energy_update = current_time;
    last_power_reading = current_reading.power;
}

void BatteryMonitor::check_voltage_thresholds() {
    // Clear previous flags
    battery_flags &= ~(BATTERY_FLAG_LOW | BATTERY_FLAG_CRITICAL);
    
    if (current_reading.voltage <= critical_voltage_threshold) {
        battery_flags |= BATTERY_FLAG_CRITICAL;
        stats.low_voltage_events++;
        trigger_callbacks();
    } else if (current_reading.voltage <= low_voltage_threshold) {
        battery_flags |= BATTERY_FLAG_LOW;
        stats.low_voltage_events++;
        trigger_callbacks();
    }
}

void BatteryMonitor::handle_state_change(BatteryState new_state) {
    BatteryState old_state = battery_state;
    battery_state = new_state;
    state_change_time = millis();
    
    if (state_change_callback) {
        state_change_callback(old_state, new_state);
    }
}

void BatteryMonitor::initialize_samples() {
    for (int i = 0; i < BATTERY_FILTER_SAMPLES; i++) {
        voltage_samples[i] = 0.0f;
        current_samples[i] = 0.0f;
    }
    sample_index = 0;
    samples_full = false;
}

void BatteryMonitor::trigger_callbacks() {
    if ((battery_flags & BATTERY_FLAG_CRITICAL) && critical_voltage_callback) {
        critical_voltage_callback(current_reading.voltage);
    }
    
    if ((battery_flags & BATTERY_FLAG_LOW) && low_voltage_callback) {
        low_voltage_callback(current_reading.voltage);
    }
}

bool BatteryMonitor::is_time_for_update() {
    uint32_t current_time = millis();
    uint32_t interval = (battery_flags & BATTERY_FLAG_CRITICAL) ? BATTERY_FAST_UPDATE : update_interval;
    
    return (current_time - last_update_time) >= interval;
}

// Utility functions implementation
namespace BatteryUtils {
    uint8_t voltage_to_percentage_curve(float voltage, const VoltageCurve& curve) {
        if (voltage <= curve.voltage[0]) return 0;
        if (voltage >= curve.voltage[10]) return 100;
        
        // Linear interpolation between curve points
        for (int i = 0; i < 10; i++) {
            if (voltage >= curve.voltage[i] && voltage <= curve.voltage[i + 1]) {
                float ratio = (voltage - curve.voltage[i]) / (curve.voltage[i + 1] - curve.voltage[i]);
                return (uint8_t)(i * 10 + ratio * 10);
            }
        }
        
        return 50; // Default fallback
    }
    
    float percentage_to_voltage_curve(uint8_t percentage, const VoltageCurve& curve) {
        if (percentage == 0) return curve.voltage[0];
        if (percentage >= 100) return curve.voltage[10];
        
        int index = percentage / 10;
        float remainder = (percentage % 10) / 10.0f;
        
        return curve.voltage[index] + remainder * (curve.voltage[index + 1] - curve.voltage[index]);
    }
    
    float estimate_battery_capacity_ah(float voltage_drop, float current_draw, uint32_t time_ms) {
        if (current_draw <= 0.0f || time_ms == 0) return 0.0f;
        
        float time_hours = time_ms / 3600000.0f;
        return current_draw * time_hours;
    }
    
    float calculate_c_rate(float current_a, float capacity_ah) {
        return (capacity_ah > 0.0f) ? (current_a / capacity_ah) : 0.0f;
    }
    
    float temperature_compensate_voltage(float voltage, float temperature_c, float temp_coeff) {
        float temp_diff = temperature_c - 25.0f; // Reference temperature 25°C
        return voltage * (1.0f + temp_coeff * temp_diff);
    }
    
    bool is_voltage_in_safe_range(float voltage, uint8_t cells, float margin) {
        float min_safe = cells * (3.0f - margin);
        float max_safe = cells * (4.2f + margin);
        return (voltage >= min_safe && voltage <= max_safe);
    }
    
    bool is_discharge_rate_safe(float current_a, float capacity_ah, float max_c_rate) {
        float c_rate = calculate_c_rate(current_a, capacity_ah);
        return c_rate <= max_c_rate;
    }
    
    float calculate_power(float voltage, float current) {
        return voltage * current;
    }
    
    float calculate_energy_wh(float power_w, uint32_t time_ms) {
        return power_w * (time_ms / 3600000.0f);
    }
    
    float calculate_charge_ah(float current_a, uint32_t time_ms) {
        return current_a * (time_ms / 3600000.0f);
    }
    
    uint32_t estimate_cycle_life(float depth_of_discharge, float avg_temperature_c) {
        // Simplified cycle life estimation for Li-Po batteries
        float base_cycles = 500.0f;
        float dod_factor = 1.0f - (depth_of_discharge - 0.2f) * 0.5f; // Reduced life with deeper discharge
        float temp_factor = 1.0f - (fabs(avg_temperature_c - 25.0f) / 50.0f) * 0.3f;
        
        return (uint32_t)(base_cycles * dod_factor * temp_factor);
    }
    
    float calculate_depth_of_discharge(float min_voltage, float max_voltage, const VoltageCurve& curve) {
        uint8_t min_percentage = voltage_to_percentage_curve(min_voltage, curve);
        uint8_t max_percentage = voltage_to_percentage_curve(max_voltage, curve);
        
        return (max_percentage - min_percentage) / 100.0f;
    }
    
    void format_voltage_string(float voltage, char* buffer, size_t buffer_size) {
        snprintf(buffer, buffer_size, "%.2f V", voltage);
    }
    
    void format_current_string(float current, char* buffer, size_t buffer_size) {
        if (fabs(current) >= 1.0f) {
            snprintf(buffer, buffer_size, "%.2f A", current);
        } else {
            snprintf(buffer, buffer_size, "%.0f mA", current * 1000.0f);
        }
    }
    
    void format_power_string(float power, char* buffer, size_t buffer_size) {
        if (power >= 1.0f) {
            snprintf(buffer, buffer_size, "%.2f W", power);
        } else {
            snprintf(buffer, buffer_size, "%.0f mW", power * 1000.0f);
        }
    }
    
    void format_energy_string(float energy, char* buffer, size_t buffer_size) {
        if (energy >= 1.0f) {
            snprintf(buffer, buffer_size, "%.3f Wh", energy);
        } else {
            snprintf(buffer, buffer_size, "%.1f mWh", energy * 1000.0f);
        }
    }
    
    void format_time_string(uint32_t seconds, char* buffer, size_t buffer_size) {
        uint32_t hours = seconds / 3600;
        uint32_t minutes = (seconds % 3600) / 60;
        uint32_t secs = seconds % 60;
        
        if (hours > 0) {
            snprintf(buffer, buffer_size, "%lu:%02lu:%02lu", hours, minutes, secs);
        } else {
            snprintf(buffer, buffer_size, "%02lu:%02lu", minutes, secs);
        }
    }
    
    const char* health_to_string(BatteryHealth health) {
        switch (health) {
            case HEALTH_EXCELLENT: return "Excellent";
            case HEALTH_GOOD: return "Good";
            case HEALTH_FAIR: return "Fair";
            case HEALTH_POOR: return "Poor";
            case HEALTH_CRITICAL: return "Critical";
            default: return "Unknown";
        }
    }
    
    const char* state_to_string(BatteryState state) {
        switch (state) {
            case STATE_CHARGING: return "Charging";
            case STATE_DISCHARGING: return "Discharging";
            case STATE_FULL: return "Full";
            case STATE_EMPTY: return "Empty";
            case STATE_FAULT: return "Fault";
            default: return "Unknown";
        }
    }
    
    uint8_t calculate_health_score(float current_capacity, float original_capacity) {
        if (original_capacity <= 0.0f) return 0;
        
        float ratio = current_capacity / original_capacity;
        return (uint8_t)(ratio * 100.0f);
    }
    
    float calculate_voltage_divider_ratio(float r1, float r2) {
        return (r1 + r2) / r2;
    }
    
    float calculate_shunt_current(float voltage_drop, float shunt_resistance) {
        return voltage_drop / shunt_resistance;
    }
    
    void generate_calibration_points(float* voltages, float* adc_readings, uint8_t num_points) {
        // Generate calibration curve points for voltage divider calibration
        for (uint8_t i = 0; i < num_points; i++) {
            float voltage = 3.0f + (i * 1.2f); // Generate points from 3V to 15V
            voltages[i] = voltage;
            adc_readings[i] = (voltage / VOLTAGE_DIVIDER_RATIO) * (BATTERY_ADC_MAX_VALUE / BATTERY_ADC_REFERENCE);
        }
    }
    
    BatteryTestResult perform_load_test(BatteryMonitor& monitor, float load_current, uint32_t duration_ms) {
        BatteryTestResult result;
        memset(&result, 0, sizeof(BatteryTestResult));
        
        // Record initial voltage
        monitor.update();
        float initial_voltage = monitor.get_voltage();
        
        // Apply load (this would need external circuitry)
        delay(duration_ms);
        
        // Record voltage under load
        monitor.update();
        float loaded_voltage = monitor.get_voltage();
        
        // Calculate internal resistance
        result.voltage_sag = initial_voltage - loaded_voltage;
        result.internal_resistance = result.voltage_sag / load_current;
        
        // Assess health based on voltage sag
        if (result.voltage_sag < 0.2f) {
            result.health_assessment = HEALTH_EXCELLENT;
        } else if (result.voltage_sag < 0.5f) {
            result.health_assessment = HEALTH_GOOD;
        } else if (result.voltage_sag < 1.0f) {
            result.health_assessment = HEALTH_FAIR;
        } else {
            result.health_assessment = HEALTH_POOR;
        }
        
        result.test_passed = (result.voltage_sag < 1.0f);
        
        return result;
    }
    
    bool perform_capacity_test(BatteryMonitor& monitor, float discharge_current) {
        // Simplified capacity test - would need full discharge cycle
        monitor.update();
        float initial_voltage = monitor.get_voltage();
        
        // This is a placeholder - real capacity test would discharge battery completely
        return (initial_voltage > 11.0f); // Basic sanity check
    }
}