#ifndef PYROTECHNIC_CONTROL_H
#define PYROTECHNIC_CONTROL_H

#include <Arduino.h>

// Pyrotechnic Control Driver
// Controls pyrotechnic devices (parachute deployment, stage separation, etc.)
// Features safety interlocks, continuity checking, and sequence control

// Safety configuration
#define PYRO_MAX_CHANNELS         4       // Maximum number of pyro channels
#define PYRO_ARM_PIN_REQUIRED     true    // Require arm pin to be active
#define PYRO_CONTINUITY_CHECK     true    // Check continuity before firing
#define PYRO_SAFE_VOLTAGE_MIN     7.0f    // Minimum safe voltage for firing (V)
#define PYRO_SAFE_VOLTAGE_MAX     16.0f   // Maximum safe voltage for firing (V)

// Timing parameters
#define PYRO_FIRE_DURATION_MS     100     // Default fire duration (ms)
#define PYRO_MIN_FIRE_DURATION    10      // Minimum fire duration (ms)
#define PYRO_MAX_FIRE_DURATION    2000    // Maximum fire duration (ms)
#define PYRO_CONTINUITY_DELAY     50      // Delay between continuity checks (ms)
#define PYRO_SAFETY_TIMEOUT       5000    // Safety timeout for armed state (ms)

// Current thresholds (for continuity checking)
#define PYRO_CONTINUITY_CURRENT_MIN  50.0f   // Minimum continuity current (mA)
#define PYRO_CONTINUITY_CURRENT_MAX  500.0f  // Maximum continuity current (mA)
#define PYRO_FIRE_CURRENT_MIN        1000.0f // Minimum fire current (mA)

// Error codes
#define PYRO_SUCCESS                0
#define PYRO_ERROR_INIT            -1
#define PYRO_ERROR_NOT_ARMED       -2
#define PYRO_ERROR_NO_CONTINUITY   -3
#define PYRO_ERROR_VOLTAGE_LOW     -4
#define PYRO_ERROR_VOLTAGE_HIGH    -5
#define PYRO_ERROR_CURRENT_LOW     -6
#define PYRO_ERROR_TIMEOUT         -7
#define PYRO_ERROR_SAFETY          -8
#define PYRO_ERROR_ALREADY_FIRED   -9
#define PYRO_ERROR_INVALID_CHANNEL -10

// Channel states
#define PYRO_STATE_SAFE            0  // Safe state, cannot fire
#define PYRO_STATE_ARMED           1  // Armed, ready to fire
#define PYRO_STATE_FIRED           2  // Has been fired
#define PYRO_STATE_FAULT           3  // Fault condition

// Channel types
#define PYRO_TYPE_DROGUE           0  // Drogue parachute
#define PYRO_TYPE_MAIN             1  // Main parachute
#define PYRO_TYPE_SEPARATION       2  // Stage separation
#define PYRO_TYPE_BACKUP           3  // Backup/redundant channel
#define PYRO_TYPE_CUSTOM           4  // Custom application

// Safety interlocks
#define PYRO_INTERLOCK_NONE        0x00  // No interlocks
#define PYRO_INTERLOCK_ARM_SWITCH  0x01  // Require arm switch
#define PYRO_INTERLOCK_ALTITUDE    0x02  // Altitude-based interlock
#define PYRO_INTERLOCK_VELOCITY    0x04  // Velocity-based interlock
#define PYRO_INTERLOCK_TIME        0x08  // Time-based interlock
#define PYRO_INTERLOCK_CONTINUITY  0x10  // Continuity check required
#define PYRO_INTERLOCK_VOLTAGE     0x20  // Battery voltage check

// Data structures
struct PyroChannelConfig {
    uint8_t fire_pin;           // Digital output pin for firing
    uint8_t sense_pin;          // Analog pin for current sensing
    uint8_t continuity_pin;     // Analog pin for continuity check
    uint16_t fire_duration_ms;  // Fire duration in milliseconds
    uint8_t channel_type;       // Channel type (drogue, main, etc.)
    uint8_t interlocks;         // Safety interlocks bitmask
    float min_fire_voltage;     // Minimum voltage required for firing
    float max_fire_voltage;     // Maximum safe voltage for firing
    bool active;                // Channel is active/configured
    char name[16];              // Human-readable channel name
};

struct PyroChannelStatus {
    uint8_t state;              // Current channel state
    bool continuity_good;       // Continuity check result
    bool voltage_ok;            // Voltage within safe range
    bool armed;                 // Channel is armed
    bool fired;                 // Channel has been fired
    float current_ma;           // Current measured current (mA)
    float voltage;              // Current voltage (V)
    uint32_t last_fire_time;    // Timestamp of last firing
    uint32_t fire_count;        // Number of times fired
    uint8_t fault_code;         // Last fault/error code
    uint32_t last_continuity_check; // Last continuity check time
};

struct PyroSystemStatus {
    bool system_armed;          // Overall system armed state
    bool master_arm_active;     // Master arm switch state
    uint8_t channels_armed;     // Number of channels armed
    uint8_t channels_fired;     // Number of channels fired
    uint8_t channels_fault;     // Number of channels in fault
    float system_voltage;       // System/battery voltage
    bool safety_ok;             // All safety checks passed
    uint32_t arm_time;          // Time when system was armed
    uint32_t last_safety_check; // Last safety check timestamp
    uint8_t active_interlocks;  // Currently active interlocks
};

// Firing sequence structure
struct PyroSequence {
    uint8_t channel_id;         // Channel to fire
    uint32_t delay_ms;          // Delay before firing (from sequence start)
    uint16_t duration_ms;       // Fire duration override (0 = use channel default)
    bool enabled;               // Sequence step enabled
    char description[32];       // Description of this step
};

struct PyroSequenceConfig {
    PyroSequence steps[8];      // Up to 8 sequence steps
    uint8_t num_steps;          // Number of active steps
    bool loop_sequence;         // Loop sequence when complete
    uint32_t loop_delay_ms;     // Delay between sequence loops
    bool abort_on_failure;      // Abort sequence if any step fails
    char name[24];              // Sequence name
};

// Event logging
struct PyroEvent {
    uint32_t timestamp;         // Event timestamp
    uint8_t channel_id;         // Channel involved
    uint8_t event_type;         // Type of event
    uint8_t error_code;         // Error code (if applicable)
    float voltage;              // System voltage at time of event
    float current;              // Channel current at time of event
    char description[32];       // Event description
};

// Event types
#define PYRO_EVENT_ARMED           1  // Channel armed
#define PYRO_EVENT_DISARMED        2  // Channel disarmed
#define PYRO_EVENT_FIRED           3  // Channel fired
#define PYRO_EVENT_CONTINUITY_OK   4  // Continuity check passed
#define PYRO_EVENT_CONTINUITY_FAIL 5  // Continuity check failed
#define PYRO_EVENT_VOLTAGE_LOW     6  // Voltage too low
#define PYRO_EVENT_VOLTAGE_HIGH    7  // Voltage too high
#define PYRO_EVENT_FAULT           8  // General fault
#define PYRO_EVENT_SAFETY_ABORT    9  // Safety system abort
#define PYRO_EVENT_SEQUENCE_START  10 // Firing sequence started
#define PYRO_EVENT_SEQUENCE_END    11 // Firing sequence completed

#define PYRO_MAX_EVENTS           50  // Maximum events to store

class PyrotechnicControl {
private:
    uint8_t master_arm_pin;
    uint8_t system_enable_pin;
    uint8_t voltage_sense_pin;
    bool initialized;
    
    PyroChannelConfig channels[PYRO_MAX_CHANNELS];
    PyroChannelStatus channel_status[PYRO_MAX_CHANNELS];
    PyroSystemStatus system_status;
    
    // Sequence control
    PyroSequenceConfig active_sequence;
    bool sequence_running;
    uint32_t sequence_start_time;
    uint8_t current_sequence_step;
    
    // Event logging
    PyroEvent event_log[PYRO_MAX_EVENTS];
    uint8_t event_index;
    uint16_t total_events;
    
    // Safety monitoring
    uint32_t last_safety_check;
    uint32_t arm_timeout;
    bool safety_override;
    
    // Statistics
    uint32_t total_fires;
    uint32_t continuity_checks;
    uint32_t safety_aborts;
    
    // Callbacks
    void (*fire_callback)(uint8_t channel, bool success);
    void (*safety_callback)(uint8_t error_code);
    void (*continuity_callback)(uint8_t channel, bool good);
    
public:
    PyrotechnicControl(uint8_t master_arm_pin, uint8_t voltage_sense_pin, uint8_t system_enable_pin = 255);
    
    // Initialization
    int8_t begin();
    bool is_initialized() const { return initialized; }
    
    // Channel configuration
    int8_t configure_channel(uint8_t channel_id, const PyroChannelConfig& config);
    int8_t enable_channel(uint8_t channel_id, bool enable);
    const PyroChannelConfig& get_channel_config(uint8_t channel_id) const;
    const PyroChannelStatus& get_channel_status(uint8_t channel_id) const;
    
    // System control
    int8_t arm_system(bool arm);
    int8_t arm_channel(uint8_t channel_id, bool arm);
    bool is_system_armed() const { return system_status.system_armed; }
    bool is_channel_armed(uint8_t channel_id) const;
    
    // Safety functions
    int8_t perform_safety_check();
    int8_t check_continuity(uint8_t channel_id);
    int8_t check_all_continuity();
    bool is_safe_to_fire(uint8_t channel_id);
    void enable_safety_override(bool enable) { safety_override = enable; }
    
    // Firing functions
    int8_t fire_channel(uint8_t channel_id, uint16_t duration_ms = 0);
    int8_t fire_multiple_channels(const uint8_t* channel_ids, uint8_t num_channels, uint16_t duration_ms = 0);
    int8_t abort_all_firing();
    
    // Sequence control
    int8_t load_sequence(const PyroSequenceConfig& sequence);
    int8_t start_sequence();
    int8_t stop_sequence();
    int8_t pause_sequence();
    int8_t resume_sequence();
    bool is_sequence_running() const { return sequence_running; }
    uint8_t get_sequence_progress() const;
    
    // Status and monitoring
    void update();
    const PyroSystemStatus& get_system_status() const { return system_status; }
    float get_system_voltage();
    uint8_t get_channels_ready() const;
    uint8_t get_channels_armed() const;
    
    // Voltage and current monitoring
    float read_channel_current(uint8_t channel_id);
    float read_system_voltage();
    bool is_voltage_in_range(float voltage);
    
    // Event logging
    void log_event(uint8_t channel_id, uint8_t event_type, uint8_t error_code = 0, const char* description = nullptr);
    const PyroEvent* get_event_log(uint16_t& num_events) const;
    void clear_event_log();
    void print_event_log() const;
    
    // Statistics
    uint32_t get_total_fires() const { return total_fires; }
    uint32_t get_continuity_checks() const { return continuity_checks; }
    uint32_t get_safety_aborts() const { return safety_aborts; }
    void reset_statistics();
    
    // Callbacks
    void set_fire_callback(void (*callback)(uint8_t channel, bool success));
    void set_safety_callback(void (*callback)(uint8_t error_code));
    void set_continuity_callback(void (*callback)(uint8_t channel, bool good));
    
    // Configuration saving/loading
    int8_t save_configuration();
    int8_t load_configuration();
    int8_t factory_reset();
    
    // Testing and diagnostics
    int8_t run_self_test();
    int8_t test_channel(uint8_t channel_id, bool low_current_test = true);
    void print_system_status() const;
    void print_channel_status(uint8_t channel_id) const;
    void print_configuration() const;
    
    // Emergency functions
    int8_t emergency_disarm();
    int8_t emergency_stop();
    bool is_emergency_stop_active() const;
    
    // Advanced features
    int8_t set_altitude_interlock(float min_altitude, float max_altitude);
    int8_t set_velocity_interlock(float max_velocity);
    int8_t set_time_interlock(uint32_t min_time_ms, uint32_t max_time_ms);
    int8_t enable_redundant_firing(uint8_t primary_channel, uint8_t backup_channel, uint32_t delay_ms);
    
private:
    // Low-level hardware control
    void set_channel_output(uint8_t channel_id, bool state);
    float read_channel_analog(uint8_t pin);
    bool read_digital_input(uint8_t pin);
    
    // Safety checking functions
    bool check_master_arm();
    bool check_system_voltage();
    bool check_channel_continuity(uint8_t channel_id);
    bool check_interlocks(uint8_t channel_id);
    bool check_arm_timeout();
    
    // Sequence management
    void update_sequence();
    void execute_sequence_step(uint8_t step);
    void complete_sequence();
    void abort_sequence(uint8_t error_code);
    
    // Current sensing and calculations
    float adc_to_current(uint16_t adc_value, bool high_range = false);
    float adc_to_voltage(uint16_t adc_value);
    bool is_continuity_current_valid(float current_ma);
    bool is_fire_current_valid(float current_ma);
    
    // State management
    void update_channel_state(uint8_t channel_id);
    void update_system_state();
    void handle_channel_fault(uint8_t channel_id, uint8_t fault_code);
    void trigger_safety_abort(uint8_t error_code);
    
    // Timing and delays
    bool is_time_elapsed(uint32_t start_time, uint32_t duration_ms);
    void safe_delay(uint32_t ms);
    
    // Event management
    void add_event(const PyroEvent& event);
    const char* event_type_to_string(uint8_t event_type);
    const char* error_code_to_string(uint8_t error_code);
    
    // Validation functions
    bool is_valid_channel(uint8_t channel_id);
    bool is_valid_configuration(const PyroChannelConfig& config);
    bool is_valid_sequence(const PyroSequenceConfig& sequence);
    
    // Hardware abstraction
    void configure_pins();
    void configure_adc();
    void configure_timers();
};

// Global instance helper
extern PyrotechnicControl* g_pyro_control;

// Utility functions
namespace PyroUtils {
    // Configuration helpers
    PyroChannelConfig create_drogue_config(uint8_t fire_pin, uint8_t sense_pin, const char* name = "Drogue");
    PyroChannelConfig create_main_config(uint8_t fire_pin, uint8_t sense_pin, const char* name = "Main");
    PyroChannelConfig create_separation_config(uint8_t fire_pin, uint8_t sense_pin, const char* name = "Separation");
    
    // Sequence builders
    PyroSequenceConfig create_standard_recovery_sequence();
    PyroSequenceConfig create_two_stage_sequence();
    PyroSequenceConfig create_test_sequence();
    
    // Safety calculations
    float calculate_safe_fire_voltage(float battery_voltage, float voltage_drop);
    uint16_t calculate_fire_duration(float target_current, float resistance);
    float estimate_channel_resistance(float voltage, float current);
    
    // Current sensing calibration
    struct CurrentSensorCalibration {
        float offset_mv;        // Zero current offset (mV)
        float sensitivity_mv_a; // Sensitivity (mV/A)
        float max_current_a;    // Maximum measurable current
        bool bidirectional;     // Can measure negative current
    };
    
    float calibrate_current_sensor(uint16_t zero_reading, uint16_t full_scale_reading, float full_scale_current);
    CurrentSensorCalibration get_acs712_calibration(uint8_t variant); // 5A, 20A, 30A variants
    CurrentSensorCalibration get_ina219_calibration(float shunt_ohms);
    
    // Resistance calculations for pyrotechnic devices
    float calculate_nichrome_resistance(float length_m, float diameter_mm);
    float calculate_pyro_power(float voltage, float resistance);
    uint16_t calculate_heating_time(float mass_g, float specific_heat, float temp_rise);
    
    // Safety analysis
    struct SafetyAnalysis {
        bool voltage_adequate;
        bool current_adequate;
        bool continuity_good;
        bool timing_safe;
        bool interlocks_satisfied;
        float safety_margin;
        uint8_t risk_level;     // 0=safe, 1=caution, 2=warning, 3=danger
    };
    
    SafetyAnalysis analyze_firing_safety(const PyroChannelConfig& config, const PyroChannelStatus& status);
    uint8_t calculate_risk_level(float voltage, float current, bool continuity);
    
    // Event analysis
    void analyze_event_log(const PyroEvent* events, uint16_t num_events);
    uint16_t count_events_by_type(const PyroEvent* events, uint16_t num_events, uint8_t event_type);
    float calculate_success_rate(const PyroEvent* events, uint16_t num_events);
    
    // Configuration validation
    bool validate_channel_pins(const PyroChannelConfig& config);
    bool validate_firing_parameters(const PyroChannelConfig& config);
    bool validate_safety_interlocks(const PyroChannelConfig& config);
    
    // Diagnostic helpers
    void print_channel_diagnostics(uint8_t channel_id, const PyroChannelStatus& status);
    void print_safety_status(const PyroSystemStatus& status);
    void format_event_string(const PyroEvent& event, char* buffer, size_t buffer_size);
    
    // Configuration templates
    namespace Templates {
        // Standard rocket recovery system
        extern const PyroChannelConfig DROGUE_CHANNEL;
        extern const PyroChannelConfig MAIN_CHANNEL;
        extern const PyroSequenceConfig STANDARD_RECOVERY;
        
        // Two-stage rocket
        extern const PyroChannelConfig SEPARATION_CHANNEL;
        extern const PyroChannelConfig SECOND_STAGE_IGNITION;
        extern const PyroSequenceConfig TWO_STAGE_SEQUENCE;
        
        // Competition rocket (dual deploy)
        extern const PyroChannelConfig APOGEE_CHANNEL;
        extern const PyroChannelConfig MAIN_DEPLOY_CHANNEL;
        extern const PyroSequenceConfig DUAL_DEPLOY_SEQUENCE;
        
        // Test configurations
        extern const PyroChannelConfig TEST_CHANNEL;
        extern const PyroSequenceConfig CONTINUITY_TEST_SEQUENCE;
    }
    
    // File I/O helpers (for SD card storage)
    int8_t save_configuration_to_file(const char* filename, const PyroChannelConfig* configs, uint8_t num_channels);
    int8_t load_configuration_from_file(const char* filename, PyroChannelConfig* configs, uint8_t max_channels);
    int8_t save_event_log_to_file(const char* filename, const PyroEvent* events, uint16_t num_events);
    
    // Communication protocol helpers
    void encode_channel_status(const PyroChannelStatus& status, uint8_t* buffer);
    void decode_channel_status(PyroChannelStatus& status, const uint8_t* buffer);
    void encode_system_status(const PyroSystemStatus& status, uint8_t* buffer);
    void decode_system_status(PyroSystemStatus& status, const uint8_t* buffer);
}

#endif // PYROTECHNIC_CONTROL_H