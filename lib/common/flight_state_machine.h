#ifndef FLIGHT_STATE_MACHINE_H
#define FLIGHT_STATE_MACHINE_H

#include <stdint.h>
#include <Arduino.h>

// Flight States (already defined in spi_protocol.h but repeated for clarity)
enum FlightState {
    STATE_IDLE = 0x00,
    STATE_ARMED = 0x01,
    STATE_BOOST = 0x02,
    STATE_COAST = 0x03,
    STATE_APOGEE = 0x04,
    STATE_DROGUE = 0x05,
    STATE_MAIN = 0x06,
    STATE_LANDED = 0x07,
    STATE_ERROR = 0xFF
};

// State transition triggers
enum StateTrigger {
    TRIGGER_ARM = 0x01,
    TRIGGER_LAUNCH_DETECT = 0x02,
    TRIGGER_BURNOUT_DETECT = 0x03,
    TRIGGER_APOGEE_DETECT = 0x04,
    TRIGGER_DROGUE_DEPLOY = 0x05,
    TRIGGER_MAIN_DEPLOY = 0x06,
    TRIGGER_LANDING_DETECT = 0x07,
    TRIGGER_TIMEOUT = 0x08,
    TRIGGER_ERROR = 0xFF,
    TRIGGER_MANUAL_OVERRIDE = 0xFE
};

// Event priorities
enum EventPriority {
    PRIORITY_LOW = 0,
    PRIORITY_NORMAL = 1,
    PRIORITY_HIGH = 2,
    PRIORITY_CRITICAL = 3
};

// Sensor data structure for state machine
struct SensorData {
    float altitude;           // Current altitude (m)
    float velocity;          // Vertical velocity (m/s)
    float acceleration;      // Vertical acceleration (m/s²)
    float high_g_accel;      // High-G accelerometer reading (g)
    float pressure;          // Barometric pressure (hPa)
    float battery_voltage;   // Battery voltage (V)
    bool gps_valid;          // GPS fix status
    uint32_t timestamp;      // Timestamp in milliseconds
};

// State machine configuration
struct StateMachineConfig {
    // Launch detection thresholds
    float launch_accel_threshold;     // Acceleration threshold for launch (m/s²)
    float launch_altitude_threshold;  // Altitude change for launch (m)
    uint32_t launch_time_threshold;   // Time above threshold (ms)
    
    // Burnout detection
    float burnout_accel_threshold;    // Acceleration drops below (m/s²)
    uint32_t burnout_time_threshold;  // Time below threshold (ms)
    
    // Apogee detection
    float apogee_velocity_threshold;  // Velocity near zero (m/s)
    float apogee_altitude_delta;      // Altitude change tolerance (m)
    uint32_t apogee_time_threshold;   // Time at apogee (ms)
    
    // Drogue deployment
    uint32_t drogue_delay;            // Delay after apogee (ms)
    float drogue_altitude;            // Alternative altitude trigger (m)
    
    // Main chute deployment
    float main_altitude;              // Main chute altitude (m AGL)
    uint32_t main_timeout;            // Max time before main deploy (ms)
    
    // Landing detection
    float landing_velocity_threshold; // Velocity threshold (m/s)
    float landing_altitude_delta;     // Altitude stability (m)
    uint32_t landing_time_threshold;  // Time stable (ms)
    
    // Safety timeouts
    uint32_t max_flight_time;         // Maximum flight time (ms)
    uint32_t max_boost_time;          // Maximum boost time (ms)
    uint32_t max_coast_time;          // Maximum coast time (ms)
};

// State machine event
struct StateMachineEvent {
    StateTrigger trigger;
    EventPriority priority;
    uint32_t timestamp;
    SensorData sensor_data;
};

// Forward declarations
class FlightStateMachine;

// State handler function type
typedef void (*StateHandler)(FlightStateMachine* fsm, const SensorData& sensors);
typedef bool (*TransitionCondition)(FlightStateMachine* fsm, const SensorData& sensors);

// State machine class
class FlightStateMachine {
private:
    FlightState current_state;
    FlightState previous_state;
    uint32_t state_entry_time;
    uint32_t flight_start_time;
    bool armed;
    
    StateMachineConfig config;
    SensorData baseline_data;
    SensorData current_data;
    
    // State handlers
    StateHandler state_handlers[9];
    
    // Internal state tracking
    float ground_altitude;
    float max_altitude;
    bool launch_detected;
    bool burnout_detected;
    bool apogee_detected;
    bool drogue_deployed;
    bool main_deployed;
    
    // Timing variables
    uint32_t launch_detect_start;
    uint32_t burnout_detect_start;
    uint32_t apogee_detect_start;
    uint32_t landing_detect_start;
    
public:
    FlightStateMachine();
    
    // Initialization and configuration
    void begin();
    void set_config(const StateMachineConfig& cfg);
    void set_baseline_data(const SensorData& baseline);
    
    // Main update function
    void update(const SensorData& sensors);
    
    // State management
    FlightState get_current_state() const { return current_state; }
    FlightState get_previous_state() const { return previous_state; }
    uint32_t get_state_time() const { return millis() - state_entry_time; }
    uint32_t get_flight_time() const;
    
    // Control functions
    bool arm_system();
    bool disarm_system();
    void force_state(FlightState new_state);
    void trigger_event(StateTrigger trigger);
    
    // Status functions
    bool is_armed() const { return armed; }
    bool is_in_flight() const;
    bool should_deploy_drogue() const;
    bool should_deploy_main() const;
    float get_altitude_agl() const;
    float get_max_altitude() const { return max_altitude; }
    
    // State handlers (public for external access)
    void handle_idle(const SensorData& sensors);
    void handle_armed(const SensorData& sensors);
    void handle_boost(const SensorData& sensors);
    void handle_coast(const SensorData& sensors);
    void handle_apogee(const SensorData& sensors);
    void handle_drogue(const SensorData& sensors);
    void handle_main(const SensorData& sensors);
    void handle_landed(const SensorData& sensors);
    void handle_error(const SensorData& sensors);
    
private:
    // Internal functions
    void transition_to_state(FlightState new_state);
    bool check_launch_conditions(const SensorData& sensors);
    bool check_burnout_conditions(const SensorData& sensors);
    bool check_apogee_conditions(const SensorData& sensors);
    bool check_landing_conditions(const SensorData& sensors);
    bool check_safety_timeouts();
    
    // Utility functions
    void reset_detection_timers();
    void update_max_altitude(float altitude);
    const char* state_to_string(FlightState state);
};

// Default configuration
extern const StateMachineConfig DEFAULT_FSM_CONFIG;

#endif // FLIGHT_STATE_MACHINE_H