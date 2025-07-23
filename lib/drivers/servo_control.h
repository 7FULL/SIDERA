#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>

// Servo Control Driver for SAMD21
// Controls standard RC servos and digital servos
// Features position feedback, speed control, and trajectory planning

// Servo configuration
#define SERVO_MAX_CHANNELS        8       // Maximum number of servo channels
#define SERVO_PWM_FREQUENCY       50      // Standard servo frequency (Hz)
#define SERVO_PWM_PERIOD_US       20000   // PWM period in microseconds (20ms)
#define SERVO_MIN_PULSE_WIDTH     1000    // Minimum pulse width (μs)
#define SERVO_MAX_PULSE_WIDTH     2000    // Maximum pulse width (μs)
#define SERVO_CENTER_PULSE_WIDTH  1500    // Center position pulse width (μs)

// Servo limits and ranges
#define SERVO_MIN_ANGLE           0       // Minimum angle (degrees)
#define SERVO_MAX_ANGLE           180     // Maximum angle (degrees)
#define SERVO_CENTER_ANGLE        90      // Center angle (degrees)
#define SERVO_MAX_SPEED           600     // Maximum speed (degrees/second)
#define SERVO_DEFAULT_SPEED       180     // Default speed (degrees/second)

// Servo types
#define SERVO_TYPE_STANDARD       0       // Standard 180° servo
#define SERVO_TYPE_CONTINUOUS     1       // Continuous rotation servo
#define SERVO_TYPE_DIGITAL        2       // High-speed digital servo
#define SERVO_TYPE_LINEAR         3       // Linear actuator
#define SERVO_TYPE_CUSTOM         4       // Custom configuration

// Control modes
#define SERVO_MODE_POSITION       0       // Position control mode
#define SERVO_MODE_SPEED          1       // Speed control mode (continuous)
#define SERVO_MODE_TORQUE         2       // Torque control mode (if supported)
#define SERVO_MODE_TRAJECTORY     3       // Trajectory following mode

// Error codes
#define SERVO_SUCCESS             0
#define SERVO_ERROR_INIT         -1
#define SERVO_ERROR_INVALID_CHANNEL -2
#define SERVO_ERROR_INVALID_ANGLE -3
#define SERVO_ERROR_INVALID_SPEED -4
#define SERVO_ERROR_TIMEOUT      -5
#define SERVO_ERROR_OVERLOAD     -6
#define SERVO_ERROR_FEEDBACK     -7
#define SERVO_ERROR_LIMITS       -8

// Status flags
#define SERVO_STATUS_ENABLED      0x01    // Servo is enabled
#define SERVO_STATUS_MOVING       0x02    // Servo is moving
#define SERVO_STATUS_AT_TARGET    0x04    // Servo has reached target
#define SERVO_STATUS_OVERLOAD     0x08    // Servo overload detected
#define SERVO_STATUS_FAULT        0x10    // General fault condition
#define SERVO_STATUS_CALIBRATED   0x20    // Servo is calibrated
#define SERVO_STATUS_FEEDBACK_OK  0x40    // Position feedback available

// Timing constants
#define SERVO_UPDATE_INTERVAL_MS  20      // Update interval (50Hz)
#define SERVO_SETTLE_TIME_MS      100     // Time to settle at position
#define SERVO_MOVE_TIMEOUT_MS     5000    // Maximum time for a move
#define SERVO_FEEDBACK_TIMEOUT_MS 1000    // Feedback timeout

// Data structures
struct ServoConfig {
    uint8_t pwm_pin;            // PWM output pin
    uint8_t feedback_pin;       // Analog feedback pin (255 = no feedback)
    uint8_t enable_pin;         // Enable/disable pin (255 = always enabled)
    uint8_t servo_type;         // Type of servo
    uint8_t control_mode;       // Control mode
    uint16_t min_pulse_us;      // Minimum pulse width (μs)
    uint16_t max_pulse_us;      // Maximum pulse width (μs)
    uint16_t center_pulse_us;   // Center pulse width (μs)
    float min_angle;            // Minimum angle (degrees)
    float max_angle;            // Maximum angle (degrees)
    float max_speed;            // Maximum speed (degrees/second)
    float gear_ratio;           // Gear ratio (for multi-turn servos)
    bool reverse_direction;     // Reverse direction flag
    bool enable_feedback;       // Use position feedback
    char name[16];              // Servo name
};

struct ServoStatus {
    float current_angle;        // Current position (degrees)
    float target_angle;         // Target position (degrees)
    float current_speed;        // Current speed (degrees/second)
    float feedback_voltage;     // Feedback voltage (if available)
    uint16_t current_pulse_us;  // Current pulse width (μs)
    uint32_t last_update_time;  // Last update timestamp
    uint32_t move_start_time;   // Move start timestamp
    uint8_t status_flags;       // Status flags bitmask
    uint8_t fault_code;         // Last fault code
    bool enabled;               // Servo enabled state
    bool moving;                // Currently moving flag
};

struct ServoCalibration {
    uint16_t min_feedback_adc;  // ADC value at minimum position
    uint16_t max_feedback_adc;  // ADC value at maximum position
    uint16_t center_feedback_adc; // ADC value at center position
    float feedback_scale;       // Scale factor for feedback
    float feedback_offset;      // Offset for feedback
    bool calibrated;            // Calibration valid flag
};

// Trajectory control
struct ServoTrajectoryPoint {
    float angle;                // Target angle (degrees)
    float speed;                // Speed to reach this point (degrees/second)
    uint32_t dwell_time_ms;     // Time to dwell at this point
    bool wait_for_completion;   // Wait for servo to reach position before continuing
};

struct ServoTrajectory {
    ServoTrajectoryPoint points[16]; // Up to 16 trajectory points
    uint8_t num_points;         // Number of active points
    uint8_t current_point;      // Current trajectory point
    bool loop_trajectory;       // Loop back to start when complete
    bool trajectory_active;     // Trajectory is currently running
    uint32_t trajectory_start_time; // Trajectory start timestamp
    char name[24];              // Trajectory name
};

// Multi-servo coordination
struct ServoGroup {
    uint8_t servo_ids[SERVO_MAX_CHANNELS]; // Servo IDs in group
    uint8_t num_servos;         // Number of servos in group
    bool synchronized;          // Synchronized movement
    float sync_tolerance;       // Position tolerance for sync (degrees)
    char name[16];              // Group name
};

// Performance statistics
struct ServoStatistics {
    uint32_t move_commands;     // Number of move commands executed
    uint32_t position_updates;  // Number of position updates
    uint32_t trajectory_points; // Number of trajectory points executed
    uint32_t fault_count;       // Number of faults occurred
    float total_movement;       // Total movement in degrees
    float max_speed_achieved;   // Maximum speed achieved
    uint32_t total_move_time;   // Total time spent moving (ms)
    float average_settle_time;  // Average settle time (ms)
};

class ServoControl {
private:
    bool initialized;
    uint32_t last_update_time;
    
    ServoConfig servos[SERVO_MAX_CHANNELS];
    ServoStatus servo_status[SERVO_MAX_CHANNELS];
    ServoCalibration servo_calibration[SERVO_MAX_CHANNELS];
    ServoTrajectory trajectories[SERVO_MAX_CHANNELS];
    ServoStatistics statistics[SERVO_MAX_CHANNELS];
    
    // Multi-servo coordination
    ServoGroup groups[4];       // Up to 4 servo groups
    uint8_t num_groups;
    
    // Global settings
    uint16_t update_interval_ms;
    bool smooth_movement;
    float global_speed_limit;
    
    // Callbacks
    void (*move_complete_callback)(uint8_t servo_id, bool success);
    void (*trajectory_complete_callback)(uint8_t servo_id);
    void (*fault_callback)(uint8_t servo_id, uint8_t fault_code);
    
public:
    ServoControl();
    
    // Initialization
    int8_t begin();
    bool is_initialized() const { return initialized; }
    
    // Servo configuration
    int8_t configure_servo(uint8_t servo_id, const ServoConfig& config);
    int8_t enable_servo(uint8_t servo_id, bool enable);
    const ServoConfig& get_servo_config(uint8_t servo_id) const;
    const ServoStatus& get_servo_status(uint8_t servo_id) const;
    
    // Basic servo control
    int8_t move_to_angle(uint8_t servo_id, float target_angle, float speed = 0);
    int8_t move_to_pulse_width(uint8_t servo_id, uint16_t pulse_width_us);
    int8_t set_speed(uint8_t servo_id, float speed); // For continuous rotation servos
    int8_t stop_servo(uint8_t servo_id);
    int8_t home_servo(uint8_t servo_id); // Move to center position
    
    // Position and status queries
    float get_current_angle(uint8_t servo_id);
    float get_target_angle(uint8_t servo_id);
    bool is_moving(uint8_t servo_id);
    bool has_reached_target(uint8_t servo_id, float tolerance = 2.0f);
    bool is_enabled(uint8_t servo_id);
    
    // Calibration
    int8_t start_calibration(uint8_t servo_id);
    int8_t calibrate_position(uint8_t servo_id, float known_angle);
    int8_t save_calibration(uint8_t servo_id);
    int8_t load_calibration(uint8_t servo_id);
    bool is_calibrated(uint8_t servo_id);
    
    // Trajectory control
    int8_t load_trajectory(uint8_t servo_id, const ServoTrajectory& trajectory);
    int8_t start_trajectory(uint8_t servo_id);
    int8_t stop_trajectory(uint8_t servo_id);
    int8_t pause_trajectory(uint8_t servo_id);
    int8_t resume_trajectory(uint8_t servo_id);
    bool is_trajectory_running(uint8_t servo_id);
    uint8_t get_trajectory_progress(uint8_t servo_id);
    
    // Multi-servo control
    int8_t create_servo_group(uint8_t group_id, const uint8_t* servo_ids, uint8_t num_servos, bool synchronized = true);
    int8_t move_group_to_angles(uint8_t group_id, const float* target_angles, float speed = 0);
    int8_t start_group_trajectory(uint8_t group_id);
    bool is_group_moving(uint8_t group_id);
    bool is_group_synchronized(uint8_t group_id);
    
    // Advanced control
    int8_t set_position_limits(uint8_t servo_id, float min_angle, float max_angle);
    int8_t set_speed_limit(uint8_t servo_id, float max_speed);
    int8_t enable_soft_limits(uint8_t servo_id, bool enable);
    int8_t set_compliance(uint8_t servo_id, float compliance); // For flexible movement
    
    // System control
    void update(); // Must be called regularly
    void emergency_stop(); // Stop all servos immediately
    void disable_all_servos();
    void enable_all_servos();
    
    // Configuration and settings
    void set_update_interval(uint16_t interval_ms);
    void enable_smooth_movement(bool enable);
    void set_global_speed_limit(float max_speed);
    
    // Statistics and monitoring
    const ServoStatistics& get_statistics(uint8_t servo_id) const;
    void reset_statistics(uint8_t servo_id);
    void reset_all_statistics();
    uint32_t get_total_move_commands() const;
    
    // Callbacks
    void set_move_complete_callback(void (*callback)(uint8_t servo_id, bool success));
    void set_trajectory_complete_callback(void (*callback)(uint8_t servo_id));
    void set_fault_callback(void (*callback)(uint8_t servo_id, uint8_t fault_code));
    
    // Diagnostic functions
    int8_t run_servo_test(uint8_t servo_id);
    int8_t check_servo_health(uint8_t servo_id);
    void print_servo_status(uint8_t servo_id) const;
    void print_all_servo_status() const;
    void print_statistics(uint8_t servo_id) const;
    
    // Configuration save/load
    int8_t save_configuration();
    int8_t load_configuration();
    
private:
    // Low-level PWM control
    void setup_pwm_timer();
    void set_pwm_pulse_width(uint8_t servo_id, uint16_t pulse_width_us);
    void disable_pwm_output(uint8_t servo_id);
    
    // Position feedback
    float read_position_feedback(uint8_t servo_id);
    void update_position_feedback(uint8_t servo_id);
    float adc_to_angle(uint8_t servo_id, uint16_t adc_value);
    
    // Movement control
    void update_servo_movement(uint8_t servo_id);
    void calculate_smooth_movement(uint8_t servo_id, float& next_angle, float dt);
    bool check_position_limits(uint8_t servo_id, float angle);
    float constrain_speed(uint8_t servo_id, float speed);
    
    // Trajectory execution
    void update_trajectory(uint8_t servo_id);
    void advance_trajectory_point(uint8_t servo_id);
    void complete_trajectory(uint8_t servo_id);
    
    // Multi-servo coordination
    void update_servo_groups();
    bool is_group_synchronized_internal(uint8_t group_id);
    void synchronize_group_movement(uint8_t group_id);
    
    // Fault detection and handling
    void check_servo_faults(uint8_t servo_id);
    void handle_servo_fault(uint8_t servo_id, uint8_t fault_code);
    bool is_move_timeout(uint8_t servo_id);
    bool is_overload_detected(uint8_t servo_id);
    
    // Utility functions
    bool is_valid_servo(uint8_t servo_id);
    float angle_to_pulse_width(uint8_t servo_id, float angle);
    float pulse_width_to_angle(uint8_t servo_id, uint16_t pulse_width_us);
    void update_servo_statistics(uint8_t servo_id);
    
    // Hardware abstraction
    void configure_timer_for_pwm();
    void enable_timer_interrupt();
    void set_timer_compare_value(uint8_t channel, uint16_t value);
    
    // State management
    void mark_movement_complete(uint8_t servo_id, bool success);
    void update_servo_flags(uint8_t servo_id);
    void reset_servo_state(uint8_t servo_id);
};

// Global instance helper
extern ServoControl* g_servo_control;

// Utility functions and helpers
namespace ServoUtils {
    // Configuration builders
    ServoConfig create_standard_servo_config(uint8_t pwm_pin, const char* name = "Servo");
    ServoConfig create_digital_servo_config(uint8_t pwm_pin, uint8_t feedback_pin, const char* name = "Digital");
    ServoConfig create_continuous_servo_config(uint8_t pwm_pin, const char* name = "Continuous");
    ServoConfig create_linear_actuator_config(uint8_t pwm_pin, uint8_t feedback_pin, float stroke_length, const char* name = "Linear");
    
    // Trajectory builders
    ServoTrajectory create_sweep_trajectory(float start_angle, float end_angle, float speed, uint8_t cycles = 1);
    ServoTrajectory create_step_trajectory(const float* angles, const uint32_t* dwell_times, uint8_t num_steps);
    ServoTrajectory create_sine_wave_trajectory(float center_angle, float amplitude, float period_s, uint8_t cycles = 1);
    ServoTrajectory create_custom_trajectory(const ServoTrajectoryPoint* points, uint8_t num_points);
    
    // Calibration helpers
    int8_t auto_calibrate_servo(ServoControl& controller, uint8_t servo_id);
    int8_t calibrate_servo_range(ServoControl& controller, uint8_t servo_id, float min_angle, float max_angle);
    ServoCalibration calculate_calibration_from_points(uint16_t min_adc, uint16_t max_adc, float min_angle, float max_angle);
    
    // Movement planning
    float calculate_move_time(float start_angle, float end_angle, float max_speed);
    float calculate_acceleration_profile(float distance, float max_speed, float accel_time);
    ServoTrajectory plan_smooth_movement(float start_angle, float end_angle, float max_speed, float acceleration);
    
    // Position and angle utilities
    float normalize_angle(float angle); // Normalize to 0-360 range
    float clamp_to_servo_range(float angle, float min_angle, float max_angle);
    float degrees_to_radians(float degrees);
    float radians_to_degrees(float radians);
    
    // Pulse width conversions
    uint16_t angle_to_pulse_width_standard(float angle);
    float pulse_width_to_angle_standard(uint16_t pulse_width_us);
    uint16_t speed_to_pulse_width_continuous(float speed); // For continuous rotation servos
    
    // Feedback analysis
    struct FeedbackAnalysis {
        float position_error;       // Difference between target and actual
        float position_drift;       // Position drift over time
        float noise_level;          // Feedback noise level
        bool feedback_valid;        // Feedback signal is valid
        float linearity_error;      // Non-linearity in feedback
    };
    
    FeedbackAnalysis analyze_position_feedback(const float* feedback_history, const float* target_history, uint16_t num_samples);
    bool is_feedback_signal_valid(float feedback_voltage, float min_voltage = 0.5f, float max_voltage = 4.5f);
    
    // Performance analysis
    struct PerformanceMetrics {
        float average_settle_time;  // Average time to reach target
        float maximum_overshoot;    // Maximum position overshoot
        float steady_state_error;   // Steady-state position error
        float repeatability;        // Position repeatability
        float response_time;        // Response time (10% to 90%)
    };
    
    PerformanceMetrics analyze_servo_performance(const ServoStatistics& stats);
    float calculate_positioning_accuracy(const float* actual_positions, const float* target_positions, uint16_t num_samples);
    
    // Multi-servo coordination
    bool check_collision_potential(const ServoConfig* configs, const float* target_angles, uint8_t num_servos);
    ServoTrajectory create_synchronized_trajectory(const ServoTrajectory* individual_trajectories, uint8_t num_servos);
    float calculate_group_movement_time(const float* start_angles, const float* target_angles, const float* max_speeds, uint8_t num_servos);
    
    // Safety and limits
    bool is_angle_safe(float angle, float min_safe, float max_safe, float margin = 5.0f);
    bool is_speed_safe(float speed, float max_safe_speed);
    float apply_soft_limits(float target_angle, float current_angle, float min_angle, float max_angle, float compliance);
    
    // Communication and control
    void encode_servo_command(uint8_t servo_id, float angle, float speed, uint8_t* buffer);
    void decode_servo_status(const uint8_t* buffer, uint8_t& servo_id, ServoStatus& status);
    void format_servo_telemetry(const ServoStatus& status, char* buffer, size_t buffer_size);
    
    // Configuration validation
    bool validate_servo_config(const ServoConfig& config);
    bool validate_trajectory(const ServoTrajectory& trajectory);
    bool validate_servo_group(const ServoGroup& group, uint8_t max_servos);
    
    // Debug and testing
    void print_servo_config(const ServoConfig& config);
    void print_trajectory_info(const ServoTrajectory& trajectory);
    void log_servo_movement(uint8_t servo_id, float start_angle, float end_angle, uint32_t move_time);
    
    // File I/O (for configuration storage)
    int8_t save_servo_config_to_file(const char* filename, const ServoConfig* configs, uint8_t num_servos);
    int8_t load_servo_config_from_file(const char* filename, ServoConfig* configs, uint8_t max_servos);
    int8_t save_trajectory_to_file(const char* filename, const ServoTrajectory& trajectory);
    int8_t load_trajectory_from_file(const char* filename, ServoTrajectory& trajectory);
    
    // Application-specific templates
    namespace Templates {
        // Flight control surfaces
        extern const ServoConfig AILERON_CONFIG;
        extern const ServoConfig ELEVATOR_CONFIG;
        extern const ServoConfig RUDDER_CONFIG;
        extern const ServoTrajectory CONTROL_SURFACE_TEST;
        
        // Gimbal control
        extern const ServoConfig GIMBAL_PITCH_CONFIG;
        extern const ServoConfig GIMBAL_YAW_CONFIG;
        extern const ServoGroup GIMBAL_GROUP;
        
        // Landing gear
        extern const ServoConfig LANDING_GEAR_CONFIG;
        extern const ServoTrajectory GEAR_RETRACT_SEQUENCE;
        extern const ServoTrajectory GEAR_DEPLOY_SEQUENCE;
        
        // Payload bay doors
        extern const ServoConfig PAYLOAD_DOOR_CONFIG;
        extern const ServoTrajectory DOOR_OPEN_SEQUENCE;
        extern const ServoTrajectory DOOR_CLOSE_SEQUENCE;
        
        // Camera/antenna positioning
        extern const ServoConfig PAN_SERVO_CONFIG;
        extern const ServoConfig TILT_SERVO_CONFIG;
        extern const ServoGroup PAN_TILT_GROUP;
        extern const ServoTrajectory SCANNING_PATTERN;
    }
}

#endif // SERVO_CONTROL_H