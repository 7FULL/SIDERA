#include "servo_control.h"
#include <math.h>

ServoControl* g_servo_control = nullptr;

// Template configurations
namespace ServoUtils {
    namespace Templates {
        const ServoConfig AILERON_CONFIG = {
            2,                              // pwm_pin
            255,                           // feedback_pin (no feedback)
            255,                           // enable_pin (always enabled)
            SERVO_TYPE_STANDARD,           // servo_type
            SERVO_MODE_POSITION,           // control_mode
            SERVO_MIN_PULSE_WIDTH,         // min_pulse_us
            SERVO_MAX_PULSE_WIDTH,         // max_pulse_us
            SERVO_CENTER_PULSE_WIDTH,      // center_pulse_us
            -45.0f,                        // min_angle
            45.0f,                         // max_angle
            SERVO_DEFAULT_SPEED,           // max_speed
            1.0f,                          // gear_ratio
            false,                         // reverse_direction
            false,                         // enable_feedback
            "Aileron"                      // name
        };
        
        const ServoConfig GIMBAL_PITCH_CONFIG = {
            3,                              // pwm_pin
            A0,                            // feedback_pin
            4,                             // enable_pin
            SERVO_TYPE_DIGITAL,            // servo_type
            SERVO_MODE_POSITION,           // control_mode
            600,                           // min_pulse_us (wider range for digital servo)
            2400,                          // max_pulse_us
            1500,                          // center_pulse_us
            -90.0f,                        // min_angle
            90.0f,                         // max_angle
            360.0f,                        // max_speed (fast digital servo)
            1.0f,                          // gear_ratio
            false,                         // reverse_direction
            true,                          // enable_feedback
            "Gimbal Pitch"                 // name
        };
    }
}

ServoControl::ServoControl() {
    initialized = false;
    last_update_time = 0;
    
    // Initialize servo configurations
    memset(servos, 0, sizeof(servos));
    memset(servo_status, 0, sizeof(servo_status));
    memset(servo_calibration, 0, sizeof(servo_calibration));
    memset(trajectories, 0, sizeof(trajectories));
    memset(statistics, 0, sizeof(statistics));
    
    // Initialize groups
    memset(groups, 0, sizeof(groups));
    num_groups = 0;
    
    // Initialize global settings
    update_interval_ms = SERVO_UPDATE_INTERVAL_MS;
    smooth_movement = true;
    global_speed_limit = SERVO_MAX_SPEED;
    
    // Initialize callbacks
    move_complete_callback = nullptr;
    trajectory_complete_callback = nullptr;
    fault_callback = nullptr;
}

int8_t ServoControl::begin() {
    // Setup PWM timer for servo control
    setup_pwm_timer();
    
    // Initialize all configured servos
    for (uint8_t i = 0; i < SERVO_MAX_CHANNELS; i++) {
        if (servos[i].pwm_pin > 0) {
            // Configure PWM pin
            pinMode(servos[i].pwm_pin, OUTPUT);
            
            // Configure enable pin if specified
            if (servos[i].enable_pin != 255) {
                pinMode(servos[i].enable_pin, OUTPUT);
                digitalWrite(servos[i].enable_pin, LOW); // Start disabled
            }
            
            // Configure feedback pin if specified
            if (servos[i].feedback_pin != 255) {
                pinMode(servos[i].feedback_pin, INPUT);
            }
            
            // Initialize servo status
            servo_status[i].current_angle = SERVO_CENTER_ANGLE;
            servo_status[i].target_angle = SERVO_CENTER_ANGLE;
            servo_status[i].current_speed = 0.0f;
            servo_status[i].current_pulse_us = servos[i].center_pulse_us;
            servo_status[i].last_update_time = millis();
            servo_status[i].move_start_time = 0;
            servo_status[i].status_flags = 0;
            servo_status[i].fault_code = SERVO_SUCCESS;
            servo_status[i].enabled = false;
            servo_status[i].moving = false;
            
            // Set initial PWM to center position
            set_pwm_pulse_width(i, servos[i].center_pulse_us);
        }
    }
    
    initialized = true;
    last_update_time = millis();
    
    return SERVO_SUCCESS;
}

int8_t ServoControl::configure_servo(uint8_t servo_id, const ServoConfig& config) {
    if (servo_id >= SERVO_MAX_CHANNELS) {
        return SERVO_ERROR_INVALID_CHANNEL;
    }
    
    if (!ServoUtils::validate_servo_config(config)) {
        return SERVO_ERROR_INIT;
    }
    
    // Copy configuration
    servos[servo_id] = config;
    
    // Reset calibration and statistics
    memset(&servo_calibration[servo_id], 0, sizeof(ServoCalibration));
    memset(&statistics[servo_id], 0, sizeof(ServoStatistics));
    
    // Initialize hardware if already initialized
    if (initialized) {
        pinMode(config.pwm_pin, OUTPUT);
        
        if (config.enable_pin != 255) {
            pinMode(config.enable_pin, OUTPUT);
            digitalWrite(config.enable_pin, LOW);
        }
        
        if (config.feedback_pin != 255) {
            pinMode(config.feedback_pin, INPUT);
        }
        
        // Reset servo to center position
        servo_status[servo_id].current_angle = (config.min_angle + config.max_angle) / 2.0f;
        servo_status[servo_id].target_angle = servo_status[servo_id].current_angle;
        servo_status[servo_id].current_pulse_us = config.center_pulse_us;
        
        set_pwm_pulse_width(servo_id, config.center_pulse_us);
    }
    
    return SERVO_SUCCESS;
}

int8_t ServoControl::enable_servo(uint8_t servo_id, bool enable) {
    if (!is_valid_servo(servo_id)) {
        return SERVO_ERROR_INVALID_CHANNEL;
    }
    
    servo_status[servo_id].enabled = enable;
    
    // Control enable pin if available
    if (servos[servo_id].enable_pin != 255) {
        digitalWrite(servos[servo_id].enable_pin, enable ? HIGH : LOW);
    }
    
    // Update status flags
    if (enable) {
        servo_status[servo_id].status_flags |= SERVO_STATUS_ENABLED;
    } else {
        servo_status[servo_id].status_flags &= ~SERVO_STATUS_ENABLED;
        servo_status[servo_id].moving = false;
        servo_status[servo_id].status_flags &= ~SERVO_STATUS_MOVING;
        
        // Disable PWM output
        disable_pwm_output(servo_id);
    }
    
    return SERVO_SUCCESS;
}

const ServoConfig& ServoControl::get_servo_config(uint8_t servo_id) const {
    static ServoConfig empty_config = {0};
    
    if (servo_id >= SERVO_MAX_CHANNELS) {
        return empty_config;
    }
    
    return servos[servo_id];
}

const ServoStatus& ServoControl::get_servo_status(uint8_t servo_id) const {
    static ServoStatus empty_status = {0};
    
    if (servo_id >= SERVO_MAX_CHANNELS) {
        return empty_status;
    }
    
    return servo_status[servo_id];
}

int8_t ServoControl::move_to_angle(uint8_t servo_id, float target_angle, float speed) {
    if (!is_valid_servo(servo_id)) {
        return SERVO_ERROR_INVALID_CHANNEL;
    }
    
    if (!servo_status[servo_id].enabled) {
        return SERVO_ERROR_INIT;
    }
    
    // Check angle limits
    if (!check_position_limits(servo_id, target_angle)) {
        return SERVO_ERROR_INVALID_ANGLE;
    }
    
    // Use default speed if not specified
    if (speed == 0) {
        speed = servos[servo_id].max_speed;
    }
    
    // Constrain speed
    speed = constrain_speed(servo_id, speed);
    
    // Update servo status
    servo_status[servo_id].target_angle = target_angle;
    servo_status[servo_id].current_speed = speed;
    servo_status[servo_id].move_start_time = millis();
    servo_status[servo_id].moving = true;
    servo_status[servo_id].status_flags |= SERVO_STATUS_MOVING;
    servo_status[servo_id].status_flags &= ~SERVO_STATUS_AT_TARGET;
    
    // Update statistics
    statistics[servo_id].move_commands++;
    
    return SERVO_SUCCESS;
}

int8_t ServoControl::move_to_pulse_width(uint8_t servo_id, uint16_t pulse_width_us) {
    if (!is_valid_servo(servo_id)) {
        return SERVO_ERROR_INVALID_CHANNEL;
    }
    
    // Convert pulse width to angle
    float target_angle = pulse_width_to_angle(servo_id, pulse_width_us);
    
    return move_to_angle(servo_id, target_angle);
}

int8_t ServoControl::set_speed(uint8_t servo_id, float speed) {
    if (!is_valid_servo(servo_id)) {
        return SERVO_ERROR_INVALID_CHANNEL;
    }
    
    if (servos[servo_id].servo_type != SERVO_TYPE_CONTINUOUS) {
        return SERVO_ERROR_INVALID_SPEED;
    }
    
    // For continuous rotation servos, speed is mapped to pulse width
    uint16_t pulse_width = ServoUtils::speed_to_pulse_width_continuous(speed);
    
    servo_status[servo_id].current_speed = speed;
    servo_status[servo_id].current_pulse_us = pulse_width;
    
    set_pwm_pulse_width(servo_id, pulse_width);
    
    return SERVO_SUCCESS;
}

int8_t ServoControl::stop_servo(uint8_t servo_id) {
    if (!is_valid_servo(servo_id)) {
        return SERVO_ERROR_INVALID_CHANNEL;
    }
    
    // Stop movement
    servo_status[servo_id].moving = false;
    servo_status[servo_id].status_flags &= ~SERVO_STATUS_MOVING;
    servo_status[servo_id].current_speed = 0.0f;
    
    // For continuous rotation servos, set to stop position
    if (servos[servo_id].servo_type == SERVO_TYPE_CONTINUOUS) {
        set_pwm_pulse_width(servo_id, servos[servo_id].center_pulse_us);
    }
    
    return SERVO_SUCCESS;
}

int8_t ServoControl::home_servo(uint8_t servo_id) {
    if (!is_valid_servo(servo_id)) {
        return SERVO_ERROR_INVALID_CHANNEL;
    }
    
    float center_angle = (servos[servo_id].min_angle + servos[servo_id].max_angle) / 2.0f;
    return move_to_angle(servo_id, center_angle);
}

float ServoControl::get_current_angle(uint8_t servo_id) {
    if (!is_valid_servo(servo_id)) {
        return 0.0f;
    }
    
    return servo_status[servo_id].current_angle;
}

float ServoControl::get_target_angle(uint8_t servo_id) {
    if (!is_valid_servo(servo_id)) {
        return 0.0f;
    }
    
    return servo_status[servo_id].target_angle;
}

bool ServoControl::is_moving(uint8_t servo_id) {
    if (!is_valid_servo(servo_id)) {
        return false;
    }
    
    return servo_status[servo_id].moving;
}

bool ServoControl::has_reached_target(uint8_t servo_id, float tolerance) {
    if (!is_valid_servo(servo_id)) {
        return false;
    }
    
    float error = fabs(servo_status[servo_id].current_angle - servo_status[servo_id].target_angle);
    return error <= tolerance;
}

bool ServoControl::is_enabled(uint8_t servo_id) {
    if (!is_valid_servo(servo_id)) {
        return false;
    }
    
    return servo_status[servo_id].enabled;
}

void ServoControl::update() {
    if (!initialized) {
        return;
    }
    
    uint32_t current_time = millis();
    
    // Check if it's time for an update
    if (current_time - last_update_time < update_interval_ms) {
        return;
    }
    
    float dt = (current_time - last_update_time) / 1000.0f; // Convert to seconds
    
    // Update all active servos
    for (uint8_t i = 0; i < SERVO_MAX_CHANNELS; i++) {
        if (servos[i].pwm_pin > 0 && servo_status[i].enabled) {
            update_servo_movement(i);
            update_position_feedback(i);
            update_trajectory(i);
            check_servo_faults(i);
            update_servo_statistics(i);
        }
    }
    
    // Update servo groups
    update_servo_groups();
    
    last_update_time = current_time;
}

void ServoControl::emergency_stop() {
    for (uint8_t i = 0; i < SERVO_MAX_CHANNELS; i++) {
        if (servos[i].pwm_pin > 0) {
            stop_servo(i);
            disable_pwm_output(i);
        }
    }
}

void ServoControl::disable_all_servos() {
    for (uint8_t i = 0; i < SERVO_MAX_CHANNELS; i++) {
        if (servos[i].pwm_pin > 0) {
            enable_servo(i, false);
        }
    }
}

void ServoControl::enable_all_servos() {
    for (uint8_t i = 0; i < SERVO_MAX_CHANNELS; i++) {
        if (servos[i].pwm_pin > 0) {
            enable_servo(i, true);
        }
    }
}

void ServoControl::set_update_interval(uint16_t interval_ms) {
    update_interval_ms = constrain(interval_ms, 10, 100); // 10-100ms range
}

void ServoControl::enable_smooth_movement(bool enable) {
    smooth_movement = enable;
}

void ServoControl::set_global_speed_limit(float max_speed) {
    global_speed_limit = constrain(max_speed, 10.0f, 1000.0f);
}

const ServoStatistics& ServoControl::get_statistics(uint8_t servo_id) const {
    static ServoStatistics empty_stats = {0};
    
    if (servo_id >= SERVO_MAX_CHANNELS) {
        return empty_stats;
    }
    
    return statistics[servo_id];
}

void ServoControl::reset_statistics(uint8_t servo_id) {
    if (servo_id < SERVO_MAX_CHANNELS) {
        memset(&statistics[servo_id], 0, sizeof(ServoStatistics));
    }
}

void ServoControl::reset_all_statistics() {
    for (uint8_t i = 0; i < SERVO_MAX_CHANNELS; i++) {
        reset_statistics(i);
    }
}

uint32_t ServoControl::get_total_move_commands() const {
    uint32_t total = 0;
    for (uint8_t i = 0; i < SERVO_MAX_CHANNELS; i++) {
        total += statistics[i].move_commands;
    }
    return total;
}

void ServoControl::set_move_complete_callback(void (*callback)(uint8_t servo_id, bool success)) {
    move_complete_callback = callback;
}

void ServoControl::set_trajectory_complete_callback(void (*callback)(uint8_t servo_id)) {
    trajectory_complete_callback = callback;
}

void ServoControl::set_fault_callback(void (*callback)(uint8_t servo_id, uint8_t fault_code)) {
    fault_callback = callback;
}

void ServoControl::print_servo_status(uint8_t servo_id) const {
    if (!is_valid_servo(servo_id)) {
        Serial.println("Invalid servo ID");
        return;
    }
    
    const ServoStatus& status = servo_status[servo_id];
    const ServoConfig& config = servos[servo_id];
    
    Serial.print("=== Servo ");
    Serial.print(servo_id);
    Serial.print(" (");
    Serial.print(config.name);
    Serial.println(") ===");
    
    Serial.print("Enabled: "); Serial.println(status.enabled ? "YES" : "NO");
    Serial.print("Current Angle: "); Serial.print(status.current_angle, 2); Serial.println("°");
    Serial.print("Target Angle: "); Serial.print(status.target_angle, 2); Serial.println("°");
    Serial.print("Current Speed: "); Serial.print(status.current_speed, 1); Serial.println("°/s");
    Serial.print("Pulse Width: "); Serial.print(status.current_pulse_us); Serial.println(" μs");
    Serial.print("Moving: "); Serial.println(status.moving ? "YES" : "NO");
    Serial.print("At Target: "); Serial.println((status.status_flags & SERVO_STATUS_AT_TARGET) ? "YES" : "NO");
    
    if (config.enable_feedback) {
        Serial.print("Feedback Voltage: "); Serial.print(status.feedback_voltage, 3); Serial.println(" V");
        Serial.print("Feedback OK: "); Serial.println((status.status_flags & SERVO_STATUS_FEEDBACK_OK) ? "YES" : "NO");
    }
    
    if (status.fault_code != SERVO_SUCCESS) {
        Serial.print("Fault Code: "); Serial.println(status.fault_code);
    }
}

void ServoControl::print_all_servo_status() const {
    Serial.println("=== All Servo Status ===");
    
    for (uint8_t i = 0; i < SERVO_MAX_CHANNELS; i++) {
        if (servos[i].pwm_pin > 0) {
            print_servo_status(i);
            Serial.println();
        }
    }
}

void ServoControl::print_statistics(uint8_t servo_id) const {
    if (!is_valid_servo(servo_id)) {
        return;
    }
    
    const ServoStatistics& stats = statistics[servo_id];
    
    Serial.print("=== Servo ");
    Serial.print(servo_id);
    Serial.println(" Statistics ===");
    
    Serial.print("Move Commands: "); Serial.println(stats.move_commands);
    Serial.print("Position Updates: "); Serial.println(stats.position_updates);
    Serial.print("Trajectory Points: "); Serial.println(stats.trajectory_points);
    Serial.print("Fault Count: "); Serial.println(stats.fault_count);
    Serial.print("Total Movement: "); Serial.print(stats.total_movement, 1); Serial.println("°");
    Serial.print("Max Speed Achieved: "); Serial.print(stats.max_speed_achieved, 1); Serial.println("°/s");
    Serial.print("Total Move Time: "); Serial.print(stats.total_move_time); Serial.println(" ms");
    Serial.print("Average Settle Time: "); Serial.print(stats.average_settle_time, 1); Serial.println(" ms");
}

// Private function implementations
void ServoControl::setup_pwm_timer() {
    // Configure timer for 50Hz PWM (20ms period)
    // This is platform-specific implementation for SAMD21
    configure_timer_for_pwm();
}

void ServoControl::set_pwm_pulse_width(uint8_t servo_id, uint16_t pulse_width_us) {
    if (!is_valid_servo(servo_id)) {
        return;
    }
    
    // Constrain pulse width to safe limits
    pulse_width_us = constrain(pulse_width_us, 
                              servos[servo_id].min_pulse_us, 
                              servos[servo_id].max_pulse_us);
    
    servo_status[servo_id].current_pulse_us = pulse_width_us;
    
    // Convert pulse width to timer compare value
    uint16_t compare_value = (pulse_width_us * 1000) / (SERVO_PWM_PERIOD_US / 1000);
    set_timer_compare_value(servo_id, compare_value);
}

void ServoControl::disable_pwm_output(uint8_t servo_id) {
    if (!is_valid_servo(servo_id)) {
        return;
    }
    
    // Disable PWM output by setting compare value to 0
    set_timer_compare_value(servo_id, 0);
}

float ServoControl::read_position_feedback(uint8_t servo_id) {
    if (!is_valid_servo(servo_id) || servos[servo_id].feedback_pin == 255) {
        return 0.0f;
    }
    
    uint16_t adc_value = analogRead(servos[servo_id].feedback_pin);
    float voltage = (adc_value / 1023.0f) * 3.3f; // Assuming 3.3V reference
    
    servo_status[servo_id].feedback_voltage = voltage;
    
    return adc_to_angle(servo_id, adc_value);
}

void ServoControl::update_position_feedback(uint8_t servo_id) {
    if (!servos[servo_id].enable_feedback) {
        return;
    }
    
    float feedback_angle = read_position_feedback(servo_id);
    
    // Update current angle based on feedback
    if (servo_calibration[servo_id].calibrated) {
        servo_status[servo_id].current_angle = feedback_angle;
        servo_status[servo_id].status_flags |= SERVO_STATUS_FEEDBACK_OK;
    }
}

float ServoControl::adc_to_angle(uint8_t servo_id, uint16_t adc_value) {
    if (!servo_calibration[servo_id].calibrated) {
        // Use linear mapping if not calibrated
        float ratio = (float)adc_value / 1023.0f;
        return servos[servo_id].min_angle + 
               ratio * (servos[servo_id].max_angle - servos[servo_id].min_angle);
    }
    
    // Use calibrated mapping
    const ServoCalibration& cal = servo_calibration[servo_id];
    float normalized = (adc_value - cal.min_feedback_adc) / 
                      (float)(cal.max_feedback_adc - cal.min_feedback_adc);
    
    return servos[servo_id].min_angle + 
           normalized * (servos[servo_id].max_angle - servos[servo_id].min_angle);
}

void ServoControl::update_servo_movement(uint8_t servo_id) {
    if (!servo_status[servo_id].moving) {
        return;
    }
    
    float current_angle = servo_status[servo_id].current_angle;
    float target_angle = servo_status[servo_id].target_angle;
    float max_speed = servo_status[servo_id].current_speed;
    
    float dt = update_interval_ms / 1000.0f; // Convert to seconds
    float angle_diff = target_angle - current_angle;
    
    // Check if we've reached the target
    if (fabs(angle_diff) <= 1.0f) { // 1 degree tolerance
        servo_status[servo_id].current_angle = target_angle;
        servo_status[servo_id].moving = false;
        servo_status[servo_id].status_flags &= ~SERVO_STATUS_MOVING;
        servo_status[servo_id].status_flags |= SERVO_STATUS_AT_TARGET;
        
        mark_movement_complete(servo_id, true);
        return;
    }
    
    // Calculate next position
    float max_move = max_speed * dt;
    float next_angle;
    
    if (smooth_movement) {
        calculate_smooth_movement(servo_id, next_angle, dt);
    } else {
        // Simple linear movement
        if (fabs(angle_diff) <= max_move) {
            next_angle = target_angle;
        } else {
            next_angle = current_angle + (angle_diff > 0 ? max_move : -max_move);
        }
    }
    
    // Update position
    servo_status[servo_id].current_angle = next_angle;
    
    // Convert angle to pulse width and update PWM
    uint16_t pulse_width = (uint16_t)angle_to_pulse_width(servo_id, next_angle);
    set_pwm_pulse_width(servo_id, pulse_width);
    
    // Update statistics
    statistics[servo_id].position_updates++;
    float movement = fabs(next_angle - current_angle);
    statistics[servo_id].total_movement += movement;
}

void ServoControl::calculate_smooth_movement(uint8_t servo_id, float& next_angle, float dt) {
    // Simple trapezoidal motion profile
    float current_angle = servo_status[servo_id].current_angle;
    float target_angle = servo_status[servo_id].target_angle;
    float max_speed = servo_status[servo_id].current_speed;
    
    float angle_diff = target_angle - current_angle;
    float direction = (angle_diff > 0) ? 1.0f : -1.0f;
    
    // Acceleration/deceleration distance (assume 30% of max speed for accel)
    float accel_speed = max_speed * 0.3f;
    float remaining_distance = fabs(angle_diff);
    
    float target_speed;
    if (remaining_distance > 10.0f) { // Far from target
        target_speed = max_speed;
    } else if (remaining_distance > 2.0f) { // Near target, start decelerating
        target_speed = max_speed * (remaining_distance / 10.0f);
    } else { // Very close, move slowly
        target_speed = accel_speed;
    }
    
    float move_distance = target_speed * dt;
    next_angle = current_angle + direction * move_distance;
    
    // Don't overshoot
    if (direction > 0 && next_angle > target_angle) {
        next_angle = target_angle;
    } else if (direction < 0 && next_angle < target_angle) {
        next_angle = target_angle;
    }
}

bool ServoControl::check_position_limits(uint8_t servo_id, float angle) {
    const ServoConfig& config = servos[servo_id];
    return (angle >= config.min_angle && angle <= config.max_angle);
}

float ServoControl::constrain_speed(uint8_t servo_id, float speed) {
    float max_speed = min(servos[servo_id].max_speed, global_speed_limit);
    return constrain(speed, 0.0f, max_speed);
}

void ServoControl::update_trajectory(uint8_t servo_id) {
    // Trajectory implementation would go here
    // This is a simplified placeholder
}

void ServoControl::update_servo_groups() {
    // Multi-servo group coordination would be implemented here
}

void ServoControl::check_servo_faults(uint8_t servo_id) {
    // Check for move timeout
    if (is_move_timeout(servo_id)) {
        handle_servo_fault(servo_id, SERVO_ERROR_TIMEOUT);
    }
    
    // Check for feedback faults
    if (servos[servo_id].enable_feedback) {
        float feedback_voltage = servo_status[servo_id].feedback_voltage;
        if (feedback_voltage < 0.1f || feedback_voltage > 4.9f) {
            handle_servo_fault(servo_id, SERVO_ERROR_FEEDBACK);
        }
    }
}

void ServoControl::handle_servo_fault(uint8_t servo_id, uint8_t fault_code) {
    servo_status[servo_id].fault_code = fault_code;
    servo_status[servo_id].status_flags |= SERVO_STATUS_FAULT;
    statistics[servo_id].fault_count++;
    
    // Stop the servo
    stop_servo(servo_id);
    
    if (fault_callback) {
        fault_callback(servo_id, fault_code);
    }
}

bool ServoControl::is_move_timeout(uint8_t servo_id) {
    if (!servo_status[servo_id].moving) {
        return false;
    }
    
    uint32_t move_time = millis() - servo_status[servo_id].move_start_time;
    return move_time > SERVO_MOVE_TIMEOUT_MS;
}

bool ServoControl::is_valid_servo(uint8_t servo_id) {
    return (servo_id < SERVO_MAX_CHANNELS && servos[servo_id].pwm_pin > 0);
}

float ServoControl::angle_to_pulse_width(uint8_t servo_id, float angle) {
    const ServoConfig& config = servos[servo_id];
    
    // Reverse direction if configured
    if (config.reverse_direction) {
        angle = config.max_angle - (angle - config.min_angle);
    }
    
    // Normalize angle to 0-1 range
    float normalized = (angle - config.min_angle) / (config.max_angle - config.min_angle);
    
    // Map to pulse width
    return config.min_pulse_us + normalized * (config.max_pulse_us - config.min_pulse_us);
}

float ServoControl::pulse_width_to_angle(uint8_t servo_id, uint16_t pulse_width_us) {
    const ServoConfig& config = servos[servo_id];
    
    // Normalize pulse width to 0-1 range
    float normalized = (pulse_width_us - config.min_pulse_us) / 
                      (float)(config.max_pulse_us - config.min_pulse_us);
    
    // Map to angle
    float angle = config.min_angle + normalized * (config.max_angle - config.min_angle);
    
    // Reverse direction if configured
    if (config.reverse_direction) {
        angle = config.max_angle - (angle - config.min_angle);
    }
    
    return angle;
}

void ServoControl::update_servo_statistics(uint8_t servo_id) {
    ServoStatistics& stats = statistics[servo_id];
    
    // Update max speed achieved
    float current_speed = servo_status[servo_id].current_speed;
    if (current_speed > stats.max_speed_achieved) {
        stats.max_speed_achieved = current_speed;
    }
    
    // Update total move time
    if (servo_status[servo_id].moving) {
        stats.total_move_time += update_interval_ms;
    }
}

void ServoControl::mark_movement_complete(uint8_t servo_id, bool success) {
    uint32_t move_time = millis() - servo_status[servo_id].move_start_time;
    
    // Update average settle time
    ServoStatistics& stats = statistics[servo_id];
    stats.average_settle_time = (stats.average_settle_time * (stats.move_commands - 1) + move_time) / stats.move_commands;
    
    if (move_complete_callback) {
        move_complete_callback(servo_id, success);
    }
}

void ServoControl::update_servo_flags(uint8_t servo_id) {
    // Update status flags based on current state
}

void ServoControl::reset_servo_state(uint8_t servo_id) {
    servo_status[servo_id].moving = false;
    servo_status[servo_id].status_flags = servo_status[servo_id].enabled ? SERVO_STATUS_ENABLED : 0;
    servo_status[servo_id].fault_code = SERVO_SUCCESS;
}

// Hardware abstraction layer (platform-specific)
void ServoControl::configure_timer_for_pwm() {
    // SAMD21-specific timer configuration would go here
    // This is a placeholder for the actual hardware setup
}

void ServoControl::enable_timer_interrupt() {
    // Enable timer interrupt for PWM generation
}

void ServoControl::set_timer_compare_value(uint8_t channel, uint16_t value) {
    // Set timer compare value for PWM channel
    // This would interface with SAMD21 TCC modules
}

// Utility functions implementation
namespace ServoUtils {
    ServoConfig create_standard_servo_config(uint8_t pwm_pin, const char* name) {
        ServoConfig config;
        memset(&config, 0, sizeof(ServoConfig));
        
        config.pwm_pin = pwm_pin;
        config.feedback_pin = 255;
        config.enable_pin = 255;
        config.servo_type = SERVO_TYPE_STANDARD;
        config.control_mode = SERVO_MODE_POSITION;
        config.min_pulse_us = SERVO_MIN_PULSE_WIDTH;
        config.max_pulse_us = SERVO_MAX_PULSE_WIDTH;
        config.center_pulse_us = SERVO_CENTER_PULSE_WIDTH;
        config.min_angle = SERVO_MIN_ANGLE;
        config.max_angle = SERVO_MAX_ANGLE;
        config.max_speed = SERVO_DEFAULT_SPEED;
        config.gear_ratio = 1.0f;
        config.reverse_direction = false;
        config.enable_feedback = false;
        strncpy(config.name, name, sizeof(config.name) - 1);
        config.name[sizeof(config.name) - 1] = '\0';
        
        return config;
    }
    
    ServoConfig create_digital_servo_config(uint8_t pwm_pin, uint8_t feedback_pin, const char* name) {
        ServoConfig config = create_standard_servo_config(pwm_pin, name);
        config.servo_type = SERVO_TYPE_DIGITAL;
        config.feedback_pin = feedback_pin;
        config.enable_feedback = true;
        config.min_pulse_us = 600; // Wider range for digital servos
        config.max_pulse_us = 2400;
        config.max_speed = 360.0f; // Faster than standard servos
        
        return config;
    }
    
    ServoConfig create_continuous_servo_config(uint8_t pwm_pin, const char* name) {
        ServoConfig config = create_standard_servo_config(pwm_pin, name);
        config.servo_type = SERVO_TYPE_CONTINUOUS;
        config.control_mode = SERVO_MODE_SPEED;
        config.min_angle = -360.0f; // Speed range
        config.max_angle = 360.0f;
        
        return config;
    }
    
    uint16_t angle_to_pulse_width_standard(float angle) {
        // Standard servo mapping: 0° = 1000μs, 180° = 2000μs
        float normalized = angle / 180.0f;
        return (uint16_t)(SERVO_MIN_PULSE_WIDTH + normalized * (SERVO_MAX_PULSE_WIDTH - SERVO_MIN_PULSE_WIDTH));
    }
    
    float pulse_width_to_angle_standard(uint16_t pulse_width_us) {
        // Standard servo mapping: 1000μs = 0°, 2000μs = 180°
        float normalized = (pulse_width_us - SERVO_MIN_PULSE_WIDTH) / (float)(SERVO_MAX_PULSE_WIDTH - SERVO_MIN_PULSE_WIDTH);
        return normalized * 180.0f;
    }
    
    uint16_t speed_to_pulse_width_continuous(float speed) {
        // Continuous servo mapping: -100% = 1000μs, 0% = 1500μs, +100% = 2000μs
        float normalized = (speed + 100.0f) / 200.0f; // Map -100 to +100 to 0 to 1
        return (uint16_t)(SERVO_MIN_PULSE_WIDTH + normalized * (SERVO_MAX_PULSE_WIDTH - SERVO_MIN_PULSE_WIDTH));
    }
    
    bool validate_servo_config(const ServoConfig& config) {
        // Basic validation
        return (config.pwm_pin > 0 && 
                config.min_pulse_us < config.max_pulse_us &&
                config.min_angle < config.max_angle &&
                config.max_speed > 0.0f &&
                config.max_speed <= 1000.0f);
    }
    
    void print_servo_config(const ServoConfig& config) {
        Serial.println("=== Servo Configuration ===");
        Serial.print("Name: "); Serial.println(config.name);
        Serial.print("PWM Pin: "); Serial.println(config.pwm_pin);
        Serial.print("Feedback Pin: "); Serial.println(config.feedback_pin);
        Serial.print("Enable Pin: "); Serial.println(config.enable_pin);
        Serial.print("Type: "); Serial.println(config.servo_type);
        Serial.print("Pulse Range: "); Serial.print(config.min_pulse_us); 
        Serial.print(" - "); Serial.print(config.max_pulse_us); Serial.println(" μs");
        Serial.print("Angle Range: "); Serial.print(config.min_angle); 
        Serial.print(" - "); Serial.print(config.max_angle); Serial.println("°");
        Serial.print("Max Speed: "); Serial.print(config.max_speed); Serial.println("°/s");
        Serial.print("Reverse Direction: "); Serial.println(config.reverse_direction ? "YES" : "NO");
    }
}