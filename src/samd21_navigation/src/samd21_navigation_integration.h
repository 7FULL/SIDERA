#ifndef SAMD21_NAVIGATION_INTEGRATION_H
#define SAMD21_NAVIGATION_INTEGRATION_H

#include <Arduino.h>
#include "../../lib/drivers/bno055_imu.h"
#include "../../lib/drivers/servo_control.h"
#include "../../lib/common/spi_protocol.h"

// SAMD21G18A-AU Navigation Controller Integration
// Manages IMU sensor and servo control for flight surfaces

// Pin definitions for SAMD21 Navigation
#define NAV_IMU_SDA_PIN           20    // BNO055 I2C SDA
#define NAV_IMU_SCL_PIN           21    // BNO055 I2C SCL
#define NAV_IMU_RESET_PIN         7     // BNO055 Reset pin
#define NAV_IMU_INT_PIN           6     // BNO055 Interrupt pin

// Servo PWM pins
#define NAV_SERVO_1_PIN           2     // Aileron/Roll control
#define NAV_SERVO_2_PIN           3     // Elevator/Pitch control
#define NAV_SERVO_3_PIN           4     // Rudder/Yaw control
#define NAV_SERVO_4_PIN           5     // Canard/Auxiliary control

// SPI communication with master
#define NAV_SPI_MOSI_PIN          11    // SPI MOSI
#define NAV_SPI_MISO_PIN          12    // SPI MISO
#define NAV_SPI_SCK_PIN           13    // SPI SCK
#define NAV_SPI_CS_PIN            10    // SPI CS (slave select)

// Status LED pins
#define NAV_LED_RED_PIN           8     // Status LED Red
#define NAV_LED_GREEN_PIN         9     // Status LED Green
#define NAV_LED_BLUE_PIN          14    // Status LED Blue

// Update intervals
#define NAV_IMU_UPDATE_RATE       10    // 10ms for 100Hz IMU update
#define NAV_SERVO_UPDATE_RATE     20    // 20ms for 50Hz servo update
#define NAV_CONTROL_UPDATE_RATE   50    // 50ms for control calculations

// Control parameters
#define NAV_MAX_ROLL_ANGLE        45.0f // Maximum roll angle (degrees)
#define NAV_MAX_PITCH_ANGLE       30.0f // Maximum pitch angle (degrees)
#define NAV_MAX_YAW_RATE          90.0f // Maximum yaw rate (deg/s)
#define NAV_SERVO_CENTER_ANGLE    90.0f // Servo center position (degrees)
#define NAV_SERVO_MAX_DEFLECTION  30.0f // Maximum servo deflection (degrees)

// Navigation data structures
struct NavigationData {
    // IMU sensor data
    struct {
        float accel_x, accel_y, accel_z;    // Linear acceleration (m/s²)
        float gyro_x, gyro_y, gyro_z;       // Angular velocity (rad/s)
        float mag_x, mag_y, mag_z;          // Magnetometer (μT)
        float quaternion[4];                // Orientation quaternion (w,x,y,z)
        float euler_roll, euler_pitch, euler_yaw; // Euler angles (degrees)
        float linear_accel_x, linear_accel_y, linear_accel_z; // Gravity-compensated accel
        uint8_t calibration_status;         // Calibration status
        bool data_valid;
        uint32_t timestamp;
    } imu;
    
    // Calculated navigation state
    struct {
        float roll, pitch, yaw;             // Current attitude (degrees)
        float roll_rate, pitch_rate, yaw_rate; // Angular rates (deg/s)
        float vertical_velocity;            // Vertical velocity (m/s)
        float acceleration_magnitude;       // Total acceleration (g)
        bool stable_flight;                 // Flight stability indicator
        uint32_t timestamp;
    } state;
    
    // Servo positions
    struct {
        float servo_1_angle;                // Aileron/Roll servo (degrees)
        float servo_2_angle;                // Elevator/Pitch servo (degrees)
        float servo_3_angle;                // Rudder/Yaw servo (degrees)
        float servo_4_angle;                // Auxiliary servo (degrees)
        bool servos_enabled;
        uint32_t timestamp;
    } servos;
};

// Control modes
enum NavigationMode {
    NAV_MODE_STABILIZE = 0,     // Attitude stabilization
    NAV_MODE_MANUAL = 1,        // Manual servo control
    NAV_MODE_AUTO = 2,          // Autonomous navigation
    NAV_MODE_FAILSAFE = 3,      // Failsafe mode (center servos)
    NAV_MODE_TEST = 4           // Test mode
};

// Flight phase for different control strategies
enum FlightPhase {
    PHASE_PRE_LAUNCH = 0,
    PHASE_BOOST = 1,
    PHASE_COAST = 2,
    PHASE_APOGEE = 3,
    PHASE_DESCENT = 4,
    PHASE_RECOVERY = 5
};

class SAMD21NavigationIntegration {
private:
    // Hardware instances
    BNO055_IMU* imu;
    ServoControl* servo_controller;
    SPISlave* spi_slave;
    
    // Navigation data
    NavigationData nav_data;
    NavigationMode current_mode;
    FlightPhase current_phase;
    
    // Control parameters
    struct ControlGains {
        float roll_kp, roll_ki, roll_kd;       // Roll PID gains
        float pitch_kp, pitch_ki, pitch_kd;    // Pitch PID gains
        float yaw_kp, yaw_ki, yaw_kd;          // Yaw PID gains
    } control_gains;
    
    // PID controllers state
    struct PIDState {
        float roll_integral, roll_derivative, roll_last_error;
        float pitch_integral, pitch_derivative, pitch_last_error;
        float yaw_integral, yaw_derivative, yaw_last_error;
        uint32_t last_update_time;
    } pid_state;
    
    // Target setpoints
    struct TargetSetpoints {
        float target_roll;          // Target roll angle (degrees)
        float target_pitch;         // Target pitch angle (degrees)
        float target_yaw_rate;      // Target yaw rate (deg/s)
        bool setpoints_valid;
        uint32_t setpoint_timestamp;
    } targets;
    
    // System status
    bool initialized;
    bool imu_calibrated;
    bool servos_armed;
    uint32_t last_imu_update;
    uint32_t last_servo_update;
    uint32_t last_control_update;
    uint32_t last_master_communication;
    
    // Safety limits
    float max_roll_error;
    float max_pitch_error;
    uint32_t max_communication_timeout;
    
    // Callbacks
    void (*attitude_callback)(float roll, float pitch, float yaw);
    void (*servo_callback)(float* servo_angles);
    void (*error_callback)(const char* error_msg);
    
public:
    SAMD21NavigationIntegration();
    ~SAMD21NavigationIntegration();
    
    // Initialization
    int8_t begin();
    int8_t initialize_imu();
    int8_t initialize_servos();
    int8_t calibrate_imu();
    bool is_initialized() const { return initialized; }
    
    // Main update function
    void update();
    
    // Navigation data access
    const NavigationData& get_navigation_data() const { return nav_data; }
    
    // Attitude access
    float get_roll() const { return nav_data.state.roll; }
    float get_pitch() const { return nav_data.state.pitch; }
    float get_yaw() const { return nav_data.state.yaw; }
    float get_vertical_acceleration() const;
    bool is_flight_stable() const { return nav_data.state.stable_flight; }
    
    // Control mode management
    int8_t set_navigation_mode(NavigationMode mode);
    NavigationMode get_navigation_mode() const { return current_mode; }
    int8_t set_flight_phase(FlightPhase phase);
    FlightPhase get_flight_phase() const { return current_phase; }
    
    // Servo control
    int8_t arm_servos();
    int8_t disarm_servos();
    bool are_servos_armed() const { return servos_armed; }
    int8_t set_servo_angle(uint8_t servo_id, float angle);
    int8_t center_all_servos();
    int8_t move_servos_to_failsafe();
    
    // Manual control setpoints
    int8_t set_target_attitude(float roll, float pitch, float yaw_rate);
    int8_t set_manual_servo_positions(float* servo_angles);
    
    // Control tuning
    int8_t set_control_gains(float roll_kp, float roll_ki, float roll_kd,
                            float pitch_kp, float pitch_ki, float pitch_kd,
                            float yaw_kp, float yaw_ki, float yaw_kd);
    int8_t load_control_gains_preset(const char* preset_name);
    
    // IMU calibration and management
    int8_t start_imu_calibration();
    uint8_t get_imu_calibration_status() const;
    bool is_imu_calibrated() const { return imu_calibrated; }
    int8_t save_imu_calibration();
    int8_t load_imu_calibration();
    
    // Communication with master
    int8_t process_master_commands();
    int8_t send_navigation_data_to_master();
    
    // Safety and failsafe
    int8_t check_safety_limits();
    int8_t activate_failsafe();
    bool is_communication_timeout() const;
    
    // Test and diagnostics
    int8_t run_servo_test();
    int8_t run_imu_test();
    void print_navigation_status() const;
    void print_servo_positions() const;
    void set_status_led(uint8_t red, uint8_t green, uint8_t blue);
    
    // Configuration
    int8_t save_navigation_config();
    int8_t load_navigation_config();
    
    // Callbacks
    void set_attitude_callback(void (*callback)(float roll, float pitch, float yaw));
    void set_servo_callback(void (*callback)(float* servo_angles));
    void set_error_callback(void (*callback)(const char* error_msg));
    
private:
    // Update functions
    void update_imu_data();
    void update_navigation_state();
    void update_control_system();
    void update_servo_outputs();
    void update_master_communication();
    
    // Control algorithms
    void stabilize_attitude();
    void manual_control();
    void autonomous_navigation();
    void failsafe_control();
    
    // PID control
    float calculate_pid_output(float error, float dt, 
                              float kp, float ki, float kd,
                              float& integral, float& derivative, float& last_error);
    void reset_pid_controllers();
    void limit_integral_windup();
    
    // Navigation calculations
    void calculate_attitude_from_imu();
    void calculate_angular_rates();
    void detect_flight_stability();
    void apply_sensor_fusion();
    
    // Servo mapping
    float map_roll_to_aileron(float roll_command);
    float map_pitch_to_elevator(float pitch_command);
    float map_yaw_to_rudder(float yaw_command);
    void apply_servo_limits(float& servo_angle);
    
    // Safety checks
    bool check_attitude_limits();
    bool check_servo_limits();
    bool check_imu_health();
    
    // Communication protocol
    void handle_spi_received_data(const uint8_t* data, uint16_t length);
    int8_t send_spi_response(uint8_t response_type, const void* data, uint16_t length);
    
    // Utility functions
    void initialize_pins();
    void configure_interrupts();
    float constrain_angle(float angle, float min_val, float max_val);
    float wrap_angle_180(float angle);
};

// Global instance
extern SAMD21NavigationIntegration* g_samd21_navigation;

// Utility functions and helpers
namespace NavigationUtils {
    // Attitude calculations
    void quaternion_to_euler(float qw, float qx, float qy, float qz, 
                            float& roll, float& pitch, float& yaw);
    void euler_to_quaternion(float roll, float pitch, float yaw,
                            float& qw, float& qx, float& qy, float& qz);
    
    // Coordinate transformations
    void body_to_earth_rotation(float roll, float pitch, float yaw,
                               float body_x, float body_y, float body_z,
                               float& earth_x, float& earth_y, float& earth_z);
    
    // Control gain tuning
    struct ControlTuning {
        float roll_kp, roll_ki, roll_kd;
        float pitch_kp, pitch_ki, pitch_kd;
        float yaw_kp, yaw_ki, yaw_kd;
        const char* name;
    };
    
    extern const ControlTuning CONSERVATIVE_GAINS;
    extern const ControlTuning AGGRESSIVE_GAINS;
    extern const ControlTuning ROCKET_GAINS;
    extern const ControlTuning TEST_GAINS;
    
    // Flight phase detection
    FlightPhase detect_flight_phase(float acceleration, float altitude, float velocity);
    const char* flight_phase_to_string(FlightPhase phase);
    const char* navigation_mode_to_string(NavigationMode mode);
    
    // Servo calibration
    struct ServoCalibration {
        float min_angle;
        float max_angle;
        float center_offset;
        float scale_factor;
    };
    
    int8_t calibrate_servo_range(uint8_t servo_id, ServoCalibration& calibration);
    int8_t apply_servo_calibration(uint8_t servo_id, const ServoCalibration& calibration);
    
    // Data validation
    bool validate_attitude(float roll, float pitch, float yaw);
    bool validate_angular_rates(float roll_rate, float pitch_rate, float yaw_rate);
    bool validate_servo_commands(float* servo_angles, uint8_t num_servos);
    
    // Performance analysis
    struct ControlPerformance {
        float roll_rms_error;
        float pitch_rms_error;
        float yaw_rms_error;
        float settling_time;
        float overshoot_percent;
        bool stable;
    };
    
    ControlPerformance analyze_control_performance(const float* attitude_log, 
                                                  const float* target_log,
                                                  uint32_t num_samples,
                                                  float sample_rate);
    
    // Configuration helpers
    void create_default_navigation_config();
    int8_t validate_navigation_config();
    void print_navigation_config();
}

#endif // SAMD21_NAVIGATION_INTEGRATION_H