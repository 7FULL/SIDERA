#include "samd21_navigation_integration.h"

// Global instance
SAMD21NavigationIntegration* g_samd21_navigation = nullptr;

SAMD21NavigationIntegration::SAMD21NavigationIntegration() :
    imu(nullptr),
    servo_controller(nullptr),
    spi_slave(nullptr),
    current_mode(NAV_MODE_STABILIZE),
    current_phase(PHASE_PRE_LAUNCH),
    initialized(false),
    imu_calibrated(false),
    servos_armed(false),
    last_imu_update(0),
    last_servo_update(0),
    last_control_update(0),
    last_master_communication(0),
    max_roll_error(45.0f),
    max_pitch_error(30.0f),
    max_communication_timeout(5000),
    attitude_callback(nullptr),
    servo_callback(nullptr),
    error_callback(nullptr)
{
    memset(&nav_data, 0, sizeof(nav_data));
    memset(&control_gains, 0, sizeof(control_gains));
    memset(&pid_state, 0, sizeof(pid_state));
    memset(&targets, 0, sizeof(targets));
    
    // Initialize with default control gains
    control_gains.roll_kp = 2.0f;
    control_gains.roll_ki = 0.1f;
    control_gains.roll_kd = 0.5f;
    control_gains.pitch_kp = 2.0f;
    control_gains.pitch_ki = 0.1f;
    control_gains.pitch_kd = 0.5f;
    control_gains.yaw_kp = 1.0f;
    control_gains.yaw_ki = 0.05f;
    control_gains.yaw_kd = 0.2f;
}

SAMD21NavigationIntegration::~SAMD21NavigationIntegration()
{
    if (imu) delete imu;
    if (servo_controller) delete servo_controller;
    if (spi_slave) delete spi_slave;
}

int8_t SAMD21NavigationIntegration::begin()
{
    initialize_pins();
    configure_interrupts();
    
    if (initialize_imu() != 0) {
        return -1;
    }
    
    if (initialize_servos() != 0) {
        return -2;
    }
    
    // Initialize SPI slave communication
    spi_slave = new SPISlave();
    if (spi_slave->begin(NAV_SPI_CS_PIN) != 0) {
        return -3;
    }
    
    initialized = true;
    
    // Start IMU calibration
    start_imu_calibration();
    
    return 0;
}

int8_t SAMD21NavigationIntegration::initialize_imu()
{
    imu = new BNO055_IMU();
    
    if (imu->begin(NAV_IMU_SDA_PIN, NAV_IMU_SCL_PIN, NAV_IMU_RESET_PIN) != 0) {
        if (error_callback) {
            error_callback("IMU initialization failed");
        }
        return -1;
    }
    
    // Configure IMU for fusion mode
    if (imu->set_operation_mode(BNO055_NDOF_MODE) != 0) {
        return -2;
    }
    
    // Set units
    if (imu->set_units(BNO055_UNIT_ACCEL_MS2, BNO055_UNIT_GYRO_DPS, BNO055_UNIT_EULER_DEGREES) != 0) {
        return -3;
    }
    
    return 0;
}

int8_t SAMD21NavigationIntegration::initialize_servos()
{
    servo_controller = new ServoControl();
    
    if (servo_controller->begin() != 0) {
        if (error_callback) {
            error_callback("Servo controller initialization failed");
        }
        return -1;
    }
    
    // Add servo channels
    if (servo_controller->add_servo(0, NAV_SERVO_1_PIN) != 0) return -2; // Aileron
    if (servo_controller->add_servo(1, NAV_SERVO_2_PIN) != 0) return -2; // Elevator
    if (servo_controller->add_servo(2, NAV_SERVO_3_PIN) != 0) return -2; // Rudder
    if (servo_controller->add_servo(3, NAV_SERVO_4_PIN) != 0) return -2; // Canard
    
    // Set servo limits and center positions
    for (int i = 0; i < 4; i++) {
        servo_controller->set_servo_limits(i, 
            NAV_SERVO_CENTER_ANGLE - NAV_SERVO_MAX_DEFLECTION,
            NAV_SERVO_CENTER_ANGLE + NAV_SERVO_MAX_DEFLECTION);
        servo_controller->set_servo_position(i, NAV_SERVO_CENTER_ANGLE);
    }
    
    return 0;
}

int8_t SAMD21NavigationIntegration::calibrate_imu()
{
    if (!imu) return -1;
    
    return imu->calibrate_sensors();
}

void SAMD21NavigationIntegration::update()
{
    uint32_t current_time = millis();
    
    // Update IMU data (100Hz)
    if (current_time - last_imu_update >= NAV_IMU_UPDATE_RATE) {
        update_imu_data();
        update_navigation_state();
        last_imu_update = current_time;
    }
    
    // Update control system (20Hz)
    if (current_time - last_control_update >= NAV_CONTROL_UPDATE_RATE) {
        update_control_system();
        last_control_update = current_time;
    }
    
    // Update servo outputs (50Hz)
    if (current_time - last_servo_update >= NAV_SERVO_UPDATE_RATE) {
        update_servo_outputs();
        last_servo_update = current_time;
    }
    
    // Update master communication
    update_master_communication();
    
    // Check safety limits
    check_safety_limits();
}

void SAMD21NavigationIntegration::update_imu_data()
{
    if (!imu || !imu->is_data_ready()) return;
    
    IMUData imu_data;
    if (imu->read_sensor_data(imu_data) == 0) {
        // Copy accelerometer data
        nav_data.imu.accel_x = imu_data.accel_x;
        nav_data.imu.accel_y = imu_data.accel_y;
        nav_data.imu.accel_z = imu_data.accel_z;
        
        // Copy gyroscope data
        nav_data.imu.gyro_x = imu_data.gyro_x * PI / 180.0f; // Convert to rad/s
        nav_data.imu.gyro_y = imu_data.gyro_y * PI / 180.0f;
        nav_data.imu.gyro_z = imu_data.gyro_z * PI / 180.0f;
        
        // Copy magnetometer data
        nav_data.imu.mag_x = imu_data.mag_x;
        nav_data.imu.mag_y = imu_data.mag_y;
        nav_data.imu.mag_z = imu_data.mag_z;
        
        // Copy quaternion data
        nav_data.imu.quaternion[0] = imu_data.quat_w;
        nav_data.imu.quaternion[1] = imu_data.quat_x;
        nav_data.imu.quaternion[2] = imu_data.quat_y;
        nav_data.imu.quaternion[3] = imu_data.quat_z;
        
        // Copy Euler angles
        nav_data.imu.euler_roll = imu_data.euler_roll;
        nav_data.imu.euler_pitch = imu_data.euler_pitch;
        nav_data.imu.euler_yaw = imu_data.euler_yaw;
        
        // Copy linear acceleration
        nav_data.imu.linear_accel_x = imu_data.linear_accel_x;
        nav_data.imu.linear_accel_y = imu_data.linear_accel_y;
        nav_data.imu.linear_accel_z = imu_data.linear_accel_z;
        
        nav_data.imu.calibration_status = imu->get_calibration_status();
        nav_data.imu.data_valid = true;
        nav_data.imu.timestamp = millis();
        
        // Update calibration status
        imu_calibrated = (nav_data.imu.calibration_status >= 2);
    }
}

void SAMD21NavigationIntegration::update_navigation_state()
{
    calculate_attitude_from_imu();
    calculate_angular_rates();
    detect_flight_stability();
    apply_sensor_fusion();
    
    nav_data.state.timestamp = millis();
}

void SAMD21NavigationIntegration::update_control_system()
{
    switch (current_mode) {
        case NAV_MODE_STABILIZE:
            stabilize_attitude();
            break;
            
        case NAV_MODE_MANUAL:
            manual_control();
            break;
            
        case NAV_MODE_AUTO:
            autonomous_navigation();
            break;
            
        case NAV_MODE_FAILSAFE:
            failsafe_control();
            break;
            
        case NAV_MODE_TEST:
            // Test mode - no automatic control
            break;
    }
}

void SAMD21NavigationIntegration::update_servo_outputs()
{
    if (!servo_controller || !servos_armed) return;
    
    // Apply servo commands
    servo_controller->set_servo_position(0, nav_data.servos.servo_1_angle);
    servo_controller->set_servo_position(1, nav_data.servos.servo_2_angle);
    servo_controller->set_servo_position(2, nav_data.servos.servo_3_angle);
    servo_controller->set_servo_position(3, nav_data.servos.servo_4_angle);
    
    nav_data.servos.timestamp = millis();
    
    // Call servo callback
    if (servo_callback) {
        float servo_angles[4] = {
            nav_data.servos.servo_1_angle,
            nav_data.servos.servo_2_angle,
            nav_data.servos.servo_3_angle,
            nav_data.servos.servo_4_angle
        };
        servo_callback(servo_angles);
    }
}

void SAMD21NavigationIntegration::stabilize_attitude()
{
    if (!nav_data.imu.data_valid || !targets.setpoints_valid) return;
    
    uint32_t current_time = millis();
    float dt = (current_time - pid_state.last_update_time) / 1000.0f;
    
    if (dt <= 0.0f || dt > 0.1f) { // Skip if dt is invalid or too large
        pid_state.last_update_time = current_time;
        return;
    }
    
    // Calculate attitude errors
    float roll_error = targets.target_roll - nav_data.state.roll;
    float pitch_error = targets.target_pitch - nav_data.state.pitch;
    float yaw_rate_error = targets.target_yaw_rate - nav_data.state.yaw_rate;
    
    // Wrap angles to ±180 degrees
    roll_error = wrap_angle_180(roll_error);
    pitch_error = wrap_angle_180(pitch_error);
    
    // Calculate PID outputs
    float roll_output = calculate_pid_output(roll_error, dt,
        control_gains.roll_kp, control_gains.roll_ki, control_gains.roll_kd,
        pid_state.roll_integral, pid_state.roll_derivative, pid_state.roll_last_error);
    
    float pitch_output = calculate_pid_output(pitch_error, dt,
        control_gains.pitch_kp, control_gains.pitch_ki, control_gains.pitch_kd,
        pid_state.pitch_integral, pid_state.pitch_derivative, pid_state.pitch_last_error);
    
    float yaw_output = calculate_pid_output(yaw_rate_error, dt,
        control_gains.yaw_kp, control_gains.yaw_ki, control_gains.yaw_kd,
        pid_state.yaw_integral, pid_state.yaw_derivative, pid_state.yaw_last_error);
    
    // Map control outputs to servo positions
    nav_data.servos.servo_1_angle = map_roll_to_aileron(roll_output);
    nav_data.servos.servo_2_angle = map_pitch_to_elevator(pitch_output);
    nav_data.servos.servo_3_angle = map_yaw_to_rudder(yaw_output);
    nav_data.servos.servo_4_angle = NAV_SERVO_CENTER_ANGLE; // Canard neutral
    
    // Apply servo limits
    apply_servo_limits(nav_data.servos.servo_1_angle);
    apply_servo_limits(nav_data.servos.servo_2_angle);
    apply_servo_limits(nav_data.servos.servo_3_angle);
    apply_servo_limits(nav_data.servos.servo_4_angle);
    
    pid_state.last_update_time = current_time;
}

void SAMD21NavigationIntegration::manual_control()
{
    // In manual mode, servo positions are set directly by master commands
    // No automatic control loops active
}

void SAMD21NavigationIntegration::autonomous_navigation()
{
    // Autonomous navigation based on flight phase
    switch (current_phase) {
        case PHASE_PRE_LAUNCH:
            // Keep control surfaces neutral
            center_all_servos();
            break;
            
        case PHASE_BOOST:
            // Minimal control during boost phase
            stabilize_attitude();
            break;
            
        case PHASE_COAST:
            // Active stabilization during coast
            stabilize_attitude();
            break;
            
        case PHASE_APOGEE:
            // Prepare for descent
            stabilize_attitude();
            break;
            
        case PHASE_DESCENT:
            // Descent stabilization
            stabilize_attitude();
            break;
            
        case PHASE_RECOVERY:
            // Center servos during recovery
            center_all_servos();
            break;
    }
}

void SAMD21NavigationIntegration::failsafe_control()
{
    // Center all servos in failsafe mode
    center_all_servos();
}

float SAMD21NavigationIntegration::calculate_pid_output(float error, float dt,
                                                       float kp, float ki, float kd,
                                                       float& integral, float& derivative, float& last_error)
{
    // Proportional term
    float proportional = kp * error;
    
    // Integral term with windup protection
    integral += error * dt;
    if (integral > 10.0f) integral = 10.0f;
    if (integral < -10.0f) integral = -10.0f;
    float integral_term = ki * integral;
    
    // Derivative term
    derivative = (error - last_error) / dt;
    float derivative_term = kd * derivative;
    
    last_error = error;
    
    return proportional + integral_term + derivative_term;
}

void SAMD21NavigationIntegration::calculate_attitude_from_imu()
{
    if (!nav_data.imu.data_valid) return;
    
    nav_data.state.roll = nav_data.imu.euler_roll;
    nav_data.state.pitch = nav_data.imu.euler_pitch;
    nav_data.state.yaw = nav_data.imu.euler_yaw;
}

void SAMD21NavigationIntegration::calculate_angular_rates()
{
    if (!nav_data.imu.data_valid) return;
    
    nav_data.state.roll_rate = nav_data.imu.gyro_x * 180.0f / PI; // Convert to deg/s
    nav_data.state.pitch_rate = nav_data.imu.gyro_y * 180.0f / PI;
    nav_data.state.yaw_rate = nav_data.imu.gyro_z * 180.0f / PI;
}

void SAMD21NavigationIntegration::detect_flight_stability()
{
    if (!nav_data.imu.data_valid) {
        nav_data.state.stable_flight = false;
        return;
    }
    
    // Simple stability detection based on angular rates
    float max_rate = max(abs(nav_data.state.roll_rate), 
                        max(abs(nav_data.state.pitch_rate), abs(nav_data.state.yaw_rate)));
    
    nav_data.state.stable_flight = (max_rate < 30.0f); // Less than 30 deg/s
}

void SAMD21NavigationIntegration::apply_sensor_fusion()
{
    // Basic sensor fusion - could be enhanced with Kalman filter
    // For now, just use the BNO055's internal fusion
}

float SAMD21NavigationIntegration::map_roll_to_aileron(float roll_command)
{
    // Simple linear mapping from roll command to aileron deflection
    float aileron_angle = NAV_SERVO_CENTER_ANGLE + (roll_command * 0.5f);
    return constrain_angle(aileron_angle, 
                          NAV_SERVO_CENTER_ANGLE - NAV_SERVO_MAX_DEFLECTION,
                          NAV_SERVO_CENTER_ANGLE + NAV_SERVO_MAX_DEFLECTION);
}

float SAMD21NavigationIntegration::map_pitch_to_elevator(float pitch_command)
{
    // Simple linear mapping from pitch command to elevator deflection
    float elevator_angle = NAV_SERVO_CENTER_ANGLE - (pitch_command * 0.5f); // Negative for proper direction
    return constrain_angle(elevator_angle,
                          NAV_SERVO_CENTER_ANGLE - NAV_SERVO_MAX_DEFLECTION,
                          NAV_SERVO_CENTER_ANGLE + NAV_SERVO_MAX_DEFLECTION);
}

float SAMD21NavigationIntegration::map_yaw_to_rudder(float yaw_command)
{
    // Simple linear mapping from yaw command to rudder deflection
    float rudder_angle = NAV_SERVO_CENTER_ANGLE + (yaw_command * 0.3f);
    return constrain_angle(rudder_angle,
                          NAV_SERVO_CENTER_ANGLE - NAV_SERVO_MAX_DEFLECTION,
                          NAV_SERVO_CENTER_ANGLE + NAV_SERVO_MAX_DEFLECTION);
}

void SAMD21NavigationIntegration::apply_servo_limits(float& servo_angle)
{
    servo_angle = constrain_angle(servo_angle,
                                 NAV_SERVO_CENTER_ANGLE - NAV_SERVO_MAX_DEFLECTION,
                                 NAV_SERVO_CENTER_ANGLE + NAV_SERVO_MAX_DEFLECTION);
}

int8_t SAMD21NavigationIntegration::center_all_servos()
{
    nav_data.servos.servo_1_angle = NAV_SERVO_CENTER_ANGLE;
    nav_data.servos.servo_2_angle = NAV_SERVO_CENTER_ANGLE;
    nav_data.servos.servo_3_angle = NAV_SERVO_CENTER_ANGLE;
    nav_data.servos.servo_4_angle = NAV_SERVO_CENTER_ANGLE;
    
    return 0;
}

int8_t SAMD21NavigationIntegration::set_target_attitude(float roll, float pitch, float yaw_rate)
{
    targets.target_roll = constrain_angle(roll, -NAV_MAX_ROLL_ANGLE, NAV_MAX_ROLL_ANGLE);
    targets.target_pitch = constrain_angle(pitch, -NAV_MAX_PITCH_ANGLE, NAV_MAX_PITCH_ANGLE);
    targets.target_yaw_rate = constrain_angle(yaw_rate, -NAV_MAX_YAW_RATE, NAV_MAX_YAW_RATE);
    targets.setpoints_valid = true;
    targets.setpoint_timestamp = millis();
    
    return 0;
}

int8_t SAMD21NavigationIntegration::check_safety_limits()
{
    if (!nav_data.imu.data_valid) return -1;
    
    // Check attitude limits
    if (abs(nav_data.state.roll) > NAV_MAX_ROLL_ANGLE + 10.0f ||
        abs(nav_data.state.pitch) > NAV_MAX_PITCH_ANGLE + 10.0f) {
        activate_failsafe();
        return -2;
    }
    
    // Check communication timeout
    if (millis() - last_master_communication > max_communication_timeout) {
        activate_failsafe();
        return -3;
    }
    
    return 0;
}

int8_t SAMD21NavigationIntegration::activate_failsafe()
{
    current_mode = NAV_MODE_FAILSAFE;
    center_all_servos();
    reset_pid_controllers();
    
    if (error_callback) {
        error_callback("Navigation failsafe activated");
    }
    
    return 0;
}

float SAMD21NavigationIntegration::constrain_angle(float angle, float min_val, float max_val)
{
    if (angle < min_val) return min_val;
    if (angle > max_val) return max_val;
    return angle;
}

float SAMD21NavigationIntegration::wrap_angle_180(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

void SAMD21NavigationIntegration::initialize_pins()
{
    // Initialize status LEDs
    pinMode(NAV_LED_RED_PIN, OUTPUT);
    pinMode(NAV_LED_GREEN_PIN, OUTPUT);
    pinMode(NAV_LED_BLUE_PIN, OUTPUT);
    
    // Initialize SPI pins
    pinMode(NAV_SPI_MOSI_PIN, INPUT);  // Slave MOSI is input
    pinMode(NAV_SPI_MISO_PIN, OUTPUT); // Slave MISO is output
    pinMode(NAV_SPI_SCK_PIN, INPUT);   // Slave SCK is input
    pinMode(NAV_SPI_CS_PIN, INPUT);    // Slave CS is input
    
    // Initialize I2C pins for IMU
    pinMode(NAV_IMU_SDA_PIN, INPUT_PULLUP);
    pinMode(NAV_IMU_SCL_PIN, INPUT_PULLUP);
    pinMode(NAV_IMU_RESET_PIN, OUTPUT);
    pinMode(NAV_IMU_INT_PIN, INPUT);
    
    // Reset IMU
    digitalWrite(NAV_IMU_RESET_PIN, LOW);
    delay(10);
    digitalWrite(NAV_IMU_RESET_PIN, HIGH);
    delay(100);
}

void SAMD21NavigationIntegration::configure_interrupts()
{
    // Configure IMU interrupt if needed
    // attachInterrupt(digitalPinToInterrupt(NAV_IMU_INT_PIN), imu_interrupt_handler, RISING);
}

void SAMD21NavigationIntegration::update_master_communication()
{
    // Process SPI communication with master
    if (spi_slave && spi_slave->is_data_available()) {
        uint8_t received_data[64];
        uint16_t length = spi_slave->read_buffer(received_data, sizeof(received_data));
        
        if (length > 0) {
            handle_spi_received_data(received_data, length);
            last_master_communication = millis();
        }
    }
}

void SAMD21NavigationIntegration::handle_spi_received_data(const uint8_t* data, uint16_t length)
{
    if (length < 2) return;
    
    uint8_t command = data[0];
    
    switch (command) {
        case 0x01: // Set navigation mode
            if (length >= 2) {
                set_navigation_mode((NavigationMode)data[1]);
            }
            break;
            
        case 0x02: // Set flight phase
            if (length >= 2) {
                set_flight_phase((FlightPhase)data[1]);
            }
            break;
            
        case 0x03: // Set target attitude
            if (length >= 13) {
                float roll, pitch, yaw_rate;
                memcpy(&roll, &data[1], sizeof(float));
                memcpy(&pitch, &data[5], sizeof(float));
                memcpy(&yaw_rate, &data[9], sizeof(float));
                set_target_attitude(roll, pitch, yaw_rate);
            }
            break;
            
        case 0x04: // Arm/disarm servos
            if (length >= 2) {
                if (data[1]) {
                    arm_servos();
                } else {
                    disarm_servos();
                }
            }
            break;
            
        case 0x05: // Request navigation data
            send_navigation_data_to_master();
            break;
    }
}

int8_t SAMD21NavigationIntegration::send_navigation_data_to_master()
{
    if (!spi_slave) return -1;
    
    uint8_t response_data[64];
    uint16_t offset = 0;
    
    // Pack navigation data
    response_data[offset++] = 0x01; // Response type: navigation data
    
    memcpy(&response_data[offset], &nav_data.state.roll, sizeof(float));
    offset += sizeof(float);
    memcpy(&response_data[offset], &nav_data.state.pitch, sizeof(float));
    offset += sizeof(float);
    memcpy(&response_data[offset], &nav_data.state.yaw, sizeof(float));
    offset += sizeof(float);
    
    response_data[offset++] = nav_data.imu.calibration_status;
    response_data[offset++] = servos_armed ? 1 : 0;
    response_data[offset++] = (uint8_t)current_mode;
    
    return spi_slave->send_response(response_data, offset);
}

// Namespace implementations
namespace NavigationUtils {
    void quaternion_to_euler(float qw, float qx, float qy, float qz,
                            float& roll, float& pitch, float& yaw)
    {
        // Convert quaternion to Euler angles
        float test = qx * qy + qz * qw;
        
        if (test > 0.499) { // Singularity at north pole
            yaw = 2 * atan2(qx, qw);
            pitch = PI / 2;
            roll = 0;
            return;
        }
        
        if (test < -0.499) { // Singularity at south pole
            yaw = -2 * atan2(qx, qw);
            pitch = -PI / 2;
            roll = 0;
            return;
        }
        
        float sqx = qx * qx;
        float sqy = qy * qy;
        float sqz = qz * qz;
        
        yaw = atan2(2 * qy * qw - 2 * qx * qz, 1 - 2 * sqy - 2 * sqz);
        pitch = asin(2 * test);
        roll = atan2(2 * qx * qw - 2 * qy * qz, 1 - 2 * sqx - 2 * sqz);
        
        // Convert to degrees
        roll *= 180.0f / PI;
        pitch *= 180.0f / PI;
        yaw *= 180.0f / PI;
    }
    
    // Control gain presets
    const ControlTuning CONSERVATIVE_GAINS = {
        .roll_kp = 1.0f, .roll_ki = 0.05f, .roll_kd = 0.2f,
        .pitch_kp = 1.0f, .pitch_ki = 0.05f, .pitch_kd = 0.2f,
        .yaw_kp = 0.5f, .yaw_ki = 0.02f, .yaw_kd = 0.1f,
        .name = "Conservative"
    };
    
    const ControlTuning AGGRESSIVE_GAINS = {
        .roll_kp = 3.0f, .roll_ki = 0.2f, .roll_kd = 0.8f,
        .pitch_kp = 3.0f, .pitch_ki = 0.2f, .pitch_kd = 0.8f,
        .yaw_kp = 2.0f, .yaw_ki = 0.1f, .yaw_kd = 0.4f,
        .name = "Aggressive"
    };
    
    const ControlTuning ROCKET_GAINS = {
        .roll_kp = 2.0f, .roll_ki = 0.1f, .roll_kd = 0.5f,
        .pitch_kp = 2.0f, .pitch_ki = 0.1f, .pitch_kd = 0.5f,
        .yaw_kp = 1.0f, .yaw_ki = 0.05f, .yaw_kd = 0.2f,
        .name = "Rocket"
    };
    
    FlightPhase detect_flight_phase(float acceleration, float altitude, float velocity)
    {
        if (acceleration > 5.0f) return PHASE_BOOST;
        if (velocity > 5.0f && acceleration < 2.0f) return PHASE_COAST;
        if (velocity < 1.0f && velocity > -1.0f) return PHASE_APOGEE;
        if (velocity < -5.0f) return PHASE_DESCENT;
        if (altitude < 100.0f && abs(velocity) < 2.0f) return PHASE_RECOVERY;
        return PHASE_PRE_LAUNCH;
    }
    
    const char* flight_phase_to_string(FlightPhase phase)
    {
        switch (phase) {
            case PHASE_PRE_LAUNCH: return "Pre-Launch";
            case PHASE_BOOST: return "Boost";
            case PHASE_COAST: return "Coast";
            case PHASE_APOGEE: return "Apogee";
            case PHASE_DESCENT: return "Descent";
            case PHASE_RECOVERY: return "Recovery";
            default: return "Unknown";
        }
    }
    
    const char* navigation_mode_to_string(NavigationMode mode)
    {
        switch (mode) {
            case NAV_MODE_STABILIZE: return "Stabilize";
            case NAV_MODE_MANUAL: return "Manual";
            case NAV_MODE_AUTO: return "Auto";
            case NAV_MODE_FAILSAFE: return "Failsafe";
            case NAV_MODE_TEST: return "Test";
            default: return "Unknown";
        }
    }
}