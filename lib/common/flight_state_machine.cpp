#include "flight_state_machine.h"

// Default configuration values
const StateMachineConfig DEFAULT_FSM_CONFIG = {
    // Launch detection
    .launch_accel_threshold = 20.0f,      // 20 m/s²
    .launch_altitude_threshold = 10.0f,   // 10 meters
    .launch_time_threshold = 500,         // 500 ms
    
    // Burnout detection
    .burnout_accel_threshold = 5.0f,      // 5 m/s²
    .burnout_time_threshold = 1000,       // 1 second
    
    // Apogee detection
    .apogee_velocity_threshold = 2.0f,    // 2 m/s
    .apogee_altitude_delta = 5.0f,        // 5 meters
    .apogee_time_threshold = 1000,        // 1 second
    
    // Drogue deployment
    .drogue_delay = 1000,                 // 1 second after apogee
    .drogue_altitude = 500.0f,            // 500m backup
    
    // Main chute deployment
    .main_altitude = 150.0f,              // 150m AGL
    .main_timeout = 60000,                // 60 seconds max
    
    // Landing detection
    .landing_velocity_threshold = 3.0f,   // 3 m/s
    .landing_altitude_delta = 2.0f,       // 2 meters
    .landing_time_threshold = 5000,       // 5 seconds
    
    // Safety timeouts
    .max_flight_time = 300000,            // 5 minutes
    .max_boost_time = 10000,              // 10 seconds
    .max_coast_time = 120000              // 2 minutes
};

FlightStateMachine::FlightStateMachine() {
    current_state = STATE_IDLE;
    previous_state = STATE_IDLE;
    state_entry_time = 0;
    flight_start_time = 0;
    armed = false;
    
    config = DEFAULT_FSM_CONFIG;
    
    ground_altitude = 0.0f;
    max_altitude = 0.0f;
    launch_detected = false;
    burnout_detected = false;
    apogee_detected = false;
    drogue_deployed = false;
    main_deployed = false;
    
    launch_detect_start = 0;
    burnout_detect_start = 0;
    apogee_detect_start = 0;
    landing_detect_start = 0;
    
    // Initialize state handlers
    state_handlers[STATE_IDLE] = [](FlightStateMachine* fsm, const SensorData& sensors) {
        fsm->handle_idle(sensors);
    };
    state_handlers[STATE_ARMED] = [](FlightStateMachine* fsm, const SensorData& sensors) {
        fsm->handle_armed(sensors);
    };
    state_handlers[STATE_BOOST] = [](FlightStateMachine* fsm, const SensorData& sensors) {
        fsm->handle_boost(sensors);
    };
    state_handlers[STATE_COAST] = [](FlightStateMachine* fsm, const SensorData& sensors) {
        fsm->handle_coast(sensors);
    };
    state_handlers[STATE_APOGEE] = [](FlightStateMachine* fsm, const SensorData& sensors) {
        fsm->handle_apogee(sensors);
    };
    state_handlers[STATE_DROGUE] = [](FlightStateMachine* fsm, const SensorData& sensors) {
        fsm->handle_drogue(sensors);
    };
    state_handlers[STATE_MAIN] = [](FlightStateMachine* fsm, const SensorData& sensors) {
        fsm->handle_main(sensors);
    };
    state_handlers[STATE_LANDED] = [](FlightStateMachine* fsm, const SensorData& sensors) {
        fsm->handle_landed(sensors);
    };
    state_handlers[STATE_ERROR] = [](FlightStateMachine* fsm, const SensorData& sensors) {
        fsm->handle_error(sensors);
    };
}

void FlightStateMachine::begin() {
    current_state = STATE_IDLE;
    state_entry_time = millis();
    reset_detection_timers();
}

void FlightStateMachine::set_config(const StateMachineConfig& cfg) {
    config = cfg;
}

void FlightStateMachine::set_baseline_data(const SensorData& baseline) {
    baseline_data = baseline;
    ground_altitude = baseline.altitude;
}

void FlightStateMachine::update(const SensorData& sensors) {
    current_data = sensors;
    update_max_altitude(sensors.altitude);
    
    // Check for safety timeouts
    if (check_safety_timeouts()) {
        transition_to_state(STATE_ERROR);
        return;
    }
    
    // Call current state handler
    if (state_handlers[current_state]) {
        state_handlers[current_state](this, sensors);
    }
}

bool FlightStateMachine::arm_system() {
    if (current_state == STATE_IDLE) {
        armed = true;
        transition_to_state(STATE_ARMED);
        return true;
    }
    return false;
}

bool FlightStateMachine::disarm_system() {
    if (current_state == STATE_ARMED) {
        armed = false;
        transition_to_state(STATE_IDLE);
        return true;
    }
    return false;
}

void FlightStateMachine::force_state(FlightState new_state) {
    transition_to_state(new_state);
}

void FlightStateMachine::trigger_event(StateTrigger trigger) {
    switch (trigger) {
        case TRIGGER_ARM:
            arm_system();
            break;
        case TRIGGER_ERROR:
            transition_to_state(STATE_ERROR);
            break;
        case TRIGGER_MANUAL_OVERRIDE:
            // Handle manual override logic
            break;
        default:
            break;
    }
}

uint32_t FlightStateMachine::get_flight_time() const {
    if (flight_start_time > 0) {
        return millis() - flight_start_time;
    }
    return 0;
}

bool FlightStateMachine::is_in_flight() const {
    return current_state >= STATE_BOOST && current_state <= STATE_MAIN;
}

bool FlightStateMachine::should_deploy_drogue() const {
    return current_state == STATE_APOGEE && !drogue_deployed;
}

bool FlightStateMachine::should_deploy_main() const {
    return (current_state == STATE_DROGUE || current_state == STATE_MAIN) && 
           !main_deployed && 
           get_altitude_agl() <= config.main_altitude;
}

float FlightStateMachine::get_altitude_agl() const {
    return current_data.altitude - ground_altitude;
}

void FlightStateMachine::transition_to_state(FlightState new_state) {
    if (new_state != current_state) {
        previous_state = current_state;
        current_state = new_state;
        state_entry_time = millis();
        
        // State entry actions
        switch (new_state) {
            case STATE_BOOST:
                if (flight_start_time == 0) {
                    flight_start_time = millis();
                }
                launch_detected = true;
                break;
            case STATE_APOGEE:
                apogee_detected = true;
                break;
            case STATE_DROGUE:
                drogue_deployed = true;
                break;
            case STATE_MAIN:
                main_deployed = true;
                break;
            default:
                break;
        }
        
        reset_detection_timers();
    }
}

// State handlers implementation
void FlightStateMachine::handle_idle(const SensorData& sensors) {
    // In idle state, just wait for arming
    // System can be armed manually or through ground command
}

void FlightStateMachine::handle_armed(const SensorData& sensors) {
    // Check for launch detection
    if (check_launch_conditions(sensors)) {
        transition_to_state(STATE_BOOST);
    }
}

void FlightStateMachine::handle_boost(const SensorData& sensors) {
    // Check for burnout (end of boost phase)
    if (check_burnout_conditions(sensors)) {
        transition_to_state(STATE_COAST);
    }
}

void FlightStateMachine::handle_coast(const SensorData& sensors) {
    // Check for apogee detection
    if (check_apogee_conditions(sensors)) {
        transition_to_state(STATE_APOGEE);
    }
}

void FlightStateMachine::handle_apogee(const SensorData& sensors) {
    // Deploy drogue after delay or immediately
    if (get_state_time() >= config.drogue_delay) {
        transition_to_state(STATE_DROGUE);
    }
}

void FlightStateMachine::handle_drogue(const SensorData& sensors) {
    // Check for main chute deployment altitude
    if (get_altitude_agl() <= config.main_altitude || 
        get_state_time() >= config.main_timeout) {
        transition_to_state(STATE_MAIN);
    }
}

void FlightStateMachine::handle_main(const SensorData& sensors) {
    // Check for landing
    if (check_landing_conditions(sensors)) {
        transition_to_state(STATE_LANDED);
    }
}

void FlightStateMachine::handle_landed(const SensorData& sensors) {
    // Flight complete, system can be disarmed
    armed = false;
}

void FlightStateMachine::handle_error(const SensorData& sensors) {
    // Error state - deploy all recovery systems
    if (!drogue_deployed) {
        drogue_deployed = true;
    }
    if (!main_deployed && get_altitude_agl() <= config.main_altitude) {
        main_deployed = true;
    }
}

// Condition checking functions
bool FlightStateMachine::check_launch_conditions(const SensorData& sensors) {
    bool accel_condition = sensors.acceleration > config.launch_accel_threshold;
    bool altitude_condition = (sensors.altitude - ground_altitude) > config.launch_altitude_threshold;
    
    if (accel_condition && altitude_condition) {
        if (launch_detect_start == 0) {
            launch_detect_start = millis();
        }
        return (millis() - launch_detect_start) >= config.launch_time_threshold;
    } else {
        launch_detect_start = 0;
        return false;
    }
}

bool FlightStateMachine::check_burnout_conditions(const SensorData& sensors) {
    bool accel_condition = sensors.acceleration < config.burnout_accel_threshold;
    
    if (accel_condition) {
        if (burnout_detect_start == 0) {
            burnout_detect_start = millis();
        }
        return (millis() - burnout_detect_start) >= config.burnout_time_threshold;
    } else {
        burnout_detect_start = 0;
        return false;
    }
}

bool FlightStateMachine::check_apogee_conditions(const SensorData& sensors) {
    bool velocity_condition = abs(sensors.velocity) < config.apogee_velocity_threshold;
    
    if (velocity_condition) {
        if (apogee_detect_start == 0) {
            apogee_detect_start = millis();
        }
        return (millis() - apogee_detect_start) >= config.apogee_time_threshold;
    } else {
        apogee_detect_start = 0;
        return false;
    }
}

bool FlightStateMachine::check_landing_conditions(const SensorData& sensors) {
    bool velocity_condition = abs(sensors.velocity) < config.landing_velocity_threshold;
    bool altitude_stable = abs(sensors.altitude - ground_altitude) < config.landing_altitude_delta;
    
    if (velocity_condition && altitude_stable) {
        if (landing_detect_start == 0) {
            landing_detect_start = millis();
        }
        return (millis() - landing_detect_start) >= config.landing_time_threshold;
    } else {
        landing_detect_start = 0;
        return false;
    }
}

bool FlightStateMachine::check_safety_timeouts() {
    uint32_t flight_time = get_flight_time();
    uint32_t state_time = get_state_time();
    
    // Overall flight timeout
    if (flight_time > config.max_flight_time) {
        return true;
    }
    
    // State-specific timeouts
    switch (current_state) {
        case STATE_BOOST:
            return state_time > config.max_boost_time;
        case STATE_COAST:
            return state_time > config.max_coast_time;
        default:
            return false;
    }
}

void FlightStateMachine::reset_detection_timers() {
    launch_detect_start = 0;
    burnout_detect_start = 0;
    apogee_detect_start = 0;
    landing_detect_start = 0;
}

void FlightStateMachine::update_max_altitude(float altitude) {
    if (altitude > max_altitude) {
        max_altitude = altitude;
    }
}

const char* FlightStateMachine::state_to_string(FlightState state) {
    switch (state) {
        case STATE_IDLE: return "IDLE";
        case STATE_ARMED: return "ARMED";
        case STATE_BOOST: return "BOOST";
        case STATE_COAST: return "COAST";
        case STATE_APOGEE: return "APOGEE";
        case STATE_DROGUE: return "DROGUE";
        case STATE_MAIN: return "MAIN";
        case STATE_LANDED: return "LANDED";
        case STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}