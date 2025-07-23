#include "pyrotechnic_control.h"
#include <string.h>

PyrotechnicControl* g_pyro_control = nullptr;

// Template configurations
namespace PyroUtils {
    namespace Templates {
        const PyroChannelConfig DROGUE_CHANNEL = {
            2,                                    // fire_pin
            A0,                                  // sense_pin
            A1,                                  // continuity_pin
            PYRO_FIRE_DURATION_MS,              // fire_duration_ms
            PYRO_TYPE_DROGUE,                   // channel_type
            PYRO_INTERLOCK_ARM_SWITCH | PYRO_INTERLOCK_CONTINUITY | PYRO_INTERLOCK_VOLTAGE, // interlocks
            PYRO_SAFE_VOLTAGE_MIN,              // min_fire_voltage
            PYRO_SAFE_VOLTAGE_MAX,              // max_fire_voltage
            true,                               // active
            "Drogue"                            // name
        };
        
        const PyroChannelConfig MAIN_CHANNEL = {
            3,                                    // fire_pin
            A2,                                  // sense_pin
            A3,                                  // continuity_pin
            PYRO_FIRE_DURATION_MS,              // fire_duration_ms
            PYRO_TYPE_MAIN,                     // channel_type
            PYRO_INTERLOCK_ARM_SWITCH | PYRO_INTERLOCK_CONTINUITY | PYRO_INTERLOCK_VOLTAGE | PYRO_INTERLOCK_ALTITUDE, // interlocks
            PYRO_SAFE_VOLTAGE_MIN,              // min_fire_voltage
            PYRO_SAFE_VOLTAGE_MAX,              // max_fire_voltage
            true,                               // active
            "Main"                              // name
        };
        
        const PyroSequenceConfig STANDARD_RECOVERY = {
            {
                {0, 0, 0, true, "Drogue deployment"},      // Channel 0, immediate
                {1, 3000, 0, true, "Main deployment"},     // Channel 1, 3 seconds later
                {255, 0, 0, false, ""},                    // Unused
                {255, 0, 0, false, ""},                    // Unused
                {255, 0, 0, false, ""},                    // Unused
                {255, 0, 0, false, ""},                    // Unused
                {255, 0, 0, false, ""},                    // Unused
                {255, 0, 0, false, ""}                     // Unused
            },
            2,                                             // num_steps
            false,                                         // loop_sequence
            0,                                             // loop_delay_ms
            true,                                          // abort_on_failure
            "Standard Recovery"                            // name
        };
    }
}

PyrotechnicControl::PyrotechnicControl(uint8_t master_arm_pin, uint8_t voltage_sense_pin, uint8_t system_enable_pin) {
    this->master_arm_pin = master_arm_pin;
    this->voltage_sense_pin = voltage_sense_pin;
    this->system_enable_pin = system_enable_pin;
    initialized = false;
    
    // Initialize channel configurations
    memset(channels, 0, sizeof(channels));
    memset(channel_status, 0, sizeof(channel_status));
    
    // Initialize system status
    memset(&system_status, 0, sizeof(PyroSystemStatus));
    
    // Initialize sequence control
    memset(&active_sequence, 0, sizeof(PyroSequenceConfig));
    sequence_running = false;
    sequence_start_time = 0;
    current_sequence_step = 0;
    
    // Initialize event logging
    memset(event_log, 0, sizeof(event_log));
    event_index = 0;
    total_events = 0;
    
    // Initialize safety monitoring
    last_safety_check = 0;
    arm_timeout = PYRO_SAFETY_TIMEOUT;
    safety_override = false;
    
    // Initialize statistics
    total_fires = 0;
    continuity_checks = 0;
    safety_aborts = 0;
    
    // Initialize callbacks
    fire_callback = nullptr;
    safety_callback = nullptr;
    continuity_callback = nullptr;
}

int8_t PyrotechnicControl::begin() {
    // Configure control pins
    pinMode(master_arm_pin, INPUT_PULLUP);
    pinMode(voltage_sense_pin, INPUT);
    
    if (system_enable_pin != 255) {
        pinMode(system_enable_pin, OUTPUT);
        digitalWrite(system_enable_pin, LOW); // Start in safe state
    }
    
    // Configure all channel pins as safe outputs
    for (uint8_t i = 0; i < PYRO_MAX_CHANNELS; i++) {
        if (channels[i].active) {
            pinMode(channels[i].fire_pin, OUTPUT);
            digitalWrite(channels[i].fire_pin, LOW); // Safe state
            pinMode(channels[i].sense_pin, INPUT);
            pinMode(channels[i].continuity_pin, INPUT);
            
            // Initialize channel status
            channel_status[i].state = PYRO_STATE_SAFE;
            channel_status[i].continuity_good = false;
            channel_status[i].voltage_ok = false;
            channel_status[i].armed = false;
            channel_status[i].fired = false;
            channel_status[i].current_ma = 0.0f;
            channel_status[i].voltage = 0.0f;
            channel_status[i].last_fire_time = 0;
            channel_status[i].fire_count = 0;
            channel_status[i].fault_code = PYRO_SUCCESS;
            channel_status[i].last_continuity_check = 0;
        }
    }
    
    // Perform initial safety check
    if (perform_safety_check() != PYRO_SUCCESS) {
        return PYRO_ERROR_SAFETY;
    }
    
    // Log initialization event
    log_event(255, PYRO_EVENT_ARMED, PYRO_SUCCESS, "System initialized");
    
    initialized = true;
    return PYRO_SUCCESS;
}

int8_t PyrotechnicControl::configure_channel(uint8_t channel_id, const PyroChannelConfig& config) {
    if (channel_id >= PYRO_MAX_CHANNELS) {
        return PYRO_ERROR_INVALID_CHANNEL;
    }
    
    if (!is_valid_configuration(config)) {
        return PYRO_ERROR_INIT;
    }
    
    // Copy configuration
    channels[channel_id] = config;
    
    // Configure hardware pins
    if (initialized) {
        pinMode(config.fire_pin, OUTPUT);
        digitalWrite(config.fire_pin, LOW);
        pinMode(config.sense_pin, INPUT);
        pinMode(config.continuity_pin, INPUT);
        
        // Reset channel status
        memset(&channel_status[channel_id], 0, sizeof(PyroChannelStatus));
        channel_status[channel_id].state = PYRO_STATE_SAFE;
    }
    
    return PYRO_SUCCESS;
}

int8_t PyrotechnicControl::enable_channel(uint8_t channel_id, bool enable) {
    if (!is_valid_channel(channel_id)) {
        return PYRO_ERROR_INVALID_CHANNEL;
    }
    
    channels[channel_id].active = enable;
    
    if (!enable) {
        // Disarm and safe the channel
        channel_status[channel_id].armed = false;
        channel_status[channel_id].state = PYRO_STATE_SAFE;
        digitalWrite(channels[channel_id].fire_pin, LOW);
    }
    
    return PYRO_SUCCESS;
}

const PyroChannelConfig& PyrotechnicControl::get_channel_config(uint8_t channel_id) const {
    static PyroChannelConfig empty_config = {0};
    
    if (channel_id >= PYRO_MAX_CHANNELS) {
        return empty_config;
    }
    
    return channels[channel_id];
}

const PyroChannelStatus& PyrotechnicControl::get_channel_status(uint8_t channel_id) const {
    static PyroChannelStatus empty_status = {0};
    
    if (channel_id >= PYRO_MAX_CHANNELS) {
        return empty_status;
    }
    
    return channel_status[channel_id];
}

int8_t PyrotechnicControl::arm_system(bool arm) {
    if (!initialized) {
        return PYRO_ERROR_INIT;
    }
    
    if (arm) {
        // Perform safety check before arming
        if (perform_safety_check() != PYRO_SUCCESS) {
            return PYRO_ERROR_SAFETY;
        }
        
        system_status.system_armed = true;
        system_status.arm_time = millis();
        log_event(255, PYRO_EVENT_ARMED, PYRO_SUCCESS, "System armed");
    } else {
        system_status.system_armed = false;
        
        // Disarm all channels
        for (uint8_t i = 0; i < PYRO_MAX_CHANNELS; i++) {
            if (channels[i].active) {
                arm_channel(i, false);
            }
        }
        
        log_event(255, PYRO_EVENT_DISARMED, PYRO_SUCCESS, "System disarmed");
    }
    
    return PYRO_SUCCESS;
}

int8_t PyrotechnicControl::arm_channel(uint8_t channel_id, bool arm) {
    if (!is_valid_channel(channel_id)) {
        return PYRO_ERROR_INVALID_CHANNEL;
    }
    
    if (!system_status.system_armed && arm) {
        return PYRO_ERROR_NOT_ARMED;
    }
    
    if (arm) {
        // Check if safe to arm
        if (!is_safe_to_fire(channel_id)) {
            return PYRO_ERROR_SAFETY;
        }
        
        channel_status[channel_id].armed = true;
        channel_status[channel_id].state = PYRO_STATE_ARMED;
        system_status.channels_armed++;
        
        log_event(channel_id, PYRO_EVENT_ARMED, PYRO_SUCCESS, channels[channel_id].name);
    } else {
        channel_status[channel_id].armed = false;
        channel_status[channel_id].state = PYRO_STATE_SAFE;
        if (system_status.channels_armed > 0) {
            system_status.channels_armed--;
        }
        
        log_event(channel_id, PYRO_EVENT_DISARMED, PYRO_SUCCESS, channels[channel_id].name);
    }
    
    return PYRO_SUCCESS;
}

bool PyrotechnicControl::is_channel_armed(uint8_t channel_id) const {
    if (!is_valid_channel(channel_id)) {
        return false;
    }
    
    return channel_status[channel_id].armed;
}

int8_t PyrotechnicControl::perform_safety_check() {
    if (!initialized) {
        return PYRO_ERROR_INIT;
    }
    
    system_status.safety_ok = true;
    
    // Check master arm switch
    if (!check_master_arm()) {
        system_status.safety_ok = false;
        if (safety_callback) safety_callback(PYRO_ERROR_NOT_ARMED);
        return PYRO_ERROR_NOT_ARMED;
    }
    
    // Check system voltage
    if (!check_system_voltage()) {
        system_status.safety_ok = false;
        if (safety_callback) safety_callback(PYRO_ERROR_VOLTAGE_LOW);
        return PYRO_ERROR_VOLTAGE_LOW;
    }
    
    // Check arm timeout
    if (!check_arm_timeout()) {
        system_status.safety_ok = false;
        if (safety_callback) safety_callback(PYRO_ERROR_TIMEOUT);
        return PYRO_ERROR_TIMEOUT;
    }
    
    system_status.last_safety_check = millis();
    return PYRO_SUCCESS;
}

int8_t PyrotechnicControl::check_continuity(uint8_t channel_id) {
    if (!is_valid_channel(channel_id)) {
        return PYRO_ERROR_INVALID_CHANNEL;
    }
    
    if (!channels[channel_id].active) {
        return PYRO_ERROR_INVALID_CHANNEL;
    }
    
    // Read continuity current
    float current = read_channel_current(channel_id);
    bool continuity_good = is_continuity_current_valid(current);
    
    channel_status[channel_id].continuity_good = continuity_good;
    channel_status[channel_id].current_ma = current;
    channel_status[channel_id].last_continuity_check = millis();
    
    continuity_checks++;
    
    if (continuity_good) {
        log_event(channel_id, PYRO_EVENT_CONTINUITY_OK, PYRO_SUCCESS, channels[channel_id].name);
    } else {
        log_event(channel_id, PYRO_EVENT_CONTINUITY_FAIL, PYRO_ERROR_NO_CONTINUITY, channels[channel_id].name);
    }
    
    if (continuity_callback) {
        continuity_callback(channel_id, continuity_good);
    }
    
    return continuity_good ? PYRO_SUCCESS : PYRO_ERROR_NO_CONTINUITY;
}

int8_t PyrotechnicControl::check_all_continuity() {
    int8_t result = PYRO_SUCCESS;
    
    for (uint8_t i = 0; i < PYRO_MAX_CHANNELS; i++) {
        if (channels[i].active) {
            if (check_continuity(i) != PYRO_SUCCESS) {
                result = PYRO_ERROR_NO_CONTINUITY;
            }
        }
    }
    
    return result;
}

bool PyrotechnicControl::is_safe_to_fire(uint8_t channel_id) {
    if (!is_valid_channel(channel_id) || !channels[channel_id].active) {
        return false;
    }
    
    const PyroChannelConfig& config = channels[channel_id];
    const PyroChannelStatus& status = channel_status[channel_id];
    
    // Check if already fired
    if (status.fired) {
        return false;
    }
    
    // Check system safety
    if (!system_status.safety_ok && !safety_override) {
        return false;
    }
    
    // Check interlocks
    if (!check_interlocks(channel_id)) {
        return false;
    }
    
    // Check voltage
    float voltage = get_system_voltage();
    if (voltage < config.min_fire_voltage || voltage > config.max_fire_voltage) {
        return false;
    }
    
    // Check continuity if required
    if ((config.interlocks & PYRO_INTERLOCK_CONTINUITY) && !status.continuity_good) {
        return false;
    }
    
    return true;
}

int8_t PyrotechnicControl::fire_channel(uint8_t channel_id, uint16_t duration_ms) {
    if (!is_valid_channel(channel_id)) {
        return PYRO_ERROR_INVALID_CHANNEL;
    }
    
    if (!channels[channel_id].active) {
        return PYRO_ERROR_INVALID_CHANNEL;
    }
    
    if (channel_status[channel_id].fired) {
        return PYRO_ERROR_ALREADY_FIRED;
    }
    
    if (!is_safe_to_fire(channel_id)) {
        log_event(channel_id, PYRO_EVENT_SAFETY_ABORT, PYRO_ERROR_SAFETY, channels[channel_id].name);
        safety_aborts++;
        return PYRO_ERROR_SAFETY;
    }
    
    // Use configured duration if not specified
    if (duration_ms == 0) {
        duration_ms = channels[channel_id].fire_duration_ms;
    }
    
    // Clamp duration to safe limits
    duration_ms = constrain(duration_ms, PYRO_MIN_FIRE_DURATION, PYRO_MAX_FIRE_DURATION);
    
    // Fire the channel
    set_channel_output(channel_id, true);
    
    // Update status
    channel_status[channel_id].fired = true;
    channel_status[channel_id].last_fire_time = millis();
    channel_status[channel_id].fire_count++;
    channel_status[channel_id].state = PYRO_STATE_FIRED;
    
    // Update system statistics
    total_fires++;
    system_status.channels_fired++;
    
    // Log event
    log_event(channel_id, PYRO_EVENT_FIRED, PYRO_SUCCESS, channels[channel_id].name);
    
    // Wait for fire duration
    delay(duration_ms);
    
    // Turn off channel
    set_channel_output(channel_id, false);
    
    // Verify firing current
    float fire_current = read_channel_current(channel_id);
    bool fire_success = is_fire_current_valid(fire_current);
    
    if (fire_callback) {
        fire_callback(channel_id, fire_success);
    }
    
    return fire_success ? PYRO_SUCCESS : PYRO_ERROR_CURRENT_LOW;
}

int8_t PyrotechnicControl::fire_multiple_channels(const uint8_t* channel_ids, uint8_t num_channels, uint16_t duration_ms) {
    if (num_channels == 0 || channel_ids == nullptr) {
        return PYRO_ERROR_INVALID_CHANNEL;
    }
    
    // Check all channels first
    for (uint8_t i = 0; i < num_channels; i++) {
        if (!is_safe_to_fire(channel_ids[i])) {
            return PYRO_ERROR_SAFETY;
        }
    }
    
    // Fire all channels simultaneously
    uint32_t fire_start = millis();
    
    for (uint8_t i = 0; i < num_channels; i++) {
        uint8_t channel = channel_ids[i];
        
        set_channel_output(channel, true);
        
        // Update status
        channel_status[channel].fired = true;
        channel_status[channel].last_fire_time = fire_start;
        channel_status[channel].fire_count++;
        channel_status[channel].state = PYRO_STATE_FIRED;
        
        log_event(channel, PYRO_EVENT_FIRED, PYRO_SUCCESS, channels[channel].name);
    }
    
    // Use first channel's duration if not specified
    if (duration_ms == 0) {
        duration_ms = channels[channel_ids[0]].fire_duration_ms;
    }
    
    duration_ms = constrain(duration_ms, PYRO_MIN_FIRE_DURATION, PYRO_MAX_FIRE_DURATION);
    
    // Wait for fire duration
    delay(duration_ms);
    
    // Turn off all channels
    for (uint8_t i = 0; i < num_channels; i++) {
        set_channel_output(channel_ids[i], false);
    }
    
    // Update statistics
    total_fires += num_channels;
    system_status.channels_fired += num_channels;
    
    return PYRO_SUCCESS;
}

int8_t PyrotechnicControl::abort_all_firing() {
    for (uint8_t i = 0; i < PYRO_MAX_CHANNELS; i++) {
        if (channels[i].active) {
            set_channel_output(i, false);
        }
    }
    
    log_event(255, PYRO_EVENT_SAFETY_ABORT, PYRO_SUCCESS, "All firing aborted");
    
    return PYRO_SUCCESS;
}

void PyrotechnicControl::update() {
    if (!initialized) {
        return;
    }
    
    uint32_t current_time = millis();
    
    // Update system status
    update_system_state();
    
    // Update all active channels
    for (uint8_t i = 0; i < PYRO_MAX_CHANNELS; i++) {
        if (channels[i].active) {
            update_channel_state(i);
        }
    }
    
    // Update firing sequence
    if (sequence_running) {
        update_sequence();
    }
    
    // Periodic safety check
    if (current_time - last_safety_check > 1000) { // Check every second
        perform_safety_check();
    }
    
    // Check for arm timeout
    if (system_status.system_armed && check_arm_timeout()) {
        emergency_disarm();
    }
}

float PyrotechnicControl::get_system_voltage() {
    return read_system_voltage();
}

uint8_t PyrotechnicControl::get_channels_ready() const {
    uint8_t ready_count = 0;
    
    for (uint8_t i = 0; i < PYRO_MAX_CHANNELS; i++) {
        if (channels[i].active && is_safe_to_fire(i)) {
            ready_count++;
        }
    }
    
    return ready_count;
}

uint8_t PyrotechnicControl::get_channels_armed() const {
    return system_status.channels_armed;
}

float PyrotechnicControl::read_channel_current(uint8_t channel_id) {
    if (!is_valid_channel(channel_id)) {
        return 0.0f;
    }
    
    uint16_t adc_value = analogRead(channels[channel_id].sense_pin);
    return adc_to_current(adc_value);
}

float PyrotechnicControl::read_system_voltage() {
    uint16_t adc_value = analogRead(voltage_sense_pin);
    return adc_to_voltage(adc_value);
}

bool PyrotechnicControl::is_voltage_in_range(float voltage) {
    return (voltage >= PYRO_SAFE_VOLTAGE_MIN && voltage <= PYRO_SAFE_VOLTAGE_MAX);
}

void PyrotechnicControl::log_event(uint8_t channel_id, uint8_t event_type, uint8_t error_code, const char* description) {
    PyroEvent event;
    event.timestamp = millis();
    event.channel_id = channel_id;
    event.event_type = event_type;
    event.error_code = error_code;
    event.voltage = get_system_voltage();
    event.current = (channel_id < PYRO_MAX_CHANNELS) ? read_channel_current(channel_id) : 0.0f;
    
    if (description) {
        strncpy(event.description, description, sizeof(event.description) - 1);
        event.description[sizeof(event.description) - 1] = '\0';
    } else {
        event.description[0] = '\0';
    }
    
    add_event(event);
}

const PyroEvent* PyrotechnicControl::get_event_log(uint16_t& num_events) const {
    num_events = total_events;
    return event_log;
}

void PyrotechnicControl::clear_event_log() {
    memset(event_log, 0, sizeof(event_log));
    event_index = 0;
    total_events = 0;
}

void PyrotechnicControl::print_event_log() const {
    Serial.println("=== Pyrotechnic Event Log ===");
    
    uint16_t events_to_show = min(total_events, (uint16_t)PYRO_MAX_EVENTS);
    
    for (uint16_t i = 0; i < events_to_show; i++) {
        uint16_t idx = (event_index + PYRO_MAX_EVENTS - events_to_show + i) % PYRO_MAX_EVENTS;
        const PyroEvent& event = event_log[idx];
        
        Serial.print(event.timestamp);
        Serial.print(" ms - Ch");
        Serial.print(event.channel_id);
        Serial.print(": ");
        Serial.print(event_type_to_string(event.event_type));
        
        if (event.error_code != PYRO_SUCCESS) {
            Serial.print(" (");
            Serial.print(error_code_to_string(event.error_code));
            Serial.print(")");
        }
        
        Serial.print(" V=");
        Serial.print(event.voltage, 2);
        Serial.print("V I=");
        Serial.print(event.current, 1);
        Serial.print("mA");
        
        if (strlen(event.description) > 0) {
            Serial.print(" - ");
            Serial.print(event.description);
        }
        
        Serial.println();
    }
}

void PyrotechnicControl::reset_statistics() {
    total_fires = 0;
    continuity_checks = 0;
    safety_aborts = 0;
}

void PyrotechnicControl::set_fire_callback(void (*callback)(uint8_t channel, bool success)) {
    fire_callback = callback;
}

void PyrotechnicControl::set_safety_callback(void (*callback)(uint8_t error_code)) {
    safety_callback = callback;
}

void PyrotechnicControl::set_continuity_callback(void (*callback)(uint8_t channel, bool good)) {
    continuity_callback = callback;
}

int8_t PyrotechnicControl::emergency_disarm() {
    // Immediately disarm all channels
    abort_all_firing();
    
    system_status.system_armed = false;
    system_status.channels_armed = 0;
    
    for (uint8_t i = 0; i < PYRO_MAX_CHANNELS; i++) {
        if (channels[i].active) {
            channel_status[i].armed = false;
            channel_status[i].state = PYRO_STATE_SAFE;
        }
    }
    
    log_event(255, PYRO_EVENT_SAFETY_ABORT, PYRO_SUCCESS, "Emergency disarm");
    
    return PYRO_SUCCESS;
}

int8_t PyrotechnicControl::emergency_stop() {
    return emergency_disarm();
}

void PyrotechnicControl::print_system_status() const {
    Serial.println("=== Pyrotechnic System Status ===");
    Serial.print("System Armed: "); Serial.println(system_status.system_armed ? "YES" : "NO");
    Serial.print("Master Arm: "); Serial.println(system_status.master_arm_active ? "ACTIVE" : "INACTIVE");
    Serial.print("Channels Armed: "); Serial.println(system_status.channels_armed);
    Serial.print("Channels Fired: "); Serial.println(system_status.channels_fired);
    Serial.print("Channels Fault: "); Serial.println(system_status.channels_fault);
    Serial.print("System Voltage: "); Serial.print(system_status.system_voltage, 2); Serial.println(" V");
    Serial.print("Safety OK: "); Serial.println(system_status.safety_ok ? "YES" : "NO");
    Serial.print("Total Fires: "); Serial.println(total_fires);
    Serial.print("Safety Aborts: "); Serial.println(safety_aborts);
}

void PyrotechnicControl::print_channel_status(uint8_t channel_id) const {
    if (!is_valid_channel(channel_id)) {
        Serial.println("Invalid channel");
        return;
    }
    
    const PyroChannelStatus& status = channel_status[channel_id];
    const PyroChannelConfig& config = channels[channel_id];
    
    Serial.print("=== Channel ");
    Serial.print(channel_id);
    Serial.print(" (");
    Serial.print(config.name);
    Serial.println(") ===");
    
    Serial.print("State: ");
    switch (status.state) {
        case PYRO_STATE_SAFE: Serial.println("SAFE"); break;
        case PYRO_STATE_ARMED: Serial.println("ARMED"); break;
        case PYRO_STATE_FIRED: Serial.println("FIRED"); break;
        case PYRO_STATE_FAULT: Serial.println("FAULT"); break;
        default: Serial.println("UNKNOWN"); break;
    }
    
    Serial.print("Armed: "); Serial.println(status.armed ? "YES" : "NO");
    Serial.print("Fired: "); Serial.println(status.fired ? "YES" : "NO");
    Serial.print("Continuity: "); Serial.println(status.continuity_good ? "GOOD" : "BAD");
    Serial.print("Voltage OK: "); Serial.println(status.voltage_ok ? "YES" : "NO");
    Serial.print("Current: "); Serial.print(status.current_ma, 1); Serial.println(" mA");
    Serial.print("Fire Count: "); Serial.println(status.fire_count);
    
    if (status.fault_code != PYRO_SUCCESS) {
        Serial.print("Fault: "); Serial.println(error_code_to_string(status.fault_code));
    }
}

// Private functions
void PyrotechnicControl::set_channel_output(uint8_t channel_id, bool state) {
    if (!is_valid_channel(channel_id)) {
        return;
    }
    
    digitalWrite(channels[channel_id].fire_pin, state ? HIGH : LOW);
}

float PyrotechnicControl::read_channel_analog(uint8_t pin) {
    int adc_value = analogRead(pin);
    return (adc_value / 1023.0f) * 3.3f; // Assuming 3.3V reference
}

bool PyrotechnicControl::read_digital_input(uint8_t pin) {
    return digitalRead(pin) == HIGH;
}

bool PyrotechnicControl::check_master_arm() {
    bool arm_active = !digitalRead(master_arm_pin); // Active low
    system_status.master_arm_active = arm_active;
    return arm_active;
}

bool PyrotechnicControl::check_system_voltage() {
    float voltage = get_system_voltage();
    system_status.system_voltage = voltage;
    return is_voltage_in_range(voltage);
}

bool PyrotechnicControl::check_channel_continuity(uint8_t channel_id) {
    if (!is_valid_channel(channel_id)) {
        return false;
    }
    
    return check_continuity(channel_id) == PYRO_SUCCESS;
}

bool PyrotechnicControl::check_interlocks(uint8_t channel_id) {
    if (!is_valid_channel(channel_id)) {
        return false;
    }
    
    const PyroChannelConfig& config = channels[channel_id];
    uint8_t interlocks = config.interlocks;
    
    // Check arm switch interlock
    if ((interlocks & PYRO_INTERLOCK_ARM_SWITCH) && !system_status.master_arm_active) {
        return false;
    }
    
    // Check voltage interlock
    if ((interlocks & PYRO_INTERLOCK_VOLTAGE) && !system_status.system_voltage) {
        return false;
    }
    
    // Check continuity interlock
    if ((interlocks & PYRO_INTERLOCK_CONTINUITY) && !channel_status[channel_id].continuity_good) {
        return false;
    }
    
    // Other interlocks would be implemented here (altitude, velocity, time)
    
    return true;
}

bool PyrotechnicControl::check_arm_timeout() {
    if (!system_status.system_armed) {
        return true; // No timeout when disarmed
    }
    
    return (millis() - system_status.arm_time) < arm_timeout;
}

float PyrotechnicControl::adc_to_current(uint16_t adc_value, bool high_range) {
    // Convert ADC reading to current (mA)
    // Assumes ACS712 current sensor with 66mV/A sensitivity
    float voltage = (adc_value / 1023.0f) * 3.3f;
    float current_a = (voltage - 1.65f) / 0.066f; // 1.65V offset, 66mV/A
    return current_a * 1000.0f; // Convert to mA
}

float PyrotechnicControl::adc_to_voltage(uint16_t adc_value) {
    // Convert ADC reading to voltage
    // Assumes voltage divider with 10:1 ratio
    float adc_voltage = (adc_value / 1023.0f) * 3.3f;
    return adc_voltage * 10.0f; // Multiply by divider ratio
}

bool PyrotechnicControl::is_continuity_current_valid(float current_ma) {
    return (current_ma >= PYRO_CONTINUITY_CURRENT_MIN && current_ma <= PYRO_CONTINUITY_CURRENT_MAX);
}

bool PyrotechnicControl::is_fire_current_valid(float current_ma) {
    return (current_ma >= PYRO_FIRE_CURRENT_MIN);
}

void PyrotechnicControl::update_channel_state(uint8_t channel_id) {
    if (!is_valid_channel(channel_id)) {
        return;
    }
    
    PyroChannelStatus& status = channel_status[channel_id];
    
    // Update current reading
    status.current_ma = read_channel_current(channel_id);
    status.voltage = get_system_voltage();
    status.voltage_ok = is_voltage_in_range(status.voltage);
    
    // Update continuity if needed
    uint32_t current_time = millis();
    if (current_time - status.last_continuity_check > PYRO_CONTINUITY_DELAY) {
        check_continuity(channel_id);
    }
    
    // Update state based on conditions
    if (status.fired) {
        status.state = PYRO_STATE_FIRED;
    } else if (status.armed && system_status.system_armed) {
        status.state = PYRO_STATE_ARMED;
    } else {
        status.state = PYRO_STATE_SAFE;
    }
}

void PyrotechnicControl::update_system_state() {
    system_status.master_arm_active = check_master_arm();
    system_status.system_voltage = get_system_voltage();
    system_status.last_safety_check = millis();
    
    // Count channel states
    system_status.channels_armed = 0;
    system_status.channels_fired = 0;
    system_status.channels_fault = 0;
    
    for (uint8_t i = 0; i < PYRO_MAX_CHANNELS; i++) {
        if (channels[i].active) {
            if (channel_status[i].armed) system_status.channels_armed++;
            if (channel_status[i].fired) system_status.channels_fired++;
            if (channel_status[i].state == PYRO_STATE_FAULT) system_status.channels_fault++;
        }
    }
}

void PyrotechnicControl::handle_channel_fault(uint8_t channel_id, uint8_t fault_code) {
    if (!is_valid_channel(channel_id)) {
        return;
    }
    
    channel_status[channel_id].state = PYRO_STATE_FAULT;
    channel_status[channel_id].fault_code = fault_code;
    channel_status[channel_id].armed = false;
    
    log_event(channel_id, PYRO_EVENT_FAULT, fault_code, channels[channel_id].name);
}

void PyrotechnicControl::add_event(const PyroEvent& event) {
    event_log[event_index] = event;
    event_index = (event_index + 1) % PYRO_MAX_EVENTS;
    
    if (total_events < PYRO_MAX_EVENTS) {
        total_events++;
    }
}

const char* PyrotechnicControl::event_type_to_string(uint8_t event_type) {
    switch (event_type) {
        case PYRO_EVENT_ARMED: return "ARMED";
        case PYRO_EVENT_DISARMED: return "DISARMED";
        case PYRO_EVENT_FIRED: return "FIRED";
        case PYRO_EVENT_CONTINUITY_OK: return "CONTINUITY_OK";
        case PYRO_EVENT_CONTINUITY_FAIL: return "CONTINUITY_FAIL";
        case PYRO_EVENT_VOLTAGE_LOW: return "VOLTAGE_LOW";
        case PYRO_EVENT_VOLTAGE_HIGH: return "VOLTAGE_HIGH";
        case PYRO_EVENT_FAULT: return "FAULT";
        case PYRO_EVENT_SAFETY_ABORT: return "SAFETY_ABORT";
        default: return "UNKNOWN";
    }
}

const char* PyrotechnicControl::error_code_to_string(uint8_t error_code) {
    switch (error_code) {
        case PYRO_SUCCESS: return "SUCCESS";
        case PYRO_ERROR_INIT: return "INIT_ERROR";
        case PYRO_ERROR_NOT_ARMED: return "NOT_ARMED";
        case PYRO_ERROR_NO_CONTINUITY: return "NO_CONTINUITY";
        case PYRO_ERROR_VOLTAGE_LOW: return "VOLTAGE_LOW";
        case PYRO_ERROR_VOLTAGE_HIGH: return "VOLTAGE_HIGH";
        case PYRO_ERROR_CURRENT_LOW: return "CURRENT_LOW";
        case PYRO_ERROR_TIMEOUT: return "TIMEOUT";
        case PYRO_ERROR_SAFETY: return "SAFETY_ERROR";
        case PYRO_ERROR_ALREADY_FIRED: return "ALREADY_FIRED";
        case PYRO_ERROR_INVALID_CHANNEL: return "INVALID_CHANNEL";
        default: return "UNKNOWN_ERROR";
    }
}

bool PyrotechnicControl::is_valid_channel(uint8_t channel_id) {
    return (channel_id < PYRO_MAX_CHANNELS && channels[channel_id].active);
}

bool PyrotechnicControl::is_valid_configuration(const PyroChannelConfig& config) {
    // Basic validation
    return (config.fire_pin > 0 && 
            config.fire_duration_ms >= PYRO_MIN_FIRE_DURATION && 
            config.fire_duration_ms <= PYRO_MAX_FIRE_DURATION &&
            config.min_fire_voltage > 0 &&
            config.max_fire_voltage > config.min_fire_voltage);
}

// Placeholder implementations for sequence control (would be fully implemented)
void PyrotechnicControl::update_sequence() {
    // Sequence control implementation would go here
}

// Utility functions implementation
namespace PyroUtils {
    PyroChannelConfig create_drogue_config(uint8_t fire_pin, uint8_t sense_pin, const char* name) {
        PyroChannelConfig config = Templates::DROGUE_CHANNEL;
        config.fire_pin = fire_pin;
        config.sense_pin = sense_pin;
        strncpy(config.name, name, sizeof(config.name) - 1);
        config.name[sizeof(config.name) - 1] = '\0';
        return config;
    }
    
    PyroChannelConfig create_main_config(uint8_t fire_pin, uint8_t sense_pin, const char* name) {
        PyroChannelConfig config = Templates::MAIN_CHANNEL;
        config.fire_pin = fire_pin;
        config.sense_pin = sense_pin;
        strncpy(config.name, name, sizeof(config.name) - 1);
        config.name[sizeof(config.name) - 1] = '\0';
        return config;
    }
    
    PyroChannelConfig create_separation_config(uint8_t fire_pin, uint8_t sense_pin, const char* name) {
        PyroChannelConfig config;
        memset(&config, 0, sizeof(PyroChannelConfig));
        
        config.fire_pin = fire_pin;
        config.sense_pin = sense_pin;
        config.continuity_pin = sense_pin; // Use same pin for continuity
        config.fire_duration_ms = PYRO_FIRE_DURATION_MS;
        config.channel_type = PYRO_TYPE_SEPARATION;
        config.interlocks = PYRO_INTERLOCK_ARM_SWITCH | PYRO_INTERLOCK_CONTINUITY | PYRO_INTERLOCK_VOLTAGE;
        config.min_fire_voltage = PYRO_SAFE_VOLTAGE_MIN;
        config.max_fire_voltage = PYRO_SAFE_VOLTAGE_MAX;
        config.active = true;
        strncpy(config.name, name, sizeof(config.name) - 1);
        config.name[sizeof(config.name) - 1] = '\0';
        
        return config;
    }
}