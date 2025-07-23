#include "mk20_sensor_integration.h"

// Global instance
MK20SensorIntegration* g_mk20_sensors = nullptr;

MK20SensorIntegration::MK20SensorIntegration() :
    high_g_accel(nullptr),
    barometer(nullptr),
    gps(nullptr),
    sd_card(nullptr),
    spi_flash(nullptr),
    pyro_control(nullptr),
    battery_monitor(nullptr),
    spi_master(nullptr),
    current_flight_state(FLIGHT_STATE_IDLE),
    previous_flight_state(FLIGHT_STATE_IDLE),
    last_fast_update(0),
    last_normal_update(0),
    last_slow_update(0),
    last_data_log(0),
    sensors_initialized(false),
    logging_enabled(false),
    high_rate_logging(false),
    launch_detect_threshold(3.0f),
    apogee_detect_threshold(0.5f),
    main_deploy_altitude(300.0f),
    ground_altitude(0.0f),
    log_sequence_number(0),
    flight_state_callback(nullptr),
    sensor_error_callback(nullptr),
    data_ready_callback(nullptr)
{
    memset(&sensor_data, 0, sizeof(sensor_data));
    strcpy(current_log_filename, "");
}

MK20SensorIntegration::~MK20SensorIntegration()
{
    if (high_g_accel) delete high_g_accel;
    if (barometer) delete barometer;
    if (gps) delete gps;
    if (sd_card) delete sd_card;
    if (spi_flash) delete spi_flash;
    if (pyro_control) delete pyro_control;
    if (battery_monitor) delete battery_monitor;
    if (spi_master) delete spi_master;
}

int8_t MK20SensorIntegration::begin()
{
    initialize_pins();
    configure_interrupts();
    
    if (initialize_sensors() != 0) {
        return -1;
    }
    
    if (configure_sensors() != 0) {
        return -2;
    }
    
    // Initialize communication with slave processors
    spi_master = new SPIMaster();
    if (spi_master->begin() != 0) {
        return -3;
    }
    
    sensors_initialized = true;
    current_flight_state = FLIGHT_STATE_IDLE;
    
    // Calibrate ground altitude
    calibrate_ground_altitude();
    
    return 0;
}

int8_t MK20SensorIntegration::initialize_sensors()
{
    // Initialize high-G accelerometer
    high_g_accel = new KX134Accelerometer();
    if (high_g_accel->begin(MK20_KX134_CS_PIN) != 0) {
        handle_sensor_error("KX134", -1);
        return -1;
    }
    
    // Initialize barometer
    barometer = new MS5611Barometer();
    if (barometer->begin(MK20_BAROMETER_CS_PIN) != 0) {
        handle_sensor_error("MS5611", -1);
        return -2;
    }
    
    // Initialize GPS
    gps = new ZOE_M8Q_GPS();
    if (gps->begin(Serial1) != 0) {
        handle_sensor_error("ZOE-M8Q", -1);
        return -3;
    }
    
    // Initialize SD card
    sd_card = new SDCardManager();
    if (sd_card->begin(MK20_SD_CARD_CS_PIN) != 0) {
        handle_sensor_error("SD_CARD", -1);
        // SD card not critical for flight, continue
    }
    
    // Initialize SPI flash
    spi_flash = new SPIFlash();
    if (spi_flash->begin(MK20_SPI_FLASH_CS_PIN) != 0) {
        handle_sensor_error("SPI_FLASH", -1);
        // Flash not critical for flight, continue
    }
    
    // Initialize pyrotechnic control
    pyro_control = new PyrotechnicControl();
    if (pyro_control->begin(MK20_PYRO_ARM_PIN, MK20_PYRO_VOLTAGE_PIN) != 0) {
        handle_sensor_error("PYRO", -1);
        return -4;
    }
    
    // Add pyrotechnic channels
    pyro_control->add_channel(0, MK20_PYRO_CH1_PIN, "Drogue");
    pyro_control->add_channel(1, MK20_PYRO_CH2_PIN, "Main");
    pyro_control->add_channel(2, MK20_PYRO_CH3_PIN, "Separation");
    pyro_control->add_channel(3, MK20_PYRO_CH4_PIN, "Backup");
    
    // Initialize battery monitor
    battery_monitor = new BatteryMonitor();
    if (battery_monitor->begin(MK20_BATTERY_VOLTAGE_PIN, MK20_BATTERY_CURRENT_PIN) != 0) {
        handle_sensor_error("BATTERY", -1);
        // Battery monitor not critical, continue
    }
    
    return 0;
}

int8_t MK20SensorIntegration::configure_sensors()
{
    // Configure high-G accelerometer for ±200g range
    if (high_g_accel->set_range(KX134_RANGE_200G) != 0) return -1;
    if (high_g_accel->set_output_data_rate(KX134_ODR_100HZ) != 0) return -1;
    
    // Configure barometer for high resolution
    if (barometer->set_resolution(MS5611_OSR_4096) != 0) return -2;
    
    // Configure GPS for airborne mode
    if (gps->set_dynamic_model(ZOE_M8Q_AIRBORNE_4G) != 0) return -3;
    if (gps->set_measurement_rate(200) != 0) return -3; // 5Hz update
    
    // Configure battery monitor
    if (battery_monitor) {
        battery_monitor->set_voltage_divider_ratio(11.0f); // 10:1 divider
        battery_monitor->calibrate_voltage_offset(0.0f);
    }
    
    return 0;
}

void MK20SensorIntegration::update()
{
    uint32_t current_time = millis();
    
    // Fast update cycle (10ms) - critical sensors
    if (current_time - last_fast_update >= MK20_SENSOR_UPDATE_FAST) {
        update_fast_sensors();
        last_fast_update = current_time;
    }
    
    // Normal update cycle (50ms) - regular sensors
    if (current_time - last_normal_update >= MK20_SENSOR_UPDATE_NORMAL) {
        update_normal_sensors();
        update_flight_state();
        last_normal_update = current_time;
    }
    
    // Slow update cycle (200ms) - system status
    if (current_time - last_slow_update >= MK20_SENSOR_UPDATE_SLOW) {
        update_slow_sensors();
        last_slow_update = current_time;
    }
    
    // Process all sensor data and calculate derived values
    process_sensor_data();
    
    // Log data if enabled
    if (logging_enabled && (current_time - last_data_log >= (high_rate_logging ? 10 : 100))) {
        log_sensor_data();
        last_data_log = current_time;
    }
    
    // Call data ready callback
    if (data_ready_callback) {
        data_ready_callback(sensor_data);
    }
}

void MK20SensorIntegration::update_fast_sensors()
{
    // Update high-G accelerometer
    if (high_g_accel && high_g_accel->is_data_ready()) {
        float accel_data[3];
        if (high_g_accel->read_acceleration(accel_data) == 0) {
            sensor_data.high_g_accel.accel_x = accel_data[0];
            sensor_data.high_g_accel.accel_y = accel_data[1];
            sensor_data.high_g_accel.accel_z = accel_data[2];
            sensor_data.high_g_accel.data_valid = true;
            sensor_data.high_g_accel.timestamp = millis();
        }
    }
    
    // Update pyrotechnic system status
    if (pyro_control) {
        sensor_data.system.pyro_armed = pyro_control->is_armed();
        sensor_data.system.pyro_status = pyro_control->get_channel_status();
    }
}

void MK20SensorIntegration::update_normal_sensors()
{
    // Update barometer
    if (barometer) {
        float pressure, temperature;
        if (barometer->read_pressure_temperature(pressure, temperature) == 0) {
            sensor_data.barometer.pressure = pressure;
            sensor_data.barometer.temperature = temperature;
            sensor_data.barometer.altitude = barometer->pressure_to_altitude(pressure);
            sensor_data.barometer.data_valid = true;
            sensor_data.barometer.timestamp = millis();
        }
    }
    
    // Update GPS
    if (gps && gps->is_data_available()) {
        GPSData gps_data;
        if (gps->read_data(gps_data) == 0) {
            sensor_data.gps.latitude = gps_data.latitude;
            sensor_data.gps.longitude = gps_data.longitude;
            sensor_data.gps.altitude_gps = gps_data.altitude;
            sensor_data.gps.speed = gps_data.speed;
            sensor_data.gps.course = gps_data.course;
            sensor_data.gps.satellites = gps_data.satellites;
            sensor_data.gps.fix_type = gps_data.fix_type;
            sensor_data.gps.data_valid = (gps_data.fix_type >= 2);
            sensor_data.gps.timestamp = millis();
        }
    }
}

void MK20SensorIntegration::update_slow_sensors()
{
    // Update battery monitor
    if (battery_monitor) {
        sensor_data.system.battery_voltage = battery_monitor->get_voltage();
        sensor_data.system.battery_current = battery_monitor->get_current();
        sensor_data.system.battery_percentage = battery_monitor->get_charge_percentage();
    }
    
    // Update system status
    sensor_data.system.sd_card_ready = (sd_card && sd_card->is_card_present());
    sensor_data.system.flash_ready = (spi_flash && spi_flash->is_available());
    sensor_data.system.free_memory = get_free_memory();
    sensor_data.system.timestamp = millis();
}

void MK20SensorIntegration::update_flight_state()
{
    previous_flight_state = current_flight_state;
    
    switch (current_flight_state) {
        case FLIGHT_STATE_IDLE:
            if (pyro_control && pyro_control->is_armed()) {
                current_flight_state = FLIGHT_STATE_ARMED;
            }
            break;
            
        case FLIGHT_STATE_ARMED:
            if (!pyro_control || !pyro_control->is_armed()) {
                current_flight_state = FLIGHT_STATE_IDLE;
            } else {
                detect_launch();
            }
            break;
            
        case FLIGHT_STATE_BOOST:
            detect_apogee();
            break;
            
        case FLIGHT_STATE_COAST:
            detect_apogee();
            break;
            
        case FLIGHT_STATE_APOGEE:
            current_flight_state = FLIGHT_STATE_DROGUE_DESCENT;
            deploy_drogue_parachute();
            break;
            
        case FLIGHT_STATE_DROGUE_DESCENT:
            detect_main_deployment();
            break;
            
        case FLIGHT_STATE_MAIN_DESCENT:
            detect_landing();
            break;
            
        case FLIGHT_STATE_LANDED:
            // Stay in landed state
            break;
            
        case FLIGHT_STATE_ERROR:
            // Stay in error state until reset
            break;
    }
    
    // Notify callback if state changed
    if (current_flight_state != previous_flight_state && flight_state_callback) {
        flight_state_callback(previous_flight_state, current_flight_state);
    }
}

void MK20SensorIntegration::detect_launch()
{
    if (sensor_data.high_g_accel.data_valid) {
        float total_accel = sqrt(
            sensor_data.high_g_accel.accel_x * sensor_data.high_g_accel.accel_x +
            sensor_data.high_g_accel.accel_y * sensor_data.high_g_accel.accel_y +
            sensor_data.high_g_accel.accel_z * sensor_data.high_g_accel.accel_z
        );
        
        if (total_accel > launch_detect_threshold) {
            current_flight_state = FLIGHT_STATE_BOOST;
            start_logging("FLIGHT");
        }
    }
}

void MK20SensorIntegration::detect_apogee()
{
    static float last_velocity = 0.0f;
    float current_velocity = get_velocity();
    
    // Apogee detected when velocity changes from positive to negative
    if (last_velocity > apogee_detect_threshold && current_velocity < -apogee_detect_threshold) {
        current_flight_state = FLIGHT_STATE_APOGEE;
    } else if (current_velocity < apogee_detect_threshold && current_velocity > -apogee_detect_threshold) {
        current_flight_state = FLIGHT_STATE_COAST;
    }
    
    last_velocity = current_velocity;
}

void MK20SensorIntegration::detect_main_deployment()
{
    if (sensor_data.barometer.data_valid) {
        float current_altitude = sensor_data.barometer.altitude - ground_altitude;
        if (current_altitude <= main_deploy_altitude) {
            current_flight_state = FLIGHT_STATE_MAIN_DESCENT;
            deploy_main_parachute();
        }
    }
}

void MK20SensorIntegration::detect_landing()
{
    static uint32_t low_velocity_start = 0;
    float velocity = get_velocity();
    
    if (abs(velocity) < 1.0f) { // Less than 1 m/s
        if (low_velocity_start == 0) {
            low_velocity_start = millis();
        } else if (millis() - low_velocity_start > 5000) { // 5 seconds of low velocity
            current_flight_state = FLIGHT_STATE_LANDED;
            stop_logging();
        }
    } else {
        low_velocity_start = 0;
    }
}

float MK20SensorIntegration::get_velocity() const
{
    static float last_altitude = 0.0f;
    static uint32_t last_time = 0;
    
    if (!sensor_data.barometer.data_valid) return 0.0f;
    
    uint32_t current_time = millis();
    float current_altitude = sensor_data.barometer.altitude;
    
    if (last_time == 0) {
        last_altitude = current_altitude;
        last_time = current_time;
        return 0.0f;
    }
    
    float velocity = MK20Utils::calculate_velocity_from_altitude(
        current_altitude, last_altitude, current_time - last_time);
    
    last_altitude = current_altitude;
    last_time = current_time;
    
    return velocity;
}

float MK20SensorIntegration::get_acceleration() const
{
    if (!sensor_data.high_g_accel.data_valid) return 0.0f;
    
    return sqrt(
        sensor_data.high_g_accel.accel_x * sensor_data.high_g_accel.accel_x +
        sensor_data.high_g_accel.accel_y * sensor_data.high_g_accel.accel_y +
        sensor_data.high_g_accel.accel_z * sensor_data.high_g_accel.accel_z
    );
}

int8_t MK20SensorIntegration::deploy_drogue_parachute()
{
    if (!pyro_control) return -1;
    return pyro_control->fire_channel(0, 1000); // Fire drogue for 1 second
}

int8_t MK20SensorIntegration::deploy_main_parachute()
{
    if (!pyro_control) return -1;
    return pyro_control->fire_channel(1, 1000); // Fire main for 1 second
}

int8_t MK20SensorIntegration::start_logging(const char* flight_name)
{
    if (!sd_card || !sd_card->is_card_present()) return -1;
    
    if (flight_name) {
        snprintf(current_log_filename, sizeof(current_log_filename), 
                "%s_%lu.csv", flight_name, millis());
    } else {
        snprintf(current_log_filename, sizeof(current_log_filename), 
                "LOG_%lu.csv", millis());
    }
    
    // Create log file with header
    String header = "Time,Accel_X,Accel_Y,Accel_Z,Pressure,Altitude,Temperature,";
    header += "GPS_Lat,GPS_Lon,GPS_Alt,GPS_Speed,GPS_Sats,Battery_V,Flight_State\n";
    
    if (sd_card->write_file(current_log_filename, header.c_str()) == 0) {
        logging_enabled = true;
        log_sequence_number = 0;
        return 0;
    }
    
    return -2;
}

int8_t MK20SensorIntegration::log_sensor_data()
{
    if (!logging_enabled || !sd_card) return -1;
    
    char log_line[256];
    snprintf(log_line, sizeof(log_line),
        "%lu,%.3f,%.3f,%.3f,%.1f,%.2f,%.1f,%.6f,%.6f,%.1f,%.1f,%d,%.2f,%d\n",
        millis(),
        sensor_data.high_g_accel.accel_x,
        sensor_data.high_g_accel.accel_y,
        sensor_data.high_g_accel.accel_z,
        sensor_data.barometer.pressure,
        sensor_data.barometer.altitude,
        sensor_data.barometer.temperature,
        sensor_data.gps.latitude,
        sensor_data.gps.longitude,
        sensor_data.gps.altitude_gps,
        sensor_data.gps.speed,
        sensor_data.gps.satellites,
        sensor_data.system.battery_voltage,
        current_flight_state
    );
    
    return sd_card->append_file(current_log_filename, log_line);
}

void MK20SensorIntegration::process_sensor_data()
{
    calculate_derived_values();
    apply_sensor_fusion();
}

void MK20SensorIntegration::calculate_derived_values()
{
    // Calculate total acceleration magnitude
    if (sensor_data.high_g_accel.data_valid) {
        float total_g = sqrt(
            sensor_data.high_g_accel.accel_x * sensor_data.high_g_accel.accel_x +
            sensor_data.high_g_accel.accel_y * sensor_data.high_g_accel.accel_y +  
            sensor_data.high_g_accel.accel_z * sensor_data.high_g_accel.accel_z
        );
        // TODO: Store in a system variable if needed
    }
}

void MK20SensorIntegration::apply_sensor_fusion()
{
    // Simple sensor fusion - could be expanded with Kalman filter TODO
    // For now, just validate data consistency
    if (sensor_data.barometer.data_valid && sensor_data.gps.data_valid) {
        float altitude_diff = abs(sensor_data.barometer.altitude - sensor_data.gps.altitude_gps);
        if (altitude_diff > 100.0f) {
            // Large altitude difference - log warning
        }
    }
}

void MK20SensorIntegration::initialize_pins()
{
    // Initialize status LEDs
    pinMode(MK20_LED_RED_PIN, OUTPUT);
    pinMode(MK20_LED_GREEN_PIN, OUTPUT);
    pinMode(MK20_LED_BLUE_PIN, OUTPUT);
    
    // Initialize SPI pins
    pinMode(MK20_SPI_MOSI_PIN, OUTPUT);
    pinMode(MK20_SPI_MISO_PIN, INPUT);
    pinMode(MK20_SPI_SCK_PIN, OUTPUT);
    pinMode(MK20_SAMD21_NAV_CS_PIN, OUTPUT);
    pinMode(MK20_SAMD21_TEL_CS_PIN, OUTPUT);
    
    // Set CS pins high (inactive)
    digitalWrite(MK20_SAMD21_NAV_CS_PIN, HIGH);
    digitalWrite(MK20_SAMD21_TEL_CS_PIN, HIGH);
    
    // Initialize GPS serial
    Serial1.begin(9600);
}

void MK20SensorIntegration::handle_sensor_error(const char* sensor_name, int8_t error_code)
{
    if (sensor_error_callback) {
        sensor_error_callback(sensor_name, error_code);
    }
    
    // Log error to SD card if available
    if (sd_card && sd_card->is_card_present()) {
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "ERROR: %s failed with code %d\n", 
                sensor_name, error_code);
        sd_card->append_file("errors.log", error_msg);
    }
}

uint32_t MK20SensorIntegration::get_free_memory()
{
    char top;
    return &top - reinterpret_cast<char*>(sbrk(0));
}

// Namespace implementations
namespace MK20Utils {
    float calculate_velocity_from_altitude(float current_alt, float previous_alt, uint32_t time_diff_ms)
    {
        if (time_diff_ms == 0) return 0.0f;
        return (current_alt - previous_alt) / (time_diff_ms / 1000.0f);
    }
    
    float calculate_vertical_acceleration(float current_vel, float previous_vel, uint32_t time_diff_ms)
    {
        if (time_diff_ms == 0) return 0.0f;
        return (current_vel - previous_vel) / (time_diff_ms / 1000.0f);
    }
    
    bool validate_sensor_data(const MK20SensorData& data)
    {
        // Basic validation checks
        if (data.high_g_accel.data_valid) {
            if (!is_acceleration_reasonable(abs(data.high_g_accel.accel_x)) ||
                !is_acceleration_reasonable(abs(data.high_g_accel.accel_y)) ||
                !is_acceleration_reasonable(abs(data.high_g_accel.accel_z))) {
                return false;
            }
        }
        
        if (data.barometer.data_valid) {
            if (!is_altitude_reasonable(data.barometer.altitude)) {
                return false;
            }
        }
        
        if (data.gps.data_valid) {
            if (!is_gps_data_reasonable(data.gps.latitude, data.gps.longitude, data.gps.altitude_gps)) {
                return false;
            }
        }
        
        return true;
    }
    
    bool is_acceleration_reasonable(float accel_g)
    {
        return (accel_g >= 0.0f && accel_g <= 200.0f); // 0 to 200g range
    }
    
    bool is_altitude_reasonable(float altitude)
    {
        return (altitude >= -500.0f && altitude <= 50000.0f); // -500m to 50km
    }
    
    bool is_gps_data_reasonable(double lat, double lon, float alt)
    {
        return (lat >= -90.0 && lat <= 90.0 && 
                lon >= -180.0 && lon <= 180.0 && 
                alt >= -1000.0f && alt <= 50000.0f);
    }
    
    // Configuration templates
    const FlightConfig DEFAULT_FLIGHT_CONFIG = {
        .main_deploy_altitude = 300.0f,
        .launch_threshold_g = 3.0f,
        .apogee_threshold_ms = 0.5f,
        .sensor_update_rate_ms = 50,
        .enable_high_rate_logging = false,
        .enable_backup_systems = true
    };
    
    const FlightConfig HIGH_ALTITUDE_CONFIG = {
        .main_deploy_altitude = 500.0f,
        .launch_threshold_g = 3.0f,
        .apogee_threshold_ms = 0.5f,
        .sensor_update_rate_ms = 20,
        .enable_high_rate_logging = true,
        .enable_backup_systems = true
    };
    
    const FlightConfig TEST_FLIGHT_CONFIG = {
        .main_deploy_altitude = 100.0f,
        .launch_threshold_g = 2.0f,
        .apogee_threshold_ms = 1.0f,
        .sensor_update_rate_ms = 100,
        .enable_high_rate_logging = false,
        .enable_backup_systems = false
    };
}