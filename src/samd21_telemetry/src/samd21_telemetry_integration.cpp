#include "samd21_telemetry_integration.h"

// Global instance
SAMD21TelemetryIntegration* g_samd21_telemetry = nullptr;

SAMD21TelemetryIntegration::SAMD21TelemetryIntegration() :
    telemetry(nullptr),
    spi_slave(nullptr),
    initialized(false),
    radio_configured(false),
    ground_station_connected(false),
    current_radio_profile(0),
    last_radio_update(0),
    last_data_transmission(0),
    last_heartbeat(0),
    last_status_update(0),
    last_rssi_update(0),
    last_master_communication(0),
    ground_command_callback(nullptr),
    connection_status_callback(nullptr),
    data_received_callback(nullptr),
    error_callback(nullptr)
{
    memset(&tel_data, 0, sizeof(tel_data));
    memset(&master_data, 0, sizeof(master_data));
    memset(&tx_queue, 0, sizeof(tx_queue));
    memset(&stats, 0, sizeof(stats));
    
    initialize_radio_profiles();
}

SAMD21TelemetryIntegration::~SAMD21TelemetryIntegration()
{
    if (telemetry) delete telemetry;
    if (spi_slave) delete spi_slave;
}

int8_t SAMD21TelemetryIntegration::begin()
{
    initialize_pins();
    configure_uart();
    
    if (initialize_radio() != 0) {
        return -1;
    }
    
    // Initialize SPI slave communication
    spi_slave = new SPISlave();
    if (spi_slave->begin(TEL_SPI_CS_PIN) != 0) {
        return -2;
    }
    
    initialized = true;
    
    // Start with default radio profile
    configure_radio_profile(0);
    
    // Set initial status
    tel_data.radio_status.radio_ready = true;
    tel_data.radio_status.timestamp = millis();
    
    return 0;
}

int8_t SAMD21TelemetryIntegration::initialize_radio()
{
    // Initialize telemetry system with Serial1 (radio UART)
    telemetry = new TelemetrySystem(Serial1, TEL_RADIO_ENABLE_PIN, TEL_RADIO_RESET_PIN);
    
    // Configure default telemetry settings
    TelemetryConfig config = TelemetryUtils::Templates::LONG_RANGE_CONFIG;
    config.radio.frequency = 433000000; // 433 MHz
    config.radio.power_level = 80; // 80% power
    strcpy(config.radio.call_sign, "ROCKET");
    
    if (telemetry->begin(config, 2) != 0) { // System ID = 2 (telemetry)
        if (error_callback) {
            error_callback("Radio initialization failed");
        }
        return -1;
    }
    
    // Set callbacks
    telemetry->set_packet_received_callback([](const TelemetryPacket& packet) {
        if (g_samd21_telemetry && g_samd21_telemetry->data_received_callback) {
            g_samd21_telemetry->data_received_callback(packet);
        }
    });
    
    telemetry->set_connection_status_callback([](bool connected) {
        if (g_samd21_telemetry) {
            g_samd21_telemetry->ground_station_connected = connected;
            if (g_samd21_telemetry->connection_status_callback) {
                g_samd21_telemetry->connection_status_callback(connected);
            }
        }
    });
    
    current_config = config;
    radio_configured = true;
    
    return 0;
}

int8_t SAMD21TelemetryIntegration::configure_radio_profile(uint8_t profile_id)
{
    if (profile_id >= 4 || !telemetry) return -1;
    
    const RadioProfile& profile = radio_profiles[profile_id];
    
    RadioConfig radio_config;
    radio_config.frequency = profile.frequency;
    radio_config.power_level = profile.power_level;
    radio_config.data_rate = profile.data_rate;
    radio_config.use_encryption = profile.use_encryption;
    radio_config.use_fec = profile.use_fec;
    
    if (telemetry->configure_radio(radio_config) != 0) {
        return -2;
    }
    
    current_radio_profile = profile_id;
    return 0;
}

void SAMD21TelemetryIntegration::update()
{
    uint32_t current_time = millis();
    
    // Update radio communication (high frequency)
    if (current_time - last_radio_update >= TEL_RADIO_UPDATE_RATE) {
        update_radio_communication();
        last_radio_update = current_time;
    }
    
    // Update master communication
    update_master_communication();
    
    // Update transmission queue
    update_transmission_queue();
    
    // Update connection status
    update_connection_status();
    
    // Send heartbeat
    if (current_time - last_heartbeat >= TEL_HEARTBEAT_RATE) {
        if (telemetry) {
            telemetry->send_heartbeat();
        }
        last_heartbeat = current_time;
    }
    
    // Update RSSI
    if (current_time - last_rssi_update >= TEL_RSSI_UPDATE_RATE) {
        if (telemetry) {
            int8_t rssi = telemetry->get_rssi();
            tel_data.radio_status.rssi = rssi;
            tel_data.radio_status.link_quality = calculate_link_quality(rssi, 
                tel_data.radio_status.packets_lost);
        }
        last_rssi_update = current_time;
    }
    
    // Update statistics
    update_statistics();
}

void SAMD21TelemetryIntegration::update_radio_communication()
{
    if (!telemetry) return;
    
    // Update telemetry system
    telemetry->update();
    
    // Process incoming packets
    process_incoming_packets();
    
    // Process ground commands
    process_ground_commands();
    
    // Update radio status
    update_radio_status();
}

void SAMD21TelemetryIntegration::update_master_communication()
{
    // Process SPI communication with master
    if (spi_slave && spi_slave->is_data_available()) {
        uint8_t received_data[128];
        uint16_t length = spi_slave->read_buffer(received_data, sizeof(received_data));
        
        if (length > 0) {
            handle_spi_received_data(received_data, length);
            last_master_communication = millis();
        }
    }
    
    // Check for communication timeout
    if (millis() - last_master_communication > TEL_CONNECTION_TIMEOUT) {
        // Master communication timeout - could trigger emergency mode
    }
}

void SAMD21TelemetryIntegration::update_transmission_queue()
{
    uint32_t current_time = millis();
    
    // Process queued transmissions
    if (current_time - last_data_transmission >= TEL_DATA_TX_RATE) {
        transmit_queued_packets();
        last_data_transmission = current_time;
    }
}

void SAMD21TelemetryIntegration::update_connection_status()
{
    // Update connection status based on last contact time
    uint32_t time_since_contact = millis() - tel_data.radio_status.last_contact_time;
    
    if (time_since_contact > TEL_CONNECTION_TIMEOUT) {
        if (ground_station_connected) {
            ground_station_connected = false;
            tel_data.radio_status.connected_to_ground = false;
            
            if (connection_status_callback) {
                connection_status_callback(false);
            }
        }
    }
}

void SAMD21TelemetryIntegration::update_statistics()
{
    stats.uptime = millis();
    
    if (tel_data.radio_status.packets_sent > 0) {
        tel_data.radio_status.connection_time = millis() - stats.uptime;
    }
}

int8_t SAMD21TelemetryIntegration::process_incoming_packets()
{
    if (!telemetry || !telemetry->has_pending_packets()) {
        return 0;
    }
    
    while (telemetry->has_pending_packets()) {
        TelemetryPacket packet = telemetry->get_next_packet();
        
        if (handle_received_packet(packet) == 0) {
            tel_data.radio_status.packets_received++;
            tel_data.radio_status.last_contact_time = millis();
            
            if (!ground_station_connected) {
                ground_station_connected = true;
                tel_data.radio_status.connected_to_ground = true;
                
                if (connection_status_callback) {
                    connection_status_callback(true);
                }
            }
        }
    }
    
    return 0;
}

int8_t SAMD21TelemetryIntegration::handle_received_packet(const TelemetryPacket& packet)
{
    switch (packet.header.packet_type) {
        case TELEM_PKT_COMMAND:
            // Handle ground command
            if (packet.header.payload_length >= sizeof(TelemetryCommand)) {
                TelemetryCommand command;
                memcpy(&command, packet.payload, sizeof(command));
                execute_ground_command((GroundCommand)command.command_id, command.parameters);
            }
            break;
            
        case TELEM_PKT_PING:
            // Respond to ping
            telemetry->send_acknowledgment(packet.header.sequence_number);
            break;
            
        case TELEM_PKT_CONFIG:
            // Handle configuration update
            break;
            
        case TELEM_PKT_ACK:
            // Handle acknowledgment
            break;
            
        default:
            break;
    }
    
    return 0;
}

int8_t SAMD21TelemetryIntegration::transmit_flight_data(const void* data, uint16_t length)
{
    if (!telemetry || !ground_station_connected) return -1;
    
    int8_t result = telemetry->send_packet(TELEM_PKT_FLIGHT_DATA, data, length, TELEM_PRIORITY_HIGH);
    
    if (result == 0) {
        tel_data.transmission_stats.flight_data_sent++;
        tel_data.radio_status.packets_sent++;
    } else {
        tel_data.radio_status.packets_lost++;
    }
    
    return result;
}

int8_t SAMD21TelemetryIntegration::transmit_gps_data(const void* data, uint16_t length)
{
    if (!telemetry || !ground_station_connected) return -1;
    
    int8_t result = telemetry->send_packet(TELEM_PKT_GPS_DATA, data, length, TELEM_PRIORITY_NORMAL);
    
    if (result == 0) {
        tel_data.transmission_stats.gps_data_sent++;
        tel_data.radio_status.packets_sent++;
    } else {
        tel_data.radio_status.packets_lost++;
    }
    
    return result;
}

int8_t SAMD21TelemetryIntegration::transmit_sensor_data(const void* data, uint16_t length)
{
    if (!telemetry || !ground_station_connected) return -1;
    
    int8_t result = telemetry->send_packet(TELEM_PKT_SENSOR_DATA, data, length, TELEM_PRIORITY_NORMAL);
    
    if (result == 0) {
        tel_data.transmission_stats.sensor_data_sent++;
        tel_data.radio_status.packets_sent++;
    } else {
        tel_data.radio_status.packets_lost++;
    }
    
    return result;
}

int8_t SAMD21TelemetryIntegration::transmit_system_status(const void* data, uint16_t length)
{
    if (!telemetry || !ground_station_connected) return -1;
    
    int8_t result = telemetry->send_packet(TELEM_PKT_SYSTEM_STATUS, data, length, TELEM_PRIORITY_LOW);
    
    if (result == 0) {
        tel_data.transmission_stats.status_data_sent++;
        tel_data.radio_status.packets_sent++;
    } else {
        tel_data.radio_status.packets_lost++;
    }
    
    return result;
}

int8_t SAMD21TelemetryIntegration::transmit_emergency_data(const void* data, uint16_t length)
{
    if (!telemetry) return -1;
    
    int8_t result = telemetry->send_emergency_packet(data, length);
    
    if (result == 0) {
        tel_data.transmission_stats.emergency_data_sent++;
        tel_data.radio_status.packets_sent++;
    } else {
        tel_data.radio_status.packets_lost++;
    }
    
    return result;
}

int8_t SAMD21TelemetryIntegration::process_ground_commands()
{
    // Check for pending commands from telemetry system
    while (telemetry && telemetry->has_pending_packets()) {
        TelemetryCommand command = telemetry->get_next_command();
        
        if (validate_ground_command((GroundCommand)command.command_id, command.parameters)) {
            execute_ground_command((GroundCommand)command.command_id, command.parameters);
            
            // Send acknowledgment
            telemetry->send_acknowledgment(0, true); // Positive ACK
        } else {
            // Send negative acknowledgment
            telemetry->send_acknowledgment(0, false); // Negative ACK
        }
    }
    
    return 0;
}

void SAMD21TelemetryIntegration::execute_ground_command(GroundCommand command, const uint8_t* parameters)
{
    switch (command) {
        case CMD_ARM_SYSTEM:
            // Forward to master processor
            forward_ground_command_to_master(command, parameters);
            break;
            
        case CMD_DISARM_SYSTEM:
            forward_ground_command_to_master(command, parameters);
            break;
            
        case CMD_DEPLOY_DROGUE:
            forward_ground_command_to_master(command, parameters);
            break;
            
        case CMD_DEPLOY_MAIN:
            forward_ground_command_to_master(command, parameters);
            break;
            
        case CMD_ABORT_FLIGHT:
            forward_ground_command_to_master(command, parameters);
            activate_emergency_mode();
            break;
            
        case CMD_SET_PARAMETER:
            // Handle parameter setting locally or forward to master
            break;
            
        case CMD_REQUEST_DATA:
            // Send requested data type
            break;
            
        case CMD_EMERGENCY_STOP:
            activate_emergency_mode();
            forward_ground_command_to_master(command, parameters);
            break;
            
        default:
            // Unknown command
            break;
    }
    
    // Call callback if set
    if (ground_command_callback) {
        ground_command_callback(command, parameters);
    }
    
    stats.command_executions++;
}

bool SAMD21TelemetryIntegration::validate_ground_command(GroundCommand command, const uint8_t* parameters)
{
    // Basic command validation
    switch (command) {
        case CMD_ARM_SYSTEM:
        case CMD_DISARM_SYSTEM:
        case CMD_DEPLOY_DROGUE:
        case CMD_DEPLOY_MAIN:
        case CMD_ABORT_FLIGHT:
        case CMD_REQUEST_DATA:
        case CMD_EMERGENCY_STOP:
            return true; // These commands are always valid
            
        case CMD_SET_PARAMETER:
            // Validate parameter setting command
            return (parameters != nullptr);
            
        default:
            return false; // Unknown command
    }
}

int8_t SAMD21TelemetryIntegration::forward_ground_command_to_master(GroundCommand command, const uint8_t* parameters)
{
    if (!spi_slave) return -1;
    
    uint8_t command_data[32];
    command_data[0] = 0x10; // Command packet type
    command_data[1] = (uint8_t)command;
    
    if (parameters) {
        memcpy(&command_data[2], parameters, 16); // Copy up to 16 parameter bytes
    }
    
    return spi_slave->send_response(command_data, 18);
}

void SAMD21TelemetryIntegration::handle_spi_received_data(const uint8_t* data, uint16_t length)
{
    if (length < 1) return;
    
    uint8_t packet_type = data[0];
    
    switch (packet_type) {
        case 0x01: // Flight data from master
            if (length >= sizeof(master_data.flight_data)) {
                memcpy(master_data.flight_data, &data[1], sizeof(master_data.flight_data));
                master_data.flight_data_ready = true;
                
                // Automatically transmit flight data
                transmit_flight_data(master_data.flight_data, sizeof(master_data.flight_data));
            }
            break;
            
        case 0x02: // GPS data from master
            if (length >= sizeof(master_data.gps_data)) {
                memcpy(master_data.gps_data, &data[1], sizeof(master_data.gps_data));
                master_data.gps_data_ready = true;
                
                // Automatically transmit GPS data
                transmit_gps_data(master_data.gps_data, sizeof(master_data.gps_data));
            }
            break;
            
        case 0x03: // Sensor data from master
            if (length >= sizeof(master_data.sensor_data)) {
                memcpy(master_data.sensor_data, &data[1], sizeof(master_data.sensor_data));
                master_data.sensor_data_ready = true;
                
                // Automatically transmit sensor data
                transmit_sensor_data(master_data.sensor_data, sizeof(master_data.sensor_data));
            }
            break;
            
        case 0x04: // System status from master
            if (length >= sizeof(master_data.system_status)) {
                memcpy(master_data.system_status, &data[1], sizeof(master_data.system_status));
                master_data.status_data_ready = true;
                
                // Automatically transmit system status
                transmit_system_status(master_data.system_status, sizeof(master_data.system_status));
            }
            break;
            
        case 0x05: // Request telemetry status
            send_telemetry_status_to_master();
            break;
    }
    
    master_data.last_update_time = millis();
}

int8_t SAMD21TelemetryIntegration::send_telemetry_status_to_master()
{
    if (!spi_slave) return -1;
    
    uint8_t status_data[32];
    uint16_t offset = 0;
    
    status_data[offset++] = 0x01; // Status response type
    status_data[offset++] = ground_station_connected ? 1 : 0;
    status_data[offset++] = (uint8_t)tel_data.radio_status.rssi;
    status_data[offset++] = tel_data.radio_status.link_quality;
    
    memcpy(&status_data[offset], &tel_data.radio_status.packets_sent, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(&status_data[offset], &tel_data.radio_status.packets_received, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    memcpy(&status_data[offset], &tel_data.radio_status.packets_lost, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    
    return spi_slave->send_response(status_data, offset);
}

int8_t SAMD21TelemetryIntegration::transmit_queued_packets()
{
    int8_t packets_sent = 0;
    
    while (tx_queue.count > 0 && packets_sent < 5) { // Limit to 5 packets per cycle
        TelemetryPacket packet;
        if (dequeue_packet(packet)) {
            if (telemetry->send_packet(packet.header.packet_type, 
                                     packet.payload, 
                                     packet.header.payload_length,
                                     packet.header.priority) == 0) {
                packets_sent++;
            } else {
                // Re-queue packet for retry if transmission failed
                if (packet.retry_count < 3) {
                    packet.retry_count++;
                    enqueue_packet(packet);
                }
            }
        }
    }
    
    return packets_sent;
}

bool SAMD21TelemetryIntegration::enqueue_packet(const TelemetryPacket& packet)
{
    if (tx_queue.count >= TEL_MAX_PACKET_BUFFER) {
        return false; // Queue full
    }
    
    tx_queue.packets[tx_queue.head] = packet;
    tx_queue.head = (tx_queue.head + 1) % TEL_MAX_PACKET_BUFFER;
    tx_queue.count++;
    
    return true;
}

bool SAMD21TelemetryIntegration::dequeue_packet(TelemetryPacket& packet)
{
    if (tx_queue.count == 0) {
        return false; // Queue empty
    }
    
    packet = tx_queue.packets[tx_queue.tail];
    tx_queue.tail = (tx_queue.tail + 1) % TEL_MAX_PACKET_BUFFER;
    tx_queue.count--;
    
    return true;
}

int8_t SAMD21TelemetryIntegration::activate_emergency_mode()
{
    // Switch to emergency radio profile
    configure_radio_profile(3); // Emergency profile
    
    // Send emergency beacon
    send_emergency_beacon();
    
    // Enable continuous transmission mode
    if (telemetry) {
        telemetry->set_transmission_mode(TELEM_MODE_CONTINUOUS);
    }
    
    return 0;
}

int8_t SAMD21TelemetryIntegration::send_emergency_beacon()
{
    if (!telemetry) return -1;
    
    uint8_t emergency_data[32];
    emergency_data[0] = 0xFF; // Emergency beacon identifier
    emergency_data[1] = 0x01; // Emergency type: general emergency
    
    uint32_t timestamp = millis();
    memcpy(&emergency_data[2], &timestamp, sizeof(timestamp));
    
    return telemetry->send_emergency_packet(emergency_data, 6);
}

void SAMD21TelemetryIntegration::update_radio_status()
{
    tel_data.radio_status.radio_ready = (telemetry != nullptr && radio_configured);
    tel_data.radio_status.timestamp = millis();
    
    if (telemetry) {
        tel_data.radio_status.rssi = telemetry->get_rssi();
        tel_data.radio_status.link_quality = (uint8_t)(telemetry->get_link_quality() * 100);
    }
}

uint8_t SAMD21TelemetryIntegration::calculate_link_quality(int8_t rssi, uint32_t packet_loss_rate)
{
    // Simple link quality calculation based on RSSI and packet loss
    uint8_t rssi_quality = 0;
    
    if (rssi > -50) rssi_quality = 100;
    else if (rssi > -60) rssi_quality = 80;
    else if (rssi > -70) rssi_quality = 60;
    else if (rssi > -80) rssi_quality = 40;
    else if (rssi > -90) rssi_quality = 20;
    else rssi_quality = 0;
    
    // Reduce quality based on packet loss
    uint8_t loss_penalty = (packet_loss_rate > 100) ? 100 : packet_loss_rate;
    rssi_quality = (rssi_quality > loss_penalty) ? (rssi_quality - loss_penalty) : 0;
    
    return rssi_quality;
}

void SAMD21TelemetryIntegration::initialize_pins()
{
    // Initialize status LEDs
    pinMode(TEL_LED_RED_PIN, OUTPUT);
    pinMode(TEL_LED_GREEN_PIN, OUTPUT);
    pinMode(TEL_LED_BLUE_PIN, OUTPUT);
    
    // Initialize radio control pins
    pinMode(TEL_RADIO_ENABLE_PIN, OUTPUT);
    pinMode(TEL_RADIO_RESET_PIN, OUTPUT);
    pinMode(TEL_RADIO_CONFIG_PIN, OUTPUT);
    pinMode(TEL_RADIO_STATUS_PIN, INPUT);
    pinMode(TEL_ANTENNA_SELECT_PIN, OUTPUT);
    pinMode(TEL_PA_ENABLE_PIN, OUTPUT);
    pinMode(TEL_LNA_ENABLE_PIN, OUTPUT);
    
    // Initialize SPI pins
    pinMode(TEL_SPI_MOSI_PIN, INPUT);  // Slave MOSI is input
    pinMode(TEL_SPI_MISO_PIN, OUTPUT); // Slave MISO is output
    pinMode(TEL_SPI_SCK_PIN, INPUT);   // Slave SCK is input
    pinMode(TEL_SPI_CS_PIN, INPUT);    // Slave CS is input
    
    // Enable radio
    digitalWrite(TEL_RADIO_ENABLE_PIN, HIGH);
    digitalWrite(TEL_PA_ENABLE_PIN, HIGH);
    digitalWrite(TEL_LNA_ENABLE_PIN, HIGH);
    digitalWrite(TEL_ANTENNA_SELECT_PIN, LOW); // Select antenna 0
    
    // Reset radio
    digitalWrite(TEL_RADIO_RESET_PIN, LOW);
    delay(10);
    digitalWrite(TEL_RADIO_RESET_PIN, HIGH);
    delay(100);
}

void SAMD21TelemetryIntegration::configure_uart()
{
    // Configure Serial1 for radio communication
    Serial1.begin(57600); // High-speed UART for radio
}

void SAMD21TelemetryIntegration::initialize_radio_profiles()
{
    // Long range LoRa profile
    radio_profiles[0] = {
        .name = "Long Range",
        .frequency = 433000000,    // 433 MHz
        .power_level = 100,        // 100% power
        .data_rate = 1200,         // 1.2 kbps
        .modulation = 0,           // LoRa modulation
        .use_encryption = true,
        .use_fec = true,
        .max_range_km = 50.0f
    };
    
    // High speed profile
    radio_profiles[1] = {
        .name = "High Speed",
        .frequency = 2400000000,   // 2.4 GHz
        .power_level = 80,         // 80% power
        .data_rate = 115200,       // 115.2 kbps
        .modulation = 1,           // FSK modulation
        .use_encryption = false,
        .use_fec = false,
        .max_range_km = 5.0f
    };
    
    // Low power profile
    radio_profiles[2] = {
        .name = "Low Power",
        .frequency = 433000000,    // 433 MHz
        .power_level = 30,         // 30% power
        .data_rate = 300,          // 300 bps
        .modulation = 0,           // LoRa modulation
        .use_encryption = true,
        .use_fec = true,
        .max_range_km = 20.0f
    };
    
    // Emergency profile
    radio_profiles[3] = {
        .name = "Emergency",
        .frequency = 433000000,    // 433 MHz
        .power_level = 100,        // 100% power
        .data_rate = 600,          // 600 bps
        .modulation = 0,           // LoRa modulation
        .use_encryption = false,   // No encryption for emergency
        .use_fec = true,
        .max_range_km = 100.0f
    };
}

// Namespace implementations
namespace TelemetryUtils {
    // Radio profiles
    const RadioProfile LONG_RANGE_PROFILE = {
        .name = "Long Range",
        .frequency = 433000000,
        .power_level = 100,
        .data_rate = 1200,
        .modulation = 0,
        .use_encryption = true,
        .use_fec = true,
        .max_range_km = 50.0f
    };
    
    const RadioProfile HIGH_SPEED_PROFILE = {
        .name = "High Speed", 
        .frequency = 2400000000,
        .power_level = 80,
        .data_rate = 115200,
        .modulation = 1,
        .use_encryption = false,
        .use_fec = false,
        .max_range_km = 5.0f
    };
    
    const RadioProfile LOW_POWER_PROFILE = {
        .name = "Low Power",
        .frequency = 433000000,
        .power_level = 30,
        .data_rate = 300,
        .modulation = 0,
        .use_encryption = true,
        .use_fec = true,
        .max_range_km = 20.0f
    };
    
    const RadioProfile EMERGENCY_PROFILE = {
        .name = "Emergency",
        .frequency = 433000000,
        .power_level = 100,
        .data_rate = 600,
        .modulation = 0,
        .use_encryption = false,
        .use_fec = true,
        .max_range_km = 100.0f
    };
    
    bool is_command_safe_during_flight(GroundCommand command)
    {
        switch (command) {
            case CMD_ARM_SYSTEM:
            case CMD_DISARM_SYSTEM:
            case CMD_REQUEST_DATA:
            case CMD_SET_PARAMETER:
                return true;
                
            case CMD_DEPLOY_DROGUE:
            case CMD_DEPLOY_MAIN:
            case CMD_ABORT_FLIGHT:
            case CMD_EMERGENCY_STOP:
                return false; // These are dangerous during flight
                
            default:
                return false;
        }
    }
    
    const char* command_to_string(GroundCommand command)
    {
        switch (command) {
            case CMD_ARM_SYSTEM: return "ARM_SYSTEM";
            case CMD_DISARM_SYSTEM: return "DISARM_SYSTEM";
            case CMD_DEPLOY_DROGUE: return "DEPLOY_DROGUE";
            case CMD_DEPLOY_MAIN: return "DEPLOY_MAIN";
            case CMD_ABORT_FLIGHT: return "ABORT_FLIGHT";
            case CMD_SET_PARAMETER: return "SET_PARAMETER";
            case CMD_REQUEST_DATA: return "REQUEST_DATA";
            case CMD_START_LOGGING: return "START_LOGGING";
            case CMD_STOP_LOGGING: return "STOP_LOGGING";
            case CMD_EMERGENCY_STOP: return "EMERGENCY_STOP";
            default: return "UNKNOWN";
        }
    }
    
    LinkAnalysis analyze_link_quality(int8_t rssi, uint32_t packets_sent, uint32_t packets_lost)
    {
        LinkAnalysis analysis;
        
        analysis.signal_strength_dbm = rssi;
        analysis.noise_floor_dbm = -100.0f; // Assumed noise floor
        analysis.snr_db = rssi - analysis.noise_floor_dbm;
        
        if (analysis.snr_db > 20) analysis.link_margin_db = 20;
        else if (analysis.snr_db > 0) analysis.link_margin_db = (uint8_t)analysis.snr_db;
        else analysis.link_margin_db = 0;
        
        if (packets_sent > 0) {
            analysis.packet_error_rate = (float)packets_lost / packets_sent;
        } else {
            analysis.packet_error_rate = 0.0f;
        }
        
        analysis.link_stable = (analysis.packet_error_rate < 0.1f && analysis.snr_db > 10.0f);
        
        return analysis;
    }
}