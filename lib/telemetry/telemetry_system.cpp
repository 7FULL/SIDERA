#include "telemetry_system.h"
#include <string.h>
#include <math.h>

TelemetrySystem* g_telemetry = nullptr;

// Template configurations
namespace TelemetryUtils {
    namespace Templates {
        const TelemetryConfig LONG_RANGE_CONFIG = {
            {
                TELEM_PROTOCOL_LORA,        // protocol
                433000000,                  // frequency (433 MHz)
                80,                         // power_level (80%)
                1200,                       // data_rate (1200 bps)
                0,                          // channel
                true,                       // use_encryption
                true,                       // use_fec
                3,                          // antenna_gain (3 dBi)
                "N0CALL"                    // call_sign
            },
            TELEM_MODE_ADAPTIVE,            // transmission_mode
            2000,                           // heartbeat_interval (2s)
            5000,                           // data_interval (5s)
            5,                              // max_retries
            5000,                           // ack_timeout (5s)
            true,                           // log_packets
            false,                          // ground_station_mode
            {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 
             0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88}, // encryption_key
            50.0f,                          // max_range_km
            -120                            // rssi_threshold
        };
        
        const TelemetryConfig HIGH_SPEED_CONFIG = {
            {
                TELEM_PROTOCOL_24GHZ,       // protocol
                2400000000,                 // frequency (2.4 GHz)
                100,                        // power_level (100%)
                250000,                     // data_rate (250 kbps)
                1,                          // channel
                false,                      // use_encryption
                false,                      // use_fec
                5,                          // antenna_gain (5 dBi)
                "N0CALL"                    // call_sign
            },
            TELEM_MODE_CONTINUOUS,          // transmission_mode
            500,                            // heartbeat_interval (0.5s)
            100,                            // data_interval (0.1s)
            3,                              // max_retries
            1000,                           // ack_timeout (1s)
            false,                          // log_packets
            false,                          // ground_station_mode
            {0},                            // encryption_key (unused)
            5.0f,                           // max_range_km
            -90                             // rssi_threshold
        };
        
        const TelemetryConfig LOW_POWER_CONFIG = {
            {
                TELEM_PROTOCOL_LORA,        // protocol
                868000000,                  // frequency (868 MHz)
                20,                         // power_level (20%)
                300,                        // data_rate (300 bps)
                2,                          // channel
                true,                       // use_encryption
                true,                       // use_fec
                2,                          // antenna_gain (2 dBi)
                "N0CALL"                    // call_sign
            },
            TELEM_MODE_EVENT_DRIVEN,        // transmission_mode
            10000,                          // heartbeat_interval (10s)
            30000,                          // data_interval (30s)
            2,                              // max_retries
            10000,                          // ack_timeout (10s)
            true,                           // log_packets
            false,                          // ground_station_mode
            {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
             0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10}, // encryption_key
            100.0f,                         // max_range_km
            -130                            // rssi_threshold
        };
    }
}

TelemetrySystem::TelemetrySystem(Stream& radio_interface, uint8_t enable_pin, uint8_t reset_pin) {
    radio_serial = &radio_interface;
    radio_enable_pin = enable_pin;
    radio_reset_pin = reset_pin;
    initialized = false;
    
    // Initialize configuration
    memset(&config, 0, sizeof(TelemetryConfig));
    local_id = 1;
    ground_station_id = 0;
    
    // Initialize packet queues
    memset(tx_queue, 0, sizeof(tx_queue));
    memset(rx_queue, 0, sizeof(rx_queue));
    tx_queue_head = tx_queue_tail = 0;
    rx_queue_head = rx_queue_tail = 0;
    sequence_counter = 0;
    
    // Initialize buffers
    memset(tx_buffer, 0, sizeof(tx_buffer));
    memset(rx_buffer, 0, sizeof(rx_buffer));
    rx_buffer_pos = 0;
    
    // Initialize connection state
    connected_to_ground = false;
    last_heartbeat_time = 0;
    last_received_time = 0;
    connection_start_time = 0;
    
    // Initialize statistics
    memset(&stats, 0, sizeof(TelemetryStats));
    
    // Initialize acknowledgment tracking
    memset(pending_acks, 0, sizeof(pending_acks));
    
    // Initialize callbacks
    packet_received_callback = nullptr;
    command_received_callback = nullptr;
    connection_status_callback = nullptr;
    error_callback = nullptr;
}

int8_t TelemetrySystem::begin(const TelemetryConfig& telem_config, uint8_t system_id) {
    config = telem_config;
    local_id = system_id;
    
    // Configure hardware pins
    if (radio_enable_pin != 255) {
        pinMode(radio_enable_pin, OUTPUT);
        digitalWrite(radio_enable_pin, HIGH); // Enable radio
    }
    
    if (radio_reset_pin != 255) {
        pinMode(radio_reset_pin, OUTPUT);
        digitalWrite(radio_reset_pin, HIGH); // Release reset
        delay(100);
    }
    
    // Initialize radio module
    int8_t result = radio_init();
    if (result != TELEM_SUCCESS) {
        return result;
    }
    
    // Configure radio parameters
    result = configure_radio(config.radio);
    if (result != TELEM_SUCCESS) {
        return result;
    }
    
    // Clear buffers and queues
    clear_buffers();
    
    initialized = true;
    last_heartbeat_time = millis();
    
    return TELEM_SUCCESS;
}

int8_t TelemetrySystem::configure_radio(const RadioConfig& radio_config) {
    if (!initialized) {
        return TELEM_ERROR_INIT;
    }
    
    // Set frequency
    int8_t result = radio_set_frequency(radio_config.frequency);
    if (result != TELEM_SUCCESS) {
        return result;
    }
    
    // Set power level
    result = radio_set_power(radio_config.power_level);
    if (result != TELEM_SUCCESS) {
        return result;
    }
    
    // Configure protocol-specific parameters
    switch (radio_config.protocol) {
        case TELEM_PROTOCOL_LORA:
            // Configure LoRa-specific parameters
            // This would interface with a LoRa module like SX1276
            break;
            
        case TELEM_PROTOCOL_433MHZ:
            // Configure 433MHz ISM band parameters
            break;
            
        case TELEM_PROTOCOL_24GHZ:
            // Configure 2.4GHz parameters
            break;
            
        default:
            return TELEM_ERROR_INIT;
    }
    
    return TELEM_SUCCESS;
}

int8_t TelemetrySystem::connect_to_ground_station() {
    if (!initialized) {
        return TELEM_ERROR_INIT;
    }
    
    // Send connection request
    uint8_t connect_data[4];
    connect_data[0] = local_id;
    connect_data[1] = 0x01; // Connection request
    connect_data[2] = 0x00; // Protocol version
    connect_data[3] = config.radio.protocol;
    
    int8_t result = send_packet(TELEM_PKT_PING, connect_data, 4, TELEM_PRIORITY_HIGH);
    if (result != TELEM_SUCCESS) {
        return result;
    }
    
    // Wait for response
    uint32_t start_time = millis();
    while (millis() - start_time < 5000) { // 5 second timeout
        update();
        
        if (connected_to_ground) {
            connection_start_time = millis();
            if (connection_status_callback) {
                connection_status_callback(true);
            }
            return TELEM_SUCCESS;
        }
        
        delay(10);
    }
    
    return TELEM_ERROR_RX_TIMEOUT;
}

int8_t TelemetrySystem::send_packet(uint8_t packet_type, const void* data, uint8_t length, uint8_t priority) {
    if (!initialized) {
        return TELEM_ERROR_INIT;
    }
    
    if (length > TELEM_MAX_PAYLOAD_SIZE) {
        return TELEM_ERROR_INVALID_PKT;
    }
    
    // Build packet
    TelemetryPacket packet;
    int8_t result = build_packet(packet_type, data, length, priority, packet);
    if (result != TELEM_SUCCESS) {
        return result;
    }
    
    // Add to transmission queue
    if (!enqueue_tx_packet(packet)) {
        return TELEM_ERROR_QUEUE_FULL;
    }
    
    return TELEM_SUCCESS;
}

int8_t TelemetrySystem::send_flight_data(const FlightTelemetryData& data) {
    return send_packet(TELEM_PKT_FLIGHT_DATA, &data, sizeof(data), TELEM_PRIORITY_HIGH);
}

int8_t TelemetrySystem::send_gps_data(const GPSTelemetryData& data) {
    return send_packet(TELEM_PKT_GPS_DATA, &data, sizeof(data), TELEM_PRIORITY_NORMAL);
}

int8_t TelemetrySystem::send_sensor_data(const SensorTelemetryData& data) {
    return send_packet(TELEM_PKT_SENSOR_DATA, &data, sizeof(data), TELEM_PRIORITY_NORMAL);
}

int8_t TelemetrySystem::send_system_status(const SystemStatusData& data) {
    return send_packet(TELEM_PKT_SYSTEM_STATUS, &data, sizeof(data), TELEM_PRIORITY_NORMAL);
}

int8_t TelemetrySystem::send_error_report(uint16_t error_code, const char* description) {
    struct ErrorPacket {
        uint16_t error_code;
        uint32_t timestamp;
        char description[32];
    } error_packet;
    
    error_packet.error_code = error_code;
    error_packet.timestamp = millis();
    strncpy(error_packet.description, description, sizeof(error_packet.description) - 1);
    error_packet.description[sizeof(error_packet.description) - 1] = '\0';
    
    return send_packet(TELEM_PKT_ERROR_REPORT, &error_packet, sizeof(error_packet), TELEM_PRIORITY_HIGH);
}

int8_t TelemetrySystem::send_emergency_packet(const void* data, uint8_t length) {
    // Emergency packets bypass normal queueing and are sent immediately
    TelemetryPacket packet;
    int8_t result = build_packet(TELEM_PKT_EMERGENCY, data, length, TELEM_PRIORITY_CRITICAL, packet);
    if (result != TELEM_SUCCESS) {
        return result;
    }
    
    // Send immediately without queueing
    uint8_t tx_data[TELEM_MAX_PACKET_SIZE];
    uint8_t tx_length = TELEM_HEADER_SIZE + packet.header.payload_length;
    
    // Copy header
    memcpy(tx_data, &packet.header, TELEM_HEADER_SIZE);
    
    // Copy payload
    memcpy(&tx_data[TELEM_HEADER_SIZE], packet.payload, packet.header.payload_length);
    
    return radio_send(tx_data, tx_length);
}

int8_t TelemetrySystem::send_heartbeat() {
    struct HeartbeatData {
        uint32_t timestamp;
        uint8_t system_id;
        uint8_t status;
        uint16_t sequence;
        float battery_voltage;
        int8_t rssi;
    } heartbeat;
    
    heartbeat.timestamp = millis();
    heartbeat.system_id = local_id;
    heartbeat.status = connected_to_ground ? 0x01 : 0x00;
    heartbeat.sequence = sequence_counter;
    heartbeat.battery_voltage = 12.0f; // Would get from battery monitor
    heartbeat.rssi = get_rssi();
    
    return send_packet(TELEM_PKT_HEARTBEAT, &heartbeat, sizeof(heartbeat), TELEM_PRIORITY_LOW);
}

void TelemetrySystem::update() {
    if (!initialized) {
        return;
    }
    
    // Process incoming data
    parse_received_data();
    
    // Process transmission queue
    process_tx_queue();
    
    // Handle pending acknowledgments
    process_pending_acks();
    
    // Update connection status
    update_connection_status();
    
    // Send periodic heartbeat
    if (millis() - last_heartbeat_time > config.heartbeat_interval) {
        send_heartbeat();
        last_heartbeat_time = millis();
    }
    
    // Update statistics
    update_statistics();
}

bool TelemetrySystem::has_pending_packets() const {
    return rx_queue_head != rx_queue_tail;
}

TelemetryPacket TelemetrySystem::get_next_packet() {
    TelemetryPacket packet;
    memset(&packet, 0, sizeof(packet));
    
    if (has_pending_packets()) {
        dequeue_rx_packet(packet);
    }
    
    return packet;
}

int8_t TelemetrySystem::get_rssi() const {
    // This would read RSSI from the radio module
    // Implementation depends on the specific radio hardware
    return -80; // Placeholder value
}

float TelemetrySystem::get_link_quality() const {
    if (stats.packets_sent == 0) {
        return 0.0f;
    }
    
    return stats.packet_success_rate;
}

void TelemetrySystem::print_statistics() const {
    Serial.println("=== Telemetry Statistics ===");
    Serial.print("Packets Sent: "); Serial.println(stats.packets_sent);
    Serial.print("Packets Received: "); Serial.println(stats.packets_received);
    Serial.print("Packets Lost: "); Serial.println(stats.packets_lost);
    Serial.print("Success Rate: "); Serial.print(stats.packet_success_rate, 1); Serial.println("%");
    Serial.print("Checksum Errors: "); Serial.println(stats.checksum_errors);
    Serial.print("Retransmissions: "); Serial.println(stats.retransmissions);
    Serial.print("Average RSSI: "); Serial.print(stats.avg_rssi); Serial.println(" dBm");
    Serial.print("Bytes Sent: "); Serial.println(stats.total_bytes_sent);
    Serial.print("Bytes Received: "); Serial.println(stats.total_bytes_received);
}

void TelemetrySystem::print_connection_status() const {
    Serial.println("=== Connection Status ===");
    Serial.print("Connected: "); Serial.println(connected_to_ground ? "Yes" : "No");
    Serial.print("Ground Station ID: "); Serial.println(ground_station_id);
    Serial.print("Local ID: "); Serial.println(local_id);
    Serial.print("RSSI: "); Serial.print(get_rssi()); Serial.println(" dBm");
    Serial.print("Link Quality: "); Serial.print(get_link_quality(), 1); Serial.println("%");
    
    if (connected_to_ground) {
        uint32_t connection_time = millis() - connection_start_time;
        Serial.print("Connection Time: "); Serial.print(connection_time / 1000); Serial.println(" seconds");
    }
}

// Private function implementations
int8_t TelemetrySystem::radio_init() {
    // Initialize radio module
    // This is hardware-specific and would depend on the radio module used
    
    // For example, with a LoRa module:
    // - Send reset command
    // - Wait for ready response
    // - Configure basic parameters
    
    delay(1000); // Wait for radio to initialize
    
    // Test communication with radio
    if (!radio_serial->available()) {
        // Try to wake up radio
        radio_serial->print("AT\r\n");
        delay(100);
    }
    
    // Check if radio responds
    uint32_t start_time = millis();
    while (millis() - start_time < 2000) {
        if (radio_serial->available()) {
            String response = radio_serial->readString();
            if (response.indexOf("OK") >= 0) {
                return TELEM_SUCCESS;
            }
        }
        delay(10);
    }
    
    return TELEM_ERROR_NO_RADIO;
}

int8_t TelemetrySystem::radio_send(const uint8_t* data, uint8_t length) {
    if (!radio_serial) {
        return TELEM_ERROR_NO_RADIO;
    }
    
    // Send data via radio
    // This implementation depends on the radio module protocol
    
    // For AT command-based radios:
    radio_serial->print("AT+SEND=");
    radio_serial->print(length);
    radio_serial->print(",");
    
    for (uint8_t i = 0; i < length; i++) {
        if (data[i] < 16) radio_serial->print("0");
        radio_serial->print(data[i], HEX);
    }
    radio_serial->print("\r\n");
    
    // Wait for transmission confirmation
    uint32_t start_time = millis();
    while (millis() - start_time < 1000) {
        if (radio_serial->available()) {
            String response = radio_serial->readString();
            if (response.indexOf("OK") >= 0) {
                stats.packets_sent++;
                stats.total_bytes_sent += length;
                return TELEM_SUCCESS;
            }
            if (response.indexOf("ERROR") >= 0) {
                return TELEM_ERROR_TX_FAILED;
            }
        }
        delay(10);
    }
    
    return TELEM_ERROR_TIMEOUT;
}

int8_t TelemetrySystem::radio_set_frequency(uint32_t frequency) {
    // Set radio frequency
    // Implementation depends on radio module
    
    radio_serial->print("AT+FREQ=");
    radio_serial->print(frequency);
    radio_serial->print("\r\n");
    
    delay(100);
    
    return TELEM_SUCCESS;
}

int8_t TelemetrySystem::radio_set_power(uint8_t power_level) {
    // Set transmission power
    // Implementation depends on radio module
    
    radio_serial->print("AT+PWR=");
    radio_serial->print(power_level);
    radio_serial->print("\r\n");
    
    delay(100);
    
    return TELEM_SUCCESS;
}

int8_t TelemetrySystem::build_packet(uint8_t packet_type, const void* data, uint8_t length, uint8_t priority, TelemetryPacket& packet) {
    // Clear packet
    memset(&packet, 0, sizeof(packet));
    
    // Build header
    packet.header.sync_bytes[0] = 0x55;
    packet.header.sync_bytes[1] = 0xAA;
    packet.header.packet_type = packet_type;
    packet.header.source_id = local_id;
    packet.header.destination_id = ground_station_id;
    packet.header.sequence_number = get_next_sequence_number();
    packet.header.priority = priority;
    packet.header.flags = 0;
    packet.header.payload_length = length;
    packet.header.encryption_type = config.radio.use_encryption ? TELEM_ENCRYPTION_XOR : TELEM_ENCRYPTION_NONE;
    
    // Copy payload
    if (data && length > 0) {
        memcpy(packet.payload, data, length);
        
        // Apply encryption if enabled
        if (config.radio.use_encryption) {
            encrypt_payload(packet.payload, length);
        }
    }
    
    // Calculate checksum
    uint8_t checksum_data[TELEM_HEADER_SIZE + length];
    memcpy(checksum_data, &packet.header, TELEM_HEADER_SIZE - 2); // Exclude checksum field
    memcpy(&checksum_data[TELEM_HEADER_SIZE - 2], packet.payload, length);
    
    packet.header.checksum = calculate_checksum(checksum_data, TELEM_HEADER_SIZE - 2 + length);
    
    // Set timestamp
    packet.timestamp = millis();
    packet.retry_count = 0;
    packet.acknowledged = false;
    
    return TELEM_SUCCESS;
}

int8_t TelemetrySystem::parse_received_data() {
    if (!radio_serial->available()) {
        return TELEM_SUCCESS;
    }
    
    // Read available data into buffer
    while (radio_serial->available() && rx_buffer_pos < TELEM_RX_BUFFER_SIZE - 1) {
        rx_buffer[rx_buffer_pos++] = radio_serial->read();
    }
    
    // Look for packet sync bytes
    for (uint16_t i = 0; i < rx_buffer_pos - 1; i++) {
        if (rx_buffer[i] == 0x55 && rx_buffer[i + 1] == 0xAA) {
            // Found potential packet start
            if (rx_buffer_pos - i >= TELEM_HEADER_SIZE) {
                // We have at least a header
                TelemetryPacketHeader* header = (TelemetryPacketHeader*)&rx_buffer[i];
                
                uint16_t total_packet_size = TELEM_HEADER_SIZE + header->payload_length;
                
                if (rx_buffer_pos - i >= total_packet_size) {
                    // We have a complete packet
                    TelemetryPacket packet;
                    
                    // Copy header
                    memcpy(&packet.header, header, TELEM_HEADER_SIZE);
                    
                    // Copy payload
                    memcpy(packet.payload, &rx_buffer[i + TELEM_HEADER_SIZE], header->payload_length);
                    
                    // Set reception info
                    packet.timestamp = millis();
                    packet.rssi = get_rssi();
                    
                    // Validate packet
                    if (validate_packet(packet)) {
                        // Decrypt payload if needed
                        if (packet.header.encryption_type != TELEM_ENCRYPTION_NONE) {
                            decrypt_payload(packet.payload, packet.header.payload_length);
                        }
                        
                        // Process packet
                        if (is_packet_for_us(packet) || is_broadcast_packet(packet)) {
                            enqueue_rx_packet(packet);
                            
                            // Send ACK if required
                            if (packet.header.flags & 0x01) { // ACK required flag
                                send_acknowledgment(packet.header.sequence_number, true);
                            }
                            
                            // Update connection status
                            if (packet.header.source_id == ground_station_id) {
                                connected_to_ground = true;
                                last_received_time = millis();
                            }
                            
                            // Call callback
                            if (packet_received_callback) {
                                packet_received_callback(packet);
                            }
                            
                            stats.packets_received++;
                            stats.total_bytes_received += total_packet_size;
                        }
                    } else {
                        stats.checksum_errors++;
                    }
                    
                    // Remove processed data from buffer
                    memmove(rx_buffer, &rx_buffer[i + total_packet_size], rx_buffer_pos - i - total_packet_size);
                    rx_buffer_pos -= (i + total_packet_size);
                    return TELEM_SUCCESS;
                }
            }
        }
    }
    
    // Clean up buffer if it's getting full
    if (rx_buffer_pos > TELEM_RX_BUFFER_SIZE - 100) {
        rx_buffer_pos = 0;
    }
    
    return TELEM_SUCCESS;
}

bool TelemetrySystem::validate_packet(const TelemetryPacket& packet) {
    // Check sync bytes
    if (packet.header.sync_bytes[0] != 0x55 || packet.header.sync_bytes[1] != 0xAA) {
        return false;
    }
    
    // Check payload length
    if (packet.header.payload_length > TELEM_MAX_PAYLOAD_SIZE) {
        return false;
    }
    
    // Verify checksum
    uint8_t checksum_data[TELEM_HEADER_SIZE + packet.header.payload_length];
    memcpy(checksum_data, &packet.header, TELEM_HEADER_SIZE - 2); // Exclude checksum field
    memcpy(&checksum_data[TELEM_HEADER_SIZE - 2], packet.payload, packet.header.payload_length);
    
    uint16_t calculated_checksum = calculate_checksum(checksum_data, TELEM_HEADER_SIZE - 2 + packet.header.payload_length);
    
    return calculated_checksum == packet.header.checksum;
}

uint16_t TelemetrySystem::calculate_checksum(const uint8_t* data, uint8_t length) {
    uint16_t checksum = 0;
    
    for (uint8_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    
    return checksum;
}

bool TelemetrySystem::enqueue_tx_packet(const TelemetryPacket& packet) {
    uint8_t next_head = (tx_queue_head + 1) % TELEM_PACKET_QUEUE_SIZE;
    
    if (next_head == tx_queue_tail) {
        return false; // Queue full
    }
    
    tx_queue[tx_queue_head] = packet;
    tx_queue_head = next_head;
    
    return true;
}

bool TelemetrySystem::dequeue_tx_packet(TelemetryPacket& packet) {
    if (tx_queue_head == tx_queue_tail) {
        return false; // Queue empty
    }
    
    packet = tx_queue[tx_queue_tail];
    tx_queue_tail = (tx_queue_tail + 1) % TELEM_PACKET_QUEUE_SIZE;
    
    return true;
}

bool TelemetrySystem::enqueue_rx_packet(const TelemetryPacket& packet) {
    uint8_t next_head = (rx_queue_head + 1) % TELEM_PACKET_QUEUE_SIZE;
    
    if (next_head == rx_queue_tail) {
        return false; // Queue full
    }
    
    rx_queue[rx_queue_head] = packet;
    rx_queue_head = next_head;
    
    return true;
}

bool TelemetrySystem::dequeue_rx_packet(TelemetryPacket& packet) {
    if (rx_queue_head == rx_queue_tail) {
        return false; // Queue empty
    }
    
    packet = rx_queue[rx_queue_tail];
    rx_queue_tail = (rx_queue_tail + 1) % TELEM_PACKET_QUEUE_SIZE;
    
    return true;
}

void TelemetrySystem::process_tx_queue() {
    static uint32_t last_tx_time = 0;
    
    if (tx_queue_head == tx_queue_tail) {
        return; // Queue empty
    }
    
    // Respect transmission interval
    if (millis() - last_tx_time < config.data_interval) {
        return;
    }
    
    TelemetryPacket packet;
    if (dequeue_tx_packet(packet)) {
        // Prepare transmission data
        uint8_t tx_data[TELEM_MAX_PACKET_SIZE];
        uint8_t tx_length = TELEM_HEADER_SIZE + packet.header.payload_length;
        
        // Copy header
        memcpy(tx_data, &packet.header, TELEM_HEADER_SIZE);
        
        // Copy payload
        memcpy(&tx_data[TELEM_HEADER_SIZE], packet.payload, packet.header.payload_length);
        
        // Send packet
        int8_t result = radio_send(tx_data, tx_length);
        
        if (result == TELEM_SUCCESS) {
            last_tx_time = millis();
            
            // Add to pending ACK list if ACK is required
            if (packet.header.flags & 0x01) {
                add_pending_ack(packet.header.sequence_number);
            }
        } else {
            // Re-queue packet for retry if not exceeded max retries
            packet.retry_count++;
            if (packet.retry_count < config.max_retries) {
                enqueue_tx_packet(packet);
            } else {
                stats.packets_lost++;
            }
        }
    }
}

void TelemetrySystem::add_pending_ack(uint8_t sequence_number) {
    for (uint8_t i = 0; i < 16; i++) {
        if (!pending_acks[i].active) {
            pending_acks[i].sequence_number = sequence_number;
            pending_acks[i].timestamp = millis();
            pending_acks[i].retry_count = 0;
            pending_acks[i].active = true;
            break;
        }
    }
}

void TelemetrySystem::process_pending_acks() {
    for (uint8_t i = 0; i < 16; i++) {
        if (pending_acks[i].active) {
            if (millis() - pending_acks[i].timestamp > config.ack_timeout) {
                // ACK timeout
                pending_acks[i].retry_count++;
                
                if (pending_acks[i].retry_count >= config.max_retries) {
                    // Give up
                    pending_acks[i].active = false;
                    stats.packets_lost++;
                } else {
                    // Retry (would need to re-queue original packet)
                    pending_acks[i].timestamp = millis();
                    stats.retransmissions++;
                }
            }
        }
    }
}

int8_t TelemetrySystem::send_acknowledgment(uint8_t sequence_number, bool positive) {
    uint8_t ack_data[2];
    ack_data[0] = sequence_number;
    ack_data[1] = positive ? 0x01 : 0x00;
    
    uint8_t packet_type = positive ? TELEM_PKT_ACK : TELEM_PKT_NACK;
    
    return send_packet(packet_type, ack_data, 2, TELEM_PRIORITY_HIGH);
}

int8_t TelemetrySystem::encrypt_payload(uint8_t* data, uint8_t length) {
    if (!config.radio.use_encryption) {
        return TELEM_SUCCESS;
    }
    
    // Simple XOR encryption
    for (uint8_t i = 0; i < length; i++) {
        data[i] ^= config.encryption_key[i % 16];
    }
    
    return TELEM_SUCCESS;
}

int8_t TelemetrySystem::decrypt_payload(uint8_t* data, uint8_t length) {
    // XOR encryption is symmetric, so decryption is the same as encryption
    return encrypt_payload(data, length);
}

void TelemetrySystem::update_connection_status() {
    if (connected_to_ground) {
        if (millis() - last_received_time > TELEM_CONN_TIMEOUT) {
            connected_to_ground = false;
            if (connection_status_callback) {
                connection_status_callback(false);
            }
        }
    }
}

void TelemetrySystem::clear_buffers() {
    rx_buffer_pos = 0;
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

uint8_t TelemetrySystem::get_next_sequence_number() {
    return ++sequence_counter;
}

bool TelemetrySystem::is_broadcast_packet(const TelemetryPacket& packet) {
    return packet.header.destination_id == 0xFF;
}

bool TelemetrySystem::is_packet_for_us(const TelemetryPacket& packet) {
    return packet.header.destination_id == local_id;
}

void TelemetrySystem::update_statistics() {
    if (stats.packets_sent > 0) {
        stats.packet_success_rate = ((float)(stats.packets_sent - stats.packets_lost) / stats.packets_sent) * 100.0f;
    }
    
    // Update average RSSI (simple moving average)
    int8_t current_rssi = get_rssi();
    stats.avg_rssi = (stats.avg_rssi * 0.9f) + (current_rssi * 0.1f);
}

// Utility function implementations
namespace TelemetryUtils {
    void format_flight_data_string(const FlightTelemetryData& data, char* buffer, size_t buffer_size) {
        snprintf(buffer, buffer_size, 
                "T:%lu,ALT:%.1f,VEL:%.1f,ACC:%.2f,ST:%d,BAT:%.2f,GPS:%d,ERR:%04X",
                data.timestamp, data.altitude, data.velocity, data.acceleration,
                data.flight_state, data.battery_voltage, data.gps_satellites, data.errors);
    }
    
    void format_gps_data_string(const GPSTelemetryData& data, char* buffer, size_t buffer_size) {
        snprintf(buffer, buffer_size,
                "T:%lu,LAT:%.8f,LON:%.8f,ALT:%.1f,SPD:%.1f,CRS:%.1f,SAT:%d,FIX:%d",
                data.timestamp, data.latitude, data.longitude, data.altitude_gps,
                data.speed, data.course, data.satellites, data.fix_type);
    }
    
    float calculate_theoretical_range(float tx_power_dbm, float rx_sensitivity_dbm, float frequency_mhz, float antenna_gain_db) {
        // Friis transmission equation
        float path_loss_db = tx_power_dbm - rx_sensitivity_dbm + 2 * antenna_gain_db;
        
        // Free space path loss: PL(dB) = 20*log10(d) + 20*log10(f) + 32.45
        // Solve for d: d = 10^((PL - 20*log10(f) - 32.45) / 20)
        
        float log_distance = (path_loss_db - 20 * log10(frequency_mhz) - 32.45) / 20.0;
        float distance_km = pow(10, log_distance);
        
        return distance_km;
    }
    
    const char* packet_type_to_string(uint8_t packet_type) {
        switch (packet_type) {
            case TELEM_PKT_HEARTBEAT: return "Heartbeat";
            case TELEM_PKT_FLIGHT_DATA: return "Flight Data";
            case TELEM_PKT_GPS_DATA: return "GPS Data";
            case TELEM_PKT_SENSOR_DATA: return "Sensor Data";
            case TELEM_PKT_SYSTEM_STATUS: return "System Status";
            case TELEM_PKT_ERROR_REPORT: return "Error Report";
            case TELEM_PKT_COMMAND: return "Command";
            case TELEM_PKT_CONFIG: return "Configuration";
            case TELEM_PKT_ACK: return "Acknowledgment";
            case TELEM_PKT_NACK: return "Negative ACK";
            case TELEM_PKT_PING: return "Ping";
            case TELEM_PKT_EMERGENCY: return "Emergency";
            default: return "Unknown";
        }
    }
    
    const char* command_id_to_string(uint8_t command_id) {
        switch (command_id) {
            case TELEM_CMD_ARM_SYSTEM: return "Arm System";
            case TELEM_CMD_DISARM_SYSTEM: return "Disarm System";
            case TELEM_CMD_DEPLOY_DROGUE: return "Deploy Drogue";
            case TELEM_CMD_DEPLOY_MAIN: return "Deploy Main";
            case TELEM_CMD_ABORT_FLIGHT: return "Abort Flight";
            case TELEM_CMD_RESET_SYSTEM: return "Reset System";
            case TELEM_CMD_SET_CONFIG: return "Set Config";
            case TELEM_CMD_GET_CONFIG: return "Get Config";
            case TELEM_CMD_START_LOGGING: return "Start Logging";
            case TELEM_CMD_STOP_LOGGING: return "Stop Logging";
            default: return "Unknown Command";
        }
    }
    
    bool validate_flight_data(const FlightTelemetryData& data) {
        // Basic sanity checks
        if (data.altitude < -1000.0f || data.altitude > 100000.0f) return false;
        if (data.velocity < -1000.0f || data.velocity > 1000.0f) return false;
        if (data.acceleration < -50.0f || data.acceleration > 50.0f) return false;
        if (data.battery_voltage < 0.0f || data.battery_voltage > 20.0f) return false;
        if (data.gps_satellites > 50) return false;
        
        return true;
    }
}