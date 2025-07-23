#include "spi_slave.h"

SPISlave* g_spi_slave_instance = nullptr;

SPISlave::SPISlave(uint8_t dev_id, uint8_t cs) {
    device_id = dev_id;
    cs_pin = cs;
    packet_received = false;
    transaction_active = false;
    
    memset(command_handlers, 0, sizeof(command_handlers));
    memset(&device_status, 0, sizeof(device_status));
    
    device_status.device_status = STATUS_OK;
    device_status.uptime = 0;
    device_status.error_count = 0;
    
    g_spi_slave_instance = this;
}

void SPISlave::begin() {
    pinMode(cs_pin, INPUT_PULLUP);
    
    SPI.begin();
    SPI.usingInterrupt(digitalPinToInterrupt(cs_pin));
    
    attachInterrupt(digitalPinToInterrupt(cs_pin), cs_interrupt_handler, FALLING);
    
    register_command_handler(CMD_PING, [](const spi_packet_t* rx, spi_packet_t* tx) {
        g_spi_slave_instance->handle_ping(rx, tx);
    });
    
    register_command_handler(CMD_STATUS_REQUEST, [](const spi_packet_t* rx, spi_packet_t* tx) {
        g_spi_slave_instance->handle_status_request(rx, tx);
    });
    
    register_command_handler(CMD_EMERGENCY_STOP, [](const spi_packet_t* rx, spi_packet_t* tx) {
        g_spi_slave_instance->handle_emergency_stop(rx, tx);
    });
}

void SPISlave::cs_interrupt_handler() {
    if (g_spi_slave_instance) {
        g_spi_slave_instance->handle_cs_interrupt();
    }
}

void SPISlave::handle_cs_interrupt() {
    if (digitalRead(cs_pin) == LOW) {
        transaction_active = true;
        
        uint8_t* rx_data = (uint8_t*)&rx_buffer;
        uint8_t* tx_data = (uint8_t*)&tx_buffer;
        
        for (int i = 0; i < SPI_PACKET_SIZE; i++) {
            while (!(SPSR & (1 << SPIF)));
            rx_data[i] = SPDR;
            SPDR = tx_data[i];
        }
        
        packet_received = true;
        transaction_active = false;
    }
}

void SPISlave::update() {
    if (packet_received) {
        process_received_packet();
        packet_received = false;
    }
    
    static unsigned long last_uptime_update = 0;
    if (millis() - last_uptime_update >= 1000) {
        device_status.uptime++;
        last_uptime_update = millis();
    }
}

void SPISlave::process_received_packet() {
    if (!validate_packet(&rx_buffer)) {
        increment_error_count();
        prepare_default_response(RESP_ERROR);
        return;
    }
    
    device_status.last_command = rx_buffer.command;
    
    if (command_handlers[rx_buffer.command]) {
        command_handlers[rx_buffer.command](&rx_buffer, &tx_buffer);
    } else {
        prepare_default_response(RESP_NACK);
    }
}

void SPISlave::register_command_handler(uint8_t command, command_handler_t handler) {
    command_handlers[command] = handler;
}

void SPISlave::set_device_status(uint8_t status) {
    device_status.device_status = status;
}

void SPISlave::increment_error_count() {
    device_status.error_count++;
}

bool SPISlave::is_transaction_active() {
    return transaction_active;
}

void SPISlave::prepare_default_response(uint8_t response_type) {
    init_packet(&tx_buffer, device_id, response_type);
}

void SPISlave::prepare_response_packet(uint8_t command, const void* data, uint8_t length) {
    init_packet(&tx_buffer, device_id, command);
    
    if (data && length > 0 && length <= SPI_DATA_SIZE) {
        memcpy(tx_buffer.data, data, length);
        tx_buffer.length = length;
    }
}

void SPISlave::handle_ping(const spi_packet_t* rx_packet, spi_packet_t* tx_packet) {
    prepare_default_response(RESP_ACK);
}

void SPISlave::handle_status_request(const spi_packet_t* rx_packet, spi_packet_t* tx_packet) {
    prepare_response_packet(RESP_STATUS, &device_status, sizeof(status_response_t));
}

void SPISlave::handle_emergency_stop(const spi_packet_t* rx_packet, spi_packet_t* tx_packet) {
    set_device_status(STATUS_ERROR);
    prepare_default_response(RESP_ACK);
}

void SPISlave::handle_servo_control(const spi_packet_t* rx_packet, spi_packet_t* tx_packet) {
    if (device_id == DEVICE_NAVIGATION && rx_packet->command == CMD_SERVO_CONTROL) {
        servo_control_t servo_data;
        unpack_servo_control(rx_packet, &servo_data);
        prepare_default_response(RESP_ACK);
    } else {
        prepare_default_response(RESP_NACK);
    }
}

void SPISlave::handle_imu_request(const spi_packet_t* rx_packet, spi_packet_t* tx_packet) {
    if (device_id == DEVICE_NAVIGATION && rx_packet->command == CMD_DATA_SEND) {
        imu_data_t imu_data;
        memset(&imu_data, 0, sizeof(imu_data_t));
        pack_imu_data(tx_packet, &imu_data);
    } else {
        prepare_default_response(RESP_NACK);
    }
}

void SPISlave::handle_telemetry_send(const spi_packet_t* rx_packet, spi_packet_t* tx_packet) {
    if (device_id == DEVICE_TELEMETRY && rx_packet->command == CMD_TELEMETRY_SEND) {
        telemetry_data_t telemetry_data;
        unpack_telemetry_data(rx_packet, &telemetry_data);
        prepare_default_response(RESP_ACK);
    } else {
        prepare_default_response(RESP_NACK);
    }
}