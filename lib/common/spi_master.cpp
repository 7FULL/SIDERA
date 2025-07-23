#include "spi_master.h"

SPIMaster::SPIMaster(uint8_t navigation_cs, uint8_t telemetry_cs, uint32_t frequency) {
    nav_cs_pin = navigation_cs;
    tel_cs_pin = telemetry_cs;
    spi_frequency = frequency;
}

void SPIMaster::begin() {
    SPI.begin();
    
    pinMode(nav_cs_pin, OUTPUT);
    pinMode(tel_cs_pin, OUTPUT);
    
    digitalWrite(nav_cs_pin, HIGH);
    digitalWrite(tel_cs_pin, HIGH);
}

uint8_t SPIMaster::get_cs_pin(uint8_t device_id) {
    switch (device_id) {
        case DEVICE_NAVIGATION:
            return nav_cs_pin;
        case DEVICE_TELEMETRY:
            return tel_cs_pin;
        default:
            return 0;
    }
}

void SPIMaster::select_device(uint8_t device_id) {
    uint8_t cs_pin = get_cs_pin(device_id);
    if (cs_pin > 0) {
        digitalWrite(cs_pin, LOW);
        delayMicroseconds(10);
    }
}

void SPIMaster::deselect_device(uint8_t device_id) {
    uint8_t cs_pin = get_cs_pin(device_id);
    if (cs_pin > 0) {
        digitalWrite(cs_pin, HIGH);
        delayMicroseconds(10);
    }
}

void SPIMaster::transfer_packet(const spi_packet_t* tx_packet, spi_packet_t* rx_packet) {
    SPI.beginTransaction(SPISettings(spi_frequency, MSBFIRST, SPI_MODE0));
    
    uint8_t* tx_data = (uint8_t*)tx_packet;
    uint8_t* rx_data = (uint8_t*)rx_packet;
    
    for (int i = 0; i < SPI_PACKET_SIZE; i++) {
        rx_data[i] = SPI.transfer(tx_data[i]);
    }
    
    SPI.endTransaction();
}

bool SPIMaster::send_packet(uint8_t device_id, const spi_packet_t* packet) {
    if (!validate_packet(packet)) {
        return false;
    }
    
    select_device(device_id);
    
    spi_packet_t rx_packet;
    transfer_packet(packet, &rx_packet);
    
    deselect_device(device_id);
    
    return rx_packet.command == RESP_ACK;
}

bool SPIMaster::receive_packet(uint8_t device_id, spi_packet_t* packet) {
    spi_packet_t tx_packet;
    init_packet(&tx_packet, DEVICE_MASTER, CMD_STATUS_REQUEST);
    
    select_device(device_id);
    transfer_packet(&tx_packet, packet);
    deselect_device(device_id);
    
    return validate_packet(packet);
}

bool SPIMaster::transaction(uint8_t device_id, const spi_packet_t* tx_packet, spi_packet_t* rx_packet) {
    if (!validate_packet(tx_packet)) {
        return false;
    }
    
    select_device(device_id);
    transfer_packet(tx_packet, rx_packet);
    deselect_device(device_id);
    
    return validate_packet(rx_packet);
}

bool SPIMaster::ping_device(uint8_t device_id) {
    spi_packet_t tx_packet, rx_packet;
    init_packet(&tx_packet, DEVICE_MASTER, CMD_PING);
    
    return transaction(device_id, &tx_packet, &rx_packet) && 
           rx_packet.command == RESP_ACK;
}

bool SPIMaster::request_status(uint8_t device_id, status_response_t* status) {
    spi_packet_t tx_packet, rx_packet;
    init_packet(&tx_packet, DEVICE_MASTER, CMD_STATUS_REQUEST);
    
    if (transaction(device_id, &tx_packet, &rx_packet) && 
        rx_packet.command == RESP_STATUS) {
        memcpy(status, rx_packet.data, sizeof(status_response_t));
        return true;
    }
    
    return false;
}

bool SPIMaster::send_servo_control(const servo_control_t* servo_data) {
    spi_packet_t packet;
    pack_servo_control(&packet, servo_data);
    return send_packet(DEVICE_NAVIGATION, &packet);
}

bool SPIMaster::request_imu_data(imu_data_t* imu_data) {
    spi_packet_t tx_packet, rx_packet;
    init_packet(&tx_packet, DEVICE_MASTER, CMD_DATA_SEND);
    
    if (transaction(DEVICE_NAVIGATION, &tx_packet, &rx_packet) && 
        rx_packet.command == CMD_IMU_DATA) {
        unpack_imu_data(&rx_packet, imu_data);
        return true;
    }
    
    return false;
}

bool SPIMaster::send_telemetry_data(const telemetry_data_t* telemetry_data) {
    spi_packet_t packet;
    pack_telemetry_data(&packet, telemetry_data);
    return send_packet(DEVICE_TELEMETRY, &packet);
}

bool SPIMaster::emergency_stop_all() {
    spi_packet_t packet;
    init_packet(&packet, DEVICE_MASTER, CMD_EMERGENCY_STOP);
    
    bool nav_stop = send_packet(DEVICE_NAVIGATION, &packet);
    bool tel_stop = send_packet(DEVICE_TELEMETRY, &packet);
    
    return nav_stop && tel_stop;
}