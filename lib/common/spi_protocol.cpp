#include "spi_protocol.h"
#include <string.h>

uint8_t calculate_checksum(const spi_packet_t* packet) {
    uint8_t checksum = 0;
    const uint8_t* data = (const uint8_t*)packet;
    
    for (int i = 0; i < SPI_PACKET_SIZE - 1; i++) {
        checksum ^= data[i];
    }
    
    return checksum;
}

bool validate_packet(const spi_packet_t* packet) {
    if (packet->start_byte != PROTOCOL_START_BYTE) {
        return false;
    }
    
    if (packet->length > SPI_DATA_SIZE) {
        return false;
    }
    
    return true;
}

void init_packet(spi_packet_t* packet, uint8_t device_id, uint8_t command) {
    memset(packet, 0, sizeof(spi_packet_t));
    packet->start_byte = PROTOCOL_START_BYTE;
    packet->device_id = device_id;
    packet->command = command;
    packet->length = 0;
}

void pack_imu_data(spi_packet_t* packet, const imu_data_t* imu_data) {
    init_packet(packet, DEVICE_NAVIGATION, CMD_IMU_DATA);
    memcpy(packet->data, imu_data, sizeof(imu_data_t));
    packet->length = sizeof(imu_data_t);
}

void unpack_imu_data(const spi_packet_t* packet, imu_data_t* imu_data) {
    if (packet->command == CMD_IMU_DATA && packet->length >= sizeof(imu_data_t)) {
        memcpy(imu_data, packet->data, sizeof(imu_data_t));
    }
}

void pack_servo_control(spi_packet_t* packet, const servo_control_t* servo_data) {
    init_packet(packet, DEVICE_MASTER, CMD_SERVO_CONTROL);
    memcpy(packet->data, servo_data, sizeof(servo_control_t));
    packet->length = sizeof(servo_control_t);
}

void unpack_servo_control(const spi_packet_t* packet, servo_control_t* servo_data) {
    if (packet->command == CMD_SERVO_CONTROL && packet->length >= sizeof(servo_control_t)) {
        memcpy(servo_data, packet->data, sizeof(servo_control_t));
    }
}

void pack_telemetry_data(spi_packet_t* packet, const telemetry_data_t* telemetry_data) {
    init_packet(packet, DEVICE_MASTER, CMD_TELEMETRY_SEND);
    memcpy(packet->data, telemetry_data, sizeof(telemetry_data_t));
    packet->length = sizeof(telemetry_data_t);
}

void unpack_telemetry_data(const spi_packet_t* packet, telemetry_data_t* telemetry_data) {
    if (packet->command == CMD_TELEMETRY_SEND && packet->length >= sizeof(telemetry_data_t)) {
        memcpy(telemetry_data, packet->data, sizeof(telemetry_data_t));
    }
}