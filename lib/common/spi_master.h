#ifndef SPI_MASTER_H
#define SPI_MASTER_H

#include "spi_protocol.h"
#include <Arduino.h>
#include <SPI.h>

class SPIMaster {
private:
    uint8_t nav_cs_pin;
    uint8_t tel_cs_pin;
    uint32_t spi_frequency;
    
public:
    SPIMaster(uint8_t navigation_cs, uint8_t telemetry_cs, uint32_t frequency = 1000000);
    
    void begin();
    bool send_packet(uint8_t device_id, const spi_packet_t* packet);
    bool receive_packet(uint8_t device_id, spi_packet_t* packet);
    bool transaction(uint8_t device_id, const spi_packet_t* tx_packet, spi_packet_t* rx_packet);
    
    // High-level functions
    bool ping_device(uint8_t device_id);
    bool request_status(uint8_t device_id, status_response_t* status);
    bool send_servo_control(const servo_control_t* servo_data);
    bool request_imu_data(imu_data_t* imu_data);
    bool send_telemetry_data(const telemetry_data_t* telemetry_data);
    bool emergency_stop_all();
    
private:
    uint8_t get_cs_pin(uint8_t device_id);
    void select_device(uint8_t device_id);
    void deselect_device(uint8_t device_id);
    void transfer_packet(const spi_packet_t* tx_packet, spi_packet_t* rx_packet);
};

#endif // SPI_MASTER_H