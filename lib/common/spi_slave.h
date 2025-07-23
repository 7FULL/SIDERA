#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

#include "spi_protocol.h"
#include <Arduino.h>
#include <SPI.h>

typedef void (*command_handler_t)(const spi_packet_t* rx_packet, spi_packet_t* tx_packet);

class SPISlave {
private:
    uint8_t device_id;
    uint8_t cs_pin;
    volatile bool packet_received;
    volatile bool transaction_active;
    
    spi_packet_t rx_buffer;
    spi_packet_t tx_buffer;
    
    command_handler_t command_handlers[256];
    status_response_t device_status;
    
public:
    SPISlave(uint8_t device_id, uint8_t cs_pin);
    
    void begin();
    void update();
    bool is_transaction_active();
    
    void register_command_handler(uint8_t command, command_handler_t handler);
    void set_device_status(uint8_t status);
    void increment_error_count();
    
    // Default handlers
    void handle_ping(const spi_packet_t* rx_packet, spi_packet_t* tx_packet);
    void handle_status_request(const spi_packet_t* rx_packet, spi_packet_t* tx_packet);
    void handle_emergency_stop(const spi_packet_t* rx_packet, spi_packet_t* tx_packet);
    
    // Navigation specific handlers
    void handle_servo_control(const spi_packet_t* rx_packet, spi_packet_t* tx_packet);
    void handle_imu_request(const spi_packet_t* rx_packet, spi_packet_t* tx_packet);
    
    // Telemetry specific handlers
    void handle_telemetry_send(const spi_packet_t* rx_packet, spi_packet_t* tx_packet);
    
    // Data access methods
    const spi_packet_t* get_last_received_packet() const { return &rx_buffer; }
    void prepare_response_packet(uint8_t command, const void* data, uint8_t length);
    
private:
    void process_received_packet();
    void prepare_default_response(uint8_t response_type);
    static void cs_interrupt_handler();
    void handle_cs_interrupt();
};

// Global instance pointer for ISR
extern SPISlave* g_spi_slave_instance;

#endif // SPI_SLAVE_H