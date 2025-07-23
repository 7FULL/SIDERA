#ifndef SPI_PROTOCOL_H
#define SPI_PROTOCOL_H

#include <stdint.h>

// Protocol Configuration
#define SPI_PACKET_SIZE 32
#define SPI_HEADER_SIZE 4
#define SPI_DATA_SIZE (SPI_PACKET_SIZE - SPI_HEADER_SIZE)
#define SPI_TIMEOUT_MS 100

// Device IDs
#define DEVICE_MASTER 0x01
#define DEVICE_NAVIGATION 0x02
#define DEVICE_TELEMETRY 0x03

// Command Types
#define CMD_PING 0x01
#define CMD_STATUS_REQUEST 0x02
#define CMD_DATA_SEND 0x03
#define CMD_CONFIG_SET 0x04
#define CMD_SERVO_CONTROL 0x05
#define CMD_IMU_DATA 0x06
#define CMD_TELEMETRY_SEND 0x07
#define CMD_EMERGENCY_STOP 0xFF

// Response Types
#define RESP_ACK 0x01
#define RESP_NACK 0x02
#define RESP_DATA 0x03
#define RESP_STATUS 0x04
#define RESP_ERROR 0xFF

// Status Flags
#define STATUS_OK 0x00
#define STATUS_ERROR 0x01
#define STATUS_BUSY 0x02
#define STATUS_NOT_READY 0x03
#define STATUS_TIMEOUT 0x04

// SPI Protocol Packet Structure
typedef struct {
    uint8_t start_byte;     // Always 0xAA
    uint8_t device_id;      // Source device ID
    uint8_t command;        // Command or response type
    uint8_t length;         // Data length (0-28)
    uint8_t data[SPI_DATA_SIZE]; // Payload data
} spi_packet_t;

// Data Structures for specific commands

// IMU Data Structure (Navigation -> Master)
typedef struct {
    float quaternion[4];    // w, x, y, z
    float acceleration[3];  // x, y, z (m/s²)
    float gyroscope[3];     // x, y, z (rad/s)
    uint8_t calibration_status;
    uint8_t system_status;
} imu_data_t;

// Servo Control Structure (Master -> Navigation)
typedef struct {
    uint16_t servo_positions[4]; // Microsecond pulse widths
    uint8_t servo_enable_mask;   // Bit mask for enabled servos
    uint8_t reserved;
} servo_control_t;

// Telemetry Data Structure (Master -> Telemetry)
typedef struct {
    float altitude;         // Barometer altitude (m)
    float velocity;         // Calculated velocity (m/s)
    float acceleration;     // High-G accelerometer (g)
    float battery_voltage;  // Battery voltage (V)
    uint8_t flight_state;   // Current flight state
    uint8_t pyro_status;    // Pyrotechnic channel status
    uint16_t timestamp;     // Milliseconds since boot
} telemetry_data_t;

// Status Response Structure
typedef struct {
    uint8_t device_status;  // Device status flags
    uint8_t last_command;   // Last received command
    uint16_t uptime;        // Device uptime in seconds
    uint8_t error_count;    // Number of errors since boot
    uint8_t reserved[3];
} status_response_t;

// Flight States
#define FLIGHT_STATE_IDLE 0x00
#define FLIGHT_STATE_ARMED 0x01
#define FLIGHT_STATE_BOOST 0x02
#define FLIGHT_STATE_COAST 0x03
#define FLIGHT_STATE_APOGEE 0x04
#define FLIGHT_STATE_DROGUE 0x05
#define FLIGHT_STATE_MAIN 0x06
#define FLIGHT_STATE_LANDED 0x07
#define FLIGHT_STATE_ERROR 0xFF

// Protocol Constants
#define PROTOCOL_START_BYTE 0xAA
#define PROTOCOL_VERSION 0x01

// Function Prototypes
uint8_t calculate_checksum(const spi_packet_t* packet);
bool validate_packet(const spi_packet_t* packet);
void init_packet(spi_packet_t* packet, uint8_t device_id, uint8_t command);
void pack_imu_data(spi_packet_t* packet, const imu_data_t* imu_data);
void unpack_imu_data(const spi_packet_t* packet, imu_data_t* imu_data);
void pack_servo_control(spi_packet_t* packet, const servo_control_t* servo_data);
void unpack_servo_control(const spi_packet_t* packet, servo_control_t* servo_data);
void pack_telemetry_data(spi_packet_t* packet, const telemetry_data_t* telemetry_data);
void unpack_telemetry_data(const spi_packet_t* packet, telemetry_data_t* telemetry_data);

#endif // SPI_PROTOCOL_H