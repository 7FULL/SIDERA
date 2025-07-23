#ifndef PLATFORM_CONFIG_H
#define PLATFORM_CONFIG_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// ATSAMD21G18A-AU Telemetry Controller - Slave 2
#define TELEMETRY_CONTROLLER
#define SLAVE_CONTROLLER_2

// System Configuration
#define CORE_FREQUENCY 48000000L
#define SPI_FREQUENCY 8000000L

// SPI Communication with Master
#define SPI_MISO_PIN 12
#define SPI_MOSI_PIN 11
#define SPI_SCK_PIN 13
#define SPI_CS_PIN 9

// Radio Module Configuration
#define RADIO_SERIAL Serial1
#define RADIO_BAUD 9600
#define RADIO_TX_PIN 0
#define RADIO_RX_PIN 1
#define RADIO_RESET_PIN 6
#define RADIO_CONFIG_PIN 7

// RGB LED Pins (Square RGB LED)
#define RGB_LED_R_PIN 2
#define RGB_LED_G_PIN 3
#define RGB_LED_B_PIN 4

// Debug and Status
#define DEBUG_SERIAL SerialUSB
#define DEBUG_BAUD 115200

// Telemetry Configuration
#define TELEMETRY_PACKET_SIZE 64
#define TELEMETRY_SEND_INTERVAL 100  // ms
#define RADIO_TIMEOUT 1000           // ms

// XBee Radio Configuration
#define XBEE_PAN_ID 0x1234
#define XBEE_CHANNEL 0x0C
#define XBEE_POWER_LEVEL 4

#endif // PLATFORM_CONFIG_H