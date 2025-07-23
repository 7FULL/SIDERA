#ifndef PLATFORM_CONFIG_H
#define PLATFORM_CONFIG_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// MK20DX256VLH7 Master Controller - MPU
#define MASTER_CONTROLLER
#define MPU_PROCESSOR

// System Configuration
#define CORE_FREQUENCY 72000000L
#define SPI_FREQUENCY 10000000L

// SPI Communication with Slave Controllers
#define SLAVE1_CS_PIN 10  // SAMD21 Navigation
#define SLAVE2_CS_PIN 9   // SAMD21 Telemetry

// Sensor Pins (connected to MK20)
#define HIGH_G_ACCEL_CS_PIN 8     // KX134-1211
#define BAROMETER_CS_PIN 7        // MS561101BA03-50
#define SD_CARD_CS_PIN 6
#define SPI_FLASH_CS_PIN 5

// GPS UART
#define GPS_SERIAL Serial1
#define GPS_BAUD 9600

// Pyrotechnic Channels (GPIO + MOSFET)
#define PYRO_CH1_PIN 2
#define PYRO_CH2_PIN 3
#define PYRO_CH3_PIN 4
#define PYRO_CH4_PIN 21

// RGB LED Pins (Square RGB LED)
#define RGB_LED_R_PIN 22
#define RGB_LED_G_PIN 23
#define RGB_LED_B_PIN 24

// Debug and Status
#define DEBUG_SERIAL Serial
#define DEBUG_BAUD 115200

// Power and Control
#define BATTERY_VOLTAGE_PIN A0
#define POWER_SWITCH_PIN 25

#endif // PLATFORM_CONFIG_H