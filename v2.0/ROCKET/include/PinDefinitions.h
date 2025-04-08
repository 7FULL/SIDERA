/**
 * Rocket Control System - RP2040 Pin Definitions
 *
 * This file defines all the GPIO pin assignments for the rocket hardware.
 */

#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

// Built-in LED
#define LED_BUILTIN 25

// I2C Bus 0 (Barometric sensors, IMU)
#define I2C0_SDA 21
#define I2C0_SCL 22

// SPI Bus 0 (SD Card)
#define SPI0_MISO 16
#define SPI0_MOSI 19
#define SPI0_SCK 18
#define SD_CS 17

// SPI Bus 1 (LoRa, Flash)
#define SPI1_MISO 12
#define SPI1_MOSI 15
#define SPI1_SCK 14
#define LORA_CS 13
#define LORA_RST 11
#define LORA_DIO0 10
#define FLASH_CS 9

// UART0 (GPS L76KB-A58)
#define GPS1_TX 0
#define GPS1_RX 1

// UART1 (GPS ATGM336H)
#define GPS2_TX 4
#define GPS2_RX 5
#define L76_STBY 14
#define ATGM_STBY 15

// 1-Wire (DS18B20)
#define ONE_WIRE_BUS 26

// Pyro channels
#define PYRO1 27
#define PYRO2 28
#define PYRO3 29
#define PYRO4 8

// Buzzer
#define BUZZER_PIN 6

// Status LEDs
#define LED_RED 2
#define LED_GREEN 3
#define LED_BLUE 7

// Battery voltage divider
#define BATTERY_VOLTAGE_PIN 30

#endif // PIN_DEFINITIONS_H