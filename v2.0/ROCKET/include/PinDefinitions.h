/**
 * Rocket Control System - RP2040 Pin Definitions
 *
 * This file defines all the GPIO pin assignments for the rocket hardware.
 */

#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

// I2C Bus 1 (Barometric sensors, IMU)
#define I2C1_SDA 2
#define I2C1_SCL 3

//sensors adress
#define BMP_ADDR 0x76
#define MPL_ADDR 0x60
#define AXL_ADDR 0x53
#define BMIO_GYR_ADDR 0x68
#define BMIO_ACCEL_ADDR 0x18

//Sensors ids
#define BMIO_GYR_ID 0x0F
#define BMIO_ACCEL_ID 0x1E
#define ADXL_ID 0xE5

// SPI Bus 0 (SD Card)
#define SPI0_MISO 16
#define SPI0_MOSI 19
#define SPI0_SCK 18
#define SD_CS 16

// SPI Bus 1 (LoRa, Flash)
#define SPI1_MISO 8
#define SPI1_MOSI 11
#define SPI1_SCK 10
#define LORA_CS 9
#define LORA_RST 12
#define LORA_DIO0 13
#define FLASH_CS 56

// UART0 (GPS L76KB-A58)
#define L76_RX 0
#define L76_TX 1
#define L76_STBY 14

// UART1 (GPS ATGM336H)
#define ATGM_RX 4
#define ATGM_TX 5
#define ATGM_STBY 15

// 1-Wire (DS18B20)
#define DS18B20_PIN 24

// Pyro channels
#define PYRO1 25
#define PYRO2 27
#define PYRO3 28
#define PYRO4 29

// Buzzer
#define BUZZER_PIN 23

// Status LEDs
#define LED_RED 21
#define LED_BLUE 22

// Battery voltage divider
#define BATTERY_VOLTAGE_PIN 26

#endif // PIN_DEFINITIONS_H