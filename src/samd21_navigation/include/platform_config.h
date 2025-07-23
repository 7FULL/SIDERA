#ifndef PLATFORM_CONFIG_H
#define PLATFORM_CONFIG_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>

// ATSAMD21G18A-AU Navigation Controller - Slave 1
#define NAVIGATION_CONTROLLER
#define SLAVE_CONTROLLER_1

// System Configuration
#define CORE_FREQUENCY 48000000L
#define SPI_FREQUENCY 8000000L

// SPI Communication with Master
#define SPI_MISO_PIN 12
#define SPI_MOSI_PIN 11
#define SPI_SCK_PIN 13
#define SPI_CS_PIN 10

// IMU Configuration (BNO055)
#define IMU_I2C_ADDRESS 0x28
#define IMU_RESET_PIN 6

// Servo Control Pins (4x PWM)
#define SERVO1_PIN 2
#define SERVO2_PIN 3
#define SERVO3_PIN 4
#define SERVO4_PIN 5

// RGB LED Pins (Square RGB LED)
#define RGB_LED_R_PIN 7
#define RGB_LED_G_PIN 8
#define RGB_LED_B_PIN 9

// I2C Configuration
#define I2C_SDA_PIN 20
#define I2C_SCL_PIN 21
#define I2C_FREQUENCY 400000L

// Debug and Status
#define DEBUG_SERIAL SerialUSB
#define DEBUG_BAUD 115200

// Navigation specific definitions
#define MAX_SERVOS 4
#define SERVO_MIN_PULSE 1000
#define SERVO_MAX_PULSE 2000
#define SERVO_CENTER_PULSE 1500

#endif // PLATFORM_CONFIG_H