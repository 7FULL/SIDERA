/**
 * Rocket Control System - FreeRTOS Task Configuration
 *
 * This file defines task priorities, stack sizes, and other FreeRTOS configurations.
 */

#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H

// Task priorities (higher number = higher priority)
#define CORE0_PRIORITY          (tskIDLE_PRIORITY + 4)  // Main core 0 task
#define CORE1_PRIORITY          (tskIDLE_PRIORITY + 3)  // Main core 1 task
#define SENSOR_READ_PRIORITY    (tskIDLE_PRIORITY + 4)  // Sensor reading task
#define EVENT_DETECT_PRIORITY   (tskIDLE_PRIORITY + 4)  // Event detection task
#define TELEMETRY_PRIORITY      (tskIDLE_PRIORITY + 2)  // Telemetry task
#define LOGGING_PRIORITY        (tskIDLE_PRIORITY + 2)  // Data logging task
#define GPS_PRIORITY            (tskIDLE_PRIORITY + 1)  // GPS processing task

// Task stack sizes (in words)
#define CORE0_STACK_SIZE        2048
#define CORE1_STACK_SIZE        2048
#define SENSOR_READ_STACK_SIZE  1024
#define EVENT_DETECT_STACK_SIZE 1024
#define TELEMETRY_STACK_SIZE    1536
#define LOGGING_STACK_SIZE      1536
#define GPS_STACK_SIZE          1536

#endif // TASK_CONFIG_H