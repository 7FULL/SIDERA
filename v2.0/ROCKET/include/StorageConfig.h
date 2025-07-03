#ifndef STORAGE_CONFIG_H
#define STORAGE_CONFIG_H

// Flash storage organization
#define FLASH_SECTOR_SIZE 4096
#define FLASH_LOG_START_ADDRESS 0
#define FLASH_LOG_SIZE 0x10000
#define FLASH_TELEMETRY_START_ADDRESS 0x10000
#define FLASH_TELEMETRY_SIZE 0x70000
#define FLASH_METADATA_ADDRESS 0x80000

// SD Card configuration
#define SD_SPI_FREQUENCY 25000000  // 25 MHz
#define SD_MAX_FILE_SIZE 4294967295UL  // Max file size (4GB)
#define SD_BUFFER_SIZE 512

// Log message format
#define LOG_FORMAT_CSV 1  // 1 for CSV, 0 for binary

#endif