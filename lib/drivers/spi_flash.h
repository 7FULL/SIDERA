#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#include <Arduino.h>
#include <SPI.h>

// Generic SPI Flash Memory Driver
// Supports common SPI flash memory chips (W25Q series, AT25DF series, etc.)
// Used for data logging and configuration storage

// SPI Communication Settings
#define FLASH_SPI_MODE SPI_MODE0
#define FLASH_SPI_SPEED 20000000  // 20MHz typical max speed
#define FLASH_SPI_BITORDER MSBFIRST

// Standard SPI Flash Commands
#define FLASH_CMD_WRITE_ENABLE    0x06  // Write Enable
#define FLASH_CMD_WRITE_DISABLE   0x04  // Write Disable
#define FLASH_CMD_READ_STATUS1    0x05  // Read Status Register 1
#define FLASH_CMD_READ_STATUS2    0x35  // Read Status Register 2
#define FLASH_CMD_READ_STATUS3    0x15  // Read Status Register 3
#define FLASH_CMD_WRITE_STATUS1   0x01  // Write Status Register 1
#define FLASH_CMD_WRITE_STATUS2   0x31  // Write Status Register 2
#define FLASH_CMD_WRITE_STATUS3   0x11  // Write Status Register 3
#define FLASH_CMD_READ_DATA       0x03  // Read Data
#define FLASH_CMD_FAST_READ       0x0B  // Fast Read
#define FLASH_CMD_PAGE_PROGRAM    0x02  // Page Program
#define FLASH_CMD_SECTOR_ERASE    0x20  // Sector Erase (4KB)
#define FLASH_CMD_BLOCK_ERASE_32K 0x52  // Block Erase (32KB)
#define FLASH_CMD_BLOCK_ERASE_64K 0xD8  // Block Erase (64KB)
#define FLASH_CMD_CHIP_ERASE      0xC7  // Chip Erase
#define FLASH_CMD_POWER_DOWN      0xB9  // Power Down
#define FLASH_CMD_RELEASE_POWER_DOWN 0xAB // Release Power Down
#define FLASH_CMD_DEVICE_ID       0x90  // Read Manufacturer/Device ID
#define FLASH_CMD_JEDEC_ID        0x9F  // Read JEDEC ID
#define FLASH_CMD_UNIQUE_ID       0x4B  // Read Unique ID

// Status Register 1 bits
#define FLASH_STATUS1_BUSY        0x01  // Write in Progress
#define FLASH_STATUS1_WEL         0x02  // Write Enable Latch
#define FLASH_STATUS1_BP0         0x04  // Block Protect bit 0
#define FLASH_STATUS1_BP1         0x08  // Block Protect bit 1
#define FLASH_STATUS1_BP2         0x10  // Block Protect bit 2
#define FLASH_STATUS1_TB          0x20  // Top/Bottom Protect
#define FLASH_STATUS1_SEC         0x40  // Sector Protect
#define FLASH_STATUS1_SRP0        0x80  // Status Register Protect 0

// Status Register 2 bits
#define FLASH_STATUS2_SRP1        0x01  // Status Register Protect 1
#define FLASH_STATUS2_QE          0x02  // Quad Enable
#define FLASH_STATUS2_LB1         0x08  // Security Register Lock bit 1
#define FLASH_STATUS2_LB2         0x10  // Security Register Lock bit 2
#define FLASH_STATUS2_LB3         0x20  // Security Register Lock bit 3
#define FLASH_STATUS2_CMP         0x40  // Complement Protect
#define FLASH_STATUS2_SUS         0x80  // Suspend Status

// Common flash memory sizes
#define FLASH_SIZE_1MB            (1024 * 1024)
#define FLASH_SIZE_2MB            (2 * 1024 * 1024)
#define FLASH_SIZE_4MB            (4 * 1024 * 1024)
#define FLASH_SIZE_8MB            (8 * 1024 * 1024)
#define FLASH_SIZE_16MB           (16 * 1024 * 1024)

// Standard flash memory parameters
#define FLASH_PAGE_SIZE           256   // Standard page size (bytes)
#define FLASH_SECTOR_SIZE         4096  // Standard sector size (bytes)
#define FLASH_BLOCK_SIZE_32K      32768 // 32KB block size
#define FLASH_BLOCK_SIZE_64K      65536 // 64KB block size

// Timeout values (milliseconds)
#define FLASH_TIMEOUT_PAGE_PROGRAM 5
#define FLASH_TIMEOUT_SECTOR_ERASE 1000
#define FLASH_TIMEOUT_BLOCK_ERASE  2000
#define FLASH_TIMEOUT_CHIP_ERASE   100000

// Error codes
#define FLASH_SUCCESS             0
#define FLASH_ERROR_INIT         -1
#define FLASH_ERROR_BUSY         -2
#define FLASH_ERROR_TIMEOUT      -3
#define FLASH_ERROR_WRITE_PROTECT -4
#define FLASH_ERROR_INVALID_ADDR -5
#define FLASH_ERROR_SPI          -6
#define FLASH_ERROR_VERIFY       -7

// Flash chip information structure
struct FlashChipInfo {
    uint8_t manufacturer_id;
    uint8_t device_id;
    uint16_t jedec_id;
    uint32_t capacity;          // Total capacity in bytes
    uint16_t page_size;         // Page size in bytes
    uint32_t sector_size;       // Sector size in bytes
    uint32_t block_size;        // Block size in bytes
    char model_name[32];        // Human readable model name
    bool detected;
};

// Flash status information
struct FlashStatus {
    bool busy;                  // Write/erase in progress
    bool write_enabled;         // Write enable latch set
    uint8_t block_protect;      // Block protection bits
    bool quad_enable;           // Quad SPI enabled
    uint8_t security_locks;     // Security register locks
    uint8_t raw_status1;        // Raw status register 1
    uint8_t raw_status2;        // Raw status register 2
};

// File system entry (simple directory structure)
struct FlashFileEntry {
    char filename[16];          // File name (null terminated)
    uint32_t start_address;     // Starting address in flash
    uint32_t size;              // File size in bytes
    uint32_t timestamp;         // Creation timestamp
    uint8_t flags;              // File flags (active, deleted, etc.)
    uint8_t reserved[3];        // Reserved for alignment
};

#define FLASH_FILE_FLAG_ACTIVE    0x01
#define FLASH_FILE_FLAG_DELETED   0x02
#define FLASH_FILE_FLAG_SYSTEM    0x04

#define FLASH_MAX_FILES           64
#define FLASH_DIRECTORY_SIZE      (FLASH_MAX_FILES * sizeof(FlashFileEntry))

class SPIFlash {
private:
    uint8_t cs_pin;
    uint32_t spi_speed;
    bool initialized;
    
    FlashChipInfo chip_info;
    
    // File system variables
    bool filesystem_enabled;
    uint32_t filesystem_start;
    uint32_t data_start;
    FlashFileEntry directory[FLASH_MAX_FILES];
    bool directory_loaded;
    
    // Statistics
    uint32_t read_operations;
    uint32_t write_operations;
    uint32_t erase_operations;
    uint32_t error_count;
    
    // Wear leveling (simple implementation)
    uint32_t* erase_counters;
    bool wear_leveling_enabled;
    
public:
    SPIFlash(uint8_t chip_select_pin, uint32_t speed = FLASH_SPI_SPEED);
    ~SPIFlash();
    
    // Initialization and detection
    int8_t begin();
    int8_t detect_chip();
    bool is_initialized() const { return initialized; }
    const FlashChipInfo& get_chip_info() const { return chip_info; }
    
    // Basic flash operations
    int8_t read(uint32_t address, uint8_t* buffer, uint32_t length);
    int8_t write(uint32_t address, const uint8_t* data, uint32_t length);
    int8_t erase_sector(uint32_t address);
    int8_t erase_block_32k(uint32_t address);
    int8_t erase_block_64k(uint32_t address);
    int8_t erase_chip();
    
    // Page-aligned operations (more efficient)
    int8_t read_page(uint32_t page_number, uint8_t* buffer);
    int8_t write_page(uint32_t page_number, const uint8_t* data);
    
    // Status and control
    FlashStatus get_status();
    bool is_busy();
    int8_t wait_for_ready(uint32_t timeout_ms = 10000);
    int8_t write_enable();
    int8_t write_disable();
    
    // Power management
    int8_t enter_power_down();
    int8_t exit_power_down();
    
    // Protection features
    int8_t set_write_protection(bool enable);
    int8_t set_block_protection(uint8_t bp_bits);
    bool is_write_protected();
    
    // Unique identification
    int8_t read_unique_id(uint8_t* id_buffer, uint8_t length = 8);
    uint32_t get_jedec_id();
    
    // File system operations (optional)
    int8_t format_filesystem();
    int8_t mount_filesystem();
    bool is_filesystem_mounted() const { return filesystem_enabled && directory_loaded; }
    
    // File operations
    int8_t create_file(const char* filename, uint32_t size);
    int8_t delete_file(const char* filename);
    int8_t write_file(const char* filename, const uint8_t* data, uint32_t length, uint32_t offset = 0);
    int8_t read_file(const char* filename, uint8_t* buffer, uint32_t length, uint32_t offset = 0);
    int32_t get_file_size(const char* filename);
    bool file_exists(const char* filename);
    int8_t list_files(char file_list[][16], int max_files);
    
    // Utility functions
    uint32_t get_total_capacity() const { return chip_info.capacity; }
    uint32_t get_available_space();
    uint32_t get_used_space();
    float get_capacity_mb() const { return chip_info.capacity / (1024.0f * 1024.0f); }
    
    // Address calculations
    uint32_t page_to_address(uint32_t page_number) const { return page_number * chip_info.page_size; }
    uint32_t address_to_page(uint32_t address) const { return address / chip_info.page_size; }
    uint32_t sector_to_address(uint32_t sector_number) const { return sector_number * chip_info.sector_size; }
    uint32_t address_to_sector(uint32_t address) const { return address / chip_info.sector_size; }
    
    // Statistics and diagnostics
    uint32_t get_read_operations() const { return read_operations; }
    uint32_t get_write_operations() const { return write_operations; }
    uint32_t get_erase_operations() const { return erase_operations; }
    uint32_t get_error_count() const { return error_count; }
    void reset_statistics();
    
    // Wear leveling
    int8_t enable_wear_leveling();
    void disable_wear_leveling();
    uint32_t get_max_erase_count();
    uint32_t get_min_erase_count();
    float get_wear_leveling_factor();
    
    // Advanced operations
    int8_t verify_write(uint32_t address, const uint8_t* expected_data, uint32_t length);
    int8_t calculate_checksum(uint32_t address, uint32_t length, uint32_t& checksum);
    int8_t find_bad_blocks(uint32_t* bad_block_list, uint8_t max_blocks);
    
    // Debugging and testing
    void print_chip_info();
    void print_status();
    void print_directory();
    int8_t memory_test(uint32_t start_address, uint32_t length);
    void hex_dump(uint32_t address, uint32_t length);
    
private:
    // Low-level SPI functions
    void spi_begin_transaction();
    void spi_end_transaction();
    uint8_t spi_transfer(uint8_t data);
    void spi_transfer_buffer(const uint8_t* tx_buffer, uint8_t* rx_buffer, uint32_t length);
    
    // Command functions
    void send_command(uint8_t command);
    void send_command_with_address(uint8_t command, uint32_t address);
    uint8_t read_status_register(uint8_t register_num = 1);
    int8_t write_status_register(uint8_t register_num, uint8_t value);
    
    // Internal utility functions
    bool is_valid_address(uint32_t address, uint32_t length = 1);
    uint32_t align_to_page(uint32_t address);
    uint32_t align_to_sector(uint32_t address);
    void increment_error_count();
    
    // File system helpers
    int8_t load_directory();
    int8_t save_directory();
    int8_t find_file_entry(const char* filename);
    int8_t find_free_file_entry();
    uint32_t find_free_space(uint32_t size);
    int8_t allocate_file_space(const char* filename, uint32_t size);
    
    // Chip detection helpers
    bool detect_winbond_chip();
    bool detect_atmel_chip();
    bool detect_micron_chip();
    bool detect_generic_chip();
    
    // Wear leveling helpers
    void update_erase_counter(uint32_t sector_address);
    uint32_t find_least_worn_sector(uint32_t required_sectors);
};

// Global instance helper
extern SPIFlash* g_flash;

// Utility functions
namespace FlashUtils {
    // Data integrity
    uint32_t calculate_crc32(const uint8_t* data, uint32_t length);
    bool verify_data_integrity(const uint8_t* original, const uint8_t* readback, uint32_t length);
    
    // Address utilities
    bool is_page_aligned(uint32_t address);
    bool is_sector_aligned(uint32_t address);
    uint32_t round_up_to_page(uint32_t address);
    uint32_t round_up_to_sector(uint32_t address);
    
    // Size formatting
    void format_size_string(uint32_t bytes, char* buffer, size_t buffer_size);
    
    // Flash memory testing
    int8_t perform_walking_ones_test(SPIFlash& flash, uint32_t start_addr, uint32_t length);
    int8_t perform_address_test(SPIFlash& flash, uint32_t start_addr, uint32_t length);
    int8_t perform_random_data_test(SPIFlash& flash, uint32_t start_addr, uint32_t length);
    
    // Data logging helpers
    struct LogEntry {
        uint32_t timestamp;
        uint16_t data_type;
        uint16_t data_length;
        uint8_t data[252];  // Total entry size = 256 bytes (page aligned)
    };
    
    int8_t append_log_entry(SPIFlash& flash, const LogEntry& entry);
    int8_t read_log_entries(SPIFlash& flash, LogEntry* entries, uint32_t max_entries, uint32_t start_index = 0);
    uint32_t count_log_entries(SPIFlash& flash);
    
    // Configuration storage
    int8_t save_config_block(SPIFlash& flash, const char* config_name, const void* config_data, uint32_t size);
    int8_t load_config_block(SPIFlash& flash, const char* config_name, void* config_data, uint32_t max_size);
    bool config_exists(SPIFlash& flash, const char* config_name);
    
    // Flash memory organization helpers
    enum FlashRegion {
        REGION_BOOTLOADER,      // Reserved for bootloader
        REGION_FIRMWARE,        // Main firmware storage
        REGION_CONFIG,          // Configuration data
        REGION_LOGS,            // Data logging
        REGION_USER_DATA,       // User application data
        REGION_FILESYSTEM       // File system area
    };
    
    struct FlashLayout {
        uint32_t bootloader_start;
        uint32_t bootloader_size;
        uint32_t firmware_start;
        uint32_t firmware_size;
        uint32_t config_start;
        uint32_t config_size;
        uint32_t logs_start;
        uint32_t logs_size;
        uint32_t filesystem_start;
        uint32_t filesystem_size;
    };
    
    FlashLayout calculate_optimal_layout(uint32_t total_capacity);
    bool validate_layout(const FlashLayout& layout, uint32_t total_capacity);
}

#endif // SPI_FLASH_H