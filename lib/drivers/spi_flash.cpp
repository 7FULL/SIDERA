#include "spi_flash.h"
#include <string.h>

SPIFlash* g_flash = nullptr;

SPIFlash::SPIFlash(uint8_t chip_select_pin, uint32_t speed) {
    cs_pin = chip_select_pin;
    spi_speed = speed;
    initialized = false;
    filesystem_enabled = false;
    directory_loaded = false;
    wear_leveling_enabled = false;
    erase_counters = nullptr;
    
    // Initialize chip info
    memset(&chip_info, 0, sizeof(FlashChipInfo));
    
    // Initialize statistics
    read_operations = 0;
    write_operations = 0;
    erase_operations = 0;
    error_count = 0;
    
    // Initialize directory
    memset(directory, 0, sizeof(directory));
}

SPIFlash::~SPIFlash() {
    if (erase_counters) {
        delete[] erase_counters;
    }
}

int8_t SPIFlash::begin() {
    // Initialize CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
    
    // Initialize SPI
    SPI.begin();
    
    // Detect flash chip
    if (detect_chip() != FLASH_SUCCESS) {
        return FLASH_ERROR_INIT;
    }
    
    // Exit power down mode (in case chip was powered down)
    exit_power_down();
    
    // Wait for chip to be ready
    if (wait_for_ready(1000) != FLASH_SUCCESS) {
        return FLASH_ERROR_TIMEOUT;
    }
    
    initialized = true;
    return FLASH_SUCCESS;
}

int8_t SPIFlash::detect_chip() {
    // Read JEDEC ID
    spi_begin_transaction();
    digitalWrite(cs_pin, LOW);
    spi_transfer(FLASH_CMD_JEDEC_ID);
    
    uint8_t manufacturer = spi_transfer(0x00);
    uint8_t memory_type = spi_transfer(0x00);
    uint8_t capacity = spi_transfer(0x00);
    
    digitalWrite(cs_pin, HIGH);
    spi_end_transaction();
    
    chip_info.manufacturer_id = manufacturer;
    chip_info.jedec_id = (manufacturer << 16) | (memory_type << 8) | capacity;
    chip_info.detected = true;
    
    // Set default parameters
    chip_info.page_size = FLASH_PAGE_SIZE;
    chip_info.sector_size = FLASH_SECTOR_SIZE;
    chip_info.block_size = FLASH_BLOCK_SIZE_64K;
    
    // Determine capacity and model based on JEDEC ID
    switch (manufacturer) {
        case 0xEF: // Winbond
            strcpy(chip_info.model_name, "Winbond W25Q");
            switch (capacity) {
                case 0x11: chip_info.capacity = FLASH_SIZE_1MB; strcat(chip_info.model_name, "80"); break;
                case 0x12: chip_info.capacity = FLASH_SIZE_2MB; strcat(chip_info.model_name, "16"); break;
                case 0x13: chip_info.capacity = FLASH_SIZE_4MB; strcat(chip_info.model_name, "32"); break;
                case 0x14: chip_info.capacity = FLASH_SIZE_8MB; strcat(chip_info.model_name, "64"); break;
                case 0x15: chip_info.capacity = FLASH_SIZE_16MB; strcat(chip_info.model_name, "128"); break;
                default: chip_info.capacity = FLASH_SIZE_4MB; strcat(chip_info.model_name, "xx"); break;
            }
            break;
            
        case 0x1F: // Atmel/Adesto
            strcpy(chip_info.model_name, "Atmel AT25DF");
            switch (capacity) {
                case 0x42: chip_info.capacity = FLASH_SIZE_2MB; strcat(chip_info.model_name, "161"); break;
                case 0x43: chip_info.capacity = FLASH_SIZE_4MB; strcat(chip_info.model_name, "321A"); break;
                case 0x44: chip_info.capacity = FLASH_SIZE_8MB; strcat(chip_info.model_name, "641A"); break;
                default: chip_info.capacity = FLASH_SIZE_4MB; strcat(chip_info.model_name, "xxx"); break;
            }
            break;
            
        case 0x20: // Micron/ST
            strcpy(chip_info.model_name, "Micron M25P");
            switch (capacity) {
                case 0x12: chip_info.capacity = FLASH_SIZE_2MB; strcat(chip_info.model_name, "16"); break;
                case 0x13: chip_info.capacity = FLASH_SIZE_4MB; strcat(chip_info.model_name, "32"); break;
                case 0x14: chip_info.capacity = FLASH_SIZE_8MB; strcat(chip_info.model_name, "64"); break;
                case 0x15: chip_info.capacity = FLASH_SIZE_16MB; strcat(chip_info.model_name, "128"); break;
                default: chip_info.capacity = FLASH_SIZE_4MB; strcat(chip_info.model_name, "xx"); break;
            }
            break;
            
        default:
            strcpy(chip_info.model_name, "Unknown Flash");
            chip_info.capacity = FLASH_SIZE_4MB; // Default assumption
            break;
    }
    
    return FLASH_SUCCESS;
}

int8_t SPIFlash::read(uint32_t address, uint8_t* buffer, uint32_t length) {
    if (!initialized || !is_valid_address(address, length)) {
        increment_error_count();
        return FLASH_ERROR_INVALID_ADDR;
    }
    
    if (wait_for_ready() != FLASH_SUCCESS) {
        increment_error_count();
        return FLASH_ERROR_TIMEOUT;
    }
    
    spi_begin_transaction();
    digitalWrite(cs_pin, LOW);
    
    // Send read command and address
    spi_transfer(FLASH_CMD_READ_DATA);
    spi_transfer((address >> 16) & 0xFF);
    spi_transfer((address >> 8) & 0xFF);
    spi_transfer(address & 0xFF);
    
    // Read data
    for (uint32_t i = 0; i < length; i++) {
        buffer[i] = spi_transfer(0x00);
    }
    
    digitalWrite(cs_pin, HIGH);
    spi_end_transaction();
    
    read_operations++;
    return FLASH_SUCCESS;
}

int8_t SPIFlash::write(uint32_t address, const uint8_t* data, uint32_t length) {
    if (!initialized || !is_valid_address(address, length)) {
        increment_error_count();
        return FLASH_ERROR_INVALID_ADDR;
    }
    
    uint32_t bytes_written = 0;
    
    while (bytes_written < length) {
        // Calculate how many bytes we can write in this page
        uint32_t page_offset = address % chip_info.page_size;
        uint32_t bytes_to_write = min(length - bytes_written, chip_info.page_size - page_offset);
        
        // Write enable
        if (write_enable() != FLASH_SUCCESS) {
            increment_error_count();
            return FLASH_ERROR_SPI;
        }
        
        // Page program
        spi_begin_transaction();
        digitalWrite(cs_pin, LOW);
        
        spi_transfer(FLASH_CMD_PAGE_PROGRAM);
        spi_transfer((address >> 16) & 0xFF);
        spi_transfer((address >> 8) & 0xFF);
        spi_transfer(address & 0xFF);
        
        for (uint32_t i = 0; i < bytes_to_write; i++) {
            spi_transfer(data[bytes_written + i]);
        }
        
        digitalWrite(cs_pin, HIGH);
        spi_end_transaction();
        
        // Wait for programming to complete
        if (wait_for_ready(FLASH_TIMEOUT_PAGE_PROGRAM) != FLASH_SUCCESS) {
            increment_error_count();
            return FLASH_ERROR_TIMEOUT;
        }
        
        address += bytes_to_write;
        bytes_written += bytes_to_write;
    }
    
    write_operations++;
    return FLASH_SUCCESS;
}

int8_t SPIFlash::erase_sector(uint32_t address) {
    if (!initialized || !is_valid_address(address)) {
        increment_error_count();
        return FLASH_ERROR_INVALID_ADDR;
    }
    
    // Align address to sector boundary
    address = align_to_sector(address);
    
    if (write_enable() != FLASH_SUCCESS) {
        increment_error_count();
        return FLASH_ERROR_SPI;
    }
    
    spi_begin_transaction();
    digitalWrite(cs_pin, LOW);
    
    spi_transfer(FLASH_CMD_SECTOR_ERASE);
    spi_transfer((address >> 16) & 0xFF);
    spi_transfer((address >> 8) & 0xFF);
    spi_transfer(address & 0xFF);
    
    digitalWrite(cs_pin, HIGH);
    spi_end_transaction();
    
    // Wait for erase to complete
    if (wait_for_ready(FLASH_TIMEOUT_SECTOR_ERASE) != FLASH_SUCCESS) {
        increment_error_count();
        return FLASH_ERROR_TIMEOUT;
    }
    
    // Update wear leveling counter
    if (wear_leveling_enabled && erase_counters) {
        update_erase_counter(address);
    }
    
    erase_operations++;
    return FLASH_SUCCESS;
}

int8_t SPIFlash::erase_chip() {
    if (!initialized) {
        increment_error_count();
        return FLASH_ERROR_INIT;
    }
    
    if (write_enable() != FLASH_SUCCESS) {
        increment_error_count();
        return FLASH_ERROR_SPI;
    }
    
    spi_begin_transaction();
    digitalWrite(cs_pin, LOW);
    spi_transfer(FLASH_CMD_CHIP_ERASE);
    digitalWrite(cs_pin, HIGH);
    spi_end_transaction();
    
    // Wait for erase to complete (can take a very long time)
    if (wait_for_ready(FLASH_TIMEOUT_CHIP_ERASE) != FLASH_SUCCESS) {
        increment_error_count();
        return FLASH_ERROR_TIMEOUT;
    }
    
    // Reset wear leveling counters
    if (wear_leveling_enabled && erase_counters) {
        uint32_t total_sectors = chip_info.capacity / chip_info.sector_size;
        memset(erase_counters, 0, total_sectors * sizeof(uint32_t));
    }
    
    erase_operations++;
    return FLASH_SUCCESS;
}

FlashStatus SPIFlash::get_status() {
    FlashStatus status;
    memset(&status, 0, sizeof(FlashStatus));
    
    if (!initialized) return status;
    
    uint8_t status1 = read_status_register(1);
    uint8_t status2 = read_status_register(2);
    
    status.busy = (status1 & FLASH_STATUS1_BUSY) != 0;
    status.write_enabled = (status1 & FLASH_STATUS1_WEL) != 0;
    status.block_protect = (status1 >> 2) & 0x07; // BP0, BP1, BP2
    status.quad_enable = (status2 & FLASH_STATUS2_QE) != 0;
    status.security_locks = (status2 >> 3) & 0x07; // LB1, LB2, LB3
    status.raw_status1 = status1;
    status.raw_status2 = status2;
    
    return status;
}

bool SPIFlash::is_busy() {
    if (!initialized) return true;
    return (read_status_register(1) & FLASH_STATUS1_BUSY) != 0;
}

int8_t SPIFlash::wait_for_ready(uint32_t timeout_ms) {
    uint32_t start_time = millis();
    
    while (is_busy()) {
        if (millis() - start_time > timeout_ms) {
            increment_error_count();
            return FLASH_ERROR_TIMEOUT;
        }
        delay(1);
    }
    
    return FLASH_SUCCESS;
}

int8_t SPIFlash::write_enable() {
    if (!initialized) return FLASH_ERROR_INIT;
    
    spi_begin_transaction();
    digitalWrite(cs_pin, LOW);
    spi_transfer(FLASH_CMD_WRITE_ENABLE);
    digitalWrite(cs_pin, HIGH);
    spi_end_transaction();
    
    // Verify write enable was set
    if ((read_status_register(1) & FLASH_STATUS1_WEL) == 0) {
        increment_error_count();
        return FLASH_ERROR_SPI;
    }
    
    return FLASH_SUCCESS;
}

int8_t SPIFlash::write_disable() {
    if (!initialized) return FLASH_ERROR_INIT;
    
    spi_begin_transaction();
    digitalWrite(cs_pin, LOW);
    spi_transfer(FLASH_CMD_WRITE_DISABLE);
    digitalWrite(cs_pin, HIGH);
    spi_end_transaction();
    
    return FLASH_SUCCESS;
}

int8_t SPIFlash::enter_power_down() {
    if (!initialized) return FLASH_ERROR_INIT;
    
    spi_begin_transaction();
    digitalWrite(cs_pin, LOW);
    spi_transfer(FLASH_CMD_POWER_DOWN);
    digitalWrite(cs_pin, HIGH);
    spi_end_transaction();
    
    delay(1); // tDP delay
    return FLASH_SUCCESS;
}

int8_t SPIFlash::exit_power_down() {
    spi_begin_transaction();
    digitalWrite(cs_pin, LOW);
    spi_transfer(FLASH_CMD_RELEASE_POWER_DOWN);
    digitalWrite(cs_pin, HIGH);
    spi_end_transaction();
    
    delay(1); // tRES1 delay
    return FLASH_SUCCESS;
}

uint32_t SPIFlash::get_jedec_id() {
    if (!initialized) return 0;
    return chip_info.jedec_id;
}

// File system operations
int8_t SPIFlash::format_filesystem() {
    if (!initialized) return FLASH_ERROR_INIT;
    
    // Erase the first sector for directory
    if (erase_sector(0) != FLASH_SUCCESS) {
        return FLASH_ERROR_SPI;
    }
    
    // Initialize empty directory
    memset(directory, 0, sizeof(directory));
    
    // Set filesystem parameters
    filesystem_start = 0;
    data_start = FLASH_DIRECTORY_SIZE;
    filesystem_enabled = true;
    directory_loaded = true;
    
    // Save empty directory
    return save_directory();
}

int8_t SPIFlash::create_file(const char* filename, uint32_t size) {
    if (!is_filesystem_mounted()) return FLASH_ERROR_INIT;
    
    // Check if file already exists
    if (find_file_entry(filename) >= 0) {
        return FLASH_ERROR_SPI; // File exists
    }
    
    // Find free entry
    int8_t entry_index = find_free_file_entry();
    if (entry_index < 0) {
        return FLASH_ERROR_SPI; // No free entries
    }
    
    // Allocate space
    uint32_t start_address = find_free_space(size);
    if (start_address == 0) {
        return FLASH_ERROR_SPI; // No space available
    }
    
    // Create file entry
    FlashFileEntry* entry = &directory[entry_index];
    strncpy(entry->filename, filename, sizeof(entry->filename) - 1);
    entry->filename[sizeof(entry->filename) - 1] = '\0';
    entry->start_address = start_address;
    entry->size = size;
    entry->timestamp = millis();
    entry->flags = FLASH_FILE_FLAG_ACTIVE;
    
    // Save directory
    return save_directory();
}

int8_t SPIFlash::write_file(const char* filename, const uint8_t* data, uint32_t length, uint32_t offset) {
    if (!is_filesystem_mounted()) return FLASH_ERROR_INIT;
    
    int8_t entry_index = find_file_entry(filename);
    if (entry_index < 0) {
        return FLASH_ERROR_SPI; // File not found
    }
    
    FlashFileEntry* entry = &directory[entry_index];
    
    if (offset + length > entry->size) {
        return FLASH_ERROR_INVALID_ADDR; // Write beyond file size
    }
    
    return write(entry->start_address + offset, data, length);
}

int8_t SPIFlash::read_file(const char* filename, uint8_t* buffer, uint32_t length, uint32_t offset) {
    if (!is_filesystem_mounted()) return FLASH_ERROR_INIT;
    
    int8_t entry_index = find_file_entry(filename);
    if (entry_index < 0) {
        return FLASH_ERROR_SPI; // File not found
    }
    
    FlashFileEntry* entry = &directory[entry_index];
    
    if (offset + length > entry->size) {
        length = entry->size - offset; // Limit read to file size
    }
    
    return read(entry->start_address + offset, buffer, length);
}

bool SPIFlash::file_exists(const char* filename) {
    return find_file_entry(filename) >= 0;
}

uint32_t SPIFlash::get_available_space() {
    if (!is_filesystem_mounted()) return 0;
    
    uint32_t used_space = FLASH_DIRECTORY_SIZE; // Directory space
    
    for (int i = 0; i < FLASH_MAX_FILES; i++) {
        if (directory[i].flags & FLASH_FILE_FLAG_ACTIVE) {
            used_space += directory[i].size;
        }
    }
    
    return chip_info.capacity - used_space;
}

void SPIFlash::print_chip_info() {
    Serial.println("=== SPI Flash Chip Information ===");
    Serial.print("Model: "); Serial.println(chip_info.model_name);
    Serial.print("JEDEC ID: 0x"); Serial.println(chip_info.jedec_id, HEX);
    Serial.print("Manufacturer ID: 0x"); Serial.println(chip_info.manufacturer_id, HEX);
    Serial.print("Capacity: "); Serial.print(get_capacity_mb(), 1); Serial.println(" MB");
    Serial.print("Page Size: "); Serial.println(chip_info.page_size);
    Serial.print("Sector Size: "); Serial.println(chip_info.sector_size);
    Serial.print("Block Size: "); Serial.println(chip_info.block_size);
    Serial.print("Detected: "); Serial.println(chip_info.detected ? "Yes" : "No");
}

void SPIFlash::print_status() {
    FlashStatus status = get_status();
    
    Serial.println("=== SPI Flash Status ===");
    Serial.print("Busy: "); Serial.println(status.busy ? "Yes" : "No");
    Serial.print("Write Enabled: "); Serial.println(status.write_enabled ? "Yes" : "No");
    Serial.print("Block Protect: "); Serial.println(status.block_protect);
    Serial.print("Quad Enable: "); Serial.println(status.quad_enable ? "Yes" : "No");
    Serial.print("Status Register 1: 0x"); Serial.println(status.raw_status1, HEX);
    Serial.print("Status Register 2: 0x"); Serial.println(status.raw_status2, HEX);
    
    Serial.println("=== Statistics ===");
    Serial.print("Read Operations: "); Serial.println(read_operations);
    Serial.print("Write Operations: "); Serial.println(write_operations);
    Serial.print("Erase Operations: "); Serial.println(erase_operations);
    Serial.print("Error Count: "); Serial.println(error_count);
}

// Private functions
void SPIFlash::spi_begin_transaction() {
    SPI.beginTransaction(SPISettings(spi_speed, FLASH_SPI_BITORDER, FLASH_SPI_MODE));
}

void SPIFlash::spi_end_transaction() {
    SPI.endTransaction();
}

uint8_t SPIFlash::spi_transfer(uint8_t data) {
    return SPI.transfer(data);
}

uint8_t SPIFlash::read_status_register(uint8_t register_num) {
    uint8_t command;
    switch (register_num) {
        case 1: command = FLASH_CMD_READ_STATUS1; break;
        case 2: command = FLASH_CMD_READ_STATUS2; break;
        case 3: command = FLASH_CMD_READ_STATUS3; break;
        default: return 0;
    }
    
    spi_begin_transaction();
    digitalWrite(cs_pin, LOW);
    spi_transfer(command);
    uint8_t status = spi_transfer(0x00);
    digitalWrite(cs_pin, HIGH);
    spi_end_transaction();
    
    return status;
}

bool SPIFlash::is_valid_address(uint32_t address, uint32_t length) {
    return (address + length <= chip_info.capacity);
}

uint32_t SPIFlash::align_to_sector(uint32_t address) {
    return (address / chip_info.sector_size) * chip_info.sector_size;
}

void SPIFlash::increment_error_count() {
    error_count++;
}

int8_t SPIFlash::find_file_entry(const char* filename) {
    for (int i = 0; i < FLASH_MAX_FILES; i++) {
        if ((directory[i].flags & FLASH_FILE_FLAG_ACTIVE) && 
            strcmp(directory[i].filename, filename) == 0) {
            return i;
        }
    }
    return -1;
}

int8_t SPIFlash::find_free_file_entry() {
    for (int i = 0; i < FLASH_MAX_FILES; i++) {
        if (!(directory[i].flags & FLASH_FILE_FLAG_ACTIVE)) {
            return i;
        }
    }
    return -1;
}

uint32_t SPIFlash::find_free_space(uint32_t size) {
    // Simple first-fit allocation
    uint32_t current_address = data_start;
    
    while (current_address + size <= chip_info.capacity) {
        bool space_free = true;
        
        // Check if this space conflicts with any existing file
        for (int i = 0; i < FLASH_MAX_FILES; i++) {
            if (directory[i].flags & FLASH_FILE_FLAG_ACTIVE) {
                uint32_t file_start = directory[i].start_address;
                uint32_t file_end = file_start + directory[i].size;
                
                if (!(current_address >= file_end || current_address + size <= file_start)) {
                    space_free = false;
                    current_address = file_end;
                    break;
                }
            }
        }
        
        if (space_free) {
            return current_address;
        }
    }
    
    return 0; // No space found
}

int8_t SPIFlash::save_directory() {
    return write(filesystem_start, (uint8_t*)directory, FLASH_DIRECTORY_SIZE);
}

// Utility functions
namespace FlashUtils {
    uint32_t calculate_crc32(const uint8_t* data, uint32_t length) {
        uint32_t crc = 0xFFFFFFFF;
        
        for (uint32_t i = 0; i < length; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 1) {
                    crc = (crc >> 1) ^ 0xEDB88320;
                } else {
                    crc = crc >> 1;
                }
            }
        }
        
        return ~crc;
    }
    
    bool verify_data_integrity(const uint8_t* original, const uint8_t* readback, uint32_t length) {
        return memcmp(original, readback, length) == 0;
    }
    
    bool is_page_aligned(uint32_t address) {
        return (address % FLASH_PAGE_SIZE) == 0;
    }
    
    bool is_sector_aligned(uint32_t address) {
        return (address % FLASH_SECTOR_SIZE) == 0;
    }
    
    uint32_t round_up_to_page(uint32_t address) {
        return ((address + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
    }
    
    uint32_t round_up_to_sector(uint32_t address) {
        return ((address + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE;
    }
    
    void format_size_string(uint32_t bytes, char* buffer, size_t buffer_size) {
        if (bytes >= 1024 * 1024) {
            snprintf(buffer, buffer_size, "%.1f MB", bytes / (1024.0f * 1024.0f));
        } else if (bytes >= 1024) {
            snprintf(buffer, buffer_size, "%.1f KB", bytes / 1024.0f);
        } else {
            snprintf(buffer, buffer_size, "%lu bytes", bytes);
        }
    }
}