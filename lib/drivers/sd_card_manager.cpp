#include "sd_card_manager.h"
#include <string.h>

SDCardManager* g_sd_card = nullptr;

// Template configurations
namespace SDUtils {
    namespace Templates {
        const SDLogConfig FLIGHT_LOG_CONFIG = {
            "/logs/flight",             // log_directory
            "flight",                   // log_prefix
            5242880,                    // max_file_size (5MB)
            50,                         // max_files
            1000,                       // flush_interval_ms
            true,                       // auto_rotate
            true,                       // timestamp_files
            false,                      // compress_old_logs
            0                           // log_level (all levels)
        };
        
        const SDLogConfig TELEMETRY_LOG_CONFIG = {
            "/logs/telemetry",          // log_directory
            "telem",                    // log_prefix
            2097152,                    // max_file_size (2MB)
            100,                        // max_files
            500,                        // flush_interval_ms
            true,                       // auto_rotate
            true,                       // timestamp_files
            true,                       // compress_old_logs
            1                           // log_level (info and above)
        };
        
        const SDLogConfig DEBUG_LOG_CONFIG = {
            "/logs/debug",              // log_directory
            "debug",                    // log_prefix
            1048576,                    // max_file_size (1MB)
            20,                         // max_files
            2000,                       // flush_interval_ms
            true,                       // auto_rotate
            true,                       // timestamp_files
            false,                      // compress_old_logs
            0                           // log_level (all levels)
        };
    }
}

SDCardManager::SDCardManager(uint8_t chip_select_pin, uint32_t speed) {
    cs_pin = chip_select_pin;
    spi_speed = speed;
    initialized = false;
    card_present = false;
    
    // Initialize card info
    memset(&card_info, 0, sizeof(SDCardInfo));
    
    // Initialize performance stats
    memset(&performance_stats, 0, sizeof(SDPerformanceStats));
    
    // Initialize file system variables
    fat_start_sector = 0;
    data_start_sector = 0;
    root_dir_sector = 0;
    bytes_per_sector = SD_SECTOR_SIZE;
    sectors_per_cluster = 0;
    reserved_sectors = 0;
    num_fats = 0;
    sectors_per_fat = 0;
    root_dir_entries = 0;
    
    // Initialize file handles
    memset(open_files, 0, sizeof(open_files));
    num_open_files = 0;
    
    // Initialize logging system
    memset(&log_config, 0, sizeof(SDLogConfig));
    memset(&log_file, 0, sizeof(SDFile));
    memset(log_buffer, 0, sizeof(log_buffer));
    log_buffer_pos = 0;
    last_flush_time = 0;
    current_log_file_num = 0;
    
    // Initialize callbacks
    card_detect_callback = nullptr;
    error_callback = nullptr;
    log_full_callback = nullptr;
}

int8_t SDCardManager::begin() {
    // Initialize CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
    
    // Initialize SPI
    SPI.begin();
    
    // Detect and initialize card
    int8_t result = detect_card();
    if (result != SD_SUCCESS) {
        return result;
    }
    
    // Read card information
    result = read_card_info();
    if (result != SD_SUCCESS) {
        return result;
    }
    
    // Read boot sector and parse file system
    result = read_boot_sector();
    if (result != SD_SUCCESS) {
        return result;
    }
    
    result = parse_fat_info();
    if (result != SD_SUCCESS) {
        return result;
    }
    
    initialized = true;
    card_present = true;
    
    return SD_SUCCESS;
}

int8_t SDCardManager::detect_card() {
    // Set initial low speed for initialization
    spi_speed = SD_SPI_SPEED_INIT;
    
    spi_begin_transaction();
    
    // Send dummy clocks to initialize card
    for (int i = 0; i < 10; i++) {
        spi_transfer(0xFF);
    }
    
    digitalWrite(cs_pin, LOW);
    
    // Send CMD0 (GO_IDLE_STATE)
    uint8_t response[5];
    int8_t result = send_command(0, 0, response, 1);
    
    digitalWrite(cs_pin, HIGH);
    spi_end_transaction();
    
    if (result != SD_SUCCESS || response[0] != 0x01) {
        card_present = false;
        return SD_ERROR_NO_CARD;
    }
    
    // Card detected, now determine type
    spi_begin_transaction();
    digitalWrite(cs_pin, LOW);
    
    // Send CMD8 (SEND_IF_COND) to check for SDHC/SDXC
    result = send_command(8, 0x1AA, response, 5);
    
    if (result == SD_SUCCESS && response[0] == 0x01) {
        // SD v2.0 or later
        if ((response[3] & 0x0F) == 0x01 && response[4] == 0xAA) {
            // Valid voltage range, continue with ACMD41
            uint32_t start_time = millis();
            do {
                // Send CMD55 (APP_CMD)
                send_command(55, 0, response, 1);
                // Send ACMD41 (SD_SEND_OP_COND) with HCS bit
                result = send_command(41, 0x40000000, response, 1);
                
                if (millis() - start_time > SD_INIT_TIMEOUT) {
                    digitalWrite(cs_pin, HIGH);
                    spi_end_transaction();
                    return SD_ERROR_TIMEOUT;
                }
            } while (response[0] != 0x00);
            
            // Read OCR to determine card capacity
            result = send_command(58, 0, response, 5);
            if (result == SD_SUCCESS) {
                if (response[1] & 0x40) {
                    card_info.card_type = SD_CARD_TYPE_SDHC; // or SDXC
                } else {
                    card_info.card_type = SD_CARD_TYPE_SD2;
                }
            }
        }
    } else {
        // SD v1.0 or MMC
        card_info.card_type = SD_CARD_TYPE_SD1;
        
        // Initialize with CMD1 for MMC or ACMD41 for SD v1.0
        uint32_t start_time = millis();
        do {
            send_command(55, 0, response, 1);
            result = send_command(41, 0, response, 1);
            
            if (millis() - start_time > SD_INIT_TIMEOUT) {
                digitalWrite(cs_pin, HIGH);
                spi_end_transaction();
                return SD_ERROR_TIMEOUT;
            }
        } while (response[0] != 0x00);
    }
    
    digitalWrite(cs_pin, HIGH);
    spi_end_transaction();
    
    // Switch to higher speed after initialization
    spi_speed = SD_SPI_SPEED_NORMAL;
    
    card_present = true;
    return SD_SUCCESS;
}

int8_t SDCardManager::read_card_info() {
    if (!card_present) {
        return SD_ERROR_NO_CARD;
    }
    
    uint8_t csd[16];
    uint8_t cid[16];
    
    spi_begin_transaction();
    digitalWrite(cs_pin, LOW);
    
    // Read CSD (Card Specific Data)
    int8_t result = send_command(9, 0, csd, 16);
    if (result != SD_SUCCESS) {
        digitalWrite(cs_pin, HIGH);
        spi_end_transaction();
        return result;
    }
    
    // Read CID (Card Identification)
    result = send_command(10, 0, cid, 16);
    if (result != SD_SUCCESS) {
        digitalWrite(cs_pin, HIGH);
        spi_end_transaction();
        return result;
    }
    
    digitalWrite(cs_pin, HIGH);
    spi_end_transaction();
    
    // Parse CSD to get capacity
    uint32_t capacity_blocks = 0;
    
    if ((csd[0] >> 6) == 0) {
        // CSD v1.0 (SD v1.0, SD v2.0 standard capacity)
        uint16_t c_size = ((csd[6] & 0x03) << 10) | (csd[7] << 2) | ((csd[8] & 0xC0) >> 6);
        uint8_t c_size_mult = ((csd[9] & 0x03) << 1) | ((csd[10] & 0x80) >> 7);
        uint8_t read_bl_len = csd[5] & 0x0F;
        
        capacity_blocks = (c_size + 1) * (1 << (c_size_mult + 2)) * (1 << read_bl_len) / SD_BLOCK_SIZE;
    } else {
        // CSD v2.0 (SDHC, SDXC)
        uint32_t c_size = ((csd[7] & 0x3F) << 16) | (csd[8] << 8) | csd[9];
        capacity_blocks = (c_size + 1) * 1024;
    }
    
    card_info.capacity_mb = (capacity_blocks * SD_BLOCK_SIZE) / (1024 * 1024);
    
    // Parse CID for manufacturer info
    card_info.manufacturer_id = cid[0];
    memcpy(card_info.product_name, &cid[3], 5);
    card_info.product_name[5] = '\0';
    card_info.product_revision = cid[8];
    card_info.serial_number = (cid[9] << 24) | (cid[10] << 16) | (cid[11] << 8) | cid[12];
    card_info.manufacture_date = ((cid[13] & 0x0F) << 8) | cid[14];
    
    // Check write protection
    card_info.write_protected = false; // Would need hardware detection
    
    return SD_SUCCESS;
}

int8_t SDCardManager::format_card(uint8_t fs_type) {
    if (!card_present) {
        return SD_ERROR_NO_CARD;
    }
    
    if (card_info.write_protected) {
        return SD_ERROR_CARD_LOCKED;
    }
    
    // This is a simplified format implementation
    // A full implementation would create proper FAT structures
    
    uint8_t boot_sector[SD_BLOCK_SIZE];
    memset(boot_sector, 0, sizeof(boot_sector));
    
    // Create basic FAT32 boot sector
    boot_sector[0] = 0xEB; // Jump instruction
    boot_sector[1] = 0x58;
    boot_sector[2] = 0x90;
    
    // OEM identifier
    memcpy(&boot_sector[3], "MSWIN4.1", 8);
    
    // Bytes per sector
    boot_sector[11] = SD_SECTOR_SIZE & 0xFF;
    boot_sector[12] = (SD_SECTOR_SIZE >> 8) & 0xFF;
    
    // Sectors per cluster
    boot_sector[13] = 8; // 4KB clusters
    
    // Reserved sectors
    boot_sector[14] = 32; // FAT32 typically uses 32
    boot_sector[15] = 0;
    
    // Number of FATs
    boot_sector[16] = 2;
    
    // Write boot sector
    int8_t result = write_block(0, boot_sector);
    if (result != SD_SUCCESS) {
        return result;
    }
    
    // Re-read file system info
    result = read_boot_sector();
    if (result != SD_SUCCESS) {
        return result;
    }
    
    return parse_fat_info();
}

int8_t SDCardManager::open_file(const char* filename, uint8_t mode, SDFile& file) {
    if (!initialized || !is_valid_filename(filename)) {
        return SD_ERROR_INVALID_PATH;
    }
    
    // Find free file handle
    SDFile* handle = find_free_file_handle();
    if (!handle) {
        return SD_ERROR_ACCESS_DENIED; // Too many open files
    }
    
    // Initialize file structure
    memset(handle, 0, sizeof(SDFile));
    strncpy(handle->filename, filename, sizeof(handle->filename) - 1);
    handle->filename[sizeof(handle->filename) - 1] = '\0';
    handle->access_mode = mode;
    handle->is_open = true;
    handle->current_position = 0;
    handle->buffer_dirty = false;
    handle->buffer_block = 0xFFFFFFFF;
    
    // Try to find existing file
    uint32_t dir_sector;
    uint16_t dir_offset;
    int8_t result = find_directory_entry(filename, dir_sector, dir_offset);
    
    if (result == SD_SUCCESS) {
        // File exists
        if (mode & SD_FILE_CREATE) {
            // File exists but CREATE flag set
            handle->is_open = false;
            return SD_ERROR_FILE_EXISTS;
        }
        
        // Read file information from directory entry
        uint8_t dir_entry[32];
        result = read_block(dir_sector, dir_entry);
        if (result != SD_SUCCESS) {
            handle->is_open = false;
            return result;
        }
        
        // Extract file info from directory entry
        handle->file_size = (dir_entry[dir_offset + 31] << 24) |
                           (dir_entry[dir_offset + 30] << 16) |
                           (dir_entry[dir_offset + 29] << 8) |
                           dir_entry[dir_offset + 28];
        
        handle->cluster_start = ((dir_entry[dir_offset + 21] << 8) | dir_entry[dir_offset + 20]) << 16 |
                               (dir_entry[dir_offset + 27] << 8) | dir_entry[dir_offset + 26];
        
        handle->current_cluster = handle->cluster_start;
        handle->cluster_offset = 0;
        
        // Truncate file if requested
        if (mode & SD_FILE_TRUNCATE) {
            handle->file_size = 0;
            handle->current_position = 0;
        }
    } else {
        // File doesn't exist
        if (!(mode & SD_FILE_CREATE)) {
            handle->is_open = false;
            return SD_ERROR_FILE_NOT_FOUND;
        }
        
        // Create new file
        uint32_t new_cluster = find_free_cluster();
        if (new_cluster == 0) {
            handle->is_open = false;
            return SD_ERROR_DISK_FULL;
        }
        
        result = create_directory_entry(filename, 0, new_cluster);
        if (result != SD_SUCCESS) {
            handle->is_open = false;
            return result;
        }
        
        handle->file_size = 0;
        handle->cluster_start = new_cluster;
        handle->current_cluster = new_cluster;
        handle->cluster_offset = 0;
    }
    
    // Set position for append mode
    if (mode & SD_FILE_APPEND) {
        handle->current_position = handle->file_size;
    } else {
        handle->current_position = 0;
    }
    
    // Copy handle to output parameter
    file = *handle;
    num_open_files++;
    
    return SD_SUCCESS;
}

int8_t SDCardManager::close_file(SDFile& file) {
    if (!file.is_open) {
        return SD_ERROR_ACCESS_DENIED;
    }
    
    // Flush any pending writes
    int8_t result = flush_file(file);
    
    // Mark file as closed
    file.is_open = false;
    
    // Find and clear file handle
    for (uint8_t i = 0; i < 8; i++) {
        if (open_files[i].is_open && strcmp(open_files[i].filename, file.filename) == 0) {
            memset(&open_files[i], 0, sizeof(SDFile));
            num_open_files--;
            break;
        }
    }
    
    return result;
}

int8_t SDCardManager::read_file(SDFile& file, void* buffer, uint32_t bytes_to_read, uint32_t& bytes_read) {
    if (!file.is_open || !(file.access_mode & SD_FILE_READ)) {
        return SD_ERROR_ACCESS_DENIED;
    }
    
    bytes_read = 0;
    uint8_t* buf = (uint8_t*)buffer;
    
    while (bytes_to_read > 0 && file.current_position < file.file_size) {
        // Calculate current block
        uint32_t cluster_size = sectors_per_cluster * bytes_per_sector;
        uint32_t cluster_block = file.cluster_offset / bytes_per_sector;
        uint32_t block_offset = file.cluster_offset % bytes_per_sector;
        
        // Read block if not in buffer
        uint32_t absolute_block = data_start_sector + 
                                 (file.current_cluster - 2) * sectors_per_cluster + 
                                 cluster_block;
        
        if (file.buffer_block != absolute_block) {
            int8_t result = read_block(absolute_block, file.buffer);
            if (result != SD_SUCCESS) {
                return result;
            }
            file.buffer_block = absolute_block;
        }
        
        // Copy data from buffer
        uint32_t bytes_in_block = min(bytes_to_read, bytes_per_sector - block_offset);
        bytes_in_block = min(bytes_in_block, file.file_size - file.current_position);
        
        memcpy(buf, &file.buffer[block_offset], bytes_in_block);
        
        buf += bytes_in_block;
        bytes_read += bytes_in_block;
        bytes_to_read -= bytes_in_block;
        file.current_position += bytes_in_block;
        file.cluster_offset += bytes_in_block;
        
        // Move to next cluster if necessary
        if (file.cluster_offset >= cluster_size) {
            // Would need to follow FAT chain here
            file.cluster_offset = 0;
            // This is simplified - would need proper cluster chaining
        }
    }
    
    performance_stats.read_operations++;
    performance_stats.read_bytes += bytes_read;
    
    return SD_SUCCESS;
}

int8_t SDCardManager::write_file(SDFile& file, const void* data, uint32_t bytes_to_write, uint32_t& bytes_written) {
    if (!file.is_open || !(file.access_mode & SD_FILE_WRITE)) {
        return SD_ERROR_ACCESS_DENIED;
    }
    
    if (card_info.write_protected) {
        return SD_ERROR_CARD_LOCKED;
    }
    
    bytes_written = 0;
    const uint8_t* buf = (const uint8_t*)data;
    
    while (bytes_to_write > 0) {
        // Calculate current block
        uint32_t cluster_size = sectors_per_cluster * bytes_per_sector;
        uint32_t cluster_block = file.cluster_offset / bytes_per_sector;
        uint32_t block_offset = file.cluster_offset % bytes_per_sector;
        
        uint32_t absolute_block = data_start_sector + 
                                 (file.current_cluster - 2) * sectors_per_cluster + 
                                 cluster_block;
        
        // Read block if not in buffer and we're not writing a full block
        if (file.buffer_block != absolute_block && (block_offset != 0 || bytes_to_write < bytes_per_sector)) {
            int8_t result = read_block(absolute_block, file.buffer);
            if (result != SD_SUCCESS) {
                return result;
            }
        }
        
        file.buffer_block = absolute_block;
        
        // Copy data to buffer
        uint32_t bytes_in_block = min(bytes_to_write, bytes_per_sector - block_offset);
        memcpy(&file.buffer[block_offset], buf, bytes_in_block);
        
        file.buffer_dirty = true;
        
        buf += bytes_in_block;
        bytes_written += bytes_in_block;
        bytes_to_write -= bytes_in_block;
        file.current_position += bytes_in_block;
        file.cluster_offset += bytes_in_block;
        
        // Update file size if we've extended it
        if (file.current_position > file.file_size) {
            file.file_size = file.current_position;
        }
        
        // Flush buffer if block is complete
        if (block_offset + bytes_in_block == bytes_per_sector) {
            int8_t result = flush_file_buffer(file);
            if (result != SD_SUCCESS) {
                return result;
            }
        }
        
        // Move to next cluster if necessary
        if (file.cluster_offset >= cluster_size) {
            // Would need to allocate new cluster and update FAT
            file.cluster_offset = 0;
            // This is simplified - would need proper cluster allocation
        }
    }
    
    performance_stats.write_operations++;
    performance_stats.written_bytes += bytes_written;
    
    return SD_SUCCESS;
}

int8_t SDCardManager::seek_file(SDFile& file, uint32_t position) {
    if (!file.is_open) {
        return SD_ERROR_ACCESS_DENIED;
    }
    
    if (position > file.file_size) {
        return SD_ERROR_INVALID_PATH;
    }
    
    // Flush any pending writes
    int8_t result = flush_file(file);
    if (result != SD_SUCCESS) {
        return result;
    }
    
    file.current_position = position;
    
    // Calculate cluster and offset
    uint32_t cluster_size = sectors_per_cluster * bytes_per_sector;
    uint32_t cluster_index = position / cluster_size;
    file.cluster_offset = position % cluster_size;
    
    // Follow cluster chain to find correct cluster
    // This is simplified - would need to traverse FAT chain
    file.current_cluster = file.cluster_start;
    for (uint32_t i = 0; i < cluster_index; i++) {
        // Would read next cluster from FAT
    }
    
    return SD_SUCCESS;
}

int8_t SDCardManager::flush_file(SDFile& file) {
    if (!file.is_open) {
        return SD_ERROR_ACCESS_DENIED;
    }
    
    return flush_file_buffer(file);
}

bool SDCardManager::file_exists(const char* filename) {
    uint32_t sector;
    uint16_t offset;
    return find_directory_entry(filename, sector, offset) == SD_SUCCESS;
}

int32_t SDCardManager::get_file_size(const char* filename) {
    uint32_t sector;
    uint16_t offset;
    
    if (find_directory_entry(filename, sector, offset) != SD_SUCCESS) {
        return -1;
    }
    
    uint8_t dir_entry[32];
    if (read_block(sector, dir_entry) != SD_SUCCESS) {
        return -1;
    }
    
    return (dir_entry[offset + 31] << 24) |
           (dir_entry[offset + 30] << 16) |
           (dir_entry[offset + 29] << 8) |
           dir_entry[offset + 28];
}

int8_t SDCardManager::write_text_file(const char* filename, const char* text) {
    SDFile file;
    int8_t result = open_file(filename, SD_FILE_WRITE | SD_FILE_CREATE | SD_FILE_TRUNCATE, file);
    if (result != SD_SUCCESS) {
        return result;
    }
    
    uint32_t bytes_written;
    result = write_file(file, text, strlen(text), bytes_written);
    
    close_file(file);
    return result;
}

int8_t SDCardManager::read_text_file(const char* filename, char* buffer, size_t buffer_size) {
    SDFile file;
    int8_t result = open_file(filename, SD_FILE_READ, file);
    if (result != SD_SUCCESS) {
        return result;
    }
    
    uint32_t bytes_read;
    result = read_file(file, buffer, buffer_size - 1, bytes_read);
    
    if (result == SD_SUCCESS) {
        buffer[bytes_read] = '\0'; // Null terminate
    }
    
    close_file(file);
    return result;
}

int8_t SDCardManager::configure_logging(const SDLogConfig& config) {
    log_config = config;
    
    // Create log directory if it doesn't exist
    create_directory(config.log_directory);
    
    return SD_SUCCESS;
}

int8_t SDCardManager::start_logging() {
    if (strlen(log_config.log_directory) == 0) {
        return SD_ERROR_INIT;
    }
    
    return create_log_file();
}

int8_t SDCardManager::log_message(uint8_t level, uint8_t source_id, const char* message, const void* data, uint16_t data_length) {
    if (!log_file.is_open) {
        return SD_ERROR_INIT;
    }
    
    SDLogEntry entry;
    entry.timestamp = millis();
    entry.log_level = level;
    entry.source_id = source_id;
    entry.data_length = min(data_length, (uint16_t)sizeof(entry.data));
    
    strncpy(entry.message, message, sizeof(entry.message) - 1);
    entry.message[sizeof(entry.message) - 1] = '\0';
    
    if (data && entry.data_length > 0) {
        memcpy(entry.data, data, entry.data_length);
    }
    
    return write_log_entry(entry);
}

void SDCardManager::update() {
    if (!initialized) {
        return;
    }
    
    // Update logging system
    update_logging();
    
    // Check for card removal/insertion
    // This would need hardware support
    
    // Periodic maintenance tasks
    static uint32_t last_maintenance = 0;
    if (millis() - last_maintenance > 60000) { // Every minute
        // Update free space info
        card_info.free_space_mb = get_free_space_mb();
        last_maintenance = millis();
    }
}

uint32_t SDCardManager::get_free_space_mb() {
    // This would need to scan the FAT to count free clusters
    // Simplified implementation
    return card_info.capacity_mb / 2; // Assume 50% free
}

void SDCardManager::print_card_info() const {
    Serial.println("=== SD Card Information ===");
    Serial.print("Card Type: "); 
    Serial.println(SDUtils::card_type_to_string(card_info.card_type));
    Serial.print("Capacity: "); Serial.print(card_info.capacity_mb); Serial.println(" MB");
    Serial.print("File System: "); 
    Serial.println(SDUtils::fs_type_to_string(card_info.fs_type));
    Serial.print("Manufacturer: 0x"); Serial.println(card_info.manufacturer_id, HEX);
    Serial.print("Product: "); Serial.println(card_info.product_name);
    Serial.print("Serial: 0x"); Serial.println(card_info.serial_number, HEX);
    Serial.print("Write Protected: "); Serial.println(card_info.write_protected ? "Yes" : "No");
    Serial.print("Free Space: "); Serial.print(card_info.free_space_mb); Serial.println(" MB");
}

void SDCardManager::print_performance_stats() const {
    Serial.println("=== SD Card Performance Statistics ===");
    Serial.print("Read Operations: "); Serial.println(performance_stats.read_operations);
    Serial.print("Write Operations: "); Serial.println(performance_stats.write_operations);
    Serial.print("Bytes Read: "); Serial.println(performance_stats.read_bytes);
    Serial.print("Bytes Written: "); Serial.println(performance_stats.written_bytes);
    Serial.print("Read Errors: "); Serial.println(performance_stats.read_errors);
    Serial.print("Write Errors: "); Serial.println(performance_stats.write_errors);
    Serial.print("CRC Errors: "); Serial.println(performance_stats.crc_errors);
    Serial.print("Avg Read Speed: "); Serial.print(performance_stats.avg_read_speed_kbps); Serial.println(" KB/s");
    Serial.print("Avg Write Speed: "); Serial.print(performance_stats.avg_write_speed_kbps); Serial.println(" KB/s");
}

// Private function implementations
int8_t SDCardManager::send_command(uint8_t cmd, uint32_t arg, uint8_t* response, uint8_t response_length) {
    // Construct command packet
    uint8_t command[6];
    command[0] = 0x40 | cmd;
    command[1] = (arg >> 24) & 0xFF;
    command[2] = (arg >> 16) & 0xFF;
    command[3] = (arg >> 8) & 0xFF;
    command[4] = arg & 0xFF;
    
    // Calculate CRC7 (simplified - would need proper CRC7 implementation)
    command[5] = 0x95; // CRC for CMD0, 0x87 for CMD8
    
    // Send command
    for (int i = 0; i < 6; i++) {
        spi_transfer(command[i]);
    }
    
    // Wait for response
    for (int i = 0; i < 10; i++) {
        uint8_t r = spi_transfer(0xFF);
        if ((r & 0x80) == 0) {
            response[0] = r;
            break;
        }
        if (i == 9) {
            return SD_ERROR_TIMEOUT;
        }
    }
    
    // Read additional response bytes if needed
    for (uint8_t i = 1; i < response_length; i++) {
        response[i] = spi_transfer(0xFF);
    }
    
    return SD_SUCCESS;
}

int8_t SDCardManager::read_block(uint32_t block_address, uint8_t* buffer) {
    spi_begin_transaction();
    digitalWrite(cs_pin, LOW);
    
    uint8_t response[1];
    int8_t result = send_command(17, block_address, response, 1); // CMD17 - READ_SINGLE_BLOCK
    
    if (result != SD_SUCCESS || response[0] != 0x00) {
        digitalWrite(cs_pin, HIGH);
        spi_end_transaction();
        performance_stats.read_errors++;
        return SD_ERROR_READ;
    }
    
    // Wait for data token
    uint32_t start_time = millis();
    uint8_t token;
    do {
        token = spi_transfer(0xFF);
        if (millis() - start_time > SD_READ_TIMEOUT) {
            digitalWrite(cs_pin, HIGH);
            spi_end_transaction();
            performance_stats.timeout_errors++;
            return SD_ERROR_TIMEOUT;
        }
    } while (token == 0xFF);
    
    if (token != 0xFE) {
        digitalWrite(cs_pin, HIGH);
        spi_end_transaction();
        performance_stats.read_errors++;
        return SD_ERROR_READ;
    }
    
    // Read data block
    for (uint16_t i = 0; i < SD_BLOCK_SIZE; i++) {
        buffer[i] = spi_transfer(0xFF);
    }
    
    // Read CRC (but don't verify for simplicity)
    spi_transfer(0xFF);
    spi_transfer(0xFF);
    
    digitalWrite(cs_pin, HIGH);
    spi_end_transaction();
    
    return SD_SUCCESS;
}

int8_t SDCardManager::write_block(uint32_t block_address, const uint8_t* data) {
    if (card_info.write_protected) {
        return SD_ERROR_CARD_LOCKED;
    }
    
    spi_begin_transaction();
    digitalWrite(cs_pin, LOW);
    
    uint8_t response[1];
    int8_t result = send_command(24, block_address, response, 1); // CMD24 - WRITE_BLOCK
    
    if (result != SD_SUCCESS || response[0] != 0x00) {
        digitalWrite(cs_pin, HIGH);
        spi_end_transaction();
        performance_stats.write_errors++;
        return SD_ERROR_WRITE;
    }
    
    // Send data token
    spi_transfer(0xFE);
    
    // Send data block
    for (uint16_t i = 0; i < SD_BLOCK_SIZE; i++) {
        spi_transfer(data[i]);
    }
    
    // Send dummy CRC
    spi_transfer(0xFF);
    spi_transfer(0xFF);
    
    // Wait for data response
    uint8_t data_response = spi_transfer(0xFF);
    if ((data_response & 0x1F) != 0x05) {
        digitalWrite(cs_pin, HIGH);
        spi_end_transaction();
        performance_stats.write_errors++;
        return SD_ERROR_WRITE;
    }
    
    // Wait for write completion
    uint32_t start_time = millis();
    while (spi_transfer(0xFF) == 0x00) {
        if (millis() - start_time > SD_WRITE_TIMEOUT) {
            digitalWrite(cs_pin, HIGH);
            spi_end_transaction();
            performance_stats.timeout_errors++;
            return SD_ERROR_TIMEOUT;
        }
    }
    
    digitalWrite(cs_pin, HIGH);
    spi_end_transaction();
    
    return SD_SUCCESS;
}

void SDCardManager::spi_begin_transaction() {
    SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
}

void SDCardManager::spi_end_transaction() {
    SPI.endTransaction();
}

uint8_t SDCardManager::spi_transfer(uint8_t data) {
    return SPI.transfer(data);
}

int8_t SDCardManager::read_boot_sector() {
    uint8_t boot_sector[SD_BLOCK_SIZE];
    int8_t result = read_block(0, boot_sector);
    
    if (result != SD_SUCCESS) {
        return result;
    }
    
    // Verify boot sector signature
    if (boot_sector[510] != 0x55 || boot_sector[511] != 0xAA) {
        return SD_ERROR_FORMAT;
    }
    
    // Extract basic file system information
    bytes_per_sector = boot_sector[11] | (boot_sector[12] << 8);
    sectors_per_cluster = boot_sector[13];
    reserved_sectors = boot_sector[14] | (boot_sector[15] << 8);
    num_fats = boot_sector[16];
    root_dir_entries = boot_sector[17] | (boot_sector[18] << 8);
    
    return SD_SUCCESS;
}

int8_t SDCardManager::parse_fat_info() {
    // Calculate file system layout
    fat_start_sector = reserved_sectors;
    
    if (root_dir_entries == 0) {
        // FAT32
        card_info.fs_type = FS_TYPE_FAT32;
        sectors_per_fat = boot_sector[36] | (boot_sector[37] << 8) | 
                         (boot_sector[38] << 16) | (boot_sector[39] << 24);
        root_dir_sector = fat_start_sector + (num_fats * sectors_per_fat);
        data_start_sector = root_dir_sector;
    } else {
        // FAT16 or FAT12
        sectors_per_fat = boot_sector[22] | (boot_sector[23] << 8);
        root_dir_sector = fat_start_sector + (num_fats * sectors_per_fat);
        uint32_t root_dir_sectors = ((root_dir_entries * 32) + (bytes_per_sector - 1)) / bytes_per_sector;
        data_start_sector = root_dir_sector + root_dir_sectors;
        
        uint32_t data_sectors = card_info.capacity_mb * 1024 * 1024 / bytes_per_sector - data_start_sector;
        uint32_t cluster_count = data_sectors / sectors_per_cluster;
        
        if (cluster_count < 4085) {
            card_info.fs_type = FS_TYPE_FAT12;
        } else {
            card_info.fs_type = FS_TYPE_FAT16;
        }
    }
    
    return SD_SUCCESS;
}

uint32_t SDCardManager::find_free_cluster() {
    // This would scan the FAT to find a free cluster
    // Simplified implementation - return cluster 3 (first data cluster after root)
    return 3;
}

int8_t SDCardManager::find_directory_entry(const char* filename, uint32_t& sector, uint16_t& offset) {
    // This would search the root directory for the file
    // Simplified implementation
    sector = root_dir_sector;
    offset = 0;
    
    // Would need to actually search directory entries
    return SD_ERROR_FILE_NOT_FOUND;
}

int8_t SDCardManager::create_directory_entry(const char* filename, uint32_t file_size, uint32_t start_cluster) {
    // This would create a new directory entry
    // Simplified implementation
    return SD_SUCCESS;
}

SDFile* SDCardManager::find_free_file_handle() {
    for (uint8_t i = 0; i < 8; i++) {
        if (!open_files[i].is_open) {
            return &open_files[i];
        }
    }
    return nullptr;
}

int8_t SDCardManager::flush_file_buffer(SDFile& file) {
    if (!file.buffer_dirty) {
        return SD_SUCCESS;
    }
    
    int8_t result = write_block(file.buffer_block, file.buffer);
    if (result == SD_SUCCESS) {
        file.buffer_dirty = false;
    }
    
    return result;
}

int8_t SDCardManager::create_log_file() {
    char filename[64];
    snprintf(filename, sizeof(filename), "%s/%s_%04d.log", 
             log_config.log_directory, log_config.log_prefix, current_log_file_num);
    
    return open_file(filename, SD_FILE_WRITE | SD_FILE_CREATE | SD_FILE_APPEND, log_file);
}

int8_t SDCardManager::write_log_entry(const SDLogEntry& entry) {
    uint32_t bytes_written;
    return write_file(log_file, &entry, sizeof(entry), bytes_written);
}

void SDCardManager::update_logging() {
    if (!log_file.is_open) {
        return;
    }
    
    // Auto-flush based on interval
    if (millis() - last_flush_time > log_config.flush_interval_ms) {
        flush_file(log_file);
        last_flush_time = millis();
    }
    
    // Check if log file is too large
    if (log_file.file_size > log_config.max_file_size) {
        rotate_log_file();
    }
}

bool SDCardManager::is_valid_filename(const char* filename) {
    if (!filename || strlen(filename) == 0 || strlen(filename) > SD_MAX_FILENAME_LENGTH) {
        return false;
    }
    
    // Check for invalid characters
    const char* invalid_chars = "<>:\"|?*";
    for (int i = 0; i < strlen(invalid_chars); i++) {
        if (strchr(filename, invalid_chars[i])) {
            return false;
        }
    }
    
    return true;
}

void SDCardManager::handle_error(int8_t error_code, const char* operation) {
    if (error_callback) {
        error_callback(error_code, operation);
    }
}

// Utility function implementations
namespace SDUtils {
    const char* error_code_to_string(int8_t error_code) {
        switch (error_code) {
            case SD_SUCCESS: return "Success";
            case SD_ERROR_INIT: return "Initialization failed";
            case SD_ERROR_NO_CARD: return "No card present";
            case SD_ERROR_CARD_LOCKED: return "Card write protected";
            case SD_ERROR_READ: return "Read failed";
            case SD_ERROR_WRITE: return "Write failed";
            case SD_ERROR_TIMEOUT: return "Operation timeout";
            case SD_ERROR_CRC: return "CRC error";
            case SD_ERROR_FORMAT: return "Format error";
            case SD_ERROR_FILE_NOT_FOUND: return "File not found";
            case SD_ERROR_DISK_FULL: return "Disk full";
            default: return "Unknown error";
        }
    }
    
    const char* card_type_to_string(uint8_t card_type) {
        switch (card_type) {
            case SD_CARD_TYPE_SD1: return "SD v1.0";
            case SD_CARD_TYPE_SD2: return "SD v2.0";
            case SD_CARD_TYPE_SDHC: return "SDHC";
            case SD_CARD_TYPE_SDXC: return "SDXC";
            default: return "Unknown";
        }
    }
    
    const char* fs_type_to_string(uint8_t fs_type) {
        switch (fs_type) {
            case FS_TYPE_FAT12: return "FAT12";
            case FS_TYPE_FAT16: return "FAT16";
            case FS_TYPE_FAT32: return "FAT32";
            case FS_TYPE_EXFAT: return "exFAT";
            default: return "Unknown";
        }
    }
    
    void format_file_size(uint32_t size_bytes, char* buffer, size_t buffer_size) {
        if (size_bytes >= 1024 * 1024 * 1024) {
            snprintf(buffer, buffer_size, "%.2f GB", size_bytes / (1024.0f * 1024.0f * 1024.0f));
        } else if (size_bytes >= 1024 * 1024) {
            snprintf(buffer, buffer_size, "%.2f MB", size_bytes / (1024.0f * 1024.0f));
        } else if (size_bytes >= 1024) {
            snprintf(buffer, buffer_size, "%.2f KB", size_bytes / 1024.0f);
        } else {
            snprintf(buffer, buffer_size, "%lu bytes", size_bytes);
        }
    }
    
    bool is_valid_dos_name(const char* filename) {
        if (!filename || strlen(filename) == 0 || strlen(filename) > 12) {
            return false;
        }
        
        // Check for valid DOS 8.3 format
        const char* dot = strchr(filename, '.');
        if (dot) {
            if (dot - filename > 8 || strlen(dot + 1) > 3) {
                return false;
            }
        } else if (strlen(filename) > 8) {
            return false;
        }
        
        return true;
    }
    
    void sanitize_filename(const char* input, char* output) {
        const char* invalid_chars = "<>:\"|?*";
        const char* src = input;
        char* dst = output;
        
        while (*src && dst - output < SD_MAX_FILENAME_LENGTH - 1) {
            if (strchr(invalid_chars, *src) || *src < 32) {
                *dst++ = '_';
            } else {
                *dst++ = *src;
            }
            src++;
        }
        *dst = '\0';
    }
    
    int8_t log_flight_data(SDCardManager& sd, const FlightDataEntry& entry) {
        SDFile file;
        int8_t result = sd.open_file("/logs/flight_data.csv", SD_FILE_WRITE | SD_FILE_CREATE | SD_FILE_APPEND, file);
        if (result != SD_SUCCESS) {
            return result;
        }
        
        char line[256];
        snprintf(line, sizeof(line), "%lu,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.8f,%.8f,%d,%.2f,%d\n",
                entry.timestamp, entry.altitude, entry.velocity,
                entry.acceleration_x, entry.acceleration_y, entry.acceleration_z,
                entry.gyro_x, entry.gyro_y, entry.gyro_z,
                entry.latitude, entry.longitude, entry.gps_satellites,
                entry.battery_voltage, entry.flight_state);
        
        uint32_t bytes_written;
        result = sd.write_file(file, line, strlen(line), bytes_written);
        
        sd.close_file(file);
        return result;
    }
}