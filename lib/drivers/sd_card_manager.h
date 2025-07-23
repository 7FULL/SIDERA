#ifndef SD_CARD_MANAGER_H
#define SD_CARD_MANAGER_H

#include <Arduino.h>
#include <SPI.h>

// SD Card Manager Driver
// High-level interface for SD card operations with file system support
// Designed for data logging, configuration storage, and telemetry archiving

// SD Card Types
#define SD_CARD_TYPE_UNKNOWN      0
#define SD_CARD_TYPE_SD1          1    // Standard Capacity (up to 2GB)
#define SD_CARD_TYPE_SD2          2    // Standard Capacity v2.0
#define SD_CARD_TYPE_SDHC         3    // High Capacity (4GB to 32GB)
#define SD_CARD_TYPE_SDXC         4    // Extended Capacity (64GB+)

// File System Types
#define FS_TYPE_UNKNOWN           0
#define FS_TYPE_FAT12             1
#define FS_TYPE_FAT16             2
#define FS_TYPE_FAT32             3
#define FS_TYPE_EXFAT             4

// SPI Configuration
#define SD_SPI_SPEED_INIT         400000    // 400kHz for initialization
#define SD_SPI_SPEED_NORMAL       4000000   // 4MHz for normal operation
#define SD_SPI_SPEED_FAST         8000000   // 8MHz for fast operation
#define SD_SPI_SPEED_MAX          25000000  // 25MHz maximum

// Command timeouts (milliseconds)
#define SD_INIT_TIMEOUT           2000      // Initialization timeout
#define SD_READ_TIMEOUT           500       // Read operation timeout  
#define SD_WRITE_TIMEOUT          1000      // Write operation timeout
#define SD_ERASE_TIMEOUT          5000      // Erase operation timeout

// Block and sector sizes
#define SD_BLOCK_SIZE             512       // Standard SD block size
#define SD_SECTOR_SIZE            512       // Sector size
#define SD_CLUSTER_SIZE_DEFAULT   4096      // Default cluster size

// Error codes
#define SD_SUCCESS                0
#define SD_ERROR_INIT            -1         // Initialization failed
#define SD_ERROR_NO_CARD         -2         // No card present
#define SD_ERROR_CARD_LOCKED     -3         // Card is write protected
#define SD_ERROR_READ            -4         // Read operation failed
#define SD_ERROR_WRITE           -5         // Write operation failed
#define SD_ERROR_ERASE           -6         // Erase operation failed
#define SD_ERROR_TIMEOUT         -7         // Operation timeout
#define SD_ERROR_CRC             -8         // CRC error
#define SD_ERROR_FORMAT          -9         // Format error
#define SD_ERROR_FILE_NOT_FOUND  -10        // File not found
#define SD_ERROR_FILE_EXISTS     -11        // File already exists
#define SD_ERROR_DIR_NOT_FOUND   -12        // Directory not found
#define SD_ERROR_DISK_FULL       -13        // Disk full
#define SD_ERROR_ACCESS_DENIED   -14        // Access denied
#define SD_ERROR_INVALID_PATH    -15        // Invalid file path

// File attributes
#define SD_ATTR_READ_ONLY         0x01
#define SD_ATTR_HIDDEN            0x02
#define SD_ATTR_SYSTEM            0x04
#define SD_ATTR_VOLUME_LABEL      0x08
#define SD_ATTR_DIRECTORY         0x10
#define SD_ATTR_ARCHIVE           0x20

// File access modes
#define SD_FILE_READ              0x01      // Read access
#define SD_FILE_WRITE             0x02      // Write access
#define SD_FILE_APPEND            0x04      // Append mode
#define SD_FILE_CREATE            0x08      // Create if not exists
#define SD_FILE_TRUNCATE          0x10      // Truncate existing file

// Directory entry limits
#define SD_MAX_FILENAME_LENGTH    255       // Maximum filename length
#define SD_MAX_PATH_LENGTH        512       // Maximum path length
#define SD_MAX_DIR_ENTRIES        100       // Maximum directory entries to list

// Data logging configurations
#define SD_LOG_BUFFER_SIZE        1024      // Log buffer size
#define SD_LOG_FLUSH_INTERVAL     5000      // Auto-flush interval (ms)
#define SD_LOG_MAX_FILE_SIZE      10485760  // 10MB max log file size
#define SD_LOG_MAX_FILES          100       // Maximum log files

// Data structures
struct SDCardInfo {
    uint8_t card_type;          // Card type (SD, SDHC, etc.)
    uint8_t fs_type;            // File system type
    uint32_t capacity_mb;       // Card capacity in MB
    uint32_t cluster_size;      // Cluster size in bytes
    uint32_t free_space_mb;     // Free space in MB
    uint32_t total_clusters;    // Total clusters
    uint32_t free_clusters;     // Free clusters
    uint16_t manufacturer_id;   // Manufacturer ID
    char product_name[6];       // Product name
    uint8_t product_revision;   // Product revision
    uint32_t serial_number;     // Serial number
    uint16_t manufacture_date;  // Manufacture date
    bool write_protected;       // Write protection status
    bool card_present;          // Card presence
    char volume_label[12];      // Volume label
};

struct SDFileInfo {
    char filename[SD_MAX_FILENAME_LENGTH + 1]; // Filename
    uint32_t file_size;         // File size in bytes
    uint32_t creation_date;     // Creation date (DOS format)
    uint32_t creation_time;     // Creation time (DOS format)
    uint32_t modified_date;     // Last modified date
    uint32_t modified_time;     // Last modified time
    uint32_t accessed_date;     // Last accessed date
    uint8_t attributes;         // File attributes
    bool is_directory;          // Is directory flag
    uint32_t cluster_start;     // Starting cluster
};

struct SDDirectoryListing {
    SDFileInfo entries[SD_MAX_DIR_ENTRIES]; // Directory entries
    uint16_t num_entries;       // Number of entries
    uint32_t total_size;        // Total size of all files
    uint16_t num_files;         // Number of files
    uint16_t num_directories;   // Number of directories
    char current_path[SD_MAX_PATH_LENGTH]; // Current directory path
};

// File handle structure
struct SDFile {
    bool is_open;               // File is open
    uint8_t access_mode;        // Access mode flags
    uint32_t file_size;         // File size
    uint32_t current_position;  // Current file position
    uint32_t cluster_start;     // Starting cluster
    uint32_t current_cluster;   // Current cluster
    uint32_t cluster_offset;    // Offset within current cluster
    char filename[SD_MAX_FILENAME_LENGTH + 1]; // Filename
    uint8_t buffer[SD_BLOCK_SIZE]; // Read/write buffer
    bool buffer_dirty;          // Buffer needs to be written
    uint32_t buffer_block;      // Block number in buffer
};

// Data logging system
struct SDLogConfig {
    char log_directory[64];     // Log directory path
    char log_prefix[16];        // Log file prefix
    uint32_t max_file_size;     // Maximum log file size
    uint16_t max_files;         // Maximum number of log files
    uint16_t flush_interval_ms; // Auto-flush interval
    bool auto_rotate;           // Auto-rotate log files
    bool timestamp_files;       // Add timestamp to filenames
    bool compress_old_logs;     // Compress old log files
    uint8_t log_level;          // Logging level filter
};

struct SDLogEntry {
    uint32_t timestamp;         // Entry timestamp
    uint8_t log_level;          // Log level (debug, info, warning, error)
    uint8_t source_id;          // Source system ID
    uint16_t data_length;       // Data length
    char message[128];          // Log message
    uint8_t data[128];          // Additional binary data
};

// Performance statistics
struct SDPerformanceStats {
    uint32_t read_operations;   // Number of read operations
    uint32_t write_operations;  // Number of write operations
    uint32_t read_bytes;        // Total bytes read
    uint32_t written_bytes;     // Total bytes written
    uint32_t read_errors;       // Read errors
    uint32_t write_errors;      // Write errors
    uint32_t crc_errors;        // CRC errors
    uint32_t timeout_errors;    // Timeout errors
    float avg_read_speed_kbps;  // Average read speed (KB/s)
    float avg_write_speed_kbps; // Average write speed (KB/s)
    uint32_t total_operation_time; // Total operation time (ms)
};

class SDCardManager {
private:
    uint8_t cs_pin;
    uint32_t spi_speed;
    bool initialized;
    bool card_present;
    
    SDCardInfo card_info;
    SDPerformanceStats performance_stats;
    
    // File system variables
    uint32_t fat_start_sector;
    uint32_t data_start_sector;
    uint32_t root_dir_sector;
    uint16_t bytes_per_sector;
    uint8_t sectors_per_cluster;
    uint16_t reserved_sectors;
    uint8_t num_fats;
    uint32_t sectors_per_fat;
    uint32_t root_dir_entries;
    
    // Open file handles
    SDFile open_files[8];       // Support up to 8 open files
    uint8_t num_open_files;
    
    // Data logging
    SDLogConfig log_config;
    SDFile log_file;
    uint8_t log_buffer[SD_LOG_BUFFER_SIZE];
    uint16_t log_buffer_pos;
    uint32_t last_flush_time;
    uint16_t current_log_file_num;
    
    // Callbacks
    void (*card_detect_callback)(bool card_present);
    void (*error_callback)(int8_t error_code, const char* operation);
    void (*log_full_callback)(const char* log_filename);
    
public:
    SDCardManager(uint8_t chip_select_pin, uint32_t speed = SD_SPI_SPEED_NORMAL);
    
    // Initialization and detection
    int8_t begin();
    int8_t detect_card();
    bool is_initialized() const { return initialized; }
    bool is_card_present() const { return card_present; }
    const SDCardInfo& get_card_info() const { return card_info; }
    
    // Card operations
    int8_t read_card_info();
    int8_t format_card(uint8_t fs_type = FS_TYPE_FAT32);
    int8_t check_card_health();
    int8_t benchmark_card();
    
    // File operations
    int8_t open_file(const char* filename, uint8_t mode, SDFile& file);
    int8_t close_file(SDFile& file);
    int8_t read_file(SDFile& file, void* buffer, uint32_t bytes_to_read, uint32_t& bytes_read);
    int8_t write_file(SDFile& file, const void* data, uint32_t bytes_to_write, uint32_t& bytes_written);
    int8_t seek_file(SDFile& file, uint32_t position);
    int8_t flush_file(SDFile& file);
    
    // File system operations
    int8_t create_file(const char* filename);
    int8_t delete_file(const char* filename);
    int8_t rename_file(const char* old_name, const char* new_name);
    int8_t copy_file(const char* source, const char* destination);
    bool file_exists(const char* filename);
    int32_t get_file_size(const char* filename);
    
    // Directory operations
    int8_t create_directory(const char* dirname);
    int8_t delete_directory(const char* dirname);
    int8_t list_directory(const char* path, SDDirectoryListing& listing);
    int8_t change_directory(const char* path);
    int8_t get_current_directory(char* path, size_t max_length);
    
    // Convenience file operations
    int8_t write_text_file(const char* filename, const char* text);
    int8_t read_text_file(const char* filename, char* buffer, size_t buffer_size);
    int8_t append_text_file(const char* filename, const char* text);
    int8_t write_binary_file(const char* filename, const void* data, uint32_t size);
    int8_t read_binary_file(const char* filename, void* buffer, uint32_t max_size, uint32_t& bytes_read);
    
    // Data logging system
    int8_t configure_logging(const SDLogConfig& config);
    int8_t start_logging();
    int8_t stop_logging();
    int8_t log_message(uint8_t level, uint8_t source_id, const char* message, const void* data = nullptr, uint16_t data_length = 0);
    int8_t flush_log();
    int8_t rotate_log_file();
    bool is_logging_active() const;
    
    // Configuration management
    int8_t save_config(const char* config_name, const void* config_data, uint32_t size);
    int8_t load_config(const char* config_name, void* config_data, uint32_t max_size, uint32_t& size);
    int8_t delete_config(const char* config_name);
    bool config_exists(const char* config_name);
    int8_t list_configs(char config_names[][32], uint8_t max_configs, uint8_t& num_configs);
    
    // Telemetry archiving
    int8_t archive_telemetry_data(const char* session_name, const void* telemetry_data, uint32_t size);
    int8_t create_flight_session(const char* session_name, const char* metadata);
    int8_t close_flight_session(const char* session_name);
    int8_t list_flight_sessions(char session_names[][64], uint8_t max_sessions, uint8_t& num_sessions);
    
    // System utilities
    void update(); // Call regularly for maintenance tasks
    int8_t defragment_card();
    int8_t verify_file_integrity(const char* filename);
    uint32_t get_free_space_mb();
    float get_card_usage_percent();
    
    // Performance and statistics
    const SDPerformanceStats& get_performance_stats() const { return performance_stats; }
    void reset_performance_stats();
    float get_average_write_speed() const;
    float get_average_read_speed() const;
    
    // Callbacks
    void set_card_detect_callback(void (*callback)(bool card_present));
    void set_error_callback(void (*callback)(int8_t error_code, const char* operation));
    void set_log_full_callback(void (*callback)(const char* log_filename));
    
    // Debug and diagnostics
    void print_card_info() const;
    void print_directory_listing(const char* path = "/");
    void print_file_info(const char* filename);
    void print_performance_stats() const;
    void print_open_files() const;
    
    // Advanced operations
    int8_t secure_erase_file(const char* filename);
    int8_t create_backup(const char* backup_name);
    int8_t restore_backup(const char* backup_name);
    int8_t compress_file(const char* source_file, const char* compressed_file);
    int8_t decompress_file(const char* compressed_file, const char* output_file);
    
private:
    // Low-level SD card operations
    int8_t send_command(uint8_t cmd, uint32_t arg, uint8_t* response, uint8_t response_length);
    int8_t wait_for_ready(uint32_t timeout_ms);
    int8_t read_block(uint32_t block_address, uint8_t* buffer);
    int8_t write_block(uint32_t block_address, const uint8_t* data);
    int8_t read_multiple_blocks(uint32_t start_block, uint32_t num_blocks, uint8_t* buffer);
    int8_t write_multiple_blocks(uint32_t start_block, uint32_t num_blocks, const uint8_t* data);
    
    // SPI communication
    void spi_begin_transaction();
    void spi_end_transaction();
    uint8_t spi_transfer(uint8_t data);
    void spi_transfer_block(const uint8_t* tx_buffer, uint8_t* rx_buffer, uint16_t length);
    
    // File system operations
    int8_t read_boot_sector();
    int8_t parse_fat_info();
    uint32_t find_free_cluster();
    int8_t allocate_cluster_chain(uint32_t start_cluster, uint32_t num_clusters);
    int8_t free_cluster_chain(uint32_t start_cluster);
    
    // Directory operations
    int8_t find_directory_entry(const char* filename, uint32_t& sector, uint16_t& offset);
    int8_t create_directory_entry(const char* filename, uint32_t file_size, uint32_t start_cluster);
    int8_t delete_directory_entry(const char* filename);
    int8_t read_directory_entries(uint32_t sector, SDFileInfo* entries, uint16_t max_entries, uint16_t& num_entries);
    
    // File management
    SDFile* find_free_file_handle();
    void close_all_files();
    int8_t flush_file_buffer(SDFile& file);
    
    // Logging system
    int8_t create_log_file();
    int8_t write_log_entry(const SDLogEntry& entry);
    void update_logging();
    
    // Utility functions
    bool is_valid_filename(const char* filename);
    void normalize_path(const char* path, char* normalized_path);
    uint16_t calculate_crc16(const uint8_t* data, uint16_t length);
    uint32_t fat_date_time();
    void dos_date_time_to_timestamp(uint16_t date, uint16_t time, uint32_t& timestamp);
    
    // Error handling
    void handle_error(int8_t error_code, const char* operation);
    void update_performance_stats(bool is_read, uint32_t bytes, uint32_t operation_time_ms, bool success);
};

// Global instance helper
extern SDCardManager* g_sd_card;

// Utility functions and helpers
namespace SDUtils {
    // File path utilities
    bool is_absolute_path(const char* path);
    void join_path(const char* base_path, const char* relative_path, char* result);
    void get_parent_directory(const char* path, char* parent);
    void get_filename_from_path(const char* path, char* filename);
    void get_file_extension(const char* filename, char* extension);
    
    // Filename validation and sanitization
    bool is_valid_dos_name(const char* filename);
    void sanitize_filename(const char* input, char* output);
    void generate_unique_filename(const char* base_name, const char* extension, char* unique_name);
    
    // File size and formatting utilities
    void format_file_size(uint32_t size_bytes, char* buffer, size_t buffer_size);
    void format_date_time(uint32_t timestamp, char* buffer, size_t buffer_size);
    uint32_t parse_file_size_string(const char* size_str); // "1.5MB" -> bytes
    
    // Data conversion utilities
    void binary_to_hex_string(const uint8_t* data, uint16_t length, char* hex_string);
    int16_t hex_string_to_binary(const char* hex_string, uint8_t* data, uint16_t max_length);
    uint32_t string_to_uint32(const char* str);
    void uint32_to_string(uint32_t value, char* str);
    
    // Configuration file helpers
    struct ConfigSection {
        char name[32];
        char key_value_pairs[16][2][64]; // 16 key-value pairs per section
        uint8_t num_pairs;
    };
    
    int8_t parse_config_file(const char* filename, ConfigSection* sections, uint8_t max_sections, uint8_t& num_sections);
    int8_t write_config_file(const char* filename, const ConfigSection* sections, uint8_t num_sections);
    const char* get_config_value(const ConfigSection* sections, uint8_t num_sections, const char* section_name, const char* key);
    
    // CSV file utilities
    int8_t write_csv_header(SDCardManager& sd, const char* filename, const char** column_names, uint8_t num_columns);
    int8_t write_csv_row(SDCardManager& sd, const char* filename, const char** values, uint8_t num_values);
    int8_t read_csv_file(SDCardManager& sd, const char* filename, char*** data, uint16_t& num_rows, uint8_t& num_columns);
    
    // Data logging templates
    struct FlightDataEntry {
        uint32_t timestamp;
        float altitude;
        float velocity;
        float acceleration_x, acceleration_y, acceleration_z;
        float gyro_x, gyro_y, gyro_z;
        float latitude, longitude;
        uint8_t gps_satellites;
        float battery_voltage;
        uint8_t flight_state;
    };
    
    struct TelemetryDataEntry {
        uint32_t timestamp;
        uint8_t packet_type;
        uint8_t source_id;
        uint16_t sequence_number;
        uint8_t data[64];
        uint16_t data_length;
        int8_t rssi;
        uint8_t checksum;
    };
    
    int8_t log_flight_data(SDCardManager& sd, const FlightDataEntry& entry);
    int8_t log_telemetry_data(SDCardManager& sd, const TelemetryDataEntry& entry);
    int8_t create_flight_log_header(SDCardManager& sd, const char* flight_name);
    
    // File compression utilities (simple RLE compression)
    uint32_t compress_data_rle(const uint8_t* input, uint32_t input_size, uint8_t* output, uint32_t max_output_size);
    uint32_t decompress_data_rle(const uint8_t* input, uint32_t input_size, uint8_t* output, uint32_t max_output_size);
    
    // Backup and restore utilities
    struct BackupManifest {
        char backup_name[32];
        uint32_t creation_timestamp;
        uint32_t total_files;
        uint32_t total_size;
        char file_list[100][64]; // List of backed up files
        uint32_t file_checksums[100]; // Checksums for integrity
    };
    
    int8_t create_backup_manifest(const BackupManifest& manifest, const char* manifest_filename);
    int8_t read_backup_manifest(const char* manifest_filename, BackupManifest& manifest);
    int8_t verify_backup_integrity(SDCardManager& sd, const char* backup_name);
    
    // Performance monitoring
    struct SDCardBenchmark {
        float sequential_read_speed;    // MB/s
        float sequential_write_speed;   // MB/s
        float random_read_speed;        // IOPS
        float random_write_speed;       // IOPS
        uint32_t read_latency_avg;      // microseconds
        uint32_t write_latency_avg;     // microseconds
        float fragmentation_level;      // percentage
        uint8_t health_score;           // 0-100
    };
    
    SDCardBenchmark run_performance_benchmark(SDCardManager& sd);
    uint8_t assess_card_health(SDCardManager& sd);
    float calculate_fragmentation_level(SDCardManager& sd);
    
    // File system maintenance
    int8_t run_file_system_check(SDCardManager& sd, bool fix_errors = false);
    int8_t optimize_file_allocation(SDCardManager& sd);
    int8_t clean_temporary_files(SDCardManager& sd);
    
    // Template configurations
    namespace Templates {
        extern const SDLogConfig FLIGHT_LOG_CONFIG;
        extern const SDLogConfig TELEMETRY_LOG_CONFIG;
        extern const SDLogConfig DEBUG_LOG_CONFIG;
        extern const SDLogConfig MINIMAL_LOG_CONFIG;
    }
    
    // Error code to string conversion
    const char* error_code_to_string(int8_t error_code);
    const char* card_type_to_string(uint8_t card_type);
    const char* fs_type_to_string(uint8_t fs_type);
    
    // Development and testing utilities
    int8_t create_test_files(SDCardManager& sd, uint8_t num_files, uint32_t file_size);
    int8_t stress_test_sd_card(SDCardManager& sd, uint32_t duration_seconds);
    void generate_test_data(uint8_t* buffer, uint32_t size, uint8_t pattern = 0);
    bool verify_test_data(const uint8_t* buffer, uint32_t size, uint8_t pattern = 0);
}

#endif // SD_CARD_MANAGER_H