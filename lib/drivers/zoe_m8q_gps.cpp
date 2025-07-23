#include "zoe_m8q_gps.h"
#include <math.h>

ZOE_M8Q_GPS* g_gps = nullptr;

ZOE_M8Q_GPS::ZOE_M8Q_GPS(HardwareSerial& serial, uint32_t baud) {
    gps_serial = &serial;
    soft_serial = nullptr;
    baud_rate = baud;
    use_hardware_serial = true;
    initialized = false;
    
    // Initialize data structures
    memset(&current_data, 0, sizeof(GPS_Data));
    
    // Initialize parsing state
    parse_state = 0;
    nmea_index = 0;
    ubx_index = 0;
    memset(&ubx_packet, 0, sizeof(UBX_Packet));
    
    // Default configuration
    update_rate_ms = 1000;  // 1Hz default
    power_mode = GPS_POWER_FULL;
    ubx_protocol_enabled = true;
    nmea_protocol_enabled = true;
    
    // Initialize statistics
    packets_received = 0;
    packets_parsed = 0;
    checksum_errors = 0;
    timeout_errors = 0;
    
    last_fix_time = 0;
    last_update_time = 0;
}

ZOE_M8Q_GPS::ZOE_M8Q_GPS(uint8_t rx_pin, uint8_t tx_pin, uint32_t baud) {
    gps_serial = nullptr;
    soft_serial = new SoftwareSerial(rx_pin, tx_pin);
    baud_rate = baud;
    use_hardware_serial = false;
    initialized = false;
    
    // Initialize same as hardware serial constructor
    memset(&current_data, 0, sizeof(GPS_Data));
    parse_state = 0;
    nmea_index = 0;
    ubx_index = 0;
    memset(&ubx_packet, 0, sizeof(UBX_Packet));
    
    update_rate_ms = 1000;
    power_mode = GPS_POWER_FULL;
    ubx_protocol_enabled = true;
    nmea_protocol_enabled = true;
    
    packets_received = 0;
    packets_parsed = 0;
    checksum_errors = 0;
    timeout_errors = 0;
    
    last_fix_time = 0;
    last_update_time = 0;
}

int8_t ZOE_M8Q_GPS::begin() {
    // Initialize serial communication
    if (use_hardware_serial) {
        gps_serial->begin(baud_rate);
    } else {
        soft_serial->begin(baud_rate);
    }
    
    delay(1000); // Wait for GPS to boot
    
    // Try to communicate with GPS
    for (int i = 0; i < 5; i++) {
        // Send a simple UBX message to test communication
        if (send_ubx_packet(UBX_CLASS_CFG, UBX_CFG_PRT, nullptr, 0) == GPS_SUCCESS) {
            initialized = true;
            break;
        }
        delay(1000);
    }
    
    if (!initialized) {
        return GPS_ERROR_INIT;
    }
    
    // Configure update rate
    set_update_rate(update_rate_ms);
    
    // Enable desired messages
    if (ubx_protocol_enabled) {
        enable_ubx_messages(true);
    }
    
    return GPS_SUCCESS;
}

int8_t ZOE_M8Q_GPS::reset() {
    // Send reset command
    uint8_t payload[4] = {0xFF, 0xFF, 0x00, 0x00}; // Hot start
    
    if (send_ubx_packet(UBX_CLASS_CFG, UBX_CFG_RST, payload, 4) != GPS_SUCCESS) {
        return GPS_ERROR_CONFIG;
    }
    
    delay(1000); // Wait for reset
    
    return begin(); // Re-initialize
}

int8_t ZOE_M8Q_GPS::set_update_rate(uint16_t rate_ms) {
    // CFG-RATE message payload
    uint8_t payload[6];
    payload[0] = rate_ms & 0xFF;        // measRate LSB
    payload[1] = (rate_ms >> 8) & 0xFF; // measRate MSB
    payload[2] = 0x01;                  // navRate (always 1)
    payload[3] = 0x00;
    payload[4] = 0x01;                  // timeRef (GPS time)
    payload[5] = 0x00;
    
    if (send_ubx_packet(UBX_CLASS_CFG, UBX_CFG_RATE, payload, 6) == GPS_SUCCESS) {
        if (wait_for_ack(UBX_CLASS_CFG, UBX_CFG_RATE)) {
            update_rate_ms = rate_ms;
            return GPS_SUCCESS;
        }
    }
    
    return GPS_ERROR_CONFIG;
}

int8_t ZOE_M8Q_GPS::enable_ubx_messages(bool enable) {
    // Enable/disable NAV-PVT message (most comprehensive)
    uint8_t payload[3];
    payload[0] = UBX_CLASS_NAV;  // Message class
    payload[1] = UBX_NAV_PVT;    // Message ID
    payload[2] = enable ? 1 : 0; // Rate (0 = disabled, 1 = every solution)
    
    return send_ubx_packet(UBX_CLASS_CFG, UBX_CFG_MSG, payload, 3);
}

void ZOE_M8Q_GPS::update() {
    Stream* stream = use_hardware_serial ? (Stream*)gps_serial : (Stream*)soft_serial;
    
    while (stream->available()) {
        uint8_t byte = stream->read();
        packets_received++;
        
        // Check for UBX sync characters
        if (byte == UBX_SYNC_CHAR_1 && parse_state == 0) {
            parse_state = 1;
            ubx_index = 0;
            ubx_packet.sync1 = byte;
        } else if (byte == UBX_SYNC_CHAR_2 && parse_state == 1) {
            parse_state = 2;
            ubx_packet.sync2 = byte;
        } else if (parse_state > 1 && parse_state < 8) {
            // Parse UBX header
            switch (parse_state) {
                case 2: ubx_packet.msg_class = byte; parse_state++; break;
                case 3: ubx_packet.msg_id = byte; parse_state++; break;
                case 4: ubx_packet.length = byte; parse_state++; break;
                case 5: 
                    ubx_packet.length |= (byte << 8); 
                    parse_state++;
                    ubx_index = 0;
                    if (ubx_packet.length > GPS_BUFFER_SIZE) {
                        reset_parse_state(); // Invalid length
                    }
                    break;
                case 6:
                    if (ubx_index < ubx_packet.length) {
                        ubx_packet.payload[ubx_index++] = byte;
                        if (ubx_index >= ubx_packet.length) {
                            parse_state++;
                        }
                    }
                    break;
                case 7: ubx_packet.checksum_a = byte; parse_state++; break;
            }
        } else if (parse_state == 8) {
            // Complete UBX packet
            ubx_packet.checksum_b = byte;
            
            // Validate checksum
            uint8_t ck_a, ck_b;
            calculate_ubx_checksum(ubx_packet, ck_a, ck_b);
            
            if (ck_a == ubx_packet.checksum_a && ck_b == ubx_packet.checksum_b) {
                parse_ubx_packet();
                packets_parsed++;
            } else {
                checksum_errors++;
            }
            
            reset_parse_state();
        } else {
            // NMEA parsing
            if (byte == '$') {
                nmea_index = 0;
                nmea_buffer[nmea_index++] = byte;
            } else if (nmea_index > 0 && nmea_index < GPS_BUFFER_SIZE - 1) {
                nmea_buffer[nmea_index++] = byte;
                
                if (byte == '\n' || byte == '\r') {
                    nmea_buffer[nmea_index] = '\0';
                    
                    if (validate_nmea_checksum((char*)nmea_buffer)) {
                        parse_nmea_sentence();
                        packets_parsed++;
                    } else {
                        checksum_errors++;
                    }
                    
                    nmea_index = 0;
                }
            } else {
                reset_parse_state();
            }
        }
    }
    
    update_statistics();
}

bool ZOE_M8Q_GPS::has_fix() const {
    return current_data.position.fix_type >= GPS_FIX_2D && current_data.position.valid;
}

bool ZOE_M8Q_GPS::has_new_data() const {
    return (millis() - last_update_time) < (update_rate_ms * 2);
}

float ZOE_M8Q_GPS::calculate_distance(double lat1, double lon1, double lat2, double lon2) {
    return GPS_Utils::haversine_distance(lat1, lon1, lat2, lon2);
}

float ZOE_M8Q_GPS::calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
    return GPS_Utils::great_circle_bearing(lat1, lon1, lat2, lon2);
}

bool ZOE_M8Q_GPS::is_inside_geofence(double center_lat, double center_lon, float radius_m) {
    if (!has_fix()) return false;
    
    float distance = calculate_distance(current_data.position.latitude, current_data.position.longitude,
                                       center_lat, center_lon);
    return distance <= radius_m;
}

uint32_t ZOE_M8Q_GPS::get_unix_timestamp() const {
    if (!current_data.datetime.valid) return 0;
    return GPS_Utils::datetime_to_unix(current_data.datetime);
}

bool ZOE_M8Q_GPS::is_time_valid() const {
    return current_data.datetime.valid;
}

bool ZOE_M8Q_GPS::is_fix_valid() const {
    return has_fix() && 
           current_data.position.satellites >= 4 &&
           current_data.position.hdop < 5.0f;
}

uint8_t ZOE_M8Q_GPS::get_fix_quality() const {
    if (!has_fix()) return 0;
    
    if (current_data.position.hdop < 1.0f && current_data.position.satellites >= 8) return 4; // Excellent
    if (current_data.position.hdop < 2.0f && current_data.position.satellites >= 6) return 3; // Good
    if (current_data.position.hdop < 5.0f && current_data.position.satellites >= 4) return 2; // Fair
    return 1; // Poor
}

float ZOE_M8Q_GPS::get_parse_success_rate() const {
    if (packets_received == 0) return 0.0f;
    return ((float)packets_parsed / packets_received) * 100.0f;
}

void ZOE_M8Q_GPS::reset_statistics() {
    packets_received = 0;
    packets_parsed = 0;
    checksum_errors = 0;
    timeout_errors = 0;
}

void ZOE_M8Q_GPS::print_position_info() {
    if (!has_fix()) {
        Serial.println("No GPS fix available");
        return;
    }
    
    Serial.println("=== GPS Position Info ===");
    Serial.print("Latitude: "); Serial.println(current_data.position.latitude, 8);
    Serial.print("Longitude: "); Serial.println(current_data.position.longitude, 8);
    Serial.print("Altitude: "); Serial.print(current_data.position.altitude, 2); Serial.println(" m");
    Serial.print("Fix Type: "); Serial.println(GPS_Utils::fix_type_to_string(current_data.position.fix_type));
    Serial.print("Satellites: "); Serial.println(current_data.position.satellites);
    Serial.print("HDOP: "); Serial.println(current_data.position.hdop, 2);
    Serial.print("Speed: "); Serial.print(current_data.velocity.speed, 2); Serial.println(" m/s");
    Serial.print("Course: "); Serial.print(current_data.velocity.course, 1); Serial.println("°");
    
    if (current_data.datetime.valid) {
        Serial.print("Time: ");
        Serial.print(current_data.datetime.year); Serial.print("-");
        Serial.print(current_data.datetime.month); Serial.print("-");
        Serial.print(current_data.datetime.day); Serial.print(" ");
        Serial.print(current_data.datetime.hour); Serial.print(":");
        Serial.print(current_data.datetime.minute); Serial.print(":");
        Serial.println(current_data.datetime.second);
    }
}

void ZOE_M8Q_GPS::print_satellite_info() {
    Serial.println("=== GPS Satellite Info ===");
    Serial.print("Satellites in view: "); Serial.println(current_data.sat_info.satellites_in_view);
    Serial.print("Satellites used: "); Serial.println(current_data.sat_info.satellites_used);
    
    for (int i = 0; i < current_data.sat_info.satellites_in_view && i < GPS_MAX_SATELLITES; i++) {
        const GPS_Satellite& sat = current_data.sat_info.satellites[i];
        Serial.print("PRN "); Serial.print(sat.prn);
        Serial.print(": El="); Serial.print(sat.elevation);
        Serial.print("° Az="); Serial.print(sat.azimuth);
        Serial.print("° SNR="); Serial.print(sat.snr);
        Serial.println(sat.used ? " (Used)" : "");
    }
}

// Private functions
void ZOE_M8Q_GPS::parse_ubx_packet() {
    switch (ubx_packet.msg_class) {
        case UBX_CLASS_NAV:
            switch (ubx_packet.msg_id) {
                case UBX_NAV_PVT:
                    parse_nav_pvt(ubx_packet.payload, ubx_packet.length);
                    break;
                case UBX_NAV_POSLLH:
                    parse_nav_posllh(ubx_packet.payload, ubx_packet.length);
                    break;
                case UBX_NAV_VELNED:
                    parse_nav_velned(ubx_packet.payload, ubx_packet.length);
                    break;
            }
            break;
    }
}

bool ZOE_M8Q_GPS::parse_nav_pvt(const uint8_t* payload, uint16_t length) {
    if (length < 84) return false; // Minimum PVT payload size
    
    // Extract time
    current_data.datetime.year = payload[4] | (payload[5] << 8);
    current_data.datetime.month = payload[6];
    current_data.datetime.day = payload[7];
    current_data.datetime.hour = payload[8];
    current_data.datetime.minute = payload[9];
    current_data.datetime.second = payload[10];
    current_data.datetime.valid = (payload[11] & 0x01) != 0;
    
    // Extract fix info
    current_data.position.fix_type = payload[20];
    current_data.position.satellites = payload[23];
    
    // Extract position (scaled integers)
    int32_t lon_raw = payload[24] | (payload[25] << 8) | (payload[26] << 16) | (payload[27] << 24);
    int32_t lat_raw = payload[28] | (payload[29] << 8) | (payload[30] << 16) | (payload[31] << 24);
    int32_t alt_raw = payload[36] | (payload[37] << 8) | (payload[38] << 16) | (payload[39] << 24);
    
    current_data.position.longitude = lon_raw * 1e-7;
    current_data.position.latitude = lat_raw * 1e-7;
    current_data.position.altitude = alt_raw * 1e-3;
    
    // Extract velocity
    int32_t velN = payload[48] | (payload[49] << 8) | (payload[50] << 16) | (payload[51] << 24);
    int32_t velE = payload[52] | (payload[53] << 8) | (payload[54] << 16) | (payload[55] << 24);
    int32_t velD = payload[56] | (payload[57] << 8) | (payload[58] << 16) | (payload[59] << 24);
    
    current_data.velocity.speed = sqrt(velN * velN + velE * velE) * 1e-3;
    current_data.velocity.course = atan2(velE, velN) * 180.0 / PI;
    if (current_data.velocity.course < 0) current_data.velocity.course += 360.0;
    current_data.velocity.climb_rate = -velD * 1e-3;
    
    // Extract accuracy estimates
    uint32_t hAcc = payload[40] | (payload[41] << 8) | (payload[42] << 16) | (payload[43] << 24);
    uint32_t vAcc = payload[44] | (payload[45] << 8) | (payload[46] << 16) | (payload[47] << 24);
    
    // Calculate HDOP approximation from accuracy
    current_data.position.hdop = hAcc * 1e-3 / 2.5f; // Rough conversion
    
    // Validate and mark data as updated
    current_data.position.valid = (current_data.position.fix_type >= GPS_FIX_2D);
    current_data.velocity.valid = current_data.position.valid;
    current_data.valid = true;
    
    mark_data_updated();
    
    return true;
}

bool ZOE_M8Q_GPS::parse_nmea_sentence() {
    char* sentence = (char*)nmea_buffer;
    
    // Determine sentence type
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        return parse_gga(sentence);
    } else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        return parse_rmc(sentence);
    }
    
    return false;
}

bool ZOE_M8Q_GPS::parse_gga(const char* sentence) {
    // Simple GGA parser (simplified for brevity)
    // In a full implementation, this would parse all GGA fields
    
    char* fields[15];
    int field_count = 0;
    char* sentence_copy = strdup(sentence);
    
    char* token = strtok(sentence_copy, ",");
    while (token && field_count < 15) {
        fields[field_count++] = token;
        token = strtok(nullptr, ",");
    }
    
    if (field_count >= 10) {
        // Parse latitude
        if (strlen(fields[2]) > 0 && strlen(fields[3]) > 0) {
            current_data.position.latitude = nmea_to_decimal_degrees(fields[2], fields[3][0]);
        }
        
        // Parse longitude
        if (strlen(fields[4]) > 0 && strlen(fields[5]) > 0) {
            current_data.position.longitude = nmea_to_decimal_degrees(fields[4], fields[5][0]);
        }
        
        // Parse fix quality and satellites
        if (strlen(fields[6]) > 0) {
            int quality = atoi(fields[6]);
            current_data.position.fix_type = (quality > 0) ? GPS_FIX_3D : GPS_FIX_NONE;
        }
        
        if (strlen(fields[7]) > 0) {
            current_data.position.satellites = atoi(fields[7]);
        }
        
        // Parse HDOP
        if (strlen(fields[8]) > 0) {
            current_data.position.hdop = atof(fields[8]);
        }
        
        // Parse altitude
        if (strlen(fields[9]) > 0) {
            current_data.position.altitude = atof(fields[9]);
        }
        
        current_data.position.valid = (current_data.position.fix_type > GPS_FIX_NONE);
        mark_data_updated();
    }
    
    free(sentence_copy);
    return true;
}

int8_t ZOE_M8Q_GPS::send_ubx_packet(uint8_t msg_class, uint8_t msg_id, const uint8_t* payload, uint16_t length) {
    Stream* stream = use_hardware_serial ? (Stream*)gps_serial : (Stream*)soft_serial;
    
    // Send UBX header
    stream->write(UBX_SYNC_CHAR_1);
    stream->write(UBX_SYNC_CHAR_2);
    stream->write(msg_class);
    stream->write(msg_id);
    stream->write(length & 0xFF);
    stream->write((length >> 8) & 0xFF);
    
    // Send payload
    for (uint16_t i = 0; i < length; i++) {
        stream->write(payload[i]);
    }
    
    // Calculate and send checksum
    uint8_t ck_a = 0, ck_b = 0;
    ck_a += msg_class; ck_b += ck_a;
    ck_a += msg_id; ck_b += ck_a;
    ck_a += (length & 0xFF); ck_b += ck_a;
    ck_a += ((length >> 8) & 0xFF); ck_b += ck_a;
    
    for (uint16_t i = 0; i < length; i++) {
        ck_a += payload[i];
        ck_b += ck_a;
    }
    
    stream->write(ck_a);
    stream->write(ck_b);
    
    return GPS_SUCCESS;
}

bool ZOE_M8Q_GPS::wait_for_ack(uint8_t msg_class, uint8_t msg_id, uint32_t timeout_ms) {
    uint32_t start_time = millis();
    
    while (millis() - start_time < timeout_ms) {
        update();
        
        // Check if we received an ACK for this message
        // This is simplified - in a full implementation, you'd track ACK/NAK messages
        delay(10);
    }
    
    return true; // Simplified - assume success
}

void ZOE_M8Q_GPS::calculate_ubx_checksum(const UBX_Packet& packet, uint8_t& ck_a, uint8_t& ck_b) {
    ck_a = 0;
    ck_b = 0;
    
    ck_a += packet.msg_class; ck_b += ck_a;
    ck_a += packet.msg_id; ck_b += ck_a;
    ck_a += (packet.length & 0xFF); ck_b += ck_a;
    ck_a += ((packet.length >> 8) & 0xFF); ck_b += ck_a;
    
    for (uint16_t i = 0; i < packet.length; i++) {
        ck_a += packet.payload[i];
        ck_b += ck_a;
    }
}

bool ZOE_M8Q_GPS::validate_nmea_checksum(const char* sentence) {
    // Find checksum delimiter
    const char* checksum_pos = strchr(sentence, '*');
    if (!checksum_pos) return false;
    
    // Calculate checksum
    uint8_t calculated_checksum = 0;
    for (const char* p = sentence + 1; p < checksum_pos; p++) {
        calculated_checksum ^= *p;
    }
    
    // Parse received checksum
    uint8_t received_checksum = (hex_char_to_byte(checksum_pos[1]) << 4) | 
                               hex_char_to_byte(checksum_pos[2]);
    
    return calculated_checksum == received_checksum;
}

uint8_t ZOE_M8Q_GPS::hex_char_to_byte(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

double ZOE_M8Q_GPS::nmea_to_decimal_degrees(const char* coord, char hemisphere) {
    if (strlen(coord) < 4) return 0.0;
    
    // Parse degrees and minutes
    char degrees_str[4] = {0};
    strncpy(degrees_str, coord, (hemisphere == 'N' || hemisphere == 'S') ? 2 : 3);
    
    int degrees = atoi(degrees_str);
    double minutes = atof(coord + strlen(degrees_str));
    
    double decimal = degrees + minutes / 60.0;
    
    if (hemisphere == 'S' || hemisphere == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
}

void ZOE_M8Q_GPS::reset_parse_state() {
    parse_state = 0;
    nmea_index = 0;
    ubx_index = 0;
}

void ZOE_M8Q_GPS::update_statistics() {
    // Update various statistics and timeouts
    uint32_t current_time = millis();
    
    if (has_fix() && (current_time - last_fix_time) > 10000) {
        // Lost fix after having one
        timeout_errors++;
    }
    
    if (has_fix()) {
        last_fix_time = current_time;
    }
}

void ZOE_M8Q_GPS::mark_data_updated() {
    last_update_time = millis();
    current_data.timestamp = last_update_time;
}

// Utility functions
namespace GPS_Utils {
    double haversine_distance(double lat1, double lon1, double lat2, double lon2) {
        const double R = 6371000.0; // Earth radius in meters
        
        double dLat = (lat2 - lat1) * PI / 180.0;
        double dLon = (lon2 - lon1) * PI / 180.0;
        
        double a = sin(dLat/2) * sin(dLat/2) +
                   cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) *
                   sin(dLon/2) * sin(dLon/2);
        
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        
        return R * c;
    }
    
    double great_circle_bearing(double lat1, double lon1, double lat2, double lon2) {
        double dLon = (lon2 - lon1) * PI / 180.0;
        lat1 = lat1 * PI / 180.0;
        lat2 = lat2 * PI / 180.0;
        
        double y = sin(dLon) * cos(lat2);
        double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
        
        double bearing = atan2(y, x) * 180.0 / PI;
        return fmod(bearing + 360.0, 360.0);
    }
    
    const char* fix_type_to_string(uint8_t fix_type) {
        switch (fix_type) {
            case GPS_FIX_NONE: return "No Fix";
            case GPS_FIX_DEAD_RECKONING: return "Dead Reckoning";
            case GPS_FIX_2D: return "2D Fix";
            case GPS_FIX_3D: return "3D Fix";
            case GPS_FIX_GPS_DR: return "GPS+DR";
            case GPS_FIX_TIME_ONLY: return "Time Only";
            default: return "Unknown";
        }
    }
    
    const char* quality_assessment(float hdop, uint8_t satellites) {
        if (satellites < 4) return "Poor - Insufficient satellites";
        if (hdop > 10.0f) return "Poor - High HDOP";
        if (hdop > 5.0f) return "Fair";
        if (hdop > 2.0f) return "Good";
        return "Excellent";
    }
    
    bool is_position_reasonable(double lat, double lon, float alt) {
        return (lat >= -90.0 && lat <= 90.0 &&
                lon >= -180.0 && lon <= 180.0 &&
                alt >= -500.0 && alt <= 20000.0);
    }
    
    uint32_t datetime_to_unix(const GPS_DateTime& dt) {
        // Simplified Unix timestamp calculation
        // In a full implementation, this would handle leap years properly
        
        uint32_t days = 0;
        
        // Count days from epoch (1970)
        for (uint16_t year = 1970; year < dt.year; year++) {
            days += is_leap_year(year) ? 366 : 365;
        }
        
        // Add days from months in current year
        for (uint8_t month = 1; month < dt.month; month++) {
            days += days_in_month(month, dt.year);
        }
        
        // Add days in current month
        days += dt.day - 1;
        
        // Convert to seconds and add time
        return days * 86400UL + dt.hour * 3600UL + dt.minute * 60UL + dt.second;
    }
    
    bool is_leap_year(uint16_t year) {
        return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    }
    
    uint8_t days_in_month(uint8_t month, uint16_t year) {
        const uint8_t days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        
        if (month == 2 && is_leap_year(year)) {
            return 29;
        }
        
        return days[month - 1];
    }
}