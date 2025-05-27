import struct
import sys


def read_telemetry_file(filename):
    """Read binary telemetry file and convert to CSV"""

    # StoredTelemetry structure format (must match C++ struct)
    # '<' = little-endian
    # I = uint32_t (4 bytes)
    # B = uint8_t (1 byte)
    # i = int32_t (4 bytes)
    # h = int16_t (2 bytes)
    # H = uint16_t (2 bytes)
    telemetry_format = '<IB i h hhhhhh h H H ii H B B'
    telemetry_size = struct.calcsize(telemetry_format)

    print("Telemetry entry size:", telemetry_size, "bytes")

    # CSV header
    print("timestamp,state,altitude_m,vert_speed_ms,accel_x_g,accel_y_g,accel_z_g," +
          "gyro_x_degs,gyro_y_degs,gyro_z_degs,temp_c,pressure_hpa,battery_v," +
          "gps_lat,gps_lon,gps_alt_m,gps_sats,flags")

    with open(filename, 'rb') as f:
        entry_count = 0
        while True:
            data = f.read(telemetry_size)
            if len(data) < telemetry_size:
                break

            # Unpack the binary data
            values = struct.unpack(telemetry_format, data)

            # Extract and scale values back to original units
            timestamp = values[0]
            state = values[1]
            altitude = values[2] / 1000.0  # mm to m
            vert_speed = values[3] / 100.0  # cm/s to m/s
            accel_x = values[4] / 100.0  # 0.01g to g
            accel_y = values[5] / 100.0
            accel_z = values[6] / 100.0
            gyro_x = values[7] / 10.0  # 0.1 deg/s to deg/s
            gyro_y = values[8] / 10.0
            gyro_z = values[9] / 10.0
            temp = values[10] / 10.0  # 0.1C to C
            pressure = values[11] / 10.0  # 0.1 hPa to hPa
            battery = values[12] / 1000.0  # mV to V
            gps_lat = values[13] / 10000000.0  # 10^-7 deg to deg
            gps_lon = values[14] / 10000000.0
            gps_alt = values[15]
            gps_sats = values[16]
            flags = values[17]

            # Print CSV row
            print(f"{timestamp},{state},{altitude:.2f},{vert_speed:.2f}," +
                  f"{accel_x:.2f},{accel_y:.2f},{accel_z:.2f}," +
                  f"{gyro_x:.2f},{gyro_y:.2f},{gyro_z:.2f}," +
                  f"{temp:.1f},{pressure:.1f},{battery:.2f}," +
                  f"{gps_lat:.7f},{gps_lon:.7f},{gps_alt}," +
                  f"{gps_sats},{flags}")

            entry_count += 1

    print(f"\nTotal entries: {entry_count}", file=sys.stderr)


if __name__ == "__main__":
    read_telemetry_file("telemetry.dat")