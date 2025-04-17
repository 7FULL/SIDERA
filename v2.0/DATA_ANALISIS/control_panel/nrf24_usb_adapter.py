import serial
import struct
import time
import threading
import queue


# Command codes matching the Arduino implementation
class CommandCodes:
    CMD_PING = 1
    CMD_DEPLOY_PARACHUTE = 2
    CMD_ABORT = 3
    CMD_REBOOT = 4
    CMD_WAKE_UP = 5
    CMD_LAUNCH = 6
    CMD_START_PLATFORM = 7
    CMD_ROCKET_READY = 8


# Telemetry packet structure
class TelemetryPacket:
    def __init__(self):
        self.altitude = 0.0
        self.temperature = 0.0
        self.pressure = 0.0
        self.acceleration_x = 0.0
        self.acceleration_y = 0.0
        self.acceleration_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.latitude = 0.0
        self.longitude = 0.0
        self.parachute_deployed = False
        self.has_reached_apogee = False
        self.landed = False
        self.timestamp = 0


# NRF24L01 USB Adapter Interface
class NRF24USBAdapter:
    def __init__(self, port=None, baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.serial = None
        self.connected = False
        self.read_thread = None
        self.stop_thread = False
        self.telemetry_queue = queue.Queue()

        # Expected packet size for telemetry data
        self.telemetry_packet_size = 56  # Adjust based on your actual packet structure

        # Buffer for incomplete packets
        self.buffer = bytearray()

        # Sync pattern for packet identification
        self.sync_pattern = b'\xAA\x55\xAA\x55'  # Example sync pattern - adjust to match your implementation

    def connect(self):
        """Connect to the NRF24 USB adapter"""
        if self.port is None:
            return False

        try:
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.connected = True

            # Flush any stale data
            self.serial.reset_input_buffer()

            # Start the read thread
            self.stop_thread = False
            self.read_thread = threading.Thread(target=self._read_serial)
            self.read_thread.daemon = True
            self.read_thread.start()

            # Send a ping command to establish communication
            self.send_command(CommandCodes.CMD_PING)

            return True
        except serial.SerialException as e:
            print(f"Error connecting to NRF24 USB adapter: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Disconnect from the NRF24 USB adapter"""
        self.stop_thread = True
        if self.read_thread:
            self.read_thread.join(2.0)

        if self.serial:
            self.serial.close()

        self.connected = False
        print("Disconnected from NRF24 USB adapter")

    def _read_serial(self):
        """Background thread to read data from the serial port"""
        while not self.stop_thread:
            if not self.connected or not self.serial:
                time.sleep(0.1)
                continue

            try:
                # Read available data
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    self.buffer.extend(data)

                    # Process the buffer for complete packets
                    self._process_buffer()
            except Exception as e:
                print(f"Error reading from serial: {e}")
                time.sleep(0.1)

    def _process_buffer(self):
        """Process the receive buffer to extract complete packets"""
        # Keep processing as long as we have at least a minimum packet size in the buffer
        min_packet_size = len(self.sync_pattern) + self.telemetry_packet_size

        while len(self.buffer) >= min_packet_size:
            # Look for sync pattern
            sync_index = self.buffer.find(self.sync_pattern)

            if sync_index == -1:
                # No sync pattern found, keep the last few bytes in case they're part of a sync pattern
                self.buffer = self.buffer[-3:]
                break

            # If we found the sync pattern but it's not at the beginning, discard data before it
            if sync_index > 0:
                self.buffer = self.buffer[sync_index:]
                continue

            # Check if we have a complete packet
            if len(self.buffer) >= min_packet_size:
                # Extract the packet payload (excluding sync pattern)
                packet_data = self.buffer[len(self.sync_pattern):len(self.sync_pattern) + self.telemetry_packet_size]

                # Parse the telemetry packet
                telemetry = self._parse_telemetry(packet_data)
                if telemetry:
                    self.telemetry_queue.put(telemetry)

                # Remove the processed packet from the buffer
                self.buffer = self.buffer[len(self.sync_pattern) + self.telemetry_packet_size:]
            else:
                # Not enough data for a complete packet, wait for more
                break

    def _parse_telemetry(self, data):
        """Parse a telemetry packet from binary data"""
        try:
            # Unpack the binary data according to the expected format
            # Adjust the struct format string to match your actual data format
            # 'fffffffffdddBBBL' for:
            # 9 floats (altitude, temp, pressure, accel_x/y/z, gyro_x/y/z)
            # 3 doubles (latitude, longitude, reserved)
            # 3 booleans (parachute_deployed, has_reached_apogee, landed)
            # 1 unsigned long (timestamp)

            values = struct.unpack('fffffffffdddBBBL', data)

            telemetry = TelemetryPacket()
            telemetry.altitude = values[0]
            telemetry.temperature = values[1]
            telemetry.pressure = values[2]
            telemetry.acceleration_x = values[3]
            telemetry.acceleration_y = values[4]
            telemetry.acceleration_z = values[5]
            telemetry.gyro_x = values[6]
            telemetry.gyro_y = values[7]
            telemetry.gyro_z = values[8]
            telemetry.latitude = values[9]
            telemetry.longitude = values[10]
            # values[11] is reserved/unused
            telemetry.parachute_deployed = bool(values[12])
            telemetry.has_reached_apogee = bool(values[13])
            telemetry.landed = bool(values[14])
            telemetry.timestamp = values[15]

            return telemetry
        except Exception as e:
            print(f"Error parsing telemetry packet: {e}")
            return None

    def send_command(self, command, parameter=0):
        """Send a command to the rocket via the NRF24 adapter"""
        if not self.connected or not self.serial:
            return False

        try:
            # Command packet structure:
            # Sync pattern + Command code (1 byte) + Parameter (4 bytes)
            command_packet = self.sync_pattern + struct.pack('BL', command, parameter)

            # Write the command to the serial port
            self.serial.write(command_packet)
            self.serial.flush()

            print(f"Sent command: {command} with parameter: {parameter}")
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False

    def get_telemetry(self):
        """Get the next telemetry packet from the queue, if available"""
        if not self.telemetry_queue.empty():
            return self.telemetry_queue.get()
        return None


# Usage example
if __name__ == "__main__":
    # This is a simple test to demonstrate the adapter functionality
    import serial.tools.list_ports

    # List available serial ports
    ports = [port.device for port in serial.tools.list_ports.comports()]
    print("Available ports:")
    for i, port in enumerate(ports):
        print(f"{i}: {port}")

    if ports:
        # Select a port
        port_index = int(input("Select port by index: "))
        selected_port = ports[port_index]

        # Create adapter instance
        adapter = NRF24USBAdapter(port=selected_port)

        # Connect to the adapter
        if adapter.connect():
            print(f"Connected to {selected_port}")

            try:
                # Main loop
                print("Receiving telemetry. Press Ctrl+C to exit.")
                while True:
                    # Check for telemetry
                    telemetry = adapter.get_telemetry()
                    if telemetry:
                        print(f"Altitude: {telemetry.altitude:.2f}m, "
                              f"Temp: {telemetry.temperature:.1f}°C, "
                              f"Accel Z: {telemetry.acceleration_z:.2f}m/s²")

                    # Example command
                    if input("Send wake up command? (y/n): ").lower() == 'y':
                        adapter.send_command(CommandCodes.CMD_WAKE_UP)

                    time.sleep(0.1)

            except KeyboardInterrupt:
                print("Interrupted by user.")

            finally:
                # Disconnect when done
                adapter.disconnect()
        else:
            print(f"Failed to connect to {selected_port}")
    else:
        print("No serial ports available.")