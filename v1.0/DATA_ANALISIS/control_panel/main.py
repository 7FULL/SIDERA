import customtkinter as ctk
import tkinter as tk
from tkinter import messagebox, filedialog
import time
import threading
import queue
import os
import csv
import struct
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from datetime import datetime
from enum import Enum, auto
import json
import serial
import serial.tools.list_ports

# Import the simulator
from rocket_simulator import RocketSimulator, CommandCodes, RocketStates


# Command codes as defined in the Arduino code
class CommandCodes(Enum):
    CMD_PING = 1
    CMD_DEPLOY_PARACHUTE = 2
    CMD_ABORT = 3
    CMD_REBOOT = 4
    CMD_WAKE_UP = 5
    CMD_LAUNCH = 6
    CMD_START_PLATFORM = 7
    CMD_ROCKET_READY = 8


# Telemetry data structure
class TelemetryData:
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
        self.state = RocketStates.IDLE


# NRF24 USB Communication Class
class NRF24USB:
    def __init__(self, port=None, baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.serial = None
        self.connected = False
        self.read_thread = None
        self.stop_thread = False
        self.telemetry_queue = queue.Queue()

    def connect(self):
        if self.port is None:
            return False

        try:
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.connected = True

            # Start the read thread
            self.stop_thread = False
            self.read_thread = threading.Thread(target=self._read_serial)
            self.read_thread.daemon = True
            self.read_thread.start()

            return True
        except serial.SerialException as e:
            print(f"Error connecting to NRF24 USB: {e}")
            self.connected = False
            return False

    def disconnect(self):
        self.stop_thread = True
        if self.read_thread:
            self.read_thread.join(2.0)

        if self.serial:
            self.serial.close()

        self.connected = False

    def _read_serial(self):
        while not self.stop_thread:
            if not self.connected or not self.serial:
                time.sleep(0.1)
                continue

            try:
                # Read telemetry data from serial
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)

                    # Process the received data into TelemetryData
                    telemetry = self._parse_telemetry(data)
                    if telemetry:
                        self.telemetry_queue.put(telemetry)
            except Exception as e:
                print(f"Error reading from serial: {e}")
                time.sleep(0.1)

    def _parse_telemetry(self, data):
        # Placeholder for telemetry parsing
        # In a real implementation, you would decode the binary data according to
        # the TelemetryData struct in the Arduino code
        try:
            # Example structure (adapt to your actual data format)
            if len(data) < 56:  # Minimum expected size
                return None

            telemetry = TelemetryData()
            # For testing purposes, just create some random data
            telemetry.altitude = 100.0
            telemetry.temperature = 25.0
            telemetry.pressure = 1013.25
            telemetry.acceleration_x = 0.0
            telemetry.acceleration_y = 0.0
            telemetry.acceleration_z = 9.8
            telemetry.gyro_x = 0.0
            telemetry.gyro_y = 0.0
            telemetry.gyro_z = 0.0
            telemetry.latitude = 40.7128
            telemetry.longitude = -74.0060
            telemetry.parachute_deployed = False
            telemetry.has_reached_apogee = False
            telemetry.landed = False
            telemetry.timestamp = int(time.time() * 1000)

            return telemetry
        except Exception as e:
            print(f"Error parsing telemetry data: {e}")
            return None

    def  send_command(self, command, parameter=0):
        if not self.connected or not self.serial:
            return False

        try:
            # Pack the command data
            command_data = struct.pack('BL', command.value, parameter)
            self.serial.write(command_data)
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False

    def get_telemetry(self):
        if not self.telemetry_queue.empty():
            return self.telemetry_queue.get()
        return None


# Flight Data Logger
class FlightDataLogger:
    def __init__(self, directory="flight_logs"):
        self.directory = directory
        self.current_log_file = None
        self.csv_writer = None
        self.ensure_directory()

    def ensure_directory(self):
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

    def start_new_log(self):
        self.ensure_directory()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"flight_log_{timestamp}.csv"
        self.current_log_file = open(os.path.join(self.directory, filename), 'w', newline='')

        # Create CSV writer and write header
        self.csv_writer = csv.writer(self.current_log_file)
        self.csv_writer.writerow([
            "Timestamp", "State", "Altitude", "Temperature", "Pressure",
            "AccelX", "AccelY", "AccelZ", "GyroX", "GyroY", "GyroZ",
            "Latitude", "Longitude", "ParachuteDeployed", "HasReachedApogee", "Landed"
        ])

        return filename

    def log_telemetry(self, telemetry, state):
        if self.csv_writer is None:
            return

        # Convert timestamp to readable format
        time_str = datetime.fromtimestamp(telemetry.timestamp / 1000.0).strftime("%H:%M:%S.%f")[:-3]

        self.csv_writer.writerow([
            time_str,
            state.name if state else "UNKNOWN",
            telemetry.altitude,
            telemetry.temperature,
            telemetry.pressure,
            telemetry.acceleration_x,
            telemetry.acceleration_y,
            telemetry.acceleration_z,
            telemetry.gyro_x,
            telemetry.gyro_y,
            telemetry.gyro_z,
            telemetry.latitude,
            telemetry.longitude,
            telemetry.parachute_deployed,
            telemetry.has_reached_apogee,
            telemetry.landed
        ])

        self.current_log_file.flush()

    def close_log(self):
        if self.current_log_file:
            self.current_log_file.close()
            self.current_log_file = None
            self.csv_writer = None


# Rocket Control Panel Application
class RocketControlPanel(ctk.CTk):
    def __init__(self):
        super().__init__()

        # Simulation mode
        self.simulation_mode = False

        # Set appearance mode and default color theme
        ctk.set_appearance_mode("Dark")
        ctk.set_default_color_theme("blue")

        # Configure window
        self.title("Rocket Control Panel")
        self.geometry("1280x800")
        self.minsize(1024, 768)

        # Initialize NRF24 USB and data logger
        self.nrf24 = NRF24USB()
        self.data_logger = FlightDataLogger()

        # Initialize simulator for testing
        self.simulator = RocketSimulator()

        # Data storage for graphs
        self.altitude_data = []
        self.time_data = []
        self.acceleration_data = {'x': [], 'y': [], 'z': []}
        self.gyro_data = {'x': [], 'y': [], 'z': []}
        self.temperature_data = []
        self.pressure_data = []

        # Current rocket state
        self.current_state = RocketStates.IDLE

        # Start time for relative timestamps
        self.start_time = time.time()

        # Create UI elements
        self.create_widgets()

        # Start update thread
        self.running = True
        self.update_thread = threading.Thread(target=self.update_telemetry)
        self.update_thread.daemon = True
        self.update_thread.start()

        # Bind close event
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def create_widgets(self):
        # Create main container with three columns
        self.grid_columnconfigure(0, weight=2)  # Graphs column
        self.grid_columnconfigure(1, weight=3)  # Info & telemetry column
        self.grid_columnconfigure(2, weight=2)  # Controls column
        self.grid_rowconfigure(0, weight=1)

        # Left column (graphs)
        self.graphs_column = ctk.CTkFrame(self)
        self.graphs_column.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.create_graphs_column()

        # Middle column (info & telemetry)
        self.info_column = ctk.CTkFrame(self)
        self.info_column.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")
        self.create_info_column()

        # Right column (controls)
        self.controls_column = ctk.CTkFrame(self)
        self.controls_column.grid(row=0, column=2, padx=5, pady=5, sticky="nsew")
        self.create_controls_column()

    def create_graphs_column(self):
        # Configure grid for four graphs
        self.graphs_column.grid_columnconfigure(0, weight=1)
        self.graphs_column.grid_rowconfigure(0, weight=1)
        self.graphs_column.grid_rowconfigure(1, weight=1)
        self.graphs_column.grid_rowconfigure(2, weight=1)
        self.graphs_column.grid_rowconfigure(3, weight=1)

        # Create four graph frames
        self.altitude_graph_frame = ctk.CTkFrame(self.graphs_column)
        self.altitude_graph_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.create_altitude_graph()

        self.accel_graph_frame = ctk.CTkFrame(self.graphs_column)
        self.accel_graph_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
        self.create_acceleration_graph()

        self.gyro_graph_frame = ctk.CTkFrame(self.graphs_column)
        self.gyro_graph_frame.grid(row=2, column=0, padx=5, pady=5, sticky="nsew")
        self.create_gyroscope_graph()

        self.temp_press_frame = ctk.CTkFrame(self.graphs_column)
        self.temp_press_frame.grid(row=3, column=0, padx=5, pady=5, sticky="nsew")
        self.create_temp_press_graph()

    def create_altitude_graph(self):
        self.altitude_fig = Figure(figsize=(5, 3), dpi=100)
        self.altitude_ax = self.altitude_fig.add_subplot(111)
        self.altitude_ax.set_title('Altitude')
        self.altitude_ax.set_xlabel('Time (s)')
        self.altitude_ax.set_ylabel('Altitude (m)')
        self.altitude_line, = self.altitude_ax.plot([], [], 'r-')

        self.altitude_canvas = FigureCanvasTkAgg(self.altitude_fig, master=self.altitude_graph_frame)
        self.altitude_canvas.draw()
        self.altitude_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def create_acceleration_graph(self):
        self.accel_fig = Figure(figsize=(5, 3), dpi=100)
        self.accel_ax = self.accel_fig.add_subplot(111)
        self.accel_ax.set_title('Acceleration')
        self.accel_ax.set_xlabel('Time (s)')
        self.accel_ax.set_ylabel('Acceleration (m/s²)')
        self.accel_x_line, = self.accel_ax.plot([], [], 'r-', label='X')
        self.accel_y_line, = self.accel_ax.plot([], [], 'g-', label='Y')
        self.accel_z_line, = self.accel_ax.plot([], [], 'b-', label='Z')
        self.accel_ax.legend()

        self.accel_canvas = FigureCanvasTkAgg(self.accel_fig, master=self.accel_graph_frame)
        self.accel_canvas.draw()
        self.accel_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def create_gyroscope_graph(self):
        self.gyro_fig = Figure(figsize=(5, 3), dpi=100)
        self.gyro_ax = self.gyro_fig.add_subplot(111)
        self.gyro_ax.set_title('Gyroscope')
        self.gyro_ax.set_xlabel('Time (s)')
        self.gyro_ax.set_ylabel('Rotation (deg/s)')
        self.gyro_x_line, = self.gyro_ax.plot([], [], 'r-', label='X')
        self.gyro_y_line, = self.gyro_ax.plot([], [], 'g-', label='Y')
        self.gyro_z_line, = self.gyro_ax.plot([], [], 'b-', label='Z')
        self.gyro_ax.legend()

        self.gyro_canvas = FigureCanvasTkAgg(self.gyro_fig, master=self.gyro_graph_frame)
        self.gyro_canvas.draw()
        self.gyro_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def create_temp_press_graph(self):
        self.temp_press_fig = Figure(figsize=(5, 3), dpi=100)
        self.temp_ax = self.temp_press_fig.add_subplot(111)
        self.press_ax = self.temp_ax.twinx()

        self.temp_ax.set_title('Temperature & Pressure')
        self.temp_ax.set_xlabel('Time (s)')
        self.temp_ax.set_ylabel('Temperature (°C)', color='r')
        self.press_ax.set_ylabel('Pressure (hPa)', color='b')

        self.temp_line, = self.temp_ax.plot([], [], 'r-', label='Temperature')
        self.press_line, = self.press_ax.plot([], [], 'b-', label='Pressure')

        self.temp_ax.tick_params(axis='y', labelcolor='r')
        self.press_ax.tick_params(axis='y', labelcolor='b')

        self.temp_press_canvas = FigureCanvasTkAgg(self.temp_press_fig, master=self.temp_press_frame)
        self.temp_press_canvas.draw()
        self.temp_press_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def create_info_column(self):
        # Configure grid for info and telemetry panels
        self.info_column.grid_columnconfigure(0, weight=1)
        self.info_column.grid_rowconfigure(0, weight=1)  # Status panel
        self.info_column.grid_rowconfigure(1, weight=3)  # Telemetry panel

        # Status panel
        self.status_frame = ctk.CTkFrame(self.info_column)
        self.status_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.create_status_panel()

        # Telemetry panel
        self.telemetry_frame = ctk.CTkFrame(self.info_column)
        self.telemetry_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
        self.create_telemetry_panel()

    def create_status_panel(self):
        # Title
        status_title = ctk.CTkLabel(self.status_frame, text="ROCKET STATUS", font=ctk.CTkFont(size=20, weight="bold"))
        status_title.pack(pady=(10, 15))

        # Status container
        status_container = ctk.CTkFrame(self.status_frame)
        status_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Configure grid for status values
        status_container.grid_columnconfigure(0, weight=1)
        status_container.grid_columnconfigure(1, weight=1)

        # Status labels - left column
        state_label = ctk.CTkLabel(status_container, text="State:", font=ctk.CTkFont(size=16, weight="bold"),
                                   anchor="w")
        state_label.grid(row=0, column=0, sticky="w", padx=10, pady=5)

        altitude_label = ctk.CTkLabel(status_container, text="Altitude:", font=ctk.CTkFont(size=16, weight="bold"),
                                      anchor="w")
        altitude_label.grid(row=1, column=0, sticky="w", padx=10, pady=5)

        max_altitude_label = ctk.CTkLabel(status_container, text="Max Altitude:",
                                          font=ctk.CTkFont(size=16, weight="bold"), anchor="w")
        max_altitude_label.grid(row=2, column=0, sticky="w", padx=10, pady=5)

        # Status labels - right column
        temp_label = ctk.CTkLabel(status_container, text="Temperature:", font=ctk.CTkFont(size=16, weight="bold"),
                                  anchor="w")
        temp_label.grid(row=0, column=1, sticky="w", padx=10, pady=5)

        pressure_label = ctk.CTkLabel(status_container, text="Pressure:", font=ctk.CTkFont(size=16, weight="bold"),
                                      anchor="w")
        pressure_label.grid(row=1, column=1, sticky="w", padx=10, pady=5)

        parachute_label = ctk.CTkLabel(status_container, text="Parachute:", font=ctk.CTkFont(size=16, weight="bold"),
                                       anchor="w")
        parachute_label.grid(row=2, column=1, sticky="w", padx=10, pady=5)

        # Status values - left column
        self.state_value = ctk.CTkLabel(status_container, text="IDLE", font=ctk.CTkFont(size=16))
        self.state_value.grid(row=0, column=0, sticky="e", padx=10, pady=5)

        self.altitude_value = ctk.CTkLabel(status_container, text="0.0 m", font=ctk.CTkFont(size=16))
        self.altitude_value.grid(row=1, column=0, sticky="e", padx=10, pady=5)

        self.max_altitude_value = ctk.CTkLabel(status_container, text="0.0 m", font=ctk.CTkFont(size=16))
        self.max_altitude_value.grid(row=2, column=0, sticky="e", padx=10, pady=5)

        # Status values - right column
        self.temp_value = ctk.CTkLabel(status_container, text="0.0 °C", font=ctk.CTkFont(size=16))
        self.temp_value.grid(row=0, column=1, sticky="e", padx=10, pady=5)

        self.pressure_value = ctk.CTkLabel(status_container, text="0.0 hPa", font=ctk.CTkFont(size=16))
        self.pressure_value.grid(row=1, column=1, sticky="e", padx=10, pady=5)

        self.parachute_value = ctk.CTkLabel(status_container, text="SECURED", font=ctk.CTkFont(size=16))
        self.parachute_value.grid(row=2, column=1, sticky="e", padx=10, pady=5)

    def create_telemetry_panel(self):
        # Title
        telemetry_title = ctk.CTkLabel(self.telemetry_frame, text="TELEMETRY DATA",
                                       font=ctk.CTkFont(size=20, weight="bold"))
        telemetry_title.pack(pady=(10, 15))

        # Create scrollable frame for telemetry data
        telemetry_scrollable_frame = ctk.CTkScrollableFrame(self.telemetry_frame)
        telemetry_scrollable_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Detailed telemetry values
        self.telemetry_values = {}

        telemetry_fields = [
            ("Acceleration X", "0.0 m/s²"),
            ("Acceleration Y", "0.0 m/s²"),
            ("Acceleration Z", "0.0 m/s²"),
            ("Gyroscope X", "0.0 deg/s"),
            ("Gyroscope Y", "0.0 deg/s"),
            ("Gyroscope Z", "0.0 deg/s"),
            ("Latitude", "0.000000°"),
            ("Longitude", "0.000000°"),
            ("Flight Time", "0s"),
            ("Last Update", "N/A"),
            ("Vertical Speed", "0.0 m/s"),
            ("Total Distance", "0.0 m"),
            ("Battery Voltage", "0.0 V"),
            ("Signal Strength", "0%"),
            ("Data Rate", "0 bytes/s")
        ]

        # Create telemetry value containers with a more organized layout
        for field, value in telemetry_fields:
            container = ctk.CTkFrame(telemetry_scrollable_frame)
            container.pack(fill=tk.X, pady=3)

            label = ctk.CTkLabel(container, text=f"{field}:", anchor="w", width=120)
            label.pack(side=tk.LEFT, padx=10, pady=2)

            value_label = ctk.CTkLabel(container, text=value, anchor="e")
            value_label.pack(side=tk.RIGHT, padx=10, pady=2)

            self.telemetry_values[field] = value_label

    def create_controls_column(self):
        # Configure grid for control panels
        self.controls_column.grid_columnconfigure(0, weight=1)
        self.controls_column.grid_rowconfigure(0, weight=0)  # Connection
        self.controls_column.grid_rowconfigure(1, weight=0)  # Commands
        self.controls_column.grid_rowconfigure(2, weight=0)  # Emergency controls
        self.controls_column.grid_rowconfigure(3, weight=0)  # Flight logs
        self.controls_column.grid_rowconfigure(4, weight=0)  # System status
        self.controls_column.grid_rowconfigure(5, weight=1)  # Extra space

        # Connection panel
        self.connection_frame = ctk.CTkFrame(self.controls_column)
        self.connection_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        self.create_connection_panel()

        # Commands panel
        self.command_frame = ctk.CTkFrame(self.controls_column)
        self.command_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        self.create_command_panel()

        # Emergency controls panel
        self.emergency_frame = ctk.CTkFrame(self.controls_column)
        self.emergency_frame.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        self.create_emergency_panel()

        # Flight logs panel
        self.log_frame = ctk.CTkFrame(self.controls_column)
        self.log_frame.grid(row=3, column=0, padx=5, pady=5, sticky="ew")
        self.create_log_panel()

        # System status panel
        self.system_frame = ctk.CTkFrame(self.controls_column)
        self.system_frame.grid(row=4, column=0, padx=5, pady=5, sticky="ew")
        self.create_system_panel()

    def create_connection_panel(self):
        # Title
        title_label = ctk.CTkLabel(self.connection_frame, text="CONNECTION", font=ctk.CTkFont(size=16, weight="bold"))
        title_label.pack(pady=5)

        # Simulation mode switch
        sim_frame = ctk.CTkFrame(self.connection_frame)
        sim_frame.pack(fill=tk.X, padx=10, pady=5)

        sim_label = ctk.CTkLabel(sim_frame, text="Simulation Mode:")
        sim_label.pack(side=tk.LEFT, padx=5)

        self.sim_switch = ctk.CTkSwitch(sim_frame, text="", command=self.toggle_simulation_mode)
        self.sim_switch.pack(side=tk.LEFT, padx=5)
        if self.simulation_mode:
            self.sim_switch.select()

        # Connection controls
        connection_controls = ctk.CTkFrame(self.connection_frame)
        connection_controls.pack(fill=tk.X, padx=10, pady=5)

        # Port selection
        port_label = ctk.CTkLabel(connection_controls, text="Port:")
        port_label.pack(side=tk.LEFT, padx=5)

        self.port_combobox = ctk.CTkComboBox(connection_controls, values=[])
        self.port_combobox.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)

        # Refresh ports button
        refresh_button = ctk.CTkButton(connection_controls, text="↻", width=30, command=self.refresh_ports)
        refresh_button.pack(side=tk.RIGHT, padx=5)

        # Connect/Disconnect button
        self.connect_button = ctk.CTkButton(self.connection_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(fill=tk.X, padx=10, pady=5)

        # Connection status
        self.connection_status = ctk.CTkLabel(self.connection_frame, text="Status: Disconnected", fg_color="#550000",
                                              corner_radius=8)
        self.connection_status.pack(fill=tk.X, padx=10, pady=5)

        # Populate the port dropdown
        self.refresh_ports()

    def create_command_panel(self):
        # Title
        title_label = ctk.CTkLabel(self.command_frame, text="COMMANDS", font=ctk.CTkFont(size=16, weight="bold"))
        title_label.pack(pady=5)

        # Wake Up button
        self.wake_up_button = ctk.CTkButton(
            self.command_frame,
            text="Wake Up Rocket",
            command=lambda: self.send_command(CommandCodes.CMD_WAKE_UP),
            state="disabled"
        )
        self.wake_up_button.pack(fill=tk.X, padx=10, pady=5)

        # Launch button
        self.launch_button = ctk.CTkButton(
            self.command_frame,
            text="LAUNCH",
            fg_color="#007700",
            hover_color="#005500",
            command=self.launch_rocket,
            state="disabled"
        )
        self.launch_button.pack(fill=tk.X, padx=10, pady=5)

        # Reboot button
        self.reboot_button = ctk.CTkButton(
            self.command_frame,
            text="Reboot Rocket",
            command=lambda: self.send_command(CommandCodes.CMD_REBOOT),
            state="disabled"
        )
        self.reboot_button.pack(fill=tk.X, padx=10, pady=5)

    def create_emergency_panel(self):
        # Title
        emergency_label = ctk.CTkLabel(self.emergency_frame, text="EMERGENCY CONTROLS",
                                       font=ctk.CTkFont(size=16, weight="bold"))
        emergency_label.pack(pady=5)

        # Deploy Parachute button
        self.deploy_parachute_button = ctk.CTkButton(
            self.emergency_frame,
            text="Deploy Parachute",
            fg_color="#FF9900",
            hover_color="#CC7700",
            command=lambda: self.send_command(CommandCodes.CMD_DEPLOY_PARACHUTE),
            state="disabled"
        )
        self.deploy_parachute_button.pack(fill=tk.X, padx=10, pady=5)

        # Abort button
        self.abort_button = ctk.CTkButton(
            self.emergency_frame,
            text="ABORT MISSION",
            fg_color="#CC0000",
            hover_color="#990000",
            command=self.abort_mission,
            state="disabled"
        )
        self.abort_button.pack(fill=tk.X, padx=10, pady=5)

    def create_log_panel(self):
        # Title
        title_label = ctk.CTkLabel(self.log_frame, text="FLIGHT LOGS", font=ctk.CTkFont(size=16, weight="bold"))
        title_label.pack(pady=5)

        # Logging controls
        log_controls = ctk.CTkFrame(self.log_frame)
        log_controls.pack(fill=tk.X, padx=10, pady=5)

        # Start/Stop logging button
        self.logging_button = ctk.CTkButton(
            log_controls,
            text="Start Logging",
            command=self.toggle_logging,
            state="disabled"
        )
        self.logging_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        # Export logs button
        self.export_button = ctk.CTkButton(
            log_controls,
            text="Export Data",
            command=self.export_flight_data,
            state="disabled"
        )
        self.export_button.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=5)

        # Log status
        self.log_status = ctk.CTkLabel(self.log_frame, text="Logging: Inactive")
        self.log_status.pack(fill=tk.X, padx=10, pady=5)

    def create_system_panel(self):
        # Title
        title_label = ctk.CTkLabel(self.system_frame, text="SYSTEM STATUS", font=ctk.CTkFont(size=16, weight="bold"))
        title_label.pack(pady=5)

        # Status container
        system_container = ctk.CTkFrame(self.system_frame)
        system_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        system_container.grid_columnconfigure(0, weight=1)
        system_container.grid_columnconfigure(1, weight=1)

        # System status indicators
        self.system_indicators = {}

        indicators = [
            ("RF Signal", "0%", "#777777"),
            ("Battery", "0%", "#777777"),
            ("Data Link", "Inactive", "#777777"),
            ("GPS Lock", "No Signal", "#777777")
        ]

        for i, (name, value, color) in enumerate(indicators):
            label = ctk.CTkLabel(system_container, text=f"{name}:", anchor="w")
            label.grid(row=i, column=0, sticky="w", padx=10, pady=3)

            status = ctk.CTkLabel(system_container, text=value, fg_color=color, corner_radius=5)
            status.grid(row=i, column=1, sticky="e", padx=10, pady=3)

            self.system_indicators[name] = status

    def refresh_ports(self):
        """Refresh the available serial ports"""
        try:
            ports = [port.device for port in serial.tools.list_ports.comports()]

            if not ports:
                ports = ["No ports available"]

            self.port_combobox.configure(values=ports)
            if ports[0] != "No ports available":
                self.port_combobox.set(ports[0])
        except Exception as e:
            print(f"Error refreshing ports: {e}")
            self.port_combobox.configure(values=["Error listing ports"])

    def toggle_simulation_mode(self):
        """Toggle between simulation and real hardware mode"""
        self.simulation_mode = self.sim_switch.get()

        # If connected, disconnect first
        if (self.nrf24.connected and not self.simulation_mode) or \
                (self.simulator.running and self.simulation_mode):
            self.toggle_connection()

        # Update UI elements
        if self.simulation_mode:
            self.connection_status.configure(text="Status: Simulation Mode", fg_color="#555500")
            self.port_combobox.configure(state="disabled")
        else:
            self.connection_status.configure(text="Status: Disconnected", fg_color="#550000")
            self.port_combobox.configure(state="normal")

    def toggle_connection(self):
        """Connect to or disconnect from the NRF24 USB adapter or simulator"""
        if self.simulation_mode:
            # Simulation mode
            if not self.simulator.running:
                # Connect to simulator
                if self.simulator.connect():
                    self.connection_status.configure(text="Status: Connected (Simulation)", fg_color="#005500")
                    self.connect_button.configure(text="Disconnect")

                    # Enable rocket control buttons
                    self.wake_up_button.configure(state="normal")
                    self.reboot_button.configure(state="normal")
                    self.logging_button.configure(state="normal")
                    self.abort_button.configure(state="normal")

                    # Update system indicators for simulation
                    self.update_system_indicators(simulation=True)

                    # Start a new log file
                    self.start_logging()
                else:
                    messagebox.showerror("Simulation Error", "Failed to start simulation.")
            else:
                # Disconnect from simulator
                self.simulator.disconnect()
                self.connection_status.configure(text="Status: Simulation Mode", fg_color="#555500")
                self.connect_button.configure(text="Connect")

                # Disable rocket control buttons
                self.wake_up_button.configure(state="disabled")
                self.launch_button.configure(state="disabled")
                self.deploy_parachute_button.configure(state="disabled")
                self.abort_button.configure(state="disabled")
                self.reboot_button.configure(state="disabled")

                # Reset system indicators
                self.reset_system_indicators()

                # Stop logging
                self.stop_logging()
        else:
            # Real hardware mode
            if not self.nrf24.connected:
                port = self.port_combobox.get()

                if port in ["No ports available", "Error listing ports"]:
                    messagebox.showerror("Connection Error", "No valid port selected.")
                    return

                self.nrf24.port = port
                if self.nrf24.connect():
                    self.connection_status.configure(text="Status: Connected", fg_color="#005500")
                    self.connect_button.configure(text="Disconnect")

                    # Enable rocket control buttons
                    self.wake_up_button.configure(state="normal")
                    self.reboot_button.configure(state="normal")
                    self.logging_button.configure(state="normal")

                    # Start a new log file
                    self.start_logging()
                else:
                    messagebox.showerror("Connection Error", f"Failed to connect to {port}.")
            else:
                self.nrf24.disconnect()
                self.connection_status.configure(text="Status: Disconnected", fg_color="#550000")
                self.connect_button.configure(text="Connect")

                # Disable rocket control buttons
                self.wake_up_button.configure(state="disabled")
                self.launch_button.configure(state="disabled")
                self.deploy_parachute_button.configure(state="disabled")
                self.abort_button.configure(state="disabled")
                self.reboot_button.configure(state="disabled")

                # Reset system indicators
                self.reset_system_indicators()

                # Stop logging
                self.stop_logging()

    def update_system_indicators(self, simulation=False):
        """Update system status indicators"""
        if simulation:
            self.system_indicators["RF Signal"].configure(text="100%", fg_color="#00AA00")
            self.system_indicators["Battery"].configure(text="95%", fg_color="#00AA00")
            self.system_indicators["Data Link"].configure(text="Active", fg_color="#00AA00")
            self.system_indicators["GPS Lock"].configure(text="Strong (9 sats)", fg_color="#00AA00")
        else:
            # Here you would update with real data from the rocket
            pass

    def reset_system_indicators(self):
        """Reset system status indicators to default state"""
        for indicator in self.system_indicators.values():
            indicator.configure(text="Inactive", fg_color="#777777")

    def launch_rocket(self):
        """Initiate rocket launch sequence with confirmation"""
        result = messagebox.askokcancel(
            "Launch Confirmation",
            "Are you sure you want to LAUNCH the rocket?\n\nThis action cannot be undone!",
            icon=messagebox.WARNING
        )

        if result:
            self.send_command(CommandCodes.CMD_LAUNCH)

    def abort_mission(self):
        """Abort mission with confirmation"""
        result = messagebox.askokcancel(
            "ABORT Confirmation",
            "Are you sure you want to ABORT the mission?\n\nThis will deploy the parachute and may damage the rocket!",
            icon=messagebox.ERROR
        )

        if result:
            self.send_command(CommandCodes.CMD_ABORT)

    def send_command(self, command, parameter=0):
        """Send a command to the rocket or simulator"""
        if self.simulation_mode:
            # Map CommandCodes to SimCommandCodes if needed (they should match)
            sim_command = CommandCodes[command.name]

            success = self.simulator.send_command(sim_command, parameter)
            if success:
                # Log the command
                current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{current_time}] Sent command to simulator: {command.name}")

                # Command-specific actions
                if command == CommandCodes.CMD_LAUNCH:
                    self.state_value.configure(text="LAUNCHING")
                elif command == CommandCodes.CMD_ABORT:
                    self.state_value.configure(text="ABORTING")
                    self.parachute_value.configure(text="DEPLOYING", text_color="#FF9900")
                elif command == CommandCodes.CMD_DEPLOY_PARACHUTE:
                    self.parachute_value.configure(text="DEPLOYING", text_color="#FF9900")

                return True
            else:
                messagebox.showerror("Command Error", f"Simulator rejected {command.name} command.")
                return False
        else:
            # Original code for real hardware
            if not self.nrf24.connected:
                messagebox.showerror("Command Error", "Not connected to rocket.")
                return False

            success = self.nrf24.send_command(command, parameter)
            if success:
                # Log the command
                current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{current_time}] Sent command: {command.name}")

                # Command-specific actions
                if command == CommandCodes.CMD_LAUNCH:
                    self.state_value.configure(text="LAUNCHING")
                elif command == CommandCodes.CMD_ABORT:
                    self.state_value.configure(text="ABORTING")
                    self.parachute_value.configure(text="DEPLOYING", text_color="#FF9900")
                elif command == CommandCodes.CMD_DEPLOY_PARACHUTE:
                    self.parachute_value.configure(text="DEPLOYING", text_color="#FF9900")

                return True
            else:
                messagebox.showerror("Command Error", f"Failed to send {command.name} command.")
                return False

    def toggle_logging(self):
        """Start or stop logging telemetry data"""
        if self.data_logger.current_log_file is None:
            self.start_logging()
        else:
            self.stop_logging()

    def start_logging(self):
        """Start a new log file"""
        try:
            filename = self.data_logger.start_new_log()
            self.logging_button.configure(text="Stop Logging")
            self.log_status.configure(text=f"Logging: Active - {os.path.basename(filename)}")
            self.export_button.configure(state="normal")
        except Exception as e:
            messagebox.showerror("Logging Error", f"Failed to start logging: {e}")

    def stop_logging(self):
        """Stop the current logging session"""
        if self.data_logger.current_log_file:
            self.data_logger.close_log()
            self.logging_button.configure(text="Start Logging")
            self.log_status.configure(text="Logging: Inactive")

    def export_flight_data(self):
        """Export collected data to a JSON file"""
        if not self.time_data:
            messagebox.showinfo("Export Info", "No flight data available to export.")
            return

        try:
            filename = filedialog.asksaveasfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Export Flight Data"
            )

            if not filename:
                return

            flight_data = {
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "time_data": self.time_data,
                "altitude_data": self.altitude_data,
                "temperature_data": self.temperature_data,
                "pressure_data": self.pressure_data,
                "acceleration_data": self.acceleration_data,
                "gyro_data": self.gyro_data,
                "max_altitude": max(self.altitude_data) if self.altitude_data else 0
            }

            with open(filename, 'w') as f:
                json.dump(flight_data, f, indent=2)

            messagebox.showinfo("Export Successful", f"Flight data exported to {filename}")
        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to export data: {e}")

    def update_telemetry(self):
        """Thread function to update telemetry data from the NRF24 USB or simulator"""
        while self.running:
            if self.simulation_mode:
                # Simulation mode
                if not self.simulator.running:
                    time.sleep(0.1)
                    continue

                telemetry = self.simulator.get_telemetry()

                if telemetry:
                    # Update UI with telemetry data
                    self.after(10, lambda t=telemetry: self.update_ui(t))
            else:
                # Real hardware mode
                if not self.nrf24.connected:
                    time.sleep(0.1)
                    continue

                telemetry = self.nrf24.get_telemetry()
                if telemetry:
                    # Update UI with telemetry data
                    self.after(10, lambda t=telemetry: self.update_ui(t))

            time.sleep(0.05)  # Update at approximately 20 Hz

    def update_ui(self, telemetry):
        """Update UI elements with new telemetry data"""
        # Determine the current state based on telemetry
        # In a real implementation, the state would come from the rocket
        prev_state = self.current_state

        # Get the state from the simulator if in simulation mode
        if self.simulation_mode:
            self.current_state = self.simulator.current_state
        else:
            # Example state determination logic (replace with actual logic from your rocket)
            if telemetry.landed:
                self.current_state = RocketStates.LANDED
            elif telemetry.parachute_deployed:
                self.current_state = RocketStates.PARACHUTE_DESCENT
            elif telemetry.has_reached_apogee:
                self.current_state = RocketStates.DESCENT
            elif telemetry.altitude > 10:  # Arbitrary threshold
                self.current_state = RocketStates.FLIGHT
            else:
                # Keep the previous state or default to IDLE
                pass

        # If state changed, update state-dependent UI
        if self.current_state != prev_state:
            self.update_state_ui()

        # Basic telemetry values
        self.altitude_value.configure(text=f"{telemetry.altitude:.1f} m")
        self.temp_value.configure(text=f"{telemetry.temperature:.1f} °C")
        self.pressure_value.configure(text=f"{telemetry.pressure:.1f} hPa")

        # Parachute status
        if telemetry.parachute_deployed:
            self.parachute_value.configure(text="DEPLOYED", text_color="#FF9900")
        else:
            self.parachute_value.configure(text="SECURED", text_color="#ff1313")

        # Update max altitude
        current_max = float(self.max_altitude_value.cget("text").split()[0])
        if telemetry.altitude > current_max:
            self.max_altitude_value.configure(text=f"{telemetry.altitude:.1f} m")

        # Update detailed telemetry values
        self.telemetry_values["Acceleration X"].configure(text=f"{telemetry.acceleration_x:.2f} m/s²")
        self.telemetry_values["Acceleration Y"].configure(text=f"{telemetry.acceleration_y:.2f} m/s²")
        self.telemetry_values["Acceleration Z"].configure(text=f"{telemetry.acceleration_z:.2f} m/s²")
        self.telemetry_values["Gyroscope X"].configure(text=f"{telemetry.gyro_x:.2f} deg/s")
        self.telemetry_values["Gyroscope Y"].configure(text=f"{telemetry.gyro_y:.2f} deg/s")
        self.telemetry_values["Gyroscope Z"].configure(text=f"{telemetry.gyro_z:.2f} deg/s")
        self.telemetry_values["Latitude"].configure(text=f"{telemetry.latitude:.6f}°")
        self.telemetry_values["Longitude"].configure(text=f"{telemetry.longitude:.6f}°")

        # Flight time (relative to first telemetry received)
        if not self.time_data:
            self.start_time = time.time()

        flight_time = time.time() - self.start_time
        self.telemetry_values["Flight Time"].configure(text=f"{int(flight_time // 60)}m {int(flight_time % 60)}s")

        # Last update time
        current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.telemetry_values["Last Update"].configure(text=current_time)

        # Update simulated values for advanced telemetry
        if "Vertical Speed" in self.telemetry_values:
            # Simple vertical speed calculation
            if len(self.altitude_data) > 1 and len(self.time_data) > 1:
                dt = self.time_data[-1] - self.time_data[-2] if len(self.time_data) > 1 else 0.1
                if dt > 0:
                    vertical_speed = (telemetry.altitude - self.altitude_data[-1]) / dt
                    self.telemetry_values["Vertical Speed"].configure(text=f"{vertical_speed:.2f} m/s")

        # Simulated battery voltage (decreasing over time)
        if "Battery Voltage" in self.telemetry_values:
            battery_voltage = 12.0 - (flight_time / 3600) * 0.5  # Lose 0.5V per hour
            self.telemetry_values["Battery Voltage"].configure(text=f"{max(9.0, battery_voltage):.2f} V")

        # Update graph data
        rel_time = flight_time
        self.time_data.append(rel_time)
        self.altitude_data.append(telemetry.altitude)
        self.temperature_data.append(telemetry.temperature)
        self.pressure_data.append(telemetry.pressure)

        self.acceleration_data['x'].append(telemetry.acceleration_x)
        self.acceleration_data['y'].append(telemetry.acceleration_y)
        self.acceleration_data['z'].append(telemetry.acceleration_z)

        self.gyro_data['x'].append(telemetry.gyro_x)
        self.gyro_data['y'].append(telemetry.gyro_y)
        self.gyro_data['z'].append(telemetry.gyro_z)

        # Keep only the last 100 data points for efficiency
        max_points = 100
        if len(self.time_data) > max_points:
            self.time_data = self.time_data[-max_points:]
            self.altitude_data = self.altitude_data[-max_points:]
            self.temperature_data = self.temperature_data[-max_points:]
            self.pressure_data = self.pressure_data[-max_points:]

            for axis in ['x', 'y', 'z']:
                self.acceleration_data[axis] = self.acceleration_data[axis][-max_points:]
                self.gyro_data[axis] = self.gyro_data[axis][-max_points:]

        # Update graphs
        self.update_graphs()

        # Update system indicators based on telemetry
        if self.simulation_mode:
            # Example: Update RF signal strength based on flight phase
            signal_strength = 100
            if self.current_state == RocketStates.FLIGHT:
                # Signal gets weaker as altitude increases
                signal_strength = max(30, 100 - (telemetry.altitude / 10))

            self.system_indicators["RF Signal"].configure(
                text=f"{int(signal_strength)}%",
                fg_color="#00AA00" if signal_strength > 70 else "#AAAA00" if signal_strength > 40 else "#AA0000"
            )

            # Update GPS lock based on flight phase
            if self.current_state in [RocketStates.FLIGHT, RocketStates.DESCENT, RocketStates.PARACHUTE_DESCENT]:
                self.system_indicators["GPS Lock"].configure(text="Strong (9 sats)", fg_color="#00AA00")
            else:
                self.system_indicators["GPS Lock"].configure(text="Good (6 sats)", fg_color="#00AA00")

        # Log telemetry
        if self.data_logger.current_log_file:
            self.data_logger.log_telemetry(telemetry, self.current_state)

    def update_state_ui(self):
        """Update UI elements based on the current rocket state"""
        # Update state display
        self.state_value.configure(text=self.current_state.name)

        # Enable/disable buttons based on state
        if self.current_state == RocketStates.IDLE:
            self.wake_up_button.configure(state="normal")
            self.launch_button.configure(state="disabled")
            self.deploy_parachute_button.configure(state="disabled")
            self.abort_button.configure(state="normal")

        elif self.current_state == RocketStates.WAKING_UP or self.current_state == RocketStates.CHECKING_ROCKET:
            self.wake_up_button.configure(state="disabled")
            self.launch_button.configure(state="disabled")
            self.deploy_parachute_button.configure(state="disabled")
            self.abort_button.configure(state="normal")

        elif self.current_state == RocketStates.WAITING_FOR_LAUNCH:
            self.wake_up_button.configure(state="disabled")
            self.launch_button.configure(state="normal")
            self.deploy_parachute_button.configure(state="disabled")
            self.abort_button.configure(state="normal")

        elif self.current_state == RocketStates.FLIGHT:
            self.wake_up_button.configure(state="disabled")
            self.launch_button.configure(state="disabled")
            self.deploy_parachute_button.configure(state="normal")
            self.abort_button.configure(state="normal")

        elif self.current_state == RocketStates.DESCENT or self.current_state == RocketStates.PARACHUTE_DESCENT:
            self.wake_up_button.configure(state="disabled")
            self.launch_button.configure(state="disabled")
            self.deploy_parachute_button.configure(state="normal")
            self.abort_button.configure(state="normal")

        elif self.current_state == RocketStates.LANDED or self.current_state == RocketStates.ERROR:
            self.wake_up_button.configure(state="normal")
            self.launch_button.configure(state="disabled")
            self.deploy_parachute_button.configure(state="disabled")
            self.abort_button.configure(state="disabled")

    def update_graphs(self):
        """Update all graph plots with current data"""
        # Altitude graph
        self.altitude_line.set_data(self.time_data, self.altitude_data)
        self.altitude_ax.relim()
        self.altitude_ax.autoscale_view()
        self.altitude_canvas.draw_idle()

        # Acceleration graph
        self.accel_x_line.set_data(self.time_data, self.acceleration_data['x'])
        self.accel_y_line.set_data(self.time_data, self.acceleration_data['y'])
        self.accel_z_line.set_data(self.time_data, self.acceleration_data['z'])
        self.accel_ax.relim()
        self.accel_ax.autoscale_view()
        self.accel_canvas.draw_idle()

        # Gyroscope graph
        self.gyro_x_line.set_data(self.time_data, self.gyro_data['x'])
        self.gyro_y_line.set_data(self.time_data, self.gyro_data['y'])
        self.gyro_z_line.set_data(self.time_data, self.gyro_data['z'])
        self.gyro_ax.relim()
        self.gyro_ax.autoscale_view()
        self.gyro_canvas.draw_idle()

        # Temperature/Pressure graph
        self.temp_line.set_data(self.time_data, self.temperature_data)
        self.press_line.set_data(self.time_data, self.pressure_data)
        self.temp_ax.relim()
        self.temp_ax.autoscale_view()
        self.press_ax.relim()
        self.press_ax.autoscale_view()
        self.temp_press_canvas.draw_idle()

    def on_close(self):
        """Handle window close event"""
        self.running = False

        # Wait for update thread to finish
        if self.update_thread.is_alive():
            self.update_thread.join(2.0)

        # Disconnect hardware or stop simulator
        if self.nrf24.connected:
            self.nrf24.disconnect()

        if self.simulator.running:
            self.simulator.disconnect()

        if self.data_logger.current_log_file:
            self.data_logger.close_log()

        self.destroy()


# Main function
if __name__ == "__main__":
    app = RocketControlPanel()
    app.mainloop()