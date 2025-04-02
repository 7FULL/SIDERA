import time
import math
import random
import queue
import threading
from enum import Enum


# Command codes matching the Arduino code
class CommandCodes(Enum):
    CMD_PING = 1
    CMD_DEPLOY_PARACHUTE = 2
    CMD_ABORT = 3
    CMD_REBOOT = 4
    CMD_WAKE_UP = 5
    CMD_LAUNCH = 6
    CMD_START_PLATFORM = 7
    CMD_ROCKET_READY = 8


# Rocket states
class RocketStates(Enum):
    IDLE = 1
    WAKING_UP = 2
    CHECKING_ROCKET = 3
    WAITING_FOR_LAUNCH = 4
    FLIGHT = 5
    DESCENT = 6
    PARACHUTE_DESCENT = 7
    LANDED = 8
    ERROR = 9


# Simulated telemetry data
class TelemetryData:
    def __init__(self):
        self.altitude = 0.0
        self.temperature = 20.0  # Starting temperature (°C)
        self.pressure = 1013.25  # Starting pressure (hPa)
        self.acceleration_x = 0.0
        self.acceleration_y = 0.0
        self.acceleration_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.latitude = 40.7128  # Example: New York
        self.longitude = -74.0060
        self.parachute_deployed = False
        self.has_reached_apogee = False
        self.landed = False
        self.timestamp = int(time.time() * 1000)  # Current time in milliseconds


# Rocket simulator class
class RocketSimulator:
    def __init__(self):
        self.telemetry = TelemetryData()
        self.current_state = RocketStates.IDLE
        self.telemetry_queue = queue.Queue()
        self.command_history = []

        # Simulation parameters
        self.max_altitude = 500.0  # Maximum altitude in meters
        self.ascent_rate = 20.0  # Meters per second during ascent
        self.descent_rate = 10.0  # Meters per second during descent
        self.parachute_descent_rate = 5.0  # Meters per second with parachute

        # Flight parameters
        self.launch_time = 0
        self.apogee_time = 0
        self.landing_time = 0
        self.ground_altitude = 0

        # Simulation control
        self.running = False
        self.sim_thread = None

    def start_simulation(self):
        """Start the simulation thread"""
        self.running = True
        self.sim_thread = threading.Thread(target=self._simulation_loop)
        self.sim_thread.daemon = True
        self.sim_thread.start()

    def stop_simulation(self):
        """Stop the simulation thread"""
        self.running = False
        if self.sim_thread:
            self.sim_thread.join(timeout=1.0)

    def _simulation_loop(self):
        """Main simulation loop"""
        last_update = time.time()

        while self.running:
            # Calculate time delta
            current_time = time.time()
            dt = current_time - last_update
            last_update = current_time

            # Update simulation based on current state
            self._update_simulation(dt)

            # Add telemetry to queue
            self.telemetry.timestamp = int(current_time * 1000)
            self.telemetry_queue.put(self._copy_telemetry())

            # Sleep to control update rate (20Hz)
            time.sleep(0.05)

    def _copy_telemetry(self):
        """Create a copy of the current telemetry data"""
        telemetry_copy = TelemetryData()
        telemetry_copy.altitude = self.telemetry.altitude
        telemetry_copy.temperature = self.telemetry.temperature
        telemetry_copy.pressure = self.telemetry.pressure
        telemetry_copy.acceleration_x = self.telemetry.acceleration_x
        telemetry_copy.acceleration_y = self.telemetry.acceleration_y
        telemetry_copy.acceleration_z = self.telemetry.acceleration_z
        telemetry_copy.gyro_x = self.telemetry.gyro_x
        telemetry_copy.gyro_y = self.telemetry.gyro_y
        telemetry_copy.gyro_z = self.telemetry.gyro_z
        telemetry_copy.latitude = self.telemetry.latitude
        telemetry_copy.longitude = self.telemetry.longitude
        telemetry_copy.parachute_deployed = self.telemetry.parachute_deployed
        telemetry_copy.has_reached_apogee = self.telemetry.has_reached_apogee
        telemetry_copy.landed = self.telemetry.landed
        telemetry_copy.timestamp = self.telemetry.timestamp
        return telemetry_copy

    def _update_simulation(self, dt):
        """Update simulation based on current state and time delta"""
        # Add noise to sensors
        self._add_sensor_noise()

        # State-specific updates
        if self.current_state == RocketStates.IDLE:
            # Idle state - baseline readings with small fluctuations
            pass

        elif self.current_state == RocketStates.WAKING_UP:
            # Waking up - transition to checking after 3 seconds
            if time.time() - self.state_transition_time > 3.0:
                self.current_state = RocketStates.CHECKING_ROCKET
                self.state_transition_time = time.time()

        elif self.current_state == RocketStates.CHECKING_ROCKET:
            # Checking sensors - transition to waiting after 2 seconds
            if time.time() - self.state_transition_time > 2.0:
                self.current_state = RocketStates.WAITING_FOR_LAUNCH
                self.state_transition_time = time.time()

        elif self.current_state == RocketStates.WAITING_FOR_LAUNCH:
            # Waiting for launch command - nothing special
            pass

        elif self.current_state == RocketStates.FLIGHT:
            # Ascent phase
            flight_time = time.time() - self.launch_time

            # Calculate altitude based on time
            self.telemetry.altitude = self._calculate_ascent_altitude(flight_time)

            # Higher acceleration and vibration during flight
            self.telemetry.acceleration_z = 15.0 + random.uniform(-2.0, 2.0)

            # Decrease in pressure with altitude
            self.telemetry.pressure = self._calculate_pressure(self.telemetry.altitude)

            # Check if reached apogee
            if self.telemetry.altitude >= self.max_altitude:
                self.telemetry.has_reached_apogee = True
                self.apogee_time = time.time()
                self.current_state = RocketStates.DESCENT
                self.state_transition_time = time.time()

        elif self.current_state == RocketStates.DESCENT:
            # Descent phase before parachute
            descent_time = time.time() - self.apogee_time

            # Calculate altitude during descent
            altitude_change = self.descent_rate * dt
            self.telemetry.altitude -= altitude_change

            # Negative acceleration during descent
            self.telemetry.acceleration_z = -12.0 + random.uniform(-3.0, 3.0)

            # Update pressure
            self.telemetry.pressure = self._calculate_pressure(self.telemetry.altitude)

            # Automatic parachute deployment at 300m or after 5 seconds
            if self.telemetry.altitude <= 300 or descent_time > 5.0:
                self.telemetry.parachute_deployed = True
                self.current_state = RocketStates.PARACHUTE_DESCENT
                self.state_transition_time = time.time()

        elif self.current_state == RocketStates.PARACHUTE_DESCENT:
            # Descent with parachute deployed

            # Calculate altitude during parachute descent
            altitude_change = self.parachute_descent_rate * dt
            self.telemetry.altitude -= altitude_change

            # Reduced acceleration with parachute
            self.telemetry.acceleration_z = -2.0 + random.uniform(-0.5, 0.5)

            # Update pressure
            self.telemetry.pressure = self._calculate_pressure(self.telemetry.altitude)

            # Check if landed
            if self.telemetry.altitude <= self.ground_altitude + 1.0:
                self.telemetry.altitude = self.ground_altitude
                self.telemetry.landed = True
                self.landing_time = time.time()
                self.current_state = RocketStates.LANDED
                self.state_transition_time = time.time()

        elif self.current_state == RocketStates.LANDED:
            # Landed state
            self.telemetry.altitude = self.ground_altitude
            self.telemetry.acceleration_z = 0.0 + random.uniform(-0.1, 0.1)

        elif self.current_state == RocketStates.ERROR:
            # Error state - erratic readings
            self.telemetry.acceleration_z = random.uniform(-20.0, 20.0)
            self.telemetry.gyro_x = random.uniform(-180.0, 180.0)
            self.telemetry.gyro_y = random.uniform(-180.0, 180.0)
            self.telemetry.gyro_z = random.uniform(-180.0, 180.0)

    def _add_sensor_noise(self):
        """Add realistic noise to sensor readings"""
        # Temperature noise
        self.telemetry.temperature += random.uniform(-0.1, 0.1)

        # Pressure noise
        self.telemetry.pressure += random.uniform(-0.2, 0.2)

        # Acceleration noise
        self.telemetry.acceleration_x = random.uniform(-0.5, 0.5)
        self.telemetry.acceleration_y = random.uniform(-0.5, 0.5)
        if self.current_state == RocketStates.IDLE or self.current_state == RocketStates.WAITING_FOR_LAUNCH:
            self.telemetry.acceleration_z = random.uniform(-0.5, 0.5)

        # Gyroscope noise
        self.telemetry.gyro_x = random.uniform(-1.0, 1.0)
        self.telemetry.gyro_y = random.uniform(-1.0, 1.0)
        self.telemetry.gyro_z = random.uniform(-1.0, 1.0)

        # GPS noise (small movements)
        self.telemetry.latitude += random.uniform(-0.00001, 0.00001)
        self.telemetry.longitude += random.uniform(-0.00001, 0.00001)

    def _calculate_ascent_altitude(self, flight_time):
        """Calculate altitude during ascent phase"""
        # Simplified model with initial acceleration
        if flight_time < 2.0:
            # Initial acceleration phase
            return self.ground_altitude + 0.5 * self.ascent_rate * flight_time * flight_time
        else:
            # Constant velocity plus diminishing acceleration
            initial_alt = self.ground_altitude + 0.5 * self.ascent_rate * 4.0  # alt at t=2
            remaining_time = flight_time - 2.0
            max_altitude_factor = 1.0 - math.exp(-remaining_time / 10.0)  # Exponential approach to max
            return initial_alt + (self.max_altitude - initial_alt) * max_altitude_factor

    def _calculate_pressure(self, altitude):
        """Calculate atmospheric pressure based on altitude"""
        # Standard atmospheric pressure formula
        return 1013.25 * math.exp(-altitude / 8500.0)

    def send_command(self, command, parameter=0):
        """Process a command sent to the rocket"""
        # Log the command
        self.command_history.append((command, parameter, time.time()))

        # Process specific commands
        if command.value == CommandCodes.CMD_WAKE_UP.value and self.current_state == RocketStates.IDLE:
            # Wake up command
            self.current_state = RocketStates.WAKING_UP
            self.state_transition_time = time.time()
            return True

        elif command.value == CommandCodes.CMD_LAUNCH.value and self.current_state == RocketStates.WAITING_FOR_LAUNCH:
            # Launch command
            self.current_state = RocketStates.FLIGHT
            self.launch_time = time.time()
            self.state_transition_time = time.time()
            return True

        elif command.value == CommandCodes.CMD_DEPLOY_PARACHUTE.value and (
                self.current_state == RocketStates.FLIGHT or
                self.current_state == RocketStates.DESCENT):
            # Deploy parachute command
            self.telemetry.parachute_deployed = True
            self.current_state = RocketStates.PARACHUTE_DESCENT
            self.state_transition_time = time.time()
            return True

        elif command.value == CommandCodes.CMD_ABORT.value:
            # Abort command - deploy parachute and go to error state
            self.telemetry.parachute_deployed = True
            self.current_state = RocketStates.ERROR
            self.state_transition_time = time.time()
            return True

        elif command.value == CommandCodes.CMD_REBOOT.value:
            # Reboot command - reset everything
            self._reset_simulation()
            return True

        elif command.value == CommandCodes.CMD_PING.value:
            # Ping command - just acknowledge
            return True

        return False  # Command not processed

    def _reset_simulation(self):
        """Reset the simulation to initial state"""
        self.telemetry = TelemetryData()
        self.current_state = RocketStates.IDLE
        self.launch_time = 0
        self.apogee_time = 0
        self.landing_time = 0
        self.state_transition_time = time.time()

    def get_telemetry(self):
        """Get the next telemetry packet from the queue, if available"""
        if not self.telemetry_queue.empty():
            return self.telemetry_queue.get()
        return None

    def connect(self):
        """Simulate connection to the rocket"""
        self.running = False  # Stop any existing simulation
        if self.sim_thread:
            self.sim_thread.join(timeout=1.0)

        # Reset simulation state
        self._reset_simulation()

        # Start the simulation loop
        self.start_simulation()
        return True

    def disconnect(self):
        """Simulate disconnection from the rocket"""
        self.stop_simulation()