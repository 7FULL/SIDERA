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

        # Simulation parameters - modified for more realistic flight
        self.max_altitude = 500.0  # Maximum altitude in meters
        self.boost_phase_duration = 3.0  # Duration of engine burn in seconds
        self.boost_acceleration = 40.0  # Acceleration during boost phase in m/s²
        self.coast_drag_coefficient = 0.05  # Drag coefficient during coast phase
        self.descent_rate = 15.0  # Initial descent rate in m/s with no parachute
        self.parachute_descent_rate = 5.0  # Descent rate with parachute in m/s
        self.terminal_velocity = 20.0  # Terminal velocity without parachute in m/s

        # Flight parameters
        self.launch_time = 0
        self.boost_end_time = 0
        self.apogee_time = 0
        self.landing_time = 0
        self.ground_altitude = 0
        self.current_velocity = 0.0  # Instantaneous velocity in m/s
        self.peak_altitude = 0.0

        # Simulation control
        self.running = False
        self.sim_thread = None
        self.state_transition_time = 0

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

            # Limit the time step to avoid instability
            dt = min(dt, 0.1)

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
            self.telemetry.acceleration_z = 9.8 + random.uniform(-0.1, 0.1)  # Normal gravity reading

        elif self.current_state == RocketStates.WAKING_UP:
            # Waking up - transition to checking after 2 seconds
            if time.time() - self.state_transition_time > 2.0:
                self.current_state = RocketStates.CHECKING_ROCKET
                self.state_transition_time = time.time()

        elif self.current_state == RocketStates.CHECKING_ROCKET:
            # Checking sensors - transition to waiting after 1.5 seconds
            if time.time() - self.state_transition_time > 1.5:
                self.current_state = RocketStates.WAITING_FOR_LAUNCH
                self.state_transition_time = time.time()

        elif self.current_state == RocketStates.WAITING_FOR_LAUNCH:
            # Waiting for launch command - nothing special
            self.telemetry.acceleration_z = 9.8 + random.uniform(-0.2, 0.2)  # Normal gravity with slight movements

        elif self.current_state == RocketStates.FLIGHT:
            flight_time = time.time() - self.launch_time

            # BOOST PHASE: Rapid acceleration for the first few seconds
            if flight_time < self.boost_phase_duration:
                # During boost, the rocket accelerates rapidly
                self.telemetry.acceleration_z = self.boost_acceleration + random.uniform(-2.0, 2.0)

                # Update velocity and position using kinematics equations
                self.current_velocity += (self.telemetry.acceleration_z - 9.8) * dt
                self.telemetry.altitude += self.current_velocity * dt

                # More vibration during engine burn
                self.telemetry.gyro_x = random.uniform(-25.0, 25.0)
                self.telemetry.gyro_y = random.uniform(-25.0, 25.0)
                self.telemetry.gyro_z = random.uniform(-15.0, 15.0)

                # Save boost end time
                self.boost_end_time = self.launch_time + self.boost_phase_duration

            # COAST PHASE: After engine burn, the rocket continues to rise but slows due to gravity and drag
            else:
                # Gravity and drag force act against motion
                drag_deceleration = self.coast_drag_coefficient * self.current_velocity * abs(self.current_velocity)

                # Direction of drag is opposite to velocity
                if self.current_velocity > 0:
                    total_acceleration = -9.8 - drag_deceleration
                else:
                    total_acceleration = -9.8 + drag_deceleration

                self.telemetry.acceleration_z = total_acceleration + random.uniform(-0.5, 0.5)

                # Update velocity and position
                self.current_velocity += total_acceleration * dt
                self.telemetry.altitude += self.current_velocity * dt

                # Less vibration during coast phase
                self.telemetry.gyro_x = random.uniform(-5.0, 5.0)
                self.telemetry.gyro_y = random.uniform(-5.0, 5.0)
                self.telemetry.gyro_z = random.uniform(-3.0, 3.0)

            # Track peak altitude
            if self.telemetry.altitude > self.peak_altitude:
                self.peak_altitude = self.telemetry.altitude

            # Check if reached apogee (when velocity becomes negative or very close to zero)
            if self.current_velocity <= 0.5 and not self.telemetry.has_reached_apogee:
                self.telemetry.has_reached_apogee = True
                self.apogee_time = time.time()
                self.current_state = RocketStates.DESCENT
                self.state_transition_time = time.time()

        elif self.current_state == RocketStates.DESCENT:
            # Descent phase before parachute
            time_since_apogee = time.time() - self.apogee_time

            # Apply gravity and drag (terminal velocity model)
            target_velocity = -min(time_since_apogee * 5.0,
                                   self.terminal_velocity)  # Gradually approach terminal velocity
            velocity_difference = target_velocity - self.current_velocity
            acceleration = -9.8 + 2.0 * velocity_difference  # Smooth approach to terminal velocity

            self.telemetry.acceleration_z = acceleration + random.uniform(-1.0, 1.0)
            self.current_velocity += self.telemetry.acceleration_z * dt
            self.telemetry.altitude += self.current_velocity * dt

            # Increased rotation during initial descent
            self.telemetry.gyro_x = random.uniform(-30.0, 30.0)
            self.telemetry.gyro_y = random.uniform(-30.0, 30.0)
            self.telemetry.gyro_z = random.uniform(-20.0, 20.0)

            # Automatic parachute deployment at 300m or after 3 seconds of descent
            if self.telemetry.altitude <= 300 or time_since_apogee > 3.0:
                self.telemetry.parachute_deployed = True
                self.current_state = RocketStates.PARACHUTE_DESCENT
                self.state_transition_time = time.time()

        elif self.current_state == RocketStates.PARACHUTE_DESCENT:
            # Descent with parachute deployed

            # Rapid deceleration when parachute first deploys
            time_since_parachute = time.time() - self.state_transition_time

            if time_since_parachute < 0.5:  # Initial parachute deployment shock
                self.telemetry.acceleration_z = 5.0 + random.uniform(-3.0,
                                                                     3.0)  # Sudden upward force as parachute opens
                self.telemetry.gyro_x = random.uniform(-50.0, 50.0)  # Chaotic rotation during deployment
                self.telemetry.gyro_y = random.uniform(-50.0, 50.0)
                self.telemetry.gyro_z = random.uniform(-30.0, 30.0)
                self.current_velocity += self.telemetry.acceleration_z * dt
            else:
                # Stable descent with parachute
                target_velocity = -self.parachute_descent_rate
                velocity_difference = target_velocity - self.current_velocity
                self.telemetry.acceleration_z = -9.8 + 3.0 * velocity_difference  # Smooth approach to target velocity
                self.current_velocity += self.telemetry.acceleration_z * dt

                # More stable rotation during parachute descent
                self.telemetry.gyro_x = random.uniform(-10.0, 10.0)
                self.telemetry.gyro_y = random.uniform(-10.0, 10.0)
                self.telemetry.gyro_z = random.uniform(-5.0, 5.0)

            # Update altitude
            self.telemetry.altitude += self.current_velocity * dt

            # Prevent going below ground level
            if self.telemetry.altitude <= self.ground_altitude:
                self.telemetry.altitude = self.ground_altitude
                self.telemetry.landed = True
                self.landing_time = time.time()
                self.current_state = RocketStates.LANDED
                self.state_transition_time = time.time()

        elif self.current_state == RocketStates.LANDED:
            # Landed state
            self.telemetry.altitude = self.ground_altitude
            self.telemetry.acceleration_z = 9.8 + random.uniform(-0.1, 0.1)  # Normal gravity
            self.current_velocity = 0.0

            # Minimal rotation when landed
            self.telemetry.gyro_x = random.uniform(-0.5, 0.5)
            self.telemetry.gyro_y = random.uniform(-0.5, 0.5)
            self.telemetry.gyro_z = random.uniform(-0.5, 0.5)

        elif self.current_state == RocketStates.ERROR:
            # Error state - erratic readings
            self.telemetry.acceleration_z = random.uniform(-20.0, 20.0)
            self.telemetry.gyro_x = random.uniform(-180.0, 180.0)
            self.telemetry.gyro_y = random.uniform(-180.0, 180.0)
            self.telemetry.gyro_z = random.uniform(-180.0, 180.0)

        # Update pressure based on altitude
        self.telemetry.pressure = self._calculate_pressure(self.telemetry.altitude)

        # Update temperature based on altitude
        self.telemetry.temperature = self._calculate_temperature(self.telemetry.altitude)

    def _add_sensor_noise(self):
        """Add realistic noise to sensor readings"""
        # Temperature noise
        self.telemetry.temperature += random.uniform(-0.1, 0.1)

        # Pressure noise
        self.telemetry.pressure += random.uniform(-0.2, 0.2)

        # Acceleration noise
        self.telemetry.acceleration_x = random.uniform(-0.5, 0.5)
        self.telemetry.acceleration_y = random.uniform(-0.5, 0.5)

        # Add slight vibration to acceleration z (if not already updated in state handling)
        if self.current_state not in [RocketStates.FLIGHT, RocketStates.DESCENT, RocketStates.PARACHUTE_DESCENT]:
            self.telemetry.acceleration_z += random.uniform(-0.2, 0.2)

        # GPS noise (small movements)
        self.telemetry.latitude += random.uniform(-0.00001, 0.00001)
        self.telemetry.longitude += random.uniform(-0.00001, 0.00001)

    def _calculate_pressure(self, altitude):
        """Calculate atmospheric pressure based on altitude using barometric formula"""
        # Standard atmospheric pressure formula
        p0 = 1013.25  # Pressure at sea level (hPa)
        T0 = 288.15  # Standard temperature at sea level (K)
        g = 9.80665  # Gravitational acceleration (m/s²)
        M = 0.0289644  # Molar mass of Earth's air (kg/mol)
        R = 8.31447  # Universal gas constant (J/(mol·K))

        # Convert altitude to pressure using the barometric formula
        pressure = p0 * math.exp((-g * M * altitude) / (R * T0))
        return pressure

    def _calculate_temperature(self, altitude):
        """Calculate temperature based on altitude using lapse rate"""
        # Assume temperature decreases by 6.5°C per 1000m (standard atmosphere lapse rate)
        ground_temp = 20.0  # Base temperature at ground level (°C)
        lapse_rate = 6.5 / 1000.0  # °C per meter

        # Calculate temperature decrease
        temp_decrease = altitude * lapse_rate

        # Return temperature at given altitude
        return ground_temp - temp_decrease

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
            self.current_velocity = 0.0  # Start from rest
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
        self.boost_end_time = 0
        self.apogee_time = 0
        self.landing_time = 0
        self.current_velocity = 0.0
        self.peak_altitude = 0.0
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