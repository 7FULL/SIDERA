import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import datetime
import re
import matplotlib.dates as mdates
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.signal import savgol_filter

# Set the style for the plots
plt.style.use('seaborn-v0_8-darkgrid')
sns.set_context("paper")

# Define custom colors for better visualizations
COLORS = {
    'primary': '#1f77b4',
    'secondary': '#ff7f0e',
    'tertiary': '#2ca02c',
    'quaternary': '#d62728',
    'warning': '#9467bd',
    'success': '#8c564b',
    'info': '#e377c2',
    'gray': '#7f7f7f',
    'gold': '#bcbd22',
    'purple': '#17becf'
}


def parse_log_file(log_text):
    """Parse the log file text into structured data."""
    lines = log_text.strip().split('\n')

    lines = lines[1:]

    # Initialize lists for different types of data
    info_logs = []
    state_logs = []
    sensor_data = []
    flight_logs = []

    for line in lines:
        if not line.strip():
            continue

        parts = line.split(',')
        timestamp = int(parts[0])

        if len(parts) > 1:
            if '[INFO]' in parts[1]:
                info_logs.append({
                    'timestamp': timestamp,
                    'message': parts[1].replace('[INFO] ', '')
                })
            elif '[STATE]' in parts[1]:
                state_logs.append({
                    'timestamp': timestamp,
                    'state': parts[1].replace('[STATE] ', '')
                })
            elif 'SensorData' == parts[1]:
                # Parse sensor data
                try:
                    sensor_data.append({
                        'timestamp': timestamp,
                        'temperature': float(parts[2]),
                        'pressure': float(parts[3]),
                        'altitude': float(parts[4]),
                        'humidity': float(parts[5]),
                        'accel_x': float(parts[6]),
                        'accel_y': float(parts[7]),
                        'accel_z': float(parts[8]),
                        'gyro_x': float(parts[9]),
                        'gyro_y': float(parts[10]),
                        'gyro_z': float(parts[11]),
                        'latitude': float(parts[12]),
                        'longitude': float(parts[13]),
                        'datetime': parts[14] if len(parts) > 14 else None
                    })
                except (ValueError, IndexError) as e:
                    print(f"Error parsing sensor data: {e} in line: {line}")
            elif 'FLIGHT:' in parts[1]:
                # Parse flight data
                match = re.search(r'Descent rate: ([\-\d\.]+) m/s at altitude ([\d\.]+)m', parts[1])
                if match:
                    flight_logs.append({
                        'timestamp': timestamp,
                        'descent_rate': float(match.group(1)),
                        'altitude': float(match.group(2))
                    })

    # Convert to DataFrames
    info_df = pd.DataFrame(info_logs)
    state_df = pd.DataFrame(state_logs)
    sensor_df = pd.DataFrame(sensor_data)
    flight_df = pd.DataFrame(flight_logs)

    return info_df, state_df, sensor_df, flight_df


def parse_log_file_only_flight(log_text):
    """Parse the log file text into structured data."""
    lines = log_text.strip().split('\n')

    lines = lines[1:]

    # Initialize lists for different types of data
    info_logs = []
    state_logs = []
    sensor_data = []
    flight_logs = []

    flight_mode = False

    for line in lines:
        if not line.strip():
            continue

        parts = line.split(',')
        timestamp = int(parts[0])

        if len(parts) > 1:
            if '[INFO]' in parts[1]:
                info_logs.append({
                    'timestamp': timestamp,
                    'message': parts[1].replace('[INFO] ', '')
                })
            elif '[STATE]' in parts[1]:
                state_logs.append({
                    'timestamp': timestamp,
                    'state': parts[1].replace('[STATE] ', '')
                })
                if parts[1].replace('[STATE] ', '') == 'FLIGHT':
                    flight_mode = True
            elif 'SensorData' == parts[1] and flight_mode:
                # Parse sensor data
                try:
                    sensor_data.append({
                        'timestamp': timestamp,
                        'temperature': float(parts[2]),
                        'pressure': float(parts[3]),
                        'altitude': float(parts[4]),
                        'humidity': float(parts[5]),
                        'accel_x': float(parts[6]),
                        'accel_y': float(parts[7]),
                        'accel_z': float(parts[8]),
                        'gyro_x': float(parts[9]),
                        'gyro_y': float(parts[10]),
                        'gyro_z': float(parts[11]),
                        'latitude': float(parts[12]),
                        'longitude': float(parts[13]),
                        'datetime': parts[14] if len(parts) > 14 else None
                    })
                except (ValueError, IndexError) as e:
                    print(f"Error parsing sensor data: {e} in line: {line}")
            elif 'FLIGHT:' in parts[1] and flight_mode:
                # Parse flight data
                match = re.search(r'Descent rate: ([\-\d\.]+) m/s at altitude ([\d\.]+)m', parts[1])
                if match:
                    flight_logs.append({
                        'timestamp': timestamp,
                        'descent_rate': float(match.group(1)),
                        'altitude': float(match.group(2))
                    })

    # Convert to DataFrames
    info_df = pd.DataFrame(info_logs)
    state_df = pd.DataFrame(state_logs)
    sensor_df = pd.DataFrame(sensor_data)
    flight_df = pd.DataFrame(flight_logs)

    return info_df, state_df, sensor_df, flight_df


def preprocess_sensor_data(sensor_df):
    """Clean and preprocess the sensor data."""
    if sensor_df.empty:
        return sensor_df

    # Convert timestamp to seconds from start
    min_timestamp = sensor_df['timestamp'].min()
    sensor_df['time_sec'] = (sensor_df['timestamp'] - min_timestamp) / 1000

    # Calculate relative time for each data point
    sensor_df['relative_time'] = sensor_df['timestamp'] - sensor_df['timestamp'].min()

    # Add total acceleration column
    sensor_df['accel_total'] = np.sqrt(
        sensor_df['accel_x'] ** 2 +
        sensor_df['accel_y'] ** 2 +
        sensor_df['accel_z'] ** 2
    )

    # Add total angular velocity column
    sensor_df['gyro_total'] = np.sqrt(
        sensor_df['gyro_x'] ** 2 +
        sensor_df['gyro_y'] ** 2 +
        sensor_df['gyro_z'] ** 2
    )

    # Calculate velocity (this is simplified - a real analysis would need more sophisticated methods)
    sensor_df['velocity'] = sensor_df['accel_total'].cumsum() * 0.01  # Assuming 10ms between readings

    # Sort by timestamp to ensure proper order
    sensor_df = sensor_df.sort_values('timestamp')

    # Calculate delta metrics
    sensor_df['delta_altitude'] = sensor_df['altitude'].diff()
    sensor_df['delta_time'] = sensor_df['time_sec'].diff()

    # Calculate vertical velocity (m/s)
    sensor_df['vertical_velocity'] = sensor_df['delta_altitude'] / sensor_df['delta_time']

    # Smoothed velocity using Savitzky-Golay filter
    try:
        sensor_df['smoothed_velocity'] = savgol_filter(
            sensor_df['vertical_velocity'].fillna(0),
            min(15, len(sensor_df) - (len(sensor_df) % 2) - 1),
            3
        )
    except Exception as e:
        print(f"Could not apply Savitzky-Golay filter: {e}")
        sensor_df['smoothed_velocity'] = sensor_df['vertical_velocity']

    return sensor_df


def analyze_flight_phases(info_df, state_df, sensor_df):
    """Identify and analyze key flight phases."""
    phases = []

    if not state_df.empty:
        for i, row in state_df.iterrows():
            phase = {
                'state': row['state'],
                'start_time': row['timestamp'],
                'end_time': None
            }

            # Set end time for previous phase
            if i > 0:
                phases[i - 1]['end_time'] = row['timestamp']

            phases.append(phase)

        # Set end time for last phase
        if phases:
            phases[-1]['end_time'] = sensor_df['timestamp'].max() if not sensor_df.empty else state_df[
                'timestamp'].max()

    phases_df = pd.DataFrame(phases)

    # Add duration
    if not phases_df.empty:
        phases_df['duration_ms'] = phases_df['end_time'] - phases_df['start_time']
        phases_df['duration_sec'] = phases_df['duration_ms'] / 1000

    return phases_df


def extract_key_metrics(info_df, state_df, sensor_df, flight_df, phases_df):
    """Extract key metrics from the flight data."""
    metrics = {}

    # Find max altitude
    if not sensor_df.empty:
        max_altitude = sensor_df['altitude'].max()
        max_altitude_idx = sensor_df['altitude'].idxmax()
        max_altitude_time = sensor_df.loc[max_altitude_idx, 'timestamp']
        metrics['max_altitude'] = max_altitude
        metrics['max_altitude_time'] = max_altitude_time

        # Calculate time to apogee
        if not state_df.empty:
            launch_time = state_df[state_df['state'] == 'FLIGHT']['timestamp'].min()
            if pd.notna(launch_time):
                metrics['time_to_apogee_sec'] = (max_altitude_time - launch_time) / 1000

        # Max velocity
        max_velocity = sensor_df['smoothed_velocity'].max()
        metrics['max_velocity'] = max_velocity

        # Max acceleration
        max_accel = sensor_df['accel_total'].max()
        metrics['max_acceleration'] = max_accel

        # Flight duration
        if not state_df.empty:
            launch_time = state_df[state_df['state'] == 'FLIGHT']['timestamp'].min()
            land_time = state_df[state_df['state'] == 'LANDED']['timestamp'].min() if 'LANDED' in state_df[
                'state'].values else sensor_df['timestamp'].max()

            if pd.notna(launch_time) and pd.notna(land_time):
                metrics['flight_duration_sec'] = (land_time - launch_time) / 1000

    # Apogee from logs
    apogee_log = info_df[info_df['message'].str.contains('Apogee detected', na=False)]
    if not apogee_log.empty:
        apogee_message = apogee_log.iloc[0]['message']
        apogee_match = re.search(r'Apogee detected at ([\d\.]+) m', apogee_message)
        if apogee_match:
            metrics['logged_apogee'] = float(apogee_match.group(1))

    return metrics


def generate_altitude_plot(sensor_df, phases_df, metrics, fig_size=(12, 8)):
    """Generate an altitude vs time plot."""
    if sensor_df.empty:
        return None

    fig, ax = plt.subplots(figsize=fig_size)

    # Plot altitude
    ax.plot(sensor_df['time_sec'], sensor_df['altitude'],
            color=COLORS['primary'], linewidth=2, label='Altitude (m)')

    # Add annotations for key events
    if 'max_altitude' in metrics:
        max_alt_time = (metrics['max_altitude_time'] - sensor_df['timestamp'].min()) / 1000
        ax.scatter([max_alt_time], [metrics['max_altitude']],
                   color='red', s=100, zorder=5)
        ax.annotate(f"Apogee: {metrics['max_altitude']:.2f}m",
                    (max_alt_time, metrics['max_altitude']),
                    xytext=(10, 10), textcoords='offset points',
                    color='red', fontweight='bold')

    # Mark flight phases if available
    if not phases_df.empty:
        phase_colors = {
            'WAKING_UP': COLORS['gray'],
            'CHECKING_ROCKET': COLORS['gray'],
            'WAITING_FOR_LAUNCH': COLORS['info'],
            'FLIGHT': COLORS['warning'],
            'DESCENT': COLORS['quaternary'],
            'PARACHUTE_DESCENT': COLORS['tertiary'],
            'LANDED': COLORS['success']
        }

        for i, phase in phases_df.iterrows():
            start_sec = (phase['start_time'] - sensor_df['timestamp'].min()) / 1000
            end_sec = (phase['end_time'] - sensor_df['timestamp'].min()) / 1000

            # Only show phases that overlap with sensor data time range
            if start_sec <= sensor_df['time_sec'].max() and end_sec >= sensor_df['time_sec'].min():
                color = phase_colors.get(phase['state'], COLORS['gray'])
                ax.axvspan(start_sec, end_sec, alpha=0.2, color=color)

                # Add phase label if duration is sufficient
                if end_sec - start_sec > 1:
                    ax.text(start_sec + (end_sec - start_sec) / 2,
                            sensor_df['altitude'].min() + 5,
                            phase['state'],
                            ha='center', va='bottom',
                            bbox=dict(facecolor='white', alpha=0.7))

    # Add grid and customize appearance
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Altitude (meters)', fontsize=12)
    ax.set_title('Rocket Altitude Profile', fontsize=14, fontweight='bold')

    # Add key metrics as text
    metrics_text = []
    if 'max_altitude' in metrics:
        metrics_text.append(f"Max Altitude: {metrics['max_altitude']:.2f}m")
    if 'time_to_apogee_sec' in metrics:
        metrics_text.append(f"Time to Apogee: {metrics['time_to_apogee_sec']:.2f}s")
    if 'flight_duration_sec' in metrics:
        metrics_text.append(f"Flight Duration: {metrics['flight_duration_sec']:.2f}s")

    if metrics_text:
        ax.text(0.02, 0.02, '\n'.join(metrics_text),
                transform=ax.transAxes,
                bbox=dict(facecolor='white', alpha=0.7),
                verticalalignment='bottom')

    plt.tight_layout()
    return fig


def generate_velocity_acceleration_plot(sensor_df, fig_size=(12, 8)):
    """Generate velocity and acceleration plots."""
    if sensor_df.empty:
        return None

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=fig_size, sharex=True)

    # Plot velocity
    ax1.plot(sensor_df['time_sec'], sensor_df['smoothed_velocity'],
             color=COLORS['secondary'], linewidth=2, label='Vertical Velocity (m/s)')

    # Add zero reference line
    ax1.axhline(y=0, color='gray', linestyle='--', alpha=0.7)

    # Mark max velocity
    max_vel_idx = sensor_df['smoothed_velocity'].idxmax()
    max_vel_time = sensor_df.loc[max_vel_idx, 'time_sec']
    max_vel = sensor_df.loc[max_vel_idx, 'smoothed_velocity']

    ax1.scatter([max_vel_time], [max_vel], color='red', s=100, zorder=5)
    ax1.annotate(f"Max: {max_vel:.2f}m/s",
                 (max_vel_time, max_vel),
                 xytext=(10, 10), textcoords='offset points',
                 color='red', fontweight='bold')

    # Plot acceleration
    ax2.plot(sensor_df['time_sec'], sensor_df['accel_total'],
             color=COLORS['tertiary'], linewidth=2, label='Total Acceleration (G)')

    # Mark max acceleration
    max_accel_idx = sensor_df['accel_total'].idxmax()
    max_accel_time = sensor_df.loc[max_accel_idx, 'time_sec']
    max_accel = sensor_df.loc[max_accel_idx, 'accel_total']

    ax2.scatter([max_accel_time], [max_accel], color='red', s=100, zorder=5)
    ax2.annotate(f"Max: {max_accel:.2f}G",
                 (max_accel_time, max_accel),
                 xytext=(10, 10), textcoords='offset points',
                 color='red', fontweight='bold')

    # Customize appearance
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax2.grid(True, linestyle='--', alpha=0.7)

    ax1.set_ylabel('Velocity (m/s)', fontsize=12)
    ax1.set_title('Rocket Velocity Profile', fontsize=14, fontweight='bold')
    ax1.legend(loc='upper right')

    ax2.set_xlabel('Time (seconds)', fontsize=12)
    ax2.set_ylabel('Acceleration (G)', fontsize=12)
    ax2.set_title('Rocket Acceleration Profile', fontsize=14, fontweight='bold')
    ax2.legend(loc='upper right')

    plt.tight_layout()
    return fig


def generate_orientation_plot(sensor_df, fig_size=(12, 8)):
    """Generate plots for gyroscope and accelerometer data."""
    if sensor_df.empty:
        return None

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=fig_size, sharex=True)

    # Plot accelerometer data
    ax1.plot(sensor_df['time_sec'], sensor_df['accel_x'],
             color=COLORS['primary'], linewidth=1, label='X-axis')
    ax1.plot(sensor_df['time_sec'], sensor_df['accel_y'],
             color=COLORS['secondary'], linewidth=1, label='Y-axis')
    ax1.plot(sensor_df['time_sec'], sensor_df['accel_z'],
             color=COLORS['tertiary'], linewidth=1, label='Z-axis')

    # Plot gyroscope data
    ax2.plot(sensor_df['time_sec'], sensor_df['gyro_x'],
             color=COLORS['primary'], linewidth=1, label='X-axis')
    ax2.plot(sensor_df['time_sec'], sensor_df['gyro_y'],
             color=COLORS['secondary'], linewidth=1, label='Y-axis')
    ax2.plot(sensor_df['time_sec'], sensor_df['gyro_z'],
             color=COLORS['tertiary'], linewidth=1, label='Z-axis')

    # Customize appearance
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax2.grid(True, linestyle='--', alpha=0.7)

    ax1.set_ylabel('Acceleration (G)', fontsize=12)
    ax1.set_title('Accelerometer Data', fontsize=14, fontweight='bold')
    ax1.legend(loc='upper right')

    ax2.set_xlabel('Time (seconds)', fontsize=12)
    ax2.set_ylabel('Angular Velocity (°/s)', fontsize=12)
    ax2.set_title('Gyroscope Data', fontsize=14, fontweight='bold')
    ax2.legend(loc='upper right')

    plt.tight_layout()
    return fig


def generate_environmental_plot(sensor_df, fig_size=(12, 8)):
    """Generate plots for temperature, pressure, and humidity."""
    if sensor_df.empty:
        return None

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=fig_size, sharex=True)

    # Plot temperature
    ax1.plot(sensor_df['time_sec'], sensor_df['temperature'],
             color=COLORS['quaternary'], linewidth=2)

    # Plot pressure
    ax2.plot(sensor_df['time_sec'], sensor_df['pressure'],
             color=COLORS['primary'], linewidth=2)

    # Plot humidity
    ax3.plot(sensor_df['time_sec'], sensor_df['humidity'],
             color=COLORS['tertiary'], linewidth=2)

    # Customize appearance
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax2.grid(True, linestyle='--', alpha=0.7)
    ax3.grid(True, linestyle='--', alpha=0.7)

    ax1.set_ylabel('Temperature (°C)', fontsize=12)
    ax1.set_title('Temperature Throughout Flight', fontsize=14, fontweight='bold')

    ax2.set_ylabel('Pressure (hPa)', fontsize=12)
    ax2.set_title('Atmospheric Pressure', fontsize=14, fontweight='bold')

    ax3.set_xlabel('Time (seconds)', fontsize=12)
    ax3.set_ylabel('Humidity (%)', fontsize=12)
    ax3.set_title('Relative Humidity', fontsize=14, fontweight='bold')

    plt.tight_layout()
    return fig


def generate_3d_trajectory(sensor_df, fig_size=(10, 8)):
    """Generate a 3D plot of the rocket's trajectory if GPS data is available."""
    if sensor_df.empty or sensor_df['latitude'].nunique() <= 1:
        return None

    fig = plt.figure(figsize=fig_size)
    ax = fig.add_subplot(111, projection='3d')

    # Extract coordinates
    lats = sensor_df['latitude'].values
    lons = sensor_df['longitude'].values
    alts = sensor_df['altitude'].values

    # Convert to relative x, y coordinates (simplified)
    x = (lons - lons[0]) * 111320 * np.cos(np.radians(lats[0]))  # meters
    y = (lats - lats[0]) * 111320  # meters

    # Plot trajectory
    scatter = ax.scatter(x, y, alts, c=sensor_df['time_sec'], cmap='viridis',
                         s=20, alpha=0.8)

    # Draw a line connecting the points
    ax.plot(x, y, alts, color='gray', alpha=0.4, linewidth=1)

    # Add colorbar for time reference
    cbar = plt.colorbar(scatter, ax=ax, pad=0.1)
    cbar.set_label('Time (seconds)')

    # Mark start and end points
    ax.scatter([x[0]], [y[0]], [alts[0]], color='green', s=100, label='Launch')
    ax.scatter([x[-1]], [y[-1]], [alts[-1]], color='red', s=100, label='Landing')

    # Customize appearance
    ax.set_xlabel('East-West Distance (m)')
    ax.set_ylabel('North-South Distance (m)')
    ax.set_zlabel('Altitude (m)')
    ax.set_title('3D Flight Trajectory', fontsize=14, fontweight='bold')
    ax.legend()

    plt.tight_layout()
    return fig


def generate_flight_summary(metrics, phases_df):
    """Generate a textual summary of the flight."""
    summary = ["# Rocket Flight Analysis Summary\n"]

    # Key metrics
    summary.append("## Key Flight Metrics\n")

    if 'max_altitude' in metrics:
        summary.append(f"- **Maximum Altitude:** {metrics['max_altitude']:.2f} meters")
    if 'logged_apogee' in metrics:
        summary.append(f"- **Logged Apogee:** {metrics['logged_apogee']:.2f} meters")
    if 'max_velocity' in metrics:
        summary.append(f"- **Maximum Velocity:** {metrics['max_velocity']:.2f} m/s")
    if 'max_acceleration' in metrics:
        summary.append(f"- **Maximum Acceleration:** {metrics['max_acceleration']:.2f} G")
    if 'time_to_apogee_sec' in metrics:
        summary.append(f"- **Time to Apogee:** {metrics['time_to_apogee_sec']:.2f} seconds")
    if 'flight_duration_sec' in metrics:
        summary.append(f"- **Total Flight Duration:** {metrics['flight_duration_sec']:.2f} seconds")

    # Flight phases
    if not phases_df.empty:
        summary.append("\n## Flight Phases\n")

        for i, phase in phases_df.iterrows():
            duration_sec = phase['duration_sec']
            start_time_sec = (phase['start_time'] - phases_df['start_time'].min()) / 1000
            summary.append(
                f"- **{phase['state']}:** Started at T+{start_time_sec:.2f}s, lasted {duration_sec:.2f} seconds")

    return "\n".join(summary)


# NEW
def generate_descent_rate_plot(sensor_df, flight_df, fig_size=(12, 8)):
    """Generate a plot comparing calculated vs logged descent rates."""
    if sensor_df.empty or flight_df.empty:
        return None

    fig, ax = plt.subplots(figsize=fig_size)

    # Plot calculated descent rate from sensor data
    ax.plot(sensor_df['time_sec'], sensor_df['smoothed_velocity'],
            color=COLORS['primary'], linewidth=2, label='Calculated Descent Rate (m/s)')

    # Process and plot logged descent rates if available
    if not flight_df.empty:
        # Convert flight_df timestamps to seconds from start
        min_timestamp = sensor_df['timestamp'].min()
        flight_df['time_sec'] = (flight_df['timestamp'] - min_timestamp) / 1000

        # Plot logged descent rates
        ax.scatter(flight_df['time_sec'], flight_df['descent_rate'],
                   color=COLORS['secondary'], s=50, alpha=0.7, label='Logged Descent Rate (m/s)')

    # Add zero reference line
    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)

    # Customize appearance
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Descent Rate (m/s)', fontsize=12)
    ax.set_title('Descent Rate Analysis', fontsize=14, fontweight='bold')
    ax.legend(loc='best')

    # Add explanatory text
    ax.text(0.02, 0.02, 'Negative values indicate downward movement',
            transform=ax.transAxes, bbox=dict(facecolor='white', alpha=0.7),
            verticalalignment='bottom')

    plt.tight_layout()
    return fig


def generate_rotation_analysis_plot(sensor_df, phases_df, fig_size=(12, 8)):
    """Generate a plot analyzing the rotation of the rocket during flight."""
    if sensor_df.empty:
        return None

    fig, ax = plt.subplots(figsize=fig_size)

    # Calculate rotation magnitude
    sensor_df['rotation_magnitude'] = np.sqrt(
        sensor_df['gyro_x'] ** 2 + sensor_df['gyro_y'] ** 2 + sensor_df['gyro_z'] ** 2
    )

    # Plot rotation magnitude
    ax.plot(sensor_df['time_sec'], sensor_df['rotation_magnitude'],
            color=COLORS['purple'], linewidth=2, label='Rotation Magnitude (°/s)')

    # Calculate and plot rolling average to show stability
    window_size = min(21, len(sensor_df))
    if window_size > 2:  # Ensure we have enough data points
        sensor_df['rotation_stability'] = sensor_df['rotation_magnitude'].rolling(window=window_size,
                                                                                  center=True).mean()
        ax.plot(sensor_df['time_sec'], sensor_df['rotation_stability'],
                color=COLORS['tertiary'], linewidth=2, linestyle='--', label='Rotation Stability (Rolling Avg)')

    # Mark flight phases if available
    if not phases_df.empty:
        phase_colors = {
            'FLIGHT': COLORS['warning'],
            'DESCENT': COLORS['quaternary'],
            'PARACHUTE_DESCENT': COLORS['tertiary'],
        }

        for i, phase in phases_df.iterrows():
            if phase['state'] in phase_colors:
                start_sec = (phase['start_time'] - sensor_df['timestamp'].min()) / 1000
                end_sec = (phase['end_time'] - sensor_df['timestamp'].min()) / 1000

                # Only show phases that overlap with sensor data time range
                if start_sec <= sensor_df['time_sec'].max() and end_sec >= sensor_df['time_sec'].min():
                    color = phase_colors.get(phase['state'], COLORS['gray'])
                    ax.axvspan(start_sec, end_sec, alpha=0.2, color=color)

                    # Add phase label if duration is sufficient
                    if end_sec - start_sec > 1:
                        ypos = sensor_df['rotation_magnitude'].max() * 0.9
                        ax.text(start_sec + (end_sec - start_sec) / 2, ypos,
                                phase['state'], ha='center', va='bottom',
                                bbox=dict(facecolor='white', alpha=0.7))

    # Find and mark significant rotation events
    threshold = sensor_df['rotation_magnitude'].mean() + 2 * sensor_df['rotation_magnitude'].std()
    significant_rotations = sensor_df[sensor_df['rotation_magnitude'] > threshold]

    if not significant_rotations.empty:
        ax.scatter(significant_rotations['time_sec'], significant_rotations['rotation_magnitude'],
                   color='red', s=50, alpha=0.6, label='Significant Rotation Events')

    # Customize appearance
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Rotation Rate (°/s)', fontsize=12)
    ax.set_title('Rocket Rotation Analysis', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right')

    plt.tight_layout()
    return fig


def generate_combined_altitude_acceleration_plot(sensor_df, phases_df, metrics, fig_size=(12, 10)):
    """Generate a combined plot with altitude and acceleration to show their correlation."""
    if sensor_df.empty:
        return None

    fig, ax1 = plt.subplots(figsize=fig_size)

    # Plot altitude on primary y-axis
    ax1.plot(sensor_df['time_sec'], sensor_df['altitude'],
             color=COLORS['primary'], linewidth=2, label='Altitude (m)')
    ax1.set_xlabel('Time (seconds)', fontsize=12)
    ax1.set_ylabel('Altitude (meters)', fontsize=12, color=COLORS['primary'])
    ax1.tick_params(axis='y', labelcolor=COLORS['primary'])

    # Create secondary y-axis for acceleration
    ax2 = ax1.twinx()
    ax2.plot(sensor_df['time_sec'], sensor_df['accel_total'],
             color=COLORS['quaternary'], linewidth=1.5, label='Acceleration (G)')
    ax2.set_ylabel('Acceleration (G)', fontsize=12, color=COLORS['quaternary'])
    ax2.tick_params(axis='y', labelcolor=COLORS['quaternary'])

    # Add event markers
    if 'max_altitude' in metrics:
        max_alt_time = (metrics['max_altitude_time'] - sensor_df['timestamp'].min()) / 1000
        ax1.scatter([max_alt_time], [metrics['max_altitude']],
                    color='green', s=100, zorder=5)
        ax1.annotate(f"Apogee: {metrics['max_altitude']:.2f}m",
                     (max_alt_time, metrics['max_altitude']),
                     xytext=(10, 10), textcoords='offset points',
                     color='green', fontweight='bold')

    # Mark key acceleration events
    max_accel_idx = sensor_df['accel_total'].idxmax()
    max_accel_time = sensor_df.loc[max_accel_idx, 'time_sec']
    max_accel = sensor_df.loc[max_accel_idx, 'accel_total']

    ax2.scatter([max_accel_time], [max_accel], color='red', s=100, zorder=5)
    ax2.annotate(f"Max Accel: {max_accel:.2f}G",
                 (max_accel_time, max_accel),
                 xytext=(10, -20), textcoords='offset points',
                 color='red', fontweight='bold')

    # Add combined legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

    # Mark flight phases if available
    if not phases_df.empty:
        for i, phase in phases_df.iterrows():
            if phase['state'] in ['FLIGHT', 'DESCENT', 'PARACHUTE_DESCENT']:
                start_sec = (phase['start_time'] - sensor_df['timestamp'].min()) / 1000
                end_sec = (phase['end_time'] - sensor_df['timestamp'].min()) / 1000

                # Add phase transitions as vertical lines
                ax1.axvline(x=start_sec, color='black', linestyle='--', alpha=0.5)
                ax1.text(start_sec, metrics['max_altitude'] * 0.1,
                         phase['state'], rotation=90, va='bottom', ha='right')

    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.set_title('Altitude and Acceleration Correlation', fontsize=14, fontweight='bold')

    plt.tight_layout()
    return fig


def generate_temperature_altitude_correlation(sensor_df, fig_size=(12, 8)):
    """Generate a plot showing the correlation between temperature and altitude."""
    if sensor_df.empty:
        return None

    fig, ax1 = plt.subplots(figsize=fig_size)

    # Plot temperature on primary y-axis
    ax1.plot(sensor_df['time_sec'], sensor_df['temperature'],
             color=COLORS['quaternary'], linewidth=2, label='Temperature (°C)')
    ax1.set_xlabel('Time (seconds)', fontsize=12)
    ax1.set_ylabel('Temperature (°C)', fontsize=12, color=COLORS['quaternary'])
    ax1.tick_params(axis='y', labelcolor=COLORS['quaternary'])

    # Create secondary y-axis for altitude
    ax2 = ax1.twinx()
    ax2.plot(sensor_df['time_sec'], sensor_df['altitude'],
             color=COLORS['primary'], linewidth=1.5, label='Altitude (m)')
    ax2.set_ylabel('Altitude (m)', fontsize=12, color=COLORS['primary'])
    ax2.tick_params(axis='y', labelcolor=COLORS['primary'])

    # Add combined legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

    # Calculate and display temperature gradient
    # Find temperature at lowest and highest points
    altitude_sorted = sensor_df.sort_values('altitude')
    temp_low = altitude_sorted.iloc[0]['temperature']
    temp_high = altitude_sorted.iloc[-1]['temperature']
    alt_low = altitude_sorted.iloc[0]['altitude']
    alt_high = altitude_sorted.iloc[-1]['altitude']

    temp_gradient = (temp_high - temp_low) / (alt_high - alt_low) if (alt_high - alt_low) > 0 else 0

    # Add temperature gradient information
    gradient_text = f"Temperature Gradient: {temp_gradient:.3f}°C/m\n"
    gradient_text += f"Temp at {alt_low:.1f}m: {temp_low:.1f}°C\n"
    gradient_text += f"Temp at {alt_high:.1f}m: {temp_high:.1f}°C"

    ax1.text(0.02, 0.02, gradient_text,
             transform=ax1.transAxes,
             bbox=dict(facecolor='white', alpha=0.7),
             verticalalignment='bottom')

    # Add regression line to show correlation
    if len(sensor_df) > 2:
        # Create a scatter plot
        ax3 = fig.add_axes([0.15, 0.15, 0.25, 0.25])  # [left, bottom, width, height]
        ax3.scatter(sensor_df['altitude'], sensor_df['temperature'],
                    alpha=0.5, s=10, color=COLORS['secondary'])

        # Add regression line if enough points
        try:
            from scipy import stats
            slope, intercept, r_value, p_value, std_err = stats.linregress(
                sensor_df['altitude'], sensor_df['temperature'])

            x = np.array([sensor_df['altitude'].min(), sensor_df['altitude'].max()])
            y = intercept + slope * x
            ax3.plot(x, y, color='red', linestyle='--')

            # Add regression info
            ax3.text(0.05, 0.95, f"R²: {r_value ** 2:.3f}\nSlope: {slope:.4f}°C/m",
                     transform=ax3.transAxes, verticalalignment='top',
                     bbox=dict(facecolor='white', alpha=0.7))
        except:
            pass

        ax3.set_xlabel('Altitude (m)')
        ax3.set_ylabel('Temp (°C)')
        ax3.set_title('Correlation')

    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.set_title('Temperature vs. Altitude Analysis', fontsize=14, fontweight='bold')

    plt.tight_layout()
    return fig


def generate_stability_analysis_plot(sensor_df, phases_df, fig_size=(12, 8)):
    """Generate a plot analyzing the stability of the rocket during flight."""
    if sensor_df.empty:
        return None

    fig, ax = plt.subplots(figsize=fig_size)

    # Calculate stability metrics
    # 1. Lateral acceleration (X and Y components)
    sensor_df['lateral_acceleration'] = np.sqrt(
        sensor_df['accel_x'] ** 2 + sensor_df['accel_y'] ** 2
    )

    # 2. Ratio of lateral to total acceleration (lower is more stable)
    sensor_df['stability_ratio'] = sensor_df['lateral_acceleration'] / sensor_df['accel_total']
    sensor_df['stability_ratio'] = sensor_df['stability_ratio'].fillna(0)  # Handle division by zero

    # 3. Apply smoothing for clearer visualization
    window_size = min(15, len(sensor_df))
    if window_size > 2:
        sensor_df['smoothed_stability'] = sensor_df['stability_ratio'].rolling(
            window=window_size, center=True).mean().fillna(0)
    else:
        sensor_df['smoothed_stability'] = sensor_df['stability_ratio']

    # Plot stability ratio
    ax.plot(sensor_df['time_sec'], sensor_df['smoothed_stability'],
            color=COLORS['info'], linewidth=2, label='Stability Ratio')

    # Add threshold lines for stability assessment
    ax.axhline(y=0.2, color='green', linestyle='--', alpha=0.7, label='Excellent Stability')
    ax.axhline(y=0.4, color='orange', linestyle='--', alpha=0.7, label='Moderate Stability')
    ax.axhline(y=0.6, color='red', linestyle='--', alpha=0.7, label='Poor Stability')

    # Mark flight phases if available
    if not phases_df.empty:
        phase_colors = {
            'FLIGHT': COLORS['warning'],
            'DESCENT': COLORS['quaternary'],
            'PARACHUTE_DESCENT': COLORS['tertiary'],
        }

        for i, phase in phases_df.iterrows():
            if phase['state'] in phase_colors:
                start_sec = (phase['start_time'] - sensor_df['timestamp'].min()) / 1000
                end_sec = (phase['end_time'] - sensor_df['timestamp'].min()) / 1000

                # Only show phases that overlap with sensor data time range
                if start_sec <= sensor_df['time_sec'].max() and end_sec >= sensor_df['time_sec'].min():
                    color = phase_colors.get(phase['state'], COLORS['gray'])
                    ax.axvspan(start_sec, end_sec, alpha=0.2, color=color)

                    # Add phase label if duration is sufficient
                    if end_sec - start_sec > 1:
                        ax.text(start_sec + (end_sec - start_sec) / 2, 0.9,
                                phase['state'], ha='center', va='bottom',
                                bbox=dict(facecolor='white', alpha=0.7))

    # Calculate average stability per phase
    if not phases_df.empty and len(sensor_df) > 0:
        phase_stability = []

        for i, phase in phases_df.iterrows():
            if phase['state'] in ['FLIGHT', 'DESCENT', 'PARACHUTE_DESCENT']:
                phase_data = sensor_df[
                    (sensor_df['timestamp'] >= phase['start_time']) &
                    (sensor_df['timestamp'] <= phase['end_time'])
                    ]

                if len(phase_data) > 0:
                    avg_stability = phase_data['smoothed_stability'].mean()
                    phase_stability.append(f"{phase['state']}: {avg_stability:.3f}")

        if phase_stability:
            ax.text(0.02, 0.02, "Average Stability Ratio:\n" + "\n".join(phase_stability),
                    transform=ax.transAxes,
                    bbox=dict(facecolor='white', alpha=0.7),
                    verticalalignment='bottom')

    # Customize appearance
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Stability Ratio', fontsize=12)
    ax.set_title('Rocket Stability Analysis', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right')

    # Set y-axis limits for better visualization
    ax.set_ylim(0, 1)

    plt.tight_layout()
    return fig


def generate_detailed_apogee_analysis(sensor_df, fig_size=(12, 8)):
    """Generate a detailed analysis of the apogee event with multiple metrics."""
    if sensor_df.empty:
        return None

    # Identify apogee index
    apogee_idx = sensor_df['altitude'].idxmax()
    apogee_time = sensor_df.loc[apogee_idx, 'time_sec']

    # Create a window around apogee for detailed analysis
    # Find data points within ±5 seconds of apogee
    apogee_window = 5  # seconds
    window_data = sensor_df[
        (sensor_df['time_sec'] >= apogee_time - apogee_window) &
        (sensor_df['time_sec'] <= apogee_time + apogee_window)
        ]

    if window_data.empty:
        return None

    # Create the plot with multiple subplots
    fig, axes = plt.subplots(3, 1, figsize=fig_size, sharex=True)

    # 1. Altitude plot
    axes[0].plot(window_data['time_sec'], window_data['altitude'],
                 color=COLORS['primary'], linewidth=2)
    axes[0].scatter([apogee_time], [window_data['altitude'].max()],
                    color='red', s=100, zorder=5)
    axes[0].annotate(f"Apogee: {window_data['altitude'].max():.2f}m",
                     (apogee_time, window_data['altitude'].max()),
                     xytext=(10, 10), textcoords='offset points',
                     color='red', fontweight='bold')
    axes[0].set_ylabel('Altitude (m)', fontsize=12)
    axes[0].set_title('Detailed Apogee Analysis', fontsize=14, fontweight='bold')
    axes[0].grid(True, linestyle='--', alpha=0.7)

    # Add vertical line at apogee
    for ax in axes:
        ax.axvline(x=apogee_time, color='black', linestyle='--', alpha=0.5)

    # 2. Velocity plot
    axes[1].plot(window_data['time_sec'], window_data['smoothed_velocity'],
                 color=COLORS['secondary'], linewidth=2)
    axes[1].set_ylabel('Velocity (m/s)', fontsize=12)
    axes[1].grid(True, linestyle='--', alpha=0.7)

    # Highlight velocity at apogee
    apogee_velocity = window_data.loc[apogee_idx, 'smoothed_velocity'] if apogee_idx in window_data.index else 0
    axes[1].scatter([apogee_time], [apogee_velocity],
                    color='red', s=100, zorder=5)
    axes[1].annotate(f"Velocity: {apogee_velocity:.2f}m/s",
                     (apogee_time, apogee_velocity),
                     xytext=(10, 10), textcoords='offset points',
                     color='red', fontweight='bold')

    # 3. Acceleration plot
    axes[2].plot(window_data['time_sec'], window_data['accel_total'],
                 color=COLORS['tertiary'], linewidth=2)
    axes[2].set_xlabel('Time (seconds)', fontsize=12)
    axes[2].set_ylabel('Acceleration (G)', fontsize=12)
    axes[2].grid(True, linestyle='--', alpha=0.7)

    # Highlight acceleration at apogee
    apogee_accel = window_data.loc[apogee_idx, 'accel_total'] if apogee_idx in window_data.index else 0
    axes[2].scatter([apogee_time], [apogee_accel],
                    color='red', s=100, zorder=5)
    axes[2].annotate(f"Accel: {apogee_accel:.2f}G",
                     (apogee_time, apogee_accel),
                     xytext=(10, 10), textcoords='offset points',
                     color='red', fontweight='bold')

    # Add explanatory data box
    apogee_metrics = [
        f"Apogee altitude: {window_data['altitude'].max():.2f}m",
        f"Time to apogee: {apogee_time:.2f}s",
        f"Velocity at apogee: {apogee_velocity:.2f}m/s",
        f"Acceleration at apogee: {apogee_accel:.2f}G"
    ]

    # Calculate apogee detection delay if possible
    apogee_zero_velocity_time = None
    for i in range(1, len(window_data)):
        if window_data.iloc[i - 1]['smoothed_velocity'] > 0 and window_data.iloc[i]['smoothed_velocity'] <= 0:
            apogee_zero_velocity_time = window_data.iloc[i]['time_sec']
            break

    if apogee_zero_velocity_time is not None and apogee_time is not None:
        delay = apogee_time - apogee_zero_velocity_time
        apogee_metrics.append(f"Detection delay: {delay:.3f}s")

    axes[0].text(0.02, 0.02, "\n".join(apogee_metrics),
                 transform=axes[0].transAxes,
                 bbox=dict(facecolor='white', alpha=0.7),
                 verticalalignment='bottom')

    plt.tight_layout()
    return fig


def generate_energy_analysis_plot(sensor_df, fig_size=(12, 8)):
    """Generate a plot analyzing the kinetic and potential energy of the rocket."""
    if sensor_df.empty:
        return None

    # Constants
    GRAVITY = 9.81  # m/s²
    ROCKET_MASS = 1.0  # kg (estimated, adjust based on your rocket)

    # Calculate energies
    sensor_df['potential_energy'] = ROCKET_MASS * GRAVITY * sensor_df['altitude']

    # Calculate instantaneous velocity magnitude
    if 'smoothed_velocity' in sensor_df.columns:
        sensor_df['speed'] = np.abs(sensor_df['smoothed_velocity'])
    else:
        # Fallback if smoothed_velocity is not available
        sensor_df['speed'] = np.sqrt(
            sensor_df['accel_total'].cumsum() * 0.01
        )  # Simple integration, assuming 10ms between readings

    sensor_df['kinetic_energy'] = 0.5 * ROCKET_MASS * sensor_df['speed'] ** 2
    sensor_df['total_energy'] = sensor_df['potential_energy'] + sensor_df['kinetic_energy']

    fig, ax = plt.subplots(figsize=fig_size)

    # Plot energies
    ax.plot(sensor_df['time_sec'], sensor_df['potential_energy'],
            color=COLORS['primary'], linewidth=2, label='Potential Energy (J)')
    ax.plot(sensor_df['time_sec'], sensor_df['kinetic_energy'],
            color=COLORS['secondary'], linewidth=2, label='Kinetic Energy (J)')
    ax.plot(sensor_df['time_sec'], sensor_df['total_energy'],
            color=COLORS['tertiary'], linewidth=2, label='Total Energy (J)')

    # Identify and mark important energy points
    max_potential_idx = sensor_df['potential_energy'].idxmax()
    max_potential_time = sensor_df.loc[max_potential_idx, 'time_sec']
    max_potential = sensor_df.loc[max_potential_idx, 'potential_energy']

    max_kinetic_idx = sensor_df['kinetic_energy'].idxmax()
    max_kinetic_time = sensor_df.loc[max_kinetic_idx, 'time_sec']
    max_kinetic = sensor_df.loc[max_kinetic_idx, 'kinetic_energy']

    # Mark maximum points
    ax.scatter([max_potential_time], [max_potential],
               color='green', s=100, zorder=5)
    ax.annotate(f"Max PE: {max_potential:.1f}J",
                (max_potential_time, max_potential),
                xytext=(10, 10), textcoords='offset points',
                color='green', fontweight='bold')

    ax.scatter([max_kinetic_time], [max_kinetic],
               color='red', s=100, zorder=5)
    ax.annotate(f"Max KE: {max_kinetic:.1f}J",
                (max_kinetic_time, max_kinetic),
                xytext=(10, 10), textcoords='offset points',
                color='red', fontweight='bold')

    # Add energy conversion points (where PE and KE lines cross)
    crossover_points = []
    for i in range(1, len(sensor_df)):
        pe_prev = sensor_df.iloc[i - 1]['potential_energy']
        ke_prev = sensor_df.iloc[i - 1]['kinetic_energy']
        pe_curr = sensor_df.iloc[i]['potential_energy']
        ke_curr = sensor_df.iloc[i]['kinetic_energy']

        if (pe_prev > ke_prev and pe_curr <= ke_curr) or (pe_prev < ke_prev and pe_curr >= ke_curr):
            crossover_points.append(i)

    for idx in crossover_points:
        time_sec = sensor_df.iloc[idx]['time_sec']
        energy = sensor_df.iloc[idx]['potential_energy']  # Should be approximately equal to KE at this point

        ax.scatter([time_sec], [energy],
                   color='purple', s=80, zorder=5, marker='*')
        ax.annotate("Energy\nConversion",
                    (time_sec, energy),
                    xytext=(0, -30), textcoords='offset points',
                    ha='center', color='purple', fontweight='bold')

    # Calculate energy efficiency
    initial_energy = sensor_df['total_energy'].iloc[0]
    max_energy = sensor_df['total_energy'].max()
    final_energy = sensor_df['total_energy'].iloc[-1]

    energy_efficiency = (final_energy / max_energy) * 100 if max_energy > 0 else 0

    # Add energy metrics
    energy_text = [
        f"Initial Energy: {initial_energy:.1f}J",
        f"Peak Energy: {max_energy:.1f}J",
        f"Final Energy: {final_energy:.1f}J",
        f"Energy Efficiency: {energy_efficiency:.1f}%",
        f"Energy Lost: {max_energy - final_energy:.1f}J"
    ]

    ax.text(0.02, 0.02, "\n".join(energy_text),
            transform=ax.transAxes,
            bbox=dict(facecolor='white', alpha=0.7),
            verticalalignment='bottom')

    # Customize appearance
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.set_xlabel('Time (seconds)', fontsize=12)
    ax.set_ylabel('Energy (Joules)', fontsize=12)
    ax.set_title('Rocket Energy Analysis', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right')

    plt.tight_layout()
    return fig


def generate_flight_phase_comparison(sensor_df, phases_df, fig_size=(14, 10)):
    """Generate a plot comparing key metrics across different flight phases."""
    if sensor_df.empty or phases_df.empty:
        return None

    # Identificar fases principales de vuelo
    major_phases = ['FLIGHT', 'DESCENT', 'PARACHUTE_DESCENT']
    phase_data = {}

    for phase in major_phases:
        # Verificar si la fase existe en los datos
        if phase in phases_df['state'].values:
            phase_rows = phases_df[phases_df['state'] == phase]
            if not phase_rows.empty:
                phase_row = phase_rows.iloc[0]
                phase_start = phase_row['start_time']
                phase_end = phase_row['end_time']
                duration_sec = phase_row['duration_sec']

                # Obtener datos del sensor para esta fase
                phase_sensor_data = sensor_df[
                    (sensor_df['timestamp'] >= phase_start) &
                    (sensor_df['timestamp'] <= phase_end)
                    ]

                if not phase_sensor_data.empty:
                    phase_data[phase] = {
                        'data': phase_sensor_data,
                        'duration_sec': duration_sec
                    }

    # Salir si no hay datos de fase
    if not phase_data:
        return None

    # Calcular métricas para cada fase
    metrics_list = []
    for phase_name, info in phase_data.items():
        data = info['data']
        duration = info['duration_sec']

        # Calcular métricas
        avg_velocity = data['smoothed_velocity'].mean() if 'smoothed_velocity' in data.columns else 0
        max_accel = data['accel_total'].max() if 'accel_total' in data.columns else 0
        avg_rotation = data['gyro_total'].mean() if 'gyro_total' in data.columns else 0
        temp_change = data['temperature'].max() - data['temperature'].min() if 'temperature' in data.columns else 0

        metrics_list.append({
            'Phase': phase_name,
            'Duration (s)': duration,
            'Avg Velocity (m/s)': avg_velocity,
            'Max Acceleration (G)': max_accel,
            'Avg Rotation (°/s)': avg_rotation,
            'Temp Change (°C)': temp_change
        })

    # Crear DataFrame de métricas
    metrics_df = pd.DataFrame(metrics_list).set_index('Phase')

    # Configurar gráficos
    metrics = metrics_df.columns
    n_metrics = len(metrics)
    n_cols = 2
    n_rows = (n_metrics + 1) // n_cols

    fig, axes = plt.subplots(n_rows, n_cols, figsize=fig_size)
    axes = axes.flatten()

    # Colores personalizados de la paleta
    colors = [COLORS['primary'], COLORS['secondary'],
              COLORS['tertiary'], COLORS['quaternary'],
              COLORS['purple']]

    # Crear un gráfico por métrica
    for i, metric in enumerate(metrics):
        ax = axes[i]
        phases = metrics_df.index
        values = metrics_df[metric]

        # Gráfico de barras
        ax.bar(phases, values, color=colors[i % len(colors)])

        # Personalización
        ax.set_title(metric, fontsize=12)
        ax.set_ylabel(metric.split('(')[-1].strip(')') if '(' in metric else metric)
        ax.grid(True, linestyle='--', alpha=0.7)

        # Rotar etiquetas del eje X
        plt.setp(ax.get_xticklabels(), rotation=45, ha='right')

    # Ocultar ejes no usados
    for j in range(n_metrics, len(axes)):
        axes[j].axis('off')

    plt.tight_layout()
    return fig


def main():
    """Main function to process data and generate visualizations."""

    log_text = None

    with open('logs/flight_162.csv', 'r') as file:
        log_text = file.read()

    if log_text is not None:
        info_df, state_df, sensor_df, flight_df = parse_log_file_only_flight(log_text)
        sensor_df = preprocess_sensor_data(sensor_df)
        phases_df = analyze_flight_phases(info_df, state_df, sensor_df)
        metrics = extract_key_metrics(info_df, state_df, sensor_df, flight_df, phases_df)

        # Generate visualizations
        altitude_fig = generate_altitude_plot(sensor_df, phases_df, metrics)
        velocity_fig = generate_velocity_acceleration_plot(sensor_df)
        orientation_fig = generate_orientation_plot(sensor_df)
        environmental_fig = generate_environmental_plot(sensor_df)
        trajectory_fig = generate_3d_trajectory(sensor_df)
        descent_rate_fig = generate_descent_rate_plot(sensor_df, flight_df)
        rotation_analysis_fig = generate_rotation_analysis_plot(sensor_df, phases_df)
        combined_altitude_acceleration_fig = generate_combined_altitude_acceleration_plot(sensor_df, phases_df, metrics)
        temperature_altitude_fig = generate_temperature_altitude_correlation(sensor_df)
        stability_analysis_fig = generate_stability_analysis_plot(sensor_df, phases_df)
        detailed_apogee_fig = generate_detailed_apogee_analysis(sensor_df)
        energy_analysis_fig = generate_energy_analysis_plot(sensor_df)
        flight_phase_comparison_fig = generate_flight_phase_comparison(sensor_df, phases_df)

        # Generate flight summary
        summary_text = generate_flight_summary(metrics, phases_df)

        # Save figures and summary to files
        if altitude_fig:
            altitude_fig.savefig('visualizations/altitude_plot.png')
        if velocity_fig:
            velocity_fig.savefig('visualizations/velocity_plot.png')
        if orientation_fig:
            orientation_fig.savefig('visualizations/orientation_plot.png')
        if environmental_fig:
            environmental_fig.savefig('visualizations/environmental_plot.png')
        if trajectory_fig:
            trajectory_fig.savefig('visualizations/trajectory_plot.png')
        if descent_rate_fig:
            descent_rate_fig.savefig('visualizations/descent_rate_plot.png')
        if rotation_analysis_fig:
            rotation_analysis_fig.savefig('visualizations/rotation_analysis_plot.png')
        if combined_altitude_acceleration_fig:
            combined_altitude_acceleration_fig.savefig('visualizations/combined_altitude_acceleration_plot.png')
        if temperature_altitude_fig:
            temperature_altitude_fig.savefig('visualizations/temperature_altitude_plot.png')
        if stability_analysis_fig:
            stability_analysis_fig.savefig('visualizations/stability_analysis_plot.png')
        if detailed_apogee_fig:
            detailed_apogee_fig.savefig('visualizations/detailed_apogee_analysis.png')
        if energy_analysis_fig:
            energy_analysis_fig.savefig('visualizations/energy_analysis_plot.png')
        if flight_phase_comparison_fig:
            flight_phase_comparison_fig.savefig('visualizations/flight_phase_comparison.png')

        with open('flight_summary.md', 'w') as file:
            file.write(summary_text)

        print("Analysis completed successfully.")
    else:
        print("No log data found.")


main()
