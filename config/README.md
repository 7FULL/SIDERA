# Configuration Files for Rocket Flight Computer

This directory contains configuration files for the rocket flight computer system.

## File Structure

```
config/
├── flight_config.txt          # Main flight configuration (COPY TO SD CARD)
├── flight_config_template.txt # Template with all options documented
├── test_config.txt            # Safe configuration for ground testing
└── README.md                  # This file
```

## Usage Instructions

### For Flight Operations:
1. Copy `flight_config.txt` to your SD card in the `/config/` directory
2. Modify parameters as needed for your specific rocket
3. Always test changes on the ground first
4. Verify `enable_test_mode=false` before flight

### For Ground Testing:
1. Copy `test_config.txt` to your SD card as `/config/flight_config.txt`
2. Verify `enable_test_mode=true` is set
3. All pyrotechnics are disabled for safety
4. Use for system testing and calibration

### Creating Custom Configurations:
1. Start with `flight_config_template.txt`
2. Read all comments carefully
3. Adjust parameters for your specific rocket
4. Validate with ground tests first

## Critical Safety Notes

⚠️ **ALWAYS VERIFY TEST MODE SETTING**
- `enable_test_mode=true` = Safe for ground testing (no pyrotechnics)
- `enable_test_mode=false` = Flight mode (pyrotechnics enabled)

⚠️ **NEVER FLY WITH TEST CONFIGURATION**
- Test configurations have pyrotechnics disabled
- Recovery systems may not function properly
- Always use flight-specific configurations for actual launches

## Parameter Guidelines

### Launch Detection
- `launch_accel_threshold`: Set 2-3x your motor's initial acceleration
- `launch_time_threshold`: 500-1000ms works for most motors
- Test by hand-moving the system to verify sensitivity

### Recovery System
- `main_altitude`: Set based on field size and local regulations
- `pyro_fire_duration`: Usually 500-1000ms for reliable deployment
- `backup_timer_*`: Redundant timers in case altitude detection fails

### Sensor Settings
- `sensor_update_rate`: 50-200Hz is typical (higher = more data/power)
- Filter values 0.1-0.3 work well for most applications
- Enable Kalman filter for better altitude/velocity estimates

## Configuration Validation

The system automatically validates configurations on startup:
- Checks parameter ranges and types
- Warns about potentially unsafe settings
- Creates default config if file is missing
- Backs up configurations before changes

## Troubleshooting

### SD Card Issues
- Ensure SD card is FAT32 formatted
- Check that `/config/` directory exists
- Verify file permissions are correct
- Try reformatting SD card if problems persist

### Configuration Errors
- Check serial output for validation messages
- Verify syntax (no spaces around `=` signs)
- Ensure all required parameters are present
- Use template file as reference for correct format

### Flight Issues
- Always perform preflight configuration check
- Verify sensor calibration values are current
- Check that recovery altitudes are appropriate for field
- Test pyrotechnic continuity before flight

## Advanced Features

### Calibration Values
- Automatically updated during ground calibration
- Compensate for sensor mounting orientation
- Account for temperature and pressure changes
- Should be refreshed before each flight day

### Logging Configuration
- Adjust `log_rate` based on SD card speed/capacity
- Enable `verbose_logging` for troubleshooting
- Monitor SD card free space before flights
- Log files are automatically named with timestamps

### Communication Settings
- `telemetry_rate` affects radio bandwidth usage
- Higher baud rates may not work with all radio modules
- Ground commands can be disabled for flight safety
- Adjust timeouts based on radio range/reliability

## Version History

- v1.0: Initial configuration system
- Future versions will add more parameters and features

For technical support or questions about configuration parameters, refer to the main project documentation or contact the development team.