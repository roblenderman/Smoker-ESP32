# Copilot Instructions for ESP32 Smoker Controller

## Project Overview
This is an ESP32-based smoker temperature controller that provides precise temperature control using multiple sensors and a PID controller. The system maintains optimal smoking conditions while preventing meat overcooking.

## Key Components
- MAX6675 thermocouple - Primary smoker chamber temperature sensor
- Two thermistors - Meat temperature monitoring
- PID controller - Maintains target smoker temperature
- Relay - Time-proportional heater control
- Rotary encoder - Setpoint adjustment and mode switching
- LCD display - Local monitoring
- Web/Telnet interface - Remote monitoring

## Critical Constraints
1. Timing Requirements:
   - MAX6675 thermocouple cannot be read more frequently than 250ms
   - Thermistors can be read more frequently (typical 5ms delay between samples)
   - Temperature readings use moving averages for stability

2. Temperature Control Logic:
   - When meat reaches target temperature, smoker setpoint automatically adjusts to meat temperature
   - PID controller manages smoker temperature
   - Temperature changes use 5°F increments
   - Valid smoker temperature range: 150°F - 350°F

## Development Workflow
1. **Building**:
   - Project uses PlatformIO with two environments:
     - `denky32_usb`: Normal USB upload
     - `denky32_ota`: OTA updates (IP: 192.168.1.225)

2. **Testing/Debugging**:
   - Telnet server on port 23 for remote monitoring
   - Web interface for status monitoring
   - LCD display shows real-time status

## Code Conventions
1. **Temperature Management**:
   - All temperatures in Fahrenheit
   - Sensor readings must use appropriate moving averages
   - Temperature offsets defined as constants for calibration

2. **Hardware Interfaces**:
   - SPI for MAX6675 (pins: CLK=18, CS=5, DO=19)
   - I2C for LCD (address 0x27)
   - ADC for thermistors (pins: 32, 33)
   - Relay control on pin 16
   - Rotary encoder on pins 13, 14, 25

## Before Making Changes
1. Always explain proposed changes before implementation
2. Check existing code to avoid duplicating functionality
3. Verify timing constraints for sensor readings
4. Allow review before committing changes
5. Document your changes in `my-instructions.md`