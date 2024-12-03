# Sensors
Temperature Sensor (TMP117), UV Sensor (VEML6075 ), Pressure Sensor, 6-Axis Sensor (ISM330DHCX)

# Multi-Sensor Data Logger and Visualizer

This project demonstrates interfacing multiple sensors using Arduino, collecting data for temperature, UV radiation, pressure, and 6-axis motion (accelerometer and gyroscope). The collected data is printed to the serial monitor for real-time monitoring.

## Features

- **Temperature Data**: Uses the TMP117 sensor for precise temperature readings in Celsius and Fahrenheit.
- **UV Data**: Measures UVA, UVB, and UV Index using the VEML6075 UV sensor.
- **Pressure Data**: Collects atmospheric pressure data in PSI and Pascal using the MicroPressure sensor.
- **6-Axis Motion Data**: Captures accelerometer and gyroscope readings using the ISM330DHCX sensor.
- **GNSS Module (SFE UBlox GNSS)**: Provides latitude, longitude, and altitude data.

## Requirements

### Hardware
- **SparkFun TMP117** (Temperature Sensor)
- **SparkFun VEML6075** (UV Sensor)
- **SparkFun MicroPressure Sensor** (Atmospheric Pressure Sensor)
- **SparkFun ISM330DHCX** (6-Axis Accelerometer and Gyroscope)
- **Arduino Board** (compatible with I²C communication, e.g., Arduino UNO R4 WiFi)
- **Qwiic Connectors** (for seamless wiring of sensors)
- **MicroSD Card Module and compatible SD card** (for data storage)
- **SparkFun UBlox GNSS Module** (GNSS Receiver)

### Software
- Arduino IDE (latest version)
- Required Arduino Libraries:
  - `Wire`
  - `SparkFun_TMP117`
  - `SparkFun_TMP117_Registers`
  - `SparkFun_MicroPressure`
  - `SparkFun_VEML6075_Arduino_Library`
  - `SparkFun_ISM330DHCX`
  - `SparkFun_u-blox_GNSS_Arduino_Library`
  - `SPI`
  - `SD`

## Setup

1. **Wiring:**
   - Connect the sensors via Qwiic connectors.
   - Ensure all sensors are connected to the correct I²C addresses:
     - TMP117: `0x48`
     - VEML6075: Default (autodetect)
     - MicroPressure: `0x18`
     - ISM330DHCX: `0x6B`
     - GNSS: `0x42`

2. **Install Libraries:**
   - Open the Arduino IDE.
   - Go to **Sketch** -> **Include Library** -> **Manage Libraries**.
   - Search for the required libraries (e.g., `SparkFun_TMP117`) and install them.

3. **Upload Code:**
   - Clone this repository or copy the code to the Arduino IDE.
   - Select the correct board and port under **Tools**.
   - Click **Upload**.

## Code Overview

### Initialization
Each sensor is initialized in the `setup()` function. Initialization messages are printed to the serial monitor to confirm successful setup.

### Sensor Data Collection
The `loop()` function reads data from all sensors and calls `printSensorReadings()` to display the readings in an organized format.

### Data Output Format
- **UV Data:** UVA, UVB, UV Index
- **Temperature Data:** Temperature in °C and °F
- **Pressure Data:** Atmospheric pressure in PSI and Pascal
- **Motion Data:** Accelerometer (X, Y, Z) and Gyroscope (X, Y, Z)

### Sample Output
```
=============== Sensor Readings ===============
UV Data:
    UVA:        5.32
    UVB:        3.15
    UV Index:   4.23

Temperature Data:
    Celsius:    23.45
    Fahrenheit: 74.21

Pressure Data:
    PSI:        14.6959
    Pa:         101325.0

6-Axis Sensor Data (ISM330DHCX):
  Accelerometer:
      X:       0.12
      Y:       -0.04
      Z:       9.81
  Gyroscope:
      X:       0.03
      Y:       -0.01
      Z:       0.00
==============================================
```

## Future Enhancements
- Integrate SD card logging for offline data storage.
- Add a real-time clock (RTC) module for timestamped data.
- Implement visualization tools using Python or MATLAB.

## License
This project is open-source and available under the [MIT License](LICENSE).

## Contact
For questions, issues, or contributions, please open an issue or submit a pull request on this repository.
