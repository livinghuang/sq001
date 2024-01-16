# Peripheral Board Overview

# SQS001: Humidity, Temperature, and Air Pressure Sensor Board

## Overview

The SQS001 is a versatile peripheral board designed to provide precise measurements of humidity, temperature, and air pressure. It integrates two essential sensors, the HDC1080 and BMP280, to fulfill these critical functions.

## Key Features

- **HDC1080 Sensor**: This high-precision sensor measures humidity and temperature with exceptional accuracy.

- **BMP280 Sensor**: Renowned for its precise air pressure and temperature measurements. Note that temperature readings are derived from the internal temperature sensor of the BMP280 chip.

- **Integration**: By combining the HDC1080 and BMP280 sensors on a single board, the SQS001 simplifies data acquisition and offers a compact solution for environmental monitoring.

- **Versatile Applications**: The SQS001 can be used in various applications, including weather monitoring, indoor climate control, and environmental sensing.

## Pinout

![SQS001 Pinout Diagram](https://github.com/livinghuang/sq001/blob/main/SQS001/SQS001.png?raw=true)

## Resources

- [Datasheet for HDC1080](link-to-hdc1080-datasheet)
- [Datasheet for BMP280](link-to-bmp280-datasheet)

For more information and detailed specifications, please refer to the datasheets linked above.

## Usage

1. **Power Supply**: Connect the VCC and GND pins to a 3.3V power source.

2. **I2C Communication**: Utilize the SDA and SCL pins for I2C communication with your microcontroller.

3. **Data Retrieval**: Retrieve humidity, temperature, and air pressure data from the HDC1080 and BMP280 sensors using your microcontroller.

4. **Alert Function (Optional)**: You can configure the ALERT pin to trigger an interrupt when specific conditions are met, providing additional control over your application.

## Example Code

Here's a sample Python code snippet to read data from the SQS001 using a Raspberry Pi:

```python
# Sample Python code to read data from SQS001 using a Raspberry Pi
import smbus2
import time

# Define the I2C bus number (usually 1 for Raspberry Pi)
bus = smbus2.SMBus(1)

# HDC1080 address
hdc1080_addr = 0x40

# BMP280 address
bmp280_addr = 0x76

# Configure HDC1080 to measure temperature and humidity
bus.write_i2c_block_data(hdc1080_addr, 0x02, [0x10])

# Read data from HDC1080
data = bus.read_i2c_block_data(hdc1080_addr, 0x00, 4)

# Convert data to temperature and humidity
temperature = ((data[0] << 8) + data[1]) / 65536.0 * 165.0 - 40.0
humidity = ((data[2] << 8) + data[3]) / 65536.0 * 100.0

# Read data from BMP280
data = bus.read_i2c_block_data(bmp280_addr, 0xF7, 8)
pressure = ((data[0] << 16) + (data[1] << 8) + data[2]) / 100.0
temperature_bmp = ((data[3] << 12) + (data[4] << 4) + (data[5] >> 4)) / 100.0

print(f"Temperature (HDC1080): {temperature}°C")
print(f"Humidity (HDC1080): {humidity}%")
print(f"Temperature (BMP280): {temperature_bmp}°C")
print(f"Pressure (BMP280): {pressure} hPa")
```
