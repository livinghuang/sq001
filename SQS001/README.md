# Peripheral Board :  SQS001

## Humidity, Temperature, and Air Pressure Sensor Board

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

- [Datasheet for HDC1080](https://github.com/livinghuang/sq001/blob/main/SQS001/hdc1080.pdf)
- [Datasheet for BMP280](https://github.com/livinghuang/sq001/blob/main/SQS001/BST_BMP280_DS001-1509562.pdf)

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

## use with SQ001 develop board

### connection

1. Directly connect 1.25-4 connector with SQ001.

2. Directly connect 2.54-5 header with SQ001.

### copy HDC1080/BMP280 library to your arduino workplace as below

/sq001

```
    ├── sq001.ino
    ├── HDC1080.cpp
    ├── HDC1080.h
    ├── BMP280.cpp
    └── BMP280.h
```

### example code in arduino

``` cpp
BMP280 bmp;
HDC1080 hdc1080;
void power_On_Sensor_Bus()
{
  pinMode(pVext, OUTPUT);
  pinMode(pSDA, OUTPUT);
  pinMode(pSCL, OUTPUT);
  digitalWrite(pSDA, HIGH);
  digitalWrite(pSCL, HIGH);
  delay(5);
  digitalWrite(pVext, HIGH);
}

void power_Off_Sensor_Bus()
{
  pinMode(pVext, OUTPUT);
  pinMode(pSDA, OUTPUT);
  pinMode(pSCL, OUTPUT);
  digitalWrite(pSDA, LOW);
  digitalWrite(pSCL, LOW);
  digitalWrite(pVext, HIGH);
}
struct hdc1080_data
{
  float temperature;
  float humidity;
} hdc1080_result;

struct bmp280_data
{
  float bmp280_internal_temperature;
  float pressure;
} bmp280_result;

bool hdc1080_fetch(void)
{

  if (!hdc1080.begin())
  {
    return 0;
  }
  float temp = hdc1080.readTemperature();
  float Humidity = hdc1080.readHumidity();
  temp = hdc1080.readTemperature();
  Humidity = hdc1080.readHumidity();
  hdc1080.end();
  Serial.printf("T=%.2f degC, Humidity=%.2f %\n", temp, Humidity);
  hdc1080_result.temperature = temp;
  hdc1080_result.humidity = Humidity;
  return 1;
}

bool bmp280_fetch(void)
{
  if (!bmp.begin())
  {
    return 0;
  }
  bmp.setSampling(BMP280::MODE_NORMAL,     /* Operating Mode. */
                  BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  BMP280::FILTER_X16,      /* Filtering. */
                  BMP280::STANDBY_MS_500); /* Standby time. */
  delay(20);
  float temp = bmp.readTemperature();
  float Pressure = (float)bmp.readPressure() / 100.0;
  int c = 0;
  while ((temp < -50) || (Pressure > 1100) || (Pressure < 500))
  {
    bmp.putBMP280ToSleep();
    delay(10);
    bmp.end();
    Serial.println("BMP ERROR");
    Serial.flush();
    bmp.begin();
    delay(10);
    bmp.setSampling(BMP280::MODE_NORMAL,     /* Operating Mode. */
                    BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    BMP280::FILTER_X16,      /* Filtering. */
                    BMP280::STANDBY_MS_500); /* Standby time. */
    temp = bmp.readTemperature();
    Pressure = (float)bmp.readPressure() / 100.0;
    delay(10);
    c++;
    if (c > 3)
    {
      return false;
    }
  }
  bmp.putBMP280ToSleep();
  delay(10);
  bmp.end();
  Serial.printf("T=%.2f degC, Pressure=%.2f hPa\n", temp, Pressure);
  bmp280_result.bmp280_internal_temperature = temp;
  bmp280_result.pressure = Pressure;
  return true;
}
```
