# XDA101

## Description

XDA101 is an innovative project that leverages the power of IoT by using a temperature and humidity sensor to collect environmental data, which is then transmitted back to a LoRaWAN server. The core of the project is the Siliq SQ001, which acts as the main control unit. For precise and reliable measurements, the project uses the Texas Instruments HDC1080 temperature sensor.

## Features

- Temperature and humidity sensing with TI HDC1080.
- Data transmission over LoRaWAN using Heltec HTCT62 ESP32_LoRaWAN.
- Easy integration with LoRaWAN servers.

## Hardware Requirements

- Heltec HTCT62 ESP32_LoRaWAN module
- TI HDC1080 temperature and humidity sensor
- Additional components (wires, power supply, etc.)

## Software Requirements

- Firmware for the ESP32_LoRaWAN module (details provided in the 'Installation' section)
- LoRaWAN server setup (details provided in the 'Usage' section)

## Installation

### Setting up the Hardware

1. Connect the TI HDC1080 sensor to the Heltec HTCT62 ESP32_LoRaWAN module.
2. Ensure all connections are secure and the power supply is correctly configured.

### Software Setup

1. Flash the ESP32_LoRaWAN module with the provided firmware.
2. Configure the module to connect to your LoRaWAN server (instructions below).

## Usage

1. Power up the XDA101 device.
2. The device will automatically start sensing temperature and humidity.
3. Data is transmitted at predefined intervals to the configured LoRaWAN server.

## Contributing

We welcome contributions to the XDA101 project! If you have suggestions or improvements, please fork the repository and create a pull request, or open an issue with the tag "enhancement".

## Credits

- Heltec for the HTCT62 ESP32_LoRaWAN module.
- Texas Instruments for the HDC1080 sensor.

## License

[Specify the license under which your project is released]
