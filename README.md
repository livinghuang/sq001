# Development Board Overview

This innovative development board is designed for enthusiasts and professionals seeking a robust platform for advanced communication and development, particularly in the realms of IoT and environmental technology. It's built around a powerful main chip and offers an array of features to support a wide range of applications.

![SQ001 Development Board](https://github.com/livinghuang/sq001/blob/main/SQ001.png?raw=true)

## Features

- **Main Chip**: RISC-V at 160MHz, offering high performance for computing and processing tasks.
- **Arduino Platform Compatibility**: Fully compatible with the Arduino platform, providing flexibility and ease of programming for developers.
- **On-Board PCB Antenna**: Supports WiFi and BLE communications, enhancing connectivity and data transmission capabilities.
- **WS2812B LED Integration**: Adds the capability for RGB LED control, perfect for custom lighting and signaling.
- **Li-ion Battery Management**: Incorporates management circuitry for Li-ion batteries, ensuring safe and efficient power usage.
- **Solar Charger Circuit**: An innovative design that allows the use of solar panels to charge batteries, boosting output to 5V.
- **Connectivity**: Features two 2.54-10 connectors for standard breadboard compatibility, and one 1.25-4 connector for external communication with sensors and other devices.

## Schematic

![SQ001 Schematic](https://github.com/livinghuang/sq001/blob/main/schematic_sq001.png?raw=true)

## Applications

The board is ideal for a range of applications, including but not limited to:
- IoT solutions
- Remote sensing
- Environmental monitoring
- Smart agriculture

## Getting Started

To begin using the development board:
1. Install the latest Arduino IDE for your development environment.
2. Connect the board to your computer using a USB cable.
3. Install necessary drivers and libraries as instructed.
4. Explore and experiment with sample projects.

## Peripheral

SQ001 offers robust peripheral support to assist users in completing their tasks efficiently. We have already introduced the initial peripheral board, SQS001, at the project's outset. Additionally, we are committed to continually developing the following target peripherals.

1. SQS001: This board integrates HDC1080 and BMP280 sensors, providing humidity, temperature, and air pressure measurements in a single package.
2. SQS1080: Designed specifically for humidity and temperature measurements, this peripheral incorporates the HDC1080 sensor.
3. SQS280: This peripheral focuses on air pressure and temperature measurements, utilizing the BMP280 sensor. Note that temperature readings are derived from the internal temperature sensor of the BMP280 chip.
4. SQS6050: Equipped with the MPU6050 sensor, this peripheral enables acceleration sensing capabilities.
5. SQC485: This peripheral employs an RS485 to UART chip for communication purposes.
6. SQCNBIOT: Designed for NBIOT connectivity, this peripheral uses a UART module to facilitate communication.
7. SQC4G: Featuring a 4G module, this peripheral utilizes a UART module for communication tasks.
8. SQSH001: This peripheral serves as a versatile housing, accommodating SMA Antennas, IPEX1 Antennas, Li-ion Batteries, and USB connectors.

We are dedicated to expanding our range of peripherals to cater to a wide range of applications and user needs.

## Community Engagement and Support

We encourage community contributions and offer support through various channels:
- **Contributions**: For those interested in contributing, guidelines are available.
- **Support**: For any issues or inquiries, contact our support team or use our GitHub issue tracker.

## License

The development board is released under the MIT License, providing freedom and flexibility in development and distribution.

# Peripheral Board Link

## SQS001

https://github.com/livinghuang/sq001/blob/main/SQS001/

