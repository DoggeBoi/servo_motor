# Servo Motor Prototype with STM32-F303RE Nucleo Board
This repository contains a proof-of-concept project demonstrating the control of a servo motor using an STM32-F303RE Nucleo board. The project is implemented in C using ST's CubeMX IDE, showcasing basic principles essential for future robotics applications.

# Features
- **Trapezoidal Profile Generator:** Allows for perfectly smooth trajectory changes at any time in the profile path.
- **PID Controller:** Implements a bilinear transform PID controller with a low pass derivative filter and dynamic integrator clamping.
- **Sensor Integration:**
  - External temperature measurement using an NTC thermistor and voltage splitter.
  - Internal temperature monitoring via the STM32F303 microcontroller's internal temperature sensor
  - Input voltage and current measurement
- **Position Sensing:** Utilizes the AS5048 absolute magnetic encoder for position feedback via SPI.
- **Communication:** Communication via CAN 2.0A using the MCP2551 transceiver for data exchange.
- **Motor Control:** Motor control through the TB6612FNG H-Bridge driver for actuation.

# To-be-added
- ROTS implementation.
- Automatic shutdown in case of detected failure.
- LSM6SDO 3-axis angle sensing using 6-axis IMU and a complementary filter.
- Current sensing using internal opamp.
- Complete CAN driver with exact memory mapping.

# Testing setup as of 19/06/24
![IMG_20240619_185021659](https://github.com/DoggeBoi/servo_motor/assets/59169880/0492c42f-4054-40ac-a414-125c2da43139)

