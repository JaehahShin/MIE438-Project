# Closed-Loop Balancing Robot

This repository contains the firmware and system architecture for a custom-built, self-stabilizing two-wheeled robot. Developed as part of the **MIE438: Microprocessors and Embedded Microcontrollers** course in the Robotics Engineering program at the University of Toronto.

## System Overview
The objective of this project was to achieve dynamic stabilization through a robust integration of hardware mechanics, sensor fusion, and real-time embedded control. Built on the **ESP32 platform using the ESP-IDF framework**, the system leverages multi-core scheduling to ensure deterministic execution of sensor processing and motor actuation.

### Key Features & Technical Implementations:
* **Hardware-in-the-Loop Control:** Implemented a custom, gain-scheduled PID controller in C (featuring zero-crossing integral resets and error clamping). Parameters were empirically tuned alongside iterative mechanical redesigns to reject physical disturbances and mitigate motor vibrations.
* **Sensor Fusion & Signal Processing:** Interfaced the MPU6050 IMU via fast-mode I2C (400kHz). Applied real-time digital signal processing, passing raw data through a 45-window Median Filter and a **1D Kalman Filter** to extract clean angular kinematics.
* **RTOS Multi-Core Architecture:** Utilized **FreeRTOS** to pin critical tasks to separate cores. The **66Hz PID computation** and sensor polling (Core 0) run concurrently with the high-frequency software-PWM stepper motor actuation (Core 1), completely eliminating blocking delays.

## Demo Video
[Link to the demonstration video](https://www.youtube.com/shorts/LyMY3naC9-M)

# ESP-IDF template app

This is a template application to be used with [Espressif IoT Development Framework](https://github.com/espressif/esp-idf).

Please check [ESP-IDF docs](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for getting started instructions.

*Code in this repository is in the Public Domain (or CC0 licensed, at your option.)
Unless required by applicable law or agreed to in writing, this
software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied.*
