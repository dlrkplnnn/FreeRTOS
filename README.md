# FreeRTOS
Servo Motor Control with FreeRTOS
This repository contains a FreeRTOS-based project where a servo motor is controlled using a potentiometer. The system uses FreeRTOS tasks to read the potentiometer value and adjust the servo motor's position accordingly.

# Project Overview
The project demonstrates how to use FreeRTOS for real-time control of hardware components like a servo motor, with inputs from a potentiometer. It includes:
- Task scheduling for periodic reading of the potentiometer values
- Control of a servo motor based on the input from the potentiometer
- Demonstration of FreeRTOS queues for communication between tasks
# Prerequisites
To run this project, you'll need the following:

- Microcontroller (e.g., STM32)
- Potentiometer
- Servo motor
- A suitable IDE (e.g., STM32CubeIDE)
# How It Works
1. The potentiometer is connected to an ADC pin, and its value is read periodically in one FreeRTOS task.
2. The servo motor is controlled via PWM on another pin. The angle of the servo is adjusted according to the potentiometer value.
3. FreeRTOS tasks ensure that the potentiometer value is read and processed in real-time, with smooth control of the servo motor.
# Usage
- Turn the potentiometer to change the servo motor's position.
- The servo motor angle will smoothly follow the potentiometer's rotation.
# Wiring Diagram
- Potentiometer: Connect the middle pin to an analog input pin (PA0) (ADC_IN0).
- Servo Motor: Connect the control wire to a PWM output pin (PA1) (TIM2_CH2).
Make sure to provide power to both the potentiometer and the servo motor as required.
# Contributing
Feel free to submit issues or pull requests for improvements or new features.
