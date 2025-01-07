# Traffic Light Control System
This project implements a traffic light control system using an STM32 microcontroller and CMSIS-RTOS. The system manages traffic lights for North-South (SN) and West-East (WE) directions, including pedestrian crossing functionality.

# Table of Contents
Project Overview

Hardware Requirements

Software Requirements

Setup Instructions

Usage

Testing

Troubleshooting

License

## Project Overview
The traffic light control system consists of the following components:

Traffic Lights: Control signals for North-South and West-East directions.

Pedestrian Crossing: Buttons for pedestrians to request crossing, with corresponding signals.

### RTOS Tasks:

Traffic_Control: Manages the main traffic light sequence.

SN_Crossing_Task: Handles pedestrian crossing for the North-South direction.

WE_Crossing_Task: Handles pedestrian crossing for the West-East direction.

Button_Interrupt: Monitors pedestrian crossing buttons.

## Hardware Requirements
STM32 microcontroller (e.g., STM32F4 Discovery Board).

LEDs:

6 LEDs for traffic lights (Green, Yellow, Red for SN and WE directions).

4 LEDs for pedestrian signals (Move and Stop for SN and WE directions).

Push buttons:

2 buttons for pedestrian crossing requests (SN and WE directions).

Resistors and connecting wires.

Breadboard (optional).

## Software Requirements
STM32CubeMX: For configuring the microcontroller and generating initialization code.

STM32CubeIDE: For writing, compiling, and debugging the code.

CMSIS-RTOS: For real-time task management.

Terminal Emulator (e.g., PuTTY or Tera Term): For debugging output (optional).

## Setup Instructions
1. Hardware Setup
Connect the LEDs to the GPIO pins of the STM32 microcontroller as follows:

Traffic Lights:

SN Green → GPIOA, Pin X

SN Yellow → GPIOA, Pin Y

SN Red → GPIOA, Pin Z

WE Green → GPIOB, Pin X

WE Yellow → GPIOB, Pin Y

WE Red → GPIOB, Pin Z

Pedestrian Signals:

SN Move → GPIOA, Pin A

SN Stop → GPIOA, Pin B

WE Move → GPIOB, Pin A

WE Stop → GPIOB, Pin B

Connect the push buttons to the GPIO pins:

SN Button → GPIOA, Pin C

WE Button → GPIOB, Pin C

Use appropriate resistors for LEDs and pull-up resistors for buttons.

2. Software Setup
Install STM32CubeMX and STM32CubeIDE:

Download and install from the STMicroelectronics website.

Generate Initialization Code:

Open STM32CubeMX and create a new project for your STM32 microcontroller.

Configure GPIO pins for LEDs and buttons.

Enable CMSIS-RTOS and configure tasks, message queues, and semaphores.

Generate initialization code and open the project in STM32CubeIDE.

Add Application Code:

Copy the provided code into the main.c file in STM32CubeIDE.

Ensure the GPIO pin definitions match your hardware setup.

Build and Flash:

Build the project in STM32CubeIDE.

Flash the program to the STM32 microcontroller.

## Usage
Power On:

Power on the STM32 microcontroller.

The traffic lights will start operating in the default sequence.

Pedestrian Crossing:

Press the SN button to request a pedestrian crossing for the North-South direction.

Press the WE button to request a pedestrian crossing for the West-East direction.

The corresponding pedestrian signals will activate, and the traffic lights will adjust accordingly.


## Acknowledgments
STMicroelectronics for STM32CubeMX and STM32CubeIDE.

ARM for CMSIS-RTOS.