# STM32 Line Follower Robot 🏎️

Project Completed Summer 2025

This repository contains the firmware for an autonomous line-following robot built around the STM32 microcontroller (STM32F4 series). 

The system utilizes a 5-channel IR sensor array to detect the line, leveraging hardware interrupts (EXTI) for immediate state updates. The drive system uses a dual DC motor setup, regulated by a custom Proportional-Derivative (PD) control loop to maintain high-speed tracking and dynamic recovery when the line is lost.

## 🧠 System Architecture

Instead of blocking the main loop with sensor polling, this architecture is entirely interrupt-driven and timer-based.

| Subsystem | Implementation Details |
| :--- | :--- |
| **Sensor Processing** | 5x IR sensors mapped to EXTI pins. A weighted average `[-4, -2, 0, 2, 4]` calculates the spatial error from the center line. |
| **Control Theory** | Custom PID loop (Currently running as PD: `Kp = 10`, `Ki = 0`, `Kd = 5`). Corrects trajectory by dynamically shifting the baseline PWM of `60%` across the left and right tracks. |
| **Motor Drive** | H-Bridge (e.g., L298N). Hardware PWM generation via `TIM1` (`CH1` & `CH2`) mapped directly to the motor driver enable pins. |
| **State Machine** | Features a failsafe "Recovery Mode." If the line is completely lost, the robot uses the last known error derivative to spin in place until the line is reacquired. |

## 📍 Reconstructed Pin Mapping

Even if the physical hardware is disassembled, the firmware is hardcoded to the following configuration:

### Motor Driver (H-Bridge)
* **Left Motor (Motor 1):** `IN1` = `PD8` | `IN2` = `PD9` | `PWM (TIM1_CH1)`
* **Right Motor (Motor 2):** `IN3` = `PD10` | `IN4` = `PD11` | `PWM (TIM1_CH2)`

### Sensor Array (GPIOA)
* **IR Array:** 5 discrete digital inputs triggering EXTI lines `1, 2, 3, 4, and 9_5`.

### Status Indicators (Onboard LEDs)
* **PD15 (Blue):** Active line tracking.
* **PD12 (Green):** Error near zero (Centered).
* **PD13 (Orange):** Recovery Mode (Searching Right).
* **PD14 (Red):** Recovery Mode (Searching Left).

## 🛠️ Build & Flash Instructions

This project was generated using STM32CubeMX and utilizes the STM32 HAL libraries.

1. Clone this repository (ensure all HAL `Drivers/` are pulled).
2. Open the `.ioc` file in **STM32CubeIDE** (or regenerate the Makefile).
3. Build the project. The custom control logic is safely contained within the `/* USER CODE BEGIN */` blocks in `main.c`.
4. Flash to the STM32 via the onboard ST-LINK.
