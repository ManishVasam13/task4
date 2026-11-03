# Task 4 – Sandstorm Traverse
### Motor Speed Control and Anti-Stall Protection

## Project Overview
This project implements a two-motor control system using an ESP32 and FreeRTOS. It demonstrates differential steering, motor speed control, simulated current monitoring, overcurrent warnings, stall detection, and automatic recovery.

## Hardware Components
- ESP32 development board
- 2 A4988 stepper motor drivers
- 2 bipolar stepper motors
- 2 potentiometers (simulated motor current inputs)
- 2 warning LEDs
- 1 buzzer
- Breadboard and jumper wires

## Hardware Selection
The L298N motor driver was initially considered, but it was unavailable in the selected Wokwi simulation environment. Therefore, two A4988 drivers and stepper motors were used as an alternative to demonstrate motor direction and speed control.

Two potentiometers were used instead of one because each motor requires independent current monitoring. Each potentiometer simulates the current of one motor over a range of 0–30 A. This allows independent testing of overcurrent and stall conditions.

**Note:** The potentiometers simulate current; they do not measure actual motor current. The stepper motor setup is a simulation substitute, not an exact DC motor implementation.

## Working Principle

### Differential Steering
The system calculates individual motor speeds using:

- Left speed = v − (w × width / 2)
- Right speed = v + (w × width / 2)

This allows straight movement, turning in both directions, stopping, and reverse movement.

### FreeRTOS Implementation
Two independent tasks are implemented:

- **Job A – Motor Control (100 Hz):** Executes every 10 ms. Calculates motor speeds, reads simulated current, detects overcurrent and stalls, and controls motor movement.
- **Job B – Communication (20 Hz):** Executes every 50 ms. Receives Serial Monitor commands and displays system telemetry and task execution counters.

The tasks use `vTaskDelayUntil()` for periodic scheduling. Critical sections protect shared data.

### Overcurrent and Anti-Stall Protection
- Above 15 A: Warning LED and buzzer activate.
- Above 20 A continuously for 350 ms: Stall is detected.
- Recovery: The affected motor reverses at 20% of the configured maximum step rate for 200 ms.
- After recovery: The motor returns to its normal movement command.

## Serial Commands

Enter commands in the Serial Monitor at 115200 baud.

| Command | Operation |
|---|---|
| `100,0,300` | Straight movement |
| `100,0.5,300` | Turn in one direction |
| `100,-0.5,300` | Turn in the opposite direction |
| `0,0,300` | Stop |
| `-100,0,300` | Reverse |

## Testing and Verification
The following functions are tested through Wokwi:

- Differential steering and motor direction
- Independent simulated current inputs
- Overcurrent warning and buzzer
- Stall detection after the specified duration
- Automatic reverse recovery
- Job A and Job B execution counters

## Simulation
**Wokwi Simulation Link:** Add your simulation URL here.

## Conclusion
This project demonstrates motor control, FreeRTOS task scheduling, independent simulated current monitoring, and automatic stall recovery using ESP32. It provides a software simulation of the required control and protection behaviour.
