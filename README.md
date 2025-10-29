# 3D-Printer-Bed-Control-System


## Project Overview

A comprehensive control system for a dual-axis 3D printer bed using **Arduino Mega**, featuring DC motor control with **PID**, encoder feedback, and a user interface.

---

## Hardware Architecture

### Microcontroller

**Arduino Mega 2560** — Main control unit

#### Interrupt Pins Allocation

* **INT0** – Emergency STOP button
* **INT1, INT2** – End limit switches (both axes)
* **INT3, INT4** – Encoder channel A inputs (both motors)

---

### Motor Control System

* **DC Motors:** 12V nominal, 40:1 gear reduction
* **Motor Drivers:** L293D H-bridge modules
* **Position Feedback:** Magnetic encoders (9 lines per revolution)
* **Mechanical Transmission:** Ballscrew with 1mm pitch, 500mm travel length

---

### User Interface Components

* **Display:** 16x4 LCD with PCF8574 I2C backpack
* **Input:** 4x4 matrix keypad
* **Safety:** Limit switches (start/end for each axis) + emergency stop

---

## Software Implementation

### Core Control Algorithms

#### PID Motor Control

```cpp
// PID Controller Structure
struct PIDController {
  double Kp, Ki, Kd;
  double integral;
  double previous_error;
  long last_time;
  
  // Control parameters tuned for:
  // - 40:1 gear reduction
  // - Ballscrew mechanical system
  // - Real-time position control
};
```

---

### Encoder Processing

* **Encoder Resolution:** 9 pulses per revolution (magnetic lines)
* **Gear Reduction:** 40:1 increases effective resolution
* **Ballscrew Conversion:** 1mm linear movement per revolution
* **Total Pulses Calculation:** Encoder pulses processed through interrupt service routines

---

### User Authentication System

#### Password Management

* **Storage:** EEPROM with checksum validation
* **Security:** Default password with change capability
* **Validation:** Real-time input checking with visual feedback
* **Error Handling:** Multiple failed attempt management

---

### Keypad Input Handling

```cpp
// Keypad Matrix Configuration
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
```

---

### Display Management

#### LCD Control via PCF8574

* **Interface:** I2C communication protocol
* **Backpack:** PCF8574 GPIO expander

**Display Modes:**

* Authentication screen
* Manual control interface
* Automatic coordinate input
* System status and errors
* Real-time position feedback

#### Screen Layout Strategy

* **Line 1:** System mode and status
* **Line 2:** Current operation feedback
* **Line 3:** Position coordinates (X, Y)
* **Line 4:** User prompts and input display

---

## Operation Modes

### Manual Control Mode

* Axis selection: Individual or coordinated movement
* Speed control: Adjustable velocity parameters
* Homing routine: Automatic zero position calibration
* Jog control: Fine-positioning adjustments

### Automatic Control Mode

* Coordinate input: Absolute position targeting
* Trajectory planning: Smooth motion interpolation
* Collision avoidance: Limit switch boundary protection
* Real-time tracking: Continuous position monitoring

---

## Safety Systems

### Limit Switch Handling

* **Start Limits:** Regular digital input pins
* **End Limits:** Interrupt-driven emergency stops
* **Debouncing:** Software filtering for reliable detection
* **Recovery Procedures:** Safe homing after limit activation

### Emergency Stop System

* **Hardware Interrupt:** INT0 immediate response
* **Motor Braking:** Fast deceleration control
* **System Lock:** Requires manual reset after activation
* **Status Indication:** Visual and audible alerts

---

## Memory Management

### EEPROM Utilization

* Password storage: Encrypted format with verification
* System parameters: Calibration data persistence
* User preferences: Mode settings retention
* Error logging: Diagnostic information storage

### SRAM Optimization

* String constants: `PROGMEM` storage for text
* Buffer management: Efficient memory allocation
* Variable sizing: Appropriate data types selection

---

## Mathematical Calculations

### Position Control

```
Encoder pulses per revolution = 9 lines × 4 edges = 36 pulses
Effective pulses after gearbox = 36 × 40 = 1440 pulses/revolution
Linear movement per revolution = 1mm (ballscrew pitch)
Pulses per mm = 1440 pulses/mm
Pulses for 30cm movement = 1440 × 300 = 432,000 pulses
```

### PID Tuning Parameters

* **Proportional (Kp):** Position error correction
* **Integral (Ki):** Steady-state error elimination
* **Derivative (Kd):** Overshoot and oscillation damping

---

## Communication Protocols

### I2C Implementation

* **LCD Display:** PCF8574 at configurable address
* **Clock Speed:** Standard 100kHz or Fast Mode 400kHz
* **Error Handling:** Bus collision recovery routines

### Serial Monitoring

* **Debug Output:** Real-time system status
* **Parameter Tuning:** Runtime adjustment capability
* **Error Reporting:** Detailed diagnostic information

---


## Dependencies

* **Keypad Library:** Matrix keypad input handling
* **LiquidCrystal_I2C:** PCF8574 LCD control
* **EEPROM Library:** Persistent storage management
* **Timer Libraries:** Real-time control timing

---

## Simulation Environment

* **Proteus 8 Professional:** Circuit simulation and validation
* **Component Models:** L293D, Arduino Mega, LCD, encoders
* **Real-time Debugging:** Virtual instrument monitoring

---

**This system represents a complete embedded solution for precise motion control with comprehensive user interface and safety features.**
