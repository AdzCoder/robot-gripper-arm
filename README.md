# Robot Gripper Arm

[![Arduino](https://img.shields.io/badge/Arduino-Uno-blue?style=flat-square)](https://www.arduino.cc/en/hardware/uno)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-CI-orange?style=flat-square)](https://platformio.org/)
[![Licence](https://img.shields.io/badge/Licence-MIT-orange?style=flat-square)](LICENSE)
[![University](https://img.shields.io/badge/University-Warwick-green?style=flat-square)](https://warwick.ac.uk/)
[![Status](https://img.shields.io/badge/Status-Educational-lightgrey?style=flat-square)](https://github.com/topics/education)

Arduino firmware for an assistive gripper arm that grips objects automatically using sensor feedback, with joystick control and grip force held by a PID current loop. Built for a second-year group design project at Warwick; this repository holds the control code, refactored from the original coursework sketch into a structured PlatformIO project.

## Background

This was the electronics and software half of the ES2C6 Electromechanical System Design project (2023/24), a six-person brief to design, build, and test an assistive device. The target was an intuitive gripper to help people with limited mobility handle everyday objects: anything from 10 g to 500 g and from 15×10×10 mm to 60×80×150 mm, gripping on demand and releasing fully when switched off. The gripper reads distance to decide when to close and motor current to decide how hard, so the user drives it with a single joystick rather than managing grip strength by hand. The mechanical design, enclosure, and full process report were the wider group deliverable; what lives here is the firmware that runs the device.

## How it works

The controller runs a fixed loop: read sensors, decide gripper state, drive the motors. When the ultrasonic sensor detects an object within range and the joystick is not pulled back, the gripper closes; a PID loop then holds the motor current (a proxy for grip force) at a setpoint so delicate objects are not crushed. With no object present it loosens on a timed release and returns the servo to its neutral angle. The joystick button toggles the whole system on and off.

## Hardware

| Component | Model | Purpose |
|-----------|-------|---------|
| Microcontroller | Arduino Uno R3 | Main controller |
| Current sensor | INA219 | Motor current, used as grip-force feedback |
| Distance sensor | HC-SR04 ultrasonic | Object detection |
| Force sensor | Force-sensing resistor (FSR) | Direct grip-pressure reading |
| Force signal conditioning | OP177 precision op-amp | Buffers the FSR divider into the analogue input |
| Drive motor | DC geared motor | Gripping mechanism |
| Positioning motor | Servo | Gripper rotation |
| Input | KY-023 joystick module | User control |

- **Power:** 12 V, 1 A (12 W) plug-in adapter
- **Rated payload:** 10–500 g
- **Object envelope:** 15×10×10 mm to 60×80×150 mm

![Circuit diagram](docs/RobotHandCircuit.png)

## Build and upload

Requires [PlatformIO](https://platformio.org/). Dependencies are declared in `platformio.ini` and fetched automatically.

```bash
git clone https://github.com/AdzCoder/robot-gripper-arm.git
cd robot-gripper-arm
pio run                    # build
pio run --target upload    # flash to the Uno
pio device monitor         # optional serial output
```

| Library | Version | Licence | Purpose |
|---------|---------|---------|---------|
| [Adafruit INA219](https://github.com/adafruit/Adafruit_INA219) | ^1.2.3 | MIT | Current sensing |
| [movingAvg](https://github.com/JChristensen/movingAvg) | ^2.3.1 | GPL-3.0 | Signal smoothing |
| [Servo](https://github.com/arduino-libraries/Servo) | ^1.2.1 | LGPL-2.1 | Servo control |

## Operation

Power on and wait for the initialisation LED. Press the joystick button to arm the system. Use the joystick to rotate the gripper; it closes automatically when an object is detected and holds a regulated grip. Pull the joystick back to override and release. Press the button again to disarm, which stops the motors.

## Repository layout

- `src/main.cpp`: controller firmware
- `data/`: captured PID and current-filter test logs, described in `data/README.md`
- `docs/RobotHandCircuit.png`: wiring diagram
- `platformio.ini`: board and dependency configuration

## Academic context

**Module:** [ES2C6 Electromechanical System Design (2023/24)](https://courses.warwick.ac.uk/modules/2023/ES2C6-15) · **Team:** Group J4, six students · **Institution:** University of Warwick, School of Engineering

The project is complete and the firmware is provided as an educational reference. Feel free to fork and adapt it.

## Acknowledgements

Firmware review by the [OmniLink](https://github.com/omnilink-tech/omnisim) team, September 2026.

## Licence

MIT Licence: see the [LICENCE](LICENSE) file for details.

---

*Developed by Adil Wahab Bhatti as part of academic coursework at the University of Warwick.*
