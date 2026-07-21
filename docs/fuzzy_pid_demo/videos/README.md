# Fuzzy PID Controller – Real Flight Demo on PX4

This folder contains demonstration materials for the real-flight testing
of a Fuzzy PID attitude controller implemented in the PX4
`mc_att_control` module.

## Hardware Platform

- MicoAir H743 V2 flight controller
- STM32H743 microcontroller
- MicoAir 4-in-1 ESC 55A
- 7-inch quadrotor frame
- PX4 Autopilot
- QGroundControl

## Controller Implementation

The Fuzzy PID controller is integrated into the PX4 multicopter
attitude-control module.

During operation, the fuzzy inference system adjusts the PID parameters
according to the attitude error and the rate of change of the error.

The implementation is designed to improve the attitude response and
adaptability of the quadrotor under real operating conditions.

## Real Flight Test Video


The video demonstrates the quadrotor operating with the Fuzzy PID
attitude controller on real hardware.

## Main Features

- Fuzzy-based online adjustment of PID parameters
- Integration with the PX4 `mc_att_control` module
- Deployment on the STM32H743 flight controller
- Parameter configuration through QGroundControl
- Real-flight validation on a 7-inch quadrotor
- Evaluation of attitude stability and controller response

## Project Repository

The source code is available in the `feature/fuzzy-PID` branch of this
repository.
