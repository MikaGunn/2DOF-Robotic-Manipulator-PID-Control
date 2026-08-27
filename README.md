# 2-DOF Robotic Manipulator with PID Control

A complete mechatronics project covering the design, simulation, fabrication, electronics integration, and closed-loop control of a **2-degree-of-freedom (2-DOF) planar robotic manipulator**.

The system combines mechanical CAD design, 3D-printed components, Arduino-based motor control, encoder feedback, inverse kinematics, and PID position control developed in MATLAB/Simulink.

<p align="center">
  <img src="docs/images/robot-overview.png" alt="2-DOF robotic manipulator" width="520">
</p>

## Project Overview

The manipulator was developed as part of a university robotics and mechatronics project. Its two rotational joints allow the end effector to move within a 2D workspace. A belt-and-pulley transmission was used for the second link to improve stability and reduce the need to mount an additional motor directly on the moving arm.

The project includes:

- Mechanical concept development and CAD modelling
- Forward and inverse kinematics
- Denavit-Hartenberg modelling
- 3D-printed custom components
- Arduino Mega 2560 control
- EMG30 DC motors with encoders
- L298N H-bridge motor driver
- Closed-loop PID position control
- MATLAB/Simulink control-system development
- Encoder feedback and PWM motor actuation
- Experimental PID tuning and response analysis

## System Architecture

The control system receives a desired end-effector position and converts it into target joint angles using inverse kinematics. Each joint then uses encoder feedback and PID control to reduce the difference between the desired and measured angular position.

```text
Target End-Effector Position (x, y)
              |
              v
      Inverse Kinematics
              |
        theta1, theta2
          /       \
         v         v
     PID Joint 1  PID Joint 2
         |           |
         v           v
     Motor Driver  Motor Driver
         |           |
         v           v
      DC Motor 1   DC Motor 2
         ^           ^
         |           |
      Encoder 1    Encoder 2
         \___________/
          Feedback Loop
```

<p align="center">
  <img src="docs/images/simulink-control.png" alt="Simulink control architecture" width="850">
</p>

## Mechanical Design

CAD modelling was carried out using **Onshape** and **SolidWorks**. The assembly includes two rotational joints, two links, a gripper, belt-and-pulley transmission, motors, bearings, and custom printed interfaces.

Two custom parts designed for the system include:

- **Gripper** — includes inner gripping channels and a square mounting interface for the arm.
- **Motor joint** — designed to securely connect the EMG30 motor to the first arm and transfer rotational motion.

The complete CAD assembly is included as a STEP file in the `CAD/` directory.

<p align="center">
  <img src="docs/images/cad-assembly.png" alt="CAD assembly" width="800">
</p>

## Electronics

### Main Components

| Component | Purpose |
|---|---|
| Arduino Mega 2560 | Main controller and encoder interface |
| 2 × EMG30 DC motors with encoders | Joint actuation and position feedback |
| L298N motor driver | Bidirectional motor control |
| 12 V external power supply | Motor power |
| GT2 belt and pulleys | Motion transmission to the second link |

The Arduino reads encoder signals, computes motor position error, and drives the motors through PWM and direction signals.

## PID Control

Each joint uses a feedback PID controller. The control objective is to track the target joint position while reducing overshoot, rise time, and steady-state error.

The tuned controller values used in the project were:

| Gain | Value |
|---|---:|
| Kp | 1.0 |
| Ki | 0.7 |
| Kd | 0.0 |

The Arduino implementation updates the PID calculation every **50 ms**, includes encoder interrupt handling, output limiting, and a minimum-PWM compensation to overcome motor deadband/static friction.

<p align="center">
  <img src="docs/images/pid-results.png" alt="PID response result" width="720">
</p>

## Kinematics

For a planar 2-link manipulator, the end-effector coordinates are determined from the link lengths and joint angles.

Forward kinematics:

```text
x = L1 cos(theta1) + L2 cos(theta1 + theta2)
y = L1 sin(theta1) + L2 sin(theta1 + theta2)
```

Inverse kinematics is used to calculate the two required joint angles for a desired `(x, y)` end-effector target.

## Repository Structure

```text
2DOF-Robotic-Manipulator-PID-Control/
├── CAD/
│   └── Robot_CAD.step
├── control/
│   ├── Arduino_PID_controller.ino
│   └── Simulink_PID_controller.slx
├── docs/
│   ├── PROJECT_NOTES.md
│   └── images/
│       ├── robot-overview.png
│       ├── cad-assembly.png
│       ├── simulink-control.png
│       └── pid-results.png
├── .gitignore
└── README.md
```

## Files

### `CAD/Robot_CAD.step`
Full 3D CAD assembly of the robotic manipulator in STEP format.

### `control/Simulink_PID_controller.slx`
MATLAB/Simulink model containing the kinematics, PID, motor-control, and encoder-feedback subsystems.

### `control/Arduino_PID_controller.ino`
Arduino implementation of the dual-motor PID controller. It includes:

- Encoder interrupts
- Independent PID loops for both motors
- PWM motor output
- Direction control
- Error threshold handling
- Minimum PWM compensation
- Automatic target switching for repeated motion tests
- Serial monitoring of position and target values

## Hardware Used

- Arduino Mega 2560
- 2 × EMG30 DC motors with encoders
- L298N dual H-bridge motor driver
- GT2 belt and pulley system
- Bearings and shaft components
- 3D-printed gripper and mechanical joints
- Wooden arm links and base
- 12 V motor power supply

## Software Used

- MATLAB
- Simulink
- Arduino IDE
- SolidWorks
- Onshape

## Key Engineering Challenges

The project highlighted several practical robotics challenges:

- Mechanical alignment influenced encoder accuracy.
- Belt tension affected vibration and motion quality.
- Gravity had a larger effect on the first joint because it supported the complete arm assembly.
- PID tuning required balancing response speed, overshoot, and steady-state error.
- Some Simulink-to-hardware compilation issues led to development of a standalone Arduino PID implementation as an alternative controller.

## Future Improvements

Possible extensions include:

- Dual-sided axle support to improve rigidity
- Aluminium or carbon-fibre links to reduce flex and vibration
- Servo-actuated gripper
- Improved cable management
- Trajectory planning instead of point-to-point motion
- Automatic PID tuning
- Workspace and path visualisation
- ROS 2 integration
- Joint-limit and safety protection

## Skills Demonstrated

`Robotics` · `Mechatronics` · `Control Systems` · `PID Control` · `Arduino` · `MATLAB` · `Simulink` · `CAD` · `SolidWorks` · `Onshape` · `3D Printing` · `Forward Kinematics` · `Inverse Kinematics` · `Encoder Feedback` · `PWM Motor Control`

## Portfolio Summary

Designed and developed a 2-DOF robotic manipulator integrating CAD and 3D-printed mechanical components, Arduino-controlled EMG30 motors, encoder feedback, inverse kinematics, and closed-loop PID position control using MATLAB/Simulink and Arduino.

## Note

The original university report is intentionally not included in this repository because it contains personal/student information. The repository contains only the technical project assets needed to demonstrate the work.
