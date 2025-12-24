# Caterpillar Rover – IIT Madras Competition

This repository contains the complete **ROS 2–based autonomous rover software stack** developed for participation in the **Caterpillar Rover Competition held at IIT Madras (IITM)**.

The project focuses on building a **robust, modular, and competition-ready ground rover** capable of reliable motion control, odometry feedback, mapping, and intelligent decision-making.

---

## Project Overview

The Caterpillar Rover is designed with a **layered robotics architecture**, separating low-level hardware control from high-level autonomy. The system is built to be scalable, debuggable, and extensible for real-world robotics challenges.

Core objectives:
- Stable rover motion using velocity control
- Accurate odometry feedback using wheel encoders
- Seamless ROS 2 integration with embedded hardware
- Readiness for SLAM and ML-based perception pipelines

---

## Key Features

- **ROS 2–based modular node architecture**
- **BlitzMCU integration** for low-level motor control and encoder feedback
- Standard ROS interfaces:
  - `/cmd_vel` – velocity commands
  - `/odom` – odometry feedback
- Real-time wheel encoder feedback loop
- Designed for integration with **SLAM Toolbox** for mapping and localization
- Extendable pipeline for **machine learning–based perception**, publishing:
  - Confidence scores
  - Detected object positions
- Clean separation between:
  - Hardware layer
  - Control layer
  - Autonomy layer

---

## System Architecture

### Low-Level Control
- Motor actuation via blitz
- Encoder data acquisition
- Hardware-to-ROS communication bridge

### Mid-Level Control
- Velocity command handling
- Odometry computation and publishing
- State and feedback management

### High-Level Autonomy
- SLAM Toolbox for localization and mapping
- ML-based perception node (planned)
- Decision-making based on perception confidence and spatial data

---

