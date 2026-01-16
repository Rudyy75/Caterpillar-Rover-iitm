# Caterpillar Rover

> **Hybrid Manual/Autonomous Bulldozer**

A sand-collecting bulldozer robot with seamless switching between manual joystick control and ROS2-based autonomous operation.

---

## Features

| Feature | Description |
|---------|-------------|
| **Dual Mode Operation** | Switch between manual and autonomous with a single button |
| **Differential Drive** | Tank-style movement with 4 drive motors |
| **Bulldozer Mechanics** | Lead screw (scoop height) + rope motor (tub angle) |
| **Hall-Effect Odometry** | 4 wheel encoders for position tracking |
| **BNO055 IMU** | Accurate heading from 9-axis IMU |
| **ROS2 Integration** | Built on Jazzy, ready for SLAM and Nav2 |
|**Crater Detection (ML)** | YOLOv5s-based crater detection using Hailo HAT |
---

## Architecture

```
Controller ESP32  ──ESP-NOW──►  ESP8266 (Motors)
                                    ▲
                                    │ ESP-NOW
                                    │
RPi (ROS2)  ───USB Serial/Blitz───► ESP32 (Sensor Hub)
    │
    ├── Blitz (packer/parser)
    ├── Odom Node
    ├── Mode Manager
    ├── Crater Detection (ML - Standalone)
    └── [Autonomous Stack]

```

**Manual Mode**: Controller → ESP8266 (direct)  
**Autonomous Mode**: ROS2 → ESP32 → ESP8266

---

## Quick Start

```bash
# Build
cd {path_to_workspace} && colcon build

# Launch
source {path_to_workspace}/install/setup.bash
ros2 launch rover manual.launch.py
```

---

## ROS2 Topics

| Topic | Direction | Purpose |
|-------|-----------|---------|
| `/velocity` | ROS→MCU | Drive commands (vy, vw) |
| `/encoder_raw` | MCU→ROS | Wheel encoder ticks |
| `/bno` | MCU→ROS | IMU orientation |
| `/odom` | Internal | Computed pose |
| `/mode_switch` | MCU→ROS | Manual/Auto toggle |

---

## Project Structure

```
src/
├── blitz/                  # Serial communication bridge
├── Crater_Detection/       # YOLOv5s crater detection (Hailo inference)
├── robot_interfaces/       # Custom ROS2 messages
├── rover/                  # Nodes and launch files
└── mcu_pio/                # ESP32 firmware (PlatformIO)
```

---

## Documentation

See [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) for detailed architecture, wiring, and motor control mapping.

---
