# Caterpillar Rover

> **Hybrid Manual/Autonomous Bulldozer | Robocon 2026**

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
| **ROS2 Integration** | Built on Humble, ready for SLAM and Nav2 |

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
    └── [Autonomous Stack]
```

**Manual Mode**: Controller → ESP8266 (direct)  
**Autonomous Mode**: ROS2 → ESP32 → ESP8266

---

## Quick Start

```bash
# Build
cd ~/Robocon-2026/Caterpillar && colcon build

# Launch
source install/setup.bash
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
├── blitz/              # Serial communication bridge
├── robot_interfaces/   # Custom ROS2 messages
├── rover/              # Nodes and launch files
└── mcu_pio/            # ESP32 firmware (PlatformIO)
```

---

## Documentation

See [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) for detailed architecture, wiring, and motor control mapping.

---
