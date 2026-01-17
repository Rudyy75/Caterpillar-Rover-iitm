# Caterpillar Rover - System Overview

> **Robocon 2026 Sand Collection Robot**  
> ROS2 + ESP32 + ESP-NOW Architecture

---

## Robot Features

| Component | Description |
|-----------|-------------|
| **Drive** | Tank/differential (4 motors) |
| **Lead Screw** | Vertical scoop control |
| **Tub Motor** | Sand tub rotation |
| **Scoop** | Front bulldozer blade |
| **Camera Servo** | Crater detection tilt |

---

## System Architecture

```mermaid
flowchart TB
    subgraph Controller["CONTROLLER"]
        CTL["ESP32<br/>Joystick + Buttons + Mode Switch"]
    end

    subgraph Rover["ROVER"]
        subgraph RPI["Raspberry Pi (ROS2)"]
            ML["ML Pipeline (Hailo)"]
            ODOM["Odometry"]
            VEL["Velocity Bridge"]
            MODE["Mode Manager"]
            BLITZ["Blitz Node"]
            SERVO["Camera Servo"]
        end

        subgraph ESP32["ESP32 (Sensor Hub + Bridge)"]
            ENC["Encoders (4x)"]
            BNO["BNO055 IMU"]
            LIM["Limit Switches"]
        end

        subgraph ESP8266["ESP8266 (Motor Driver)"]
            DRIVE["Drive Motors"]
            LEAD["Lead Screw"]
            TUB["Tub Angle"]
        end
    end

    CTL -->|ESP-NOW| RPI
    RPI -->|USB Serial<br/>Blitz| ESP32
    ESP32 -->|ESP-NOW| ESP8266
```

---

## ROS2 Nodes

| Node | Purpose |
|------|---------|
| `blitz_node` | Serial bridge (RPi5 optimized) |
| `odom_node` | Wheel odometry + TF (10Hz) |
| `velocity_bridge` | Nav2 → MCU with crater avoidance |
| `mode_manager` | Manual/Auto switching |
| `ml_bridge` | ML detection to ROS |
| `camera_servo_node` | Crater-tracking servo |
| `auto_mission` | Autonomous excavation loop |

---

## Key Topics

| Topic | Type | Flow |
|-------|------|------|
| `/velocity` | Velocity | ROS → MCU |
| `/encoder_raw` | EncoderRaw | MCU → ROS |
| `/bno` | BnoReading | MCU → ROS |
| `/mode_switch` | ModeSwitch | MCU → ROS |
| `/ml_pipeline` | String | ML → ROS |
| `/servo_cmd` | ServoCmd | ROS → MCU |
| `/odom` | Odometry | Internal |
| `/cmd_vel` | Twist | Nav2 → Bridge |

---

## Wiring

| Connection | GPIO |
|------------|------|
| Mode Switch | ESP32 GPIO27 |
| Encoders | GPIO32-35 |
| BNO055 (I2C) | GPIO21/22 |
| Camera Servo | GPIO17 |
