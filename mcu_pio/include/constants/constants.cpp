// Global variables for Blitz data storage
// Contains all struct instances for incoming/outgoing data

#pragma once

// ============ ROS → MCU Data ============
Velocity velocity;           // Incoming velocity commands from ROS
bool hasNewVelocity = false; // Flag to indicate new velocity received

// ============ MCU → ROS Data ============
EncoderRaw encoderRaw;       // Encoder tick counts
LimitSwitches limitSwitches; // Limit switch states
ModeSwitch modeSwitch;       // Mode switch state
BnoReading bnoReading;       // BNO055 IMU data
