// Global variables for Blitz data storage
// Contains all struct instances for incoming/outgoing data

#pragma once

// ============ ROS → MCU Data ============
Velocity velocity = {0.0f, 0.0f, 0.0f};  // Initialize to zero
bool hasNewVelocity = false;

ActuatorCmd actuatorCmd = {0, 0};  // Initialize to zero
bool hasNewActuatorCmd = false;

// ============ MCU → ROS Data ============
EncoderRaw encoderRaw = {0, 0, 0, 0};
LimitSwitches limitSwitches = {false, false, false};
ModeSwitch modeSwitch = {false};
BnoReading bnoReading = {0.0f, 0.0f, 0.0f};


