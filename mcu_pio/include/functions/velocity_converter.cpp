// Velocity to JoyData converter
// Converts ROS velocity commands to joystick format for ESP8266

#pragma once
#include "pinmap/caterpillar_pins.h"

// JoyData struct (must match ESP8266 receiver)
typedef struct {
  int16_t x;
  int16_t y;
  uint8_t right_flag;
  uint8_t left_flag;
  uint8_t limit_flag;
} JoyData;

// Convert ROS Velocity + ActuatorCmd to JoyData for ESP8266
JoyData velocityToJoyData(const Velocity &vel, const ActuatorCmd &act) {
  // Explicitly initialize ALL fields to zero
  JoyData data;
  memset(&data, 0, sizeof(JoyData));

  // For differential drive:
  // - Y axis controls forward/backward (from vy)
  // - X axis controls turning (from vw)
  // - vx is ignored (differential drive can't strafe)

  // JOYSTICK MAPPING:
  // ESP8266 Code: left = y + x, right = y - x
  // Positive Y = forward (both motors same direction)
  // Positive X = turn right (left > right)
  //
  // NOTE: X and Y are swapped due to ESP-NOW struct interpretation
  // NOTE: Negated vy because positive should go forward

  data.x = (int16_t)constrain(-vel.vy * VY_TO_JOY_SCALE, -512.0f, 512.0f);  // Forward/back
  data.y = (int16_t)constrain(vel.vw * VW_TO_JOY_SCALE, -512.0f, 512.0f);   // Turning

  // Apply actuator commands from ROS
  data.right_flag = act.lead_screw;
  data.left_flag = act.tub_angle;
  
  // Keep limit_flag = 1 to maintain autonomous mode
  data.limit_flag = 1;

  return data;
}

