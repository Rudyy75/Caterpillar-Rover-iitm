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
  JoyData data;

  // For differential drive:
  // - Y axis controls forward/backward (from vy)
  // - X axis controls turning (from vw)
  // - vx is ignored (differential drive can't strafe)

  // JOYSTICK MAPPING ANALYSIS:
  // Code 1: Left = y + x, Right = y - x
  // If x > 0: Left > Right -> Robot turns RIGHT (CW)
  // ROS Convention: vw > 0 -> Robot turns LEFT (CCW)
  // Therefore: vw positive must map to x negative

  data.y = (int16_t)constrain(vel.vy * VY_TO_JOY_SCALE, -512.0f, 512.0f);
  data.x = (int16_t)constrain(-vel.vw * VW_TO_JOY_SCALE, -512.0f,
                              512.0f); // Note the negative sign

  // Apply actuator commands from ROS
  // right_flag: Lead screw (0=Stop, 1=Up, 2=Down)
  // left_flag: Tub angle (0=Stop, 1=CW, 2=CCW)
  data.right_flag = act.lead_screw;
  data.left_flag = act.tub_angle;
  data.limit_flag = 0; // Not used in autonomous mode

  return data;
}

