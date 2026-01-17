// Camera Servo Controller
// Handles single tilt servo for crater detection camera
// 
// Normal mode: 80-100 degrees (center = 90)
// Flipped mode: 260-280 degrees (center = 270)
// Servo wraps at 360 degrees

#pragma once
#include <ESP32Servo.h>
#include "pinmap/caterpillar_pins.h"

// ============ Servo Pin ============
#define SERVO_TILT_PIN GPIO_NUM_17  // Camera tilt servo

// ============ Servo Object ============
Servo tiltServo;

// ============ Current State ============
uint16_t currentTiltAngle = 90;

// ============ Functions ============

void setTiltAngle(uint16_t angle) {
  // Servo range is typically 0-180 degrees
  // For angles > 180, we map to 0-180 range
  // 90 = center normal, 270 = center flipped (maps to 90 on servo, but inverted)
  
  uint16_t servoAngle;
  
  if (angle <= 180) {
    // Normal range: 0-180 maps directly
    servoAngle = angle;
  } else {
    // Flipped range: 181-360 maps to 180-0 (inverted)
    // 270 -> 90 (center but inverted)
    // 260 -> 100
    // 280 -> 80
    servoAngle = 360 - angle;
  }
  
  // Clamp to valid servo range
  if (servoAngle > 180) servoAngle = 180;
  
  tiltServo.write(servoAngle);
  currentTiltAngle = angle;
}

void setupServos() {
  // Attach servo to pin
  tiltServo.attach(SERVO_TILT_PIN);
  
  // Set to default position (90 degrees - center)
  setTiltAngle(90);
}

void applyServoCmd(const ServoCmd& cmd) {
  setTiltAngle(cmd.tilt_angle);
}

