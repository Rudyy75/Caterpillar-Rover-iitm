// Caterpillar Rover Pin Configuration
// MODIFY THESE VALUES FOR YOUR HARDWARE SETUP

#pragma once
// #include <Arduino.h>

// ============ ESP8266 MAC Address ============
// Replace with your ESP8266's MAC address
const uint8_t ESP8266_MAC[] = {0x84, 0xCC, 0xA8, 0x9E, 0x85, 0x8A};

// ============ Hall-Effect Encoder Pins ============
// Connect encoder signal pins to these GPIOs
// Use interrupt-capable pins (most ESP32 GPIOs support interrupts)
#define ENC_FL_PIN GPIO_NUM_34 // Front-left wheel encoder
#define ENC_FR_PIN GPIO_NUM_35 // Front-right wheel encoder
#define ENC_BL_PIN GPIO_NUM_32 // Back-left wheel encoder
#define ENC_BR_PIN GPIO_NUM_33 // Back-right wheel encoder

// ============ Encoder Configuration ============
// Get these values from your motor/encoder datasheet
#define PULSES_PER_REV 12    // Hall sensor pulses per motor shaft revolution
#define GEAR_RATIO 30        // Motor gearbox ratio (1 if direct drive)
#define WHEEL_DIAMETER_M 0.1 // Wheel diameter in meters
#define WHEEL_BASE_M 0.3     // Distance between left and right wheels (meters)

// Derived constants (don't modify)
#define TICKS_PER_WHEEL_REV (PULSES_PER_REV * GEAR_RATIO)
#define METERS_PER_TICK (3.14159265 * WHEEL_DIAMETER_M / TICKS_PER_WHEEL_REV)

// ============ Mode Switch Detection ============
// Connect ESP8266 D0 pin to this GPIO
#define MODE_SWITCH_PIN GPIO_NUM_27

// ============ Limit Switches (Rover Actions) ============
// Connect limit switches for action feedback
#define LIMIT_SW_1_PIN GPIO_NUM_25
#define LIMIT_SW_2_PIN GPIO_NUM_26
#define LIMIT_SW_3_PIN GPIO_NUM_14

// ============ BNO055 I2C ============
// Default I2C pins on ESP32: SDA=21, SCL=22
#define BNO_SDA_PIN 21
#define BNO_SCL_PIN 22

// ============ Velocity to JoyData Conversion ============
// Scale factors for converting ROS velocity to joystick range
#define VY_TO_JOY_SCALE 512.0f // Maps vy (m/s) to joystick Y (-512 to 512)
#define VW_TO_JOY_SCALE 512.0f // Maps vw (rad/s) to joystick X (-512 to 512)
