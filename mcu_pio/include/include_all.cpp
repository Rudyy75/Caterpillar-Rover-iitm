// Master include file for Caterpillar Rover ESP32
// Include order matters!

// Arduino and standard libraries
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <vector>

// Blitz communication library
#include "blitz.hpp"
#include "blitz_timer.cpp"

// Pin configuration
#include "pinmap/caterpillar_pins.h"

// Blitz data structures (must come after blitz.hpp)
#include "constants/constants.cpp"

// Functions
#include "functions/encoder_reader.cpp"
#include "functions/store_data.cpp"
#include "functions/timer_cb.cpp"
#include "functions/velocity_converter.cpp"
