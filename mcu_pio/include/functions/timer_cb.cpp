// Timer callback - sends sensor data to ROS periodically
// Called by BlitzTimer at configured interval

void timer_cb() {
  // ============ Send Encoder Data ============
  encoderRaw = getEncoderTicks();
  send_data(pack_data<EncoderRaw>(encoderRaw, ENCODER_RAW));

  // ============ Send BNO055 Data ============
  // bnoReading is updated in main loop
  send_data(pack_data<BnoReading>(bnoReading, BNO_READING));

  // ============ Send Limit Switch States ============
  limitSwitches.ls1 = !digitalRead(LIMIT_SW_1_PIN); // Active low
  limitSwitches.ls2 = !digitalRead(LIMIT_SW_2_PIN);
  limitSwitches.ls3 = !digitalRead(LIMIT_SW_3_PIN);
  send_data(pack_data<LimitSwitches>(limitSwitches, LIMIT_SWITCHES));
}

// Timer instance - sends data at 20Hz (50ms interval)
BlitzTimer t1(timer_cb, 50);
