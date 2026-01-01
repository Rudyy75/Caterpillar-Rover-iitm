// Caterpillar Rover ESP32 - Main Program
// Bridges ROS2 (Serial/Blitz) to ESP8266 (ESP-NOW)

#include "include_all.cpp"

// ============ BNO055 IMU ============
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
sensors_event_t bnoEvent;

// ============ ESP-NOW ============
esp_now_peer_info_t peerInfo;

// ESP-NOW send callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optional: handle send status
}

// ============ Mode Switch ============
volatile bool modeChanged = false;
bool isAutonomous = false;
bool bnoConnected = false;  // Track if BNO055 is connected

void IRAM_ATTR mode_switch_isr() { modeChanged = true; }

// ============ Setup ============
void setup() {
  // Serial for Blitz communication with ROS
  Serial.begin(115200);

  // I2C for BNO055
  Wire.begin(BNO_SDA_PIN, BNO_SCL_PIN);

  // Initialize BNO055
  if (!bno.begin()) {
    debug_state("BNO055 not found - skipping IMU");
    bnoConnected = false;
  } else {
    bno.setExtCrystalUse(true);
    debug_state("BNO055 OK");
    bnoConnected = true;
  }

  // Initialize encoders
  setupEncoders();
  debug_state("Encoders OK");

  // Initialize limit switches
  pinMode(LIMIT_SW_1_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SW_2_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SW_3_PIN, INPUT_PULLUP);

  // Initialize mode switch detection with PULLDOWN to prevent noise
  pinMode(MODE_SWITCH_PIN, INPUT_PULLDOWN);
  attachInterrupt(MODE_SWITCH_PIN, mode_switch_isr, CHANGE);

  // ============ ESP-NOW Setup ============
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    debug_state("ESP-NOW init failed!");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // Register ESP8266 peer
  memcpy(peerInfo.peer_addr, ESP8266_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    debug_state("Failed to add ESP8266 peer!");
  } else {
    debug_state("ESP-NOW ready");
  }

  debug_state("Caterpillar Rover Ready!");
}

// ============ Main Loop ============
void loop() {
  // 1. Receive velocity commands from ROS via Blitz
  std::vector<uint8_t> payload = receive_data();
  store_data(payload);

  // 2. Read BNO055 orientation (only if connected)
  if (bnoConnected) {
    bno.getEvent(&bnoEvent);
    bnoReading.yaw = bnoEvent.orientation.x;
    bnoReading.pitch = bnoEvent.orientation.y;
    bnoReading.roll = bnoEvent.orientation.z;
  }


  // 3. Check mode switch (READ STATE directly from ESP8266 D0)
  // D0 HIGH = autonomous mode, D0 LOW = manual mode
  // Debounce to prevent noise
  if (modeChanged) {
    static unsigned long lastChangeTime = 0;
    unsigned long now = millis();
    
    bool currentPinState = digitalRead(MODE_SWITCH_PIN);
    
    // Only accept change after debounce period (200ms)
    if (now - lastChangeTime > 200) {
      // Follow the actual pin state, don't toggle
      if (currentPinState != isAutonomous) {
        isAutonomous = currentPinState;
        lastChangeTime = now;
        
        // Notify ROS about mode change
        modeSwitch.autonomous = isAutonomous;
        send_data(pack_data<ModeSwitch>(modeSwitch, MODE_SWITCH));
        
        if (isAutonomous) {
          debug_state("AUTONOMOUS MODE");
        } else {
          debug_state("MANUAL MODE");
        }
      }
    }
    
    modeChanged = false;
  }

  // 4. In autonomous mode, forward ROS velocity + actuator to ESP8266
  if (isAutonomous && (hasNewVelocity || hasNewActuatorCmd)) {
    JoyData joyData = velocityToJoyData(velocity, actuatorCmd);
    esp_now_send(ESP8266_MAC, (uint8_t *)&joyData, sizeof(joyData));
    hasNewVelocity = false;
    hasNewActuatorCmd = false;
  }

  // 5. Spin timer callbacks (sends sensor data to ROS)
  t1.spin();
}
