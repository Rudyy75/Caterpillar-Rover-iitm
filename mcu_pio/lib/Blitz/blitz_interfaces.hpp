// Blitz interfaces for Caterpillar rover
// Must match interfaces.py msg_id and struct fields exactly

#pragma once
#include <stdint.h>

// ============ Packet IDs ============
enum PacketID : uint8_t {
  VELOCITY = 1,       // ROS → MCU
  ENCODER_RAW = 2,    // MCU → ROS
  LIMIT_SWITCHES = 3, // MCU → ROS
  MODE_SWITCH = 4,    // MCU → ROS
  BNO_READING = 5,    // MCU → ROS
  ACTUATOR_CMD = 6    // ROS → MCU
};

// ============ ROS → MCU Structs ============

#pragma pack(push, 1)
struct Velocity {
  float vx; // Lateral velocity (ignored for diff drive)
  float vy; // Forward velocity (m/s)
  float vw; // Angular velocity (rad/s)
};
#pragma pack(pop)

#pragma pack(push, 1)
struct ActuatorCmd {
  uint8_t lead_screw; // 0=Stop, 1=Up, 2=Down
  uint8_t tub_angle;  // 0=Stop, 1=CW, 2=CCW
};
#pragma pack(pop)

// ============ MCU → ROS Structs ============

#pragma pack(push, 1)
struct EncoderRaw {
  int32_t fl_ticks; // Front-left wheel
  int32_t fr_ticks; // Front-right wheel
  int32_t bl_ticks; // Back-left wheel
  int32_t br_ticks; // Back-right wheel
};
#pragma pack(pop)

#pragma pack(push, 1)
struct LimitSwitches {
  bool ls1;
  bool ls2;
  bool ls3;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct ModeSwitch {
  bool autonomous;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct BnoReading {
  float yaw; // Heading angle
  float pitch;
  float roll;
};
#pragma pack(pop)

// ============ Packet Size Lookup ============

size_t get_packet_size(uint8_t id) {
  switch (id) {
  case VELOCITY:
    return sizeof(Velocity);
  case ACTUATOR_CMD:
    return sizeof(ActuatorCmd);
  case ENCODER_RAW:
    return sizeof(EncoderRaw);
  case LIMIT_SWITCHES:
    return sizeof(LimitSwitches);
  case MODE_SWITCH:
    return sizeof(ModeSwitch);
  case BNO_READING:
    return sizeof(BnoReading);
  default:
    return 0;
  }
}

