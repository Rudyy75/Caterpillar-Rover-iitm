// Parse incoming Blitz packets and store data
// Called every loop iteration with received payload

void store_data(std::vector<uint8_t> payload) {
  if (payload.empty())
    return;

  uint8_t id = payload[0];

  switch (id) {
  case VELOCITY:
    velocity = parse_struct<Velocity>(payload);
    hasNewVelocity = true;
    break;

    // Add more cases here if ROS sends other message types

  default:
    // Unknown packet ID - ignore
    break;
  }
}
