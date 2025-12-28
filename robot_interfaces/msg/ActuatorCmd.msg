# Actuator command for lead screw and tub angle motors
# Used for autonomous excavation control

# Lead screw (right_flag on ESP8266)
# 0 = Stop, 1 = Up (raise scoop), 2 = Down (lower scoop)
uint8 lead_screw

# Tub angle (left_flag on ESP8266)
# 0 = Stop, 1 = Rotate CW (scoop inward), 2 = Rotate CCW (scoop outward)
uint8 tub_angle
