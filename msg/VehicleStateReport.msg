# Autoware.Auto calls this state report
# todo: might remove as it is very similar to VehicleStatus, except for fuel, hand_brake, horn
# Source: https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_vehicle_msgs/msg/VehicleStateReport.idl

Header header

uint8 fuel
uint8 blinker
uint8 headlight
uint8 wiper
uint8 gear
uint8 mode
bool hand_brake
bool horn

# Declare constants
uint8 BLINKER_OFF = 1
uint8 BLINKER_LEFT = 2
uint8 BLINKER_RIGHT = 3
uint8 BLINKER_HAZARD = 4
uint8 HEADLIGHT_OFF = 1
uint8 HEADLIGHT_ON = 2
uint8 HEADLIGHT_HIGH = 3
uint8 WIPER_OFF = 1
uint8 WIPER_LOW = 2
uint8 WIPER_HIGH = 3
uint8 WIPER_CLEAN = 14  # Match WipersCommand::ENABLE_CLEAN
uint8 GEAR_DRIVE = 1
uint8 GEAR_REVERSE = 2
uint8 GEAR_PARK = 3
uint8 GEAR_LOW = 4
uint8 GEAR_NEUTRAL = 5
uint8 MODE_AUTONOMOUS = 1
uint8 MODE_MANUAL = 2
uint8 MODE_DISENGAGED = 3
uint8 MODE_NOT_READY = 4
