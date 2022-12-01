# todo: trim and remove redundant messages, e.g vehicle status, even Odom
# header
std_msgs/Header header

# Times
time start_time  # initialized at the beginning. Could be useful for debugging.
duration duration_from_start  # could be useful for debugging
duration time_change  # dT, change in time

# front and rear tire (steer) angles
float32 front_wheel_angle  # delta in rads [-0.5, 0.5]
float32 front_wheel_angle_deg  # degrees [-30, 30]
float32 rear_wheel_angle  # rads [-0.5, 0.5]
float32 rear_wheel_angle_deg  # rads [-30, 30]

# Curvature
float32 curvature

# Speeds
float32 longitudinal_velocity  # m/s
float32 longitudinal_velocity_kmph  # km/h
float32 longitudinal_velocity_mph  # miles per hour (US)
float32 lateral_velocity  # m/s
float32 lateral_velocity_kmph  # km/h
float32 lateral_velocity_mph  # miles per hour (US)
float32 heading_rate # rads/s. Heading rate (same as yaw rate)

# Accelerations
float32 longitudinal_acceleration  # m/s^2
float32 lateral_acceleration  # m/s^2

# Vehicle Actuator States
float32 steering_wheel_angle  # rads [-9.3, 10.3]
float32 steering_wheel_angle_deg  # degrees [-530, 530]
float32 accelerator_pedal # [0, 100]
float32 brake_pedal # [0, 100]
uint8 gear_state
bool parking_brake

#uint8 NONE=0
#uint8 PARK=1
#uint8 REVERSE=2
#uint8 NEUTRAL=3
#uint8 DRIVE=4
#uint8 LOW=5

# Pose and Twist (redundant)
# In order,the parameters are: (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
geometry_msgs/PoseWithCovariance location
geometry_msgs/TwistWithCovariance speeds

# Odom (Redundant)
nav_msgs/Odometry odometry

# Autoware Vehicle Status
autoware_msgs/VehicleStatus vehicle_status

# wheel speeds (fr, fl, rr, rl)
float32 front_right
float32 front_left
float32 rear_left
float32 rear_right

# travel direction
int8 travel_direction

# Vehicle Location, e.g. coordinates, waypoint index
