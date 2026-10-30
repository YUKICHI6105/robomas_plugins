std_msgs/Header header
uint8 motor
bool c620
uint8 mode
# Velocity-limited position control (firmware MotorCtrl::setFrame case 4).
uint8 MODE_STABLEPOS=4
uint8 temp
float32 velkp
float32 velki
float32 poskp
float32 tyoku_vel_target
float32 tyoku_pos_target
float32 stable_pos_limit_vel