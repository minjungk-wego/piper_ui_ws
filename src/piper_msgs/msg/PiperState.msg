uint8 INIT       = 0
uint8 MANUAL     = 1
uint8 MOVING     = 2
uint8 STOWED     = 3
uint8 ERROR      = 4
uint8 DEPLOYING  = 5
uint8 MEASURING  = 6
uint8 RETRACTING = 7

uint8 state
string error_detail
float32[] joint_temperatures_celsius
bool thermal_warning
