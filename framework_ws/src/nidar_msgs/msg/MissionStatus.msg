uint8 SETUP=0
uint8 SCANNING=1
uint8 DELIVERING=2
uint8 RTL=3
uint8 COMPLETE=4

std_msgs/Header header
uint8 total_survivors_detected
uint8 total_deliveries_complete
float32 elapsed_time_sec
uint8 active_drones
uint8 phase
