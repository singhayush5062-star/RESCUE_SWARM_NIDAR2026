uint8 PENDING=0
uint8 EN_ROUTE=1
uint8 DROPPED=2
uint8 CONFIRMED=3

std_msgs/Header header
uint32 survivor_id
string assigned_drone_id
uint8 status
float32 drop_distance_m
