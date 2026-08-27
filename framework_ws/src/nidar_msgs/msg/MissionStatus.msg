uint8 SETUP=0
uint8 SCANNING=1
uint8 DELIVERING=2
uint8 RTL=3
uint8 COMPLETE=4
# A run that ended without finishing -- mission error, or a future
# Phase 6 operator abort. Distinct from COMPLETE so the GCS never shows a
# failed run as a successful one, and so elapsed_time_sec on a terminal
# message can be read as "time to failure" rather than "mission duration".
uint8 ABORTED=5

std_msgs/Header header
uint8 total_survivors_detected
uint8 total_deliveries_complete
# Wall-clock seconds since the operator pressed Start: live while a run is
# in progress, frozen at the final total once it ends. Owned and published
# by nidar_mission_clock.
float32 elapsed_time_sec
uint8 active_drones
uint8 phase
# True while a mission run is in progress. Distinguishes "0.0 seconds
# because nothing has run yet" from "0.0 seconds because the run just
# started", which elapsed_time_sec alone cannot express.
bool mission_running
