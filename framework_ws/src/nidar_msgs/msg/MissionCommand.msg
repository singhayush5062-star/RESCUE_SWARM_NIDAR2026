# Mission load/start commands from GCS, relayed by nidar_gcs_bridge.
uint8 LOAD=0
uint8 START=1

uint8 action
string mission_json   # Full mission JSON (only set for LOAD)
