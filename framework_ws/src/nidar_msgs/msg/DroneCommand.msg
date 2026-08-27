# Manual drone control command from GCS, relayed by nidar_gcs_bridge.
string drone_id       # "drone0" or "all"
string action         # "arm", "disarm", "takeoff", "set_launch_position"
float32 altitude_m    # For takeoff
float64 latitude      # For set_launch_position (NaN = not specified)
float64 longitude     # For set_launch_position (NaN = not specified)
