std_msgs/Header header
uint32 detection_id
float32 confidence
float32 bbox_x
float32 bbox_y
float32 bbox_w
float32 bbox_h
string drone_id
# ByteTrack identity, stable across frames for the same physical person as
# seen by this drone. -1 means the tracker did not assign an id to this
# detection (a fresh, not-yet-confirmed track).
#
# detection_id counts *observations* and increments every frame a person is
# visible, so counting it double-counts one survivor many times over. track_id
# is what identifies the person: count distinct track_ids, not detections.
# Scope is one drone -- two drones seeing the same survivor produce two
# unrelated track_ids, and reconciling those needs geographic positions, which
# is Phase 3's survivor aggregator.
int32 track_id
