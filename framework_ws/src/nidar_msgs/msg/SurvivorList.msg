# The swarm's current, deduplicated belief about where survivors are.
#
# Published whole rather than incrementally: the list is small (single-digit
# survivors), consumers want the complete picture to render, and a full
# snapshot means a GCS that connects mid-mission is immediately correct
# instead of waiting for each survivor to be re-detected.
std_msgs/Header header
SurvivorTag[] survivors
