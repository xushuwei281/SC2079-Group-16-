# Explicit enable/disable gate for perception_node's continuous TrackedTarget
# stream. Kept separate from the always-on-or-off continuous_inference param
# so tracking-rate inference only costs Pi cycles during an active
# approach/orbit maneuver, not for the whole mission.
bool enable
float32 rate_hz   # requested inference rate while enabled; 0 = node default (10.0)
