# Best-priority live detection for the current camera frame, published while
# perception's tracking mode is enabled (see TrackingControl.msg). Consumed by
# bullseye_orbit_node as a continuous visual-servo error signal.
std_msgs/Header header
bool valid              # false = no qualifying detection this frame
bool is_marker          # bullseye/marker sentinel vs a real numbered symbol
int32 symbol_id         # 0 sentinel for marker, else 11-40
string symbol_name
float32 confidence
float32 cx_norm         # bbox center x, normalized [-1, 1], 0 = frame center
float32 cy_norm         # bbox center y, normalized [-1, 1], 0 = frame center
float32 width_norm      # bbox width / frame width -- range proxy
float32 height_norm     # bbox height / frame height
