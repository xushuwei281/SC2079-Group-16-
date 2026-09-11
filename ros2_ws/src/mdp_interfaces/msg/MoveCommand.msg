# One compatibility movement primitive. motion_controller_node supervises it
# with live telemetry and publishes cmd_vel; callers do not encode UART packets.
#
# Supported movement codes (centimetres for distances, degrees for turns):
#   FC <dist_cm>   forward straight
#   BC <dist_cm>   backward straight
#   FL <angle_deg> forward-left turn
#   FR <angle_deg> forward-right turn
#   BL <angle_deg> backward-left turn
#   BR <angle_deg> backward-right turn
#   FU <dist_cm>   forward until ultrasound reads dist_cm
#   BU <dist_cm>   backward until ultrasound reaches or exceeds dist_cm
string command
int16 value
