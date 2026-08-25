# One discrete move primitive for the STM32's 5-byte packet protocol
# (2-char command code + 3-digit ASCII value). The bridge node does the
# actual byte encoding -- callers just fill this in.
#
# Valid command codes (confirmed from the ported STM_Ref firmware's
# comm_task dispatch, raspberry-pi/../stm32/src/main.c):
#   FC <dist_cm>   forward straight
#   BC <dist_cm>   backward straight
#   FL <angle_deg> forward-left turn
#   FR <angle_deg> forward-right turn
#   BL <angle_deg> backward-left turn
#   BR <angle_deg> backward-right turn
#   FU <dist_cm>   forward until ultrasound reads dist_cm
#   BU <dist_cm>   backward until ultrasound reads dist_cm
string command
int16 value
