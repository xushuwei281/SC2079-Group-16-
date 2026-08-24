#include "main.h"
#include <stdint.h>

#define MOTOR_KP 250
#define MOTOR_KI 30
#define MOTOR_KD 80//50

int motor_pid_correction(float left_dist, float right_dist, int8_t left_dir, int8_t right_dir);
void reset_pid_error();
