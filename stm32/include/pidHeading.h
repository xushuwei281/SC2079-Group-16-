#include "main.h"
#include <stdint.h>

#define HEADING_KP 3
#define HEADING_KI 3
#define HEADING_KD 5

int heading_pid_correction(float desired_heading, float current_heading, int8_t left_dir, int8_t right_dir);
void reset_heading_pid_error();
