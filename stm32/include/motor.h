#include "main.h"
#include <stdint.h>

#define MOTOR_PWM_MAX 6000 //safe value!
#define MOTOR_PWM_MIN 375 //minimum speed

void motor_encoder_init(TIM_HandleTypeDef *l_enc, TIM_HandleTypeDef *r_enc);
void reset_Encoders();
float motor_getDist();
int get_left_encoder();
int get_right_encoder();
int get_test_left_encoder();
