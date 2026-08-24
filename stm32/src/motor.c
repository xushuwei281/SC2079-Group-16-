#include "motor.h"

//timers
static TIM_HandleTypeDef *l_enc_tim, *r_enc_tim;

// motor data
static int16_t lLastCount = 0, rLastCount = 0;

void motor_encoder_init(TIM_HandleTypeDef *l_enc, TIM_HandleTypeDef *r_enc) {
	//assign timer pointers.
//	motor_pwm_tim = pwm;
	l_enc_tim = l_enc;
	r_enc_tim = r_enc;

	//start Encoders and PWM for L, R motors.
	HAL_TIM_Encoder_Start_IT(l_enc, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(r_enc, TIM_CHANNEL_ALL);
//	HAL_TIM_PWM_Start(pwm, L_CHANNEL);
//	HAL_TIM_PWM_Start(pwm, R_CHANNEL);
}

static void timer_reset(TIM_HandleTypeDef *htim) {
	__HAL_TIM_SET_COUNTER(htim, 0);
}

void reset_Encoders() {
	timer_reset(l_enc_tim);
	timer_reset(r_enc_tim);

//	lLastCount = rLastCount = 0;
}

static int getCount(TIM_HandleTypeDef *enc_tim) {
	int16_t  counter = __HAL_TIM_GET_COUNTER(enc_tim);
	return (int) counter;
}

float motor_getDist() {//not in use
//	int lCount = getCount(l_enc_tim),
//			rCount = getCount(r_enc_tim);
	int16_t lCount = __HAL_TIM_GET_COUNTER(l_enc_tim);

	//left motor is opposite in direction to right motor.
	lCount = -lCount;

	int l_count = lCount;
//	int pulses = (lCount + rCount) / 2; //average.
//	if (pulses < 0) pulses = -pulses; //flip.

	return 1.1;//get_dist_cm(l_count);
}

int get_left_encoder() {
	int16_t  counter = __HAL_TIM_GET_COUNTER(l_enc_tim);
	return (int) -counter;
}

int get_test_left_encoder() {
	uint16_t  counter = __HAL_TIM_GET_COUNTER(l_enc_tim);
	return (int) counter;
}

int get_right_encoder(){
	int16_t  counter = __HAL_TIM_GET_COUNTER(r_enc_tim);
	return (int) counter;
}
