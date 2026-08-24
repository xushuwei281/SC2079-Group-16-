#include "pidMotor.h"

//Motor PID stuff
static float motor_error = 0;	//For "P"
static float motor_error_area = 0;	//For "I"
static float motor_error_rate = 0;	// For "D"
static int motor_correction = 0;	//PID output used to correct encoder disparity between the two motor
static float motor_error_old=0, motor_error_change;	// to calculate D for PID control
static float dt;// time elapse

static uint32_t millisOld = 0;
static uint32_t millisNow = 0; // time value

int motor_pid_correction(float left_dist, float right_dist, int8_t left_dir, int8_t right_dir){
	uint32_t millisNow; // time value
	int motor_correction = 0;

	millisNow = HAL_GetTick();

    dt = (millisNow - millisOld); // time elapsed in millisecond
    millisOld = millisNow; // store the current time for next round

    motor_error = left_dist - right_dist;	//Error is left-right

    motor_error_area = motor_error_area + motor_error*dt/1000; // area under error for Ki

    motor_error_change = motor_error - motor_error_old; // change in error
    motor_error_old = motor_error; //store the error for next round
    motor_error_rate = (motor_error_change)*1000/dt; // for Kd - dt in millsecond

    motor_correction = (int)(motor_error*MOTOR_KP + motor_error_area*MOTOR_KI + motor_error_rate*MOTOR_KD);

    if(left_dir+left_dir<0)
    	motor_correction = - motor_correction;

    return (int)motor_correction;//motor_correction;

	  //motor_error = ...;	//For "P"
	  //int32_t motor_error_area = ...;	//For "I"
	  //int32_t motor_error_rate = ...;	// For "D"
	  //int32_t motor_correction = ...;
}

void reset_encoder_pid_error(){
	millisOld = HAL_GetTick();
	motor_error_area = 0;
	motor_error_old = 0;
}
