/*
 * pidMotor.c
 *
 *  Created on: Aug 27, 2026
 *      Author: liuyanzhi
 */


#include "pidMotor.h"

static float motor_error = 0, motor_error_area = 0, motor_error_rate = 0;
static float motor_error_old = 0, motor_error_change;
static int   motor_correction = 0;
static float dt;
static uint32_t millisOld = 0, millisNow = 0;

int motor_pid_correction(float left_cm, float right_cm)
{
    millisNow = HAL_GetTick();
    dt = (millisNow - millisOld);
    millisOld = millisNow;
    if (dt <= 0) return motor_correction;

    motor_error = left_cm - right_cm;          /* +ve = left ran further */

    motor_error_area += motor_error * dt / 1000;
    if (motor_error_area >  MOTOR_MAXAREA) motor_error_area =  MOTOR_MAXAREA;
    if (motor_error_area < -MOTOR_MAXAREA) motor_error_area = -MOTOR_MAXAREA;

    motor_error_change = motor_error - motor_error_old;
    motor_error_old    = motor_error;
    motor_error_rate   = motor_error_change * 1000 / dt;

    motor_correction = (int)(motor_error * MOTOR_KP
                           + motor_error_area * MOTOR_KI
                           + motor_error_rate * MOTOR_KD);
    return motor_correction;
}

void reset_motor_pid_error(void)
{
    millisOld = HAL_GetTick();
    motor_error_area = 0;
    motor_error_old  = 0;
}
