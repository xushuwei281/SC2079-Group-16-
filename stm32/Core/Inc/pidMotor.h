/*
 * pidmotor.h
 *
 *  Created on: Aug 27, 2026
 *      Author: liuyanzhi
 */

#ifndef INC_PIDMOTOR_H_
#define INC_PIDMOTOR_H_

#include "main.h"
#include <stdint.h>

#define MOTOR_KP    67.0f   /* 300 -> 300*1600/7200 */
#define MOTOR_KI    11.0f   /* 50                    */
#define MOTOR_KD     0.0f   /* they set D to 0 - keep it 0 */
#define MOTOR_MAXAREA 10.0f

int  motor_pid_correction(float left_cm, float right_cm);
void reset_motor_pid_error(void);

#endif /* INC_PIDMOTOR_H_ */
