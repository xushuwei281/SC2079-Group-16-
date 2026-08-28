/*
 * pidHeading.h
 *
 *  Created on: Aug 27, 2026
 *      Author: liuyanzhi
 */

#ifndef INC_PIDHEADING_H_
#define INC_PIDHEADING_H_

#include "main.h"
#include <stdint.h>

/* You are on the same 10.06 us servo tick as the reference project now
   (PSC 160), so its gains apply directly - no halving needed. */
#define HEADING_KP      3.0f //3
#define HEADING_KI      3.0f
#define HEADING_KD      5.0f
#define MAXERRORAREA   10.0f    /* integral clamp - stops windup */

int  heading_pid_correction(float desired_heading, float current_heading);
void reset_heading_pid_error(void);

#endif /* INC_PIDHEADING_H_ */
