#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "main.h"
#include "ICM20948.h"
#include <math.h>
//#include "dist.h"

#define ICM_I2C_ADDR 0
#define GRAVITY 9.80665e-4f //in cm/ms^2
typedef struct {
//	float irDist[2];		//distance from IR sensor, [L, R].
//	volatile float usDist;	//distance from ultrasound sensor.

	float gyroZ;			//gyroscope Z reading.
	float accel[3];			//accelerometer [X, Y, Z] readings.
	float heading;			//heading from -180 to 180 degrees.

	float gyroZ_bias;
	float accel_bias[3];
	float heading_bias;
} Sensors;

void sensors_init(I2C_HandleTypeDef *hi2c1_ptr, Sensors *sensors_ptr);
//void sensors_read_irDist();
void sensors_us_trig();
//void sensors_read_usDist(float pulse_s);
void sensors_read_gyroZ();
void sensors_read_accel();
void sensors_read_heading(float msElapsed, float gyroZ);
void sensors_set_bias(uint16_t count);

#endif /* INC_SENSORS_H_ */
