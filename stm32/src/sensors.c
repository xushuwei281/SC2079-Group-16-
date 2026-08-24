#include "sensors.h"

static const uint8_t GYRO_SENS = GYRO_FULL_SCALE_250DPS;
static const uint8_t ACCEL_SENS = ACCEL_FULL_SCALE_2G;
static const float a_irDist = 0.95;
static const float a_usDist = 0.58;
static const float a_accel = 0.8;
static const float a_mag = 0.9;
static float magOld[2];
static float headingRaw, headingOld;

static float lpf(float a, float old, float new) {
	return a * old + (1 - a) * new;
}

static I2C_HandleTypeDef *hi2c_ptr;

static TIM_HandleTypeDef *hic_ptr;
static Sensors *sensors_ptr;


void sensors_init(I2C_HandleTypeDef *i2c_ptr, Sensors *sens_ptr){
	hi2c_ptr = i2c_ptr;

	sensors_ptr = sens_ptr;

	ICM20948_init(hi2c_ptr, ICM_I2C_ADDR, GYRO_SENS, ACCEL_SENS);

	sens_ptr->gyroZ_bias = 0;
	sens_ptr->accel_bias[0] = sens_ptr->accel_bias[1] = sens_ptr->accel_bias[2] = 0;


}

void sensors_read_gyroZ() {
	float val;
	ICM20948_readGyroscope_Z(hi2c_ptr, ICM_I2C_ADDR, GYRO_SENS, &val);
	sensors_ptr->gyroZ = (val - sensors_ptr->gyroZ_bias); //convert to ms
}


void sensors_read_accel() {
	float accel_new[3];
	ICM20948_readAccelerometer_all(hi2c_ptr, ICM_I2C_ADDR, ACCEL_SENS, accel_new);
	for (int i = 0; i < 3; i++) {
		sensors_ptr->accel[i] = (accel_new[i] - sensors_ptr->accel_bias[i]) * GRAVITY;
	}
}

void sensors_read_heading(float msElapsed, float gyroZ) {
	sensors_ptr->heading = angle_diff_180(
		angle_get(msElapsed, gyroZ, read_mag_angle()),
		sensors_ptr->heading_bias
	);
}

void sensors_set_bias(uint16_t count) {
	uint16_t i;
	uint8_t j;
	float gyroZTotal = 0, gyroZ = 0,
		accelTotal[3] = {0}, accel[3];

	for (i = 0; i < count; i++) {
		ICM20948_readGyroscope_Z(hi2c_ptr, ICM_I2C_ADDR, GYRO_SENS, &gyroZ); //gyroscope bias
		gyroZTotal += gyroZ;

		ICM20948_readAccelerometer_all(hi2c_ptr, ICM_I2C_ADDR, ACCEL_SENS, accel); //accelerometer bias
		for (j = 0; j < 3; j++) accelTotal[j] += accel[j];
	}

	sensors_ptr->gyroZ_bias = gyroZTotal / count;

	for (i = 0; i < 3; i++) sensors_ptr->accel_bias[i] = accelTotal[i] / count;
	sensors_ptr->accel_bias[2] -= GRAVITY; //normally z accelerometer should read gravity.

}
