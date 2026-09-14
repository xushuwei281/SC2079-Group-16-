#ifndef VELOCITY_CONTROL_H
#define VELOCITY_CONTROL_H

/* Hardware-independent controller math. All lengths are millimetres, time seconds.
 * Minimum radius is the supplied 21 cm calibration; wheelbase, track and PI
 * constants remain nominal commissioning values requiring measurement. */
#include <math.h>
#include <stdint.h>

#define VELOCITY_MAX_MM_S 350
#define VELOCITY_MAX_MRAD_S 1750
#define VELOCITY_MAX_WHEEL_MM_S 480.0f
#define VELOCITY_WATCHDOG_MS 750U
#define VELOCITY_SENSOR_TIMEOUT_MS 300U
#define VELOCITY_FRONT_STOP_CM 12U
#define VELOCITY_IR_STOP_CM 10U
#define VELOCITY_MIN_RADIUS_MM 210.0f
#define VELOCITY_WHEELBASE_MM 160.0f
#define VELOCITY_TRACK_MM 150.0f
/* Calibrated steering endpoints and center: trimmed center to 144 (+1 count right),
 * and left endpoint tightened from 101 to 94 (full lock SERVOMIN) to achieve true
 * 21 cm radius on left turns (eliminating the ~10 deg position undershoot on FL 180). */
#define VELOCITY_SERVO_CENTER 144
#define VELOCITY_SERVO_LEFT 94
#define VELOCITY_SERVO_RIGHT 206
#define VELOCITY_PWM_MAX 1330.0f
#define VELOCITY_PWM_FEEDFORWARD 2.5f
#define VELOCITY_PWM_KP 2.0f
#define VELOCITY_PWM_KI 2.0f

typedef struct {
    float integral;
    int direction;
} VelocityPI;

static inline float velocity_clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static inline int velocity_watchdog_expired(int16_t speed, uint32_t now, uint32_t received)
{
    return speed != 0 && now - received > VELOCITY_WATCHDOG_MS;
}

static inline int velocity_decode(const uint8_t packet[5], int16_t *speed, int16_t *yaw)
{
    int32_t v = (uint16_t)packet[1] | ((uint16_t)packet[2] << 8);
    int32_t w = (uint16_t)packet[3] | ((uint16_t)packet[4] << 8);
    if (v >= 32768) v -= 65536;
    if (w >= 32768) w -= 65536;
    if (packet[0] != 'V' || v < -VELOCITY_MAX_MM_S || v > VELOCITY_MAX_MM_S ||
        w < -VELOCITY_MAX_MRAD_S || w > VELOCITY_MAX_MRAD_S) return 0;
    *speed = (int16_t)v;
    *yaw = v == 0 ? 0 : (int16_t)w; /* Ackermann vehicles cannot rotate in place. */
    return 1;
}

static inline void velocity_targets(int16_t speed, int16_t yaw,
                                    float *left, float *right, uint16_t *servo)
{
    float curvature = speed == 0 ? 0.0f : (yaw * 0.001f) / speed;
    curvature = velocity_clamp(curvature, -1.0f / VELOCITY_MIN_RADIUS_MM,
                               1.0f / VELOCITY_MIN_RADIUS_MM);
    float steering = atanf(VELOCITY_WHEELBASE_MM * curvature) /
                     atanf(VELOCITY_WHEELBASE_MM / VELOCITY_MIN_RADIUS_MM);
    float endpoint = steering >= 0.0f ? VELOCITY_SERVO_LEFT : VELOCITY_SERVO_RIGHT;
    *servo = (uint16_t)(VELOCITY_SERVO_CENTER + fabsf(steering) *
                       (endpoint - VELOCITY_SERVO_CENTER) + 0.5f);
    *left = speed * (1.0f - curvature * VELOCITY_TRACK_MM * 0.5f);
    *right = speed * (1.0f + curvature * VELOCITY_TRACK_MM * 0.5f);
    /* Keep individual wheel speeds within the same commissioned speed limit. */
    float peak = fmaxf(fabsf(*left), fabsf(*right));
    if (peak > VELOCITY_MAX_WHEEL_MM_S) {
        *left *= VELOCITY_MAX_WHEEL_MM_S / peak;
        *right *= VELOCITY_MAX_WHEEL_MM_S / peak;
    }
}

static inline int16_t velocity_pi(VelocityPI *state, float target, float measured, float dt)
{
    int direction = target > 0.0f ? 1 : (target < 0.0f ? -1 : 0);
    if (!direction || direction != state->direction) state->integral = 0.0f;
    state->direction = direction;
    if (!direction) return 0;
    float error = fabsf(target) - measured * direction;
    float proposed = velocity_clamp(state->integral + error * dt, -200.0f, 400.0f);
    float output = fabsf(target) * VELOCITY_PWM_FEEDFORWARD + error * VELOCITY_PWM_KP +
                   proposed * VELOCITY_PWM_KI;
    if ((output >= 0.0f && output <= VELOCITY_PWM_MAX) ||
        (output > VELOCITY_PWM_MAX && error < 0.0f) || (output < 0.0f && error > 0.0f)) {
        state->integral = proposed;
    }
    return (int16_t)(direction * velocity_clamp(output, 0.0f, VELOCITY_PWM_MAX));
}

/* The validity timestamps are sensor-task heartbeats. A zero range is the wire
 * protocol's no-return/out-of-range value, so only positive ranges are close. */
static inline int velocity_safety(int forward, uint32_t now, uint32_t us_tick,
                                  uint32_t ir_tick, int us_valid, int ir_valid,
                                  uint16_t us, uint16_t ir1, uint16_t ir2)
{
    if (!forward) return 0;
    if (!us_valid || !ir_valid || now - us_tick > VELOCITY_SENSOR_TIMEOUT_MS ||
        now - ir_tick > VELOCITY_SENSOR_TIMEOUT_MS) return 2;
    if ((us > 0 && us <= VELOCITY_FRONT_STOP_CM) ||
        (ir1 > 0 && ir1 <= VELOCITY_IR_STOP_CM) ||
        (ir2 > 0 && ir2 <= VELOCITY_IR_STOP_CM)) return 1;
    return 0;
}

#endif
