#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "velocity_control.h"

static void test_packets(void)
{
    int16_t v, w;
    const uint8_t reverse[] = {'V', 0x6a, 0xff, 0xe8, 0x03};
    assert(velocity_decode(reverse, &v, &w));
    assert(v == -150 && w == 1000);
    const uint8_t overspeed[] = {'V', 0x2d, 0x01, 0, 0};
    assert(!velocity_decode(overspeed, &v, &w));
    const uint8_t invalid_yaw[] = {'V', 100, 0, 0xb1, 0x04};
    assert(!velocity_decode(invalid_yaw, &v, &w));
    const uint8_t rotate[] = {'V', 0, 0, 0xe8, 0x03};
    assert(velocity_decode(rotate, &v, &w) && v == 0 && w == 0);
}

static void test_kinematics(void)
{
    float left, right;
    uint16_t servo;
    velocity_targets(150, 0, &left, &right, &servo);
    assert(left == 150 && right == 150 && servo == VELOCITY_SERVO_CENTER);
    velocity_targets(150, 1200, &left, &right, &servo);
    assert(servo == VELOCITY_SERVO_LEFT && left > 0 && right > left);
    float curvature = (right - left) / (0.5f * (right + left) * VELOCITY_TRACK_MM);
    assert(fabsf(curvature - 1.0f / 210.0f) < 1e-6f);
    velocity_targets(-150, 1200, &left, &right, &servo);
    assert(servo == VELOCITY_SERVO_RIGHT && left < right && right < 0);
    velocity_targets(300, -1200, &left, &right, &servo);
    assert(fabsf(left) <= 300.001f && fabsf(right) <= 300.001f);
    assert(servo > VELOCITY_SERVO_CENTER);
    velocity_targets(0, 1200, &left, &right, &servo);
    assert(left == 0 && right == 0 && servo == VELOCITY_SERVO_CENTER);
}

static void test_feedback_and_stop(void)
{
    VelocityPI pi = {0};
    int16_t stalled = velocity_pi(&pi, 150, 0, 0.01f);
    pi = (VelocityPI){0};
    int16_t tracking = velocity_pi(&pi, 150, 150, 0.01f);
    assert(stalled > tracking && tracking > 0);
    for (int i = 0; i < 10000; i++) {
        int16_t pwm = velocity_pi(&pi, 300, -1000, 0.01f);
        assert(pwm >= 0 && pwm <= VELOCITY_PWM_MAX);
    }
    assert(velocity_pi(&pi, 0, 150, 0.01f) == 0 && pi.integral == 0);
    assert(velocity_pi(&pi, -150, 0, 0.01f) < 0);
    assert(velocity_pi(&pi, 150, 0, 0.01f) > 0);
}

static void test_safety(void)
{
    assert(!velocity_watchdog_expired(150, 300, 0));
    assert(velocity_watchdog_expired(150, 301, 0));
    assert(velocity_watchdog_expired(-150, 301, 0));
    assert(!velocity_watchdog_expired(0, 100000, 0));
    assert(!velocity_watchdog_expired(150, 20, UINT32_MAX - 10));
    assert(velocity_safety(1, 100, 100, 100, 1, 1, 13, 11, 11) == 0);
    assert(velocity_safety(1, 100, 100, 100, 1, 1, 12, 11, 11) == 1);
    assert(velocity_safety(1, 100, 100, 100, 1, 1, 30, 10, 11) == 1);
    assert(velocity_safety(1, 401, 100, 401, 1, 1, 0, 20, 20) == 2);
    assert(velocity_safety(1, 401, 401, 100, 1, 1, 30, 0, 0) == 2);
    assert(velocity_safety(1, 0, 0, 0, 0, 0, 0, 0, 0) == 2);
    assert(velocity_safety(0, 401, 0, 0, 0, 0, 1, 1, 1) == 0);
    /* Tick subtraction remains correct over the 32-bit HAL counter wrap. */
    assert(velocity_safety(1, 20, UINT32_MAX - 10, UINT32_MAX - 10,
                           1, 1, 30, 20, 20) == 0);
}

int main(void)
{
    test_packets();
    test_kinematics();
    test_feedback_and_stop();
    test_safety();
    puts("velocity controller: packet, Ackermann, PI and safety tests passed");
    return 0;
}
