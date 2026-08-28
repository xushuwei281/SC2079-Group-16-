#include "pidHeading.h"

// Heading PID state variables
static float heading_error = 0;            // For "P"
static float heading_error_area = 0;       // For "I"
static float heading_error_rate = 0;       // For "D"
static int heading_correction = 0;         // PID output for turning
static float heading_error_old = 0, heading_error_change; // For derivative calculation
static float dt;                       // Time elapsed (in milliseconds)

static uint32_t millisOld = 0;
static uint32_t millisNow = 0;

int heading_pid_correction(float desired_heading, float current_heading)
{
    // Get the current time
    millisNow = HAL_GetTick();

    // Calculate dt (in ms) and update the old time value
    dt = (millisNow - millisOld);
    millisOld = millisNow;

    if (dt <= 0) return heading_correction;   // guard: no divide-by-zero in D

    // Calculate error: difference between desired and current heading
    heading_error = desired_heading - current_heading;

    // area under error for Ki
    heading_error_area = heading_error_area + heading_error * dt / 1000;

    if(heading_error_area>MAXERRORAREA)
    {
        heading_error_area = MAXERRORAREA;
    }
    else if(heading_error_area<-MAXERRORAREA)
    {
        heading_error_area = -MAXERRORAREA;
    }

    heading_error_change = heading_error - heading_error_old; // change in error
    heading_error_old = heading_error; // store the error for next round
    heading_error_rate = (heading_error_change) * 100/ dt; // for Kd - dt in millsecond (100?)

    // Combine terms to compute the PID output
    heading_correction = (int)(heading_error * HEADING_KP + heading_error_area * HEADING_KI + heading_error_rate * HEADING_KD);

    return heading_correction;
}

void reset_heading_pid_error(void)
{
    millisOld = HAL_GetTick();
    heading_error_area = 0;
    heading_error_old = 0;
}
