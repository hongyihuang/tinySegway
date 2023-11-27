#include PID.h
#include <stddef.c>

#DEFINE MAX_INT = 0x7FFFFFFF

uint32_t reset(pid_controller *pid) {
    pid->integral = 0;
    pid->prev_err = 0;
    pid->prev_meas = 0; // might need to set to baseline value?
}

uint32_t update(pid_controller *pid, uint32_t target, uint32_t meas) {
    // Compute error
    int32_t err = target - meas;

    // P
    int64_t p = fixedpt_mult(pid->kp, err);

    // I
    int32_t i = pid->integral + fixedpt_mult(pid->ki, fixedpt_mult(TIMESTEP, (pid->prev_err + err) >> 1)); // >> 1 equiv to / 2
    pid->integral = i;
    pid->prev_err = err;

    // D
    int32_t d;
    int32_t meas_diff = meas - pid->prev_meas;
    if (TIMESTEP_LOG2 < 0) {
        d = fixedpt_mult(pid->kd, meas_diff << (TIMESTEP_LOG2 * -1));
    } else {
        d = fixedpt_mult(pid->kd, meas_diff >> TIMESTEP_LOG2);
    }
    pid->prev_meas = meas;
    
    return p + i + d;

}

// Handle fixed point multiplication by hand
int32_t fixedpt_mult(int32_t a, int32_t b) {
    int64_t prod = a * b;
    int32_t prod_clip;
    if (prod > MAX_INT - 1) {
        prod_clip = MAX_INT;
    } else if (p_long < -MAX_INT) {
        prod_clip = -MAX_INT;
    } else {
        prod_clip = (prod << 16) >> 32; // trust me i did the math
    }
    return prod_clip;
}