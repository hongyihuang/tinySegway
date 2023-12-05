#include "PID.h"
#include <stdint.h>
#include <stdio.h>

#define MAX_INT 0x7FFFFFFF
#define MAX_MOTOR 300
#define MIN_MOTOR -300

void pid_init(pid_controller *pid, int32_t kp, int32_t ki, int32_t kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

// void pid_set_target(pid_controller *pid, int32_t target) {
//   pid->target = target;
// }

void pid_reset(pid_controller *pid, int32_t initial_meas) {
    pid->integral = 0;
    pid->prev_err = 0;
    pid->prev_meas = initial_meas; // might need to set to baseline value?
}

int32_t pid_update(pid_controller *pid, int32_t target, int32_t meas) {
    // Compute error
    int32_t err = target - meas;
    printf("err: %d\n", err);

    // P
    int32_t p = fixedpt_mult(pid->kp, err);

    // I
    //int32_t i = pid->integral + fixedpt_mult(pid->ki, fixedpt_mult(TIMESTEP, (pid->prev_err + err) >> 1)); // >> 1 equiv to / 2
    int32_t i = pid->integral + fixedpt_mult(pid->ki, ((pid->prev_err + err) >> 1) >> TIMESTEP);
    pid->integral = i;
    pid->prev_err = err;

    // D
    int32_t d;
    int32_t meas_diff = meas - pid->prev_meas;
    /*if (TIMESTEP > 1) {
        d = fixedpt_mult(pid->kd, meas_diff << (TIMESTEP_LOG2 * -1));
    } else {
        d = fixedpt_mult(pid->kd, meas_diff >> TIMESTEP_LOG2);
    }*/
    d = fixedpt_mult(pid->kd, meas_diff << TIMESTEP); //for 0.5 second timestep, multiply d by 2
    pid->prev_meas = meas;
    
    //subtract from prev pwm value?
    //TODO: saturation
    int32_t motor_speed = (p + i + d) >> 6;
    printf("mot1: %d\n", motor_speed);
    motor_speed = motor_speed >> 6;
    printf("mot2: %d\n", motor_speed);
    if (motor_speed > MAX_MOTOR) {
      return MAX_MOTOR;
    } else if (motor_speed < MIN_MOTOR) {
      return MIN_MOTOR;
    } else {
      return motor_speed;
    }

}

// Handle fixed point multiplication by hand
int32_t fixedpt_mult(int32_t a, int32_t b) {
    int64_t prod = a * b;
    int32_t prod_clip;
    if (prod > MAX_INT - 1) {
        prod_clip = MAX_INT;
    } else if (prod < -MAX_INT) {
        prod_clip = -MAX_INT;
    } else {
        prod_clip = prod;//(prod << 16) >> 32; // trust me i did the math
    }
    return prod_clip;
}