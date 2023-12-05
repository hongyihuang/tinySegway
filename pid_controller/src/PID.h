#include <stdint.h>

#define TIMESTEP 1 // 0.5 second, inverted (=2), log2 (=1)
#define TIMESTEP_D -1 //can be negative!!!

typedef struct {
    //Parameters
    int32_t kp;
    int32_t ki;
    int32_t kd;

    //Integration
    int32_t integral;
    int32_t prev_err;

    //Derivative
    int32_t prev_meas;

    //Last PWM?

} pid_controller;

int32_t pid_update(pid_controller *pid, int32_t target, int32_t meas);
void pid_init(pid_controller *pid, int32_t kp, int32_t ki, int32_t kd);
void pid_reset(pid_controller *pid, int32_t initial_meas);
//void pid_set_target(pid_controller *pid, int32_t target);
int32_t fixedpt_mult(int32_t a, int32_t b);