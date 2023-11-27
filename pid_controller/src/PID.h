#include <stddef.h>

#DEFINE TIMESTEP 
#DEFINE TIMESTEP_LOG2 //can be negative!!!

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

} pid_controller;

uint32_t update(pid_controller *pid, int32_t target, int32_t meas);
uint32_t reset(pid_controller *pid);