#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;

    float i_sum;
    float last_err;

    float out_min;
    float out_max;
    float i_min;
    float i_max;
} PID_t;

void PID_Init(PID_t *p,
              float kp,
              float ki,
              float kd,
              float out_min,
              float out_max,
              float i_min,
              float i_max);

float PID_Step(PID_t *p, float err);

#endif
