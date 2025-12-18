#include "pid.h"

static float clamp_f(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void PID_Init(PID_t *p,
              float kp,
              float ki,
              float kd,
              float out_min,
              float out_max,
              float i_min,
              float i_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;

    p->i_sum = 0.0f;
    p->last_err = 0.0f;

    p->out_min = out_min;
    p->out_max = out_max;
    p->i_min   = i_min;
    p->i_max   = i_max;
}

float PID_Step(PID_t *p, float err)
{
    float d = err - p->last_err;
    p->last_err = err;

    p->i_sum += p->ki * err;
    p->i_sum = clamp_f(p->i_sum, p->i_min, p->i_max);

    float out = p->kp * err + p->i_sum + p->kd * d;
    out = clamp_f(out, p->out_min, p->out_max);

    return out;
}
