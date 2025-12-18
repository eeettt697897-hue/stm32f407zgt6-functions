#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <stdint.h>
#include "dji_motor.h"

typedef enum {
  MOTOR_MODE_IDLE = 0,
  MOTOR_MODE_VEL,
  MOTOR_MODE_POS
} MotorMode_t;

typedef struct {
  MotorMode_t mode;

  int32_t target_rpm;
  int32_t target_deg_x100;

  DJI_Motor_t fb;

  int16_t iq_cmd;
} MotorCtrl_t;

void MotorTask_Init(MotorCtrl_t *mc);
void MotorTask_SetVel(MotorCtrl_t *mc, int32_t rpm);
void MotorTask_SetPos(MotorCtrl_t *mc, int32_t deg);
void MotorTask_1ms(MotorCtrl_t *mc);

#endif
