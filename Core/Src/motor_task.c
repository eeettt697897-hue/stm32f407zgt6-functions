#include "motor_task.h"
#include "pid.h"

/* 速度 PID */
static PID_t pid_spd;

void MotorTask_Init(MotorCtrl_t *mc)
{
  mc->mode = MOTOR_MODE_IDLE;
  mc->target_rpm = 0;
  mc->target_deg_x100 = 0;
  mc->iq_cmd = 0;

  DJI_Motor_Init(&mc->fb);

  PID_Init(&pid_spd,
           0.6f, 0.02f, 0.0f,
           -12000.0f, 12000.0f,
           -8000.0f,  8000.0f);
}

void MotorTask_SetVel(MotorCtrl_t *mc, int32_t rpm)
{
  mc->mode = MOTOR_MODE_VEL;
  mc->target_rpm = rpm;
}

void MotorTask_SetPos(MotorCtrl_t *mc, int32_t deg)
{
  mc->mode = MOTOR_MODE_POS;
  mc->target_deg_x100 = deg * 100;
}

void MotorTask_1ms(MotorCtrl_t *mc)
{
  int32_t rpm_ref = 0;

  if (mc->mode == MOTOR_MODE_VEL) {
    rpm_ref = mc->target_rpm;
  }
  else if (mc->mode == MOTOR_MODE_POS) {
    int32_t pos_x100 = DJI_Motor_GetAngleDeg_x100(&mc->fb);
    int32_t err_x100 = mc->target_deg_x100 - pos_x100;
    rpm_ref = err_x100 / 100;  // 简单比例，真实可再调
  }

  float err = (float)(rpm_ref - mc->fb.speed_rpm);
  mc->iq_cmd = (int16_t)PID_Step(&pid_spd, err);

  /* 这里暂不发 CAN，等你确认接口后再加 */
}
