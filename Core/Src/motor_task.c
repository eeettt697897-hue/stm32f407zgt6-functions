#include "motor_task.h"
#include "pid.h"
#include "main.h"
#include "dji_motor.h"

extern CAN_HandleTypeDef hcan1;

/* 速度 PID */
static PID_t pid_spd;

static void CAN_SendCurrent_0x200(int16_t i1, int16_t i2, int16_t i3, int16_t i4)
{
  CAN_TxHeaderTypeDef txh;
  uint8_t data[8];
  uint32_t mailbox;

  txh.StdId = 0x200;
  txh.IDE   = CAN_ID_STD;
  txh.RTR   = CAN_RTR_DATA;
  txh.DLC   = 8;
  txh.TransmitGlobalTime = DISABLE;

  DJI_PackCurrentCmd_0x200(i1, i2, i3, i4, data);

  HAL_CAN_AddTxMessage(&hcan1, &txh, data, &mailbox);
}


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
    rpm_ref = err_x100 / 100;
  }

  float err = (float)(rpm_ref - mc->fb.speed_rpm);
  mc->iq_cmd = (int16_t)PID_Step(&pid_spd, err);

  CAN_SendCurrent_0x200(mc->iq_cmd, 0, 0, 0);

}
