#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include <stdint.h>

typedef struct {
    uint16_t angle_raw;      // 0~8191
    int16_t  speed_rpm;      // rpm
    int16_t  current_raw;    // current feedback
    uint8_t  temp;           // temperature

    int32_t  angle_acc;      // multi-turn accumulated angle (raw counts)
    uint16_t last_angle_raw;
    uint8_t  inited;
} DJI_Motor_t;

void DJI_Motor_Init(DJI_Motor_t *m);

/* 解析 0x201~0x208 反馈帧 */
void DJI_Motor_OnFeedback(DJI_Motor_t *m, const uint8_t data[8]);

/* 获取多圈角度（degree ×100） */
int32_t DJI_Motor_GetAngleDeg_x100(const DJI_Motor_t *m);

/* 打包 0x200 控制帧（四个电机电流） */
void DJI_PackCurrentCmd_0x200(int16_t i1,
                             int16_t i2,
                             int16_t i3,
                             int16_t i4,
                             uint8_t out8[8]);

#endif
