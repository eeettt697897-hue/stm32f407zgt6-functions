#include "dji_motor.h"

static int16_t be16(const uint8_t *p)
{
    return (int16_t)((p[0] << 8) | p[1]);
}

void DJI_Motor_Init(DJI_Motor_t *m)
{
    m->angle_raw = 0;
    m->speed_rpm = 0;
    m->current_raw = 0;
    m->temp = 0;

    m->angle_acc = 0;
    m->last_angle_raw = 0;
    m->inited = 0;
}

void DJI_Motor_OnFeedback(DJI_Motor_t *m, const uint8_t data[8])
{
    uint16_t angle = (uint16_t)((data[0] << 8) | data[1]);
    int16_t  speed = be16(&data[2]);
    int16_t  curr  = be16(&data[4]);
    uint8_t  temp  = data[6];

    m->angle_raw   = angle;
    m->speed_rpm   = speed;
    m->current_raw = curr;
    m->temp        = temp;

    if (m->inited == 0) {
        m->last_angle_raw = angle;
        m->angle_acc = 0;
        m->inited = 1;
        return;
    }

    int32_t diff = (int32_t)angle - (int32_t)m->last_angle_raw;

    if (diff > 4096)  diff -= 8192;
    if (diff < -4096) diff += 8192;

    m->angle_acc += diff;
    m->last_angle_raw = angle;
}

int32_t DJI_Motor_GetAngleDeg_x100(const DJI_Motor_t *m)
{
    /* 8192 counts = 360 deg */
    return (int32_t)((m->angle_acc * 36000) / 8192);
}

void DJI_PackCurrentCmd_0x200(int16_t i1,
                             int16_t i2,
                             int16_t i3,
                             int16_t i4,
                             uint8_t out8[8])
{
    out8[0] = (uint8_t)(i1 >> 8);
    out8[1] = (uint8_t)(i1);
    out8[2] = (uint8_t)(i2 >> 8);
    out8[3] = (uint8_t)(i2);
    out8[4] = (uint8_t)(i3 >> 8);
    out8[5] = (uint8_t)(i3);
    out8[6] = (uint8_t)(i4 >> 8);
    out8[7] = (uint8_t)(i4);
}
