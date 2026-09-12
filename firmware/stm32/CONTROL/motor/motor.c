/**
 * 电机抽象层实现
 */
#include "motor.h"

/* ---- 编码器 ↔ 角度 ---- */
float Motor_EncoderToAngle(int32_t encoder)
{
    return (float)encoder / (float)MOTOR_POSITION_COUNTS_PER_REV * 360.0f;
}

int32_t Motor_AngleToEncoder(float angle_deg)
{
    return (int32_t)(angle_deg / 360.0f * (float)MOTOR_COMMAND_PULSES_PER_REV);
}

/* ---- 初始化 ---- */
void Motor_Init(Motor_State *m, ZDT_Port *port)
{
    m->port          = port;
    m->encoder_raw   = 0;
    m->encoder_zero  = 0;
    m->angle_deg     = 0.0f;
    m->speed_rpm     = 0;
    m->online        = 0;
    m->enabled       = 0;
    m->busy          = 0;
}

/* ---- 使能 ---- */
u8 Motor_Enable(Motor_State *m)
{
    u8 ret = ZDT_Enable(m->port, ZDT_ENABLE);
    if (ret == 0) {
        m->enabled = 1;
        m->online  = 1;
    }
    return ret;
}

u8 Motor_Disable(Motor_State *m)
{
    u8 ret = ZDT_Enable(m->port, ZDT_DISABLE);
    if (ret == 0) m->enabled = 0;
    return ret;
}

/* ---- 急停 ---- */
u8 Motor_EmergencyStop(Motor_State *m)
{
    u8 data[] = {0x98, 0x00};
    /* Emergency path sends immediately and never waits for a reply. */
    ZDT_SendRaw(m->port, ZDT_CMD_STOP, data, 2);
    m->speed_rpm = 0;
    m->enabled = 0;
    return 0;
}

/* ---- 速度控制 ---- */
u8 Motor_SetSpeed(Motor_State *m, int16_t speed_rpm)
{
    u8 dir;
    u16 abs_speed;

    if (speed_rpm == 0) {
        return ZDT_Stop(m->port);
    }

    dir       = (speed_rpm > 0) ? ZDT_DIR_CCW : ZDT_DIR_CW;
    abs_speed = (u16)((speed_rpm > 0) ? speed_rpm : -speed_rpm);

    return ZDT_GoSpeed(m->port, dir, abs_speed);
}

/* ---- 相对位置移动 ---- */
u8 Motor_MoveRelative(Motor_State *m, float delta_deg, u16 speed_rpm)
{
    int32_t pulses;
    u8 dir;

    pulses = Motor_AngleToEncoder(delta_deg);
    dir    = (pulses >= 0) ? ZDT_DIR_CCW : ZDT_DIR_CW;
    if (pulses < 0) pulses = -pulses;

    return ZDT_GoPos(m->port, dir, speed_rpm, (u32)pulses, 0);
}

/* ---- 绝对位置移动 ---- */
u8 Motor_MoveAbsolute(Motor_State *m, float angle_deg, u16 speed_rpm)
{
    int32_t pulses;
    u8 dir;

    pulses = Motor_AngleToEncoder(angle_deg);
    dir    = (pulses >= 0) ? ZDT_DIR_CCW : ZDT_DIR_CW;
    if (pulses < 0) pulses = -pulses;

    return ZDT_GoPos(m->port, dir, speed_rpm, (u32)pulses, 1);
}

/* ---- 反馈 ---- */
u8 Motor_ReadPosition(Motor_State *m)
{
    int32_t pos;
    u8 ret = ZDT_ReadPos(m->port, &pos);
    if (ret == 0) {
        m->encoder_raw = pos;
        m->angle_deg   = Motor_EncoderToAngle(pos - m->encoder_zero);
        m->online      = 1;
    } else {
        m->online = 0;
    }
    return ret;
}

u8 Motor_ReadSpeed(Motor_State *m)
{
    int16_t spd;
    u8 ret = ZDT_ReadSpeed(m->port, &spd);
    if (ret == 0) m->speed_rpm = spd;
    return ret;
}

/* ---- 设零点 ---- */
u8 Motor_SetZero(Motor_State *m)
{
    int32_t pos;
    u8 ret = ZDT_ReadPos(m->port, &pos);
    if (ret == 0) {
        m->encoder_zero = pos;
        m->angle_deg    = 0.0f;
    }
    return ret;
}

/* ---- 在线检测 ---- */
u8 Motor_Ping(Motor_State *m)
{
    int32_t pos;
    u8 ret = ZDT_ReadPos(m->port, &pos);
    m->online = (ret == 0) ? 1 : 0;
    return ret;
}
