/**
 * 位置式 PID 控制器实现
 */
#include "pid.h"
#include <math.h>

void PID_Init(PID_State *pid, float kp, float ki, float kd,
              float out_max, float deadband)
{
    pid->kp        = kp;
    pid->ki        = ki;
    pid->kd        = kd;
    pid->integral  = 0.0f;
    pid->last_error = 0.0f;
    pid->out_max   = out_max;
    pid->deadband  = deadband;
}

void PID_Reset(PID_State *pid)
{
    pid->integral   = 0.0f;
    pid->last_error = 0.0f;
}

void PID_SetTuning(PID_State *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float PID_Update(PID_State *pid, float error)
{
    float p_out, i_out, d_out, output;

    /* 死区内必须严格输出0，同时清掉历史积分，避免目标居中后仍缓慢爬行。 */
    if (fabs(error) < pid->deadband) {
        PID_Reset(pid);
        return 0.0f;
    }

    /* 比例 */
    p_out = pid->kp * error;

    /* 积分 (带分离: 大误差时不积分) */
    if (fabs(error) < pid->out_max * 0.5f) {
        pid->integral += error;
        /* 积分限幅 */
        if (pid->integral >  pid->out_max) pid->integral =  pid->out_max;
        if (pid->integral < -pid->out_max) pid->integral = -pid->out_max;
    }
    i_out = pid->ki * pid->integral;

    /* 微分 */
    d_out = pid->kd * (error - pid->last_error);
    pid->last_error = error;

    /* 合成 + 限幅 */
    output = p_out + i_out + d_out;
    if (output >  pid->out_max) output =  pid->out_max;
    if (output < -pid->out_max) output = -pid->out_max;

    return output;
}
