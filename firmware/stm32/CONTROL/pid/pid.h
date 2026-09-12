#ifndef __PID_H__
#define __PID_H__
#include "sys.h"

/* ================================================================
 * 位置式 PID 控制器
 *
 * 用于将像素误差转换为电机速度指令.
 * 带输出限幅 + 积分分离 + 死区.
 * ================================================================ */

typedef struct {
    float kp, ki, kd;          /* PID 参数 */
    float integral;            /* 积分累加 */
    float last_error;          /* 上次误差 */
    float out_max;             /* 输出限幅 (绝对值) */
    float deadband;            /* 死区 (绝对值) */
} PID_State;

/* 初始化 */
void PID_Init(PID_State *pid, float kp, float ki, float kd,
              float out_max, float deadband);

/* 单次更新: error = 目标 - 实际, 返回控制量 */
float PID_Update(PID_State *pid, float error);

/* 重置积分和上次误差 */
void PID_Reset(PID_State *pid);

/* 在线调参 */
void PID_SetTuning(PID_State *pid, float kp, float ki, float kd);

#endif
