#ifndef __GIMBAL_H__
#define __GIMBAL_H__
#include "sys.h"
#include "motor.h"
#include "pid.h"
#include "k230_proto.h"

/* ================================================================
 * 2轴云台追踪控制器
 *
 * Yaw:   水平旋转 (USART1)
 * Pitch: 俯仰角度 (USART3)
 *
 * K230 摄像头 800x480, 中心 (400, 240)
 * ================================================================ */

/* ================================================================
 * 用户可调配置
 * ================================================================ */
#define GIMBAL_ASSEMBLED_MODE          1
#define GIMBAL_COMMISSIONING_MODE      1

#define GIMBAL_CAM_CX                400     /* 画面中心 X (800/2) */
#define GIMBAL_CAM_CY                240     /* 画面中心 Y (480/2) */

#define GIMBAL_YAW_DIRECTION          1.0f   /* 方向反了改为 -1.0f */
#define GIMBAL_PITCH_DIRECTION        1.0f   /* 方向反了改为 -1.0f */

#define GIMBAL_YAW_KP                 0.10f
#define GIMBAL_YAW_KI                 0.0f
#define GIMBAL_YAW_KD                 0.0f
#define GIMBAL_PITCH_KP               0.08f
#define GIMBAL_PITCH_KI               0.0f
#define GIMBAL_PITCH_KD               0.0f

#define GIMBAL_YAW_SPEED_MAX_RPM      5.0f
#define GIMBAL_PITCH_SPEED_MAX_RPM    3.0f
#define GIMBAL_YAW_DEADBAND_PX        8.0f
#define GIMBAL_PITCH_DEADBAND_PX      8.0f

/* Pitch 限位相对于开机软件零点。
 * 这是当前临时值，最终需要根据实物机械极限重新测量，并预留安全余量。 */
#define GIMBAL_PITCH_MIN_DEG         (-45.0f)
#define GIMBAL_PITCH_MAX_DEG           90.0f
#define GIMBAL_PITCH_SOFT_ZONE_DEG     10.0f
#define GIMBAL_PITCH_HARD_MARGIN_DEG    3.0f

typedef struct {
    Motor_State *yaw;               /* Yaw 电机 (水平) */
    Motor_State *pitch;             /* Pitch 电机 (俯仰) */

    PID_State    pid_yaw;           /* Yaw PID */
    PID_State    pid_pitch;         /* Pitch PID */

    K230_Frame   target;            /* 最新目标 */
    u8           tracking;          /* 1=追踪中 */

} Gimbal_System;

/* 初始化 */
void Gimbal_Init(Gimbal_System *g, Motor_State *yaw, Motor_State *pitch);

/* 使能/失能双电机 */
void Gimbal_Enable(Gimbal_System *g);
void Gimbal_Disable(Gimbal_System *g);
void Gimbal_EmergencyStop(Gimbal_System *g);

/* 设当前角度为零点 */
void Gimbal_SetZero(Gimbal_System *g);

/* 追踪更新 — 每周期调用一次 (~20Hz)
   1. 查询 K230 最新帧
   2. 计算像素误差
   3. PID 输出 → 电机速度指令 */
void Gimbal_Update(Gimbal_System *g);

/* 读当前角度 */
void Gimbal_GetAngles(Gimbal_System *g, float *yaw_deg, float *pitch_deg);

#endif
