/**
 * 2轴云台追踪控制器实现
 *
 * 控制策略:
 *   速度模式 PID — 像素误差直接映射为电机转速.
 *   目标偏离中心越远, 电机转得越快.
 *   死区内电机停止, 避免抖动.
 */
#include "gimbal.h"

/* Motor firmware reports encoder angle opposite to the signed speed command:
 * negative speed increases angle; positive speed decreases angle. */
static float Gimbal_ApplyPitchLimit(const Gimbal_System *g,
                                    float requested_speed)
{
    float angle;
    float remaining;
    float scale;

    angle = g->pitch->angle_deg;

    if (requested_speed < 0.0f) {
        /* Negative speed moves toward the Pitch upper limit. */
        remaining = GIMBAL_PITCH_MAX_DEG - angle;
        if (remaining <= 0.0f) return 0.0f;
        if (remaining < GIMBAL_PITCH_SOFT_ZONE_DEG) {
            scale = remaining / GIMBAL_PITCH_SOFT_ZONE_DEG;
            if (scale < 0.0f) scale = 0.0f;
            if (scale > 1.0f) scale = 1.0f;
            requested_speed *= scale;
        }
    } else if (requested_speed > 0.0f) {
        /* Positive speed moves toward the Pitch lower limit. */
        remaining = angle - GIMBAL_PITCH_MIN_DEG;
        if (remaining <= 0.0f) return 0.0f;
        if (remaining < GIMBAL_PITCH_SOFT_ZONE_DEG) {
            scale = remaining / GIMBAL_PITCH_SOFT_ZONE_DEG;
            if (scale < 0.0f) scale = 0.0f;
            if (scale > 1.0f) scale = 1.0f;
            requested_speed *= scale;
        }
    }

    return requested_speed;
}

static int16_t Gimbal_RpmToCommand(float requested_speed, float max_rpm)
{
    float limited_speed;

    limited_speed = requested_speed;
    if (limited_speed > max_rpm) limited_speed = max_rpm;
    if (limited_speed < -max_rpm) limited_speed = -max_rpm;

    if (limited_speed > -0.5f && limited_speed < 0.5f) return 0;
    if (limited_speed > 0.0f) return (int16_t)(limited_speed + 0.5f);
    return (int16_t)(limited_speed - 0.5f);
}

void Gimbal_Init(Gimbal_System *g, Motor_State *yaw, Motor_State *pitch)
{
    g->yaw   = yaw;
    g->pitch = pitch;

    g->tracking      = 0;
    g->target.has_target = 0;
    g->target.cx = GIMBAL_CAM_CX;
    g->target.cy = GIMBAL_CAM_CY;

    /* PID 初始化 */
    PID_Init(&g->pid_yaw,
             GIMBAL_YAW_KP, GIMBAL_YAW_KI, GIMBAL_YAW_KD,
             GIMBAL_YAW_SPEED_MAX_RPM, GIMBAL_YAW_DEADBAND_PX);
    PID_Init(&g->pid_pitch,
             GIMBAL_PITCH_KP, GIMBAL_PITCH_KI, GIMBAL_PITCH_KD,
             GIMBAL_PITCH_SPEED_MAX_RPM, GIMBAL_PITCH_DEADBAND_PX);
}

/* ---- 使能 ---- */
void Gimbal_Enable(Gimbal_System *g)
{
    Motor_Enable(g->yaw);
    Motor_Enable(g->pitch);
}

void Gimbal_Disable(Gimbal_System *g)
{
    Motor_Disable(g->yaw);
    Motor_Disable(g->pitch);
}

void Gimbal_EmergencyStop(Gimbal_System *g)
{
    Motor_EmergencyStop(g->yaw);
    Motor_EmergencyStop(g->pitch);
    g->tracking = 0;
}

/* ---- 设零点 ---- */
void Gimbal_SetZero(Gimbal_System *g)
{
    Motor_SetZero(g->yaw);
    Motor_SetZero(g->pitch);
}

/* ---- 查询角度 ---- */
void Gimbal_GetAngles(Gimbal_System *g, float *yaw_deg, float *pitch_deg)
{
    /* 返回主循环最近一次读取的缓存，避免重复阻塞查询。 */
    *yaw_deg   = g->yaw->angle_deg;
    *pitch_deg = g->pitch->angle_deg;
}

/* ================================================================
 * 追踪更新 (每周期调用)
 * ================================================================ */
void Gimbal_Update(Gimbal_System *g)
{
    float error_x, error_y;
    float speed_yaw, speed_pitch;
    K230_Frame f;

    /* 1. 摄像头失联时，不能维持最后一次速度。 */
    if (!K230_LinkAlive()) {
        if (g->tracking) {
            Motor_SetSpeed(g->yaw, 0);
            Motor_SetSpeed(g->pitch, 0);
            PID_Reset(&g->pid_yaw);
            PID_Reset(&g->pid_pitch);
            g->tracking = 0;
        }
        return;
    }

    /* 2. 原子获取 K230 新数据 */
    if (!K230_GetNewFrame(&f)) {
        return;  /* 无新帧, 维持当前速度 */
    }

    /* 3. 目标丢失 → 停转 */
    if (!f.has_target) {
        g->target = f;
        if (g->tracking) {
            Motor_SetSpeed(g->yaw,   0);
            Motor_SetSpeed(g->pitch, 0);
            PID_Reset(&g->pid_yaw);
            PID_Reset(&g->pid_pitch);
            g->tracking = 0;
        }
        return;
    }

    /* 4. 计算像素误差 */
    error_x = GIMBAL_YAW_DIRECTION *
              (float)((int16_t)f.cx - GIMBAL_CAM_CX);  /* + = 偏右 */
    error_y = GIMBAL_PITCH_DIRECTION *
              (float)((int16_t)f.cy - GIMBAL_CAM_CY);  /* + = 偏下 */

    /* 5. PID 计算 → 电机转速 */
    speed_yaw   = PID_Update(&g->pid_yaw,   error_x);
    speed_pitch = PID_Update(&g->pid_pitch, error_y);

    /* Yaw has no angle limit. Pitch slows only while moving toward a limit;
     * the opposite command always remains available to leave the boundary. */
    speed_pitch = Gimbal_ApplyPitchLimit(g, speed_pitch);

    /* 注意: Yaw 和 Pitch 的方向取决于机械安装.
       如果电机转反了, 在 error 前加负号即可 */
    /* error_x>0 (目标偏右) → Yaw 正转 (CCW) 带动摄像头向右 */
    /* error_y>0 (目标偏下) → Pitch 正转 带动摄像头向下 */

    /* 6. 发送速度指令 */
    Motor_SetSpeed(g->yaw,
                   Gimbal_RpmToCommand(speed_yaw,
                                       GIMBAL_YAW_SPEED_MAX_RPM));
    Motor_SetSpeed(g->pitch,
                   Gimbal_RpmToCommand(speed_pitch,
                                       GIMBAL_PITCH_SPEED_MAX_RPM));

    g->target   = f;
    g->tracking = 1;
}
