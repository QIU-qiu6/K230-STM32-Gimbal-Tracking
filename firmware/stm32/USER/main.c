/**
 * ================================================================
 * 2-Electronic-Contest Gimbal — STM32F103C8T6 Main
 *
 * UART1 (PA9/PA10)  -> Yaw motor   (horizontal, gear+slip-ring)
 * UART3 (PB10/PB11) -> Pitch motor (tilt)
 * UART2 (PA2/PA3)   -> K230 camera (RXNE interrupt, 115200)
 *
 * Control:
 *   K230 -> color board pixel coords ->
 *   STM32 PID -> motor speed commands ->
 *   2-axis tracking keeps board centered
 * ================================================================
 */
#include "sys.h"
#include "stm32f10x.h"
#include "delay.h"
#include "uart1_ttl.h"
#include "uart3_ttl.h"
#include "zdt_proto.h"
#include "k230_proto.h"
#include "motor.h"
#include "gimbal.h"
#include "estop.h"
#include "oled.h"
#include <stdio.h>

/* ---- Globals ---- */
volatile uint32_t g_sys_tick = 0;

static ZDT_Port    g_yaw_port;
static ZDT_Port    g_pitch_port;
static Motor_State g_yaw_motor;
static Motor_State g_pitch_motor;
static Gimbal_System g_gimbal;
static u8 g_motors_ok = 0;
static u8 g_estop_handled = 0;
static u8 g_fault_latched = 0;
static u8 g_oled_ok = 0;
static u8 g_yaw_diag = 0;
static u8 g_pitch_diag = 0;
static u8 g_yaw_pos_fail = 0;
static u8 g_pitch_pos_fail = 0;
/* 00=none, 08/09=position Y/P, 11=Pitch angle. */
static u8 g_fault_code = 0;

/* ================================================================
 * SysTick — 1ms
 * ================================================================ */
void SysTick_Handler(void)
{
    g_sys_tick++;
    K230_Tick1ms();
    Estop_Tick1ms();
}

static void SysTick_Init(void)
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    SysTick->LOAD = 72000 - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk    |
                    SysTick_CTRL_ENABLE_Msk;
}

/* ================================================================
 * NVIC
 * ================================================================ */
static void NVIC_Config(void)
{
    NVIC_InitTypeDef n;

    n.NVIC_IRQChannel                   = USART1_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 1;
    n.NVIC_IRQChannelSubPriority        = 1;
    n.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&n);

    /* USART2 IRQ — K230 RXNE byte-by-byte receive */
    n.NVIC_IRQChannel                   = USART2_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 1;
    n.NVIC_IRQChannelSubPriority        = 0;
    n.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&n);

    n.NVIC_IRQChannel                   = USART3_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 1;
    n.NVIC_IRQChannelSubPriority        = 2;
    n.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&n);
}

/* ================================================================
 * Motor self-test
 * ================================================================ */
static u8 MotorSelfTest(void)
{
    g_yaw_diag = Motor_Ping(&g_yaw_motor);
    g_pitch_diag = Motor_Ping(&g_pitch_motor);
    return (g_yaw_diag == 0 && g_pitch_diag == 0) ? 1 : 0;
}

/* ================================================================
 * Main
 * ================================================================ */
int main(void)
{
    uint32_t last_100hz, last_10hz, last_1hz, tick, tracking_enable_tick;
    float yaw_deg, pitch_deg;
    u8 yaw_ret, pitch_ret;

    /* ---- 1. System init ---- */
    Estop_Init();
    SysTick_Init();
    Delay_Init();
    NVIC_Config();

    /* ---- 2. UART init ---- */
    UART1_TTL_Init(115200);     /* Yaw motor */
    UART3_TTL_Init(115200);     /* Pitch motor */
    K230_Init(115200);          /* K230 camera (RXNE interrupt) */
    g_oled_ok = OLED_Init();

    /* 等待 UART TX 引脚稳定, 避免首字符丢失 */
    Delay_ms(50);
    K230_SendString("\r\n");    /* 空行唤醒接收端 */

    /* ---- 3. Port binding ---- */
    g_yaw_port.send  = UART1_SendBytes;
    g_yaw_port.flush = UART1_Flush;
    g_yaw_port.recv  = UART1_RecvBytes;
    g_yaw_port.addr  = 0x01;
    g_yaw_port.last_rx_len = 0;
    g_yaw_port.last_error = 0;

    g_pitch_port.send  = UART3_SendBytes;
    g_pitch_port.flush = UART3_Flush;
    g_pitch_port.recv  = UART3_RecvBytes;
    g_pitch_port.addr  = 0x01;
    g_pitch_port.last_rx_len = 0;
    g_pitch_port.last_error = 0;

    /* ---- 4. Motor + Gimbal init ---- */
    Motor_Init(&g_yaw_motor,   &g_yaw_port);
    Motor_Init(&g_pitch_motor, &g_pitch_port);
    Gimbal_Init(&g_gimbal, &g_yaw_motor, &g_pitch_motor);

    /* ---- 5. Self-test + enable ---- */
    K230_SendString("Gimbal Dual-Motor\r\n");
    K230_SendString("STM32_READY\r\n");

    /* 急停按下时不向电机发送任何使能或自检命令。 */
    K230_SendString("Self-test...\r\n");
    if (!Estop_IsLatched() && MotorSelfTest()) {
        g_motors_ok = 1;
        K230_SendString("Motors:OK\r\n");
    } else {
        K230_SendString("Motors:FAIL\r\n");
    }

    if (g_motors_ok &&
        (Motor_SetZero(&g_yaw_motor) != 0 || Motor_SetZero(&g_pitch_motor) != 0)) {
        g_motors_ok = 0;
        g_fault_latched = 1;
        K230_SendString("Zero:FAIL\r\n");
    }

    if (g_motors_ok && !Estop_IsLatched()) {
        K230_SendString("Enable...\r\n");
        Gimbal_Enable(&g_gimbal);
        if (g_yaw_motor.enabled && g_pitch_motor.enabled) {
            /* Enabling can change the driver's reported multi-turn reference.
             * Wait for both encoders to settle, then establish the software
             * zero used by the mechanical angle limits. */
            Delay_ms(100);
            yaw_ret = Motor_SetZero(&g_yaw_motor);
            pitch_ret = Motor_SetZero(&g_pitch_motor);
            if (yaw_ret == 0 && pitch_ret == 0) {
                K230_SendString("ARMED\r\n");
            } else {
                Gimbal_EmergencyStop(&g_gimbal);
                g_motors_ok = 0;
                g_fault_latched = 1;
                g_fault_code = (yaw_ret != 0) ? 4 : 5;
                g_yaw_diag = yaw_ret;
                g_pitch_diag = pitch_ret;
                K230_SendString("PostEnableZero:FAIL\r\n");
            }
        } else {
            Gimbal_EmergencyStop(&g_gimbal);
            g_motors_ok = 0;
            g_fault_latched = 1;
            K230_SendString("Enable:FAIL\r\n");
        }
    } else if (Estop_IsLatched()) {
        g_estop_handled = 1;
        K230_SendString("ESTOP_AT_BOOT\r\n");
    } else {
        g_fault_latched = 1;
        K230_SendString("MOTOR_FAULT\r\n");
    }
    K230_SendString("Ready.\r\n");

    /* ---- 6. Main loop ---- */
    last_100hz = last_10hz = last_1hz = g_sys_tick;
    tracking_enable_tick = g_sys_tick;

    while (1) {
        tick = g_sys_tick;

        /* Physical E-stop is latched until MCU reset/power-cycle. */
        if (Estop_IsLatched() && !g_estop_handled) {
            Gimbal_EmergencyStop(&g_gimbal);
            g_estop_handled = 1;
            K230_SendString("ESTOP\r\n");
        }

        /* ---- 50Hz: tracking update; K230 frames are slower than this ---- */
        if (tick - last_100hz >= 20) {
            last_100hz = tick;
            if (g_motors_ok && !g_fault_latched && !Estop_IsLatched() &&
                (u32)(tick - tracking_enable_tick) >= 1500u) {
                Gimbal_Update(&g_gimbal);
            }
        }

        /* ---- 10Hz: safety check (cached values only) ---- */
        if (tick - last_10hz >= 100) {
            last_10hz = tick;

            if (g_motors_ok && !Estop_IsLatched()) {
                yaw_ret = Motor_ReadPosition(&g_yaw_motor);
                pitch_ret = Motor_ReadPosition(&g_pitch_motor);
                g_yaw_diag = yaw_ret;
                g_pitch_diag = pitch_ret;

                if (yaw_ret == 0) g_yaw_pos_fail = 0;
                else if (g_yaw_pos_fail < 255) g_yaw_pos_fail++;
                if (pitch_ret == 0) g_pitch_pos_fail = 0;
                else if (g_pitch_pos_fail < 255) g_pitch_pos_fail++;

                /* One noisy/late frame is not a permanent motor fault.
                 * Stop only after five consecutive failures (about 0.5 s). */
                if (g_yaw_pos_fail >= 5 || g_pitch_pos_fail >= 5) {
                    Gimbal_EmergencyStop(&g_gimbal);
                    g_motors_ok = 0;
                    g_fault_latched = 1;
                    g_fault_code = (g_yaw_pos_fail >= 5) ? 8 : 9;
                    K230_SendString("MOTOR_OFFLINE\r\n");
                } else if ((u32)(tick - tracking_enable_tick) >= 1000u &&
                           pitch_ret == 0 &&
                           (g_pitch_motor.angle_deg <
                                GIMBAL_PITCH_MIN_DEG -
                                GIMBAL_PITCH_HARD_MARGIN_DEG ||
                            g_pitch_motor.angle_deg >
                                GIMBAL_PITCH_MAX_DEG +
                                GIMBAL_PITCH_HARD_MARGIN_DEG)) {
                    Gimbal_EmergencyStop(&g_gimbal);
                    g_motors_ok = 0;
                    g_fault_latched = 1;
                    g_fault_code = 11;
                    K230_SendString("PITCH_ANGLE_FAULT\r\n");
                }
            }
        }

        /* ---- 1Hz: read encoder + online check + status ---- */
        if (tick - last_1hz >= 1000) {
            last_1hz = tick;

            Gimbal_GetAngles(&g_gimbal, &yaw_deg, &pitch_deg);
            {
                char buf[64];
                sprintf(buf, "Y:%.1f P:%.1f %s\r\n",
                        yaw_deg, pitch_deg,
                        g_gimbal.tracking ? "T" : "L");
                K230_SendString(buf);
            }

            if (g_oled_ok) {
                char line[24];
#if !GIMBAL_ASSEMBLED_MODE
                OLED_ShowLine(0, "GIMBAL BENCH");
#elif GIMBAL_COMMISSIONING_MODE
                OLED_ShowLine(0, "GIMBAL COMM");
#else
                OLED_ShowLine(0, "GIMBAL DUAL");
#endif
                if (!g_motors_ok) {
                    sprintf(line, "F:%02u Y:E%02X L:%u", g_fault_code, g_yaw_diag,
                            g_yaw_port.last_rx_len);
                    OLED_ShowLine(1, line);
                    sprintf(line, "%02X %02X %02X %02X %02X %02X",
                            g_yaw_port.last_rx[0], g_yaw_port.last_rx[1],
                            g_yaw_port.last_rx[2], g_yaw_port.last_rx[3],
                            g_yaw_port.last_rx[4], g_yaw_port.last_rx[5]);
                    OLED_ShowLine(2, line);
                    sprintf(line, "P:E%02X L:%u", g_pitch_diag,
                            g_pitch_port.last_rx_len);
                    OLED_ShowLine(3, line);
                    sprintf(line, "YA:%+.1f PA:%+.1f", yaw_deg, pitch_deg);
                    OLED_ShowLine(4, line);
                } else {
                    sprintf(line, "YAW:%+.1f", yaw_deg);
                    OLED_ShowLine(1, line);
                    sprintf(line, "PITCH:%+.1f", pitch_deg);
                    OLED_ShowLine(2, line);
                    if (!K230_LinkAlive()) {
                        OLED_ShowLine(3, "CAM:LOST");
                    } else if (!g_gimbal.target.has_target) {
                        OLED_ShowLine(3, "TARGET:NONE");
                    } else {
                        sprintf(line, "T:%u,%u", g_gimbal.target.cx,
                                g_gimbal.target.cy);
                        OLED_ShowLine(3, line);
                    }
                    sprintf(line, "COMM:Y%u P%u", g_yaw_pos_fail, g_pitch_pos_fail);
                    OLED_ShowLine(4, line);
                }
                if (Estop_IsLatched()) OLED_ShowLine(5, "STATE:ESTOP");
                else if (g_fault_latched) OLED_ShowLine(5, "STATE:FAULT");
                else if (g_gimbal.tracking) OLED_ShowLine(5, "STATE:TRACK");
                else OLED_ShowLine(5, "STATE:IDLE");
                OLED_ShowLine(6, Estop_IsLatched() ? "ESTOP:PRESSED" : "ESTOP:READY");
                if (!OLED_Refresh()) g_oled_ok = 0;
            }
        }
    }
}
