#ifndef __ZDT_PROTO_H__
#define __ZDT_PROTO_H__
#include "sys.h"

/* ================================================================
 * ZDT 闭环步进电机 — 自定义串口协议 (Emm_V5 固件)
 *
 * 帧格式: [Addr(1B)] [Cmd(1B)] [Data(NB)] [0x6B]
 *
 * 多电机支持: 每个电机绑定一个 ZDT_Port (不同UART通道)
 * ================================================================ */

/* ---- 端口抽象 (多电机核心) ---- */
typedef struct {
    void (*send)(u8 *buf, u16 len);           /* 发送函数 */
    void (*flush)(void);                      /* 清空接收缓冲 */
    u16  (*recv)(u8 *buf, u16 max, u32 to);   /* 接收函数 (直接寄存器访问) */
    u8   addr;                                /* 电机地址 (默认0x01) */
    u8   last_rx[12];
    u8   last_rx_len;
    u8   last_error;
} ZDT_Port;

/* ---- 通信参数 ---- */
#define ZDT_BAUDRATE       115200
#define ZDT_CHECKSUM_6B    0x6B

/* ---- 功能码 ---- */
#define ZDT_CMD_ENABLE     0xF3
#define ZDT_CMD_SPEED      0xF6
#define ZDT_CMD_POS_TRAP   0xFD
#define ZDT_CMD_STOP       0xFE
#define ZDT_CMD_READ_POS   0x36
#define ZDT_CMD_READ_SPEED 0x35
#define ZDT_CMD_MULTI_SYNC 0xFF

/* ---- 返回码 ---- */
#define ZDT_RET_OK    0x02
#define ZDT_RET_BUSY  0xE2
#define ZDT_RET_ERROR 0xEE

/* ---- 方向 ---- */
#define ZDT_DIR_CW    0x00
#define ZDT_DIR_CCW   0x01

/* ---- 标志 ---- */
#define ZDT_ENABLE     0x01
#define ZDT_DISABLE    0x00
#define ZDT_SYNC_OFF   0x00
#define ZDT_SYNC_ON    0x01

/* ---- API (所有函数第一个参数为 ZDT_Port*) ---- */
u8  ZDT_Enable  (ZDT_Port *p, u8 en);
u8  ZDT_GoPos   (ZDT_Port *p, u8 dir, u16 speed, u32 pulses, u8 absolute);
u8  ZDT_GoSpeed (ZDT_Port *p, u8 dir, u16 speed);
u8  ZDT_Stop    (ZDT_Port *p);
u8  ZDT_ReadPos (ZDT_Port *p, int32_t *pos);
u8  ZDT_ReadSpeed(ZDT_Port *p, int16_t *spd);
u8  ZDT_Sync    (ZDT_Port *p);

/* 发送原始命令帧 (用于诊断) */
u8  ZDT_SendRaw (ZDT_Port *p, u8 cmd, u8 *data, u8 len);

#endif
