/**
 * ZDT 闭环步进电机协议实现 (Emm_V5 固件)
 *
 * 从 motor_test 已验证版本移植, 加 ZDT_Port 支持多路UART.
 * 关键:
 *   - ZDT_SendRaw 内无 Delay_ms → 避免 ORE 丢字节
 *   - 接收端由底层 UART 驱动保证直接寄存器访问
 *   - 校验字节固定 0x6B
 */
#include "zdt_proto.h"

/* ---- 内部: 发送命令帧 [Addr][Cmd][Data...][0x6B] ---- */
u8 ZDT_SendRaw(ZDT_Port *p, u8 cmd, u8 *data, u8 len)
{
    u8 buf[32], i, idx;
    idx = 0;
    buf[idx++] = p->addr;
    buf[idx++] = cmd;
    for (i = 0; i < len; i++) buf[idx++] = data[i];
    buf[idx++] = ZDT_CHECKSUM_6B;

    p->flush();
    p->send(buf, idx);
    return 0;
}

/* ---- 内部: 接收响应 ----
   响应格式: [Addr][Cmd][Status或Data...][0x6B]
   - 动作命令: raw[2] = Status (0x02=OK, 0xE2=Busy, 0xEE=Error)
   - 查询命令: raw[2..] = Data
   @retval 0=OK, 0xFF=超时, 0xFE=地址不匹配, 0xE2=Busy, 0xEE=Error
   ------------------------------------------------------------ */
static u8 ZDT_Recv(ZDT_Port *p, u8 expected_cmd, u8 *buf, u8 *len, u32 to_ms)
{
    u8 raw[32];
    u16 rx, i;

    p->last_rx_len = 0;
    p->last_error = 0;
    *len = 0;
    rx = p->recv(raw, 32, to_ms);
    p->last_rx_len = (rx > 12) ? 12 : (u8)rx;
    for (i = 0; i < p->last_rx_len; i++) p->last_rx[i] = raw[i];
    if (rx < 3) {
        p->last_error = 0xFF;
        return 0xFF;
    }

    for (i = 0; i < rx; i++) buf[i] = raw[i];
    *len = (u8)rx;

    if (raw[0] != p->addr) {
        p->last_error = 0xFE;
        return 0xFE;
    }
    if (raw[1] != expected_cmd || raw[rx - 1] != ZDT_CHECKSUM_6B) {
        p->last_error = 0xFD;
        return 0xFD;
    }

    /* raw[1] = 命令回显, raw[2] = 状态字节 */
    if (raw[2] == ZDT_RET_BUSY) {
        p->last_error = ZDT_RET_BUSY;
        return ZDT_RET_BUSY;
    }
    if (raw[2] == ZDT_RET_ERROR) {
        p->last_error = ZDT_RET_ERROR;
        return ZDT_RET_ERROR;
    }
    return 0;
}

/* ================================================================
 * 高层指令
 * ================================================================ */

u8 ZDT_Enable(ZDT_Port *p, u8 en)
{
    u8 d[] = {0xAB, en, 0x00};
    u8 rsp[16], rl, ret;
    ZDT_SendRaw(p, ZDT_CMD_ENABLE, d, 3);
    ret = ZDT_Recv(p, ZDT_CMD_ENABLE, rsp, &rl, 50);
    if (ret != 0) return ret;
    return (rl == 4 && rsp[2] == ZDT_RET_OK) ? 0 : 0xFD;
}

u8 ZDT_GoPos(ZDT_Port *p, u8 dir, u16 speed, u32 pulses, u8 absolute)
{
    u8 d[10];
    d[0] = dir;
    d[1] = (speed >> 8) & 0xFF;
    d[2] = speed & 0xFF;
    d[3] = 0;                         /* accel=0 直通 */
    d[4] = (pulses >> 24) & 0xFF;
    d[5] = (pulses >> 16) & 0xFF;
    d[6] = (pulses >> 8)  & 0xFF;
    d[7] = pulses & 0xFF;
    d[8] = absolute;
    d[9] = ZDT_SYNC_OFF;

    u8 rsp[16], rl, ret;
    ZDT_SendRaw(p, ZDT_CMD_POS_TRAP, d, 10);
    ret = ZDT_Recv(p, ZDT_CMD_POS_TRAP, rsp, &rl, 100);
    if (ret != 0) return ret;
    return (rl == 4 && rsp[2] == ZDT_RET_OK) ? 0 : 0xFD;
}

u8 ZDT_GoSpeed(ZDT_Port *p, u8 dir, u16 speed)
{
    u8 d[] = {dir, (speed>>8)&0xFF, speed&0xFF, 0, 0};
    u8 rsp[16], rl, ret;
    ZDT_SendRaw(p, ZDT_CMD_SPEED, d, 5);
    ret = ZDT_Recv(p, ZDT_CMD_SPEED, rsp, &rl, 50);
    if (ret != 0) return ret;
    return (rl == 4 && rsp[2] == ZDT_RET_OK) ? 0 : 0xFD;
}

u8 ZDT_Stop(ZDT_Port *p)
{
    u8 d[] = {0x98, 0x00};
    u8 rsp[16], rl, ret;
    ZDT_SendRaw(p, ZDT_CMD_STOP, d, 2);
    ret = ZDT_Recv(p, ZDT_CMD_STOP, rsp, &rl, 50);
    if (ret != 0) return ret;
    return (rl == 4 && rsp[2] == ZDT_RET_OK) ? 0 : 0xFD;
}

u8 ZDT_ReadPos(ZDT_Port *p, int32_t *pos)
{
    u8 rsp[16], rl;
    u8 ret;
    u32 magnitude;
    ZDT_SendRaw(p, ZDT_CMD_READ_POS, NULL, 0);
    ret = ZDT_Recv(p, ZDT_CMD_READ_POS, rsp, &rl, 50);
    /* 官方格式: [Addr][36][Sign][Pos3..Pos0][6B]，共8字节。 */
    if (ret != 0 || rl != 8 || rsp[1] != ZDT_CMD_READ_POS ||
        rsp[7] != ZDT_CHECKSUM_6B) {
        *pos = 0;
        if (ret == 0) p->last_error = 0xFD;
        return (ret == 0) ? 0xFD : ret;
    }
    magnitude = ((u32)rsp[3]<<24)|((u32)rsp[4]<<16)|
                ((u32)rsp[5]<<8) |(u32)rsp[6];
    if (magnitude > 0x7FFFFFFFUL) {
        *pos = 0;
        p->last_error = 0xFD;
        return 0xFD;
    }
    *pos = rsp[2] ? -(int32_t)magnitude : (int32_t)magnitude;
    return 0;
}

u8 ZDT_ReadSpeed(ZDT_Port *p, int16_t *spd)
{
    u8 rsp[16], rl;
    u8 ret;
    ZDT_SendRaw(p, ZDT_CMD_READ_SPEED, NULL, 0);
    ret = ZDT_Recv(p, ZDT_CMD_READ_SPEED, rsp, &rl, 50);
    /* 官方格式: [Addr][35][Sign][Spd_H][Spd_L][6B]，共6字节。 */
    if (ret != 0 || rl != 6 || rsp[1] != ZDT_CMD_READ_SPEED ||
        rsp[5] != ZDT_CHECKSUM_6B) {
        *spd = 0;
        if (ret == 0) p->last_error = 0xFD;
        return (ret == 0) ? 0xFD : ret;
    }
    *spd = (int16_t)(((u16)rsp[3] << 8) | rsp[4]);
    if (rsp[2]) *spd = -*spd;
    return 0;
}

u8 ZDT_Sync(ZDT_Port *p)
{
    u8 d[] = {0x66};
    u8 rsp[16], rl;
    ZDT_SendRaw(p, ZDT_CMD_MULTI_SYNC, d, 1);
    return ZDT_Recv(p, ZDT_CMD_MULTI_SYNC, rsp, &rl, 20);
}
