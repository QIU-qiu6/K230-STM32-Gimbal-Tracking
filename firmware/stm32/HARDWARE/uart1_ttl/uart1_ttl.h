#ifndef __UART1_TTL_H__
#define __UART1_TTL_H__
#include "sys.h"

/* Correct X42S TTL wiring: PA9 TX -> R/A/H, PA10 RX <- T/B/L. */

/* PA9=TX, PA10=RX, 直连电机R/A/H和T/B/L */
void UART1_TTL_Init(uint32_t baudrate);
void UART1_SendByte(u8 data);
void UART1_SendBytes(u8 *buf, u16 len);
void UART1_SendString(const char *str);
u8   UART1_RecvByte(u8 *byte, u32 timeout_ms);
u16  UART1_RecvBytes(u8 *buf, u16 max_len, u32 timeout_ms);
void UART1_Flush(void);
void UART1_RX_IRQ(void);

#endif
