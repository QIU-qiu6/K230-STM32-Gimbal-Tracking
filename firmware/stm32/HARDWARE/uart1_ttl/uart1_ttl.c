#include "uart1_ttl.h"

extern volatile uint32_t g_sys_tick;

#define UART1_RX_BUF_SIZE 32u
static volatile u8 uart1_rx_buf[UART1_RX_BUF_SIZE];
static volatile u8 uart1_rx_head = 0;
static volatile u8 uart1_rx_tail = 0;

static u8 UART1_PopByte(u8 *byte)
{
    u8 tail = uart1_rx_tail;
    if (tail == uart1_rx_head) return 0;
    *byte = uart1_rx_buf[tail];
    uart1_rx_tail = (u8)((tail + 1u) % UART1_RX_BUF_SIZE);
    return 1;
}

void UART1_TTL_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate = baudrate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void UART1_RX_IRQ(void)
{
    u32 sr = USART1->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        u8 data = (u8)(USART1->DR & 0xFF);
        if (sr & USART_SR_RXNE) {
            u8 next = (u8)((uart1_rx_head + 1u) % UART1_RX_BUF_SIZE);
            if (next != uart1_rx_tail) {
                uart1_rx_buf[uart1_rx_head] = data;
                uart1_rx_head = next;
            }
        }
    }
}

void UART1_SendByte(u8 data)
{
    USART_SendData(USART1, data);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

void UART1_SendBytes(u8 *buf, u16 len)
{
    u16 i;
    for (i = 0; i < len; i++) UART1_SendByte(buf[i]);
}

void UART1_SendString(const char *str)
{
    while (*str) UART1_SendByte((u8)*str++);
}

u8 UART1_RecvByte(u8 *byte, u32 timeout_ms)
{
    u32 start = g_sys_tick;
    while ((u32)(g_sys_tick - start) < timeout_ms) {
        if (UART1_PopByte(byte)) return 1;
    }
    return 0;
}

u16 UART1_RecvBytes(u8 *buf, u16 max_len, u32 timeout_ms)
{
    u16 count = 0;
    u32 start = g_sys_tick;
    u32 last_byte = start;

    while ((u32)(g_sys_tick - start) < timeout_ms && count < max_len) {
        if (UART1_PopByte(&buf[count])) {
            count++;
            last_byte = g_sys_tick;
        } else if (count > 0 && (u32)(g_sys_tick - last_byte) >= 2u) {
            break;
        }
    }
    return count;
}

void UART1_Flush(void)
{
    volatile u32 dummy;
    uart1_rx_tail = uart1_rx_head;
    dummy = USART1->SR;
    dummy = USART1->DR;
    (void)dummy;
}
