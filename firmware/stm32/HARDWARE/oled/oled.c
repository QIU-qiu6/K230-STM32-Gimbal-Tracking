#include "oled.h"
#include "stm32f10x.h"
#include <string.h>

#define OLED_ADDR_WRITE  0x78u
#define OLED_SCL_PIN     GPIO_Pin_6
#define OLED_SDA_PIN     GPIO_Pin_7

static u8 g_oled_buffer[1024];

static void I2C_Delay(void)
{
    volatile u8 i;
    for (i = 0; i < 24; i++) { __NOP(); }
}

static void SCL_High(void) { GPIOB->BSRR = OLED_SCL_PIN; }
static void SCL_Low(void)  { GPIOB->BRR  = OLED_SCL_PIN; }
static void SDA_High(void) { GPIOB->BSRR = OLED_SDA_PIN; }
static void SDA_Low(void)  { GPIOB->BRR  = OLED_SDA_PIN; }

static void I2C_Start(void)
{
    SDA_High(); SCL_High(); I2C_Delay();
    SDA_Low(); I2C_Delay(); SCL_Low();
}

static void I2C_Stop(void)
{
    SDA_Low(); SCL_High(); I2C_Delay();
    SDA_High(); I2C_Delay();
}

static u8 I2C_WriteByte(u8 value)
{
    u8 i, nack;
    for (i = 0; i < 8; i++) {
        if (value & 0x80u) SDA_High(); else SDA_Low();
        I2C_Delay(); SCL_High(); I2C_Delay(); SCL_Low();
        value <<= 1;
    }
    SDA_High(); I2C_Delay();
    SCL_High(); I2C_Delay();
    nack = (GPIOB->IDR & OLED_SDA_PIN) ? 1 : 0;
    SCL_Low();
    return nack ? 0 : 1;
}

static u8 OLED_WriteCommand(u8 cmd)
{
    u8 ok;
    I2C_Start();
    ok = I2C_WriteByte(OLED_ADDR_WRITE);
    if (ok) ok = I2C_WriteByte(0x00);
    if (ok) ok = I2C_WriteByte(cmd);
    I2C_Stop();
    return ok;
}

static void Glyph5x7(char ch, u8 out[5])
{
    static const u8 digits[10][5] = {
        {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
        {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
        {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
        {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E}
    };
    static const u8 letters[26][5] = {
        {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},
        {0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},
        {0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
        {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},
        {0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},
        {0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
        {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},
        {0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},
        {0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
        {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},
        {0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},
        {0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
        {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43}
    };

    memset(out, 0, 5);
    if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
    if (ch >= '0' && ch <= '9') memcpy(out, digits[ch - '0'], 5);
    else if (ch >= 'A' && ch <= 'Z') memcpy(out, letters[ch - 'A'], 5);
    else if (ch == '-') { out[0]=out[1]=out[2]=out[3]=out[4]=0x08; }
    else if (ch == '+') { out[1]=0x08; out[2]=0x1C; out[3]=0x08; }
    else if (ch == '.') { out[2]=0x60; out[3]=0x60; }
    else if (ch == ':') { out[2]=0x36; out[3]=0x36; }
    else if (ch == '/') { out[0]=0x20; out[1]=0x10; out[2]=0x08; out[3]=0x04; out[4]=0x02; }
}

void OLED_Clear(void)
{
    memset(g_oled_buffer, 0, sizeof(g_oled_buffer));
}

void OLED_ShowLine(u8 row, const char *text)
{
    u8 x = 0;
    u8 glyph[5];
    u8 i;
    u16 base;

    if (row >= 8) return;
    base = (u16)row * 128u;
    memset(&g_oled_buffer[base], 0, 128);
    while (*text && x <= 122) {
        Glyph5x7(*text++, glyph);
        for (i = 0; i < 5; i++) g_oled_buffer[base + x + i] = glyph[i];
        x = (u8)(x + 6);
    }
}

u8 OLED_Refresh(void)
{
    u8 page, x, ok;
    for (page = 0; page < 8; page++) {
        if (!OLED_WriteCommand((u8)(0xB0u + page)) ||
            !OLED_WriteCommand(0x00) || !OLED_WriteCommand(0x10)) return 0;
        I2C_Start();
        ok = I2C_WriteByte(OLED_ADDR_WRITE);
        if (ok) ok = I2C_WriteByte(0x40);
        for (x = 0; ok && x < 128; x++) {
            ok = I2C_WriteByte(g_oled_buffer[(u16)page * 128u + x]);
        }
        I2C_Stop();
        if (!ok) return 0;
    }
    return 1;
}

u8 OLED_Init(void)
{
    GPIO_InitTypeDef gpio;
    static const u8 init_cmds[] = {
        0xAE,0x20,0x02,0xB0,0xC8,0x00,0x10,0x40,0x81,0x7F,
        0xA1,0xA6,0xA8,0x3F,0xA4,0xD3,0x00,0xD5,0x80,0xD9,
        0xF1,0xDA,0x12,0xDB,0x40,0x8D,0x14,0xAF
    };
    u8 i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin = OLED_SCL_PIN | OLED_SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &gpio);
    SCL_High(); SDA_High();

    for (i = 0; i < sizeof(init_cmds); i++) {
        if (!OLED_WriteCommand(init_cmds[i])) return 0;
    }
    OLED_Clear();
    OLED_ShowLine(0, "GIMBAL BOOT");
    return OLED_Refresh();
}
