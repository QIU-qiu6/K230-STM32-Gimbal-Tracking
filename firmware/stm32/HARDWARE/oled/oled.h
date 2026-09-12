#ifndef __OLED_H__
#define __OLED_H__

#include "sys.h"

/* SSD1306 128x64, I2C address 0x3C, PB6=SCL, PB7=SDA. */
u8 OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowLine(u8 row, const char *text);
u8 OLED_Refresh(void);

#endif
