#ifndef __ST7735S_H
#define __ST7735S_H

/* ST7735S 1.8寸 128x160、四线SPI显示屏驱动。 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 模块排针顺序：1 GND、2 VCC、3 SCL、4 SDA、5 RES、6 DC、7 CS、8 BLK。 */
#define ST7735S_WIDTH       (128u)
#define ST7735S_HEIGHT      (160u)

/* 这些引脚未被迁移工程现有外设占用。若实际接线不同，只需修改本段宏；
 * USART2/3、编码器和舵机引脚不要改动。实物侧排针没有PA5，因此采用
 * GPIO模拟SPI：PC5=SCL、PA7=SDA。四根控制线使用PC0..PC3。 */
#define ST7735S_USE_SOFT_SPI (1u)
#define ST7735S_SPI         SPI1
#define ST7735S_SCK_PORT    GPIOC
#define ST7735S_SCK_PIN     GPIO_Pin_5       /* 软件SPI时钟 */
#define ST7735S_MOSI_PORT   GPIOA
#define ST7735S_MOSI_PIN    GPIO_Pin_7       /* 软件SPI数据 */
#define ST7735S_CS_PORT     GPIOC
#define ST7735S_CS_PIN      GPIO_Pin_0       /* 片选，低有效 */
#define ST7735S_DC_PORT     GPIOC
#define ST7735S_DC_PIN      GPIO_Pin_1       /* 数据/命令：0命令，1数据 */
#define ST7735S_RES_PORT    GPIOC
#define ST7735S_RES_PIN     GPIO_Pin_2       /* 复位，低有效 */
#define ST7735S_BLK_PORT    GPIOC
#define ST7735S_BLK_PIN     GPIO_Pin_3       /* 背光使能，高电平打开 */

/* Common RGB565 colors. */
#define ST7735S_BLACK       (0x0000u)
#define ST7735S_BLUE        (0x001Fu)
#define ST7735S_RED         (0xF800u)
#define ST7735S_GREEN       (0x07E0u)
#define ST7735S_CYAN        (0x07FFu)
#define ST7735S_YELLOW      (0xFFE0u)
#define ST7735S_WHITE       (0xFFFFu)

void ST7735S_Init(void);
void ST7735S_Fill(uint16_t color);
void ST7735S_DrawChar(uint8_t x, uint8_t y, char c,
                      uint16_t foreground, uint16_t background);

#ifdef __cplusplus
}
#endif

#endif /* __ST7735S_H */
