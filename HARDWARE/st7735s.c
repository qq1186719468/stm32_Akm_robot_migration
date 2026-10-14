#include "st7735s.h"

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#if (ST7735S_USE_SOFT_SPI == 0u)
#include "stm32f4xx_spi.h"
#endif
#include "delay.h"

/* Module-specific offsets are deliberately isolated for easy panel adjustment. */
#define ST7735S_COL_OFFSET  (0u)
#define ST7735S_ROW_OFFSET  (0u)

static void st7735s_init_control_gpio(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef gpio;

    /* Each control signal is initialized independently so the four macros in
     * st7735s.h may safely be moved to different GPIO ports later. */
    gpio.GPIO_Pin = pin;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(port, &gpio);
}

static void st7735s_write8(uint8_t value)
{
    uint8_t bit;

#if (ST7735S_USE_SOFT_SPI == 1u)
    /* 侧排针没有PA5，使用PC5/PA7模拟SPI；显示刷新只有1 Hz，CPU占用可接受。 */
    for (bit = 0u; bit < 8u; bit++)
    {
        GPIO_ResetBits(ST7735S_SCK_PORT, ST7735S_SCK_PIN);
        if ((value & 0x80u) != 0u)
        {
            GPIO_SetBits(ST7735S_MOSI_PORT, ST7735S_MOSI_PIN);
        }
        else
        {
            GPIO_ResetBits(ST7735S_MOSI_PORT, ST7735S_MOSI_PIN);
        }
        __NOP();
        GPIO_SetBits(ST7735S_SCK_PORT, ST7735S_SCK_PIN);
        __NOP();
        value <<= 1;
    }
    GPIO_ResetBits(ST7735S_SCK_PORT, ST7735S_SCK_PIN);
#else
    /* 若以后改用硬件SPI，只需将宏置0并恢复对应SPI外设配置。 */
    while (SPI_I2S_GetFlagStatus(ST7735S_SPI, SPI_I2S_FLAG_TXE) == RESET) {}
    SPI_I2S_SendData(ST7735S_SPI, value);
    while (SPI_I2S_GetFlagStatus(ST7735S_SPI, SPI_I2S_FLAG_BSY) == SET) {}
#endif
}

static void st7735s_command(uint8_t command)
{
    GPIO_ResetBits(ST7735S_CS_PORT, ST7735S_CS_PIN);
    GPIO_ResetBits(ST7735S_DC_PORT, ST7735S_DC_PIN);
    st7735s_write8(command);
    GPIO_SetBits(ST7735S_DC_PORT, ST7735S_DC_PIN);
    GPIO_SetBits(ST7735S_CS_PORT, ST7735S_CS_PIN);
}

static void st7735s_command_data(uint8_t command, const uint8_t *data, uint8_t count)
{
    uint8_t index;

    GPIO_ResetBits(ST7735S_CS_PORT, ST7735S_CS_PIN);
    GPIO_ResetBits(ST7735S_DC_PORT, ST7735S_DC_PIN);
    st7735s_write8(command);
    GPIO_SetBits(ST7735S_DC_PORT, ST7735S_DC_PIN);
    for (index = 0u; index < count; index++)
    {
        st7735s_write8(data[index]);
    }
    GPIO_SetBits(ST7735S_CS_PORT, ST7735S_CS_PIN);
}

static void st7735s_set_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    uint8_t column[4];
    uint8_t row[4];

    column[0] = 0u;
    column[1] = x0 + ST7735S_COL_OFFSET;
    column[2] = 0u;
    column[3] = x1 + ST7735S_COL_OFFSET;
    row[0] = 0u;
    row[1] = y0 + ST7735S_ROW_OFFSET;
    row[2] = 0u;
    row[3] = y1 + ST7735S_ROW_OFFSET;
    st7735s_command_data(0x2Au, column, 4u); /* CASET */
    st7735s_command_data(0x2Bu, row, 4u);    /* RASET */
    st7735s_command(0x2Cu);                   /* RAMWR */
}

static void st7735s_write_color(uint16_t color)
{
    st7735s_write8((uint8_t)(color >> 8));
    st7735s_write8((uint8_t)color);
}

void ST7735S_Init(void)
{
    GPIO_InitTypeDef gpio;
#if (ST7735S_USE_SOFT_SPI == 0u)
    SPI_InitTypeDef spi;
#endif
    static const uint8_t b1[] = {0x05u, 0x3Cu, 0x3Cu};
    static const uint8_t b3[] = {0x05u, 0x3Cu, 0x3Cu, 0x05u, 0x3Cu, 0x3Cu};
    static const uint8_t c0[] = {0x28u, 0x08u, 0x04u};
    static const uint8_t c2[] = {0x0Du, 0x00u};
    static const uint8_t c3[] = {0x8Du, 0x2Au};
    static const uint8_t c4[] = {0x8Du, 0xEEu};
    static const uint8_t e0[] = {0x04u,0x22u,0x07u,0x0Au,0x2Eu,0x30u,0x25u,0x2Au,
                                 0x28u,0x26u,0x2Eu,0x3Au,0x00u,0x01u,0x03u,0x13u};
    static const uint8_t e1[] = {0x04u,0x16u,0x06u,0x0Du,0x2Du,0x26u,0x23u,0x27u,
                                 0x27u,0x25u,0x2Du,0x3Bu,0x00u,0x01u,0x04u,0x13u};
    uint8_t value;

    /* Enable all GPIO banks that may be selected by the configurable control
     * macros.  This avoids a hidden clock dependency when a line is rewired. */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |
                           RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD |
                           RCC_AHB1Periph_GPIOE, ENABLE);
#if (ST7735S_USE_SOFT_SPI == 1u)
    /* 软件SPI两根线必须保持普通推挽输出，不能配置成SPI复用功能。 */
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Pin = ST7735S_SCK_PIN;
    GPIO_Init(ST7735S_SCK_PORT, &gpio);
    gpio.GPIO_Pin = ST7735S_MOSI_PIN;
    GPIO_Init(ST7735S_MOSI_PORT, &gpio);
    GPIO_ResetBits(ST7735S_SCK_PORT, ST7735S_SCK_PIN);
    GPIO_ResetBits(ST7735S_MOSI_PORT, ST7735S_MOSI_PIN);
#else
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    gpio.GPIO_Pin = ST7735S_SCK_PIN | ST7735S_MOSI_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ST7735S_SCK_PORT, &gpio);
    GPIO_PinAFConfig(ST7735S_SCK_PORT, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(ST7735S_MOSI_PORT, GPIO_PinSource7, GPIO_AF_SPI1);
#endif

    /* Control lines are outputs; initialize each selected port independently. */
    st7735s_init_control_gpio(ST7735S_CS_PORT, ST7735S_CS_PIN);
    st7735s_init_control_gpio(ST7735S_DC_PORT, ST7735S_DC_PIN);
    st7735s_init_control_gpio(ST7735S_RES_PORT, ST7735S_RES_PIN);
    st7735s_init_control_gpio(ST7735S_BLK_PORT, ST7735S_BLK_PIN);
    GPIO_SetBits(ST7735S_CS_PORT, ST7735S_CS_PIN);
    GPIO_SetBits(ST7735S_DC_PORT, ST7735S_DC_PIN);
    GPIO_SetBits(ST7735S_RES_PORT, ST7735S_RES_PIN);
    GPIO_ResetBits(ST7735S_BLK_PORT, ST7735S_BLK_PIN);

#if (ST7735S_USE_SOFT_SPI == 0u)
    SPI_I2S_DeInit(ST7735S_SPI);
    spi.SPI_Direction = SPI_Direction_1Line_Tx;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_1Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7u;
    SPI_Init(ST7735S_SPI, &spi);
    SPI_Cmd(ST7735S_SPI, ENABLE);
#endif

    /* Hardware reset followed by the panel's supplied ST7735S sequence. */
    GPIO_ResetBits(ST7735S_RES_PORT, ST7735S_RES_PIN);
    delay_ms(1u);
    GPIO_SetBits(ST7735S_RES_PORT, ST7735S_RES_PIN);
    delay_ms(120u);
    st7735s_command(0x11u); /* Sleep Out */
    delay_ms(120u);
    st7735s_command_data(0xB1u, b1, 3u);
    st7735s_command_data(0xB2u, b1, 3u);
    st7735s_command_data(0xB3u, b3, 6u);
    value = 0x03u;
    st7735s_command_data(0xB4u, &value, 1u);
    st7735s_command_data(0xC0u, c0, 3u);
    value = 0xC0u;
    st7735s_command_data(0xC1u, &value, 1u);
    st7735s_command_data(0xC2u, c2, 2u);
    st7735s_command_data(0xC3u, c3, 2u);
    st7735s_command_data(0xC4u, c4, 2u);
    value = 0x1Au;
    st7735s_command_data(0xC5u, &value, 1u);
    value = 0xC0u;
    st7735s_command_data(0x36u, &value, 1u); /* MADCTL: module supplied orientation */
    st7735s_command_data(0xE0u, e0, 16u);
    st7735s_command_data(0xE1u, e1, 16u);
    value = 0x05u;
    st7735s_command_data(0x3Au, &value, 1u); /* RGB565 */
    st7735s_command(0x29u);                  /* Display On */
    GPIO_SetBits(ST7735S_BLK_PORT, ST7735S_BLK_PIN);
    ST7735S_Fill(ST7735S_BLACK);
}

void ST7735S_Fill(uint16_t color)
{
    uint32_t count;

    st7735s_set_window(0u, 0u, ST7735S_WIDTH - 1u, ST7735S_HEIGHT - 1u);
    GPIO_ResetBits(ST7735S_CS_PORT, ST7735S_CS_PIN);
    GPIO_SetBits(ST7735S_DC_PORT, ST7735S_DC_PIN);
    for (count = 0u; count < ((uint32_t)ST7735S_WIDTH * ST7735S_HEIGHT); count++)
    {
        st7735s_write_color(color);
    }
    GPIO_SetBits(ST7735S_CS_PORT, ST7735S_CS_PIN);
}

/* 5x7 ASCII glyphs, sufficient for the diagnostic page and intentionally small. */
static const uint8_t s_font5x7[96][5] = {
    {0,0,0,0,0},{0,0,0x5F,0,0},{0,7,0,7,0},{0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,8,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},
    {0,5,3,0,0},{0,0x1C,0x22,0x41,0},{0,0x41,0x22,0x1C,0},{0x14,8,0x3E,8,0x14},
    {8,8,0x3E,8,8},{0,0x50,0x30,0,0},{8,8,8,8,8},{0,0x60,0x60,0,0},{0x20,0x10,8,4,2},
    {0x3E,0x51,0x49,0x45,0x3E},{0,0x42,0x7F,0x40,0},{0x42,0x61,0x51,0x49,0x46},
    {0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30},{1,0x71,9,5,3},{0x36,0x49,0x49,0x49,0x36},{6,0x49,0x49,0x29,0x1E},
    {0,0x36,0x36,0,0},{0,0x56,0x36,0,0},{8,0x14,0x22,0x41,0},{0x14,0x14,0x14,0x14,0x14},
    {0x41,0x22,0x14,8,0},{2,1,0x51,9,6},{0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},
    {0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41},{0x7F,9,9,9,1},{0x3E,0x41,0x49,0x49,0x7A},{0x7F,8,8,8,0x7F},
    {0,0x41,0x7F,0x41,0},{0x20,0x40,0x41,0x3F,1},{0x7F,8,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,2,0x0C,2,0x7F},{0x7F,4,8,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},{0x7F,9,9,9,6},
    {0x3E,0x41,0x51,0x21,0x5E},{0x7F,9,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},{1,1,0x7F,1,1},
    {0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,8,0x14,0x63},
    {7,8,0x70,8,7},{0x61,0x51,0x49,0x45,0x43},{0,0x7F,0x41,0x41,0},{2,4,8,0x10,0x20},
    {0,0x41,0x41,0x7F,0},{4,2,1,2,4},{0x40,0x40,0x40,0x40,0x40},{0,1,2,4,0},
    {0x20,0x54,0x54,0x54,0x78},{0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},{8,0x7E,9,1,2},{0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,8,4,4,0x78},{0,0x44,0x7D,0x40,0},{0x20,0x40,0x44,0x3D,0},{0x7F,0x10,0x28,0x44,0},
    {0,0x41,0x7F,0x40,0},{0x7C,4,0x18,4,0x78},{0x7C,8,4,4,0x78},{0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,8},{8,0x14,0x14,0x18,0x7C},{0x7C,8,4,4,8},{0x48,0x54,0x54,0x54,0x20},
    {4,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},{0x44,0x64,0x54,0x4C,0x44},{0,8,0x36,0x41,0},
    {0,0,0x7F,0,0},{0,0x41,0x36,8,0},{0x10,8,8,0x10,8},{0,0,0,0,0}
};

void ST7735S_DrawChar(uint8_t x, uint8_t y, char c,
                      uint16_t foreground, uint16_t background)
{
    uint8_t glyph_index;
    uint8_t column;
    uint8_t row;
    uint8_t bits;

    if ((x > (ST7735S_WIDTH - 6u)) || (y > (ST7735S_HEIGHT - 8u)))
    {
        return;
    }
    glyph_index = ((c >= 32) && (c <= 127)) ? (uint8_t)(c - 32) : 0u;
    st7735s_set_window(x, y, x + 5u, y + 7u);
    GPIO_ResetBits(ST7735S_CS_PORT, ST7735S_CS_PIN);
    GPIO_SetBits(ST7735S_DC_PORT, ST7735S_DC_PIN);
    for (row = 0u; row < 8u; row++)
    {
        for (column = 0u; column < 6u; column++)
        {
            if (column < 5u)
            {
                bits = s_font5x7[glyph_index][column];
            }
            st7735s_write_color((bits & (uint8_t)(1u << row)) ? foreground : background);
        }
    }
    GPIO_SetBits(ST7735S_CS_PORT, ST7735S_CS_PIN);
}
