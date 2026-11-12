#include "lcd_st7789.h"

#include <stdbool.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "board.h"
#include "stm32f1xx_hal.h"

static void cs_low(void)
{
    HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
}

static void cs_high(void)
{
    HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}

// Diagnostics readable through SWD.
volatile uint32_t g_lcd_spi_ok;
volatile uint32_t g_lcd_spi_err;
volatile uint32_t g_lcd_bands;

// ---------------------------------------------------------------------------
// DMA for SPI1_TX: DMA1 Channel 3
// ---------------------------------------------------------------------------
static DMA_HandleTypeDef s_hdma_tx;
static SemaphoreHandle_t s_dma_done;
static volatile bool s_dma_error;
static volatile bool s_dma_inited;

static void lcd_dma_init(void)
{
    if (s_dma_inited) {
        return;
    }
    __HAL_RCC_DMA1_CLK_ENABLE();

    s_hdma_tx.Instance = DMA1_Channel3;
    s_hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    s_hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    s_hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
    s_hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    s_hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    s_hdma_tx.Init.Mode = DMA_NORMAL;
    s_hdma_tx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&s_hdma_tx);
    __HAL_LINKDMA(&hspi1, hdmatx, s_hdma_tx);

    // Priority 5 = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, so the ISR may
    // call the FreeRTOS FromISR API.
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);

    s_dma_done = xSemaphoreCreateBinary();
    s_dma_inited = true;
}

void DMA1_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&s_hdma_tx);
}

void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi1);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    (void)hspi;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_dma_done, &woken);
    portYIELD_FROM_ISR(woken);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    (void)hspi;
    s_dma_error = true;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_dma_done, &woken);
    portYIELD_FROM_ISR(woken);
}

static void lcd_cmd(uint8_t cmd)
{
    HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET);
    cs_low();
    HAL_SPI_Transmit(&LCD_SPI_HANDLE, &cmd, 1, 100);
    cs_high();
}

static void lcd_data(const uint8_t *data, uint16_t len)
{
    if (len == 0) {
        return;
    }
    HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET);
    cs_low();
    HAL_SPI_Transmit(&LCD_SPI_HANDLE, (uint8_t *)data, len, 1000);
    cs_high();
}

static void lcd_cmd_data(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    lcd_cmd(cmd);
    lcd_data(data, len);
}

static void lcd_set_window(int x0, int y0, int x1, int y1)
{
    x0 += LCD_GAP_X;
    x1 += LCD_GAP_X;
    y0 += LCD_GAP_Y;
    y1 += LCD_GAP_Y;

    uint8_t buf[4];
    buf[0] = (uint8_t)(x0 >> 8);
    buf[1] = (uint8_t)x0;
    buf[2] = (uint8_t)(x1 >> 8);
    buf[3] = (uint8_t)x1;
    lcd_cmd_data(0x2A, buf, 4);  // CASET

    buf[0] = (uint8_t)(y0 >> 8);
    buf[1] = (uint8_t)y0;
    buf[2] = (uint8_t)(y1 >> 8);
    buf[3] = (uint8_t)y1;
    lcd_cmd_data(0x2B, buf, 4);  // RASET

    lcd_cmd(0x2C);  // RAMWR
}

void lcd_init(void)
{
    lcd_dma_init();

    // Hardware reset
    HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(120);

    lcd_cmd(0x01);  // SWRESET
    HAL_Delay(150);

    lcd_cmd(0x11);  // SLPOUT
    HAL_Delay(120);

    static const uint8_t colmod[] = {0x55};  // 16 bit/pixel
    lcd_cmd_data(0x3A, colmod, sizeof(colmod));

    static const uint8_t madctl[] = {0x00};  // no mirror, RGB order
    lcd_cmd_data(0x36, madctl, sizeof(madctl));

    static const uint8_t porctrl[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    lcd_cmd_data(0xB2, porctrl, sizeof(porctrl));

    static const uint8_t gctrl[] = {0x35};
    lcd_cmd_data(0xB7, gctrl, sizeof(gctrl));

    static const uint8_t vcoms[] = {0x19};
    lcd_cmd_data(0xBB, vcoms, sizeof(vcoms));

    static const uint8_t lcmctrl[] = {0x2C};
    lcd_cmd_data(0xC0, lcmctrl, sizeof(lcmctrl));

    static const uint8_t vdvvrhen[] = {0x01};
    lcd_cmd_data(0xC2, vdvvrhen, sizeof(vdvvrhen));

    static const uint8_t vrhs[] = {0x12};
    lcd_cmd_data(0xC3, vrhs, sizeof(vrhs));

    static const uint8_t vdvs[] = {0x20};
    lcd_cmd_data(0xC4, vdvs, sizeof(vdvs));

    static const uint8_t frctrl2[] = {0x0F};
    lcd_cmd_data(0xC6, frctrl2, sizeof(frctrl2));

    static const uint8_t pwctrl1[] = {0xA4, 0xA1};
    lcd_cmd_data(0xD0, pwctrl1, sizeof(pwctrl1));

    static const uint8_t pgamma[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F,
                                     0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
    lcd_cmd_data(0xE0, pgamma, sizeof(pgamma));

    static const uint8_t ngamma[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F,
                                     0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
    lcd_cmd_data(0xE1, ngamma, sizeof(ngamma));

    if (LCD_INVERT_COLOR) {
        lcd_cmd(0x21);  // INVON
    } else {
        lcd_cmd(0x20);  // INVOFF
    }

    lcd_cmd(0x13);  // NORON
    HAL_Delay(10);
    lcd_cmd(0x29);  // DISPON
    HAL_Delay(100);
}

void lcd_draw_band(int y, int h, const uint16_t *pixels)
{
    if (h <= 0) {
        return;
    }
    lcd_set_window(0, y, LCD_H_RES - 1, y + h - 1);
    HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET);
    cs_low();

    uint16_t len = (uint16_t)(LCD_H_RES * h * sizeof(uint16_t));
    bool ok = false;
    s_dma_error = false;
    if (HAL_SPI_Transmit_DMA(&LCD_SPI_HANDLE, (uint8_t *)pixels, len) == HAL_OK) {
        if (xSemaphoreTake(s_dma_done, pdMS_TO_TICKS(100)) == pdTRUE && !s_dma_error) {
            ok = true;
        }
    }
    if (ok) {
        g_lcd_spi_ok++;
    } else {
        g_lcd_spi_err++;
        HAL_SPI_Abort(&LCD_SPI_HANDLE);
    }

    // Make sure the last bit is out before releasing CS.
    uint32_t t0 = HAL_GetTick();
    while ((hspi1.Instance->SR & SPI_SR_BSY) != 0U && (HAL_GetTick() - t0) < 5U) {
    }

    cs_high();
    g_lcd_bands++;
}

void lcd_fill(uint16_t color)
{
    static uint16_t line[LCD_H_RES];
    for (int i = 0; i < LCD_H_RES; i++) {
        line[i] = color;
    }
    for (int y = 0; y < LCD_V_RES; y += LCD_BAND_LINES) {
        int h = (y + LCD_BAND_LINES > LCD_V_RES) ? (LCD_V_RES - y) : LCD_BAND_LINES;
        lcd_set_window(0, y, LCD_H_RES - 1, y + h - 1);
        HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET);
        cs_low();
        for (int r = 0; r < h; r++) {
            HAL_SPI_Transmit(&LCD_SPI_HANDLE, (uint8_t *)line,
                             (uint16_t)(LCD_H_RES * sizeof(uint16_t)), 1000);
        }
        cs_high();
    }
}
