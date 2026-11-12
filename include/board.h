// Board wiring for STM32F103C8T6 (Blue Pill) + ST7789V3 240x280 + SCD41.
// Change the pins here, everything else follows.

#pragma once

#include "stm32f1xx_hal.h"

// Handles are defined in src/main.c
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

// ---------------------------------------------------------------------------
// ST7789 240x280 on SPI1 (PA5 = SCK, PA7 = MOSI)
// ---------------------------------------------------------------------------
#define LCD_SPI_HANDLE hspi1
#define LCD_GPIO_PORT GPIOA

#define LCD_SCK_PIN GPIO_PIN_5   // PA5, SPI1_SCK
#define LCD_MOSI_PIN GPIO_PIN_7  // PA7, SPI1_MOSI
#define LCD_CS_PIN GPIO_PIN_4    // PA4, software CS
#define LCD_DC_PIN GPIO_PIN_3    // PA3
#define LCD_RST_PIN GPIO_PIN_2   // PA2
// Backlight is wired straight to 3V3: a GPIO cannot source 60-100 mA.

#define LCD_H_RES 240
#define LCD_V_RES 280

// The glass is 240x280 inside the 240x320 GRAM.
#define LCD_GAP_X 0
#define LCD_GAP_Y 20

#define LCD_INVERT_COLOR 1
#define LCD_SWAP_BYTES 1

// Bring-up test: full-screen colour cycling, ignores the UI.
#define LCD_SELF_TEST_COLORS 0

// 0 = blocking HAL_SPI_Transmit (checked, stable), 1 = DMA1 Channel 3.
// The DMA path is implemented and works, but on this panel/wiring it locks up
// after a while, so it is off by default.
#define LCD_USE_DMA 0

// SPI1 = 36 MHz at 72 MHz APB2 (divide by 2). With blocking transfers this is
// reliable; with DMA the panel locked up at 36 and 18 MHz, use prescaler 4/8
// there.
#define LCD_SPI_PRESCALER SPI_BAUDRATEPRESCALER_2

// Off-screen band buffer: 240 x 4 x 2 = 1920 bytes. The "C8" clones very often
// carry a C6 die with only 10 KB of SRAM, so the budget is tight.
#define LCD_BAND_LINES 4

// ---------------------------------------------------------------------------
// SCD41 on I2C1 (PB6 = SCL, PB7 = SDA), 100 kHz, address 0x62
// ---------------------------------------------------------------------------
#define SCD41_I2C_HANDLE hi2c1
#define SCD41_I2C_ADDR 0x62

// ---------------------------------------------------------------------------
// Console: USART1, PA9 = TX, PA10 = RX, 115200 8N1
// ---------------------------------------------------------------------------
#define LOG_UART_HANDLE huart1
