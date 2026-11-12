#pragma once

#include <stdint.h>

// ST7789 240x280, 4-wire SPI, RGB565 (big endian on the wire).
// All buffers are uint16_t in CPU (little endian) order; the driver swaps bytes
// when LCD_SWAP_BYTES is set in board.h.

void lcd_init(void);
void lcd_draw_band(int y, int h, const uint16_t *pixels);
void lcd_fill(uint16_t color);
