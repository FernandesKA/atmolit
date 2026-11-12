/*
 * atmolit — STM32F103C8T6 (Blue Pill) + ST7789V3 240x280 + SCD41.
 *
 * FreeRTOS tasks:
 *   lcd  — panel init, splash screen, bottom area redrawn every 40 ms
 *   scd  — SCD41 bring-up, periodic measurement every 5 s
 *
 * RAM budget (20 KB total):
 *   4800 B  band buffer (240 x 10 x 2)
 *   8192 B  FreeRTOS heap (tasks, queues, mutex)
 *   ~2 KB   HAL handles, globals, main stack
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "board.h"
#include "font8x8.h"
#include "lcd_st7789.h"
#include "scd41.h"

// ---------------------------------------------------------------------------
// HAL handles (defined here, used by the drivers through board.h)
// ---------------------------------------------------------------------------
SPI_HandleTypeDef hspi1;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

// Console output: one whole line per call, serialised between tasks.
static SemaphoreHandle_t g_log_mutex;

static void log_printf(const char *fmt, ...)
{
    static char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0 || g_log_mutex == NULL) {
        return;
    }
    if (n > (int)sizeof(buf) - 1) {
        n = (int)sizeof(buf) - 1;
    }
    if (xSemaphoreTake(g_log_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        HAL_UART_Transmit(&LOG_UART_HANDLE, (uint8_t *)buf, (uint16_t)n, 100);
        xSemaphoreGive(g_log_mutex);
    }
}

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------
static uint16_t g_band[LCD_H_RES * LCD_BAND_LINES];

static SemaphoreHandle_t g_meas_mutex;
static bool g_sensor_ok;
static bool g_meas_valid;
static scd41_measurement_t g_meas;

// Watchdog and liveness counters (readable through SWD).
static IWDG_HandleTypeDef hiwdg;
volatile uint32_t g_scd_reads;
volatile uint32_t g_scd_ok_reads;
volatile uint32_t g_scd_stack_hw;
volatile uint32_t g_lcd_stack_hw;
volatile uint32_t g_sup_stack_hw;

// ---------------------------------------------------------------------------
// Colours (SWAP_BYTES because the panel expects big-endian RGB565)
// ---------------------------------------------------------------------------
static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t c = (uint16_t)(((r & 0xF8U) << 8) | ((g & 0xFCU) << 3) | (b >> 3));
#if LCD_SWAP_BYTES
    c = (uint16_t)((c >> 8) | (c << 8));
#endif
    return c;
}

#define COL_WHITE rgb565(255, 255, 255)
#define COL_GREY rgb565(150, 160, 175)
#define COL_ACCENT rgb565(80, 220, 190)
#define COL_RED rgb565(220, 40, 40)
#define COL_GREEN rgb565(30, 200, 60)
#define COL_BLUE rgb565(40, 80, 220)
#define COL_YELLOW rgb565(230, 210, 30)
#define COL_MAGENTA rgb565(200, 40, 180)
#define COL_BLACK rgb565(0, 0, 0)

// Dashboard palette
#define COL_TEXT rgb565(232, 238, 245)
#define COL_MUTED rgb565(143, 160, 181)
#define COL_CARD rgb565(22, 32, 46)
#define COL_BORDER rgb565(46, 62, 84)
#define COL_OK rgb565(53, 208, 127)
#define COL_WARN rgb565(242, 201, 76)
#define COL_BAD rgb565(242, 153, 74)
#define COL_ALARM rgb565(235, 87, 87)
#define COL_GRAPH_BG rgb565(14, 22, 34)
#define COL_GRAPH_FILL rgb565(24, 70, 55)
#define COL_GRAPH_GRID rgb565(32, 46, 62)

// ---------------------------------------------------------------------------
// Drawing helpers working on one band
// ---------------------------------------------------------------------------
static void fill_rect(uint16_t *buf, int by, int bh, int x, int y, int w, int h, uint16_t color)
{
    int x0 = x < 0 ? 0 : x;
    int x1 = x + w > LCD_H_RES ? LCD_H_RES : x + w;
    int y0 = y < by ? by : y;
    int y1 = y + h > by + bh ? by + bh : y + h;
    for (int yy = y0; yy < y1; yy++) {
        uint16_t *row = buf + (yy - by) * LCD_H_RES;
        for (int xx = x0; xx < x1; xx++) {
            row[xx] = color;
        }
    }
}

static void draw_text(uint16_t *buf, int by, int bh, int x, int y, const char *s, int scale,
                      uint16_t color)
{
    for (int i = 0; s[i] != '\0'; i++) {
        unsigned char ch = (unsigned char)s[i];
        if (ch > 127) {
            ch = '?';
        }
        for (int row = 0; row < 8; row++) {
            uint8_t bits = font8x8_basic[ch][row];
            if (bits == 0) {
                continue;
            }
            for (int col = 0; col < 8; col++) {
                if ((bits & (1U << col)) != 0) {
                    fill_rect(buf, by, bh, x + i * 8 * scale + col * scale, y + row * scale,
                              scale, scale, color);
                }
            }
        }
    }
}

static void draw_text_center(uint16_t *buf, int by, int bh, int y, const char *s, int scale,
                             uint16_t color)
{
    int w = (int)strlen(s) * 8 * scale;
    draw_text(buf, by, bh, (LCD_H_RES - w) / 2, y, s, scale, color);
}

// ---------------------------------------------------------------------------
// Dashboard
// ---------------------------------------------------------------------------
static volatile bool g_ui_dirty = true;
static bool g_live_dot;

// CO2 history for the trend graph: 48 samples x 5 s = 4 minutes.
#define GRAPH_SAMPLES 48
static uint16_t g_hist[GRAPH_SAMPLES];
static uint8_t g_hist_len;
static uint8_t g_hist_pos;

static void hist_push(uint16_t ppm)
{
    g_hist[g_hist_pos] = ppm;
    g_hist_pos = (uint8_t)((g_hist_pos + 1) % GRAPH_SAMPLES);
    if (g_hist_len < GRAPH_SAMPLES) {
        g_hist_len++;
    }
}

static uint16_t hist_get(int i)
{
    int idx = (g_hist_pos + GRAPH_SAMPLES - g_hist_len + i) % GRAPH_SAMPLES;
    return g_hist[idx];
}

static uint16_t co2_color(uint16_t ppm)
{
    if (ppm < 800) {
        return COL_OK;
    }
    if (ppm < 1200) {
        return COL_WARN;
    }
    if (ppm < 2000) {
        return COL_BAD;
    }
    return COL_ALARM;
}

static const char *co2_status(uint16_t ppm)
{
    if (ppm < 800) {
        return "GOOD";
    }
    if (ppm < 1200) {
        return "FAIR";
    }
    if (ppm < 2000) {
        return "POOR";
    }
    return "VENTILATE";
}

static void draw_card(uint16_t *buf, int by, int bh, int x, int y, int w, int h,
                      const char *title, float value, bool valid, const char *unit)
{
    fill_rect(buf, by, bh, x, y, w, h, COL_CARD);
    fill_rect(buf, by, bh, x, y, w, 1, COL_BORDER);
    fill_rect(buf, by, bh, x, y + h - 1, w, 1, COL_BORDER);
    fill_rect(buf, by, bh, x, y, 1, h, COL_BORDER);
    fill_rect(buf, by, bh, x + w - 1, y, 1, h, COL_BORDER);

    draw_text(buf, by, bh, x + 10, y + 10, title, 1, COL_MUTED);

    char ip[8] = "--";
    char fp[4] = "";
    if (valid) {
        int v10 = (int)(value * 10.0f + (value >= 0 ? 0.5f : -0.5f));
        snprintf(ip, sizeof(ip), "%d", v10 / 10);
        snprintf(fp, sizeof(fp), ".%d", abs(v10 % 10));
    }

    int ipw = (int)strlen(ip) * 8 * 3;
    draw_text(buf, by, bh, x + 10, y + 34, ip, 3, valid ? COL_TEXT : COL_MUTED);
    draw_text(buf, by, bh, x + 10 + ipw, y + 50, fp, 1, COL_MUTED);
    draw_text(buf, by, bh, x + 10 + ipw + 16, y + 50, unit, 1, COL_MUTED);
}

static void draw_text_right(uint16_t *buf, int by, int bh, int x_right, int y, const char *s,
                            int scale, uint16_t color)
{
    int w = (int)strlen(s) * 8 * scale;
    draw_text(buf, by, bh, x_right - w, y, s, scale, color);
}

static void draw_graph(uint16_t *buf, int by, int bh)
{
    const int gx = 10;
    const int gy = 194;
    const int gw = 220;
    const int gh = 80;

    if (by + bh <= gy || by >= gy + gh) {
        return;
    }

    const int px = 44;       // plot area, leaves room for the ppm labels
    const int py = 200;
    const int pw = 182;
    const int ph = 58;

    fill_rect(buf, by, bh, gx, gy, gw, 1, COL_BORDER);
    fill_rect(buf, by, bh, gx, gy + gh - 1, gw, 1, COL_BORDER);
    fill_rect(buf, by, bh, gx, gy, 1, gh, COL_BORDER);
    fill_rect(buf, by, bh, gx + gw - 1, gy, 1, gh, COL_BORDER);
    fill_rect(buf, by, bh, px, py, pw, ph, COL_GRAPH_BG);

    // horizontal reference lines: 800 and 1200 ppm
    int y800 = py + ph - 1 - (800 - 400) * (ph - 1) / 1600;
    int y1200 = py + ph - 1 - (1200 - 400) * (ph - 1) / 1600;
    fill_rect(buf, by, bh, px, y800, pw, 1, COL_GRAPH_GRID);
    fill_rect(buf, by, bh, px, y1200, pw, 1, COL_GRAPH_GRID);

    // Y axis labels (right aligned, left of the plot)
    static const uint16_t levels[] = {2000, 1200, 800, 400};
    for (unsigned i = 0; i < sizeof(levels) / sizeof(levels[0]); i++) {
        int ly = py + ph - 1 - ((int)levels[i] - 400) * (ph - 1) / 1600;
        char lab[6];
        snprintf(lab, sizeof(lab), "%u", (unsigned)levels[i]);
        int ly_text = ly - 4;
        if (ly_text < py - 2) {
            ly_text = py - 2;
        }
        if (ly_text > py + ph - 6) {
            ly_text = py + ph - 6;
        }
        draw_text_right(buf, by, bh, px - 4, ly_text, lab, 1, COL_MUTED);
    }

    // X axis labels: -4m / -2m / now
    draw_text(buf, by, bh, px, py + ph + 4, "-4m", 1, COL_MUTED);
    draw_text(buf, by, bh, px + pw / 2 - 12, py + ph + 4, "-2m", 1, COL_MUTED);
    draw_text_right(buf, by, bh, px + pw, py + ph + 4, "now", 1, COL_MUTED);

    if (g_hist_len == 0) {
        return;
    }

    // The window is always 4 minutes wide: with fewer samples the line grows
    // from the right edge, so the -4m/-2m/now labels stay truthful.
    uint16_t line_color = co2_color(hist_get(g_hist_len - 1));

    for (int i = 0; i < g_hist_len; i++) {
        int x0 = (pw - 1) - (g_hist_len - 1 - i) * (pw - 1) / (GRAPH_SAMPLES - 1);
        int x1 = (i + 1 < g_hist_len)
                     ? (pw - 1) - (g_hist_len - 2 - i) * (pw - 1) / (GRAPH_SAMPLES - 1)
                     : x0 + 1;

        int ppm0 = (int)hist_get(i);
        if (ppm0 < 400) {
            ppm0 = 400;
        }
        if (ppm0 > 2000) {
            ppm0 = 2000;
        }
        int y0 = py + ph - 1 - (ppm0 - 400) * (ph - 1) / 1600;

        int y1 = y0;
        if (i + 1 < g_hist_len) {
            int ppm1 = (int)hist_get(i + 1);
            if (ppm1 < 400) {
                ppm1 = 400;
            }
            if (ppm1 > 2000) {
                ppm1 = 2000;
            }
            y1 = py + ph - 1 - (ppm1 - 400) * (ph - 1) / 1600;
        }

        int span = x1 - x0;
        if (span < 1) {
            span = 1;
        }
        for (int x = x0; x <= x1 && x < pw; x++) {
            int yv = y0 + (y1 - y0) * (x - x0) / span;
            fill_rect(buf, by, bh, px + x, yv + 1, 1, (py + ph) - yv - 1, COL_GRAPH_FILL);
            fill_rect(buf, by, bh, px + x, yv, 1, 2, line_color);
        }
    }
}

static void render_band(int by, int bh)
{
    uint16_t *buf = g_band;

    // background: dark, slightly lighter towards the bottom
    for (int row = 0; row < bh; row++) {
        int t = (by + row) * 255 / (LCD_V_RES - 1);
        uint8_t r = (uint8_t)(11 + 7 * t / 255);
        uint8_t g = (uint8_t)(18 + 14 * t / 255);
        uint8_t b = (uint8_t)(32 + 16 * t / 255);
        uint16_t c = rgb565(r, g, b);
        uint16_t *line = buf + row * LCD_H_RES;
        for (int x = 0; x < LCD_H_RES; x++) {
            line[x] = c;
        }
    }

    scd41_measurement_t m = {0};
    bool valid = false;
    if (xSemaphoreTake(g_meas_mutex, 0) == pdTRUE) {
        m = g_meas;
        valid = g_meas_valid;
        xSemaphoreGive(g_meas_mutex);
    }

    draw_text_center(buf, by, bh, 4, "AIR MONITOR", 1, COL_MUTED);

    // 1 Hz heartbeat dot, top-right corner
    if (g_live_dot) {
        fill_rect(buf, by, bh, LCD_H_RES - 10, 0, 4, 4, COL_OK);
    }

    // big CO2 number + unit
    char num[8];
    if (valid) {
        snprintf(num, sizeof(num), "%u", (unsigned)m.co2_ppm);
    } else {
        snprintf(num, sizeof(num), "----");
    }
    uint16_t color = valid ? co2_color(m.co2_ppm) : COL_MUTED;
    int num_w = (int)strlen(num) * 8 * 5;
    int ppm_w = 3 * 8;
    int x = (LCD_H_RES - (num_w + 8 + ppm_w)) / 2;
    draw_text(buf, by, bh, x, 18, num, 5, color);
    draw_text(buf, by, bh, x + num_w + 8, 46, "ppm", 1, COL_MUTED);

    draw_text_center(buf, by, bh, 66, valid ? co2_status(m.co2_ppm) : "NO DATA", 2, color);

    // quality scale: 400 .. 2000 ppm
    int gx = 16;
    int gy = 96;
    int gw = LCD_H_RES - 32;
    int gh = 10;
    fill_rect(buf, by, bh, gx, gy, gw / 4, gh, COL_OK);
    fill_rect(buf, by, bh, gx + gw / 4, gy, gw / 4, gh, COL_WARN);
    fill_rect(buf, by, bh, gx + gw / 2, gy, gw / 2, gh, COL_BAD);

    int pos = 0;
    if (valid) {
        pos = ((int)m.co2_ppm - 400) * gw / 1600;
        if (pos < 0) {
            pos = 0;
        }
        if (pos > gw - 3) {
            pos = gw - 3;
        }
    }
    fill_rect(buf, by, bh, gx + pos, gy - 3, 3, gh + 6, COL_TEXT);
    draw_text(buf, by, bh, gx, gy + 14, "400", 1, COL_MUTED);
    draw_text(buf, by, bh, gx + gw - 24, gy + 14, "2000", 1, COL_MUTED);

    // temperature and humidity cards
    draw_card(buf, by, bh, 10, 126, 105, 64, "TEMP", m.temperature_c, valid, "C");
    draw_card(buf, by, bh, 125, 126, 105, 64, "HUM", m.humidity_rh, valid, "%");

    // trend graph
    draw_graph(buf, by, bh);
}

static void draw_all(void)
{
    for (int y = 0; y < LCD_V_RES; y += LCD_BAND_LINES) {
        int h = (y + LCD_BAND_LINES > LCD_V_RES) ? (LCD_V_RES - y) : LCD_BAND_LINES;
        render_band(y, h);
        lcd_draw_band(y, h, g_band);
    }
}

// ---------------------------------------------------------------------------
// Tasks
// ---------------------------------------------------------------------------
static void display_task(void *arg)
{
    (void)arg;
    lcd_init();

#if LCD_SELF_TEST_COLORS
    const uint16_t colors[] = {COL_RED, COL_GREEN, COL_BLUE, COL_WHITE, COL_BLACK};
    unsigned idx = 0;
    for (;;) {
        for (int y = 0; y < LCD_V_RES; y += LCD_BAND_LINES) {
            for (int i = 0; i < LCD_H_RES * LCD_BAND_LINES; i++) {
                g_band[i] = colors[idx];
            }
            lcd_draw_band(y, LCD_BAND_LINES, g_band);
        }
        idx = (idx + 1) % (sizeof(colors) / sizeof(colors[0]));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif

    draw_all();
    log_printf("[lcd] ST7789 240x280 initialised\r\n");

    uint32_t ticks = 0;
    for (;;) {
        if (g_ui_dirty) {
            g_ui_dirty = false;
            draw_all();
        }
        if (++ticks % 10 == 0) {
            // 1 Hz heartbeat: a dot in the header and the LED. This also proves
            // that the panel is still accepting data between sensor samples.
            g_live_dot = !g_live_dot;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            render_band(0, LCD_BAND_LINES);
            lcd_draw_band(0, LCD_BAND_LINES, g_band);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        g_lcd_stack_hw = uxTaskGetStackHighWaterMark(NULL);
    }
}

// ---------------------------------------------------------------------------
// Watchdog supervisor: feeds the IWDG only while both worker tasks progress.
// No heartbeat from the display task, or no sensor poll for 20 s, stops the
// feeding and the IWDG resets the MCU in ~4 s.
// ---------------------------------------------------------------------------
static void supervisor_task(void *arg)
{
    (void)arg;
    uint32_t last_bands = g_lcd_bands;
    uint32_t last_reads = g_scd_reads;
    uint32_t last_ok = g_scd_ok_reads;
    int scd_stall = 0;
    int no_data = 0;
    int display_stall = 0;

    HAL_IWDG_Refresh(&hiwdg);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        bool alive = true;

        if (g_lcd_bands == last_bands) {
            // Allow a few seconds: the heartbeat and this check both run at 1 Hz
            // and can drift out of phase.
            if (++display_stall > 5) {
                alive = false;
            }
        } else {
            display_stall = 0;
        }
        last_bands = g_lcd_bands;

        if (g_scd_reads != last_reads) {
            last_reads = g_scd_reads;
            scd_stall = 0;
        } else if (++scd_stall > 20) {
            alive = false;  // no sensor attempt for 20 s
        }

        // Sensor detected but no fresh data for 60 s: I2C is wedged, reboot.
        if (g_scd_ok_reads != last_ok) {
            last_ok = g_scd_ok_reads;
            no_data = 0;
        } else if (g_sensor_ok && ++no_data > 60) {
            alive = false;
        }

        if (alive) {
            HAL_IWDG_Refresh(&hiwdg);
        }
        // Otherwise stop feeding and let the IWDG reset the MCU.

        static int tick10;
        if (++tick10 >= 10) {
            tick10 = 0;
            g_sup_stack_hw = uxTaskGetStackHighWaterMark(NULL);
            log_printf("[sup] up %lus bands %lu scd %lu/%lu heap %u sup_hw %lu\r\n",
                   (unsigned long)(xTaskGetTickCount() / 1000U), (unsigned long)g_lcd_bands,
                   (unsigned long)g_scd_ok_reads, (unsigned long)g_scd_reads,
                   (unsigned)xPortGetFreeHeapSize(), (unsigned long)g_sup_stack_hw);
        }
    }
}

static void sensor_task(void *arg)
{
    (void)arg;

    for (;;) {
        scd41_scan_bus();  // debug: results readable through SWD
        if (scd41_init() == 0) {
            g_sensor_ok = true;
            uint64_t serial = 0;
            if (scd41_serial(&serial) == 0) {
                log_printf("[scd] SCD41 serial %08lX%04lX\r\n", (unsigned long)(serial >> 16),
                       (unsigned long)(serial & 0xFFFF));
            }
            if (scd41_start_periodic() == 0) {
                log_printf("[scd] periodic measurement started (first value in ~5 s)\r\n");
            }

            for (;;) {
                vTaskDelay(pdMS_TO_TICKS(5000));
                scd41_measurement_t m;
                if (scd41_read(&m) == 0) {
                    if (xSemaphoreTake(g_meas_mutex, portMAX_DELAY) == pdTRUE) {
                        g_meas = m;
                        g_meas_valid = true;
                        xSemaphoreGive(g_meas_mutex);
                    }
                    g_ui_dirty = true;
                    hist_push(m.co2_ppm);
                    g_scd_ok_reads++;
                    log_printf("[scd] CO2 %u ppm, T %d.%d C, RH %d.%d %%\r\n", (unsigned)m.co2_ppm,
                           (int)m.temperature_c, (int)(m.temperature_c * 10) % 10,
                           (int)m.humidity_rh, (int)(m.humidity_rh * 10) % 10);
                } else {
                    log_printf("[scd] read failed\r\n");
                    // A stuck bus can hold SDA low; re-init the peripheral.
                    static int fails;
                    if (++fails >= 3) {
                        fails = 0;
                        log_printf("[scd] I2C stuck, re-initialising\r\n");
                        HAL_I2C_DeInit(&hi2c1);
                        HAL_I2C_Init(&hi2c1);
                    }
                }
                g_scd_reads++;
                g_scd_stack_hw = uxTaskGetStackHighWaterMark(NULL);
            }
        }

        g_sensor_ok = false;
        log_printf("[scd] sensor not found on I2C1 (PB6/PB7), retrying\r\n");
        g_scd_reads++;
        g_scd_stack_hw = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

// ---------------------------------------------------------------------------
// Hardware init
// ---------------------------------------------------------------------------
static void Error_Handler(void)
{
    for (;;) {
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL = RCC_PLL_MUL9;  // 8 MHz * 9 = 72 MHz
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        // Many clones have a bad or missing 8 MHz crystal: fall back to the
        // internal oscillator, 8 MHz / 2 * 16 = 64 MHz.
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        osc.HSEState = RCC_HSE_OFF;
        osc.HSIState = RCC_HSI_ON;
        osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        osc.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
        osc.PLL.PLLMUL = RCC_PLL_MUL16;
        if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
            Error_Handler();
        }
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                    RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;  // 36 MHz
    clk.APB2CLKDivider = RCC_HCLK_DIV1;  // 72 MHz
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

static void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    // PA2 RST, PA3 DC, PA4 CS
    gpio.Pin = LCD_RST_PIN | LCD_DC_PIN | LCD_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LCD_GPIO_PORT, &gpio);
    HAL_GPIO_WritePin(LCD_GPIO_PORT, LCD_CS_PIN | LCD_DC_PIN | LCD_RST_PIN, GPIO_PIN_SET);

    // PA5 SCK, PA7 MOSI
    gpio.Pin = LCD_SCK_PIN | LCD_MOSI_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LCD_GPIO_PORT, &gpio);

    // PB6 SCL, PB7 SDA (I2C1)
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    // PA9 TX, PA10 RX (USART1)
    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    // PC13: onboard LED, active low. Used as a heartbeat.
    gpio.Pin = GPIO_PIN_13;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  // off
}

static void SPI1_Init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = LCD_SPI_PRESCALER;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

static void I2C1_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

static void USART1_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

// printf() -> USART1
int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&LOG_UART_HANDLE, (uint8_t *)ptr, (uint16_t)len, 100);
    return len;
}

// ---------------------------------------------------------------------------
// FreeRTOS hooks
// ---------------------------------------------------------------------------
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    for (;;) {
    }
}

void vApplicationMallocFailedHook(void)
{
    for (;;) {
    }
}

// ---------------------------------------------------------------------------
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    USART1_Init();
    SPI1_Init();
    I2C1_Init();

    // newlib buffers stdout when it is not a tty: flush every printf.
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("\r\n\r\natmolit: STM32F103C8T6 + ST7789V3 240x280 + SCD41\r\n");
    printf("SYSCLK %lu Hz, FreeRTOS %s\r\n", (unsigned long)SystemCoreClock,
           tskKERNEL_VERSION_NUMBER);

    uint32_t csr = RCC->CSR;
    printf("reset cause: %s%s%s%s%s%s\r\n",
           (csr & RCC_CSR_PORRSTF) ? "POR " : "",
           (csr & RCC_CSR_PINRSTF) ? "PIN " : "",
           (csr & RCC_CSR_SFTRSTF) ? "SW " : "",
           (csr & RCC_CSR_IWDGRSTF) ? "IWDG " : "",
           (csr & RCC_CSR_WWDGRSTF) ? "WWDG " : "",
           (csr & RCC_CSR_LPWRRSTF) ? "LPWR " : "");
    __HAL_RCC_CLEAR_RESET_FLAGS();

    // 3 quick blinks: proof of life before the scheduler starts.
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(100);
    }

    g_meas_mutex = xSemaphoreCreateMutex();
    if (g_meas_mutex == NULL) {
        Error_Handler();
    }

    g_log_mutex = xSemaphoreCreateMutex();
    if (g_log_mutex == NULL) {
        Error_Handler();
    }

    xTaskCreate(display_task, "lcd", 320, NULL, 2, NULL);
    xTaskCreate(sensor_task, "scd", 384, NULL, 3, NULL);
    xTaskCreate(supervisor_task, "sup", 320, NULL, 1, NULL);

    // IWDG last: ~4 s at LSI 40 kHz, frozen while the core is halted by a
    // debugger so SWD inspection does not reset the board.
    __HAL_DBGMCU_FREEZE_IWDG();
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 2500;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }

    vTaskStartScheduler();

    Error_Handler();
    return 0;
}
