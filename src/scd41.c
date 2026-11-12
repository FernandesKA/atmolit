#include "scd41.h"

#include "board.h"
#include "stm32f1xx_hal.h"

#define CMD_START_PERIODIC 0x21B1
#define CMD_STOP_PERIODIC  0x3F86
#define CMD_READ_MEAS      0xEC05
#define CMD_SERIAL         0x3682

// CRC-8, poly 0x31, init 0xFF (Sensirion application note).
static uint8_t crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0xFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x31U) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static int send_cmd(uint16_t cmd, uint32_t exec_ms)
{
    uint8_t buf[2] = {(uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF)};
    if (HAL_I2C_Master_Transmit(&SCD41_I2C_HANDLE, SCD41_I2C_ADDR << 1, buf, 2, 100) != HAL_OK) {
        return -1;
    }
    if (exec_ms > 0) {
        HAL_Delay(exec_ms);
    }
    return 0;
}

static int read_words(uint8_t *out, uint32_t words)
{
    if (HAL_I2C_Master_Receive(&SCD41_I2C_HANDLE, (SCD41_I2C_ADDR << 1) | 1, out,
                               (uint16_t)(words * 3), 100) != HAL_OK) {
        return -1;
    }
    for (uint32_t i = 0; i < words; i++) {
        if (crc8(&out[i * 3], 2) != out[i * 3 + 2]) {
            return -2;
        }
    }
    return 0;
}

int scd41_init(void)
{
    // The sensor needs up to 1 s after power-up before it answers on I2C.
    for (int i = 0; i < 10; i++) {
        if (HAL_I2C_IsDeviceReady(&SCD41_I2C_HANDLE, SCD41_I2C_ADDR << 1, 2, 50) == HAL_OK) {
            return 0;
        }
        HAL_Delay(100);
    }
    return -1;
}

volatile uint8_t g_i2c_scan[128];
volatile uint32_t g_scan_count;

void scd41_scan_bus(void)
{
    for (int a = 0; a < 128; a++) {
        g_i2c_scan[a] = 0;
    }
    for (int a = 0x08; a < 0x78; a++) {
        if (HAL_I2C_IsDeviceReady(&SCD41_I2C_HANDLE, (uint16_t)(a << 1), 1, 20) == HAL_OK) {
            g_i2c_scan[a] = 1;
        }
    }
    g_scan_count++;
}

int scd41_start_periodic(void)
{
    return send_cmd(CMD_START_PERIODIC, 2);
}

int scd41_stop_periodic(void)
{
    return send_cmd(CMD_STOP_PERIODIC, 500);
}

int scd41_read(scd41_measurement_t *out)
{
    uint8_t raw[9];
    if (out == NULL) {
        return -1;
    }
    if (send_cmd(CMD_READ_MEAS, 2) != 0) {
        return -1;
    }
    if (read_words(raw, 3) != 0) {
        return -2;
    }

    uint16_t co2 = (uint16_t)((raw[0] << 8) | raw[1]);
    uint16_t traw = (uint16_t)((raw[3] << 8) | raw[4]);
    uint16_t hraw = (uint16_t)((raw[6] << 8) | raw[7]);

    out->co2_ppm = co2;
    out->temperature_c = -45.0f + 175.0f * (float)traw / 65535.0f;
    out->humidity_rh = 100.0f * (float)hraw / 65535.0f;
    return 0;
}

int scd41_serial(uint64_t *serial)
{
    uint8_t raw[9];
    if (serial == NULL) {
        return -1;
    }
    if (send_cmd(CMD_SERIAL, 2) != 0) {
        return -1;
    }
    if (read_words(raw, 3) != 0) {
        return -2;
    }
    *serial = ((uint64_t)((raw[0] << 8) | raw[1]) << 32) |
              ((uint64_t)((raw[3] << 8) | raw[4]) << 16) |
              ((uint64_t)((raw[6] << 8) | raw[7]));
    return 0;
}
