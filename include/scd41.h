#pragma once

#include <stdbool.h>
#include <stdint.h>

// Sensirion SCD41 / SCD40 / SCD4x, I2C address 0x62, max 100 kHz.

typedef struct {
    uint16_t co2_ppm;
    float temperature_c;
    float humidity_rh;
} scd41_measurement_t;

// All functions return 0 on success, negative on error.
int scd41_init(void);
int scd41_start_periodic(void);
int scd41_stop_periodic(void);
int scd41_read(scd41_measurement_t *out);
int scd41_serial(uint64_t *serial);

// Debug: full I2C scan, g_i2c_scan[addr] = 1 if a device ACKed.
void scd41_scan_bus(void);
extern volatile uint8_t g_i2c_scan[128];
