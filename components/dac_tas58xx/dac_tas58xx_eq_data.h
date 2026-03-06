#pragma once
/**
 * TAS5825M EQ biquad address tables (Book 0xAA).
 *
 * Band definitions match the mrtoy-me/esphome-tas58xx reference.
 * Coefficients are computed at runtime — see calc_peaking_biquad().
 */

#include <stdint.h>

/* ─── TAS5825M EQ biquad address table (Book 0xAA) ───
 * Each entry: { page, sub_address }
 * From mrtoy-me/esphome-tas58xx reference for TAS5825M.
 */
typedef struct {
  uint8_t page;
  uint8_t sub_addr;
} eq_bq_addr_t;

static const eq_bq_addr_t eq_left_addr[15] = {
    {0x01, 0x30}, // BQ1  Left  -  20 Hz
    {0x01, 0x44}, // BQ2  Left  -  31.5 Hz
    {0x01, 0x58}, // BQ3  Left  -  50 Hz
    {0x01, 0x6C}, // BQ4  Left  -  80 Hz
    {0x02, 0x08}, // BQ5  Left  - 125 Hz
    {0x02, 0x1C}, // BQ6  Left  - 200 Hz
    {0x02, 0x30}, // BQ7  Left  - 315 Hz
    {0x02, 0x44}, // BQ8  Left  - 500 Hz
    {0x02, 0x58}, // BQ9  Left  - 800 Hz
    {0x02, 0x6C}, // BQ10 Left  - 1250 Hz
    {0x03, 0x08}, // BQ11 Left  - 2000 Hz
    {0x03, 0x1C}, // BQ12 Left  - 3150 Hz
    {0x03, 0x30}, // BQ13 Left  - 5000 Hz
    {0x03, 0x44}, // BQ14 Left  - 8000 Hz
    {0x03, 0x58}, // BQ15 Left  - 16000 Hz
};

static const eq_bq_addr_t eq_right_addr[15] = {
    {0x03, 0x6C}, // BQ1  Right -  20 Hz
    {0x04, 0x08}, // BQ2  Right -  31.5 Hz
    {0x04, 0x1C}, // BQ3  Right -  50 Hz
    {0x04, 0x30}, // BQ4  Right -  80 Hz
    {0x04, 0x44}, // BQ5  Right - 125 Hz
    {0x04, 0x58}, // BQ6  Right - 200 Hz
    {0x04, 0x6C}, // BQ7  Right - 315 Hz
    {0x05, 0x08}, // BQ8  Right - 500 Hz
    {0x05, 0x1C}, // BQ9  Right - 800 Hz
    {0x05, 0x30}, // BQ10 Right - 1250 Hz
    {0x05, 0x44}, // BQ11 Right - 2000 Hz
    {0x05, 0x58}, // BQ12 Right - 3150 Hz
    {0x05, 0x6C}, // BQ13 Right - 5000 Hz
    {0x06, 0x08}, // BQ14 Right - 8000 Hz
    {0x06, 0x1C}, // BQ15 Right - 16000 Hz
};
