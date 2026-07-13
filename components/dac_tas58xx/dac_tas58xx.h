#pragma once

#include "dac.h"

/**
 * TAS58xx (TAS5825M) DAC driver ops — register with dac_register() before
 * calling dac_init().
 */
extern const dac_ops_t dac_tas58xx_ops;

/** Sub level-trim limits (dB), relative to the master volume. */
#define TAS58XX_SUB_OFFSET_MIN_DB (-15.0f)
#define TAS58XX_SUB_OFFSET_MAX_DB (15.0f)

/**
 * Set the sub (PBTL mono) volume offset in dB, relative to the master volume.
 * Positive raises the bass level, negative lowers it. Clamped to
 * [TAS58XX_SUB_OFFSET_MIN_DB, TAS58XX_SUB_OFFSET_MAX_DB]. Safe to call before
 * dac_init(); the value is applied on the next volume update.
 */
void dac_tas58xx_set_sub_offset_db(float offset_db);

/** Get the current sub volume offset in dB. */
float dac_tas58xx_get_sub_offset_db(void);
