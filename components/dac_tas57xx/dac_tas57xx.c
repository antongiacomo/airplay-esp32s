/**
 * Implementation of control interface to TI TAX57xx DAC/Amp chips
 * tas5754m datasheet:
 * https://www.ti.com/lit/ds/symlink/tas5754m.pdf
 */

#include "dac_tas57xx.h"
#include "board_utils.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define TAS575x (0x98 >> 1)
#define TAS578x (0x90 >> 1)

// TAS578x device ID register (Book 0, Page 0)
#define TAS578x_REG_DEVICE_ID 0x67

#define I2C_TIMEOUT    100
#define I2C_LINE_SPEED 100000

static const char TAG[] = "TAS57xx DAC";

struct tas57xx_cmd_s {
  uint8_t reg;
  uint8_t value;
};

// Registers applied after the HF config (not covered by the HF flow).
// HF exits standby unmuted, so mute first to prevent pop.
static const struct tas57xx_cmd_s tas57xx_init_seq[] = {
    {0x00, 0x00}, // select page 0
    {0x03, 0x11}, // mute both channels before any other change
    {0x0d, 0x10}, // use SCK for PLL
    {0x25, 0x08}, // ignore SCK halt
    {0x08, 0x10}, // Mute control enable (GPIO3)
    {0x54, 0x02}, // Mute output control
    {0x3D, 0x6C}, // Set chan B volume -70dB
    {0x3E, 0x6C}, // Set chan A volume -70dB
    {0xff, 0xff}  // end of table
};

// Commands available - care to match ordinal with struct below
typedef enum {
  TAS57XX_ACTIVE = 0,
  TAS57XX_STANDBY,
  TAS57XX_DOWN,
  TAS57XX_ANALOGUE_OFF,
  TAS57XX_ANALOGUE_ON,
  TAS57XX_SET_VOLUME_A_L,
  TAS57XX_SET_VOLUME_B_R,
  TAS57XX_MUTE,
  TAS57XX_UNMUTE,
} tas57xx_cmd_e;

static const struct tas57xx_cmd_s tas57xx_cmd[] = {
    {0x02, 0x00}, // TAS57XX_ACTIVE
    {0x02, 0x10}, // TAS57XX_STANDBY
    {0x02, 0x01}, // TAS57XX_DOWN
    {0x56, 0x10}, // TAS57XX_ANALOGUE_OFF
    {0x56, 0x00}, // TAS57XX_ANALOGUE_ON
    {0x3E, 0x30}, // TAS57XX_SET_VOLUME_A_L - Channel A
    {0x3D, 0x30}, // TAS57XX_SET_VOLUME_B_R - Channel B
    {0x03, 0x11}, // TAS57XX_MUTE (BA)
    {0x03, 0x00}, // TAS57XX_UNMUTE (BA)
};

#define TAS57XX_MAX_DEVICES 2

typedef struct {
  uint8_t addr;                   // 7-bit I2C address
  i2c_master_dev_handle_t handle; // per-device I2C handle
  uint8_t *hf_buf;                // cached hybrid flow (NULL if none)
  long hf_size;
  bool is_sub; // true for index > 0 (sub / .1 channel)
} tas57xx_dev_t;

static tas57xx_dev_t s_devs[TAS57XX_MAX_DEVICES];
static int s_dev_count = 0;
static i2c_master_bus_handle_t s_bus_handle = NULL;
static dac_power_mode_t s_power_state = DAC_POWER_OFF;
static SemaphoreHandle_t s_dac_mutex = NULL;

// Sub level trim (dB) added to the master volume for sub devices, and the
// cached master volume so the trim can be re-applied on its own.
static float s_sub_offset_db = 0.0f;
static float s_last_airplay_db = -15.0f;

// Candidate TAS575x addresses (ADR strap 0x98/0x9A/0x9C/0x9E >> 1).
// 0x4C is always treated as the mains (L/R) device at index 0.
static const uint8_t tas575x_addrs[] = {TAS575x, 0x4D, 0x4E, 0x4F};

static esp_err_t write_cmd(i2c_master_dev_handle_t handle, tas57xx_cmd_e cmd,
                           ...);
static int tas57xx_detect_all(i2c_master_bus_handle_t bus);

/**
 * Write a hybrid flow configuration byte stream to the DAC.
 * Format: [reg, len, data[0..len-1], ...] terminated by 0xFF, 0xFF.
 * The HF config manages its own standby entry/exit.
 */
static esp_err_t tas57xx_write_hf(i2c_master_dev_handle_t handle,
                                  const uint8_t *stream) {
  esp_err_t err;
  int pos = 0;
  while (!(stream[pos] == 0xFF && stream[pos + 1] == 0xFF)) {
    uint8_t reg = stream[pos];
    uint8_t len = stream[pos + 1];
    const uint8_t *data = &stream[pos + 2];
    err = board_i2c_write(handle, reg, data, len);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "HF write failed at offset %d (reg 0x%02X): %s", pos, reg,
               esp_err_to_name(err));
      return err;
    }
    pos += 2 + len;
  }
  ESP_LOGI(TAG, "HybridFlow loaded");
  return ESP_OK;
}

// Load a hybrid-flow for device index i and program it.
//   single device   -> /spiffs/hf/tas57xx_fw.bin   (legacy name)
//   multiple devices -> /spiffs/hf/tas57xx_fw<i>.bin (mains=0, sub=1, ...)
// A mains device (index 0) with no indexed file falls back to the legacy name.
// A sub with no hybrid-flow runs in normal BTL stereo (no crossover).
static void tas57xx_load_hf(int i, bool multi) {
  tas57xx_dev_t *d = &s_devs[i];

  // TAS578x has no miniDSP / hybrid-flow.
  if (d->addr == TAS578x) {
    return;
  }

  char path[48];
  if (multi) {
    snprintf(path, sizeof(path), "/spiffs/hf/tas57xx_fw%d.bin", i);
  } else {
    snprintf(path, sizeof(path), "/spiffs/hf/tas57xx_fw.bin");
  }

  FILE *f = fopen(path, "rb");
  if (!f && multi && i == 0) {
    // Mains falls back to the legacy unindexed name.
    snprintf(path, sizeof(path), "/spiffs/hf/tas57xx_fw.bin");
    f = fopen(path, "rb");
  }

  if (f) {
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *buf = malloc(size);
    if (buf && fread(buf, 1, size, f) == (size_t)size) {
      d->hf_buf = buf;
      d->hf_size = size;
      tas57xx_write_hf(d->handle, buf);
      ESP_LOGI(TAG, "Loaded HF %s for @0x%02X", path, d->addr);
    } else {
      ESP_LOGE(TAG, "Failed to read HF file %s", path);
      free(buf);
    }
    fclose(f);
    if (d->hf_buf) {
      return;
    }
  }

  // No hybrid-flow present. The device runs in its normal (safe) BTL stereo
  // configuration. A sub with no HF has no low-pass/mono routing, so it plays
  // full-range — provide a mono low-pass flow as tas57xx_fw<i>.bin.
  if (d->is_sub) {
    ESP_LOGW(TAG,
             "No HF for sub @0x%02X — running full-range BTL (no crossover). "
             "Provide a mono low-pass flow as /spiffs/hf/tas57xx_fw%d.bin.",
             d->addr, i);
  } else {
    ESP_LOGI(TAG, "No HF at %s, running default stereo", path);
  }
}

static esp_err_t tas57xx_init(void *i2c_bus) {
  esp_err_t err = ESP_OK;

  if (s_dac_mutex == NULL) {
    s_dac_mutex = xSemaphoreCreateMutex();
    if (s_dac_mutex == NULL) {
      ESP_LOGE(TAG, "Failed to create DAC mutex");
      return ESP_ERR_NO_MEM;
    }
  }

  s_bus_handle = (i2c_master_bus_handle_t)i2c_bus;
  if (s_bus_handle == NULL) {
    ESP_LOGE(TAG, "No I2C bus handle provided");
    return ESP_ERR_INVALID_ARG;
  }

  // Detect all TAS57xx chips on the bus (0x4C = mains at index 0).
  s_dev_count = tas57xx_detect_all(s_bus_handle);
  if (s_dev_count == 0) {
    ESP_LOGW(TAG, "No TAS57xx detected");
    return ESP_ERR_NOT_FOUND;
  }
  ESP_LOGI(TAG, "TAS57xx devices detected: %d", s_dev_count);

  for (int i = 0; i < s_dev_count; i++) {
    err = board_i2c_add_device(s_bus_handle, s_devs[i].addr, I2C_LINE_SPEED,
                               &s_devs[i].handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Could not add device @0x%02X to bus: %s", s_devs[i].addr,
               esp_err_to_name(err));
      return err;
    }
  }

  // Read chip identity for the primary device.
  if (s_devs[0].addr == TAS578x) {
    uint8_t page = 0x00;
    board_i2c_write(s_devs[0].handle, 0x00, &page, 1);
    uint8_t device_id = 0;
    if (board_i2c_read(s_devs[0].handle, TAS578x_REG_DEVICE_ID, &device_id,
                       1) == ESP_OK) {
      ESP_LOGI(TAG, "TAS578x device ID: 0x%02X", device_id);
    }
  } else {
    ESP_LOGI(TAG, "TAS575x detected (no device ID register)");
  }

  // Load hybrid-flows (or PBTL fallback for a sub) per device.
  bool multi = s_dev_count > 1;
  for (int i = 0; i < s_dev_count; i++) {
    tas57xx_load_hf(i, multi);
  }

  // Apply additional init registers to every device.
  for (int i = 0; i < s_dev_count; i++) {
    for (int k = 0; tas57xx_init_seq[k].reg != 0xff; k++) {
      err = board_i2c_write(s_devs[i].handle, tas57xx_init_seq[k].reg,
                            &tas57xx_init_seq[k].value, sizeof(uint8_t));
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write init reg 0x%02x @0x%02X: %s",
                 tas57xx_init_seq[k].reg, s_devs[i].addr, esp_err_to_name(err));
        return err;
      }
    }
  }

  return err;
}

static esp_err_t tas57xx_deinit(void) {
  esp_err_t err = ESP_OK;

  for (int i = 0; i < s_dev_count; i++) {
    if (s_devs[i].handle) {
      esp_err_t e = board_i2c_remove_device(s_devs[i].handle);
      if (e != ESP_OK) {
        ESP_LOGE(TAG, "failed to remove @0x%02X from i2c bus, err: %s",
                 s_devs[i].addr, esp_err_to_name(e));
        err = e;
      }
      s_devs[i].handle = NULL;
    }
    free(s_devs[i].hf_buf);
    s_devs[i].hf_buf = NULL;
    s_devs[i].hf_size = 0;
  }
  s_dev_count = 0;
  s_bus_handle = NULL;

  if (s_dac_mutex != NULL) {
    vSemaphoreDelete(s_dac_mutex);
    s_dac_mutex = NULL;
  }
  return err;
}

/**
 * Re-apply HF config (or PBTL fallback) and init registers after a full
 * shutdown. Shutdown (reg 0x02=0x01) loses miniDSP RAM contents.
 */
static void tas57xx_restore_config(void) {
  for (int i = 0; i < s_dev_count; i++) {
    tas57xx_dev_t *d = &s_devs[i];
    if (d->hf_buf) {
      esp_err_t err = tas57xx_write_hf(d->handle, d->hf_buf);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restore HF @0x%02X: %s", d->addr,
                 esp_err_to_name(err));
      }
    }
    for (int k = 0; tas57xx_init_seq[k].reg != 0xff; k++) {
      board_i2c_write(d->handle, tas57xx_init_seq[k].reg,
                      &tas57xx_init_seq[k].value, sizeof(uint8_t));
    }
  }
}

static void tas57xx_enable_speaker(bool enable) {
  for (int i = 0; i < s_dev_count; i++) {
    write_cmd(s_devs[i].handle,
              enable ? TAS57XX_ANALOGUE_ON : TAS57XX_ANALOGUE_OFF);
  }
}

static void tas57xx_set_power_mode(dac_power_mode_t mode) {
  xSemaphoreTake(s_dac_mutex, portMAX_DELAY);
  tas57xx_enable_speaker(false);
  switch (mode) {
  case DAC_POWER_STANDBY:
    for (int i = 0; i < s_dev_count; i++) {
      write_cmd(s_devs[i].handle, TAS57XX_MUTE);
      write_cmd(s_devs[i].handle, TAS57XX_STANDBY);
    }
    if (s_power_state == DAC_POWER_OFF) {
      // Wait for standby state to settle before writing miniDSP config
      vTaskDelay(pdMS_TO_TICKS(50));
      tas57xx_restore_config();
    }
    break;
  case DAC_POWER_ON:
    for (int i = 0; i < s_dev_count; i++) {
      write_cmd(s_devs[i].handle, TAS57XX_MUTE);
      write_cmd(s_devs[i].handle, TAS57XX_ACTIVE);
    }
    // Allow PLL lock and charge pump settling before unmuting
    vTaskDelay(pdMS_TO_TICKS(50));
    for (int i = 0; i < s_dev_count; i++) {
      write_cmd(s_devs[i].handle, TAS57XX_UNMUTE);
    }
    tas57xx_enable_speaker(true);
    break;
  case DAC_POWER_OFF:
    for (int i = 0; i < s_dev_count; i++) {
      write_cmd(s_devs[i].handle, TAS57XX_MUTE);
      write_cmd(s_devs[i].handle, TAS57XX_DOWN);
    }
    break;
  default:
    ESP_LOGW(TAG, "Unhandled power mode");
    break;
  }
  s_power_state = mode;
  xSemaphoreGive(s_dac_mutex);
}

static void tas57xx_enable_line_out(bool enable) {
  (void)enable;
  ESP_LOGW(TAG, "Not supported yet");
}

// Map an AirPlay volume (-30..0 dB) to a DAC dB level using the 2:1 curve.
static float tas57xx_map_volume_db(float volume_airplay_db) {
  if (volume_airplay_db > 0.0f) {
    volume_airplay_db = 0.0f;
  }
  if (volume_airplay_db < -30.0f) {
    volume_airplay_db = -30.0f;
  }

  // Volume mapping (2:1 scaling):
  // AirPlay 0 dB    -> DAC CONFIG_TAS57XX_MAX_VOLUME
  // AirPlay -25 dB  -> DAC (MAX - 50)
  // AirPlay -30..-25 dB -> DAC mute(-127)..(MAX-50) (steep roll-off)
  float max_db = (float)CONFIG_TAS57XX_MAX_VOLUME;
  float db_level;
  if (volume_airplay_db >= -25.0f) {
    // 2:1 linear scaling: 25 dB AirPlay range -> 50 dB DAC range
    db_level = max_db + (volume_airplay_db * 2.0f);
  } else {
    // Roll-off: map -30..-25 to -127..(MAX-50)
    float normalized = (volume_airplay_db + 30.0f) / 5.0f;
    float rolloff_top = max_db - 50.0f;
    db_level = -127.0f + normalized * (127.0f + rolloff_top);
  }
  return db_level;
}

// Convert a DAC dB level to a register value (0x00=0dB, 0xFE=-127dB).
static uint8_t tas57xx_db_to_reg(float db_level) {
  if (db_level > 0.0f) {
    db_level = 0.0f;
  }
  if (db_level < -127.0f) {
    db_level = -127.0f;
  }
  return (uint8_t)(-db_level * 2.0f);
}

// Re-apply the cached master volume to every device, adding the sub offset to
// any sub device. Caller must hold s_dac_mutex.
static void tas57xx_apply_volume_locked(void) {
  float base_db = tas57xx_map_volume_db(s_last_airplay_db);
  uint8_t main_reg = tas57xx_db_to_reg(base_db);
  uint8_t sub_reg = tas57xx_db_to_reg(base_db + s_sub_offset_db);

  ESP_LOGD(TAG,
           "Volume: AirPlay %.1f dB -> DAC %.1f dB (main 0x%02X, sub %+.1f dB "
           "0x%02X)",
           s_last_airplay_db, base_db, main_reg, s_sub_offset_db, sub_reg);

  for (int i = 0; i < s_dev_count; i++) {
    uint8_t reg_val = s_devs[i].is_sub ? sub_reg : main_reg;
    write_cmd(s_devs[i].handle, TAS57XX_SET_VOLUME_A_L, reg_val);
    write_cmd(s_devs[i].handle, TAS57XX_SET_VOLUME_B_R, reg_val);
  }
}

static void tas57xx_set_volume(float volume_airplay_db) {
  xSemaphoreTake(s_dac_mutex, portMAX_DELAY);
  if (volume_airplay_db > 0.0f) {
    volume_airplay_db = 0.0f;
  }
  if (volume_airplay_db < -30.0f) {
    volume_airplay_db = -30.0f;
  }
  s_last_airplay_db = volume_airplay_db;
  tas57xx_apply_volume_locked();
  xSemaphoreGive(s_dac_mutex);
}

void dac_tas57xx_set_sub_offset_db(float offset_db) {
  if (offset_db > TAS57XX_SUB_OFFSET_MAX_DB) {
    offset_db = TAS57XX_SUB_OFFSET_MAX_DB;
  }
  if (offset_db < TAS57XX_SUB_OFFSET_MIN_DB) {
    offset_db = TAS57XX_SUB_OFFSET_MIN_DB;
  }

  // May be called (e.g. from the web server) before the DAC is initialised;
  // store the value and let the next volume update apply it.
  if (s_dac_mutex == NULL) {
    s_sub_offset_db = offset_db;
    return;
  }

  xSemaphoreTake(s_dac_mutex, portMAX_DELAY);
  s_sub_offset_db = offset_db;
  tas57xx_apply_volume_locked();
  xSemaphoreGive(s_dac_mutex);
  ESP_LOGI(TAG, "Sub volume offset: %+.1f dB", offset_db);
}

float dac_tas57xx_get_sub_offset_db(void) {
  return s_sub_offset_db;
}

const dac_ops_t dac_tas57xx_ops = {
    .init = tas57xx_init,
    .deinit = tas57xx_deinit,
    .set_volume = tas57xx_set_volume,
    .set_power_mode = tas57xx_set_power_mode,
    .enable_speaker = tas57xx_enable_speaker,
    .enable_line_out = tas57xx_enable_line_out,
};

static esp_err_t write_cmd(i2c_master_dev_handle_t handle, tas57xx_cmd_e cmd,
                           ...) {
  va_list args;
  esp_err_t err = ESP_OK;
  va_start(args, cmd);

  switch (cmd) {
  case TAS57XX_SET_VOLUME_A_L:
  case TAS57XX_SET_VOLUME_B_R:
    uint8_t val = (uint8_t)va_arg(args, int);
    err = board_i2c_write(handle, tas57xx_cmd[cmd].reg, &val, sizeof(uint8_t));
    break;
  default:
    err = board_i2c_write(handle, tas57xx_cmd[cmd].reg,
                          &(tas57xx_cmd[cmd].value), sizeof(uint8_t));
  }

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed i2c write to TAS57xx: %s", esp_err_to_name(err));
  }

  va_end(args);
  return err;
}

/**
 * Detect all TAS57xx chips on the bus and populate s_devs[].
 * A single TAS578x is supported for legacy boards; otherwise every responding
 * TAS575x address (0x4C..0x4F) is added, with 0x4C as the mains at index 0.
 * Returns the number of devices found.
 */
static int tas57xx_detect_all(i2c_master_bus_handle_t bus) {
  if (!bus) {
    ESP_LOGE(TAG, "Invalid i2c handle!");
    return 0;
  }

  memset(s_devs, 0, sizeof(s_devs));

  // Legacy single TAS578x.
  if (i2c_master_probe(bus, TAS578x, I2C_TIMEOUT) == ESP_OK) {
    s_devs[0].addr = TAS578x;
    ESP_LOGI(TAG, "Detected TAS578x @0x%02X", TAS578x);
    return 1;
  }

  int count = 0;
  for (size_t i = 0; i < sizeof(tas575x_addrs) && count < TAS57XX_MAX_DEVICES;
       i++) {
    if (i2c_master_probe(bus, tas575x_addrs[i], I2C_TIMEOUT) == ESP_OK) {
      s_devs[count].addr = tas575x_addrs[i];
      s_devs[count].is_sub = (count > 0);
      ESP_LOGI(TAG, "Detected TAS575x @0x%02X (%s)", tas575x_addrs[i],
               count == 0 ? "mains" : "sub");
      count++;
    }
  }
  return count;
}
