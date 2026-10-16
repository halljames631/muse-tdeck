/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * LILYGO T-Deck: ESP32-S3FN16R8 (16 MB flash, 8 MB octal PSRAM),
 * 2.8" 320x240 ST7789 on SPI, GT911 touch, ES7210 dual-mic ADC,
 * MAX98357A I2S speaker amp and ES7210 mic (both uninitialized:
 * display-only for now), BOOT on GPIO0, native USB Serial/JTAG.
 *
 * Pin source: Xinyuan-LilyGO/T-Deck (MIT): its utilities header and
 * the Microphone, I2SPlay and Touchpad examples, plus schematic.pdf.
 * ST7789 orientation: the vendor runs TFT_eSPI setRotation(1) (landscape),
 * which programs MADCTL = MX|MV; esp_lcd's swap_xy(true)+mirror(true,false)
 * is the same setting (lib/TFT_eSPI/TFT_Drivers/ST7789_Rotation.h).
 * Touch orientation: vendor Touchpad example, setSwapXY(true) and
 * setMirrorXY(false, true) at 320x240.
 *
 * Display/touch structure follows the in-repo AIPI (ST7789 +
 * esp_lv_adapter, hand-driven) and Waveshare LCD-7 (GT911) boards.
 * Audio (MAX98357A/ES7210) is intentionally uninitialized for now:
 * display-only bring-up. Touch verification still required.
 */
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/usb_serial_jtag.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "muse_board.h"
#include "muse_mem.h"
#include "muse_settings.h"
#include "muse_chat.h"
#include "esp_system.h"  /* esp_restart */

static const char *TAG = "tdeck";

/* Peripheral power: HIGH or the display, touch and audio stay dark. */
#define POWERON_GPIO GPIO_NUM_10

#define LCD_H_RES 320
#define LCD_V_RES 240
#define LCD_HOST SPI3_HOST
#define LCD_SCLK GPIO_NUM_40
#define LCD_MOSI GPIO_NUM_41
#define LCD_CS   GPIO_NUM_12
#define LCD_DC   GPIO_NUM_11
#define LCD_BL   GPIO_NUM_42
#define DRAW_BUF_LINES 32  /* 320*32*2 = 20 KB per DMA buffer */

#define I2C_SDA GPIO_NUM_18
#define I2C_SCL GPIO_NUM_8
#define TOUCH_INT GPIO_NUM_16

/* MAX98357A speaker amp: I2S0 TX (not initialized yet). */
#define SPK_BCK  GPIO_NUM_7
#define SPK_WS   GPIO_NUM_5
#define SPK_DOUT GPIO_NUM_6

/* ES7210 dual-mic ADC: I2S1 RX at 0x40 (not initialized yet). */
#define MIC_MCLK GPIO_NUM_48
#define MIC_WS   GPIO_NUM_21
#define MIC_BCK  GPIO_NUM_47
#define MIC_DIN  GPIO_NUM_14
#define ES7210_ADDR 0x40

/* SPI bus-mates of the display: SX1262 LoRa and the SD slot. Parked. */
#define RADIO_CS GPIO_NUM_9
#define SD_CS    GPIO_NUM_39

#define TALK_GPIO GPIO_NUM_0  /* BOOT */
#define BATT_ADC_CH ADC_CHANNEL_3  /* GPIO4, through the 100K/100K divider */

static i2c_master_bus_handle_t s_i2c;
static esp_lcd_panel_handle_t s_panel;
static esp_lcd_touch_handle_t s_touch;
static muse_gpio_button_t s_talk;
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;  /* NULL when eFuse calibration is missing */

#define KB_ADDR 0x55
#define KB_INT  GPIO_NUM_46

static i2c_master_dev_handle_t s_kb;

static void keyboard_init(void)
{
    const i2c_device_config_t kb_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = KB_ADDR,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(s_i2c, &kb_cfg, &s_kb) != ESP_OK) {
        s_kb = NULL;
    }
}

static char keyboard_poll(void)
{
    uint8_t b = 0;
    if (!s_kb || i2c_master_receive(s_kb, &b, 1, 20) != ESP_OK || b == 0) {
        return 0;
    }
    return (char)b;
}

static void keyboard_set_brightness(uint8_t level)
{
    if (!s_kb) {
        return;
    }
    const uint8_t cmd[2] = { 0x01, level };
    i2c_master_transmit(s_kb, cmd, sizeof(cmd), 50);
}


static esp_err_t init(void)
{
    /* Peripherals first: the vendor header says this pin must be HIGH
     * when using the board's peripherals. */
    ESP_RETURN_ON_ERROR(gpio_set_direction(POWERON_GPIO, GPIO_MODE_OUTPUT),
                        TAG, "poweron dir");
    ESP_RETURN_ON_ERROR(gpio_set_level(POWERON_GPIO, 1), TAG, "poweron");

    /* The LoRa radio and the SD slot share the display's SPI bus
     * (MOSI 41 / MISO 38 / SCK 40). Muse uses neither: keep their
     * chip-selects high so they never answer the display's traffic. */
    const gpio_config_t cs = {
        .pin_bit_mask = (1ULL << LCD_CS) | (1ULL << RADIO_CS) |
                        (1ULL << SD_CS),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cs), TAG, "spi cs");
    ESP_RETURN_ON_ERROR(gpio_set_level(LCD_CS, 1), TAG, "tft cs");
    ESP_RETURN_ON_ERROR(gpio_set_level(RADIO_CS, 1), TAG, "radio cs");
    ESP_RETURN_ON_ERROR(gpio_set_level(SD_CS, 1), TAG, "sd cs");

    const i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_cfg, &s_i2c), TAG, "i2c");
    keyboard_init();
    ESP_RETURN_ON_ERROR(muse_gpio_button_init(&s_talk, TALK_GPIO),
                        TAG, "talk button");
    /* BOOT is held at flashing time; don't count that as a press. */
    s_talk.pressed = gpio_get_level(TALK_GPIO) == 0;

    const adc_oneshot_unit_init_cfg_t adc_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&adc_cfg, &s_adc), TAG, "adc");
    const adc_oneshot_chan_cfg_t ch_cfg = {
        .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, BATT_ADC_CH, &ch_cfg),
                        TAG, "adc channel");
    const adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1, .chan = BATT_ADC_CH, .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali) != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration unavailable; battery_mv will read 0");
        s_cali = NULL;
    }
    return ESP_OK;
}

static void ondevice_setup(void);
static void chat_task(void *arg);

static lv_display_t *display_start(lv_indev_t **touch)
{
    const ledc_timer_config_t bl_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    const ledc_channel_config_t bl_ch = {
        .gpio_num = LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    if (ledc_timer_config(&bl_timer) != ESP_OK ||
        ledc_channel_config(&bl_ch) != ESP_OK) {
        return NULL;
    }

    const spi_bus_config_t bus = {
        .sclk_io_num = LCD_SCLK,
        .mosi_io_num = LCD_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = LCD_H_RES * DRAW_BUF_LINES * 2,
    };
    if (spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) {
        return NULL;
    }
    esp_lcd_panel_io_handle_t io;
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = LCD_CS,
        .dc_gpio_num = LCD_DC,
        .spi_mode = 0,
        .pclk_hz = 40 * 1000 * 1000,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    if (esp_lcd_new_panel_io_spi(LCD_HOST, &io_cfg, &io) != ESP_OK) {
        return NULL;
    }
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = GPIO_NUM_NC,  /* not routed; software reset */
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    if (esp_lcd_new_panel_st7789(io, &panel_cfg, &s_panel) != ESP_OK) {
        return NULL;
    }
    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    /* Landscape, matching the vendor's TFT_eSPI setRotation(1) = MX|MV. */
    esp_lcd_panel_swap_xy(s_panel, true);
    esp_lcd_panel_mirror(s_panel, true, false);
    esp_lcd_panel_disp_on_off(s_panel, true);

    /* GT911: vendor Touchpad example probes 0x5D (INT selects 0x14). */
    uint8_t addr = 0x5D;
    if (i2c_master_probe(s_i2c, addr, 50) != ESP_OK) {
        addr = 0x14;
        if (i2c_master_probe(s_i2c, addr, 50) != ESP_OK) {
            ESP_LOGE(TAG, "GT911 not found");
            return NULL;
        }
    }
    ESP_LOGI(TAG, "GT911 at 0x%02x", addr);
    esp_lcd_panel_io_handle_t tp_io;
    esp_lcd_panel_io_i2c_config_t tp_io_cfg =
        ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    tp_io_cfg.dev_addr = addr;
    tp_io_cfg.scl_speed_hz = 400000;
    if (esp_lcd_new_panel_io_i2c(s_i2c, &tp_io_cfg, &tp_io) != ESP_OK) {
        return NULL;
    }
    esp_lcd_touch_io_gt911_config_t gt_cfg = { .dev_addr = addr };
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = TOUCH_INT,
        .levels = { .reset = 0, .interrupt = 0 },
        /* Vendor Touchpad example: swapXY(true), mirrorXY(false, true). */
        .flags = { .swap_xy = 1, .mirror_x = 0, .mirror_y = 1 },
        .driver_data = &gt_cfg,
    };
    if (esp_lcd_touch_new_i2c_gt911(tp_io, &tp_cfg, &s_touch) != ESP_OK) {
        return NULL;
    }

    esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    adapter_cfg.task_core_id = MUSE_UI_CORE;
    adapter_cfg.task_priority = MUSE_UI_PRIORITY;
    if (esp_lv_adapter_init(&adapter_cfg) != ESP_OK) {
        return NULL;
    }
    const esp_lv_adapter_display_config_t disp_cfg = {
        .panel = s_panel,
        .panel_io = io,
        .profile = {
            .interface = ESP_LV_ADAPTER_PANEL_IF_OTHER,
            .rotation = ESP_LV_ADAPTER_ROTATE_0,
            .hor_res = LCD_H_RES,
            .ver_res = LCD_V_RES,
            .buffer_height = DRAW_BUF_LINES,
            .use_psram = false,
            .require_double_buffer = true,
        },
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE,
    };
    lv_display_t *disp = esp_lv_adapter_register_display(&disp_cfg);
    if (!disp) {
        return NULL;
    }
    const esp_lv_adapter_touch_config_t tp_lv_cfg =
        ESP_LV_ADAPTER_TOUCH_DEFAULT_CONFIG(disp, s_touch);
    *touch = esp_lv_adapter_register_touch(&tp_lv_cfg);
    if (!*touch || esp_lv_adapter_start() != ESP_OK) {
        return NULL;
    }
    ondevice_setup();
    /* Chat task disabled for reboot isolation test. */
    // xTaskCreate(chat_task, "tdeck_chat", 8192, NULL, 5, NULL);
    return disp;
}

static bool display_lock(int timeout_ms)
{
    return esp_lv_adapter_lock(timeout_ms) == ESP_OK;
}

static void set_brightness(int pct)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pct * 1023 / 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void panel_sleep(bool sleep)
{
    esp_lcd_panel_disp_on_off(s_panel, !sleep);
}

static void display_pause(bool pause)
{
    if (pause) {
        esp_lv_adapter_pause(-1);
    } else {
        esp_lv_adapter_resume();
    }
}

/* Audio (MAX98357A speaker, ES7210 mic) is not initialized
 * yet: this board runs display-only until the codec plumbing lands.
 */

static unsigned poll_buttons(void)
{
    return muse_gpio_button_poll(&s_talk);
}

static void wait_buttons(int timeout_ms)
{
    muse_gpio_buttons_wait((muse_gpio_button_t *const[]){ &s_talk }, 1,
                           timeout_ms);
}

/* Battery on GPIO4 through the 100K/100K divider (VBAT/2), per the
 * schematic. Curve is a plain Li-ion map; verify against a meter. */
static esp_err_t read_power(muse_power_t *out)
{
    out->battery_mv = 0;
    out->battery_pct = -1;
    out->charging = false;  /* no charge-status GPIO verified */
    out->usb = usb_serial_jtag_is_connected();
    if (!s_cali) {
        return ESP_OK;
    }
    int sum = 0;
    for (int i = 0; i < 8; i++) {
        int raw;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc, BATT_ADC_CH, &raw), TAG,
                            "adc read");
        sum += raw;
    }
    int mv = 0;
    ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(s_cali, sum / 8, &mv), TAG,
                        "adc cali");
    out->battery_mv = mv * 2;
    int pct = (out->battery_mv - 3300) * 100 / (4200 - 3300);
    out->battery_pct = pct < 0 ? 0 : pct > 100 ? 100 : pct;
    return ESP_OK;
}

static esp_err_t power_off(void)
{
    /* No software power latch on this board: kill the peripherals and
     * deep-sleep until BOOT or RESET, like the BOX-3 board does. */
    set_brightness(0);
    esp_lcd_panel_disp_on_off(s_panel, false);
    gpio_set_level(POWERON_GPIO, 0);
    while (gpio_get_level(TALK_GPIO) == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    ESP_RETURN_ON_ERROR(esp_sleep_enable_ext0_wakeup(TALK_GPIO, 0), TAG,
                        "wake");
    esp_deep_sleep_start();
    return ESP_FAIL;
}

static const muse_board_t s_board = {
    .name = "LILYGO T-Deck",
    .width = LCD_H_RES,
    .height = LCD_V_RES,
    .round = false,
    .touch = true,
    .touch_talk = true,  /* on-screen mic; BOOT is the physical talk key */
    .diagonal_in = 2.8f,
    .talk_button = "boot",
    .talk_hint = { LV_ALIGN_TOP_LEFT, 8, 8 },
    .frame_ms = 40,
    .init = init,
    .display_start = display_start,
    .display_lock = display_lock,
    .display_unlock = esp_lv_adapter_unlock,
    .set_brightness = set_brightness,
    .panel_sleep = panel_sleep,
    .display_pause = display_pause,
    .audio_init = NULL,  /* display-only: no speaker/mic yet */
    .poll_buttons = poll_buttons,
    .wait_buttons = wait_buttons,
    .read_power = read_power,
    .power_off = power_off,
};


/* ---- On-device setup via the T-Deck keyboard ----
 *
 * The keyboard is a small MCU on I2C (address 0x55, INT on GPIO46) that
 * reports ASCII keystrokes: a one-byte read returns 0 when idle. This
 * screen lets the board provision itself -- WiFi SSID, password and the
 * Hatch token -- typed on the physical keyboard, with no laptop or phone.
 * It appears when no WiFi is saved yet, or when BOOT is held at boot.
 * Enter advances fields (saves on the last one), Backspace deletes,
 * ESC skips. The SAVE button is also tappable. */

static void setup_save_cb(lv_event_t *e)
{
    *(bool *)lv_event_get_user_data(e) = true;
}

/* Push the SDK's HOMEHUB_WIFI_SSID override into the Link layer after boot.
 * The override in app_run() connects via wifi_mgr directly, but the Muse UI
 * and tunnel read status from Link (muse_glue.c), which needs s_ssid set. */
static void link_wifi_push_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(15000));  /* SDK fully up */
#ifdef CONFIG_HOMEHUB_WIFI_SSID
    if (CONFIG_HOMEHUB_WIFI_SSID[0] != '\0') {
        muse_settings_set_wifi(CONFIG_HOMEHUB_WIFI_SSID,
                               CONFIG_HOMEHUB_WIFI_PASSWORD);
        ESP_LOGI(TAG, "pushed HOMEHUB WiFi '%s' to Link", CONFIG_HOMEHUB_WIFI_SSID);
    }
#endif
    vTaskDelete(NULL);
}

static void ondevice_setup(void)
{
    /* SDK dev override: CONFIG_HOMEHUB_WIFI_SSID connects automatically at
     * boot (see main/app.c). If set, skip our setup screen entirely. */
#ifdef CONFIG_HOMEHUB_WIFI_SSID
    if (CONFIG_HOMEHUB_WIFI_SSID[0] != '\0') {
        ESP_LOGI(TAG, "HOMEHUB_WIFI_SSID override active ('%s'); skipping setup",
                 CONFIG_HOMEHUB_WIFI_SSID);
        xTaskCreate(link_wifi_push_task, "tdeck_linkwifi", 4096, NULL, 5, NULL);
        return;
    }
#endif
#ifdef CONFIG_TDECK_WIFI_SSID
    if (CONFIG_TDECK_WIFI_SSID[0] != '\0') {
        muse_settings_set_wifi(CONFIG_TDECK_WIFI_SSID, CONFIG_TDECK_WIFI_PASS);
        ESP_LOGI(TAG, "using TDECK hard-coded WiFi '%s'", CONFIG_TDECK_WIFI_SSID);
        return;
    }
#endif
    char ssid[MUSE_SSID_MAX + 1] = {0};
    char pass[MUSE_PASS_MAX + 1] = {0};
    muse_settings_wifi(ssid, pass);
    if (ssid[0] != '\0' && gpio_get_level(TALK_GPIO) != 0) {
        ESP_LOGI(TAG, "saved WiFi '%s' found; skipping setup", ssid);
        return;
    }

    ESP_LOGI(TAG, "on-device setup: type credentials on the keyboard");
    set_brightness(100);  /* SDK only lights the backlight after display_start */
    keyboard_set_brightness(120);

    lv_obj_t *scr = lv_screen_active();  /* SDK builds on this too; clean up after */
    if (!scr) {
        ESP_LOGW(TAG, "no screen yet; skipping on-device setup");
        return;
    }

    display_lock(-1);  /* LVGL is not thread-safe; the render task is running */
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "DEVICE SETUP");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 6);

    /* If the token is baked into the firmware, don't ask for it. */
    const bool token_done =
#ifdef CONFIG_GADGET_SDK_TOKEN
        CONFIG_GADGET_SDK_TOKEN[0] != '\0';
#else
        false;
#endif
    const int nfields = token_done ? 2 : 3;
    static const char *const labels[3] = {
        "WiFi name", "WiFi password", "Token (mgst_...)" };
    static const uint32_t maxlen[3] = {
        MUSE_SSID_MAX, MUSE_PASS_MAX, MUSE_TOKEN_MAX };
    lv_obj_t *tas[3];
    for (int i = 0; i < nfields; i++) {
        lv_obj_t *lb = lv_label_create(scr);
        lv_label_set_text(lb, labels[i]);
        lv_obj_set_pos(lb, 12, 30 + i * 50);
        tas[i] = lv_textarea_create(scr);
        lv_textarea_set_one_line(tas[i], true);
        lv_textarea_set_max_length(tas[i], maxlen[i]);
        if (i == 1) {
            lv_textarea_set_password_mode(tas[i], true);
        }
        if (i == 0 && ssid[0] != '\0') {
            lv_textarea_set_text(tas[i], ssid);
        }
        lv_obj_set_size(tas[i], 296, 30);
        lv_obj_set_pos(tas[i], 12, 46 + i * 50);
    }

    lv_obj_t *hint = lv_label_create(scr);
    lv_label_set_text(hint, "Enter: next   Bksp: del   ESC: skip");
    lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 182);

    bool save = false;
    lv_obj_t *btn = lv_button_create(scr);
    lv_obj_set_size(btn, 200, 34);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 198);
    lv_obj_t *bl = lv_label_create(btn);
    lv_label_set_text(bl, "SAVE & REBOOT");
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn, setup_save_cb, LV_EVENT_CLICKED, &save);

    int focus = 0;
    bool done = false;
    for (int i = 0; i < nfields; i++) {
        lv_obj_set_style_border_width(tas[i], i == focus ? 2 : 0,
                                      LV_PART_MAIN);
    }
    esp_lv_adapter_unlock();
    while (!done) {
        char k = keyboard_poll();
        if (k) {
            display_lock(-1);
        if (k >= 0x20 && k <= 0x7E) {
            lv_textarea_add_char(tas[focus], (uint32_t)(unsigned char)k);
        } else if (k == 0x08 || k == 0x7F) {
            lv_textarea_delete_char(tas[focus]);
        } else if (k == '\r' || k == '\n') {
            if (++focus >= nfields) {
                save = true;
            } else {
                for (int i = 0; i < nfields; i++) {
                    lv_obj_set_style_border_width(tas[i], i == focus ? 2 : 0,
                                                  LV_PART_MAIN);
                }
            }
        } else if (k == 0x1B) {
            done = true;  /* ESC: skip */
        }
            esp_lv_adapter_unlock();
        }
        if (save) {
            done = true;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    display_lock(-1);

    bool changed = false;
    if (save) {
        const char *s = lv_textarea_get_text(tas[0]);
        const char *pw = lv_textarea_get_text(tas[1]);
        const char *t = nfields > 2 ? lv_textarea_get_text(tas[2]) : "";
        if (s[0] != '\0') {
            muse_settings_set_wifi(s, pw);
            ESP_LOGI(TAG, "wifi saved for '%s'", s);
            changed = true;
        }
        if (!token_done && t[0] != '\0' &&
            muse_settings_set_hatch_token(t, false) == ESP_OK) {
            ESP_LOGI(TAG, "token saved");
            changed = true;
        }
    }
    lv_obj_clean(scr);  /* remove our widgets; leave the screen for the SDK */
    esp_lv_adapter_unlock();
    if (changed) {
        vTaskDelay(pdMS_TO_TICKS(300));
        esp_restart();
    }
    /* Skipped or nothing entered: the SDK shows its own setup screen. */
}


/* ---- On-device chat ----
 *
 * A standalone text terminal on the T-Deck: type on the physical keyboard,
 * Enter sends via muse_hatch_text_turn(), replies stream back through
 * muse_hatch_turn_event() and appear on screen. Starts after the SDK's UI
 * is up, then takes over the display. */

/* Caller must hold the display lock. */
static void chat_add_message(lv_obj_t *hist, const char *who, const char *text)
{
    lv_obj_t *lb = lv_label_create(hist);
    lv_label_set_text_fmt(lb, "%s: %s", who, text);
    lv_label_set_long_mode(lb, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lb, 296);
    lv_obj_scroll_to_view(lb, LV_ANIM_OFF);
}

static void chat_task(void *arg)
{
    /* Let the SDK finish connecting. Show a hint; take the screen only when
     * the user presses Enter, so we never fight the SDK mid-connection. */
    vTaskDelay(pdMS_TO_TICKS(8000));
    display_lock(-1);
    lv_obj_t *hint = lv_label_create(lv_screen_active());
    lv_label_set_text(hint, "ENTER for chat");
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -4);
    esp_lv_adapter_unlock();

    /* Wait for Enter. */
    while (1) {
        char k = keyboard_poll();
        if (k == '\r' || k == '\n') {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    display_lock(-1);
    lv_obj_delete(hint);
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_screen_load(scr);
    lv_obj_t *hist = lv_obj_create(scr);
    lv_obj_set_size(hist, 320, 188);
    lv_obj_set_pos(hist, 0, 0);
    lv_obj_set_flex_flow(hist, LV_FLEX_FLOW_COLUMN);
    lv_obj_t *input = lv_textarea_create(scr);
    lv_textarea_set_one_line(input, true);
    lv_textarea_set_max_length(input, 512);
    lv_obj_set_size(input, 310, 36);
    lv_obj_set_pos(input, 5, 196);
    esp_lv_adapter_unlock();

    display_lock(-1);
    chat_add_message(hist, "Sys", "Type a message below, Enter sends.");
    esp_lv_adapter_unlock();

    char ibuf[513];
    size_t ilen = 0;
    char reply[8192];
    size_t rlen = 0;
    bool waiting = false;
    lv_obj_t *reply_lb = NULL;

    while (1) {
        char k = keyboard_poll();
        if (k && !waiting) {
            display_lock(-1);
            if (k >= 0x20 && k <= 0x7E && ilen < sizeof(ibuf) - 1) {
                ibuf[ilen++] = k;
                ibuf[ilen] = '\0';
                lv_textarea_set_text(input, ibuf);
            } else if ((k == 0x08 || k == 0x7F) && ilen > 0) {
                ibuf[--ilen] = '\0';
                lv_textarea_set_text(input, ibuf);
            } else if ((k == '\r' || k == '\n') && ilen > 0) {
                chat_add_message(hist, "You", ibuf);
                char *msg = strdup(ibuf);
                ibuf[0] = '\0';
                ilen = 0;
                lv_textarea_set_text(input, "");
                /* Unlock around the send; it can block. */
                esp_lv_adapter_unlock();
                muse_hatch_text_turn(msg);  /* frees msg */
                display_lock(-1);
                waiting = true;
                rlen = 0;
                reply[0] = '\0';
                reply_lb = lv_label_create(hist);
                lv_label_set_text(reply_lb, "Muse: ...");
                lv_label_set_long_mode(reply_lb, LV_LABEL_LONG_WRAP);
                lv_obj_set_width(reply_lb, 296);
            }
            esp_lv_adapter_unlock();
        }
        if (waiting) {
            char evtext[1024];
            muse_hatch_ev_t ev = muse_hatch_turn_event(evtext, sizeof(evtext));
            if (ev == MUSE_HATCH_EV_REPLY && evtext[0]) {
                size_t n = strlen(evtext);
                if (rlen + n < sizeof(reply) - 1) {
                    memcpy(reply + rlen, evtext, n);
                    rlen += n;
                    reply[rlen] = '\0';
                    display_lock(-1);
                    lv_label_set_text_fmt(reply_lb, "Muse: %s", reply);
                    lv_obj_scroll_to_view(reply_lb, LV_ANIM_OFF);
                    esp_lv_adapter_unlock();
                }
            } else if (ev == MUSE_HATCH_EV_DONE) {
                waiting = false;
                reply_lb = NULL;
            } else if (ev == MUSE_HATCH_EV_ERROR) {
                display_lock(-1);
                lv_label_set_text_fmt(reply_lb, "Error: %s", evtext);
                esp_lv_adapter_unlock();
                waiting = false;
                reply_lb = NULL;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

/* Home Link's app_main starts Muse with this board (main/main.c). */
const muse_board_t *muse_board_get(void)
{
    return &s_board;
}
