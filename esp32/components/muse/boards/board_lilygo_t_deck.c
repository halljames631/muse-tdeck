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
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

#include "muse_board.h"
#include "muse_mem.h"
#include "muse_settings.h"
#include "muse_chat.h"
#include "muse_link.h"
#include "muse_wifi.h"
#include "esp_system.h"  /* esp_restart */
#include "sdkconfig.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "lwip/inet.h"

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

/* ---- Direct WiFi manager (bypasses SDK Link layer) ---- */
static bool tdeck_wifi_connect(const char *ssid, const char *pass);
static void wifi_connect_task(void *arg)
{
    char *ssid = (char *)arg;
    /* Password follows ssid in the buffer: ssid\0pass */
    char *pass = ssid + strlen(ssid) + 1;
    tdeck_wifi_connect(ssid, pass);
    free(ssid);
    vTaskDelete(NULL);
}

static bool s_wifi_connected = false;
static char s_wifi_ip[16] = {0};
static SemaphoreHandle_t s_wifi_done = NULL;

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        ESP_LOGI(TAG, "wifi disconnected, retrying");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        snprintf(s_wifi_ip, sizeof(s_wifi_ip), IPSTR, IP2STR(&e->ip_info.ip));
        s_wifi_connected = true;
        ESP_LOGI(TAG, "wifi connected, ip=%s", s_wifi_ip);
        if (s_wifi_done) xSemaphoreGive(s_wifi_done);
    }
}

static bool tdeck_wifi_connect(const char *ssid, const char *pass)
{
    if (!ssid || !ssid[0]) return false;
    ESP_LOGI(TAG, "connecting to '%s'", ssid);

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_config_t wc = {0};
    strlcpy((char *)wc.sta.ssid, ssid, sizeof(wc.sta.ssid));
    if (pass) strlcpy((char *)wc.sta.password, pass, sizeof(wc.sta.password));
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_start();

    s_wifi_done = xSemaphoreCreateBinary();
    /* Wait up to 20s for connection */
    bool ok = xSemaphoreTake(s_wifi_done, pdMS_TO_TICKS(20000)) == pdTRUE;
    vSemaphoreDelete(s_wifi_done);
    s_wifi_done = NULL;
    return ok && s_wifi_connected;
}

/* ---- Console interceptor: captures @chat JSON replies ---- */
#define CHAT_Q_LEN 8
typedef struct {
    char type[16];
    char text[512];
} chat_msg_t;
static QueueHandle_t s_chat_q = NULL;
static FILE *s_orig_stdout = NULL;

static int chat_intercept_write(void *ctx, const char *buf, int n)
{
    (void)ctx;
    /* Scan for @chat lines */
    const char *p = buf;
    const char *end = buf + n;
    while (p < end) {
        const char *line_end = memchr(p, '\n', end - p);
        if (!line_end) line_end = end;
        if ((line_end - p) > 6 && memcmp(p, "@chat ", 6) == 0) {
            /* Parse {"seq":N,"type":"TYPE",...,"text":"TEXT"} */
            chat_msg_t m = {0};
            const char *t = strstr(p, "\"type\":\"");
            if (t) {
                t += 8;
                const char *q = strchr(t, '"');
                if (q && (q - t) < (int)sizeof(m.type)) {
                    memcpy(m.type, t, q - t);
                }
            }
            const char *x = strstr(p, "\"text\":\"");
            if (x) {
                x += 8;
                /* Unescape simple sequences, copy until closing quote */
                char *o = m.text;
                size_t cap = sizeof(m.text) - 1;
                while (*x && *x != '"' && cap > 0) {
                    if (*x == '\\' && x[1]) {
                        x++;
                        *o++ = (*x == 'n') ? '\n' : (*x == 't') ? '\t' : *x;
                        x++; cap--;
                    } else {
                        *o++ = *x++; cap--;
                    }
                }
                *o = '\0';
            }
            if (m.type[0] && s_chat_q) {
                xQueueSend(s_chat_q, &m, 0);
            }
        }
        p = line_end + 1;
    }
    /* Forward to original stdout so USB serial still works */
    if (s_orig_stdout) {
        return fwrite(buf, 1, n, s_orig_stdout);
    }
    return n;
}

static void chat_intercept_init(void)
{
    s_chat_q = xQueueCreate(CHAT_Q_LEN, sizeof(chat_msg_t));
    s_orig_stdout = stdout;
    /* Replace stdout with our interceptor */
    stdout = funopen(NULL, NULL, chat_intercept_write, NULL, NULL);
    if (!stdout) {
        stdout = s_orig_stdout;
        ESP_LOGW(TAG, "funopen failed; chat replies won't be captured");
    } else {
        ESP_LOGI(TAG, "console interceptor installed");
    }
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
    /* Start the chat UI (revamped: handles WiFi status + messaging). */
    xTaskCreate(chat_task, "tdeck_chat", 12288, NULL, 5, NULL);
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

/* Robust WiFi: retry muse_settings_set_wifi() until Link reports connected.
 * The keeper task may not be ready on the first try. */
static void wifi_retry_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(20000));  /* let SDK fully boot */
    const char *ssid = NULL;
    const char *pass = "";
#ifdef CONFIG_HOMEHUB_WIFI_SSID
    if (CONFIG_HOMEHUB_WIFI_SSID[0]) {
        ssid = CONFIG_HOMEHUB_WIFI_SSID;
#ifdef CONFIG_HOMEHUB_WIFI_PASSWORD
        pass = CONFIG_HOMEHUB_WIFI_PASSWORD;
#endif
    }
#endif
    if (!ssid) {
        ESP_LOGW(TAG, "no WiFi SSID configured");
        vTaskDelete(NULL);
        return;
    }
    for (int i = 0; i < 6; i++) {
        ESP_LOGI(TAG, "WiFi attempt %d for '%s'", i + 1, ssid);
        muse_settings_set_wifi(ssid, pass);
        /* Also kick the Link layer directly */
        muse_wifi_apply();
        vTaskDelay(pdMS_TO_TICKS(15000));
        if (muse_wifi_connected()) {
            ESP_LOGI(TAG, "WiFi connected!");
            break;
        }
        ESP_LOGW(TAG, "WiFi not connected yet, retrying");
    }
    vTaskDelete(NULL);
}

static void ondevice_setup(void)
{
    /* Copy Kconfig SDK token into runtime settings if not already saved.
     * muse_hatch_configured() checks s.token, not the Kconfig value. */
#ifdef CONFIG_GADGET_SDK_TOKEN
    if (CONFIG_GADGET_SDK_TOKEN[0] != '\0') {
        if (muse_settings_hatch_token_len() == 0) {
            muse_settings_set_hatch_token(CONFIG_GADGET_SDK_TOKEN, false);
            ESP_LOGI(TAG, "saved Kconfig SDK token to muse settings");
        }
        /* Also overwrite homehub NVS (identity.c reads sdk_token from there,
         * overriding Kconfig if present). Fixes stale token issue. */
        nvs_handle_t h;
        bool need_reboot = false;
        if (nvs_open("homehub", NVS_READWRITE, &h) == ESP_OK) {
            char cur[64] = {0};
            size_t len = sizeof(cur);
            bool same = (nvs_get_str(h, "sdk_token", cur, &len) == ESP_OK &&
                         strcmp(cur, CONFIG_GADGET_SDK_TOKEN) == 0);
            if (!same) {
                nvs_set_str(h, "sdk_token", CONFIG_GADGET_SDK_TOKEN);
                nvs_commit(h);
                ESP_LOGI(TAG, "wrote new SDK token to homehub NVS; rebooting");
                need_reboot = true;
            }
            nvs_close(h);
        }
        if (need_reboot) {
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        }
    }
#endif
    /* Revamped: connect WiFi directly via ESP-IDF. Credentials from
     * CONFIG_HOMEHUB_WIFI_SSID/PASSWORD (SDK standard), or TDECK_ fallback.
     * The chat task shows the UI; this just ensures network is up. */
    const char *ssid = NULL;
    const char *pass = NULL;
#ifdef CONFIG_HOMEHUB_WIFI_SSID
    if (CONFIG_HOMEHUB_WIFI_SSID[0] != '\0') {
        ssid = CONFIG_HOMEHUB_WIFI_SSID;
#ifdef CONFIG_HOMEHUB_WIFI_PASSWORD
        pass = CONFIG_HOMEHUB_WIFI_PASSWORD;
#endif
    }
#endif
#ifdef CONFIG_TDECK_WIFI_SSID
    if (!ssid && CONFIG_TDECK_WIFI_SSID[0] != '\0') {
        ssid = CONFIG_TDECK_WIFI_SSID;
#ifdef CONFIG_TDECK_WIFI_PASS
        pass = CONFIG_TDECK_WIFI_PASS;
#endif
    }
#endif
    if (ssid) {
        ESP_LOGI(TAG, "starting WiFi retry task for '%s'", ssid);
        xTaskCreate(wifi_retry_task, "tdeck_wifiretry", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGW(TAG, "no WiFi credentials configured");
    }
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
    (void)arg;
    /* Wait for SDK UI to settle before taking over the screen. */
    vTaskDelay(pdMS_TO_TICKS(12000));
    /* Revamped chat: full-screen UI, keyboard input.
     * NOTE: console interceptor disabled (caused black screen). */
    /* chat_intercept_init(); */

    display_lock(-1);
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_screen_load(scr);

    /* Status bar */
    lv_obj_t *status = lv_label_create(scr);
    lv_obj_set_pos(status, 5, 2);
    lv_obj_set_width(status, 310);
    lv_label_set_text(status, s_wifi_connected ? "WiFi OK" : "Connecting WiFi...");

    /* Message history */
    lv_obj_t *hist = lv_obj_create(scr);
    lv_obj_set_size(hist, 320, 170);
    lv_obj_set_pos(hist, 0, 20);
    lv_obj_set_flex_flow(hist, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(hist, LV_DIR_VER);

    /* Input field */
    lv_obj_t *input = lv_textarea_create(scr);
    lv_textarea_set_one_line(input, true);
    lv_textarea_set_max_length(input, 512);
    lv_textarea_set_placeholder_text(input, "Type message, Enter sends");
    lv_obj_set_size(input, 310, 36);
    lv_obj_set_pos(input, 5, 196);
    esp_lv_adapter_unlock();

    display_lock(-1);
    chat_add_message(hist, "Sys", "T-Deck Muse Chat. Type below, Enter sends.");
    if (!muse_wifi_connected()) {
        chat_add_message(hist, "Sys", "Waiting for WiFi...");
    }
    esp_lv_adapter_unlock();

    char ibuf[513] = {0};
    size_t ilen = 0;
    bool waiting = false;
    lv_obj_t *reply_lb = NULL;
    char reply_accum[4096] = {0};

    while (1) {
        /* Update WiFi status (use Link API, not our disabled direct var) */
        bool wup = muse_wifi_connected();
        display_lock(-1);
        lv_label_set_text(status, wup ? "WiFi OK - type below" : "Connecting WiFi...");
        esp_lv_adapter_unlock();

        /* Keyboard input */
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
                if (!muse_wifi_connected()) {
                    chat_add_message(hist, "Sys", "No WiFi yet, can't send.");
                } else {
                    chat_add_message(hist, "You", ibuf);
                    char *msg = strdup(ibuf);
                    ibuf[0] = '\0'; ilen = 0;
                    lv_textarea_set_text(input, "");
                    esp_lv_adapter_unlock();
                    muse_hatch_text_turn(msg);  /* frees msg */
                    display_lock(-1);
                    waiting = true;
                    reply_accum[0] = '\0';
                    reply_lb = lv_label_create(hist);
                    lv_label_set_text(reply_lb, "Muse: thinking...");
                    lv_label_set_long_mode(reply_lb, LV_LABEL_LONG_WRAP);
                    lv_obj_set_width(reply_lb, 296);
                    lv_obj_scroll_to_view(reply_lb, LV_ANIM_OFF);
                }
            }
            esp_lv_adapter_unlock();
        }

        /* Check for @chat replies from the interceptor queue */
        if (waiting && s_chat_q) {
            chat_msg_t m;
            while (xQueueReceive(s_chat_q, &m, 0) == pdTRUE) {
                if (strcmp(m.type, "text") == 0 && m.text[0]) {
                    /* Append partial text */
                    strlcat(reply_accum, m.text, sizeof(reply_accum));
                    display_lock(-1);
                    lv_label_set_text_fmt(reply_lb, "Muse: %s", reply_accum);
                    lv_obj_scroll_to_view(reply_lb, LV_ANIM_OFF);
                    esp_lv_adapter_unlock();
                } else if (strcmp(m.type, "done") == 0 ||
                           strcmp(m.type, "message_done") == 0) {
                    waiting = false;
                    display_lock(-1);
                    lv_obj_scroll_to_view(reply_lb, LV_ANIM_OFF);
                    esp_lv_adapter_unlock();
                    reply_lb = NULL;
                    break;
                } else if (strcmp(m.type, "error") == 0) {
                    display_lock(-1);
                    lv_label_set_text_fmt(reply_lb, "Error: %s", m.text[0] ? m.text : "?");
                    esp_lv_adapter_unlock();
                    waiting = false;
                    reply_lb = NULL;
                    break;
                } else if (strcmp(m.type, "busy") == 0) {
                    display_lock(-1);
                    lv_label_set_text(reply_lb, "Muse: working...");
                    esp_lv_adapter_unlock();
                }
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
