# LILYGO T-Deck as a Muse Gadget — Custom Deployment Guide

Target: **LILYGO T-Deck, ESP32-S3FN16R8 (16 MB flash, 8 MB octal PSRAM)**
SDK: **facebookincubator/muse-gadget-sdk**, `esp32/` firmware (Home Link + Muse UI)
ESP-IDF: **v6.0.1** (the only supported version — others break on missing IDF headers)

## Read this first

The T-Deck is **not** one of the SDK's supported boards (checked against
`esp32/devices/` and the Supported-boards table in `esp32/AGENTS.md` on
2026-10-15). So this is a **new-board port** following the recipe in
`esp32/devices/AGENTS.md`, with every pin below verified against the vendor
source — the `utilities.h` pin map, `Microphone`/`I2SPlay`/`Touchpad`
examples, and schematic in
[Xinyuan-LilyGO/T-Deck](https://github.com/Xinyuan-LilyGO/T-Deck) (MIT),
not guessed. Where a value can only be settled on hardware, it is marked
**verify on hardware**.

What you will create (names follow the repo's conventions):

| # | File | Purpose |
|---|---|---|
| 1 | `esp32/devices/sdkconfig.muse-lilygo-t-deck` | Board overlay: target, flash/PSRAM, console, board select |
| 2 | `esp32/components/muse/boards/board_lilygo_t_deck.c` | `muse_board_t`: power, display, touch, audio, buttons, battery |
| 3 | `esp32/components/muse/Kconfig` | Add `MUSE_BOARD_LILYGO_T_DECK` + `MUSE_BOARD_ID` default |
| 4 | `esp32/components/muse/CMakeLists.txt` | Compile the new board file |
| 5 | `esp32/components/muse/idf_component.yml` | Pull `esp_lcd_touch_gt911` for this board |
| 6 | `esp32/tools/muse/board.sh` | Add `tdeck` alias |
| 7 | `esp32/tools/muse/ports.py` | Map `tdeck` to native USB + serial commands |
| 8 | `esp32/tools/muse/avatar.py` | Map board name → `tdeck` (for avatar updates) |

The T-Deck has a screen, mic, and speaker, so per `devices/AGENTS.md`
("A board with a screen, mic and speaker should run the full UI") this is a
**UI-kind** port: overlay on top of `sdkconfig.muse`, 16 MB flash uses
`partitions_muse.csv` (already selected by `sdkconfig.muse`).

---

## 1. I2S audio pipeline (mic + speaker)

### Verified hardware map

| Function | Chip | Pins (GPIO) | Notes |
|---|---|---|---|
| Peripheral power | — | **10** (`BOARD_POWERON`) | Drive **HIGH** in `init()` or nothing analog works |
| Speaker amp | **MAX98357A** (from T-Deck `schematic.pdf`) | I2S TX: BCK **7**, WS **5**, DOUT **6** | No MCLK, no control interface — fixed gain |
| Microphone ADC | **ES7210**, I2C addr **0x40** (`ES7210_AD1_AD0_00`, vendor `lib/es7210`) | I2S RX: MCLK **48**, WS/LRCK **21**, BCK/SCK **47**, DIN **14** | Vendor runs it 16 kHz / 16-bit, ESP32 as I2S master |
| Shared I2C bus | — | SDA **18**, SCL **8** | Also carries GT911 touch + keyboard |

Muse's audio core runs at `MUSE_AUDIO_RATE` = **16000 Hz**
(`components/muse/muse_audio.h`), which matches the vendor's 16 kHz mic
example exactly — no resampling needed.

### Pipeline design

Two independent I2S peripherals (they cannot share one: different pin sets
and different roles):

- **I2S_NUM_0, TX only** → MAX98357A speaker. Standard Philips, 16-bit
  stereo, 16 kHz. MCLK = `GPIO_NUM_NC` (the MAX98357A derives its clock
  internally; the vendor's `I2SPlay` example also passes no MCLK).
- **I2S_NUM_1, RX only** → ES7210 dual mic. Standard Philips, 16-bit
  stereo, 16 kHz, MCLK on GPIO48 (the ES7210 needs it; the vendor's
  `Microphone` example wires `mck_io_num = BOARD_ES7210_MCLK`).

The ES7210 side follows the exact pattern already proven in-repo by
`components/muse/boards/board_ostb_echoear_2st.c` (it probes `0x40` and
builds the mic with `es7210_codec_new()` + `ES7210_SEL_MIC1 |
ES7210_SEL_MIC2`).

The MAX98357A has **no control interface**, but Muse's `muse_audio.c`
drives the speaker exclusively through `esp_codec_dev_*` calls
(`open`/`write`/`close`/`set_out_vol`). The documented solution (see
`esp_codec_dev` README, "Customize a new codec device") is a minimal
`audio_codec_if_t` whose ops are no-ops returning `ESP_OK` — the amp is
fixed-function. Implement it in the board file following
`device/template_codec/` in `espressif/esp-audio-dev`, checking exact
signatures against
`managed_components/espressif__esp_codec_dev/include/audio_codec_if.h`
after your first build. Consequence to know: **speaker volume is fixed in
hardware**; Muse's volume slider will have no effect unless you add a
software gain stage later.

### `audio_init()` for the board file

```c
#define I2C_SDA GPIO_NUM_18
#define I2C_SCL GPIO_NUM_8

/* MAX98357A speaker amp: I2S0 TX, no MCLK, no control interface. */
#define SPK_BCK  GPIO_NUM_7
#define SPK_WS   GPIO_NUM_5
#define SPK_DOUT GPIO_NUM_6

/* ES7210 dual-mic ADC: I2S1 RX, I2C 0x40. */
#define MIC_MCLK GPIO_NUM_48
#define MIC_WS   GPIO_NUM_21
#define MIC_BCK  GPIO_NUM_47
#define MIC_DIN  GPIO_NUM_14

static esp_err_t audio_init(esp_codec_dev_handle_t *spk,
                            esp_codec_dev_handle_t *mic)
{
    /* --- speaker: I2S0 TX -> MAX98357A --- */
    i2s_chan_handle_t tx;
    i2s_chan_config_t tx_chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    tx_chan_cfg.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&tx_chan_cfg, &tx, NULL),
                        TAG, "spk i2s channel");
    const i2s_std_config_t tx_std = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(MUSE_AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = { .mclk = GPIO_NUM_NC, .bclk = SPK_BCK,
                      .ws = SPK_WS, .dout = SPK_DOUT,
                      .din = GPIO_NUM_NC },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx, &tx_std),
                        TAG, "spk i2s init");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(tx), TAG, "spk i2s enable");

    audio_codec_i2s_cfg_t spk_data_cfg =
        { .port = I2S_NUM_0, .rx_handle = NULL, .tx_handle = tx };
    const audio_codec_data_if_t *spk_data =
        audio_codec_new_i2s_data(&spk_data_cfg);
    /* Fixed-function amp: minimal audio_codec_if_t, all ops succeed,
     * set_vol is a no-op (see note above). */
    const audio_codec_if_t *spk_codec = max98357a_codec_new();
    esp_codec_dev_cfg_t spk_cfg = { .dev_type = ESP_CODEC_DEV_TYPE_OUT,
                                    .codec_if = spk_codec,
                                    .data_if = spk_data };
    *spk = esp_codec_dev_new(&spk_cfg);

    /* --- mic: I2S1 RX <- ES7210, controlled over I2C --- */
    ESP_RETURN_ON_ERROR(i2c_master_probe(s_i2c, 0x40, 100),
                        TAG, "ES7210 probe");
    i2s_chan_handle_t rx;
    i2s_chan_config_t rx_chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&rx_chan_cfg, NULL, &rx),
                        TAG, "mic i2s channel");
    const i2s_std_config_t rx_std = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(MUSE_AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = { .mclk = MIC_MCLK, .bclk = MIC_BCK,
                      .ws = MIC_WS, .dout = GPIO_NUM_NC,
                      .din = MIC_DIN },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx, &rx_std),
                        TAG, "mic i2s init");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx), TAG, "mic i2s enable");

    audio_codec_i2s_cfg_t mic_data_cfg =
        { .port = I2S_NUM_1, .rx_handle = rx, .tx_handle = NULL };
    const audio_codec_data_if_t *mic_data =
        audio_codec_new_i2s_data(&mic_data_cfg);
    audio_codec_i2c_cfg_t mic_ctrl_cfg =
        { .port = I2C_NUM_0, .addr = 0x40, .bus_handle = s_i2c };
    const audio_codec_ctrl_if_t *mic_ctrl =
        audio_codec_new_i2c_ctrl(&mic_ctrl_cfg);
    es7210_codec_cfg_t adc = { .ctrl_if = mic_ctrl,
                               .mic_selected = ES7210_SEL_MIC1 |
                                               ES7210_SEL_MIC2 };
    const audio_codec_if_t *mic_codec = es7210_codec_new(&adc);
    esp_codec_dev_cfg_t mic_cfg = { .dev_type = ESP_CODEC_DEV_TYPE_IN,
                                    .codec_if = mic_codec,
                                    .data_if = mic_data };
    *mic = esp_codec_dev_new(&mic_cfg);
    return *spk && *mic ? ESP_OK : ESP_FAIL;
}
```

Board-struct audio fields: `.mic_slot = -1` (mix both ES7210 mics, as the
BOX-3 board does), `.set_mic_gain = NULL` (let `esp_codec_dev` drive the
ES7210 gain; Muse calls `esp_codec_dev_set_in_gain()`).

### `init()` power sequencing (do not skip)

```c
#define POWERON_GPIO GPIO_NUM_10
#define TFT_CS  GPIO_NUM_12
#define RADIO_CS GPIO_NUM_9
#define SD_CS   GPIO_NUM_39

static esp_err_t init(void)
{
    /* Peripherals (display, audio, touch) are dead until this is HIGH. */
    gpio_set_direction(POWERON_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(POWERON_GPIO, 1);

    /* Park the SPI bus-mates: LoRa SX1262 and the SD slot share the
     * display's SPI bus (MOSI 41 / MISO 38 / SCK 40). Muse uses neither. */
    const gpio_config_t cs_out = {
        .pin_bit_mask = (1ULL << TFT_CS) | (1ULL << RADIO_CS) |
                        (1ULL << SD_CS),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cs_out);
    gpio_set_level(TFT_CS, 1);
    gpio_set_level(RADIO_CS, 1);
    gpio_set_level(SD_CS, 1);

    /* I2C bus 0 for ES7210 + GT911. */
    const i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_cfg, &s_i2c), TAG, "i2c");
    /* BOOT (GPIO0) as the talk button; touch gives a second path. */
    return muse_gpio_button_init(&s_talk, GPIO_NUM_0);
}
```

---

## 2. Display driver (ST7789 320×240 + GT911 touch)

### Verified hardware map

| Function | Pins (GPIO) | Notes |
|---|---|---|
| SPI bus | SCLK **40**, MOSI **41**, MISO **38** (shared, display doesn't need MISO) | Shared with LoRa + SD; park their CS (above) |
| ST7789 | CS **12**, DC **11** | No reset GPIO routed (vendor examples never drive one) |
| Backlight | **42** | Vendor `BOARD_TFT_BACKLIGHT` / `BOARD_BL_PIN` |
| Touch (GT911) | I2C SDA **18** / SCL **8**, INT **16**, addr **0x5D** (`GT911_SLAVE_ADDRESS_L`; probe `0x14` as fallback) | 320×240, vendor uses swapXY + mirrorY |

Orientation, derived from the vendor's own settings and verified against
`lib/TFT_eSPI/TFT_Drivers/ST7789_Rotation.h` in the T-Deck repo: the vendor
runs `tft.setRotation(1)` (landscape), which programs MADCTL =
`MX | MV | <color order>`. In `esp_lcd` terms that is exactly:

```c
esp_lcd_panel_swap_xy(s_panel, true);
esp_lcd_panel_mirror(s_panel, true, false);
```

with `width = 320`, `height = 240`. (This matches the in-repo AIPI board's
ST7789 setup.) Use `LCD_RGB_ELEMENT_ORDER_BGR`; if red/blue look swapped on
first boot, flip to `RGB` — a 10-second check.

### `display_start()` for the board file

```c
#define LCD_H_RES 320
#define LCD_V_RES 240
#define LCD_HOST  SPI3_HOST
#define LCD_SCLK  GPIO_NUM_40
#define LCD_MOSI  GPIO_NUM_41
#define LCD_CS    GPIO_NUM_12
#define LCD_DC    GPIO_NUM_11
#define LCD_BL    GPIO_NUM_42
#define DRAW_BUF_LINES 32   /* 320*32*2 = 20 KB per DMA buffer */

static lv_display_t *display_start(lv_indev_t **touch)
{
    /* Backlight: LEDC PWM like the in-repo AIPI board. */
    const ledc_timer_config_t bl_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    const ledc_channel_config_t bl_ch = {
        .gpio_num = LCD_BL, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .duty = 0,
    };
    if (ledc_timer_config(&bl_timer) != ESP_OK ||
        ledc_channel_config(&bl_ch) != ESP_OK) {
        return NULL;
    }

    const spi_bus_config_t bus = {
        .sclk_io_num = LCD_SCLK, .mosi_io_num = LCD_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC, .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = LCD_H_RES * DRAW_BUF_LINES * 2,
    };
    if (spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) {
        return NULL;
    }
    esp_lcd_panel_io_handle_t io;
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = LCD_CS, .dc_gpio_num = LCD_DC,
        .spi_mode = 0, .pclk_hz = 40 * 1000 * 1000,
        .trans_queue_depth = 10, .lcd_cmd_bits = 8, .lcd_param_bits = 8,
    };
    if (esp_lcd_new_panel_io_spi(LCD_HOST, &io_cfg, &io) != ESP_OK) {
        return NULL;
    }
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = GPIO_NUM_NC,   /* not routed; software reset */
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    if (esp_lcd_new_panel_st7789(io, &panel_cfg, &s_panel) != ESP_OK) {
        return NULL;
    }
    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_swap_xy(s_panel, true);    /* = vendor setRotation(1) */
    esp_lcd_panel_mirror(s_panel, true, false);
    esp_lcd_panel_disp_on_off(s_panel, true);

    /* GT911 touch: vendor Touchpad example uses addr 0x5D, INT on GPIO16,
     * 320x240 coords, swapXY(true), mirrorXY(false, true). */
    ESP_RETURN_ON_FALSE(
        i2c_master_probe(s_i2c, 0x5D, 50) == ESP_OK ||
        i2c_master_probe(s_i2c, 0x14, 50) == ESP_OK,
        NULL, TAG, "GT911 probe");
    esp_lcd_panel_io_handle_t tp_io;
    esp_lcd_panel_io_i2c_config_t tp_io_cfg =
        ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    if (esp_lcd_new_panel_io_i2c(s_i2c, &tp_io_cfg, &tp_io) != ESP_OK) {
        return NULL;
    }
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES, .y_max = LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC, .int_gpio_num = GPIO_NUM_16,
        .levels = { .reset = 0, .interrupt = 0 },
        .flags = { .swap_xy = 1, .mirror_x = 0, .mirror_y = 1 },
    };
    if (esp_lcd_touch_new_i2c_gt911(tp_io, &tp_cfg, &s_touch) != ESP_OK) {
        return NULL;
    }

    /* LVGL through esp_lv_adapter, same as the in-repo AIPI board. */
    esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    adapter_cfg.task_core_id = MUSE_UI_CORE;
    adapter_cfg.task_priority = MUSE_UI_PRIORITY;
    if (esp_lv_adapter_init(&adapter_cfg) != ESP_OK) {
        return NULL;
    }
    const esp_lv_adapter_display_config_t disp_cfg = {
        .panel = s_panel, .panel_io = io,
        .profile = {
            .interface = ESP_LV_ADAPTER_PANEL_IF_OTHER,
            .rotation = ESP_LV_ADAPTER_ROTATE_0,
            .hor_res = LCD_H_RES, .ver_res = LCD_V_RES,
            .buffer_height = DRAW_BUF_LINES,
            .use_psram = false, .require_double_buffer = true,
        },
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE,
    };
    lv_display_t *disp = esp_lv_adapter_register_display(&disp_cfg);
    if (!disp || esp_lv_adapter_start() != ESP_OK) {
        return NULL;
    }
    *touch = lv_display_get_indev(disp);  /* touch input device */
    return disp;
}
```

**Verify on hardware:** if the image is mirrored or touch axes are off,
flip the `mirror()` flags first, then the touch `mirror_x/mirror_y` — the
panel and touch transforms are independent.

Board-struct display fields:

```c
.name = "LILYGO T-Deck",
.width = 320, .height = 240,
.diagonal_in = 2.8f,
.round = false, .touch = true, .touch_talk = true,
.talk_button = "boot",
.talk_hint = { LV_ALIGN_TOP_LEFT, 8, 8 },
.frame_ms = 40,
.display_start = display_start,
.display_lock = display_lock,        /* esp_lv_adapter_lock */
.display_unlock = esp_lv_adapter_unlock,
.set_brightness = set_brightness,   /* LEDC duty on GPIO42 */
.panel_sleep = panel_sleep,         /* esp_lcd_panel_disp_on_off sleep */
```

`.touch_talk = true` gives push-to-talk through the on-screen mic even if
your unit's BOOT button is awkward to reach.

### What this leaves out (deliberately)

- **QWERTY keyboard**: driven by an onboard ESP32-C3 over I2C (INT on
  GPIO46). Not in Muse's input contract; wiring it into `poll_buttons()`
  as `MUSE_BTN_*` nav keys is optional follow-up work.
- **Trackball**: no Muse contract; skip.
- **LoRa SX1262** (CS 9, BUSY 13, RST 17, DIO1 45): Muse doesn't use it.
  Its CS is parked HIGH in `init()` so it never fights the display.
- **GPS** (43/44, optional module): out of scope.

---

## 3. `sdkconfig` for 16 MB flash + 8 MB octal PSRAM

Create `esp32/devices/sdkconfig.muse-lilygo-t-deck`. It loads **last**,
after `sdkconfig.defaults` and `sdkconfig.muse`. The base `sdkconfig.defaults`
assumes 8 MB flash and **quad** PSRAM — both must be overridden for the
`ESP32-S3FN16R8` (`R8` = 8 MB **octal** PSRAM).

```ini
# Copyright (c) Meta Platforms, Inc. and affiliates.
# Licensed under the Apache License, Version 2.0 (the "License");
# ... (keep the repo's Apache header)

# LILYGO T-Deck: ESP32-S3FN16R8, 16 MB flash, 8 MB octal PSRAM,
# 320x240 ST7789, GT911 touch, ES7210 mic, MAX98357A speaker.
# Native USB Serial/JTAG (303a:1001), BOOT on GPIO0.
# Pin source: https://github.com/Xinyuan-LilyGO/T-Deck (utilities.h, MIT)
# Load last, after sdkconfig.defaults and sdkconfig.muse.
CONFIG_IDF_TARGET="esp32s3"
CONFIG_MUSE_BOARD_LILYGO_T_DECK=y
CONFIG_HOMEHUB_BUTTON_GPIO=0
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_ESPTOOLPY_FLASHSIZE="16MB"
CONFIG_ESPTOOLPY_FLASHMODE_QIO=y
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y

# PSRAM is required for the UI, images, voice session and tunnel.
# FN16R8 = 8 MB *octal* PSRAM: override sdkconfig.defaults' quad setting.
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_MODE_QUAD=n
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_SPIRAM_FETCH_INSTRUCTIONS=y
CONFIG_SPIRAM_RODATA=y
CONFIG_SPIRAM_XIP_FROM_PSRAM=y
CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y
CONFIG_ESP32S3_INSTRUCTION_CACHE_32KB=y
CONFIG_ESP32S3_DATA_CACHE_64KB=y
CONFIG_ESP32S3_DATA_CACHE_LINE_64B=y
CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=2
CONFIG_BT_NIMBLE_MEM_ALLOC_MODE_EXTERNAL=y
CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y
CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY=y
CONFIG_SPIRAM_IGNORE_NOTFOUND=n
CONFIG_ESP_WIFI_STATIC_TX_BUFFER_NUM=8
```

Notes:

- `sdkconfig.muse` already selects `partitions_muse.csv` (dual 4 MB app
  slots — needs the full 16 MB) and `CONFIG_ESPTOOLPY_FLASHSIZE_16MB`; the
  lines above restate the sizing so the overlay is self-documenting. With
  16 MB flash you do **not** need `partitions_muse_8mb.csv`.
- PSRAM size is auto-detected at boot; the boot log's `PSRAM: 8MB` line is
  your confirmation. `CONFIG_SPIRAM_IGNORE_NOTFOUND=n` makes a missing
  PSRAM a hard boot failure instead of a silent one.
- `CONFIG_HOMEHUB_BUTTON_GPIO=0` is BOOT; the full UI uses it as the talk
  button via `poll_buttons()`.

### The other four edits

**`esp32/components/muse/Kconfig`** — in the `MUSE_BOARD` choice, next to
the other S3 boards:

```kconfig
        config MUSE_BOARD_LILYGO_T_DECK
            bool "LILYGO T-Deck (ESP32-S3, 320x240 ST7789, GT911 touch)"
            depends on IDF_TARGET_ESP32S3
```

and in `MUSE_BOARD_ID`, above `default "none"`:

```kconfig
        default "lilygo_t_deck" if MUSE_BOARD_LILYGO_T_DECK
```

**`esp32/components/muse/CMakeLists.txt`** — beside the other
`elseif()` board lines:

```cmake
        elseif(CONFIG_MUSE_BOARD_LILYGO_T_DECK)
            list(APPEND srcs "boards/board_lilygo_t_deck.c")
```

**`esp32/components/muse/idf_component.yml`** — extend the GT911 rule so
the touch driver is pulled for this board:

```yaml
  espressif/esp_lcd_touch_gt911:
    version: "1.2.1"
    rules:
      - if: "$CONFIG{MUSE_BOARD_ID} in [\"waveshare_s3_lcd7\", \"lilygo_t_deck\"]"
```

**`esp32/tools/muse/board.sh`** — usage comment gains `tdeck`, and the
case list gains:

```sh
    tdeck)    profile=lilygo-t-deck;    target=esp32s3 ;;
```

(no `baud=` override: native USB takes the default 460800).

**`esp32/tools/muse/ports.py`** — the T-Deck flashes over the chip's own
USB Serial/JTAG (`303a:1001`, shows as `/dev/ttyACM0` on Linux):

```python
    "tdeck": USJ,
```

and add `"tdeck"` to `COMMANDS`. In `tools/muse/avatar.py`, map the
board's `name` (`"LILYGO T-Deck"`) to the `tdeck` alias in `BOARDS`.

After any overlay/Kconfig change, delete the stale generated config before
rebuilding (`build-muse-lilygo-t-deck/sdkconfig` overrides later edits to
the defaults files), then confirm every overlay line landed:

```sh
o=devices/sdkconfig.muse-lilygo-t-deck; s=build-muse-lilygo-t-deck/sdkconfig
grep '^CONFIG_' "$o" | while IFS= read -r l; do
  k=${l%%=*}
  case $l in *=n) ! grep -q "^$k=" "$s" ;; *) grep -qxF "$l" "$s" ;; esac ||
    echo "not applied: $l"
done
```

---

## 4. Build environment, token, compile, flash

### 4.1 Install ESP-IDF v6.0.1 (exact)

```sh
mkdir -p ~/esp && cd ~/esp
git clone -b v6.0.1 --recursive \
  https://github.com/espressif/esp-idf.git esp-idf-v6.0.1
cd esp-idf-v6.0.1
./install.sh esp32s3
. ./export.sh            # puts idf.py on PATH for this shell
```

`tools/muse/board.sh` finds IDF on its own from `~/.espressif/esp-idf-v6.0.1`,
`$IDF_PATH`, `~/esp/esp-idf-v6.0.1`, `~/esp/esp-idf-v6`, `~/esp/esp-idf`, or
`$IDF_EXPORT` — or just keep `export.sh` sourced. Do **not** substitute v5.x:
the AGENTS.md warns that missing-header errors (`gcm.h`, `gpio_ll.h`) mean
the wrong IDF tag.

Python 3 and a C compiler are also needed for the host tests.

### 4.2 Clone the SDK and add your board files

```sh
git clone https://github.com/facebookincubator/muse-gadget-sdk.git
cd muse-gadget-sdk/esp32
# ... apply edits #1–#8 from sections 2–3 ...
```

### 4.3 Inject your SDK developer token

Every application build needs your token (`mgst_…`, from
[gadgets.muse.ai](https://gadgets.muse.ai) → Account → SDK tokens).
Without it the build warns and the gadget will stop pairing once Muse
requires tokens. **Never commit it or print it in full.**

```sh
# Option A: menuconfig (recommended)
idf.py -B build-muse-lilygo-t-deck menuconfig
# → "ESP32 Device SDK" → Gadget SDK token → paste mgst_… → save

# Option B: append to the build dir's sdkconfig after first configure
printf 'CONFIG_GADGET_SDK_TOKEN="mgst_…"\n' \
  >> build-muse-lilygo-t-deck/sdkconfig
```

The generated `sdkconfig` is gitignored — the token never lands in the repo.

### 4.4 Compile

```sh
cd muse-gadget-sdk/esp32
tools/muse/board.sh build tdeck
# log: /tmp/muse_build_tdeck.log ; output: build-muse-lilygo-t-deck/
```

This is equivalent to:

```sh
idf.py -B build-muse-lilygo-t-deck -DIDF_TARGET=esp32s3 \
  -DSDKCONFIG=build-muse-lilygo-t-deck/sdkconfig \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;devices/sdkconfig.muse;devices/sdkconfig.muse-lilygo-t-deck" \
  build
```

Check the `check_sizes` line: app slots are 4 MB on Muse builds — flag
anything under 10% free. Then run the host tests:

```sh
python3 -m unittest discover -s tests -p 'test_*.py'
```

### 4.5 Flash over native USB-C

The T-Deck exposes the ESP32-S3's **native USB Serial/JTAG**
(`303a:1001`) — no serial-driver install needed. On Linux it appears as
`/dev/ttyACM0` (add yourself to the `dialout` group); on macOS as
`/dev/cu.usbmodem*`.

```sh
# Sanity: confirm the chip before you write another board's pin map to it
python -m esptool -p /dev/ttyACM0 chip-id      # expect esp32s3

tools/muse/board.sh flash tdeck /dev/ttyACM0
# or explicitly:
idf.py -B build-muse-lilygo-t-deck -DIDF_TARGET=esp32s3 \
  -DSDKCONFIG=build-muse-lilygo-t-deck/sdkconfig \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;devices/sdkconfig.muse;devices/sdkconfig.muse-lilygo-t-deck" \
  -p /dev/ttyACM0 flash
```

`flash` writes bootloader, partition table (at `0x10000`), otadata,
phy_init and the app to `ota_0`. NVS is left alone, so pairing and Wi-Fi
survive reflashes. If the chip won't enter the bootloader: hold BOOT, tap
RESET, release BOOT, flash again. Secure Boot stays **off** — flashing
never burns eFuses, so you can keep reflashing.

### 4.6 Verify the boot

```sh
idf.py -B build-muse-lilygo-t-deck -p /dev/ttyACM0 monitor
# or: python3 tools/muse/monitor.py /dev/ttyACM0 30
```

Healthy boot shows:

- `link.main: Muse Gadget starting`
- `muse: board: LILYGO T-Deck` (from your board file's `.name`)
- PSRAM found: `... PSRAM: 8MB ...` (octal mode)
- No panic / no reboot loop
- Status breathes **orange** → BLE advertises as `MuseGadget-XXXXXX`

### 4.7 Pair

1. Open the Muse app → add a gadget → pick `MuseGadget-XXXXXX`.
   This uses **community pairing v5** — the app must support v5 and list
   community devices.
2. When it breathes **blue**, confirm on the device: short-press BOOT
   (GPIO0) or tap the on-screen mic (`.touch_talk`).
3. **Green** = connected (tunnel/session up). Hold the talk control, speak,
   release — Muse answers through the MAX98357A speaker.
4. Iterating on Wi-Fi? Skip BLE provisioning with
   `CONFIG_HOMEHUB_WIFI_SSID` / `CONFIG_HOMEHUB_WIFI_PASSWORD` in
   menuconfig. The device still needs one pairing for its token.

---

## Hardware verification checklist (first boot)

- [ ] Display lights, image not mirrored, red/blue not swapped
      (else flip `mirror()` flags / `LCD_RGB_ELEMENT_ORDER`)
- [ ] Touch hits the right spots (else adjust touch `mirror_x/mirror_y`)
- [ ] `i2c_master_probe` finds ES7210 at `0x40` (fallback `0x41`) and
      GT911 at `0x5D` (fallback `0x14`)
- [ ] Mic captures: speak, watch `muse_audio` read levels in the log
- [ ] Speaker plays Muse's reply at a sane level (fixed gain — hardware)
- [ ] Battery reads on GPIO4 (ADC1 ch3); charging state is not wired —
      `read_power()` should report `battery_mv` only if you calibrate it
- [ ] BLE shows `MuseGadget-XXXXXX`; pairing completes; green on connect

Per the repo's own porting rule: until this runs on your hardware, the port
is *built, not run* — say so in any PR or issue you file upstream.

## Sources

- Pin map, audio/display/touch examples, schematic:
  [Xinyuan-LilyGO/T-Deck](https://github.com/Xinyuan-LilyGO/T-Deck)
  (`examples/*/utilities.h`, `Microphone.ino`, `I2SPlay.ino`,
  `Touchpad.ino`, `schematic/schematic.pdf`) — MIT
- SDK build/porting contract: `esp32/AGENTS.md`, `esp32/devices/AGENTS.md`
  in [facebookincubator/muse-gadget-sdk](https://github.com/facebookincubator/muse-gadget-sdk) — Apache 2.0
- Reference board implementations in-repo: `board_aipi.c` (ST7789 +
  esp_lv_adapter, hand-driven), `board_espressif_box_3.c` (GT911 touch,
  ES7210 mic), `board_ostb_echoear_2st.c` (ES7210 via `es7210_codec_new`),
  `board_waveshare_s3_lcd7.c` (GT911 via `esp_lcd_touch_gt911`)
- ST7789 rotation→MADCTL mapping: `lib/TFT_eSPI/TFT_Drivers/ST7789_Rotation.h`
  in the T-Deck repo (`setRotation(1)` = `MX|MV`)
- Custom `audio_codec_if_t` for the MAX98357A: `esp_codec_dev` README,
  "Customize a new codec device" (`espressif/esp-audio-dev`)
