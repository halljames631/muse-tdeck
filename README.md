# Muse Gadget on the LILYGO T-Deck

A board port that runs the [Muse Gadget SDK](https://github.com/facebookincubator/muse-gadget-sdk)
(`esp32/` firmware: Home Link + the Muse avatar/voice/settings UI) on the
**LILYGO T-Deck** (ESP32-S3FN16R8 — 16 MB flash, 8 MB octal PSRAM, 2.8"
320×240 ST7789, GT911 touch, ES7210 dual mic, MAX98357A speaker).

This repo holds **only the port**: the new board file, the sdkconfig
overlay, the small edits to shared SDK files, and the deployment guide.
`apply.sh` clones the upstream SDK at the pinned commit and drops these
files in, giving you a complete, buildable tree.

## Hardware

| Part | Connection |
|---|---|
| Display | ST7789, SPI: SCK 40, MOSI 41, CS 12, DC 11, backlight 42 |
| Touch | GT911, I2C SDA 18 / SCL 8, INT 16, addr 0x5D |
| Microphone | ES7210 ADC, I2S1 (MCLK 48, WS 21, BCK 47, DIN 14), I2C 0x40 |
| Speaker | MAX98357A amp, I2S0 (BCK 7, WS 5, DOUT 6), fixed gain |
| Button | BOOT on GPIO0 (talk); on-screen mic also works |
| Power | GPIO10 must be driven HIGH (done in firmware) |
| USB | Native USB Serial/JTAG (`303a:1001`) — no driver needed |

Pin source: [Xinyuan-LilyGO/T-Deck](https://github.com/Xinyuan-LilyGO/T-Deck)
(`utilities.h`, examples, schematic).

## Build

Requirements: ESP-IDF **v6.0.1** (exact — other versions are unsupported),
Python 3, a C compiler.

```sh
./apply.sh            # clones upstream SDK @ fa07304, copies the port in
cd muse-gadget-sdk/esp32
. ~/esp/esp-idf-v6.0.1/export.sh
tools/muse/board.sh build tdeck
```

Your SDK token (`mgst_…`, from gadgets.muse.ai → Account → SDK tokens) is
**not** baked into the build. After flashing, send it over the serial
console (see "Flash" below):

```
>sdk_token=mgst_…
```

The firmware stores it in NVS and restarts with it. It survives reflashes.

## Flash

```sh
cd muse-gadget-sdk/esp32
tools/muse/board.sh flash tdeck /dev/ttyACM0   # Linux; /dev/cu.usbmodem* on macOS
```

Or with esptool directly, using the prebuilt image from the
[releases](../../releases) (if published):

```sh
esptool.py -p /dev/ttyACM0 -b 460800 --chip esp32s3 \
  write_flash 0x0 tdeck-muse-gadget.bin
```

Then open a monitor, set the token (above), and pair from the Muse app:
the gadget advertises as `MuseGadget-XXXXXX`. Short-press BOOT (or tap the
on-screen mic) to confirm pairing.

## Files

| File | What it is |
|---|---|
| `esp32/components/muse/boards/board_lilygo_t_deck.c` | The board: power, ST7789+LVGL, GT911 touch, dual-I2S audio, buttons, battery |
| `esp32/devices/sdkconfig.muse-lilygo-t-deck` | Build overlay: esp32s3, 16 MB flash, 8 MB octal PSRAM, native USB |
| `esp32/components/muse/{Kconfig,CMakeLists.txt,idf_component.yml}` | Board registration + GT911 touch component |
| `esp32/tools/muse/{board.sh,ports.py,avatar.py}` | `tdeck` build/flash alias, USB + avatar tooling |
| `esp32/{AGENTS.md,devices/README.md}` | Board tables |
| `tdeck-muse-gadget-guide.md` | Full deployment guide: audio/display mapping, config reference |

Upstream SDK: Apache 2.0. This port: same. Pin map: LilyGo (MIT).
