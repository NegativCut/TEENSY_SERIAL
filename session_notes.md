# TEENSY_USB_TFT_ENC_SERIAL — Session Notes
*(Ported from pico_usb_tft_enc — Pico history retained below)*

## Project Overview
Teensy 4.0 with ST7735S TFT display, rotary encoder, USB-A host (native, soldered to bottom D+/D- pads) → FT232R → RS232 → GW Instek GDM-8251A multimeter. Multi-profile instrument display with EEPROM profile persistence.

---

## Hardware

| Peripheral | Pin  | GPIO |
|------------|------|------|
| TFT MOSI (SDA) | 5  | GP3 |
| TFT SCK (SCL)  | 4  | GP2 |
| TFT RST        | 6  | GP4 |
| TFT DC         | 7  | GP5 |
| TFT CS         | 9  | GP6 |
| TFT BLK        | 10 | GP7 |
| Encoder SW     | 11 | GP8 |
| Encoder A      | 12 | GP9 |
| Encoder B      | 14 | GP10 |
| USB-A D+       | 32 | GP27 ── 22Ω ── USB-A pin 3 |
| USB-A D−       | 34 | GP28 ── 22Ω ── USB-A pin 2 |
| USB VBUS sense | 31 | GP24 (HIGH = PC on micro-USB) |

---

## Confirmed Working

### TFT Display
- ST7735S 128x160
- **Adafruit_ST7735 hardware SPI constructor (`&SPI`) produces white screen**
- **TFT_eSPI with hardware SPI0 at 27MHz works** — `TFT_SPI_PORT 0`, pins GP2/GP3
- Init tab: `ST7735_BLACKTAB` in User_Setup.h
- TFT labels: SDA=MOSI, SCL=SCK
- User_Setup.h location: `G:\09 DROPBOX\CODE\libraries\TFT_eSPI\User_Setup.h`
- Backup of previous project setup: `User_Setup_pico_matrix.h` in same folder

### Rotary Encoder
- PIO quadrature decoder using `quadrature.pio` / `quadrature.pio.h`
- Runs on **pio0** (pio1 used by USB host)
- Switch on GP8 with `INPUT_PULLUP`
- Read position: `pio_sm_exec_wait_blocking` + `pio_sm_get_blocking`
- TFT flicker fix: use `tft.setTextColor(fg, bg)` two-argument form — no `fillRect` needed

### USB Serial (CDC)
- **USB Stack: Adafruit TinyUSB** (required for USB host)
- `#include "hardware/clocks.h"` + `set_sys_clock_khz(120000, true)` must be **first line of setup()**
- `while (!Serial)` is safe with TinyUSB stack

### USB Host — Flash Drive (MSC)
- Include order critical: `SdFat_Adafruit_Fork.h` → `pio_usb.h` → `Adafruit_TinyUSB.h`
- USB host on **pio1**: `pio_cfg.pio_tx_num=1`, `pio_cfg.pio_rx_num=1`
- Objects: `Adafruit_USBH_Host`, `Adafruit_USBH_MSC_BlockDevice`, `FatVolume`
- File API: `FatFile` / `File32` with `f.open(&fatfs, filename, flags)`
- `fatfs.begin(&msc_block_dev)` called from **core0** after mount flag — NOT in `tuh_msc_mount_cb`
- Flash drive must be **FAT32** — exFAT will not mount
- Reference: `FURNACE_CTRL_PICO\FIRMWARE\USB_HOST_TEST\USB_HOST_TEST.ino`

### USB Host — FT232R RS232 ✓ WORKING
- Target device: GW Instek GDM-8251A multimeter (SCPI over RS232, 9600 8N1)
- FT232R VID=0x0403 PID=0x6001, EP_OUT=0x02, EP_IN=0x81 (discovered dynamically from descriptor)
- SCPI commands terminated with `\r\n`; responses terminated with `\n\r`
- Confirmed: `*idn?` → `GW,GDM8251A,2.30`; `val1?` → live measurement value
- Baud divisor for 9600: `wValue=0x4138` (3MHz / 312.5), `wIndex=0` for SET_BAUD_RATE
- RESET / SET_DATA / SET_FLOW_CTRL use `wIndex=1` (INTERFACE_A)
- Assert DTR+RTS: `FTDI_SIO_SET_MODEM_CTRL` (req=0x01), `wValue=0x0303`, `wIndex=1`
- **Required init sequence on mount:** `syst:rem` → `*rst` → `delay(6000)` → `*cls` → query
  - `*rst` resets meter; 6 second wait required before meter responds to queries
- **pio_usb cannot handle concurrent IN and OUT bulk transfers** — freezes core1
- Solution: serialize TX/RX via pending flags; all transfers from `loop1()` only; callbacks only clear flags
- Use `usbh_edpt_claim` + `usbh_edpt_xfer` + `usbh_edpt_release` — NOT `tuh_edpt_xfer`
- Must call `usbh_edpt_release()` explicitly in `xfer_cb` before clearing pending flag
- Custom class driver via `usbh_app_driver_get_cb` required for bulk endpoint access
- `[rx_cb] len=2` = FTDI status-only packet (no UART data) — suppress in log; use as heartbeat later
- Reference PC software: `G:\09 dropbox\code\GIT_REPOS\GW-8251-CONTROL\`

---

## Architecture

```
Core 0: Serial I/O, TFT updates (via volatile flags from core1)
Core 1: USBHost.task() + serialized TX/RX scheduling
        USB callbacks: set flags only, never submit transfers
```

---

## Key Rules / Lessons Learned

1. `set_sys_clock_khz(120000, true)` must be first line of setup()
2. Include order: `SdFat_Adafruit_Fork.h` → `pio_usb.h` → `Adafruit_TinyUSB.h`
3. Software SPI only for this TFT module
4. Encoder on pio0, USB host on pio1
5. All TinyUSB transfers submitted from `loop1()` only — never from callbacks, never from core0
6. pio_usb: no concurrent IN/OUT bulk transfers — serialize with pending flags
7. Before making major code changes — propose plan and confirm first

---

## Firmware Files

| File | Status | Purpose |
|------|--------|---------|
| `FIRMWARE/pico_usb_tft_enc/pico_usb_tft_enc.ino` | Working | Main sketch: TFT + encoder + USB host MSC |
| `FIRMWARE/comport_test/comport_test.ino` | Working | Minimal CDC serial test |
| `FIRMWARE/ft232_test/ft232_test.ino` | Working | FT232R USB host → RS232 SCPI comms (GDM-8251A) |

---

## 2026-03-07 — Session 1
- Established project from empty directory
- Resolved TFT (white screen → software SPI), serial (TinyUSB stack), SPI freeze issues
- Encoder working with PIO quadrature decoder
- USB host MSC (flash drive) working: list, read, write
- FT232R RS232 host: enumerates and configures correctly; TX/RX serialization approach being tested
- GDM-8251A identified as target RS232 device; SCPI protocol documented

## 2026-03-09 — Session 4
- Single-core question answered: dual-core requirement comes from pio_usb, not app logic; Teensy 4.0 native USB host would allow single-core operation
- IN endpoint poll rate throttled to every 5ms (was unconditional every loop iteration) — reduces bus pressure
- `USBHost.task(10)` — added 10ms timeout parameter
- Both changes aimed at reducing pio_usb instability under sustained transfers

## 2026-03-09 — Session 5 (Teensy port)
- Project moved to `G:\09 DROPBOX\CODE\GIT_REPOS\TEENSY_USB_TFT_ENC_SERIAL`
- New sketch: Teensy 4.0, USBHost_t36 (native USB), single core, profile system with EEPROM persistence
- arduino-cli installed to `C:\Users\Dell\AppData\Local\arduino-cli\`; Teensy core 1.60.0
- `setup_t` name collision fixed: renamed to `tft_setup_t` in TFT_eSPI.h and TFT_eSPI.cpp
- TFT_eSPI User_Setup.h updated for Teensy 4.0 pins; Pico backup saved as `User_Setup_pico_usb_tft_enc.h`
- First build: clean compile — FLASH 67KB/2MB, RAM1 81KB used — standing by for hardware test

## 2026-03-09 — Session 3
- TFT_eSPI with hardware SPI0 at 27MHz confirmed working — replaces Adafruit slow software SPI
- Core1 freeze root cause: pio_usb bug under sustained transfers — not fixable in application code
- Multicore recovery attempted (multicore_reset_core1 + PIO1 clear) — failed, TinyUSB global state cannot reinitialise without full reset
- Workaround: hardware watchdog (2s timeout, petted by Core1 only) — silent auto-reset on freeze
- Serial writes guarded with `if (Serial)` — device works standalone without PC connection
- Decision: port to Teensy 4.0 — native USB host hardware eliminates pio_usb instability
- User_Setup.h for TFT_eSPI backed up as User_Setup_pico_matrix.h

## 2026-03-08 — Session 2
- FT232R RS232 → GDM-8251A SCPI: fully working end-to-end
- Key fixes: custom TinyUSB class driver; usbh_edpt_claim/xfer/release; usbh_edpt_release in xfer_cb
- Correct FTDI wIndex values; DTR+RTS assertion; dynamic endpoint discovery from descriptor
- GDM-8251A required init: syst:rem → *rst → 6s delay → *cls before responding to queries
- DB9 loopback test confirmed MAX232 TX+RX paths working
- Next: integrate FT232R SCPI into main pico_usb_tft_enc.ino sketch
