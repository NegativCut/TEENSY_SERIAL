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
| `FIRMWARE/la_test/la_test.ino` | Working | Logic analyser: 1–8ch capture, TFT waveform, binary file to USB |

---

## 2026-03-14 — Session 10 (DMA capture sketch)

### la_dma.ino — new sketch in FIRMWARE/la_dma/
- Replaces per-edge ISRs with QTimer1-triggered DMA → GPIO6_DR → circular buffer
- **DMA trigger:** QTimer1 ch0 (DMAMUX_SOURCE_QTIMER1_READ0=48) — PIT has no DMAMUX source on IMXRT1062
- **DMA enable:** TMR_DMA_CMPLD1DE (compare-load 1 event triggers transfer)
- **GPIO bit positions confirmed:** pins 14,15,16,17,20,21,22,23 = GPIO1/GPIO6 bits 18,19,23,22,26,27,24,25
- **Clock:** QTimer driven by IPG (F_BUS/2 ≈ 66 MHz); TMR_COMP = 66M/SAMPLE_RATE - 1
- **Default SAMPLE_RATE:** 5 MHz (200 ns resolution) → DMA_HALF fills every 409 µs
- **DMA buffer:** 2×2048 uint32_t in OCRAM (DMAMEM), power-of-2 aligned for DMOD circular addressing
- **Main loop:** processes ready DMA halves via `process_half()` — extract 8-ch state byte, detect changes, generate events
- **Event format:** same v2 binary as la_test — compatible with Logic_Analyser_Viewer without changes
- **TFT/USB/encoder:** same as la_test (armed=counter 1Hz, disarmed=waveform 10Hz)
- **Timestamps:** DWT-based, interpolated from DMA half-end timestamp: `t = t_half_end - (DMA_HALF-1-i) * CYCLES_PER_SAMPLE`
- **Known unknowns:** IPG clock assumed 66 MHz — verify with DEBUG serial output; adjust TMR_IPG_HZ if measured rate differs
- **Arduino IDE broke** (arduino-cli setup from session 5 wiped boardsmanager.additional.urls); restored manually — see .claude/memory/reference_arduino_ide.md
- **pymupdf installed** for PDF parsing (ST7735S.pdf datasheet); see .claude/memory/reference_pdf_parser.md

### DMA capture — reverted (both attempts failed)
- **Attempt 1 — wrong DMAMUX source:** `DMAMUX_SOURCE_QTIMER1_READ0` (48) is input-capture mode; compare trigger needs `DMAMUX_SOURCE_QTIMER1_WRITE0_CMPLD1` (52). Fixed source but DMA still silent — no TFT counter increment, no USB data.
- **Attempt 2 — IntervalTimer ISR polling:** Every 200ns ISR reads GPIO6_DR, diffs against previous — worse than la_test. USB stack starved at 5MHz ISR rate.
- **Root cause (not identified until after both attempts):** Single-core IMXRT1062 with USBHost_t36 cannot run high-frequency GPIO polling AND USB bulk writes simultaneously. No XBAR routing exists to make QTimer compare directly DMA-read GPIO without ISR involvement. DMAMUX READ sources are for capturing timer input, not triggering GPIO reads.
- **Decision:** `la_dma/` deleted, reverted to `la_test`.

### la_test.ino — cleanup and logic fixes
- **Removed all waveform display infrastructure:** `disp_buf[256]`, `disp_write`, `disp_shown`, `DISP_SIZE`, `WAVE_X`, `WAVE_W`, `draw_waveform()` — all gone. No ISR writes to disp_buf.
- **TFT behaviour now:** disarmed = settings screen only (channel count, pin list, next file); armed = event counter at 1 Hz only.
- **Renamed:** `draw_idle()` → `draw_settings()`; status text "FRZ"→"DONE", "DISARMD"→"READY"
- **Drive unmount while armed bug fixed:** previously called `log_file = File()` without closing first — silently abandoned unflushed data. Now calls `log_file.close()` before nulling.
- **la_dma/ deleted** from working directory (was never committed).

## 2026-03-13 — Session 9 (LA firmware performance review)

### Performance fixes applied to `la_test.ino`
- **TFT blocks USB flush (main bottleneck):** `draw_waveform()` takes 15–30ms (full fillRect + redraw over SPI). During that time `log_flush_ready()` cannot run → overflows at high event rates.
  - Fix: call `log_flush_ready()` immediately BEFORE `draw_waveform()` (drain pending half first), then again after (catch any half filled during draw). TFT always updates — no skipping.
  - Previous attempt (skip TFT frame if pp_ready set) caused TFT freeze at high event rates — reverted.
- **Redundant `scan_log_files()` on arm:** `log_open()` was calling `scan_log_files()` (99× `myFS.exists()`) despite `log_file_num` already being set on drive mount. Removed — now just increments cached value.
- **BUF_HALF increase reverted:** Doubling to 8192 doubled USB write size (24KB → 49KB). At the test signal frequency, 49KB write took longer than the other half took to fill → continuous overflow → only 29ms of data captured. Kept at 4096.
- **Session notes reliability:** `CLAUDE.md` added to both project roots — session notes now written proactively without prompting.
- **Binary analysis of la02.bin:** 3.31s capture, 5ch, 37,837 ev/s. Confirmed 483ms FAT32 cluster-boundary write stall at t=1.976s. The ~64ms gaps repeating every 100ms are natural signal idle periods (device under test, not firmware).
- **preAllocate / seek-extend / truncate all abandoned** — `File` from USBHost_t36 doesn't support `preAllocate`; seek-extend caused 0-byte files; truncate caused 0-byte files. Reverted all.
- **Root cause of stalls identified via binary analysis:** flash drive erase block boundary write latency spikes (~330–360ms). Not FAT cluster allocation.
- **Fix: DMAMEM ping-pong buffer + larger BUF_HALF** — `pp_buf` moved to OCRAM with `DMAMEM` (384KB, leaves DTCM free). `BUF_HALF` increased 4096→32768 (fill time 228ms @ 143k ev/s). Covers all stalls ≤228ms. `pp_pos` widened to `uint32_t` throughout.
- **la04.bin:** 39k ev/s, 330ms stalls visible as gaps. **la05.bin:** 143k ev/s, buffer covers 190–226ms stalls; 358ms stall still causes overflow at this rate.
- **la06.bin:** 225k ev/s, old drive — 135 gaps >10ms, worst stall 554ms. Drive could not keep up; max sustainable rate ~59k ev/s on that drive.
- **la01.bin (new fast drive):** 342k ev/s, zero drive stalls. All gaps are 64ms signal idle periods. Firmware confirmed gap-free at 342k ev/s. Old drive was root cause of all data gaps.
- **Final config:** `BUF_HALF=32768` in OCRAM (`DMAMEM`), `pp_pos` widened to `uint32_t`. Fast USB drive required for high event rates.

## 2026-03-12 — Session 8 (Logic Analyser)

### la_test.ino — developed from scratch
- 8-channel (configurable 1–8) interrupt edge capture, ping-pong buffer → USB flash drive `.bin`
- ISR toggle pattern (`ch_state ^= 1`) — prevents impossible consecutive same-direction events; no digitalRead race
- Ping-pong buffer: 2×4096 events; main loop flushes ready halves; separate 256-event circular disp_buf for TFT
- Binary file format v1 → v2: added `uint32_t cpu_hz` to header; timestamps are `ARM_DWT_CYCCNT` (1.67ns @ 600MHz)
- DWT cycle counter enabled in setup: `ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA`
- Encoder rotation (disarmed) selects 1–8 channels; saved to EEPROM byte 1; ISRs attached for active channels only
- Lane height on TFT scales dynamically: `(SCR_H - STATUS_H) / num_channels`
- USB file numbering: `scan_log_files()` scans drive for highest `la*.bin`, starts from `highest + 1`; blank drive → `la01.bin`
- EEPROM byte 0 = file counter fallback (no drive); byte 1 = num_channels

### Logic_Analyser_Viewer — C# WinForms .NET 8
- Repo: https://github.com/NegativCut/Logic_Analyser_Viewer
- 8-channel waveform viewer, 1920×1080, GDI+ rendering, zoom/pan/scrollbar
- Binary loader: handles v1 (µs timestamps) and v2 (cycle timestamps ÷ cpu_hz/1e6)
- CSV loader: extracts raw events from step-function row transitions
- Glitch filter: per-channel single-pass pair removal; `NumericUpDown` 0–10000 µs in status bar; live, preserves view
- Status bar: filename, format, event count, span, event rate, active channels, view span, file size
- Drag-and-drop file loading; auto-detect .bin/.csv by extension

### Hardware / interfacing
- Capture pins 14,15,16,17,20,21,22,23 — pins 18/19 skipped (I2C SDA/SCL with 2.2kΩ pull-ups)
- TXS0108EPWR level shifter: works for passive sniffing (Teensy inputs = high-Z, auto-direction → B side drives A)
  - Not recommended for I2C by TI but works passively; not ideal for active I2C participation
  - VCCA=3.3V, VCCB=5V, OE=3.3V; 100nF decoupling on each VCC
- Encoder pins: ENC_A=3, ENC_B=4, ENC_SW=2 (same across la_test and main sketch)

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

## 2026-03-11 — Session 7
- USB flash drive support integrated into main sketch
- USBDrive + USBFilesystem added alongside USBSerial — auto-detected on connect
- Storage mode: LOG:OFF/ON status bar, button toggles logging
- Log files: log01.csv–log99.csv, EEPROM-backed counter, new file per on/off cycle
- log_open / log_close / log_write helpers — ready for LA mode data source later
- usbdrive_test sandbox sketch confirmed working (mount, write, read-back PASS)
- Stability test: >232,000 readings with no freeze — Teensy native USB host confirmed stable

## 2026-03-10 — Session 6
- Main sketch confirmed working end-to-end on Teensy 4.0 hardware
- USB host, FT232R, RS232, SCPI, TFT scroll, encoder profile select all working
- Fixes applied: menu title truncated to 21 chars; encoder detent logic corrected (divide by 4 before compare)
- USB connection status indicator: 2px green/red bar on right edge of TFT
- GDM-8251A live readings confirmed on display

## 2026-03-09 — Session 5 (Teensy port)
- Project moved to `G:\09 DROPBOX\CODE\GIT_REPOS\TEENSY_USB_TFT_ENC_SERIAL`
- New sketch: Teensy 4.0, USBHost_t36 (native USB), single core, profile system with EEPROM persistence
- arduino-cli installed to `C:\Users\Dell\AppData\Local\arduino-cli\`; Teensy core 1.60.0
- `setup_t` name collision fixed: renamed to `tft_setup_t` in TFT_eSPI.h and TFT_eSPI.cpp
- TFT_eSPI User_Setup.h updated for Teensy 4.0 pins; Pico backup saved as `User_Setup_pico_usb_tft_enc.h`
- First build: clean compile — FLASH 67KB/2MB, RAM1 81KB used
- TFT test confirmed working on Teensy 4.0 hardware
- Top row pixel fix: `rowstart=1` added to BLACKTAB case 0 in `ST7735_Rotation.h` (TFT_ROWSTART in User_Setup.h is ineffective — not used by library)

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
