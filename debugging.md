# TEENSY_USB_TFT_ENC_SERIAL — Debugging Log

## Open Issues

| # | Issue | Status |
|---|-------|--------|
| 1 | ~~First flash — untested~~ | **Resolved** — all hardware confirmed working |

---

Tracks what has been tried and the outcome. Do not re-try failed approaches.

---

## Resolved on Pico (carried over for reference)

| Issue | Resolution |
|-------|-----------|
| pio_usb Core1 freeze after sustained USB transfers | **Platform change** — Teensy 4.0 native USB host eliminates pio_usb entirely |
| TFT software SPI too slow | **Resolved** — TFT_eSPI hardware SPI; carried over to Teensy |
| Serial nil output | **Resolved** — `if (Serial)` guards throughout |

---

## TFT Display

### Top row of pixels cut off
- BLACKTAB at rotation 0 sets no rowstart (stays 0) — one pixel row clipped at top
- `TFT_ROWSTART` in User_Setup.h has no effect — not used anywhere in the library
- **Fix:** Added `rowstart = 1` to BLACKTAB case 0 in `TFT_Drivers/ST7735_Rotation.h`
- Confirmed working

---

## Build — Teensy 4.0

### setup_t name collision (TFT_eSPI vs USBHost_t36)
- Both libraries define `typedef setup_t` with different struct layouts
- **Fix:** Renamed `setup_t` → `tft_setup_t` in `TFT_eSPI.h` and `TFT_eSPI.cpp`
- Only affects `getSetup()` which is not used in this sketch

### Compiler warnings (non-fatal)
- `%-16s` / `%-17s` format truncation in snprintf — intentional, TFT line width is 22 chars
- `TOUCH_CS` not defined — touch unused, harmless

### Memory (first build, Teensy 4.0)
- FLASH: 67KB / 2MB (3.3%)
- RAM1: 81KB used, 411KB free
- RAM2: 12KB used, 512KB free

---

## Notes
- TFT_eSPI `User_Setup.h` — Teensy 4.0 pins: MOSI=11, SCK=13, CS=10, DC=9, RST=8
- Pico setup backed up as `User_Setup_pico_usb_tft_enc.h`
- USBHost_t36 bundled with Teensy core — no custom class driver needed
- Single-core operation — no dual-core constraints
