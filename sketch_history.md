# pico_usb_tft_enc — Sketch Change History

Tracks significant rewrites and what broke/worked after each.
Most recent at top.

---

## Current state (2026-03-09, session 3 end)
- Serial: BROKEN — nil output
- TFT: working but hideously slow (20-line software SPI redraw per update)
- Freeze: still occurring
- Changes made this session:
  - Removed `while (!Serial)` from setup() at some point — broke Serial
  - Multiple TFT scroll rewrites: hardware scroll → circular buffer full redraw → page-based → circular buffer again
  - Added scpi_rx_ack response-triggered queries (intended to fix freeze)
  - Moved all Serial writes to Core0 via ring buffer (intended to fix freeze)

---

## ft232_test.ino (reference — known working)
- Serial: WORKING (`while (!Serial)` present)
- FT232R SCPI comms: WORKING
- TFT: not present
- Freeze: not observed (no TFT, simpler loop)

---

## Session 2 end state (2026-03-08)
- Serial: WORKING
- FT232R SCPI comms: WORKING end-to-end
- TFT: working (no scroll issues noted)
- Freeze: not yet observed
- Known good: `while (!Serial)` was present

---

## TODO
- Before any future rewrite, snapshot the working state here first
- Init git repo so we have actual history

