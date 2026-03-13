---
name: Arduino IDE preferences — board URLs and config
description: Location and content of Arduino IDE 1.8 preferences, including board manager URLs restored after arduino-cli broke them
type: reference
---

## Preferences file
`C:\Users\Dell\AppData\Local\Arduino15\preferences.txt`

**Important:** Arduino IDE overwrites this file on exit. Only edit it while the IDE is closed.

## Board manager URLs (restored 2026-03-14)
```
https://www.pjrc.com/teensy/package_teensy_index.json
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
https://arduino.esp8266.com/stable/package_esp8266com_index.json
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
https://mcudude.github.io/MiniCore/package_MCUdude_MiniCore_index.json
http://digistump.com/package_digistump_index.json
https://lowpowerlab.com/arduino/package_LowPowerLab_index.json
http://drazzy.com/package_drazzy.com_index.json
```
(jolly excluded — user does not want it)

## Key settings confirmed working
- `board=teensy40`
- `sketchbook.path=G:\09 DROPBOX\CODE`
- `custom_speed=teensy40_600` (600 MHz)
- `custom_usb=teensy40_serial`
- `serial.port.label=COM23 Serial`

## What broke it
arduino-cli was installed in session 5 (`C:\Users\Dell\AppData\Local\arduino-cli\`).
Its setup cleared/reset `boardsmanager.additional.urls` in preferences.txt and changed
`sketchbook.path` from `C:\Users\Dell\Documents\Arduino` to `G:\09 DROPBOX\CODE`.

## arduino-cli config (separate)
`C:\Users\Dell\AppData\Local\Arduino15\arduino-cli.yaml`
- Only has Teensy URL; user: `G:\09 DROPBOX\CODE`
