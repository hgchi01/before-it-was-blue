# ESP32 wearable

There are two sketches. **Use `BeforeItWasBlue` for the exhibition.**

| Sketch | Network | Page |
|---|---|---|
| `BeforeItWasBlue/` (main) | ESP32 makes its own Wi-Fi, no internet needed | served from the ESP32 (`data/`) |
| `PorcelainDemo/` (older) | ESP32 joins a Wi-Fi with internet | shows before-it-was-blue.vercel.app |

## BeforeItWasBlue (offline)

Hardware: dual TEC (GPIO 26 + 33), MAX6675 (SCK 18, CS 27, SO 19), vibration motor (GPIO 25).
Skin target 37.5 °C, hard cutoff 41 °C.

### Upload
1. Arduino IDE 2.x, board "ESP32 Dev Module" (Espressif core 3.x), plugin
   [arduino-littlefs-upload](https://github.com/earlephilhower/arduino-littlefs-upload).
2. Open `BeforeItWasBlue/BeforeItWasBlue.ino` and upload the sketch.
3. Close Serial Monitor, then Ctrl/Cmd + Shift + P → **Upload LittleFS to Pico/ESP8266/ESP32**
   (uploads the `data/` folder: page, images, fonts, ~370 KB).
4. On the iPad/phone join Wi-Fi **PorcelainDemo** (password `12345678`) and open **http://192.168.4.1**.

### Changing the page
Don't edit `data/index.html` by hand. Edit the main `index.html` at the top of the repo (the
Vercel site), then run:

```
python3 tools/build_esp32.py
```

This rebuilds `data/` with the exhibition settings (list in `CHANGES` inside the script):
45 s firing, "Press and hold the porcelain to fire", "Too hot 🥵", local fonts, and direct
calls to the ESP32. Then repeat step 3.

### What the page sends
| When | Request |
|---|---|
| finger down / up | `/start?t=<°C>` / `/stop?t=<°C>` |
| every 0.25 s while pressed | `/kiln?t=<°C>` |
| first time ≥ 1200 °C / ≥ 1300 °C | `/vib1` (single buzz) / `/vib2` (double buzz) |
| Reset / Start again | `/reset` (also clears a safety stop once the sensor is below target) |
| every 0.5 s | `/status` (shows a red badge on a safety stop, sensor problem or lost connection) |
| page hidden / screen locked / tab closed | `/off` |

### Safety
- TEC off at 41 °C, on sensor errors, and after 1300 °C until Reset.
- TEC off if the page goes quiet for 3 s while heating or holding a temperature
  (`PAGE_TIMEOUT_MS`), and immediately when the page is hidden.
- Measure the real skin temperature before anyone wears it.
