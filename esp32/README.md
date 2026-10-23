# PorcelainDemo (ESP32) + website

The ESP32 drives the fingertip TEC heater (MAX6675 + PID, 37 °C target, 40 °C hard stop) and the vibration motor.
The screen is the website https://before-it-was-blue.vercel.app, shown full-screen inside a small page the ESP32 serves.

```
iPad/phone  ──opens──►  http://<esp32-ip>   (ESP32 page)
                          └─ iframe: https://before-it-was-blue.vercel.app
website ──postMessage──► ESP32 page ──/start /stop /kiln?t= /reset──► ESP32
```

Why not open the vercel.app link directly? Browsers block an https page from calling an http device,
so the ESP32 page has to be the one that talks to the ESP32.

## Setup
1. In `PorcelainDemo.ino` set `WIFI_SSID` / `WIFI_PASS` to a Wi-Fi **with internet**, **2.4 GHz**
   (router or phone hotspot; on iPhone turn on *Maximize Compatibility*). University Wi-Fi with a login page
   or "eduroam" usually won't work.
2. Upload, open Serial Monitor at 115200. It prints `Open: http://192.168.x.x  or  http://porcelain.local`.
3. Join the same Wi-Fi on the iPad/phone and open that `http://` address in Safari.
   Optional: Share → *Add to Home Screen* for a full-screen kiosk.

## Behaviour
- Press and hold the fragment → `/start` (heater on, PID to 37 °C). Release → `/stop`.
- While holding, the site sends its simulated kiln temperature ~5×/s (`/kiln?t=`). The ESP32 vibrates once at 1200 °C and twice at 1300 °C.
- Reset / Start again → `/reset` (also clears a latched fault once the sensor reads OK).
- Safety: heater goes off if the page goes quiet for 1.5 s, the screen is locked, or the sensor faults.
- A small red badge at the bottom appears only for a safety stop, a missing sensor, or when the ESP32 isn't reachable.
- If the Wi-Fi can't be joined, the ESP32 starts its own network `PorcelainDemo` (password `12345678`) as a fallback; the website needs internet, so that network only shows a "no internet" message.
- Serial Monitor: `H` start / `S` stop still work for bench tests.
