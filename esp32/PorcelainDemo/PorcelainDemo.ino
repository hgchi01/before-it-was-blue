#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// ============================================================
// Porcelain Interactive Demo
// ESP32 + MAX6675 + MOSFET TEC + Vibration
//
// The screen is now the website https://before-it-was-blue.vercel.app
// The ESP32 serves a small page that shows that website full-screen
// and passes its press / release / temperature messages to this sketch.
//
// HOW TO USE
//  1. Put your Wi-Fi name and password below (the same Wi-Fi the
//     iPad / phone will use; it needs internet, 2.4 GHz).
//  2. Upload. Serial Monitor (115200) prints the address, e.g.
//     "Open: http://192.168.1.42  or  http://porcelain.local"
//  3. On the iPad / phone (same Wi-Fi) open that http:// address.
//     Do NOT open the vercel.app link directly - that one can't
//     reach the ESP32 (browsers block https -> http device calls).
// ============================================================

// ---------------- Pins ----------------
const int PIN_TEC_GATE = 26;
const int PIN_TC_SCK   = 18;
const int PIN_TC_CS    = 27;
const int PIN_TC_SO    = 19;
const int PIN_VIB      = 25;

// ---------------- WiFi ----------------
// Shared Wi-Fi (router or phone hotspot) - must have internet.
const char* WIFI_SSID = "your-wifi-name";
const char* WIFI_PASS = "your-wifi-password";
const char* MDNS_NAME = "porcelain";              // -> http://porcelain.local

// Fallback: if the shared Wi-Fi can't be joined, the ESP32 makes its own
// network so you can still see the status page (the website needs internet,
// so it will show a "no internet" message on this network).
const char* AP_SSID = "PorcelainDemo";
const char* AP_PASS = "12345678";

const char* SITE_URL = "https://before-it-was-blue.vercel.app/";

WebServer server(80);

// ---------------- Real TEC control ----------------
const float TARGET_C = 37.0f;
const float HARD_STOP_C = 40.0f;

const float MIN_PLAUSIBLE_C = 15.0f;
const float MAX_PLAUSIBLE_C = 45.0f;
const float MAX_SAMPLE_JUMP_C = 3.0f;

const float KP = 24.0f;
const float KI = 1.0f;
const float KD = 2.5f;

const unsigned long SAMPLE_MS = 500;
const unsigned long WINDOW_MS = 2000;
const unsigned long SENSOR_STALE_MS = 1100;

const float MAX_DUTY = 80.0f;

// If the page stops talking while the heater is on (screen locked, Wi-Fi drop),
// the interaction is paused and the TEC turns off.
const unsigned long PAGE_TIMEOUT_MS = 1500;

// ---------------- Kiln temperature (comes from the website) ----------------
float kilnTemp = 25.0f;

const float KILN_MIN = 25.0f;
const float KILN_MAX = 1400.0f;   // website goes up to ~1388 C when overfired

bool interactionActive = false;

bool vib1200Done = false;
bool vib1300Done = false;

unsigned long lastPageMsgAt = 0;
bool serialTest = false;   // started with 'H' in Serial Monitor -> no page timeout

// ---------------- PID states ----------------
enum RunMode {
  IDLE,
  RUNNING,
  FAULT
};

RunMode mode = IDLE;

float tempC = NAN;
float prevTempC = NAN;

float derivativeFiltered = 0.0f;
float integral = 0.0f;
float dutyPercent = 0.0f;

int consecutiveGoodReads = 0;

unsigned long lastSampleAt = 0;
unsigned long lastGoodAt = 0;
unsigned long windowStartedAt = 0;

const char* lastFault = "";

// ============================================================
// Vibration
// ============================================================

enum VibState {
  VIB_IDLE,
  VIB_SINGLE_ON,
  VIB_DOUBLE_ON1,
  VIB_DOUBLE_GAP,
  VIB_DOUBLE_ON2
};

VibState vibState = VIB_IDLE;
unsigned long vibStateStarted = 0;

void vibrationOff() {
  digitalWrite(PIN_VIB, LOW);
}

void startSingleVibration() {
  vibState = VIB_SINGLE_ON;
  vibStateStarted = millis();
  digitalWrite(PIN_VIB, HIGH);
}

void startDoubleVibration() {
  vibState = VIB_DOUBLE_ON1;
  vibStateStarted = millis();
  digitalWrite(PIN_VIB, HIGH);
}

void updateVibration() {
  unsigned long now = millis();

  switch (vibState) {
    case VIB_SINGLE_ON:
      if (now - vibStateStarted >= 250) {
        vibrationOff();
        vibState = VIB_IDLE;
      }
      break;

    case VIB_DOUBLE_ON1:
      if (now - vibStateStarted >= 180) {
        vibrationOff();
        vibState = VIB_DOUBLE_GAP;
        vibStateStarted = now;
      }
      break;

    case VIB_DOUBLE_GAP:
      if (now - vibStateStarted >= 180) {
        digitalWrite(PIN_VIB, HIGH);
        vibState = VIB_DOUBLE_ON2;
        vibStateStarted = now;
      }
      break;

    case VIB_DOUBLE_ON2:
      if (now - vibStateStarted >= 180) {
        vibrationOff();
        vibState = VIB_IDLE;
      }
      break;

    default:
      break;
  }
}

// ============================================================
// TEC output
// ============================================================

void heaterOff() {
  digitalWrite(PIN_TEC_GATE, LOW);
}

void enterFault(const char* reason) {
  heaterOff();

  dutyPercent = 0.0f;
  integral = 0.0f;
  derivativeFiltered = 0.0f;

  interactionActive = false;
  mode = FAULT;
  lastFault = reason;

  Serial.print("FAULT (OUTPUT OFF): ");
  Serial.println(reason);
}

// ============================================================
// MAX6675
// ============================================================

uint16_t readMAX6675Raw() {
  uint16_t raw = 0;

  digitalWrite(PIN_TC_SCK, LOW);
  digitalWrite(PIN_TC_CS, LOW);

  delayMicroseconds(10);

  for (int i = 15; i >= 0; --i) {
    digitalWrite(PIN_TC_SCK, LOW);
    delayMicroseconds(10);

    if (digitalRead(PIN_TC_SO)) {
      raw |= (uint16_t)(1U << i);
    }

    digitalWrite(PIN_TC_SCK, HIGH);
    delayMicroseconds(10);
  }

  digitalWrite(PIN_TC_CS, HIGH);
  digitalWrite(PIN_TC_SCK, LOW);

  return raw;
}

bool sampleTemperature(float& result, const char*& error) {
  uint16_t raw = readMAX6675Raw();

  if (raw & 0x8000) {
    error = "Invalid MAX6675 frame";
    return false;
  }

  if (raw & 0x0004) {
    error = "Thermocouple open";
    return false;
  }

  if (raw & 0x0002) {
    error = "Invalid MAX6675 bit";
    return false;
  }

  float t = (float)((raw >> 3) & 0x0FFF) * 0.25f;

  if (t < MIN_PLAUSIBLE_C || t > MAX_PLAUSIBLE_C) {
    error = "Implausible temperature";
    return false;
  }

  if (!isnan(prevTempC) && fabsf(t - prevTempC) > MAX_SAMPLE_JUMP_C) {
    error = "Unexpected temperature jump";
    return false;
  }

  result = t;
  return true;
}

// ============================================================
// PID
// ============================================================

void updatePID(float previous, float current) {
  const float dt = SAMPLE_MS / 1000.0f;

  float error = TARGET_C - current;

  if (error <= 0.0f) {
    dutyPercent = 0.0f;
    integral = 0.0f;
    derivativeFiltered = 0.0f;
    heaterOff();
    return;
  }

  float rate = (current - previous) / dt;

  derivativeFiltered = 0.75f * derivativeFiltered + 0.25f * rate;

  float tentativeIntegral = integral + error * dt;
  tentativeIntegral = constrain(tentativeIntegral, 0.0f, 35.0f);

  float tentativeOutput = KP * error + KI * tentativeIntegral - KD * derivativeFiltered;

  if (tentativeOutput < MAX_DUTY) {
    integral = tentativeIntegral;
  }

  dutyPercent = constrain(KP * error + KI * integral - KD * derivativeFiltered, 0.0f, MAX_DUTY);
}

void applyTimeProportionalOutput() {
  if (mode != RUNNING || !interactionActive) {
    heaterOff();
    return;
  }

  unsigned long now = millis();

  if (now - lastGoodAt > SENSOR_STALE_MS || isnan(tempC)) {
    enterFault("Temperature reading timed out");
    return;
  }

  if (now - windowStartedAt >= WINDOW_MS) {
    windowStartedAt += ((now - windowStartedAt) / WINDOW_MS) * WINDOW_MS;
  }

  unsigned long onTime = (unsigned long)(WINDOW_MS * dutyPercent / 100.0f);

  bool requestHeat = dutyPercent > 0.0f && tempC < TARGET_C && (now - windowStartedAt < onTime);

  digitalWrite(PIN_TEC_GATE, requestHeat ? HIGH : LOW);
}

// ============================================================
// Interaction
// ============================================================

void startInteraction() {
  if (mode == FAULT) {
    Serial.println("Cannot start: fault latched (press Reset on the screen).");
    return;
  }

  if (consecutiveGoodReads < 3 || isnan(tempC)) {
    Serial.println("Wait for valid temperature readings");
    return;
  }

  interactionActive = true;
  mode = RUNNING;

  integral = 0.0f;
  derivativeFiltered = 0.0f;
  dutyPercent = 0.0f;

  windowStartedAt = millis();

  Serial.println("INTERACTION START");
}

void pauseInteraction() {
  interactionActive = false;

  heaterOff();

  dutyPercent = 0.0f;
  integral = 0.0f;
  derivativeFiltered = 0.0f;

  if (mode != FAULT) {
    mode = IDLE;
  }

  Serial.println("INTERACTION PAUSED");
}

// ============================================================
// Kiln temperature from the website
// ============================================================

void setKilnFromSite(float t) {
  kilnTemp = constrain(t, KILN_MIN, KILN_MAX);

  if (!vib1200Done && kilnTemp >= 1200.0f) {
    vib1200Done = true;
    startSingleVibration();
    Serial.println("1200C -> SINGLE VIBRATION");
  }

  if (!vib1300Done && kilnTemp >= 1300.0f) {
    vib1300Done = true;
    startDoubleVibration();
    Serial.println("1300C -> DOUBLE VIBRATION");
  }
}

void readKilnArg() {
  if (server.hasArg("t")) {
    setKilnFromSite(server.arg("t").toFloat());
  }
}

// ============================================================
// Web page: shows the website full-screen and relays its messages
// ============================================================

const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<title>Before It Was Blue</title>
<style>
html,body{margin:0;height:100%;overflow:hidden;background:#0e1520;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}
iframe{position:fixed;inset:0;width:100%;height:100%;border:0;display:block}
#badge{position:fixed;left:50%;bottom:calc(10px + env(safe-area-inset-bottom));transform:translateX(-50%);
  padding:7px 14px;border-radius:999px;font-size:13px;color:#fff;background:rgba(160,40,30,.92);
  box-shadow:0 4px 16px rgba(0,0,0,.3);pointer-events:none;display:none;white-space:nowrap}
#offline{position:fixed;inset:0;display:none;align-items:center;justify-content:center;text-align:center;
  padding:24px;color:#e8edf3;background:#0e1520;font-size:16px;line-height:1.5}
#offline b{display:block;font-size:20px;margin-bottom:8px}
</style>
</head>
<body>
<iframe id="site" src="%SITE%" allow="fullscreen"></iframe>
<div id="offline"><div><b>Can't load the website</b>This Wi-Fi has no internet connection.<br>Join a Wi-Fi with internet on both the ESP32 and this device, then reload.</div></div>
<div id="badge"></div>
<script>
const SITE_ORIGIN = new URL("%SITE%").origin;
let ready = false, holding = false;

function send(path){ return fetch(path, {cache:"no-store"}).catch(()=>{}); }

addEventListener("message", e => {
  if (e.origin !== SITE_ORIGIN) return;
  const m = e.data;
  if (!m || m.src !== "biwb") return;
  if (m.t === "ready") { ready = true; document.getElementById("offline").style.display = "none"; }
  else if (m.t === "hold") { holding = m.on; send((m.on ? "/start?t=" : "/stop?t=") + m.temp); }
  else if (m.t === "temp") { send("/kiln?t=" + m.temp); }
  else if (m.t === "reset") { holding = false; send("/reset"); }
});

// Website never said hello -> probably no internet on this Wi-Fi.
setTimeout(() => { if (!ready) document.getElementById("offline").style.display = "flex"; }, 10000);

// Release the heater if the page is hidden (screen locked, app switched).
document.addEventListener("visibilitychange", () => { if (document.hidden && holding) { holding = false; send("/stop"); } });

// Small badge only when something needs attention.
const badge = document.getElementById("badge");
function showBadge(t){ badge.textContent = t; badge.style.display = t ? "block" : "none"; }
async function poll(){
  try {
    const r = await fetch("/status", {cache:"no-store"});
    const d = await r.json();
    if (d.mode === "FAULT") showBadge("Wearable safety stop: " + d.fault + " · tap Reset");
    else if (!d.sensorReady) showBadge("Wearable warming up… waiting for sensor");
    else showBadge("");
  } catch (_) { showBadge("Wearable not reachable"); }
}
setInterval(poll, 1000); poll();
</script>
</body>
</html>
)rawliteral";

// ============================================================
// Web handlers
// ============================================================

String pageHtml;

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html", pageHtml);
}

void handleStart() {
  serialTest = false;
  lastPageMsgAt = millis();
  readKilnArg();
  startInteraction();
  server.send(200, "text/plain", "START");
}

void handleStop() {
  lastPageMsgAt = millis();
  readKilnArg();
  pauseInteraction();
  server.send(200, "text/plain", "STOP");
}

void handleKiln() {
  lastPageMsgAt = millis();
  readKilnArg();
  server.send(200, "text/plain", "OK");
}

void handleReset() {
  pauseInteraction();

  kilnTemp = KILN_MIN;

  vib1200Done = false;
  vib1300Done = false;

  vibrationOff();
  vibState = VIB_IDLE;

  // Reset also clears a latched fault, once the sensor reads fine again.
  if (mode == FAULT && consecutiveGoodReads >= 3) {
    mode = IDLE;
    lastFault = "";
    Serial.println("FAULT CLEARED");
  }

  server.send(200, "text/plain", "RESET");
  Serial.println("SIMULATION RESET");
}

void handleStatus() {
  String json = "{";

  json += "\"kiln\":";
  json += String(kilnTemp, 1);

  json += ",\"real\":";
  json += isnan(tempC) ? String("0") : String(tempC, 2);

  json += ",\"duty\":";
  json += String(dutyPercent, 0);

  json += ",\"sensorReady\":";
  json += (consecutiveGoodReads >= 3 && !isnan(tempC)) ? "true" : "false";

  json += ",\"mode\":";
  if (mode == RUNNING)    json += "\"RUN\"";
  else if (mode == FAULT) json += "\"FAULT\"";
  else                    json += "\"IDLE\"";

  json += ",\"fault\":\"";
  json += lastFault;
  json += "\"}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

// ============================================================
// Setup
// ============================================================

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(MDNS_NAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Joining Wi-Fi \"");
  Serial.print(WIFI_SSID);
  Serial.print("\"");

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(false);   // faster replies to the page
    Serial.print("Open: http://");
    Serial.print(WiFi.localIP());
    if (MDNS.begin(MDNS_NAME)) {
      Serial.print("  or  http://");
      Serial.print(MDNS_NAME);
      Serial.print(".local");
    }
    Serial.println();
  } else {
    Serial.println("Could not join the Wi-Fi. Starting fallback network instead.");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("Fallback Wi-Fi: ");
    Serial.print(AP_SSID);
    Serial.print("  Open: http://");
    Serial.println(WiFi.softAPIP());
  }
}

void setup() {
  pinMode(PIN_TEC_GATE, OUTPUT);
  heaterOff();

  pinMode(PIN_VIB, OUTPUT);
  vibrationOff();

  pinMode(PIN_TC_SCK, OUTPUT);
  pinMode(PIN_TC_CS, OUTPUT);
  pinMode(PIN_TC_SO, INPUT);

  digitalWrite(PIN_TC_CS, HIGH);
  digitalWrite(PIN_TC_SCK, LOW);

  Serial.begin(115200);
  delay(700);

  Serial.println();
  Serial.println("=== PORCELAIN INTERACTIVE DEMO ===");

  pageHtml = String(PAGE);
  pageHtml.replace("%SITE%", SITE_URL);

  setupWiFi();

  server.on("/", handleRoot);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/kiln", handleKiln);
  server.on("/reset", handleReset);
  server.on("/status", handleStatus);

  server.begin();

  Serial.println("Web server started");
  Serial.println("H = start");
  Serial.println("S = stop");
}

// ============================================================
// Loop
// ============================================================

void loop() {
  unsigned long now = millis();

  server.handleClient();

  // Temperature sampling
  if (now - lastSampleAt >= SAMPLE_MS) {
    lastSampleAt = now;

    const char* reason = "Unknown sensor error";
    float measured = NAN;

    if (!sampleTemperature(measured, reason)) {
      consecutiveGoodReads = 0;
      enterFault(reason);
    } else {
      float previous = isnan(tempC) ? measured : tempC;

      prevTempC = measured;
      tempC = measured;
      lastGoodAt = now;

      if (consecutiveGoodReads < 10)
        ++consecutiveGoodReads;

      if (tempC >= HARD_STOP_C) {
        enterFault("Hard temperature cutoff reached");
      } else if (mode == RUNNING && interactionActive) {
        updatePID(previous, tempC);
      }

      Serial.print("Real: ");
      Serial.print(tempC, 2);
      Serial.print(" C | Kiln: ");
      Serial.print(kilnTemp, 0);
      Serial.print(" C | Duty: ");
      Serial.print(dutyPercent, 0);
      Serial.print("% | ");

      if (mode == RUNNING)    Serial.println("RUN");
      else if (mode == FAULT) Serial.println("FAULT");
      else                    Serial.println("IDLE");
    }
  }

  // Safety: page went quiet while heating -> pause
  if (interactionActive && !serialTest && now - lastPageMsgAt > PAGE_TIMEOUT_MS) {
    Serial.println("Page stopped responding");
    pauseInteraction();
  }

  // Serial fallback (for bench testing without the page)
  while (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'H' || cmd == 'h') {
      serialTest = true;
      startInteraction();
    }

    if (cmd == 'S' || cmd == 's') {
      pauseInteraction();
    }
  }

  updateVibration();
  applyTimeProportionalOutput();
}
