// ============================================================================
//  pH electrode simulator and leakage tester: ESP32-C3 + ADS1115 + OLED SSD1306
// ============================================================================
//  Shows two voltages at once:
//    A  - the simulator voltage (what the electrode would produce), bipolar (±)
//    S  - output S of the pH board under test
//  so the board's transfer characteristic is visible directly: how much S it
//  gives for a known input A. From two calibration points (series resistor
//  shorted) it then works out the board's input leakage current.
//
//  --------------------------------------------------------------------------
//  WIRING (docs/wiring.png). ADS1115 is powered from 3.3 V - 5 V I2C would kill the ESP32-C3
//  --------------------------------------------------------------------------
//    ESP32-C3           ->  SDA=GPIO5, SCL=GPIO6  (leave GPIO8/9 alone - strapping/BOOT)
//    OLED SSD1306 0x3C  ->  VCC=3V3, GND=GND, SDA=GPIO5, SCL=GPIO6
//    ADS1115     0x48   ->  VDD=3V3, GND=GND, SDA=GPIO5, SCL=GPIO6, ADDR->GND
//    I2C pull-ups 4.7k to 3V3 (the modules usually have them already).
//
//  JST 3-pin = the pH board connector: [ GND ][ +5 V board supply ][ output S ]
//    +5 V comes from the ESP's 5V pin (i.e. from USB). Board ground returns to the
//    calibrator's GND through the current shunt Rsh.
//
//  CHANNEL A (simulator millivolts) - DIFFERENTIAL AIN0-AIN1:
//    AIN0 <- trimmer wiper (taken BEFORE the series resistor, on the low-impedance side)
//    AIN1 <- the trimmer leg that goes to the BNC shield (the reference)
//    The simulator runs on its own battery, i.e. it floats; A0-A1 is the signed voltage
//    between wiper and reference, range at 3.3 V about ±1.6 V.
//    Never wire the BNC centre (the high-impedance node) to the ADS: the ADS input
//    would itself become the leakage being measured.
//
//  CHANNEL S (pH board output, 0..5 V) - SINGLE-ENDED AIN2 to GND:
//    S -> divider ÷2 (R3=R4=10k, bottom to GND) -> AIN2. 0..5 V becomes 0..2.5 V,
//    which is safe for the ADS at 3.3 V. The firmware multiplies back by S_DIVIDER.
//
//  BOARD CURRENT - SINGLE-ENDED AIN3: low-side shunt Rsh in the board's ground.
//
//  --------------------------------------------------------------------------
//  Serial (USB CDC, 115200): prints CSV "millis,A_mV,S_V,pH" and takes commands:
//    z  - autozero channel A (store the present A as 0.000 V in NVS)
//    Z  - clear the autozero
//    r  - print the stored calibration
//    v  - wiring check: each ADS input AIN0..AIN3 single-ended against GND
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>
#include <Adafruit_ADS1X15.h>
#include <U8g2lib.h>   // display through u8g2 (Adafruit_SSD1306 would not drive this panel)

// Web interface: own access point + captive DNS + asynchronous server.
// The async server runs in the AsyncTCP task, so HTTP never blocks measuring or the display,
// and I2C (ADS/OLED) is done ONLY in loop(); web handlers just read shared values
// and raise flags - neither path ever holds up the other.
#include <WiFi.h>
#include <esp_wifi.h>      // esp_wifi_set_country (allow channels 12/13 in the EU)
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>

// Calibrator firmware version - shown on the page (/info) and on Serial at start-up.
#define CAL_VERSION "1.1.1"

// The Wi-Fi network is set on the web page (Wi-Fi block) and kept in NVS (namespace "wifi").
// An optional src/wifi_secret.h (not in git) gives a default network for your own build -
// used only until another one is saved on the page or «Forget the network» is pressed.
#if __has_include("wifi_secret.h")
#include "wifi_secret.h"   // WIFI_SSID / WIFI_PASS
#endif
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASS
#define WIFI_PASS ""
#endif

// ----------------------------- Settings -------------------------------------
static const uint8_t PIN_SDA = 5;
static const uint8_t PIN_SCL = 6;

static const uint8_t ADDR_ADS  = 0x48;   // ADDR->GND
static const uint8_t ADDR_OLED = 0x3C;   // usual 128x64 address

static const uint8_t  OLED_W = 128;
static const uint8_t  OLED_H = 64;

// Samples averaged per channel. 25 at 250 SPS = exactly 100 ms = 5 periods of 50 Hz
// mains -> hum is rejected best (matters for high-impedance measurements, 100 MOhm).
static const uint8_t  SAMPLES = 25;

// Display/log update period, ms
static const uint32_t PERIOD_MS = 100;

// Adaptive smoothing of A and S: a real change passes AT ONCE (no lag), while
// flicker in the last digit is smoothed by an EMA. SNAP = the "at once" threshold.
static const float DISP_ALPHA = 0.35f;   // smoothing strength for small noise
static const float SNAP_A = 0.0004f;     // A: a step > 0.4 mV passes at once
static const float SNAP_S = 0.003f;      // S: a step > 3 mV passes at once

// ADS1115 gain ranges (pick for your voltages):
//   A: ±1.6 V -> GAIN_TWO (±2.048 V, 62.5 µV/LSB). If the simulator only gives
//      small voltages (e.g. ±0.5 V), GAIN_FOUR (±1.024) gives more resolution.
//   S: after ÷2 it is 0..2.5 V -> GAIN_ONE (±4.096 V) with headroom.
static const adsGain_t GAIN_A = GAIN_TWO;
static const adsGain_t GAIN_S = GAIN_ONE;

// Sign of channel A. Check (README, «Sign of channel A»): button pressed, multimeter
// between BNC centre and shield - A must have the SAME sign as the meter.
// If the signs differ, change -1.0 <-> 1.0 here (or swap the AIN0/AIN1 wires).
// -1.0 is for the author's calibrator: its wiper and reference leg are the other way round.
static const float A_SIGN = -1.0f;

// External divider on input S, in series: S -[10k]- AIN2 -[10k]- ground,
// so AIN2 = S/2 and the code multiplies back by 2. The ADS input stays below ~2.1 V
// even at S = 4.2 V (within the 3.3 V supply).
static const float S_DIVIDER = 2.0f;

// S measurement: 0 -> single-ended AIN2 to GND (S = 0..5 V through the ÷2 divider).
//                1 -> differential AIN2-AIN3 (kept in case S is ever bipolar).
#define S_DIFFERENTIAL 0

// pH board supply current: low-side shunt Rsh in the board's ground -> AIN3.
//   All of the board's return current flows through Rsh; I = V(AIN3) / R_SHUNT.
//   A few mA in 10 Ohm -> tens of mV (within ±0.512 V).
static const adsGain_t GAIN_I  = GAIN_EIGHT;   // ±0.512 V (15.6 µV/LSB) -> up to ~51 mA at 10 Ohm
static const float     R_SHUNT = 10.0f;        // Ohm (shunt in the board's ground)

// pH from the channel A millivolts (Nernst): pH 7 at 0 mV, "+" mV -> acid.
//   0 mV=pH7, +59.16 mV=pH6, +414 mV≈pH0, -59.16 mV=pH8, -414 mV≈pH14.
static const float PH_ISO   = 7.0f;      // pH at 0 mV
static const float PH_SLOPE = 59.16f;    // mV per pH (25 °C)

// --- Web / access point / DNS ---
static const char* AP_SSID = "pH-Calibrator";   // the calibrator's own network
static const char* AP_PASS = "calib1234";        // its password (>= 8 characters)
static const char* HOSTNAME = "calibrator";      // mDNS -> http://calibrator.local
static const byte  DNS_PORT = 53;

// Series resistor between the simulator source and the board input (for the leakage).
// Can be changed on the web page; 50 MOhm by default. Kept in NVS.
static const float R_SERIES_DEFAULT = 50.0e6f;   // Ohm

// Leakage limit, nA. The page shows how many times the leakage exceeds it.
// Changed on the web page, kept in NVS.
static const float LEAK_NORM_DEFAULT = 5.0f;   // nA

// ----------------------------- Objects --------------------------------------
Adafruit_ADS1115 ads;
// u8g2, hardware I2C on the same pins (rotation, reset, clock=SCL, data=SDA)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE, PIN_SCL, PIN_SDA);
Preferences prefs;

bool  adsOk  = false;
bool  oledOk = false;

// Calibration (NVS namespace "cal"): only the autozero of A.
// The dead multipliers scaleA/scaleS and the offset offsetS are gone - a stale
// sclS left in NVS by an older firmware used to double the S reading.
float offsetA = 0.0f;   // channel A autozero, V (command 'z')

// --- Web server and DNS ---
DNSServer      dnsServer;
AsyncWebServer server(80);
bool           wifiOk = false;
bool           apMode = false;   // true = own access point is up (fallback mode)
uint32_t       wifiStartMs = 0;  // when the STA connection started (timeout in loop)
bool           staConnected = false;
char           g_ip[20]     = "";   // IP shown on the OLED (ASCII)
uint32_t       connMsgUntil = 0;    // show the network screen until this time, ms
String         wifiSsid, wifiPass;  // network: from NVS ("wifi"), else from wifi_secret.h
uint32_t       restartAt = 0;       // restart after a network is saved (0 = no)

// Latest readings - written by loop(), read by the web handlers (another task).
// 32-bit floats are read/written atomically on RISC-V; volatile forces a re-read.
// Telemetry only, no lock needed.
volatile float g_E_mV    = 0.0f;   // A, mV (signed)
volatile float g_S_V     = 0.0f;   // S, V
volatile float g_pH      = 7.0f;
volatile float g_Vin_mV  = 0.0f;   // calculated board input, mV
volatile float g_Ileak_nA= 0.0f;   // leakage current, nA
volatile float g_noise_mV= 0.0f;   // spread of S within a cycle, mV (RMS)
volatile float g_I_mA    = 0.0f;   // pH board supply current, mA (shunt on AIN3)

// Leakage calibration (NVS namespace "leak"): the line S = zeroS + gainS*E.
bool  haveP1 = false, haveP2 = false;
float E1_mV = 0, S1_V = 0;         // point 1 (button pressed)
float E2_mV = 0, S2_V = 0;         // point 2
float gainS  = 0;                  // V per mV (slope of S over E)
float zeroS  = 0;                  // V (S at E=0)
float Rseries = R_SERIES_DEFAULT;  // Ohm
float leakNormNA = LEAK_NORM_DEFAULT;  // leakage limit, nA
bool  calValid = false;            // is the line known (two points)

// Request flags from the web handlers -> carried out by loop() (I2C/NVS in one task).
volatile bool  req_capP1  = false;
volatile bool  req_capP2  = false;
volatile bool  req_calRst = false;
volatile bool  req_zeroA  = false;
volatile bool  req_setR   = false;
volatile float req_Rval   = 0.0f;
volatile bool  req_setNorm = false;
volatile float req_NormVal = 0.0f;
volatile bool  req_wifiSave   = false;   // save req_ssid/req_pass and restart
volatile bool  req_wifiForget = false;   // erase the network and restart as an access point
char           req_ssid[33]   = "";
char           req_pass[64]   = "";

// ----------------------------- Utilities ------------------------------------
static void loadCal() {
  prefs.begin("cal", true);
  offsetA = prefs.getFloat("offA", 0.0f);
  prefs.end();
}

static void saveFloat(const char* key, float v) {
  prefs.begin("cal", false);
  prefs.putFloat(key, v);
  prefs.end();
}

// Wi-Fi network: the one saved on the page (NVS) wins. Key "ssid" present but empty =
// the network was forgotten on purpose -> access point at once, no wifi_secret.h fallback.
static void loadWifi() {
  bool saved = false;
  if (prefs.begin("wifi", true)) {
    saved = prefs.isKey("ssid");
    if (saved) { wifiSsid = prefs.getString("ssid", ""); wifiPass = prefs.getString("pass", ""); }
    prefs.end();
  }
  if (!saved) { wifiSsid = WIFI_SSID; wifiPass = WIFI_PASS; }
}

static void saveWifi(const char* ssid, const char* pass) {
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
}

// String inside JSON quotes: an SSID may contain quotes and backslashes.
static String jsonEsc(const String& s) {
  String o; o.reserve(s.length() + 2);
  for (size_t i = 0; i < s.length(); i++) {
    char ch = s[i];
    if (ch == '"' || ch == '\\') { o += '\\'; o += ch; }
    else if ((uint8_t)ch < 0x20) { char b[8]; snprintf(b, sizeof(b), "\\u%04x", ch); o += b; }
    else o += ch;
  }
  return o;
}

// The line S=zeroS+gainS*E from the two points (if they differ in E).
static void computeLeakCal() {
  calValid = false;
  if (haveP1 && haveP2 && fabsf(E2_mV - E1_mV) > 1e-3f) {
    gainS = (S2_V - S1_V) / (E2_mV - E1_mV);   // V/mV
    zeroS = S1_V - gainS * E1_mV;              // V at E=0
    calValid = true;
  }
}

// Load the leakage calibration from NVS (namespace "leak").
static void loadLeak() {
  prefs.begin("leak", true);
  haveP1 = prefs.getBool("h1", false);
  haveP2 = prefs.getBool("h2", false);
  E1_mV  = prefs.getFloat("E1", 0);  S1_V = prefs.getFloat("S1", 0);
  E2_mV  = prefs.getFloat("E2", 0);  S2_V = prefs.getFloat("S2", 0);
  Rseries= prefs.getFloat("R",  R_SERIES_DEFAULT);
  leakNormNA = prefs.getFloat("norm", LEAK_NORM_DEFAULT);
  prefs.end();
  computeLeakCal();
}

// Save the whole leakage state (points + R + limit) to NVS.
static void saveLeak() {
  prefs.begin("leak", false);
  prefs.putBool("h1", haveP1);  prefs.putBool("h2", haveP2);
  prefs.putFloat("E1", E1_mV);  prefs.putFloat("S1", S1_V);
  prefs.putFloat("E2", E2_mV);  prefs.putFloat("S2", S2_V);
  prefs.putFloat("R",  Rseries);
  prefs.putFloat("norm", leakNormNA);
  prefs.end();
}

static void i2cScan() {
  Serial.println(F("I2C scan:"));
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  found 0x%02X\n", a);
    }
  }
}

// Averaged channel A (differential AIN0-AIN1), returns volts.
static float readA() {
  ads.setGain(GAIN_A);
  int32_t acc = 0;
  for (uint8_t i = 0; i < SAMPLES; i++) acc += ads.readADC_Differential_0_1();
  float v = A_SIGN * ads.computeVolts((int16_t)(acc / SAMPLES));   // polarity
  return v - offsetA;
}

// Averaged channel S, returns volts. Also works out the noise (RMS) -> g_noise_mV.
static float readS() {
  ads.setGain(GAIN_S);
  int16_t s[SAMPLES];
  int32_t acc = 0;
  for (uint8_t i = 0; i < SAMPLES; i++) {
#if S_DIFFERENTIAL
    s[i] = ads.readADC_Differential_2_3();
#else
    s[i] = ads.readADC_SingleEnded(2);
#endif
    acc += s[i];
  }
  float meanCnt = (float)acc / SAMPLES;
  // RMS in counts -> volts at the board output (divider included)
  float sq = 0;
  for (uint8_t i = 0; i < SAMPLES; i++) { float d = s[i] - meanCnt; sq += d * d; }
  float stdCnt = sqrtf(sq / SAMPLES);
  g_noise_mV = ads.computeVolts((int16_t)lroundf(stdCnt)) * S_DIVIDER * 1000.0f;
  float v = ads.computeVolts((int16_t)lroundf(meanCnt));
  return v * S_DIVIDER;   // ×2: undo the external ÷2 divider on input S
}

// pH board supply current: mean of AIN3 (single-ended) - the drop across Rsh -> mA.
static float readI() {
  ads.setGain(GAIN_I);
  int32_t acc = 0;
  for (uint8_t i = 0; i < SAMPLES; i++) acc += ads.readADC_SingleEnded(3);
  float v = ads.computeVolts((int16_t)(acc / SAMPLES));   // drop across the shunt, V
  if (v < 0) v = 0;                                        // noise around zero
  return v / R_SHUNT * 1000.0f;                            // mA
}

// Signed voltage with 3 decimals: "+1.234"
static void fmtV(char* buf, size_t n, float v) {
  snprintf(buf, n, "%+.3f", (double)v);
}

// Does a device answer at this address (u8g2.begin does not check the ACK, so we do)
static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// ----------------------------- Display --------------------------------------
// pH from the channel A voltage (in volts), Nernst
static float phFromA(float va) {
  return PH_ISO - (va * 1000.0f) / PH_SLOPE;
}

static void drawScreen(float va, float vs) {
  if (!oledOk) return;
  char lph[24], nA[16], nS[16];
  snprintf(lph, sizeof(lph), "pH %.2f  %dmA", phFromA(va), (int)lroundf(g_I_mA));
  snprintf(nA,  sizeof(nA),  "%.1f", (va * 1000.0f));   // signed number, large
  snprintf(nS,  sizeof(nS),  "%.3f", vs);

  oled.clearBuffer();
  int x;

  // pH - large, centred (baseline a little lower so the top is not clipped)
  oled.setFont(u8g2_font_helvB14_tr);
  x = (128 - oled.getStrWidth(lph)) / 2;
  oled.drawStr(x, 19, lph);

  // A: small letter + large digits + small unit "mV"
  oled.setFont(u8g2_font_helvB12_tr);
  oled.drawStr(0, 42, "A: ");
  x = oled.getStrWidth("A: ") + 4;
  oled.setFont(u8g2_font_helvB14_tr);
  oled.drawStr(x, 42, nA);
  x += oled.getStrWidth(nA) + 4;
  oled.setFont(u8g2_font_helvB12_tr);
  oled.drawStr(x, 42, "mV");

  // S: small letter + large digits + small unit "V"
  oled.setFont(u8g2_font_helvB12_tr);
  oled.drawStr(0, 63, "S: ");
  x = oled.getStrWidth("S: ") + 4;
  oled.setFont(u8g2_font_helvB14_tr);
  oled.drawStr(x, 63, nS);
  x += oled.getStrWidth(nS) + 4;
  oled.setFont(u8g2_font_helvB12_tr);
  oled.drawStr(x, 63, "V");

  oled.sendBuffer();
}

// Network status screen: 3 centred lines (Latin only - the _tr fonts have no Cyrillic).
static void drawStatus(const char* l1, const char* l2, const char* l3) {
  if (!oledOk) return;
  oled.clearBuffer();
  int w;
  oled.setFont(u8g2_font_helvB12_tr);
  w = oled.getStrWidth(l1); oled.drawStr((128 - w) / 2, 20, l1);
  oled.setFont(u8g2_font_8x13_tr);
  w = oled.getStrWidth(l2); oled.drawStr((128 - w) / 2, 40, l2);
  if (l3 && l3[0]) {
    oled.setFont(u8g2_font_6x10_tr);
    w = oled.getStrWidth(l3); oled.drawStr((128 - w) / 2, 56, l3);
  }
  oled.sendBuffer();
}

static void showError(const char* msg) {
  if (!oledOk) return;
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(0, 20, msg);
  oled.sendBuffer();
}

// ----------------------------- Serial commands ------------------------------
static void handleSerial(float va) {
  if (!Serial.available()) return;
  char c = (char)Serial.read();
  switch (c) {
    case 'z':  // autozero A: the present A becomes 0.000
      offsetA += va;               // va already includes the old offsetA -> add
      saveFloat("offA", offsetA);
      Serial.printf("Autozero A: offsetA=%.5f V\n", offsetA);
      break;
    case 'Z':  // clear the autozero of A
      offsetA = 0.0f;
      saveFloat("offA", offsetA);
      Serial.println(F("Autozero A cleared"));
      break;
    case 'r':  // print the calibration
      Serial.printf("offA=%.5f\n", offsetA);
      break;
    case 'v':  // wiring check: AIN1 at ~1.65 V = VMID divider, ~0 V = tied to GND
      ads.setGain(GAIN_ONE);
      for (uint8_t ch = 0; ch < 4; ch++) {
        int32_t acc = 0;
        for (uint8_t i = 0; i < SAMPLES; i++) acc += ads.readADC_SingleEnded(ch);
        Serial.printf("AIN%u = %.4f V\n", ch, ads.computeVolts((int16_t)(acc / SAMPLES)));
      }
      break;
    default:
      break;
  }
}

// ----------------------------- Web page -------------------------------------
static const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>pH calibrator</title>
<style>
/* Palette as on the KH station dashboard: grey background + green accent */
:root{
  --bg:#2c2c2c; --card:#3b3b3b; --card2:#333; --line:#555;
  --fg:#fff; --dim:#b9b9b9; --muted:#8d8d8d;
  --ok:#3fb950; --bad:#f05252; --warn:#e8a33d; --accent:#3fb950; --blue:#58a6ff;
  color-scheme:dark;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.4 system-ui,-apple-system,"Segoe UI",Arial,sans-serif;padding:14px}
h1{font-size:18px;margin:0}
.top{display:flex;align-items:center;gap:10px;margin:0 0 12px;flex-wrap:wrap}
.top .ver{font-size:12px;color:var(--muted);margin-left:auto}
.lang{display:flex;gap:4px}
.lang button{flex:none;min-width:0;margin:0;padding:2px;line-height:0;background:none;border:1px solid transparent;border-radius:3px;opacity:.4;cursor:pointer}
.lang button:hover{opacity:.8}
.lang button.on{opacity:1;border-color:var(--line)}
.lang svg{display:block;width:21px;height:14px}
html:not([lang="uk"]) .i-uk{display:none}
html[lang="uk"] .i-en{display:none}
.ban{background:#4a3a1c;border:1px solid var(--warn);border-radius:8px;padding:10px 12px;margin:0 0 12px;color:#fff}
.ban button{margin:0 0 0 8px;padding:6px 10px}
h2{font-size:14px;color:var(--dim);margin:18px 0 8px;text-transform:uppercase;letter-spacing:.05em}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:10px}
.tile{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:10px 12px}
.tile .k{font-size:12px;color:var(--dim)}
.tile .v{font-size:22px;font-weight:600;margin-top:2px}
.tile .u{font-size:12px;color:var(--dim);font-weight:400}
.tile.ph .v{color:var(--accent)}
.tile .sub{font-size:14px;color:var(--dim);margin-top:4px}
.tile .sub b{font-size:19px;color:var(--accent);font-weight:700}
.leak .v{font-size:22px;color:var(--warn)}
button{background:var(--card);color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:10px 14px;font-size:14px;cursor:pointer;margin:4px 6px 4px 0}
button.p{border-color:var(--accent);color:var(--accent)}
button.g{background:var(--card2);color:var(--dim)}
button:active{transform:translateY(1px)}
.row{display:flex;flex-wrap:wrap;align-items:center;gap:8px}
input,select{background:#2c2c2c;border:1px solid var(--line);color:var(--fg);border-radius:6px;padding:8px;width:90px;font-size:14px}
input.w,select.w{width:220px;max-width:100%}
.pt{font-size:13px;color:var(--muted);margin:2px 0}
.ok{color:var(--ok)}.bad{color:var(--bad)}
small{color:var(--dim)}
.graphs{display:flex;flex-wrap:wrap;gap:12px;margin:8px 0}
.gcol{flex:1 1 300px;min-width:240px;display:flex;flex-direction:column}
.gcap{font-size:12px;color:var(--dim);margin:0 0 4px}
.gcanvas{width:100%;height:240px;background:var(--card);border:1px solid var(--line);border-radius:8px;cursor:crosshair;touch-action:none}
.block{border:1px solid var(--line);border-radius:10px;padding:2px 14px 16px;margin:16px 0;background:var(--card2)}
.block>h2:first-child{margin-top:14px;color:var(--accent)}
h3{font-size:13px;color:var(--dim);margin:16px 0 6px;text-transform:uppercase;letter-spacing:.04em}
.hint{font-size:12px;color:var(--muted);margin:4px 0 0}
.block>h2{cursor:pointer;user-select:none}
.block.collapsed .bbody{display:none}
.car{display:inline-block;font-size:11px;color:var(--dim);transition:transform .15s}
.block.collapsed .car{transform:rotate(-90deg)}
.camb{color:var(--warn)}
.cgrn{color:var(--accent)}
input[type=checkbox]{width:auto;margin:0 5px 0 0;vertical-align:middle}
.phwrap{display:flex;flex-direction:column;align-items:flex-start;margin:12px 0}
#cPh{width:100%;aspect-ratio:1/1;background:var(--card);border:1px solid var(--line);border-radius:8px;cursor:crosshair}
</style></head><body>
<div class="top"><h1>pH calibrator</h1><span class="lang" id="lang"><button type="button" data-l="en" title="English" aria-label="English"><svg viewBox="0 0 60 30" preserveAspectRatio="none" aria-hidden="true"><clipPath id="ukS"><path d="M0 0v30h60V0z"/></clipPath><clipPath id="ukT"><path d="M30 15h30v15zv15H0zH0V0zV0h30z"/></clipPath><g clip-path="url(#ukS)"><path d="M0 0v30h60V0z" fill="#012169"/><path d="M0 0l60 30m0-30L0 30" stroke="#fff" stroke-width="6"/><path d="M0 0l60 30m0-30L0 30" clip-path="url(#ukT)" stroke="#C8102E" stroke-width="4"/><path d="M30 0v30M0 15h60" stroke="#fff" stroke-width="10"/><path d="M30 0v30M0 15h60" stroke="#C8102E" stroke-width="6"/></g></svg></button><button type="button" data-l="uk" title="Українська" aria-label="Українська"><svg viewBox="0 0 3 2" aria-hidden="true"><path d="M0 0h3v1H0z" fill="#0057B7"/><path d="M0 1h3v1H0z" fill="#FFD700"/></svg></button></span><span class="ver" id="ver"></span></div>

<div class="ban" id="apban" style="display:none">No Wi-Fi network yet: the calibrator runs its own access point.<button class="p" onclick="openWifi()">Wi-Fi setup</button></div>

<h2>Current readings</h2>
<div class="grid">
  <div class="tile"><div class="k">A (simulator)</div><div class="v"><span id="e">–</span> <span class="u">mV</span> · <span class="u">pH</span> <b id="phA" class="camb">–</b></div></div>
  <div class="tile"><div class="k">S (board output)</div><div class="v"><span id="s">–</span> <span class="u">V</span> · <span class="u">pH</span> <b id="phS" class="cgrn">–</b></div></div>
  <div class="tile"><div class="k">S noise</div><div class="v"><span id="noise">–</span> <span class="u">mV</span></div></div>
  <div class="tile"><div class="k">Board current</div><div class="v"><span id="cur">–</span> <span class="u">mA</span></div></div>
  <div class="tile"><div class="k">Gain (zero <span id="zerot">–</span> V)</div><div class="v"><span id="gaint">–</span> <span class="u">V/V</span></div></div>
</div>

<section class="block" id="blk1">
<h2 onclick="togBlk('blk1')"><span class="car">▾</span> Block 1 · Board leakage</h2>
<div class="bbody">
<div class="grid">
  <div class="tile leak"><div class="k">Leakage current</div><div class="v"><span id="il">–</span> <span class="u">nA</span></div></div>
  <div class="tile"><div class="k">Over the limit (limit <span id="normlbl">–</span> nA)</div><div class="v" id="over">–</div></div>
  <div class="tile"><div class="k">Board input (calculated)</div><div class="v"><span id="vin">–</span> <span class="u">mV</span></div></div>
</div>
<small id="leakhint">Calibrate with two points below first - the leakage is calculated from them.</small>

<h3>Calibration (button PRESSED, series resistor shorted)</h3>
<p class="pt i-en"><b>E</b> is the simulator voltage, the same one shown above as <b>A</b> (in mV); you set it with the trimmer. The pressed button shorts the series resistor (takes it out of the circuit), so the board input gets exactly E - a clean reference point. Release the button to measure the leakage.</p>
<p class="pt i-uk"><b>E</b> — це напруга імітатора, та сама, що вгорі показана як <b>A</b> (у мВ); ви задаєте її підстроєчним резистором. Натиснута кнопка закорочує послідовний резистор (прибирає його з кола), тож на вхід плати йде рівно E — це дає чисту опорну точку. Для виміру витоку кнопку відпустіть.</p>
<p class="pt i-en"><b>Step 1.</b> Set some voltage E with the trimmer (watch A above) → «Take point 1».<br>
<b>Step 2.</b> Set a clearly different E (the farther from the first, the more accurate) → «Take point 2».<br>
The calibrator draws the board's «E → S» line (gain and zero) and from then on calculates the leakage current.</p>
<p class="pt i-uk"><b>Крок 1.</b> Виставте резистором якусь напругу E (дивіться на A вгорі) → «Зняти точку 1».<br>
<b>Крок 2.</b> Виставте помітно іншу E (що далі від першої, то точніше) → «Зняти точку 2».<br>
Прилад побудує пряму «E → S» плати (підсилення і нуль), і після цього рахуватиме струм витоку.</p>
<div class="pt">Point 1: <span id="p1">—</span></div>
<div class="pt">Point 2: <span id="p2">—</span></div>
<div class="pt">Line: gain <b><span id="gain">—</span></b> V/V, zero <b><span id="zero">—</span></b> V — <span id="valid" class="bad">not ready</span></div>
<div class="pt">With the leakage the input is shifted by <b><span id="leakoff">—</span></b> mV → error <b><span id="leakph">—</span></b> pH (dashed on the chart).</div>
<div class="row">
  <button class="p" onclick="post('/p1')">Take point 1</button>
  <button class="p" onclick="post('/p2')">Take point 2</button>
  <button class="g" onclick="post('/calreset')">Reset calibration</button>
</div>
<div class="row">
  <label>Resistor R, MΩ:</label>
  <input id="rin" type="number" step="0.1" min="0.1">
  <button onclick="setR()">Save R</button>
  <button class="g" onclick="post('/zeroa')">Autozero A</button>
  <label>Leakage limit, nA:</label>
  <input id="nin" type="number" step="0.001" min="0.001">
  <button onclick="setNorm()">Save limit</button>
</div>

<div class="graphs">
  <div class="gcol" style="flex:3 1 0">
    <div class="gcap">Leakage over time, nA (wheel: zoom, drag: pan, double-click: whole history)</div><canvas id="cLeak" class="gcanvas"></canvas>
    <div class="gcap" style="margin-top:10px">Calibration: E (mV) → S (V)</div><canvas id="cCal" class="gcanvas"></canvas>
  </div>
  <div class="gcol" style="flex:2 1 0;max-width:420px">
    <div class="gcap">pH: reading vs true — green without leakage, dashed with leakage</div><canvas id="cPh"></canvas>
  </div>
</div>
</div>
</section>

<section class="block" id="blk2">
<h2 onclick="togBlk('blk2')"><span class="car">▾</span> Block 2 · Electrode and pH</h2>
<div class="bbody">
<div class="graphs"><div class="gcol"><div class="gcap">Reference pH (A, simulator) over time (wheel: zoom, drag: pan, double-click: whole history)</div><canvas id="cPhA" class="gcanvas"></canvas></div></div>
<div class="graphs"><div class="gcol"><div class="gcap"><span class="i-en">Real pH (S, board/electrode) over time — <b style="color:var(--accent)">pH</b> (left axis) + <b style="color:var(--blue)">S voltage, V</b> (right axis). wheel: zoom, drag: pan, double-click: all</span><span class="i-uk">рН реальний (S, плата/електрод) у часі — <b style="color:var(--accent)">рН</b> (ліва вісь) + <b style="color:var(--blue)">напруга S, В</b> (права вісь). коліщатко: зум, тягнути: зсув, 2× клік: усе</span></div><canvas id="cPhS" class="gcanvas"></canvas></div></div>

<h3>Electrode equilibrium</h3>
<p class="pt i-en">How fast the pH settles after the electrode is dipped or the solution changes. The guide is the <b>real pH</b> (from S). Press «Start» right after dipping; the calibrator watches the drift and declares equilibrium once the swing over 10 seconds falls inside the quiet band.</p>
<p class="pt i-uk">Як швидко встановлюється рН після занурення електрода або зміни розчину. Орієнтир — <b>рН реальний</b> (з S). Натисніть «Старт» одразу після занурення; прилад стежить за дрейфом і оголошує рівновагу, коли коливання за 10 секунд спадуть у межі смуги спокою.</p>
<div class="row">
  <button class="p" onclick="eqStart()">Start</button>
  <button class="g" onclick="eqStop()">Stop</button>
  <label>Quiet band, pH:</label>
  <input id="eqband" type="number" step="0.005" min="0.005" value="0.02">
  <button class="g" onclick="selStart()">Pick limits on the chart</button>
</div>
<p class="hint i-en">Also, without waiting: press «Pick limits on the chart» and click the <b>start</b> and the <b>end</b> of the transient on the «real pH» chart - the calibrator works out the time, drift, t63 and equilibrium pH for exactly that stretch.</p>
<p class="hint i-uk">Додатково, без очікування: натисніть «Обрати межі на графіку» і клікніть на графіку «рН реальний» <b>початок</b> і <b>кінець</b> перехідного процесу — прилад порахує час, дрейф, t63 і рівноважний рН саме за цим відрізком.</p>
<div class="grid">
  <div class="tile"><div class="k">State</div><div class="v" id="eqstate">—</div></div>
  <div class="tile"><div class="k">Time since start</div><div class="v"><span id="eqt">—</span> <span class="u">s</span></div></div>
  <div class="tile"><div class="k">Drift over 10 s</div><div class="v"><span id="eqdrift">—</span> <span class="u">pH</span></div></div>
  <div class="tile"><div class="k">Time to equilibrium</div><div class="v"><span id="eqset">—</span> <span class="u">s</span></div></div>
  <div class="tile"><div class="k">t63 (time constant)</div><div class="v"><span id="eqt63">—</span> <span class="u">s</span></div></div>
  <div class="tile"><div class="k">Equilibrium pH</div><div class="v" id="eqph">—</div></div>
</div>

<h3>Data and drift</h3>
<p class="hint i-en">The history lives <b>in the browser, not in the calibrator's flash</b>: 1 point/s, a buffer for ~14 h (a night). Do not close the tab or let the computer sleep. The data is also kept in the browser's storage and survives a page reload. For an archive press «Download CSV».</p>
<p class="hint i-uk">Історія живе <b>у браузері, не у флеші приладу</b>: запис 1 точка/с, буфер на ~14 год (ніч). Не закривайте вкладку і не давайте компʼютеру заснути. Дані ще й самі осідають у памʼяті браузера і переживають перезавантаження сторінки. Для архіву натисніть «Завантажити CSV».</p>
<div class="row">
  <button class="p" onclick="expCsv()">Download CSV</button>
  <button class="g" onclick="clrLog()">Clear history</button>
  <span class="hint">points in memory: <b id="npts">0</b></span>
</div>
</div>
</section>

<section class="block" id="blk3">
<h2 onclick="togBlk('blk3')"><span class="car">▾</span> Wi-Fi</h2>
<div class="bbody">
<div class="pt" id="wst">–</div>
<div class="row">
  <button class="g" onclick="scanWifi()">Find networks</button>
  <select id="wlist" class="w" onchange="if(this.value)document.getElementById('wssid').value=this.value"></select>
</div>
<div class="row">
  <label>Network (SSID):</label>
  <input id="wssid" class="w" maxlength="32" autocomplete="off">
  <label>Password:</label>
  <input id="wpass" class="w" type="password" maxlength="63" autocomplete="new-password">
</div>
<div class="row">
  <button class="p" onclick="saveWifi()">Save and restart</button>
  <button class="g" onclick="forgetWifi()">Forget the network</button>
</div>
<p class="hint i-en">The calibrator restarts and joins the network; the page is then at <b>http://calibrator.local/</b> (or the IP shown on the screen). If it cannot connect within 20 s it brings up its own access point <b>pH-Calibrator</b> (password <b>calib1234</b>) again, with the page at <b>http://192.168.4.1/</b>. Only 2.4 GHz networks. An empty password means an open network.</p>
<p class="hint i-uk">Прилад перезапуститься і під'єднається до мережі; сторінка тоді за адресою <b>http://calibrator.local/</b> (або IP з екрана). Якщо за 20 с під'єднатися не вдасться, він знову підніме власну точку доступу <b>pH-Calibrator</b> (пароль <b>calib1234</b>), сторінка — <b>http://192.168.4.1/</b>. Лише мережі 2.4 ГГц. Порожній пароль — відкрита мережа.</p>
</div>
</section>

<script>
// --- Language, EN / UA - the same mechanism as the KH station dashboard.
// Short labels: each text node remembers its English original and is translated through
// the UA dictionary. Strings with numbers go through T() with {0} placeholders. Long
// paragraphs with markup have two copies (.i-en / .i-uk) and CSS shows one. What the
// DEVICE says (server replies, Serial) stays English. The choice is per browser; the
// first visit follows the browser's language.
const UA={
'pH calibrator':'Калібратор pH','Current readings':'Поточні покази','A (simulator)':'A (імітатор)','S (board output)':'S (вихід плати)',
'S noise':'Шум S','Board current':'Струм плати','Gain (zero':'Підсилення (нуль','V)':'В)',
'mV':'мВ','V':'В','nA':'нА','mA':'мА','s':'с','pH':'рН','V/V':'В/В',
'No Wi-Fi network yet: the calibrator runs its own access point.':'Мережу Wi-Fi ще не задано: прилад працює як власна точка доступу.',
'Wi-Fi setup':'Налаштувати Wi-Fi',
'Block 1 · Board leakage':'Блок 1 · Витік плати','Leakage current':'Струм витоку','Over the limit (limit':'Понад норму (норма','nA)':'нА)',
'Board input (calculated)':'Вхід плати (розрах.)',
'Calibrate with two points below first - the leakage is calculated from them.':'Для розрахунку спершу зробіть калібрування двома точками нижче.',
'Calibration (button PRESSED, series resistor shorted)':'Калібрування (кнопка НАТИСНУТА, послідовний резистор закорочено)',
'Point 1:':'Точка 1:','Point 2:':'Точка 2:','Line: gain':'Пряма: підсилення','V/V, zero':'В/В, нуль','V —':'В —',
'not ready':'не готово','ready':'готово',
'With the leakage the input is shifted by':'З витоком вхід зсунуто на','mV → error':'мВ → похибка','pH (dashed on the chart).':'pH (пунктир на графіку).',
'Take point 1':'Зняти точку 1','Take point 2':'Зняти точку 2','Reset calibration':'Скинути калібрування',
'Resistor R, MΩ:':'Резистор R, МОм:','Save R':'Зберегти R','Autozero A':'Автонуль A','Leakage limit, nA:':'Норма витоку, нА:','Save limit':'Зберегти норму',
'Leakage over time, nA (wheel: zoom, drag: pan, double-click: whole history)':'Витік у часі, нА (коліщатко: зум, тягнути: зсув, подвійний клік: уся історія)',
'Calibration: E (mV) → S (V)':'Калібрування: E (мВ) → S (В)',
'pH: reading vs true — green without leakage, dashed with leakage':'pH: показ vs справжній — зелена без витоку, пунктир з витоком',
'Block 2 · Electrode and pH':'Блок 2 · Електрод і рН',
'Reference pH (A, simulator) over time (wheel: zoom, drag: pan, double-click: whole history)':'рН опорний (A, імітатор) у часі (коліщатко: зум, тягнути: зсув, подвійний клік: уся історія)',
'Electrode equilibrium':'Рівновага електрода','Start':'Старт','Stop':'Стоп','Quiet band, pH:':'Смуга спокою, рН:','Pick limits on the chart':'Обрати межі на графіку',
'State':'Стан','Time since start':'Час від старту','Drift over 10 s':'Дрейф за 10 с','Time to equilibrium':'Час до рівноваги',
't63 (time constant)':'t63 (стала часу)','Equilibrium pH':'Рівноважний рН',
'Data and drift':'Дані і дрейф','Download CSV':'Завантажити CSV','Clear history':'Очистити історію','points in memory:':'точок у памʼяті:',
'within the limit':'в нормі','×{0} over':'×{0} понад','E={0} mV, S={1} V':'E={0} мВ, S={1} В',
'settling…':'стабілізується…','equilibrium':'рівновага','stopped':'зупинено','equilibrium (manual)':'рівновага (вручну)',
'click the START on the real pH chart':'клікніть ПОЧАТОК на графіку рН реальний','now click the END':'тепер клікніть КІНЕЦЬ',
'too few points in the selected window':'замало точок у вибраному вікні',
'waiting for calibration…':'чекаю калібрування…','waiting for data…':'чекаю дані…','no points in the window':'нема точок у вікні',
'pH reading':'pH показ','pH true':'pH справжній','true {0}':'справж {0}','reading {0}':'показ {0}','zoom':'зум',
'start':'початок','end':'кінець','{0} s':'{0} с','{0} s from the start':'{0} с від початку',
'Find networks':'Знайти мережі','Network (SSID):':'Мережа (SSID):','Password:':'Пароль:','Save and restart':'Зберегти і перезапустити',
'Forget the network':'Забути мережу','searching…':'шукаю…','— choose a network —':'— оберіть мережу —','no networks found':'мереж не знайдено',', open':', відкрита',
'Connected to «{0}» · {1} · signal {2} dBm':'Під\'єднано до «{0}» · {1} · сигнал {2} dBm',
'Access point «{0}» · {1} · could not connect to «{2}»':'Точка доступу «{0}» · {1} · не вдалося під\'єднатися до «{2}»',
'Access point «{0}» · {1} · no network saved':'Точка доступу «{0}» · {1} · мережу не задано',
'Connecting to «{0}»…':'Під\'єднуюсь до «{0}»…',
'Enter the network name (1-32 characters).':'Введіть назву мережі (1-32 символи).',
'The password is 8-63 characters, or empty for an open network.':'Пароль — 8-63 символи, або порожній для відкритої мережі.',
'Save «{0}» and restart the calibrator?':'Зберегти «{0}» і перезапустити прилад?',
'Forget the saved network and restart as an access point?':'Забути збережену мережу і перезапуститися точкою доступу?',
'Restarting… reconnect in ~20 s.':'Перезапуск… під\'єднайтеся знову за ~20 с.'
};
let LANG='en';
{ const auto=(navigator.language||'').toLowerCase().startsWith('uk')?'uk':'en';
  let l=null; try{ l=localStorage.getItem('lang'); }catch(e){}
  LANG=(l==='uk'||l==='en')?l:auto; }
function T(s,...a){ let r=(LANG==='uk'&&UA[s]!==undefined)?UA[s]:s;
  a.forEach((x,i)=>{ r=r.split('{'+i+'}').join(x); }); return r; }
// Never translated: the two-language paragraphs, network names, the version.
const I18N_SKIP='.i-en,.i-uk,#wst,#wlist,#ver,script,style';
const ORIG=new WeakMap();
function trNode(n){
  const p=n.parentElement; if(!p||p.closest(I18N_SKIP)) return;
  let o=ORIG.get(n); const cur=n.nodeValue;
  if(!o||cur!==o.shown) o={en:cur};            // text the page has just written is the English original
  const m=o.en.match(/^(\s*)([\s\S]*?)(\s*)$/), key=m[2].replace(/\s+/g,' ');
  const tr=(LANG==='uk'&&key&&UA[key]!==undefined)?m[1]+UA[key]+m[3]:o.en;
  o.shown=tr; ORIG.set(n,o);
  if(cur!==tr) n.nodeValue=tr;
}
function trTree(root){ const w=document.createTreeWalker(root,NodeFilter.SHOW_TEXT); let n; while((n=w.nextNode())) trNode(n); }
const i18nMo=new MutationObserver(recs=>{
  i18nMo.disconnect();
  recs.forEach(r=>{ if(r.type==='characterData') trNode(r.target);
    else r.addedNodes.forEach(n=>{ if(n.nodeType===3) trNode(n); else if(n.nodeType===1) trTree(n); }); });
  i18nObserve();
});
function i18nObserve(){ i18nMo.observe(document.body,{subtree:true,childList:true,characterData:true}); }
function setLang(l){ LANG=l; try{ localStorage.setItem('lang',l); }catch(e){} applyLang(); relang(); }
function applyLang(){
  document.documentElement.lang=LANG; document.title=T('pH calibrator');
  document.querySelectorAll('#lang button').forEach(b=>b.classList.toggle('on',b.dataset.l===LANG));
  i18nMo.disconnect(); trTree(document.body); i18nObserve();
}
document.querySelectorAll('#lang button').forEach(b=>b.addEventListener('click',e=>{ e.stopPropagation(); setLang(b.dataset.l); }));
// After a language switch, repaint at once everything built from numbers through T().
function relang(){ try{ renderInfo(); if(lastD){ fill(lastD); drawCal(lastD); drawPh(lastD); } drawTSAll(); }catch(e){} }

function fmt(x,d){return (x==null||isNaN(x))?'–':Number(x).toFixed(d)}
function fmtN(x){return (x==null||isNaN(x))?'–':(Math.abs(x)<0.1?Number(x).toFixed(3):Number(x).toFixed(1))}
function fmtL(x){if(x==null||isNaN(x))return '–';let v=Number(x);if(Math.abs(v)<0.005)v=0;return Math.abs(v)<1?v.toFixed(2):v.toFixed(1);}
let rEdited=false, nEdited=false;
document.getElementById('rin').addEventListener('input',()=>rEdited=true);
document.getElementById('nin').addEventListener('input',()=>nEdited=true);

const CS=getComputedStyle(document.documentElement);
const COL={line:CS.getPropertyValue('--line').trim(),dim:CS.getPropertyValue('--muted').trim(),
  grn:CS.getPropertyValue('--accent').trim(),amb:CS.getPropertyValue('--warn').trim(),
  bad:CS.getPropertyValue('--bad').trim(),blu:CS.getPropertyValue('--blue').trim(),fg:'#fff'};
const log=[]; const LOG_MAX=50000;   // the whole session: {t, pa (reference pH), ps (real pH), sv (S, V), il (leakage nA)}
let lastLogT=0; const LOG_EVERY=1000;  // the screen updates 4×/s, the history gets 1 point/s -> 50000 ≈ 14 h (a night)
const tsview={};                     // zoom window of a time chart (id -> {a,b} in ms); absent = whole history
const panS={};                       // drag (pan) state of a chart
let eq={run:false,t0:0,p0:null,samp:[],done:false,tset:0,t63:'—',pfin:0,tsAbs:0};  // automatic equilibrium measurement
const msel={mode:0,a:0,b:0};         // manual equilibrium limits on the chart (0=off, 1=waiting for start, 2=waiting for end)
let lastD=null; const maps={}, hover={};   // values shown on hover over the charts

function fit(cv){const dpr=window.devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;
  if(cv.width!=Math.round(w*dpr)||cv.height!=Math.round(h*dpr)){cv.width=Math.round(w*dpr);cv.height=Math.round(h*dpr);}
  const c=cv.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);return {c,w,h};}

function drawCal(d){
  const cv=document.getElementById('cCal');if(!cv)return;const {c,w,h}=fit(cv);
  c.clearRect(0,0,w,h);const x0=50,x1=w-12,y0=12,y1=h-24;
  c.strokeStyle=COL.line;c.lineWidth=1;c.beginPath();c.moveTo(x0,y0);c.lineTo(x0,y1);c.lineTo(x1,y1);c.stroke();
  c.font='11px system-ui';
  const Es=[d.e];if(d.h1)Es.push(d.e1);if(d.h2)Es.push(d.e2);
  let eMn=Math.min(...Es),eMx=Math.max(...Es);if(eMn===eMx){eMn-=50;eMx+=50;}
  const ep=(eMx-eMn)*0.15;eMn-=ep;eMx+=ep;
  const Ss=[d.s];if(d.h1)Ss.push(d.s1);if(d.h2)Ss.push(d.s2);
  if(d.valid){Ss.push(d.zero+d.gain/1000*eMn,d.zero+d.gain/1000*eMx);
    const dl=d.vin-d.e;Ss.push(d.zero+d.gain/1000*(eMn+dl),d.zero+d.gain/1000*(eMx+dl));}
  let sMn=Math.min(...Ss),sMx=Math.max(...Ss);if(sMn===sMx){sMn-=0.1;sMx+=0.1;}
  const spd=(sMx-sMn)*0.15;sMn-=spd;sMx+=spd;
  const X=e=>x0+(x1-x0)*(e-eMn)/(eMx-eMn),Y=s=>y1-(y1-y0)*(s-sMn)/(sMx-sMn);
  c.textAlign='right';
  for(let i=0;i<=2;i++){const s=sMn+(sMx-sMn)*i/2,yy=Y(s);
    c.globalAlpha=.35;c.strokeStyle=COL.line;c.beginPath();c.moveTo(x0,yy);c.lineTo(x1,yy);c.stroke();c.globalAlpha=1;
    c.fillStyle=COL.dim;c.fillText(s.toFixed(2),x0-4,yy+3);}
  c.textAlign='center';c.fillStyle=COL.dim;
  for(let i=0;i<=2;i++){const e=eMn+(eMx-eMn)*i/2;c.fillText(e.toFixed(0),X(e),y1+14);}
  if(d.valid){c.strokeStyle=COL.grn;c.lineWidth=2;c.beginPath();
    c.moveTo(X(eMn),Y(d.zero+d.gain/1000*eMn));c.lineTo(X(eMx),Y(d.zero+d.gain/1000*eMx));c.stroke();
    const dl=d.vin-d.e;c.setLineDash([5,4]);c.strokeStyle=COL.amb;c.lineWidth=1.8;c.beginPath();
    c.moveTo(X(eMn),Y(d.zero+d.gain/1000*(eMn+dl)));c.lineTo(X(eMx),Y(d.zero+d.gain/1000*(eMx+dl)));c.stroke();c.setLineDash([]);}
  c.fillStyle=COL.fg;
  if(d.h1){c.beginPath();c.arc(X(d.e1),Y(d.s1),3.5,0,7);c.fill();}
  if(d.h2){c.beginPath();c.arc(X(d.e2),Y(d.s2),3.5,0,7);c.fill();}
  c.fillStyle=COL.amb;c.beginPath();c.arc(X(d.e),Y(d.s),4.5,0,7);c.fill();
  maps.cCal={x0,x1,y0,y1,readout:function(px){
    const e=eMn+(eMx-eMn)*(px-x0)/(x1-x0);
    let my=null,sl=null;if(d.valid){sl=d.zero+d.gain/1000*e;my=Y(sl);}
    return {mx:X(e),my:my,lines:['E='+e.toFixed(0)+' '+T('mV'), sl!=null?('S='+sl.toFixed(3)+' '+T('V')):'']};}};
  drawHover('cCal',c,w,h,maps.cCal);
}

function drawPh(d){
  const cv=document.getElementById('cPh');if(!cv)return;const {c,w,h}=fit(cv);
  c.clearRect(0,0,w,h);const x0=36,x1=w-8,y0=8,y1=h-30;
  c.strokeStyle=COL.line;c.lineWidth=1;c.beginPath();c.moveTo(x0,y0);c.lineTo(x0,y1);c.lineTo(x1,y1);c.stroke();
  c.font='11px system-ui';c.fillStyle=COL.dim;c.textAlign='left';
  if(!d.valid){maps.cPh=null;c.fillText(T('waiting for calibration…'),x0+8,y0+16);return;}
  const p1=7-d.e1/59.16,p2=7-d.e2/59.16;
  let lo=Math.min(p1,p2),hi=Math.max(p1,p2);const pad=(hi-lo)*0.12||1;lo-=pad;hi+=pad;
  const err=(d.vin-d.e)/59.16;
  const X=p=>x0+(x1-x0)*(p-lo)/(hi-lo),Y=p=>y1-(y1-y0)*(p-lo)/(hi-lo);
  c.textAlign='right';
  for(let i=0;i<=4;i++){const p=lo+(hi-lo)*i/4;
    c.globalAlpha=.3;c.strokeStyle=COL.line;c.beginPath();c.moveTo(x0,Y(p));c.lineTo(x1,Y(p));c.stroke();c.globalAlpha=1;
    c.fillStyle=COL.dim;c.fillText(p.toFixed(1),x0-4,Y(p)+3);}
  c.textAlign='center';for(let i=0;i<=4;i++){const p=lo+(hi-lo)*i/4;c.fillText(p.toFixed(1),X(p),y1+14);}
  c.textAlign='left';c.fillText(T('pH reading'),4,y0+2);c.textAlign='right';c.fillText(T('pH true'),x1,y1+26);
  c.strokeStyle=COL.grn;c.lineWidth=2;c.beginPath();c.moveTo(X(lo),Y(lo));c.lineTo(X(hi),Y(hi));c.stroke();
  c.setLineDash([5,4]);c.strokeStyle=COL.amb;c.lineWidth=1.8;c.beginPath();c.moveTo(X(lo),Y(lo-err));c.lineTo(X(hi),Y(hi-err));c.stroke();c.setLineDash([]);
  const pt=7-d.e/59.16;
  c.fillStyle=COL.grn;c.beginPath();c.arc(X(pt),Y(pt),3.5,0,7);c.fill();
  c.fillStyle=COL.amb;c.beginPath();c.arc(X(pt),Y(pt-err),4.5,0,7);c.fill();
  maps.cPh={x0,x1,y0,y1,readout:function(px){
    const p=lo+(hi-lo)*(px-x0)/(x1-x0);
    return {mx:X(p),my:Y(p-err),lines:[T('true {0}',p.toFixed(2)),T('reading {0}',(p-err).toFixed(2))]};}};
  drawHover('cPh',c,w,h,maps.cPh);
}

// Generic chart of a value over time: whole history, zoom and pan.
// id - canvas, arr - the log array, key - field, color, unit - hover label (English, goes through T).
// marks - a number (one «equilibrium» marker) or an array of {t,label,color}.
// k2/col2/u2/msp2 - optional second curve with its own right axis, e.g. voltage S over pH.
function drawTS(id,arr,key,color,unit,marks,minSpan,k2,col2,u2,msp2){
  const cv=document.getElementById(id);if(!cv){maps[id]=null;return;}const {c,w,h}=fit(cv);
  const two=!!k2;c.clearRect(0,0,w,h);const x0=48,x1=w-(two?46:10),y0=12,y1=h-18;
  c.strokeStyle=COL.line;c.lineWidth=1;c.beginPath();c.moveTo(x0,y0);c.lineTo(x0,y1);c.lineTo(x1,y1);c.stroke();
  c.font='11px system-ui';c.textAlign='left';c.fillStyle=COL.dim;
  const all=arr.filter(p=>p[key]!=null&&!isNaN(p[key]));
  if(all.length<2){maps[id]=null;c.fillText(T('waiting for data…'),x0+8,y0+16);return;}
  const tF0=all[0].t,tFN=all[all.length-1].t;
  let va=tF0,vb=tFN;const vw=tsview[id];
  if(vw){va=Math.max(tF0,vw.a);vb=Math.min(tFN,vw.b);if(vb-va<500){va=tF0;vb=tFN;}}
  const zoomed=va>tF0||vb<tFN;
  let lo=0,hi=all.length-1;
  while(lo<all.length&&all[lo].t<va)lo++;if(lo>0)lo--;
  while(hi>0&&all[hi].t>vb)hi--;if(hi<all.length-1)hi++;
  const pts=all.slice(lo,hi+1);
  if(pts.length<2){maps[id]=null;c.fillText(T('no points in the window'),x0+8,y0+16);return;}
  function scale(kk,msp){let mn=1e9,mx=-1e9;for(const p of pts){const v=p[kk];if(v==null||isNaN(v))continue;if(v<mn)mn=v;if(v>mx)mx=v;}
    if(mn>mx){mn=0;mx=1;}if(mn===mx){mn-=0.1;mx+=0.1;}
    if(msp&&(mx-mn)<msp){const cc=(mn+mx)/2;mn=cc-msp/2;mx=cc+msp/2;}
    const rng=mx-mn,adec=rng<0.03?3:(rng<0.3?2:(rng<3?1:0)),dd=rng*0.12;mn-=dd;mx+=dd;
    return {mn,mx,adec,Y:v=>y1-(y1-y0)*(v-mn)/(mx-mn)};}
  const S1=scale(key,minSpan),mn=S1.mn,mx=S1.mx,adec=S1.adec,Y=S1.Y;
  const S2=two?scale(k2,msp2):null,Y2=S2?S2.Y:null;
  const sp=Math.max(1,vb-va),X=t=>x0+(x1-x0)*(t-va)/sp;
  c.textAlign='right';
  for(let i=0;i<=2;i++){const v=mn+(mx-mn)*i/2,yy=Y(v);
    c.globalAlpha=.35;c.strokeStyle=COL.line;c.beginPath();c.moveTo(x0,yy);c.lineTo(x1,yy);c.stroke();c.globalAlpha=1;
    let t=v.toFixed(adec);if(/^-0\.?0*$/.test(t))t=t.slice(1);
    c.fillStyle=two?color:COL.dim;c.fillText(t,x0-4,yy+3);}
  if(two){c.textAlign='left';
    for(let i=0;i<=2;i++){const v=S2.mn+(S2.mx-S2.mn)*i/2;c.fillStyle=col2;c.fillText(v.toFixed(S2.adec),x1+4,Y2(v)+3);}}
  c.fillStyle=COL.dim;c.textAlign='left';c.fillText(T('{0} s',Math.round((va-tF0)/1000)),x0+1,y1+14);
  c.textAlign='right';c.fillText(T('{0} s',Math.round((vb-tF0)/1000)),x1,y1+14);
  const stride=Math.max(1,Math.floor(pts.length/1800));
  if(two){c.strokeStyle=col2;c.lineWidth=1.4;c.globalAlpha=.9;c.beginPath();
    let st=false;for(let i=0;i<pts.length;i+=stride){const p=pts[i];if(p[k2]==null||isNaN(p[k2]))continue;const xx=X(p.t),yy=Y2(p[k2]);st?c.lineTo(xx,yy):c.moveTo(xx,yy);st=true;}
    c.stroke();c.globalAlpha=1;}
  c.strokeStyle=color;c.lineWidth=1.8;c.beginPath();
  for(let i=0;i<pts.length;i+=stride){const p=pts[i];i?c.lineTo(X(p.t),Y(p[key])):c.moveTo(X(p.t),Y(p[key]));}
  {const p=pts[pts.length-1];c.lineTo(X(p.t),Y(p[key]));}c.stroke();
  const mks=Array.isArray(marks)?marks:(marks?[{t:marks,label:'equilibrium',color:COL.fg}]:[]);
  for(const mk of mks){if(!mk||!(mk.t>=va&&mk.t<=vb))continue;const mxp=X(mk.t),cl=mk.color||COL.fg;
    c.strokeStyle=cl;c.lineWidth=1.5;c.setLineDash([4,3]);
    c.beginPath();c.moveTo(mxp,y0);c.lineTo(mxp,y1);c.stroke();c.setLineDash([]);
    const rt=mxp>x1-64;c.fillStyle=cl;c.font='10px system-ui';c.textAlign=rt?'right':'left';
    c.fillText(T(mk.label||''),mxp+(rt?-3:3),y0+10);}
  if(zoomed){c.fillStyle=COL.amb;c.font='10px system-ui';c.textAlign='right';c.fillText(T('zoom'),x1,y0+10);}
  maps[id]={x0,x1,y0,y1,va,vb,tF0,tFN,readout:function(px){
    const tt=va+(vb-va)*(px-x0)/(x1-x0);
    let bi=0,bd=1e9;for(let i=0;i<pts.length;i++){const dx=Math.abs(pts[i].t-tt);if(dx<bd){bd=dx;bi=i;}}
    const p=pts[bi],fromStart=Math.round((p.t-tF0)/1000);
    let vt=p[key].toFixed(Math.max(2,adec));if(/^-0\.?0*$/.test(vt))vt=vt.slice(1);
    const L=[vt+' '+T(unit)];
    if(two&&p[k2]!=null&&!isNaN(p[k2]))L.push(p[k2].toFixed(Math.max(2,S2.adec))+' '+T(u2));
    L.push(T('{0} s from the start',fromStart));
    return {mx:X(p.t),my:Y(p[key]),lines:L};}};
  drawHover(id,c,w,h,maps[id]);
}
function drawTSAll(){
  drawTS('cLeak',log,'il',COL.amb,'nA',0,0);
  drawTS('cPhA',log,'pa',COL.amb,'pH',0,0.1);
  const mk=[];
  if(msel.a)mk.push({t:msel.a,label:'start',color:COL.amb});
  if(msel.b)mk.push({t:msel.b,label:'end',color:COL.amb});
  if(!msel.a&&!msel.b&&eq.done&&eq.tsAbs)mk.push({t:eq.tsAbs,label:'equilibrium',color:COL.fg});
  drawTS('cPhS',log,'ps',COL.grn,'pH',mk,0.1,'sv',COL.blu,'V',0);
  const n=document.getElementById('npts');if(n)n.textContent=log.length;
}

// Electrode equilibrium: how fast the pH settles (works on the real pH from S).
function eqSet(id,t){document.getElementById(id).textContent=t;}
function eqStart(){eq={run:true,t0:Date.now(),p0:null,samp:[],done:false,tset:0,t63:'—',pfin:0,tsAbs:0};
  eqSet('eqstate','settling…');['eqt','eqdrift','eqset','eqt63'].forEach(i=>eqSet(i,'—'));eqSet('eqph','—');}
function eqStop(){eq.run=false;eqSet('eqstate', eq.done?'equilibrium':'stopped');}
function eqTick(ps){
  if(!eq.run||ps==null||isNaN(ps))return;
  const now=Date.now();if(eq.p0==null)eq.p0=ps;
  eq.samp.push({t:now,p:ps});const cut=now-300000;while(eq.samp.length&&eq.samp[0].t<cut)eq.samp.shift();
  const elapsed=(now-eq.t0)/1000;eqSet('eqt',elapsed.toFixed(0));
  const WIN=10000,w=eq.samp.filter(s=>s.t>=now-WIN);
  if(w.length<2)return;
  let mn=1e9,mx=-1e9;for(const s of w){if(s.p<mn)mn=s.p;if(s.p>mx)mx=s.p;}
  const ptp=mx-mn;eqSet('eqdrift',ptp.toFixed(3));
  if(eq.done)return;
  const band=Math.max(0.005,parseFloat(document.getElementById('eqband').value)||0.02);
  if(elapsed>=WIN/1000 && ptp<=band){
    eq.done=true;eq.tset=elapsed;eq.pfin=(mn+mx)/2;eq.tsAbs=now;
    eqSet('eqset',eq.tset.toFixed(0));eqSet('eqph',eq.pfin.toFixed(2));eqSet('eqstate','equilibrium');
    const target=eq.p0+0.63*(eq.pfin-eq.p0);
    for(const s of eq.samp){if((eq.pfin>=eq.p0&&s.p>=target)||(eq.pfin<eq.p0&&s.p<=target)){eq.t63=((s.t-eq.t0)/1000).toFixed(1);break;}}
    eqSet('eqt63',eq.t63);
  }else{eqSet('eqstate','settling…');}
}

// Manual variant: the limits of the transient are clicked on the «real pH» chart.
function selStart(){msel.mode=1;msel.a=0;msel.b=0;eq.done=false;
  eqSet('eqstate','click the START on the real pH chart');['eqt','eqdrift','eqset','eqt63'].forEach(i=>eqSet(i,'—'));eqSet('eqph','—');drawTSAll();}
function eqManual(){
  const a=Math.min(msel.a,msel.b),b=Math.max(msel.a,msel.b);
  const seg=log.filter(p=>p.ps!=null&&!isNaN(p.ps)&&p.t>=a&&p.t<=b);
  if(seg.length<2){eqSet('eqstate','too few points in the selected window');return;}
  const t0=seg[0].t,tEnd=seg[seg.length-1].t,p0=seg[0].ps;
  const tailFrom=tEnd-Math.min(10000,(tEnd-t0)*0.25);const tail=seg.filter(s=>s.t>=tailFrom);
  let sum=0,mn=1e9,mx=-1e9;for(const s of tail){sum+=s.ps;if(s.ps<mn)mn=s.ps;if(s.ps>mx)mx=s.ps;}
  const pfin=sum/tail.length,drift=mx-mn,tset=(tEnd-t0)/1000;
  const target=p0+0.63*(pfin-p0);let t63='—';
  for(const s of seg){if((pfin>=p0&&s.ps>=target)||(pfin<p0&&s.ps<=target)){t63=((s.t-t0)/1000).toFixed(1);break;}}
  eqSet('eqstate','equilibrium (manual)');eqSet('eqt',tset.toFixed(0));eqSet('eqdrift',drift.toFixed(3));
  eqSet('eqset',tset.toFixed(0));eqSet('eqt63',t63);eqSet('eqph',pfin.toFixed(2));
}

// Hover overlay: vertical line, a dot on the curve and a label with the value.
function drawHover(id,c,w,h,m){
  const hv=hover[id];if(!hv||!m||!m.readout)return;
  const px=Math.max(m.x0,Math.min(m.x1,hv.x));
  const r=m.readout(px);if(!r)return;
  c.save();
  c.globalAlpha=.6;c.strokeStyle=COL.dim;c.lineWidth=1;c.setLineDash([3,3]);
  c.beginPath();c.moveTo(r.mx,m.y0);c.lineTo(r.mx,m.y1);c.stroke();c.setLineDash([]);c.globalAlpha=1;
  if(r.my!=null){c.fillStyle=COL.fg;c.beginPath();c.arc(r.mx,r.my,3,0,7);c.fill();}
  const lines=r.lines.filter(Boolean);c.font='11px system-ui';c.textAlign='left';
  let tw=0;for(const t of lines)tw=Math.max(tw,c.measureText(t).width);
  const bw=tw+10,bh=lines.length*14+6;let bx=r.mx+8;if(bx+bw>m.x1)bx=r.mx-8-bw;if(bx<m.x0)bx=m.x0;
  c.fillStyle='rgba(20,20,20,.85)';c.strokeStyle=COL.line;c.lineWidth=1;
  c.beginPath();c.rect(bx,m.y0+4,bw,bh);c.fill();c.stroke();
  c.fillStyle=COL.fg;lines.forEach((t,i)=>c.fillText(t,bx+5,m.y0+4+14+i*14));
  c.restore();
}
function hookHover(id,redraw){
  const cv=document.getElementById(id);if(!cv)return;
  cv.addEventListener('pointermove',ev=>{const b=cv.getBoundingClientRect();hover[id]={x:ev.clientX-b.left,y:ev.clientY-b.top};redraw();});
  cv.addEventListener('pointerleave',()=>{hover[id]=null;redraw();});
}
// Time charts: hover + wheel zoom + drag to pan + double click = whole history.
function hookTS(id){
  const cv=document.getElementById(id);if(!cv)return;
  cv.addEventListener('pointermove',ev=>{const b=cv.getBoundingClientRect();const x=ev.clientX-b.left,y=ev.clientY-b.top;
    const ps=panS[id],m=maps[id];
    if(ps&&ps.active&&m){const sp=m.vb-m.va,pw=(m.x1-m.x0)||1;let dt=(ps.x-x)*sp/pw;
      let a=ps.a+dt,b=ps.b+dt;if(a<m.tF0){b+=m.tF0-a;a=m.tF0;}if(b>m.tFN){a-=b-m.tFN;b=m.tFN;}if(a<m.tF0)a=m.tF0;
      hover[id]=null;tsview[id]=(b>=m.tFN-1)?null:{a,b};drawTSAll();return;}
    hover[id]={x,y};drawTSAll();});
  cv.addEventListener('pointerleave',()=>{hover[id]=null;const ps=panS[id];if(ps)ps.active=false;drawTSAll();});
  cv.addEventListener('pointerdown',ev=>{const b=cv.getBoundingClientRect();const m=maps[id];if(!m)return;
    if(id==='cPhS'&&msel.mode){const x=Math.max(m.x0,Math.min(m.x1,ev.clientX-b.left));
      const tt=m.va+(m.vb-m.va)*(x-m.x0)/((m.x1-m.x0)||1);
      if(msel.mode===1){msel.a=tt;msel.b=0;msel.mode=2;eqSet('eqstate','now click the END');}
      else{msel.b=tt;msel.mode=0;eqManual();}
      drawTSAll();return;}
    panS[id]={active:true,x:ev.clientX-b.left,a:m.va,b:m.vb};try{cv.setPointerCapture(ev.pointerId)}catch(e){}});
  cv.addEventListener('pointerup',()=>{const ps=panS[id];if(ps)ps.active=false;});
  cv.addEventListener('dblclick',()=>{tsview[id]=null;drawTSAll();});
  cv.addEventListener('wheel',ev=>{ev.preventDefault();const m=maps[id];if(!m)return;
    const rc=cv.getBoundingClientRect();const frac=Math.min(1,Math.max(0,(ev.clientX-rc.left-m.x0)/((m.x1-m.x0)||1)));
    const tc=m.va+(m.vb-m.va)*frac,full=m.tFN-m.tF0;
    let span=Math.min(full,Math.max(1000,(m.vb-m.va)*(ev.deltaY<0?0.8:1.25)));
    let a=tc-span*frac,b=a+span;if(a<m.tF0){a=m.tF0;b=a+span;}if(b>m.tFN){b=m.tFN;a=b-span;}if(a<m.tF0)a=m.tF0;
    tsview[id]=(span>=full)?null:{a,b};drawTSAll();},{passive:false});
}
hookTS('cLeak');hookTS('cPhA');hookTS('cPhS');
hookHover('cCal',()=>{try{if(lastD)drawCal(lastD);}catch(e){}});
hookHover('cPh',()=>{try{if(lastD)drawPh(lastD);}catch(e){}});

// Export the whole history to a CSV file (the browser downloads it); autosave to browser storage.
function expCsv(){
  let s='t_ms,pH_real_S,S_V\n';
  for(const p of log){s+=p.t+','+(p.ps!=null?p.ps.toFixed(3):'')+','+(p.sv!=null?p.sv.toFixed(4):'')+'\n';}
  const bl=new Blob([s],{type:'text/csv'});const u=URL.createObjectURL(bl);
  const a=document.createElement('a');a.href=u;a.download='calibrator_'+new Date().toISOString().replace(/[:.]/g,'-')+'.csv';
  document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(u),1000);
}
function clrLog(){log.length=0;delete tsview.cLeak;delete tsview.cPhA;delete tsview.cPhS;try{localStorage.removeItem('calLog')}catch(e){}drawTSAll();}
function saveLog(){try{const n=log.length,st=Math.max(0,n-LOG_MAX);localStorage.setItem('calLog',JSON.stringify(log.slice(st)));}catch(e){}}
function loadLog(){try{const r=localStorage.getItem('calLog');if(r){const a=JSON.parse(r);if(Array.isArray(a))for(const p of a)log.push(p);}}catch(e){}}
loadLog();setInterval(saveLog,30000);
function togBlk(id){document.getElementById(id).classList.toggle('collapsed');saveUI();}
// Collapsed blocks (and other page toggles) survive a reload - in browser storage.
function saveUI(){try{const s={};document.querySelectorAll('.block').forEach(b=>s[b.id]=b.classList.contains('collapsed'));localStorage.setItem('calUI',JSON.stringify(s));}catch(e){}}
function loadUI(){try{const r=localStorage.getItem('calUI');if(!r)return;const s=JSON.parse(r);for(const id in s){const el=document.getElementById(id);if(el)el.classList.toggle('collapsed',!!s[id]);}}catch(e){}}
loadUI();

// Fill the readings from a /data packet (and again after a language switch).
function fill(d){
  const psv=d.valid?(7-d.vin/59.16):null;   // real pH (from board output S)
  e.textContent=fmt(d.e,1); s.textContent=fmt(d.s,3);
  phA.textContent=fmt(d.ph,2); phS.textContent=(psv!=null)?psv.toFixed(2):'–';
  noise.textContent=fmt(d.noise,2);
  cur.textContent=fmt(d.i,2);
  il.textContent=d.valid?fmtL(d.il):'–';
  vin.textContent=d.valid?fmt(d.vin,1):'–';
  document.getElementById('normlbl').textContent=fmtN(d.norm);
  const ov=document.getElementById('over');
  if(d.valid){
    const k=Math.abs(d.il)/((d.norm>0)?d.norm:1);
    if(k<=1){ov.textContent='within the limit';ov.style.color=COL.grn;}
    else{ov.textContent=T('×{0} over',(k>=10?k.toFixed(0):k.toFixed(1)));ov.style.color=(Math.abs(d.il)>=1.0?COL.bad:COL.amb);}
  }else{ov.textContent='–';ov.style.color='';}
  p1.textContent=d.h1?T('E={0} mV, S={1} V',fmt(d.e1,1),fmt(d.s1,3)):'—';
  p2.textContent=d.h2?T('E={0} mV, S={1} V',fmt(d.e2,1),fmt(d.s2,3)):'—';
  gain.textContent=d.valid?fmt(d.gain,3):'—';
  zero.textContent=d.valid?fmt(d.zero,3):'—';
  gaint.textContent=d.valid?fmt(Math.abs(d.gain),2):'–';
  document.getElementById('zerot').textContent=d.valid?fmt(d.zero,2):'–';
  if(d.valid){const dl=d.vin-d.e;document.getElementById('leakoff').textContent=fmt(dl,0);document.getElementById('leakph').textContent=fmt(Math.abs(dl)/59.16,1);}
  else{document.getElementById('leakoff').textContent='—';document.getElementById('leakph').textContent='—';}
  const v=document.getElementById('valid');
  v.textContent=d.valid?'ready':'not ready'; v.className=d.valid?'ok':'bad';
  document.getElementById('leakhint').style.display=d.valid?'none':'';
  if(!rEdited) document.getElementById('rin').value=fmt(d.r,2);
  if(!nEdited) document.getElementById('nin').value=fmtN(d.norm);
  return psv;
}

let busy=false;
async function refresh(){
  if(busy) return; busy=true;                         // do not pile up requests
  const ctrl=new AbortController(); const to=setTimeout(()=>ctrl.abort(),2000);
  try{
    const d=await (await fetch('/data',{signal:ctrl.signal})).json();
    lastD=d;
    const psv=fill(d);
    const now=Date.now();
    if(now-lastLogT>=LOG_EVERY){lastLogT=now;
      log.push({t:now,pa:d.ph,ps:psv,sv:d.s,il:(d.valid?d.il:null)}); if(log.length>LOG_MAX)log.shift();}
    eqTick(psv);
    drawCal(d); drawPh(d); drawTSAll();
  }catch(err){}
  finally{ clearTimeout(to); busy=false; }
}
async function post(u){await fetch(u,{method:'POST'});setTimeout(refresh,120)}
async function setR(){
  const meg=document.getElementById('rin').value;
  await fetch('/setr?meg='+encodeURIComponent(meg),{method:'POST'});
  rEdited=false; setTimeout(refresh,120);
}
async function setNorm(){
  const na=document.getElementById('nin').value;
  await fetch('/setnorm?na='+encodeURIComponent(na),{method:'POST'});
  nEdited=false; setTimeout(refresh,120);
}

// --- Wi-Fi: status, network scan, save. The calibrator restarts with the new network.
let info=null,ssidFilled=false;
function renderInfo(){
  const w=document.getElementById('wst');if(!info){w.textContent='–';return;}
  document.getElementById('ver').textContent='v'+info.ver;
  if(!info.ap&&info.ok) w.textContent=T('Connected to «{0}» · {1} · signal {2} dBm',info.ssid,info.ip,info.rssi);
  else if(!info.ap) w.textContent=T('Connecting to «{0}»…',info.ssid);
  else if(info.ssid) w.textContent=T('Access point «{0}» · {1} · could not connect to «{2}»',info.apssid,info.ip,info.ssid);
  else w.textContent=T('Access point «{0}» · {1} · no network saved',info.apssid,info.ip);
  document.getElementById('apban').style.display=info.ap?'':'none';
  const ws=document.getElementById('wssid');if(!ssidFilled&&info.ssid){ws.value=info.ssid;ssidFilled=true;}
}
async function loadInfo(){try{info=await (await fetch('/info')).json();renderInfo();}catch(e){}}
function openWifi(){const b=document.getElementById('blk3');b.classList.remove('collapsed');saveUI();
  b.scrollIntoView({behavior:'smooth'});document.getElementById('wssid').focus();}
let scanning=false;
async function scanWifi(){
  if(scanning)return; scanning=true;
  const sel=document.getElementById('wlist');sel.innerHTML='';
  const o=document.createElement('option');o.value='';o.textContent=T('searching…');sel.appendChild(o);
  try{
    for(let i=0;i<20;i++){                      // a scan takes a few seconds: poll once a second
      const r=await (await fetch('/scan')).json();
      if(!r.busy){
        sel.innerHTML='';const h=document.createElement('option');h.value='';
        h.textContent=r.n.length?T('— choose a network —'):T('no networks found');sel.appendChild(h);
        r.n.sort((a,b)=>b.r-a.r).forEach(n=>{const op=document.createElement('option');op.value=n.s;
          op.textContent=n.s+'  ('+n.r+' dBm'+(n.e?'':T(', open'))+')';sel.appendChild(op);});
        break;}
      await new Promise(f=>setTimeout(f,1000));
    }
  }catch(e){}
  scanning=false;
}
async function saveWifi(){
  const ss=document.getElementById('wssid').value.trim(),pw=document.getElementById('wpass').value;
  if(ss.length<1||ss.length>32){alert(T('Enter the network name (1-32 characters).'));return;}
  if(pw.length&&(pw.length<8||pw.length>63)){alert(T('The password is 8-63 characters, or empty for an open network.'));return;}
  if(!confirm(T('Save «{0}» and restart the calibrator?',ss)))return;
  try{const r=await fetch('/wifi',{method:'POST',body:new URLSearchParams({ssid:ss,pass:pw})});
    if(!r.ok){alert(await r.text());return;}}catch(e){}
  document.getElementById('wst').textContent=T('Restarting… reconnect in ~20 s.');
}
async function forgetWifi(){
  if(!confirm(T('Forget the saved network and restart as an access point?')))return;
  try{await fetch('/wififorget',{method:'POST'});}catch(e){}
  document.getElementById('wst').textContent=T('Restarting… reconnect in ~20 s.');
}

window.addEventListener('resize',()=>{try{drawTSAll();if(lastD){drawCal(lastD);drawPh(lastD);}}catch(e){}});
applyLang();
loadInfo();setInterval(loadInfo,5000);
setInterval(refresh,250); refresh();
</script>
</body></html>)HTML";

// JSON of the present state (runs in the AsyncTCP task - read only).
static void sendData(AsyncWebServerRequest* req) {
  char buf[512];
  snprintf(buf, sizeof(buf),
    "{\"e\":%.1f,\"s\":%.3f,\"ph\":%.2f,\"vin\":%.1f,\"il\":%.3f,\"noise\":%.2f,"
    "\"h1\":%d,\"e1\":%.1f,\"s1\":%.3f,\"h2\":%d,\"e2\":%.1f,\"s2\":%.3f,"
    "\"valid\":%d,\"gain\":%.3f,\"zero\":%.3f,\"r\":%.3f,\"norm\":%.3f,\"i\":%.2f}",
    g_E_mV, g_S_V, g_pH, g_Vin_mV, g_Ileak_nA, g_noise_mV,
    haveP1 ? 1 : 0, E1_mV, S1_V, haveP2 ? 1 : 0, E2_mV, S2_V,
    calValid ? 1 : 0, gainS * 1000.0f, zeroS, Rseries / 1.0e6f, leakNormNA, g_I_mA);
  req->send(200, "application/json", buf);
}

// Network status and version for the Wi-Fi block (the page polls every 5 s).
static void sendInfo(AsyncWebServerRequest* req) {
  String j = "{\"ver\":\"" CAL_VERSION "\",\"ap\":";
  j += apMode ? 1 : 0;
  j += ",\"ok\":";   j += staConnected ? 1 : 0;
  j += ",\"ssid\":\"";   j += jsonEsc(wifiSsid);
  j += "\",\"ip\":\"";  j += g_ip;
  j += "\",\"rssi\":"; j += staConnected ? WiFi.RSSI() : 0;
  j += ",\"apssid\":\""; j += AP_SSID; j += "\"}";
  req->send(200, "application/json", j);
}

// Network scan is asynchronous: the first request starts it, the next ones get {"busy":1}
// until it is done; the finished list is served once and deleted (the next request rescans).
static void sendScan(AsyncWebServerRequest* req) {
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_FAILED) { WiFi.scanNetworks(true); n = WIFI_SCAN_RUNNING; }
  if (n == WIFI_SCAN_RUNNING) { req->send(200, "application/json", "{\"busy\":1}"); return; }
  String j = "{\"n\":[";
  bool first = true;
  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    if (!s.length()) continue;                       // hidden network
    bool dup = false;                                // several access points of one network - one line
    for (int k = 0; k < i && !dup; k++) dup = (WiFi.SSID(k) == s);
    if (dup) continue;
    if (!first) j += ',';
    first = false;
    j += "{\"s\":\""; j += jsonEsc(s);
    j += "\",\"r\":"; j += WiFi.RSSI(i);
    j += ",\"e\":";   j += (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? 0 : 1;
    j += '}';
  }
  j += "]}";
  WiFi.scanDelete();
  req->send(200, "application/json", j);
}

// Web server routes (shared by STA and the fallback AP). The server listens at once.
static void startServerRoutes() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){ r->send(200, "text/html", INDEX_HTML); });
  server.on("/data", HTTP_GET, sendData);
  server.on("/p1",  HTTP_POST, [](AsyncWebServerRequest* r){ req_capP1  = true; r->send(200, "text/plain", "ok"); });
  server.on("/p2",  HTTP_POST, [](AsyncWebServerRequest* r){ req_capP2  = true; r->send(200, "text/plain", "ok"); });
  server.on("/calreset", HTTP_POST, [](AsyncWebServerRequest* r){ req_calRst = true; r->send(200, "text/plain", "ok"); });
  server.on("/zeroa", HTTP_POST, [](AsyncWebServerRequest* r){ req_zeroA = true; r->send(200, "text/plain", "ok"); });
  server.on("/setr", HTTP_POST, [](AsyncWebServerRequest* r){
    if (r->hasParam("meg")) {
      float meg = r->getParam("meg")->value().toFloat();
      if (meg > 0.001f) { req_Rval = meg * 1.0e6f; req_setR = true; }
    }
    r->send(200, "text/plain", "ok");
  });
  server.on("/setnorm", HTTP_POST, [](AsyncWebServerRequest* r){
    if (r->hasParam("na")) {
      float na = r->getParam("na")->value().toFloat();
      if (na > 0.0f) { req_NormVal = na; req_setNorm = true; }
    }
    r->send(200, "text/plain", "ok");
  });
  server.on("/info", HTTP_GET, sendInfo);
  server.on("/scan", HTTP_GET, sendScan);
  // New network: validated here, saved and restarted by loop() (NVS in one task).
  server.on("/wifi", HTTP_POST, [](AsyncWebServerRequest* r){
    String s = r->hasParam("ssid", true) ? r->getParam("ssid", true)->value() : String();
    String p = r->hasParam("pass", true) ? r->getParam("pass", true)->value() : String();
    if (s.length() < 1 || s.length() > 32) { r->send(400, "text/plain", "SSID: 1-32 characters"); return; }
    if (p.length() && (p.length() < 8 || p.length() > 63)) {
      r->send(400, "text/plain", "Password: 8-63 characters, or empty for an open network"); return;
    }
    strlcpy(req_ssid, s.c_str(), sizeof(req_ssid));
    strlcpy(req_pass, p.c_str(), sizeof(req_pass));
    req_wifiSave = true;
    r->send(200, "text/plain", "ok");
  });
  server.on("/wififorget", HTTP_POST, [](AsyncWebServerRequest* r){ req_wifiForget = true; r->send(200, "text/plain", "ok"); });
  server.onNotFound([](AsyncWebServerRequest* r){ r->redirect("/"); });   // captive portal
  server.begin();
  wifiOk = true;
}

// Own access point + captive DNS. AP_STA mode, so the network scan works from the page.
// STA attempts are stopped: their channel hopping while reconnecting breaks the access point.
static void startAP() {
  apMode = true;
  WiFi.setAutoReconnect(false);
  WiFi.disconnect();
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);      // SuperMini - see wifiBegin()
  IPAddress ip = WiFi.softAPIP();
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(DNS_PORT, "*", ip);
  MDNS.begin(HOSTNAME);
  MDNS.addService("http", "tcp", 80);
  snprintf(g_ip, sizeof(g_ip), "%s", ip.toString().c_str());
  connMsgUntil = millis() + 10000;         // show the AP address on the OLED for 10 s
  Serial.printf("Access point \"%s\" (password %s): http://%s/ - set the network in the Wi-Fi block\n",
                AP_SSID, AP_PASS, g_ip);
}

// Start Wi-Fi in the BACKGROUND - does NOT block the display. wifiTick() in loop() finishes it.
static void wifiBegin() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(HOSTNAME);
  // Allow channels 1..13 (EU routers may sit on 12/13).
  wifi_country_t ctry = { {'F','R',0}, 1, 13, 20, WIFI_COUNTRY_POLICY_MANUAL };
  esp_wifi_set_country(&ctry);
  // ESP32-C3 SuperMini: lower TX power - otherwise its antenna breaks the handshake.
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  startServerRoutes();   // the server listens at once and answers as soon as there is an IP
  if (!wifiSsid.length()) {                 // no network set -> access point at once
    Serial.println(F("Wi-Fi: no network set"));
    startAP();
    return;
  }
  WiFi.begin(wifiSsid.c_str(), wifiPass.length() ? wifiPass.c_str() : nullptr);
  wifiStartMs = millis();
  Serial.printf("Wi-Fi: connecting to \"%s\" in the background...\n", wifiSsid.c_str());
}

// Wi-Fi state: called every cycle from loop(), never blocks.
static void wifiTick() {
  if (apMode) return;                       // already in fallback mode
  if (WiFi.status() == WL_CONNECTED) {
    if (!staConnected) {
      staConnected = true;
      MDNS.begin(HOSTNAME);
      MDNS.addService("http", "tcp", 80);
      snprintf(g_ip, sizeof(g_ip), "%s", WiFi.localIP().toString().c_str());
      connMsgUntil = millis() + 4000;      // show the IP on the OLED for 4 s
      Serial.printf("Wi-Fi OK: http://%s/ or http://%s.local/\n", g_ip, HOSTNAME);
    }
    return;
  }
  // Timeout only for the FIRST connection. A network that was there and dropped comes back
  // through autoReconnect: a short router outage used to switch to the access point for good.
  if (!staConnected && millis() - wifiStartMs > 20000) {
    Serial.printf("Wi-Fi: could not connect to \"%s\" within 20 s\n", wifiSsid.c_str());
    startAP();
  }
}

// Saving / erasing the network from the page, and the restart. Separate from
// processWebRequests, which runs only with an ADS - the network must be settable without one.
static void processWifiRequests() {
  if (req_wifiSave) {
    req_wifiSave = false;
    saveWifi(req_ssid, req_pass);
    Serial.printf("Wi-Fi: network \"%s\" saved, restarting\n", req_ssid);
    restartAt = millis() + 1500;            // let the reply reach the browser
  }
  if (req_wifiForget) {
    req_wifiForget = false;
    saveWifi("", "");
    Serial.println(F("Wi-Fi: network forgotten, restarting as an access point"));
    restartAt = millis() + 1500;
  }
  if (restartAt && (int32_t)(millis() - restartAt) >= 0) {
    drawStatus("Restart", "", "");
    delay(100);
    ESP.restart();
  }
}

// Carry out the web requests (only in loop -> I2C/NVS in one task).
static void processWebRequests(float va, float vs) {
  if (req_zeroA)  { req_zeroA = false; offsetA += va; saveFloat("offA", offsetA);
                    Serial.printf("Autozero A (web): offsetA=%.5f V\n", offsetA); }
  if (req_capP1)  { req_capP1 = false; E1_mV = va * 1000.0f; S1_V = vs; haveP1 = true;
                    computeLeakCal(); saveLeak(); Serial.println(F("Point 1 taken")); }
  if (req_capP2)  { req_capP2 = false; E2_mV = va * 1000.0f; S2_V = vs; haveP2 = true;
                    computeLeakCal(); saveLeak(); Serial.println(F("Point 2 taken")); }
  if (req_calRst) { req_calRst = false; haveP1 = haveP2 = calValid = false;
                    E1_mV = S1_V = E2_mV = S2_V = 0; saveLeak();
                    Serial.println(F("Leakage calibration cleared")); }
  if (req_setR)   { req_setR = false; Rseries = req_Rval; saveLeak();
                    Serial.printf("R = %.3f MOhm\n", Rseries / 1.0e6f); }
  if (req_setNorm){ req_setNorm = false; leakNormNA = req_NormVal; saveLeak();
                    Serial.printf("Leakage limit = %.3f nA\n", leakNormNA); }
}

// ----------------------------- setup / loop ---------------------------------
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);         // do not stall the loop for 100 ms when no monitor is open
  delay(250);                       // briefly: let the I2C modules and USB CDC settle

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  loadCal();
  loadLeak();
  loadWifi();

  // Display and ADS FIRST, so the screen lights up as soon as power is applied
  oled.setI2CAddress(ADDR_OLED << 1);     // u8g2 takes an 8-bit address
  for (uint8_t i = 0; i < 5 && !(oledOk && adsOk); i++) {
    if (!oledOk && i2cPresent(ADDR_OLED)) { oled.begin(); oled.setContrast(255); oledOk = true; }
    if (!adsOk)  adsOk = ads.begin(ADDR_ADS, &Wire);
    if (!(oledOk && adsOk)) delay(120);
  }
  if (adsOk) ads.setDataRate(RATE_ADS1115_250SPS);

  // FIRST FRAME at once - the network state (the screen lights up right after power-on)
  if (oledOk) drawStatus("WiFi...", wifiSsid.c_str(), "connecting");
  Serial.printf("pH calibrator v%s\n", CAL_VERSION);
  i2cScan();
  if (!oledOk) Serial.println(F("OLED not found (0x3C)"));
  if (!adsOk)  Serial.println(F("ADS1115 not found (0x48)"));

  wifiBegin();   // Wi-Fi in the BACKGROUND - the screen is already on, loop carries on

  Serial.printf("offA=%.5f\n", offsetA);
  Serial.println(F("Ready. Commands: z=autozero A, Z=clear it, r=show calibration, v=input voltages."));
}

void loop() {
  static uint32_t last = 0;
  if (apMode) dnsServer.processNextRequest();   // captive DNS only in the fallback AP mode
  if (millis() - last < PERIOD_MS) return;
  last = millis();

  wifiTick();   // background Wi-Fi state (connection / fallback AP), never blocks
  processWifiRequests();

  // While connecting, and for a few seconds after, the OLED shows the network state
  bool netScreen = (!staConnected && !apMode) || (millis() < connMsgUntil);
  if (netScreen) {
    if (!staConnected && !apMode) drawStatus("WiFi...", wifiSsid.c_str(), "connecting");
    else drawStatus(staConnected ? "WiFi OK" : "AP mode", g_ip,
                    staConnected ? "calibrator.local" : AP_SSID);
  }

  // re-initialise if a module came alive / was plugged in later (both)
  if (!oledOk && i2cPresent(ADDR_OLED)) {
    oled.begin(); oled.setContrast(255); oledOk = true;
  }
  if (!adsOk) {
    adsOk = ads.begin(ADDR_ADS, &Wire);
    if (adsOk) ads.setDataRate(RATE_ADS1115_250SPS);
  }
  if (!adsOk) return;   // nothing to measure without the ADS

  float va = readA();
  float vs = readS();
  // Light smoothing (EMA) - removes last-digit flicker on the screen without slowing
  // the measurement. va/vs are smoothed from here on (screen, CSV, web, points).
  static float fVa = 0, fVs = 0; static bool fInit = false;
  if (!fInit) { fVa = va; fVs = vs; fInit = true; }
  // a real change -> at once (no lag); small flicker -> EMA
  fVa = (fabsf(va - fVa) > SNAP_A) ? va : fVa + DISP_ALPHA * (va - fVa);
  fVs = (fabsf(vs - fVs) > SNAP_S) ? vs : fVs + DISP_ALPHA * (vs - fVs);
  va = fVa; vs = fVs;

  g_I_mA = readI();   // board current - read before drawScreen so the screen is fresh

  if (!netScreen) drawScreen(va, vs);   // readings only when the network screen is not up

  // Update the values shared with the web interface (read by the AsyncTCP task)
  float e_mV = va * 1000.0f;
  g_E_mV = e_mV;
  g_S_V  = vs;
  g_pH   = phFromA(va);
  if (calValid && fabsf(gainS) > 1e-9f) {
    float vin_mV = (vs - zeroS) / gainS;                          // board input, mV (on the E scale)
    g_Vin_mV   = vin_mV;
    g_Ileak_nA = ((vin_mV - e_mV) * 1.0e-3f) / Rseries * 1.0e9f;  // (Vin-E)/R -> nA
  } else {
    g_Vin_mV = 0.0f; g_Ileak_nA = 0.0f;
  }

  // Carry out the deferred requests from the web page (I2C/NVS only here)
  processWebRequests(va, vs);

  // CSV on Serial: millis, A(mV), S(V), pH
  Serial.printf("%lu,%.1f,%.4f,%.2f\n", (unsigned long)millis(), va * 1000.0f, vs, phFromA(va));

  handleSerial(va);
}
