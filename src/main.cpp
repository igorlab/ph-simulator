// ============================================================================
//  Калібратор pH-плати на ESP32-C3 + ADS1115 + OLED SSD1306
// ============================================================================
//  Показує одночасно дві напруги:
//    A  — еталонна напруга імітатора (точка А з малюнка 12), біполярна (±)
//    S  — вихід S плати рН, яку калібруємо
//  Так одразу видно передавальну характеристику плати: скільки S дає на
//  відомий вхід A.
//
//  --------------------------------------------------------------------------
//  ПІДКЛЮЧЕННЯ (ADS1115 живиться від 3.3 В! I2C 5В спалив би ESP32-C3)
//  --------------------------------------------------------------------------
//    ESP32-C3           ->  SDA=GPIO5, SCL=GPIO6  (НЕ чіпати GPIO8/9 — strapping/BOOT)
//    OLED SSD1306 0x3C  ->  VCC=3V3, GND=GND, SDA=GPIO5, SCL=GPIO6
//    ADS1115     0x48   ->  VDD=3V3, GND=GND, SDA=GPIO5, SCL=GPIO6, ADDR->GND
//    Підтяжки I2C 4.7к до 3V3 (на модулях зазвичай уже стоять).
//
//  JST 3-pin = інтерфейс до плати рН: [ мінус=GND ][ +5В живлення плати ][ вихід S ]
//    +5В беремо з піна 5V ESP (тобто з USB). Земля плати = GND калібратора.
//
//  КАНАЛ A (мілівольти імітатора) — ДИФЕРЕНЦІЙНО AIN0-AIN1:
//    AIN0 <- повзунок (центральна ніжка потенціометра 10к)
//    AIN1 <- ліва ніжка потенціометра; ЦЕЙ ВУЗОЛ прив'язати до VMID (~1.65 В).
//    Вхід «плаваючий» (імітатор на своїй батарейці, ізольований). Диф-входи
//    ADS не ізольовані, тож без прив'язки до землі ADS вони «спливуть» і дадуть
//    сміття — саме тому ліва ніжка сидить на VMID. Результат A0-A1 = знакова
//    напруга між повзунком і лівою ніжкою; діапазон при 3.3 В ~ ±1.6 В.
//    ВАЖЛИВО: землю імітатора нікуди більше НЕ з'єднувати (лише ліва ніжка->VMID),
//    інакше закоротиш VMID.
//
//  КАНАЛ S (вихід плати рН, 0..5 В) — ОДНОПОЛЮСНО AIN2 відносно GND:
//    S -> дільник ÷2 (R3=R4=10к, низ на GND) -> AIN2. Так 0..5 В стає 0..2.5 В,
//    що безпечно для ADS при 3.3 В. У прошивці множимо назад на S_DIVIDER.
//
//  VMID (опора для AIN1): дільник 2×10к від 3V3 до GND + 0.1мкФ від середини
//  до GND (гасить кидки заряду входів ADS). Для макс. точності — буфер на ОП.
//
//  --------------------------------------------------------------------------
//  Serial (USB CDC, 115200): друкує CSV "millis,A_мВ,S_В,pH,S_сире_В,падіння_шунта_В,I_мА" + приймає команди:
//    z  — автонуль каналу A (запам'ятати поточне A як 0.000 В у NVS)
//    Z  — скинути автонуль
//    r  — показати збережені калібрування
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>
#include <Adafruit_ADS1X15.h>
#include <U8g2lib.h>   // дисплей через u8g2 (Adafruit_SSD1306 не заводила цю панель)

// Веб-інтерфейс: власна точка доступу + captive-DNS + асинхронний сервер.
// Асинхронний сервер працює у таску AsyncTCP, тож HTTP не блокує вимір/екран,
// а I2C (ADS/OLED) робиться ТІЛЬКИ в loop(); веб-обробники лише читають спільні
// значення й виставляють прапорці — жоден тракт не тримає інший.
#include <WiFi.h>
#include <esp_wifi.h>      // esp_wifi_set_country (дозволити канали 12/13 у EU)
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#if __has_include("wifi_secret.h")
#include "wifi_secret.h"   // локальні WIFI_SSID / WIFI_PASS; файл не в репозиторії
#else
#define WIFI_SSID ""
#define WIFI_PASS ""
#endif

// ----------------------------- Налаштування ---------------------------------
static const uint8_t PIN_SDA = 5;
static const uint8_t PIN_SCL = 6;

// Напруга живлення USB — вбудований АЦП ESP32-C3 (окремий від ADS1115, бо всі 4 входи ADS зайняті).
// GPIO4 = ADC1_CH4. Зовнішній дільник 15k(верх)+10k(низ): на піні 0.4·USB (2.0 В при 5.0 В).
static const uint8_t PIN_USB_ADC    = 4;
static const float   USB_DIV_DEFAULT = 2.5f;   // USB = Vpin·(15k+10k)/10k = Vpin·2.5

static const uint8_t ADDR_ADS  = 0x48;   // ADDR->GND
static const uint8_t ADDR_OLED = 0x3C;   // типова адреса 128x64

static const uint8_t  OLED_W = 128;
static const uint8_t  OLED_H = 64;

// Скільки вибірок усереднювати на канал. 25 при 250 SPS = рівно 100 мс = 5 періодів
// мережі 50 Гц -> наводка гаситься найкраще (важливо для високоомних вимірів, 100 МОм).
static const uint8_t  SAMPLES = 25;

// Період оновлення екрана/логу, мс (100 мс = ~10 оновлень/с)
static const uint32_t PERIOD_MS = 100;

// Адаптивний згладжувач показів A і S: помітна зміна проходить МИТТЄВО (без лагу),
// а дрібне миготіння молодшого розряду згладжується EMA. SNAP — поріг «одразу».
static const float DISP_ALPHA = 0.35f;   // сила згладжування дрібного шуму
static const float SNAP_A = 0.0004f;     // A: стрибок > 0.4 мВ -> одразу
static const float SNAP_S = 0.003f;      // S: стрибок > 3 мВ -> одразу

// Діапазони підсилення ADS1115 (обери під свої напруги):
//   A: ±1.6 В -> GAIN_TWO (±2.048 В, 62.5 мкВ/LSB). Якщо імітатор дає лише
//      малі мілівольти (напр. ±0.5 В) — можна GAIN_FOUR (±1.024) для роздільності.
//   S: після ÷2 це 0..2.5 В -> GAIN_ONE (±4.096 В) з запасом.
static const adsGain_t GAIN_A = GAIN_TWO;
static const adsGain_t GAIN_S = GAIN_ONE;

// Знак каналу A: -1.0, бо повзунок/ліва ніжка підключені навпаки (1.0 = як є).
static const float A_SIGN = -1.0f;

// Зовнішній дільник на вході S, послідовно: S -[10к]- AIN2 -[10к]- земля,
// тож AIN2 = S/2, у коді множимо назад на 2. Так вхід ADS не перевищує ~2.1 В
// навіть при S = 4.2 В (у межах живлення 3.3 В).
static const float S_DIVIDER = 2.0f;

// Вимір S: 0 -> однополюсний AIN2 відносно GND (S = 0..5 В через дільник ÷2).
//          1 -> диференційний AIN2-AIN3 (лишено на випадок біполярного S).
#define S_DIFFERENTIAL 0

// Струм споживання плати рН: низькосторонній шунт Rш у землі плати -> AIN3.
//   Уся зворотна земля плати йде через Rш; I = V(AIN3) / R_SHUNT_OHM.
//   Кілька мА при 10 Ом -> ~десятки мВ (у межах ±0.512 В).
static const adsGain_t GAIN_I  = GAIN_EIGHT;   // ±0.512 В (15.6 мкВ/LSB) -> до ~51 мА при 10 Ом
static const float     R_SHUNT_OHM = 10.0f;    // Ом (шунт у землі плати)
static const uint8_t   LEAK_CAL_S_REV = 1;     // ревізія 1: точки калібрування зняті за S із поправкою на шунт

// pH з мілівольтів каналу A (Нернст): pH7 при 0 мВ, "+" мВ -> кислота.
//   0 мВ=pH7, +59.16 мВ=pH6, +414 мВ≈pH0, -59.16 мВ=pH8, -414 мВ≈pH14.
static const float PH_ISO   = 7.0f;      // pH при 0 мВ
static const float PH_SLOPE = 59.16f;    // мВ на 1 pH (25°C)

// --- Веб / точка доступу / DNS ---
static const char* AP_SSID = "pH-Calibrator";   // ім'я мережі приладу
static const char* AP_PASS = "calib1234";        // пароль (>=8 символів)
static const char* HOSTNAME = "calibrator";      // mDNS -> http://calibrator.local
static const byte  DNS_PORT = 53;

// Послідовний резистор між джерелом імітатора і входом плати (для струму витоку).
// Значення можна змінити на веб-сторінці; типово 10 МОм. Зберігається в NVS.
static const float R_SERIES_DEFAULT = 10.0e6f;   // Ом

// Норма (допустимий) струм витоку, нА. Дашборд показує, у скільки разів витік
// вищий за неї. Змінюється на веб-сторінці, зберігається в NVS.
static const float LEAK_NORM_DEFAULT = 5.0f;   // нА

// ----------------------------- Об'єкти --------------------------------------
Adafruit_ADS1115 ads;
// u8g2, апаратний I2C на тих самих пінах (rotation, reset, clock=SCL, data=SDA)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE, PIN_SCL, PIN_SDA);
Preferences prefs;

bool  adsOk  = false;
bool  oledOk = false;

// Калібрування (у NVS, namespace "cal"): лише autozero A.
// Мертві множники scaleA/scaleS та зсув offsetS прибрано — стале значення
// sclS у NVS (від попередньої прошивки) двоїло показ S.
float offsetA = 0.0f;   // автонуль каналу A, В (команда 'z')

// --- Веб-сервер і DNS ---
DNSServer      dnsServer;
AsyncWebServer server(80);
AsyncEventSource events("/events");   // SSE: сервер пушить телеметрію (як у титраторі), клієнт — EventSource
bool           wifiOk = false;
bool           apMode = false;   // true = піднято власну точку доступу (запасний режим)
uint32_t       wifiStartMs = 0;  // коли почали під'єднання STA (для тайм-ауту у loop)
bool           staConnected = false;
char           g_ip[20]     = "";   // IP/адреса приладу для показу на OLED (ASCII)
uint32_t       connMsgUntil = 0;    // показувати екран стану мережі до цього часу, мс

// Останні виміри — оновлює loop(), читають веб-обробники (інший таск).
// 32-бітні float на RISC-V читаються/пишуться атомарно; volatile гарантує
// перечитування. Це лише телеметрія, блокування не потрібне.
volatile float g_E_mV    = 0.0f;   // A, мВ (зі знаком)
volatile float g_S_V     = 0.0f;   // S з поправкою на шунт (вихід плати відносно її GND), В
volatile float g_S_raw_V = 0.0f;   // S сире: вимір ADS відносно GND калібратора, В
volatile float g_shuntDrop_V = 0.0f; // I·Rш, віднятий від сирого S, В
volatile bool  g_shuntValid = false; // поправка застосована (струм у межах)
volatile float g_pH      = 7.0f;
volatile float g_Vin_mV  = 0.0f;   // розрахований вхід плати, мВ
volatile float g_Ileak_nA= 0.0f;   // струм витоку, нА
volatile float g_noise_mV= 0.0f;   // розкид S за цикл, мВ (СКВ)
volatile float g_I_mA    = 0.0f;   // струм споживання плати рН, мА (шунт на AIN3)
volatile float g_usb_V   = 0.0f;   // напруга живлення USB, В (вбудований АЦП GPIO4)
float g_usb_pin_mV = 0.0f;         // останнє сире читання на піні дільника, мВ (для калібр.)
float usbScale     = USB_DIV_DEFAULT;  // калібрований коеф. дільника USB (NVS "usbc")

// Калібрування витоку (NVS namespace "leak"): лінія S = zeroS + gainS*E.
bool  haveP1 = false, haveP2 = false;
float E1_mV = 0, S1_V = 0;         // точка 1 (вимикач замкнено)
float E2_mV = 0, S2_V = 0;         // точка 2
float gainS  = 0;                  // В на мВ (нахил S по E)
float zeroS  = 0;                  // В (S при E=0)
float Rseries = R_SERIES_DEFAULT;  // Ом
float leakNormNA = LEAK_NORM_DEFAULT;  // норма (допустимий) витік, нА
bool  calValid = false;            // чи порахована пряма (дві точки)

// Прапорці-запити від веб-обробників -> виконує loop() (щоб I2C/NVS робив 1 таск).
volatile bool  req_capP1  = false;
volatile bool  req_capP2  = false;
volatile bool  req_calRst = false;
volatile bool  req_zeroA  = false;
volatile bool  req_setR   = false;
volatile float req_Rval   = 0.0f;
volatile bool  req_setNorm = false;
volatile float req_NormVal = 0.0f;

// --- Буферне калібрування електрода (3 точки, NVS "phbuf") ---
// Калібруємо у мВ ЕЛЕКТРОДА (g_Vin_mV, відновлений з S), двосегментна мапа vin->pH.
// Передумова: пряма E->S (calValid) вже порахована — тоді vin має сенс.
static const float BUF_PH[3] = { 4.01f, 6.86f, 9.18f };  // кислий / середній / лужний
float bufVin[3]  = { 0, 0, 0 };          // зняті мВ електрода у кожному буфері
bool  bufHave[3] = { false, false, false };
bool  phCalValid = false;                // чи є всі 3 валідні точки
float phSlopeAcid = 0, phSlopeBase = 0;  // мВ/pH (кислий 4-7 / лужний 7-9 сегмент)
float phOffset_mV = 0;                   // мВ електрода при pH 7 (зсув електрода)
volatile float g_pH_real = 7.0f;         // pH з виходу плати (буфер.калібр. або ідеал. Нернст)
volatile int   req_bufCap = 0;           // 0=нема; 1..3 = зняти буфер (індекс+1)
volatile bool  req_bufClr = false;
float bufTempC = 25.0f;                   // температура буферів, °C (поправка номіналів)
// Температурні коеф. буферів (pH/°C, лінеаризація біля 25°C): 4.01 майже плоский,
// 6.86 фосфат, 9.18 борат — найчутливіший.
static const float BUF_TC[3] = { 0.0012f, -0.0025f, -0.0082f };
volatile bool  req_setTemp = false;
volatile float req_TempVal = 25.0f;
volatile bool  req_setBuf = false;       // відновлення буферів: вписати збережені vin (мВ) + T
volatile float req_bufV[3] = { 0, 0, 0 };
volatile float req_bufTcV = 25.0f;
volatile bool  req_calUsb = false;       // калібрувати дільник USB за відомою напругою
volatile float req_usbTrue = 5.0f;

// ----------------------------- Утиліти --------------------------------------
static void loadCal() {
  prefs.begin("cal", true);
  offsetA = prefs.getFloat("offA", 0.0f);
  prefs.end();
}

static void loadUsbCal() { prefs.begin("usbc", true);  usbScale = prefs.getFloat("k", USB_DIV_DEFAULT); prefs.end(); }
static void saveUsbCal() { prefs.begin("usbc", false); prefs.putFloat("k", usbScale);                   prefs.end(); }

static void saveFloat(const char* key, float v) {
  prefs.begin("cal", false);
  prefs.putFloat(key, v);
  prefs.end();
}

// Порахувати пряму S=zeroS+gainS*E з двох точок (якщо вони різні за E).
static void computeLeakCal() {
  calValid = false;
  if (haveP1 && haveP2 && fabsf(E2_mV - E1_mV) > 1e-3f) {
    gainS = (S2_V - S1_V) / (E2_mV - E1_mV);   // В/мВ
    zeroS = S1_V - gainS * E1_mV;              // В при E=0
    calValid = true;
  }
}

// Завантажити калібрування витоку з NVS (namespace "leak").
static void loadLeak() {
  prefs.begin("leak", true);
  haveP1 = prefs.getBool("h1", false);
  haveP2 = prefs.getBool("h2", false);
  E1_mV  = prefs.getFloat("E1", 0);  S1_V = prefs.getFloat("S1", 0);
  E2_mV  = prefs.getFloat("E2", 0);  S2_V = prefs.getFloat("S2", 0);
  Rseries= prefs.getFloat("R",  R_SERIES_DEFAULT);
  leakNormNA = prefs.getFloat("norm", LEAK_NORM_DEFAULT);
  const uint8_t sRev = prefs.getUChar("srev", 0);
  prefs.end();
  // Точки з прошивки до поправки зняті за сирим S. Змішувати їх із живим S без шунта не можна:
  // Vin і витік зіпсуються, тому потрібне одноразове нове двоточкове калібрування.
  if (sRev != LEAK_CAL_S_REV) haveP1 = haveP2 = false;
  computeLeakCal();
}

// Зберегти весь стан витоку (точки + R) в NVS.
static void saveLeak() {
  prefs.begin("leak", false);
  prefs.putBool("h1", haveP1);  prefs.putBool("h2", haveP2);
  prefs.putFloat("E1", E1_mV);  prefs.putFloat("S1", S1_V);
  prefs.putFloat("E2", E2_mV);  prefs.putFloat("S2", S2_V);
  prefs.putFloat("R",  Rseries);
  prefs.putFloat("norm", leakNormNA);
  prefs.putUChar("srev", LEAK_CAL_S_REV);
  prefs.end();
}

// Температурно-скоригований номінал буфера i (25°C-значення + коеф.*(T-25)).
static inline float bufPhAt(int i) { return BUF_PH[i] + BUF_TC[i] * (bufTempC - 25.0f); }

// Порахувати двосегментну мапу vin(мВ)->pH з трьох буферних точок.
// vin спадає зі зростанням pH (Нернст: vin=59.16*(7-pH)); пивот — середній буфер.
static void computePhCal() {
  phCalValid = false;
  if (!(bufHave[0] && bufHave[1] && bufHave[2])) return;
  float p0 = bufPhAt(0), p1 = bufPhAt(1), p2 = bufPhAt(2);
  float dphA = p1 - p0, dphB = p2 - p1;
  if (fabsf(dphA) < 0.1f || fabsf(dphB) < 0.1f) return;
  phSlopeAcid = (bufVin[1] - bufVin[0]) / dphA;   // ~ -59 мВ/pH
  phSlopeBase = (bufVin[2] - bufVin[1]) / dphB;
  if (fabsf(phSlopeAcid) < 1e-3f || fabsf(phSlopeBase) < 1e-3f) return;
  phOffset_mV = bufVin[1] + phSlopeAcid * (7.0f - p1);   // vin при pH7 (зсув)
  phCalValid = true;
}

// pH з мВ електрода за буферним калібруванням (двосегментно, з екстраполяцією).
static float phFromBuf(float vin_mV) {
  float p1 = bufPhAt(1);
  if (vin_mV >= bufVin[1])                                  // кислий бік (vin вищий)
    return p1 + (vin_mV - bufVin[1]) / phSlopeAcid;
  return p1 + (vin_mV - bufVin[1]) / phSlopeBase;           // лужний бік
}

static void loadPhCal() {
  prefs.begin("phbuf", true);
  for (int i = 0; i < 3; i++) {
    char k[6];
    snprintf(k, sizeof(k), "v%d", i); bufVin[i]  = prefs.getFloat(k, 0);
    snprintf(k, sizeof(k), "h%d", i); bufHave[i] = prefs.getBool(k, false);
  }
  bufTempC = prefs.getFloat("tc", 25.0f);
  prefs.end();
  computePhCal();
}

static void savePhCal() {
  prefs.begin("phbuf", false);
  for (int i = 0; i < 3; i++) {
    char k[6];
    snprintf(k, sizeof(k), "v%d", i); prefs.putFloat(k, bufVin[i]);
    snprintf(k, sizeof(k), "h%d", i); prefs.putBool(k, bufHave[i]);
  }
  prefs.putFloat("tc", bufTempC);
  prefs.end();
}

static void i2cScan() {
  Serial.println(F("I2C scan:"));
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  знайдено 0x%02X\n", a);
    }
  }
}

// Усереднений вимір каналу A (диференційний AIN0-AIN1), повертає вольти.
static float readA() {
  ads.setGain(GAIN_A);
  int32_t acc = 0;
  for (uint8_t i = 0; i < SAMPLES; i++) acc += ads.readADC_Differential_0_1();
  float v = A_SIGN * ads.computeVolts((int16_t)(acc / SAMPLES));   // інверсія полярності
  return v - offsetA;
}

// Усереднений вимір каналу S, повертає вольти. Побіжно рахує шум (СКВ) -> g_noise_mV.
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
  // СКВ у відліках -> вольти на вході плати (з урахуванням дільника)
  float sq = 0;
  for (uint8_t i = 0; i < SAMPLES; i++) { float d = s[i] - meanCnt; sq += d * d; }
  float stdCnt = sqrtf(sq / SAMPLES);
  g_noise_mV = ads.computeVolts((int16_t)lroundf(stdCnt)) * S_DIVIDER * 1000.0f;
  float v = ads.computeVolts((int16_t)lroundf(meanCnt));
  return v * S_DIVIDER;   // ×2: компенсуємо зовнішній дільник ÷2 на вході S
}

// Струм споживання плати рН: середнє AIN3 (однополюсно) — падіння на шунті Rш -> мА.
static float readI() {
  ads.setGain(GAIN_I);
  int32_t acc = 0;
  for (uint8_t i = 0; i < SAMPLES; i++) acc += ads.readADC_SingleEnded(3);
  float v = ads.computeVolts((int16_t)(acc / SAMPLES));   // падіння на шунті, В
  if (v < 0) v = 0;                                        // шум біля нуля
  return v / R_SHUNT_OHM * 1000.0f;                        // мА
}

// AIN2 міряє S відносно GND калібратора, а GND плати вище на I·Rш (її струм тече через шунт).
// Тож власний вихід плати = сире S мінус це додатне падіння.
static bool correctSForShunt(float sRaw_V, float current_mA, float& sBoard_V, float& shuntDrop_V) {
  if (!isfinite(sRaw_V) || !isfinite(current_mA) || current_mA < 0.0f) {
    sBoard_V = sRaw_V; shuntDrop_V = 0.0f;
    return false;
  }
  shuntDrop_V = current_mA * R_SHUNT_OHM / 1000.0f;
  sBoard_V = sRaw_V - shuntDrop_V;
  if (!isfinite(sBoard_V) || !isfinite(shuntDrop_V)) {
    sBoard_V = sRaw_V; shuntDrop_V = 0.0f;
    return false;
  }
  return true;
}

// Напруга живлення USB: вбудований АЦП ESP32-C3 на GPIO4 через зовнішній дільник 15k/10k.
// Канал A3 ADS зайнятий струмом плати, тож USB міряємо власним АЦП контролера.
static float readUsb() {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < 16; i++) acc += analogReadMilliVolts(PIN_USB_ADC);
  g_usb_pin_mV = acc / 16.0f;                   // мВ на піні (калібрований eFuse АЦП)
  return g_usb_pin_mV * usbScale / 1000.0f;     // В на вході USB
}

// Формат напруги зі знаком, 3 знаки після коми: "+1.234"
static void fmtV(char* buf, size_t n, float v) {
  snprintf(buf, n, "%+.3f", (double)v);
}

// Чи відповідає пристрій на адресі (u8g2.begin не перевіряє ACK, тож детектимо самі)
static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// ----------------------------- Екран ----------------------------------------
// pH з напруги каналу A (у вольтах) за Нернстом
static float phFromA(float va) {
  return PH_ISO - (va * 1000.0f) / PH_SLOPE;
}

static void drawScreen(float va, float vs) {
  if (!oledOk) return;
  char lph[24], nA[16], nS[16];
  snprintf(lph, sizeof(lph), "pH %.2f  %dmA", phFromA(va), (int)lroundf(g_I_mA));
  snprintf(nA,  sizeof(nA),  "%.1f", (va * 1000.0f));   // число зі знаком, велике
  snprintf(nS,  sizeof(nS),  "%.3f", vs);

  oled.clearBuffer();
  int x;

  // pH — великим, по центру (baseline трохи нижче, щоб не різало верх)
  oled.setFont(u8g2_font_helvB14_tr);
  x = (128 - oled.getStrWidth(lph)) / 2;
  oled.drawStr(x, 19, lph);

  // A: дрібна літера + великі цифри + дрібна одиниця "mV"
  oled.setFont(u8g2_font_helvB12_tr);
  oled.drawStr(0, 42, "A: ");
  x = oled.getStrWidth("A: ") + 4;
  oled.setFont(u8g2_font_helvB14_tr);
  oled.drawStr(x, 42, nA);
  x += oled.getStrWidth(nA) + 4;
  oled.setFont(u8g2_font_helvB12_tr);
  oled.drawStr(x, 42, "mV");

  // S: дрібна літера + великі цифри + дрібна одиниця "V"
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

// Екран стану мережі: 3 рядки по центру (латиниця — _tr не має кирилиці).
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

// ----------------------------- Serial-команди -------------------------------
static void handleSerial(float va) {
  if (!Serial.available()) return;
  char c = (char)Serial.read();
  switch (c) {
    case 'z':  // автонуль A: поточне A стає 0.000
      offsetA += va;               // va вже містить попередній offsetA -> додаємо
      saveFloat("offA", offsetA);
      Serial.printf("Автонуль A: offsetA=%.5f V\n", offsetA);
      break;
    case 'Z':  // скинути автонуль A
      offsetA = 0.0f;
      saveFloat("offA", offsetA);
      Serial.println(F("Автонуль A скинуто"));
      break;
    case 'r':  // показати калібрування
      Serial.printf("offA=%.5f\n", offsetA);
      break;
    default:
      break;
  }
}

// ----------------------------- Веб-сторінка ---------------------------------
static const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="uk"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Калібратор pH</title>
<style>
/* Палітра як на kh-station: сірий фон + зелений акцент */
:root{
  --bg:#2c2c2c; --card:#3b3b3b; --card2:#333; --line:#555;
  --fg:#fff; --dim:#b9b9b9; --muted:#8d8d8d;
  --ok:#3fb950; --bad:#f05252; --warn:#e8a33d; --accent:#3fb950; --blue:#58a6ff;
  color-scheme:dark;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.4 system-ui,-apple-system,"Segoe UI",Arial,sans-serif;padding:14px}
h1{font-size:18px;margin:0 0 12px}
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
input{background:#2c2c2c;border:1px solid var(--line);color:var(--fg);border-radius:6px;padding:8px;width:90px;font-size:14px}
.pt{font-size:13px;color:var(--muted);margin:2px 0}
.ok{color:var(--ok)}.bad{color:var(--bad)}
small{color:var(--dim)}
.graphs{display:flex;flex-wrap:wrap;gap:12px;margin:8px 0}
.gcol{flex:1 1 300px;min-width:240px;display:flex;flex-direction:column}
.gcap{font-size:12px;color:var(--dim);margin:0 0 4px}
.gcanvas{width:100%;height:240px;background:var(--card);border:1px solid var(--line);border-radius:8px;cursor:crosshair;touch-action:pan-y}
.rz{font-size:12px;padding:5px 9px;margin:6px 0 0;border-color:var(--warn);color:var(--warn)}
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
#cPh{width:100%;flex:1 1 auto;min-height:440px;background:var(--card);border:1px solid var(--line);border-radius:8px;cursor:crosshair}
.hdr{display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap}
.htl{display:flex;align-items:center}
.lang{display:flex;gap:4px}
.lang button{margin:0;padding:6px 11px;font-size:13px;min-width:44px}
.lang button.lsel{border-color:var(--accent);color:var(--accent);font-weight:700}
.dot{width:11px;height:11px;border-radius:50%;display:inline-block;margin-left:10px;background:var(--muted);transition:background .3s,box-shadow .3s}
.dot.on{background:var(--ok);box-shadow:0 0 7px var(--ok)}
.dot.off{background:var(--muted)}
.seg{font-size:12px;padding:5px 11px;margin:0 4px 0 0}
.seg.on{border-color:var(--accent);color:var(--accent);font-weight:700}
.gtog{cursor:pointer;user-select:none}
.gcol.collapsed .gbody{display:none}
.gcol.collapsed .car{transform:rotate(-90deg)}
</style></head><body>
<div class="hdr"><div class="htl"><h1 data-t="title">Калібратор pH</h1><span id="online" class="dot off"></span></div>
  <div class="lang"><button id="lua" class="lsel" onclick="setLang('ua')">UA</button><button id="len" onclick="setLang('en')">EN</button></div></div>

<h2 data-t="h.cur">Поточні покази</h2>
<div class="grid">
  <div class="tile"><div class="k" data-t="k.A">A (імітатор)</div><div class="v"><span id="e">–</span> <span class="u" data-t="u.mv">мВ</span> · <span class="u" data-t="u.ph">рН</span> <b id="phA" class="camb">–</b></div></div>
  <div class="tile"><div class="k" data-t="k.S">S (вихід плати, з поправкою шунта)</div><div class="v"><span id="s">–</span> <span class="u" data-t="u.v">В</span> · <span class="u" data-t="u.ph">рН</span> <b id="phS" class="cgrn">–</b></div></div>
  <div class="tile"><div class="k" data-t="k.sraw">S сире / падіння шунта</div><div class="v"><span id="sraw">–</span> / <span id="sdrop">–</span> <span class="u" data-t="u.v">В</span></div></div>
  <div class="tile"><div class="k" data-t="k.noise">Шум S</div><div class="v"><span id="noise">–</span> <span class="u" data-t="u.mv">мВ</span></div></div>
  <div class="tile"><div class="k" data-t="k.cur">Струм плати</div><div class="v"><span id="cur">–</span> <span class="u" data-t="u.ma">мА</span></div></div>
  <div class="tile"><div class="k" data-t="k.usb">Живлення USB</div><div class="v"><span id="usb">–</span> <span class="u" data-t="u.v">В</span> <button class="g" style="padding:2px 8px;font-size:12px;margin:0 0 0 6px" onclick="calUsb()" data-t="btn.calusb">калібр.</button></div></div>
  <div class="tile"><div class="k" data-t="k.gaint">Підсилення (нуль <span id="zerot">–</span> В)</div><div class="v"><span id="gaint">–</span> <span class="u" data-t="u.vv">В/В</span></div></div>
</div>

<section class="block" id="blk1">
<h2 onclick="togBlk('blk1')" data-t="h.blk1"><span class="car">▾</span> Блок 1 · Витік плати</h2>
<div class="bbody">
<div class="grid">
  <div class="tile leak"><div class="k" data-t="k.il">Струм витоку</div><div class="v"><span id="il">–</span> <span class="u" data-t="u.na">нА</span></div></div>
  <div class="tile"><div class="k" data-t="k.over">Понад норму (норма <span id="normlbl">–</span> нА)</div><div class="v" id="over">–</div></div>
  <div class="tile"><div class="k" data-t="k.vin">Вхід плати (розрах.)</div><div class="v"><span id="vin">–</span> <span class="u" data-t="u.mv">мВ</span></div></div>
</div>
<small id="leakhint" data-t="leakhint">Для розрахунку спершу зробіть калібрування двома точками нижче.</small>

<h3 data-t="h.calzero">Калібрування (кнопка НАТИСНУТА, 10 МОм закорочено)</h3>
<p class="pt" data-t="p.e"><b>E</b> — це напруга імітатора, та сама, що вгорі показана як <b>A</b> (у мВ); ви задаєте її підстроєчним резистором. Натиснута кнопка закорочує послідовний резистор 10 МОм (прибирає його з кола), тож на вхід плати йде рівно E — це дає чисту опорну точку. Для виміру витоку кнопку відпустіть.</p>
<p class="pt" data-t="p.steps"><b>Крок 1.</b> Виставте резистором якусь напругу E (дивіться на A вгорі) → «Зняти точку 1».<br>
<b>Крок 2.</b> Виставте помітно іншу E (що далі від першої, то точніше) → «Зняти точку 2».<br>
Прилад побудує пряму «E → S» плати (підсилення і нуль), і після цього рахуватиме струм витоку.</p>
<div class="pt" data-t="lbl.p1">Точка 1: <span id="p1">—</span></div>
<div class="pt" data-t="lbl.p2">Точка 2: <span id="p2">—</span></div>
<div class="pt" data-t="s.line">Пряма: підсилення <b><span id="gain">—</span></b> В/В, нуль <b><span id="zero">—</span></b> В — <span id="valid" class="bad">не готово</span></div>
<div class="pt" data-t="s.leakoff">З витоком вхід зсунуто на <b><span id="leakoff">—</span></b> мВ → похибка <b><span id="leakph">—</span></b> pH (пунктир на графіку).</div>
<div class="row">
  <button class="p" onclick="post('/p1')" data-t="btn.p1">Зняти точку 1</button>
  <button class="p" onclick="post('/p2')" data-t="btn.p2">Зняти точку 2</button>
  <button class="g" onclick="postC('/calreset','cfm.cal')" data-t="btn.calreset">Скинути калібрування</button>
</div>
<div class="row">
  <label data-t="lbl.R">Резистор R, МОм:</label>
  <input id="rin" type="number" step="0.1" min="0.1">
  <button onclick="setR()" data-t="btn.saveR">Зберегти R</button>
  <button class="g" onclick="post('/zeroa')" data-t="btn.zeroa">Автонуль A</button>
  <label data-t="lbl.norm">Норма витоку, нА:</label>
  <input id="nin" type="number" step="0.001" min="0.001">
  <button onclick="setNorm()" data-t="btn.savenorm">Зберегти норму</button>
</div>

<div class="graphs">
  <div class="gcol" style="flex:3 1 0">
    <div class="gcap" data-t="gcap.leak">Витік у часі, нА (протягни рамку: ширина — зум X, висота — зум Y; щипок на дотику; подвійний клік — скинути)</div><canvas id="cLeak" class="gcanvas"></canvas><button class="rz" id="rz_cLeak" onclick="zoomReset('cLeak')" data-t="btn.rz" hidden>скинути зум</button>
    <div class="gcap" style="margin-top:10px" data-t="gcap.cal">Калібрування: E (мВ) → S (В)</div><canvas id="cCal" class="gcanvas"></canvas>
  </div>
  <div class="gcol" style="flex:2 1 0;max-width:420px">
    <div class="gcap" data-t="gcap.ph">pH: показ vs справжній — зелена без витоку, пунктир з витоком</div><canvas id="cPh"></canvas>
  </div>
</div>
</div>
</section>

<section class="block" id="blk2">
<h2 onclick="togBlk('blk2')" data-t="h.blk2"><span class="car">▾</span> Блок 2 · Електрод і рН</h2>
<div class="bbody">
<div class="graphs"><div class="gcol" id="gcPhA"><div class="gcap gtog" onclick="togGraph('gcPhA')"><span class="car">▾</span> <span data-t="gcap.phA">рН опорний (A, імітатор) у часі (протягни рамку: ширина — зум X, висота — зум Y; щипок; подвійний клік — скинути)</span></div><div class="gbody"><canvas id="cPhA" class="gcanvas"></canvas><button class="rz" id="rz_cPhA" onclick="zoomReset('cPhA')" data-t="btn.rz" hidden>скинути зум</button></div></div></div>
<div class="row"><button class="p" onclick="measStart()" data-t="btn.meas">Почати вимір</button>
  <span class="hint" data-t="hint.meas">після калібрування — чистий pH від цієї мітки (калібрувальні занурення не показуються); до калібрування показується напруга S</span></div>
<div class="graphs"><div class="gcol"><div class="gcap" id="cPhSCap">рН реальний · вимір</div>
<div class="row" style="margin:2px 0 6px"><span class="hint" data-t="lbl.show">будувати:</span><button class="seg on" id="sg_ph" onclick="setPhSMode('ph')" data-t="seg.ph">рН</button><button class="seg" id="sg_sv" onclick="setPhSMode('sv')" data-t="seg.sv">S, В</button><button class="seg" id="sg_mv" onclick="setPhSMode('mv')" data-t="seg.mv">мВ електрода</button></div>
<canvas id="cPhS" class="gcanvas"></canvas><button class="rz" id="rz_cPhS" onclick="zoomReset('cPhS')" data-t="btn.rz" hidden>скинути зум</button></div></div>

<h3 data-t="h.bufcal">Калібрування електрода (3 буфери)</h3>
<p class="pt" data-t="p.bufcal">Відображає вихід плати у <b>справжній pH</b> за трьома буферами (двосегментно: кислий 4–7, лужний 7–9). Передумови: вгорі має бути готове калібрування плати (пряма E→S), і плата живлена від робочого джерела. Занурте електрод у буфер, дочекайтесь рівноваги (нижче) → зніміть точку.</p>
<div class="row">
  <label data-t="lbl.bt">Температура буферів, °C:</label>
  <input id="btin" type="number" step="0.5" value="25">
  <button class="g" onclick="setTemp()" data-t="btn.setT">Застосувати T</button>
  <span class="hint" data-t="hint.bt">поправляє номінали (особливо лужний 9,18)</span>
</div>
<div class="grid">
  <div class="tile"><div class="k"><span data-t="k.buf0">pH 4,01 · кислий</span> <span id="pc0" class="u"></span></div><div class="v"><span id="bv0">–</span> <span class="u" data-t="u.mv">мВ</span> <b id="bs0" class="camb">–</b></div></div>
  <div class="tile"><div class="k"><span data-t="k.buf1">pH 6,86 · середній</span> <span id="pc1" class="u"></span></div><div class="v"><span id="bv1">–</span> <span class="u" data-t="u.mv">мВ</span> <b id="bs1" class="camb">–</b></div></div>
  <div class="tile"><div class="k"><span data-t="k.buf2">pH 9,18 · лужний</span> <span id="pc2" class="u"></span></div><div class="v"><span id="bv2">–</span> <span class="u" data-t="u.mv">мВ</span> <b id="bs2" class="camb">–</b></div></div>
</div>
<div class="row">
  <button class="p" onclick="post('/bufcap?i=0')" data-t="btn.cap0">Зняти 4,01</button>
  <button class="p" onclick="post('/bufcap?i=1')" data-t="btn.cap1">Зняти 6,86</button>
  <button class="p" onclick="post('/bufcap?i=2')" data-t="btn.cap2">Зняти 9,18</button>
  <button class="g" onclick="postC('/bufclr','cfm.buf')" data-t="btn.bufclr">Скинути</button>
</div>
<div class="grid">
  <div class="tile"><div class="k" data-t="k.state">Стан</div><div class="v" id="pcstate">–</div></div>
  <div class="tile"><div class="k" data-t="k.slopeA">Нахил кислий</div><div class="v"><span id="psa">–</span> <span class="u">%</span></div></div>
  <div class="tile"><div class="k" data-t="k.slopeB">Нахил лужний</div><div class="v"><span id="psb">–</span> <span class="u">%</span></div></div>
  <div class="tile"><div class="k" data-t="k.off7">Зсув при pH7</div><div class="v"><span id="pof">–</span> <span class="u" data-t="u.mv">мВ</span></div></div>
</div>
<p class="hint" data-t="hint.health">Добрий електрод: нахил <b>92–103 %</b>, зсув <b>&lt; ±30 мВ</b>. Поза цим — мити/міняти. Коли калібрування готове, «рН реальний» рахується за буферами (інакше — ідеальний Нернст).</p>
<div class="gcap" data-t="gcap.calph">Калібрування електрода: pH → мВ — <b style="color:var(--accent)">вимір</b> (двосегментно) vs <b style="color:var(--warn)">ідеал Нернста −59,16</b></div>
<canvas id="cCalPh" class="gcanvas" style="height:300px;cursor:default"></canvas>

<h3 data-t="h.eq">Рівновага електрода</h3>
<p class="pt" data-t="p.eq">Як швидко встановлюється рН після занурення електрода або зміни розчину. Орієнтир — <b>рН реальний</b> (з S). Натисніть «Старт» одразу після занурення; прилад стежить за дрейфом і оголошує рівновагу, коли коливання за 10 секунд спадуть у межі смуги спокою.</p>
<div class="row">
  <button class="p" onclick="eqStart()" data-t="btn.eqstart">Старт</button>
  <button class="g" onclick="eqStop()" data-t="btn.eqstop">Стоп</button>
  <label data-t="lbl.eqband">Смуга спокою, рН:</label>
  <input id="eqband" type="number" step="0.005" min="0.005" value="0.02">
  <button class="g" onclick="selStart()" data-t="btn.sel">Обрати межі на графіку</button>
</div>
<p class="hint" data-t="hint.eq2">Додатково, без очікування: натисніть «Обрати межі на графіку» і клікніть на графіку «рН реальний» <b>початок</b> і <b>кінець</b> перехідного процесу — прилад порахує час, дрейф, t63 і рівноважний рН саме за цим відрізком.</p>
<div class="grid">
  <div class="tile"><div class="k" data-t="k.state">Стан</div><div class="v" id="eqstate">—</div></div>
  <div class="tile"><div class="k" data-t="k.eqt">Час від старту</div><div class="v"><span id="eqt">—</span> <span class="u" data-t="u.s">с</span></div></div>
  <div class="tile"><div class="k" data-t="k.eqdrift">Дрейф за 10 с</div><div class="v"><span id="eqdrift">—</span> <span class="u" data-t="u.ph">рН</span></div></div>
  <div class="tile"><div class="k" data-t="k.eqset">Час до рівноваги</div><div class="v"><span id="eqset">—</span> <span class="u" data-t="u.s">с</span></div></div>
  <div class="tile"><div class="k" data-t="k.eqt63">t63 (стала часу)</div><div class="v"><span id="eqt63">—</span> <span class="u" data-t="u.s">с</span></div></div>
  <div class="tile"><div class="k" data-t="k.eqph">Рівноважний рН</div><div class="v" id="eqph">—</div></div>
</div>

<h3 data-t="h.data">Дані і дрейф</h3>
<p class="hint" data-t="hint.data">Історія живе <b>у браузері, не у флеші приладу</b>: запис 1 точка/с, буфер на ~14 год (ніч). Не закривайте вкладку і не давайте компʼютеру заснути. Дані ще й самі осідають у памʼяті браузера і переживають перезавантаження сторінки. Для архіву натисніть «Завантажити CSV».</p>
<div class="row">
  <button class="g" id="pauseBtn" onclick="togPause()">Пауза</button>
  <button class="p" onclick="expCsv()" data-t="btn.csv">Завантажити CSV</button>
  <button class="g" onclick="if(confirm(T('cfm.hist')))clrLog()" data-t="btn.clr">Очистити історію</button>
  <span class="hint"><span data-t="hint.npts">точок у памʼяті:</span> <b id="npts">0</b></span>
</div>
</div>
</section>

<script>
// --- Двомовний інтерфейс (UA/EN). Статичні елементи мають data-t; динамічні рядки йдуть через T(). ---
const L={ua:{
 'title':'Калібратор pH','h.cur':'Поточні покази',
 'k.A':'A (імітатор)','k.S':'S (вихід плати, з поправкою шунта)','k.sraw':'S сире / падіння шунта','k.noise':'Шум S','k.cur':'Струм плати',
 'k.usb':'Живлення USB','btn.calusb':'калібр.','pr.usb':'Справжня напруга USB, В (виміряй тестером):',
 'k.gaint':'Підсилення (нуль <span id="zerot">–</span> В)',
 'h.blk1':'<span class="car">▾</span> Блок 1 · Витік плати',
 'k.il':'Струм витоку','k.over':'Понад норму (норма <span id="normlbl">–</span> нА)','k.vin':'Вхід плати (розрах.)',
 'leakhint':'Для розрахунку спершу зробіть калібрування двома точками нижче.',
 'h.calzero':'Калібрування (кнопка НАТИСНУТА, 10 МОм закорочено)',
 'p.e':'<b>E</b> — це напруга імітатора, та сама, що вгорі показана як <b>A</b> (у мВ); ви задаєте її підстроєчним резистором. Натиснута кнопка закорочує послідовний резистор 10 МОм (прибирає його з кола), тож на вхід плати йде рівно E — це дає чисту опорну точку. Для виміру витоку кнопку відпустіть.',
 'p.steps':'<b>Крок 1.</b> Виставте резистором якусь напругу E (дивіться на A вгорі) → «Зняти точку 1».<br><b>Крок 2.</b> Виставте помітно іншу E (що далі від першої, то точніше) → «Зняти точку 2».<br>Прилад побудує пряму «E → S» плати (підсилення і нуль), і після цього рахуватиме струм витоку.',
 'lbl.p1':'Точка 1: <span id="p1">—</span>','lbl.p2':'Точка 2: <span id="p2">—</span>',
 's.line':'Пряма: підсилення <b><span id="gain">—</span></b> В/В, нуль <b><span id="zero">—</span></b> В — <span id="valid" class="bad">не готово</span>',
 's.leakoff':'З витоком вхід зсунуто на <b><span id="leakoff">—</span></b> мВ → похибка <b><span id="leakph">—</span></b> pH (пунктир на графіку).',
 'btn.p1':'Зняти точку 1','btn.p2':'Зняти точку 2','btn.calreset':'Скинути калібрування',
 'lbl.R':'Резистор R, МОм:','btn.saveR':'Зберегти R','btn.zeroa':'Автонуль A','lbl.norm':'Норма витоку, нА:','btn.savenorm':'Зберегти норму',
 'gcap.leak':'Витік у часі, нА (протягни рамку: ширина — зум X, висота — зум Y; щипок на дотику; подвійний клік — скинути)',
 'gcap.cal':'Калібрування: E (мВ) → S (В)','gcap.ph':'pH: показ vs справжній — зелена без витоку, пунктир з витоком','btn.rz':'скинути зум',
 'h.blk2':'<span class="car">▾</span> Блок 2 · Електрод і рН',
 'gcap.phA':'рН опорний (A, імітатор) у часі (протягни рамку: ширина — зум X, висота — зум Y; щипок; подвійний клік — скинути)',
 'btn.meas':'Почати вимір',
 'hint.meas':'після калібрування — чистий pH від цієї мітки (калібрувальні занурення не показуються); до калібрування показується напруга S',
 'h.bufcal':'Калібрування електрода (3 буфери)',
 'p.bufcal':'Відображає вихід плати у <b>справжній pH</b> за трьома буферами (двосегментно: кислий 4–7, лужний 7–9). Передумови: вгорі має бути готове калібрування плати (пряма E→S), і плата живлена від робочого джерела. Занурте електрод у буфер, дочекайтесь рівноваги (нижче) → зніміть точку.',
 'lbl.bt':'Температура буферів, °C:','btn.setT':'Застосувати T','hint.bt':'поправляє номінали (особливо лужний 9,18)',
 'k.buf0':'pH 4,01 · кислий','k.buf1':'pH 6,86 · середній','k.buf2':'pH 9,18 · лужний',
 'btn.cap0':'Зняти 4,01','btn.cap1':'Зняти 6,86','btn.cap2':'Зняти 9,18','btn.bufclr':'Скинути',
 'k.state':'Стан','k.slopeA':'Нахил кислий','k.slopeB':'Нахил лужний','k.off7':'Зсув при pH7',
 'hint.health':'Добрий електрод: нахил <b>92–103 %</b>, зсув <b>&lt; ±30 мВ</b>. Поза цим — мити/міняти. Коли калібрування готове, «рН реальний» рахується за буферами (інакше — ідеальний Нернст).',
 'gcap.calph':'Калібрування електрода: pH → мВ — <b style="color:var(--accent)">вимір</b> (двосегментно) vs <b style="color:var(--warn)">ідеал Нернста −59,16</b>',
 'h.eq':'Рівновага електрода',
 'p.eq':'Як швидко встановлюється рН після занурення електрода або зміни розчину. Орієнтир — <b>рН реальний</b> (з S). Натисніть «Старт» одразу після занурення; прилад стежить за дрейфом і оголошує рівновагу, коли коливання за 10 секунд спадуть у межі смуги спокою.',
 'btn.eqstart':'Старт','btn.eqstop':'Стоп','lbl.eqband':'Смуга спокою, рН:','btn.sel':'Обрати межі на графіку',
 'hint.eq2':'Додатково, без очікування: натисніть «Обрати межі на графіку» і клікніть на графіку «рН реальний» <b>початок</b> і <b>кінець</b> перехідного процесу — прилад порахує час, дрейф, t63 і рівноважний рН саме за цим відрізком.',
 'k.eqt':'Час від старту','k.eqdrift':'Дрейф за 10 с','k.eqset':'Час до рівноваги','k.eqt63':'t63 (стала часу)','k.eqph':'Рівноважний рН',
 'h.data':'Дані і дрейф',
 'hint.data':'Історія живе <b>у браузері, не у флеші приладу</b>: запис 1 точка/с, буфер на ~14 год (ніч). Не закривайте вкладку і не давайте компʼютеру заснути. Дані ще й самі осідають у памʼяті браузера і переживають перезавантаження сторінки. Для архіву натисніть «Завантажити CSV».',
 'btn.csv':'Завантажити CSV','btn.clr':'Очистити історію','hint.npts':'точок у памʼяті:','u.ph':'рН',
 // динамічні
 'valid.ready':'готово','valid.notready':'не готово','over.ok':'в нормі','over.over':' понад',
 'pc.ok':'готово · електрод ОК','pc.check':'готово · перевір електрод','pc.notready':'не готово',
 'wait.cal':'чекаю калібрування…','wait.data':'чекаю дані…','nowin':'нема точок у вікні','zoom':'зум',
 'ph.shown':'pH показ','ph.true':'pH справжній','hv.true':'справж ','hv.shown':'показ ',
 'mk.eq':'рівновага','mk.start':'початок','mk.end':'кінець','fromStart':' с від початку',
 'u.na':'нА','u.v':'В','u.mv':'мВ','u.ma':'мА','u.vv':'В/В','u.s':'с','ax.s':' с',
 'cap.calib':'рН реальний (калібрований) · вимір','cap.mark':' (від мітки)','cap.ideal':'рН реальний (ідеал. Нернст) · вимір','cap.volt':'напруга S (В) — рН буде після калібрування плати',
 'calph.notready':'калібрування буферами ще не готове','calph.acid':'кислий ','calph.alk':'лужний ','calph.off1':'зсув ','calph.off2':' мВ @pH7',
 'eq.stab':'стабілізується…','eq.eq':'рівновага','eq.stopped':'зупинено','eq.clickStart':'клікніть ПОЧАТОК на графіку рН реальний','eq.clickEnd':'тепер клікніть КІНЕЦЬ','eq.fewpts':'замало точок у вибраному вікні','eq.eqManual':'рівновага (вручну)',
 'on.live':'плата відповідає','on.dead':'нема відповіді',
 'lbl.show':'будувати:','seg.ph':'рН','seg.sv':'S, В','seg.mv':'мВ електрода',
 'cap.svmeas':'напруга S (В) · вимір','cap.mvmeas':'вхід електрода (мВ) · вимір',
 'cfm.cal':'Скинути калібрування плати (точки E→S)?','cfm.buf':'Скинути калібрування електрода (3 буфери)?','cfm.hist':'Очистити всю історію графіків?',
 'btn.pause':'Пауза','btn.resume':'Відновити'
},en:{
 'title':'pH Calibrator','h.cur':'Current readings',
 'k.A':'A (imitator)','k.S':'S (board output, shunt corrected)','k.sraw':'S raw / shunt drop','k.noise':'S noise','k.cur':'Board current',
 'k.usb':'USB supply','btn.calusb':'cal','pr.usb':'True USB voltage, V (measure with a meter):',
 'k.gaint':'Gain (zero <span id="zerot">–</span> V)',
 'h.blk1':'<span class="car">▾</span> Block 1 · Board leakage',
 'k.il':'Leakage current','k.over':'Above norm (norm <span id="normlbl">–</span> nA)','k.vin':'Board input (calc.)',
 'leakhint':'First do the two-point calibration below to enable the calculation.',
 'h.calzero':'Calibration (button PRESSED, 10 MΩ shorted)',
 'p.e':'<b>E</b> is the imitator voltage — the same one shown above as <b>A</b> (in mV); you set it with the trimmer. The pressed button shorts out the 10 MΩ series resistor (removes it from the circuit), so exactly E reaches the board input — a clean reference point. Release the button to measure leakage.',
 'p.steps':'<b>Step 1.</b> Set some voltage E with the trimmer (watch A above) → “Capture point 1”.<br><b>Step 2.</b> Set a noticeably different E (the farther from the first, the better) → “Capture point 2”.<br>The device builds the board’s “E → S” line (gain and zero), after which it computes the leakage current.',
 'lbl.p1':'Point 1: <span id="p1">—</span>','lbl.p2':'Point 2: <span id="p2">—</span>',
 's.line':'Line: gain <b><span id="gain">—</span></b> V/V, zero <b><span id="zero">—</span></b> V — <span id="valid" class="bad">not ready</span>',
 's.leakoff':'With leakage the input is shifted by <b><span id="leakoff">—</span></b> mV → error <b><span id="leakph">—</span></b> pH (dashed on the graph).',
 'btn.p1':'Capture point 1','btn.p2':'Capture point 2','btn.calreset':'Reset calibration',
 'lbl.R':'Resistor R, MΩ:','btn.saveR':'Save R','btn.zeroa':'Auto-zero A','lbl.norm':'Leak norm, nA:','btn.savenorm':'Save norm',
 'gcap.leak':'Leakage over time, nA (drag a box: width zooms X, height zooms Y; pinch on touch; double-click to reset)',
 'gcap.cal':'Calibration: E (mV) → S (V)','gcap.ph':'pH: shown vs true — green without leak, dashed with leak','btn.rz':'reset zoom',
 'h.blk2':'<span class="car">▾</span> Block 2 · Electrode & pH',
 'gcap.phA':'Reference pH (A, imitator) over time (drag a box: width zooms X, height zooms Y; pinch; double-click to reset)',
 'btn.meas':'Start measurement',
 'hint.meas':'after calibration — clean pH from this mark (calibration dips are not shown); before calibration voltage S is shown',
 'h.bufcal':'Electrode calibration (3 buffers)',
 'p.bufcal':'Maps the board output to <b>true pH</b> using three buffers (two-segment: acid 4–7, alkaline 7–9). Prerequisites: the board calibration above must be ready (the E→S line) and the board powered from its working supply. Immerse the electrode in a buffer, wait for equilibrium (below) → capture the point.',
 'lbl.bt':'Buffer temperature, °C:','btn.setT':'Apply T','hint.bt':'corrects the nominal values (especially alkaline 9.18)',
 'k.buf0':'pH 4.01 · acid','k.buf1':'pH 6.86 · middle','k.buf2':'pH 9.18 · alkaline',
 'btn.cap0':'Capture 4.01','btn.cap1':'Capture 6.86','btn.cap2':'Capture 9.18','btn.bufclr':'Reset',
 'k.state':'State','k.slopeA':'Acid slope','k.slopeB':'Alkaline slope','k.off7':'Offset at pH7',
 'hint.health':'A good electrode: slope <b>92–103 %</b>, offset <b>&lt; ±30 mV</b>. Outside that — clean/replace it. When calibration is ready, “Real pH” is computed from the buffers (otherwise — ideal Nernst).',
 'gcap.calph':'Electrode calibration: pH → mV — <b style="color:var(--accent)">measured</b> (two-segment) vs <b style="color:var(--warn)">ideal Nernst −59.16</b>',
 'h.eq':'Electrode equilibrium',
 'p.eq':'How fast pH settles after immersing the electrode or changing the solution. Reference — <b>Real pH</b> (from S). Press “Start” right after immersion; the device tracks drift and declares equilibrium when the swing over 10 seconds falls within the settle band.',
 'btn.eqstart':'Start','btn.eqstop':'Stop','lbl.eqband':'Settle band, pH:','btn.sel':'Pick bounds on the graph',
 'hint.eq2':'Alternatively, without waiting: press “Pick bounds on the graph” and click the <b>start</b> and <b>end</b> of the transient on the “Real pH” graph — the device computes time, drift, t63 and the equilibrium pH for exactly that segment.',
 'k.eqt':'Time from start','k.eqdrift':'Drift over 10 s','k.eqset':'Time to equilibrium','k.eqt63':'t63 (time constant)','k.eqph':'Equilibrium pH',
 'h.data':'Data & drift',
 'hint.data':'The history lives <b>in the browser, not in the device flash</b>: 1 point/s, a buffer for ~14 h (overnight). Don’t close the tab or let the computer sleep. The data also persists in browser memory and survives a page reload. For an archive press “Download CSV”.',
 'btn.csv':'Download CSV','btn.clr':'Clear history','hint.npts':'points in memory:','u.ph':'pH',
 // dynamic
 'valid.ready':'ready','valid.notready':'not ready','over.ok':'within norm','over.over':' over',
 'pc.ok':'ready · electrode OK','pc.check':'ready · check electrode','pc.notready':'not ready',
 'wait.cal':'awaiting calibration…','wait.data':'awaiting data…','nowin':'no points in window','zoom':'zoom',
 'ph.shown':'pH shown','ph.true':'pH true','hv.true':'true ','hv.shown':'shown ',
 'mk.eq':'equilibrium','mk.start':'start','mk.end':'end','fromStart':' s from start',
 'u.na':'nA','u.v':'V','u.mv':'mV','u.ma':'mA','u.vv':'V/V','u.s':'s','ax.s':' s',
 'cap.calib':'Real pH (calibrated) · measurement','cap.mark':' (from mark)','cap.ideal':'Real pH (ideal Nernst) · measurement','cap.volt':'Voltage S (V) — pH after board calibration',
 'calph.notready':'buffer calibration not ready yet','calph.acid':'acid ','calph.alk':'alkaline ','calph.off1':'offset ','calph.off2':' mV @pH7',
 'eq.stab':'stabilizing…','eq.eq':'equilibrium','eq.stopped':'stopped','eq.clickStart':'click START on the Real pH graph','eq.clickEnd':'now click END','eq.fewpts':'too few points in the selected window','eq.eqManual':'equilibrium (manual)',
 'on.live':'board responding','on.dead':'no response',
 'lbl.show':'plot:','seg.ph':'pH','seg.sv':'S, V','seg.mv':'electrode mV',
 'cap.svmeas':'Voltage S (V) · measurement','cap.mvmeas':'Electrode input (mV) · measurement',
 'cfm.cal':'Reset board calibration (E→S points)?','cfm.buf':'Reset electrode calibration (3 buffers)?','cfm.hist':'Clear all graph history?',
 'btn.pause':'Pause','btn.resume':'Resume'
}};
let lang='ua'; try{lang=localStorage.getItem('calLang')||'ua';}catch(e){} if(lang!=='en')lang='ua';
function T(k){return (L[lang]&&L[lang][k]!=null)?L[lang][k]:(L.ua[k]!=null?L.ua[k]:k);}
function applyLang(){
  document.documentElement.lang=(lang==='ua')?'uk':'en';
  document.title=T('title');
  document.querySelectorAll('[data-t]').forEach(el=>{const k=el.getAttribute('data-t');if(L[lang][k]!=null)el.innerHTML=L[lang][k];});
  const a=document.getElementById('lua'),b=document.getElementById('len');
  if(a)a.classList.toggle('lsel',lang==='ua'); if(b)b.classList.toggle('lsel',lang==='en');
  try{updPauseLbl();}catch(e){}
}
function setLang(x){lang=(x==='en')?'en':'ua';try{localStorage.setItem('calLang',lang);}catch(e){}applyLang();
  try{drawTSAll();if(lastD){drawCal(lastD);drawPh(lastD);}}catch(e){}}

function fmt(x,d){return (x==null||isNaN(x))?'–':Number(x).toFixed(d)}
function fmtN(x){return (x==null||isNaN(x))?'–':(Math.abs(x)<0.1?Number(x).toFixed(3):Number(x).toFixed(1))}
function fmtL(x){if(x==null||isNaN(x))return '–';let v=Number(x);if(Math.abs(v)<0.005)v=0;return Math.abs(v)<1?v.toFixed(2):v.toFixed(1);}
let rEdited=false, nEdited=false, tEdited=false;
document.getElementById('rin').addEventListener('input',()=>rEdited=true);
document.getElementById('nin').addEventListener('input',()=>nEdited=true);
document.getElementById('btin').addEventListener('input',()=>tEdited=true);

const CS=getComputedStyle(document.documentElement);
const COL={line:CS.getPropertyValue('--line').trim(),dim:CS.getPropertyValue('--muted').trim(),
  grn:CS.getPropertyValue('--accent').trim(),amb:CS.getPropertyValue('--warn').trim(),
  bad:CS.getPropertyValue('--bad').trim(),blu:CS.getPropertyValue('--blue').trim(),fg:'#fff'};
const log=[]; const LOG_MAX=50000;   // уся історія сесії: {t, pa (рН опорний), ps (рН реальний), sv (S, В), il (витік нА)}
let lastLogT=0; const LOG_EVERY=1000;  // екран оновлюється 4×/с, а в історію пишемо 1 точку/с → 50000 ≈ 14 год (ніч)
const tsview={};                     // вікно зуму графіка часу (id -> {a,b} у мс); відсутнє = уся історія від старту
const selR={}, pinchS={}, tapT={}; let lastTouch=0;  // новий зум: виділення мишею (px), щипок, час останнього тапу, захист від compat-mouse після дотику
const yview={};                      // вертикальне Y-вікно графіка (id -> {lo,hi} у одиницях величини); відсутнє = авто
let eq={run:false,t0:0,p0:null,samp:[],done:false,tset:0,t63:'—',pfin:0,tsAbs:0};  // автоматичний вимір рівноваги
const msel={mode:0,a:0,b:0};         // ручний вибір меж рівноваги на графіку (0=вимк, 1=чекає початок, 2=чекає кінець)
let lastD=null; const maps={}, hover={};   // показ значень при наведенні на графіки

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
    return {mx:X(e),my:my,lines:['E='+e.toFixed(0)+' '+T('u.mv'), sl!=null?('S='+sl.toFixed(3)+' '+T('u.v')):'']};}};
  drawHover('cCal',c,w,h,maps.cCal);
}

function drawPh(d){
  const cv=document.getElementById('cPh');if(!cv)return;const {c,w,h}=fit(cv);
  c.clearRect(0,0,w,h);const x0=36,x1=w-8,y0=8,y1=h-30;
  c.strokeStyle=COL.line;c.lineWidth=1;c.beginPath();c.moveTo(x0,y0);c.lineTo(x0,y1);c.lineTo(x1,y1);c.stroke();
  c.font='11px system-ui';c.fillStyle=COL.dim;c.textAlign='left';
  if(!d.valid){maps.cPh=null;c.fillText(T('wait.cal'),x0+8,y0+16);return;}
  const p1=7-d.e1/59.16,p2=7-d.e2/59.16;
  let lo=Math.min(p1,p2),hi=Math.max(p1,p2);const pad=(hi-lo)*0.12||1;lo-=pad;hi+=pad;
  const err=(d.vin-d.e)/59.16;
  const X=p=>x0+(x1-x0)*(p-lo)/(hi-lo),Y=p=>y1-(y1-y0)*(p-lo)/(hi-lo);
  c.textAlign='right';
  for(let i=0;i<=4;i++){const p=lo+(hi-lo)*i/4;
    c.globalAlpha=.3;c.strokeStyle=COL.line;c.beginPath();c.moveTo(x0,Y(p));c.lineTo(x1,Y(p));c.stroke();c.globalAlpha=1;
    c.fillStyle=COL.dim;c.fillText(p.toFixed(1),x0-4,Y(p)+3);}
  c.textAlign='center';for(let i=0;i<=4;i++){const p=lo+(hi-lo)*i/4;c.fillText(p.toFixed(1),X(p),y1+14);}
  c.textAlign='left';c.fillText(T('ph.shown'),4,y0+2);c.textAlign='right';c.fillText(T('ph.true'),x1,y1+26);
  c.strokeStyle=COL.grn;c.lineWidth=2;c.beginPath();c.moveTo(X(lo),Y(lo));c.lineTo(X(hi),Y(hi));c.stroke();
  c.setLineDash([5,4]);c.strokeStyle=COL.amb;c.lineWidth=1.8;c.beginPath();c.moveTo(X(lo),Y(lo-err));c.lineTo(X(hi),Y(hi-err));c.stroke();c.setLineDash([]);
  const pt=7-d.e/59.16;
  c.fillStyle=COL.grn;c.beginPath();c.arc(X(pt),Y(pt),3.5,0,7);c.fill();
  c.fillStyle=COL.amb;c.beginPath();c.arc(X(pt),Y(pt-err),4.5,0,7);c.fill();
  maps.cPh={x0,x1,y0,y1,readout:function(px){
    const p=lo+(hi-lo)*(px-x0)/(x1-x0);
    return {mx:X(p),my:Y(p-err),lines:[T('hv.true')+p.toFixed(2),T('hv.shown')+(p-err).toFixed(2)]};}};
  drawHover('cPh',c,w,h,maps.cPh);
}

// Універсальний графік величини в часі з усією історією, зумом і зсувом.
// id-канвас, arr-масив log, key-поле, color-колір, unit-підпис у підказці, markT-мітка (0 якщо нема).
// Один графік у часі. marks — число (один маркер «рівновага») або масив {t,label,color}.
// k2/col2/u2/msp2 — необовʼязкова друга крива (власна права вісь): напр. напруга S поверх рН.
function drawTS(id,arr,key,color,unit,marks,minSpan,k2,col2,u2,msp2,prec){
  const cv=document.getElementById(id);if(!cv){maps[id]=null;return;}const {c,w,h}=fit(cv);
  const two=!!k2;c.clearRect(0,0,w,h);const x0=48,x1=w-(two?46:10),y0=12,y1=h-18;
  c.strokeStyle=COL.line;c.lineWidth=1;c.beginPath();c.moveTo(x0,y0);c.lineTo(x0,y1);c.lineTo(x1,y1);c.stroke();
  c.font='11px system-ui';c.textAlign='left';c.fillStyle=COL.dim;
  const all=arr.filter(p=>p[key]!=null&&!isNaN(p[key]));
  if(all.length<2){maps[id]=null;c.fillText(T('wait.data'),x0+8,y0+16);return;}
  const tF0=all[0].t,tFN=all[all.length-1].t;
  let va=tF0,vb=tFN;const vw=tsview[id];
  if(vw){va=Math.max(tF0,vw.a);vb=Math.min(tFN,vw.b);if(vb-va<500){va=tF0;vb=tFN;}}
  const zoomed=va>tF0||vb<tFN||!!yview[id];
  let lo=0,hi=all.length-1;
  while(lo<all.length&&all[lo].t<va)lo++;if(lo>0)lo--;
  while(hi>0&&all[hi].t>vb)hi--;if(hi<all.length-1)hi++;
  const pts=all.slice(lo,hi+1);
  if(pts.length<2){maps[id]=null;c.fillText(T('nowin'),x0+8,y0+16);return;}
  function scale(kk,msp,fixed){let mn,mx,adec;
    if(fixed){mn=Math.min(fixed.lo,fixed.hi);mx=Math.max(fixed.lo,fixed.hi);if(mx-mn<1e-9){mn-=0.05;mx+=0.05;}
      adec=(mx-mn)<0.03?3:((mx-mn)<0.3?2:((mx-mn)<3?1:0));          // ручне Y-вікно: без паддінга і minSpan
    }else{mn=1e9;mx=-1e9;for(const p of pts){const v=p[kk];if(v==null||isNaN(v))continue;if(v<mn)mn=v;if(v>mx)mx=v;}
      if(mn>mx){mn=0;mx=1;}if(mn===mx){mn-=0.1;mx+=0.1;}
      if(msp&&(mx-mn)<msp){const cc=(mn+mx)/2;mn=cc-msp/2;mx=cc+msp/2;}
      const rng=mx-mn,dd=rng*0.12;adec=rng<0.03?3:(rng<0.3?2:(rng<3?1:0));mn-=dd;mx+=dd;}
    return {mn,mx,adec,Y:v=>y1-(y1-y0)*(v-mn)/(mx-mn)};}
  const S1=scale(key,minSpan,yview[id]),mn=S1.mn,mx=S1.mx,adec=S1.adec,Y=S1.Y;
  const S2=two?scale(k2,msp2):null,Y2=S2?S2.Y:null;
  const sp=Math.max(1,vb-va),X=t=>x0+(x1-x0)*(t-va)/sp;
  c.textAlign='right';
  for(let i=0;i<=2;i++){const v=mn+(mx-mn)*i/2,yy=Y(v);
    c.globalAlpha=.35;c.strokeStyle=COL.line;c.beginPath();c.moveTo(x0,yy);c.lineTo(x1,yy);c.stroke();c.globalAlpha=1;
    let t=v.toFixed(adec);if(/^-0\.?0*$/.test(t))t=t.slice(1);
    c.fillStyle=two?color:COL.dim;c.fillText(t,x0-4,yy+3);}
  if(two){c.textAlign='left';
    for(let i=0;i<=2;i++){const v=S2.mn+(S2.mx-S2.mn)*i/2;c.fillStyle=col2;c.fillText(v.toFixed(S2.adec),x1+4,Y2(v)+3);}}
  c.fillStyle=COL.dim;c.textAlign='left';c.fillText(Math.round((va-tF0)/1000)+T('ax.s'),x0+1,y1+14);
  c.textAlign='right';c.fillText(Math.round((vb-tF0)/1000)+T('ax.s'),x1,y1+14);
  const stride=Math.max(1,Math.floor(pts.length/1800));
  if(two){c.strokeStyle=col2;c.lineWidth=1.4;c.globalAlpha=.9;c.beginPath();
    let st=false;for(let i=0;i<pts.length;i+=stride){const p=pts[i];if(p[k2]==null||isNaN(p[k2]))continue;const xx=X(p.t),yy=Y2(p[k2]);st?c.lineTo(xx,yy):c.moveTo(xx,yy);st=true;}
    c.stroke();c.globalAlpha=1;}
  c.strokeStyle=color;c.lineWidth=1.8;c.beginPath();
  for(let i=0;i<pts.length;i+=stride){const p=pts[i];i?c.lineTo(X(p.t),Y(p[key])):c.moveTo(X(p.t),Y(p[key]));}
  {const p=pts[pts.length-1];c.lineTo(X(p.t),Y(p[key]));}c.stroke();
  const mks=Array.isArray(marks)?marks:(marks?[{t:marks,label:T('mk.eq'),color:COL.fg}]:[]);
  for(const mk of mks){if(!mk||!(mk.t>=va&&mk.t<=vb))continue;const mxp=X(mk.t),cl=mk.color||COL.fg;
    c.strokeStyle=cl;c.lineWidth=1.5;c.setLineDash([4,3]);
    c.beginPath();c.moveTo(mxp,y0);c.lineTo(mxp,y1);c.stroke();c.setLineDash([]);
    const rt=mxp>x1-64;c.fillStyle=cl;c.font='10px system-ui';c.textAlign=rt?'right':'left';
    c.fillText(mk.label||'',mxp+(rt?-3:3),y0+10);}
  if(zoomed){c.fillStyle=COL.amb;c.font='10px system-ui';c.textAlign='right';c.fillText(T('zoom'),x1,y0+10);}
  const sr=selR[id];                       // 2D-прямокутник виділення мишею: ширина→зум X, висота→зум Y
  if(sr){const ax=Math.max(x0,Math.min(sr.ax,x1)),bx=Math.max(x0,Math.min(sr.bx,x1));
    const ay=Math.max(y0,Math.min(sr.ay,y1)),by=Math.max(y0,Math.min(sr.by,y1));
    const rx=Math.min(ax,bx),rw=Math.abs(bx-ax),ry=Math.min(ay,by),rh=Math.abs(by-ay);
    c.save();c.fillStyle='rgba(255,255,255,.12)';c.fillRect(rx,ry,rw,rh);
    c.strokeStyle=COL.amb;c.lineWidth=1;c.strokeRect(rx+.5,ry+.5,Math.max(0,rw-1),Math.max(0,rh-1));c.restore();}
  const rb=document.getElementById('rz_'+id);if(rb)rb.hidden=!zoomed;   // кнопка «скинути зум» — лише коли зумлено
  maps[id]={x0,x1,y0,y1,va,vb,tF0,tFN,ymn:mn,ymx:mx,readout:function(px){
    const tt=va+(vb-va)*(px-x0)/(x1-x0);
    let bi=0,bd=1e9;for(let i=0;i<pts.length;i++){const dx=Math.abs(pts[i].t-tt);if(dx<bd){bd=dx;bi=i;}}
    const p=pts[bi],fromStart=Math.round((p.t-tF0)/1000);
    let vt=p[key].toFixed(prec!=null?prec:Math.max(2,adec));if(/^-0\.?0*$/.test(vt))vt=vt.slice(1);
    const L=[vt+' '+unit];
    if(two&&p[k2]!=null&&!isNaN(p[k2]))L.push(p[k2].toFixed(Math.max(2,S2.adec))+' '+u2);
    L.push(fromStart+T('fromStart'));
    return {mx:X(p.t),my:Y(p[key]),lines:L};}};
  drawHover(id,c,w,h,maps[id]);
}
function drawTSAll(){
  drawTS('cLeak',log,'il',COL.amb,T('u.na'),0,0);
  drawTS('cPhA',log,'pa',COL.amb,T('u.ph'),0,0.1);
  const mk=[];
  if(msel.a)mk.push({t:msel.a,label:T('mk.start'),color:COL.amb});
  if(msel.b)mk.push({t:msel.b,label:T('mk.end'),color:COL.amb});
  if(!msel.a&&!msel.b&&eq.done&&eq.tsAbs)mk.push({t:eq.tsAbs,label:T('mk.eq'),color:COL.fg});
  const cap=document.getElementById('cPhSCap');
  // Що будувати на графіку виміру — за селектором (рН / S / мВ). Уся історія вже в логу,
  // тож перемикання лише перемальовує, не починає з нуля. Точність: pH 3, S 3, мВ 1.
  if(phSMode==='sv'){
    if(cap)cap.textContent=T('cap.svmeas');
    drawTS('cPhS',log,'sv',COL.blu,T('u.v'),mk,0,null,null,null,null,3);
  }else if(phSMode==='mv'){
    if(cap)cap.textContent=T('cap.mvmeas');
    drawTS('cPhS',log,'vin',COL.amb,T('u.mv'),mk,0,null,null,null,null,1);
  }else{ // 'ph'
    const pcv=lastD&&lastD.pcv;
    if(pcv){ if(cap)cap.textContent=T('cap.calib')+(measT?T('cap.mark'):'');
      const seg=measT?log.filter(p=>p.t>=measT):log;
      drawTS('cPhS',seg,'ps',COL.grn,T('u.ph'),mk,0.1,null,null,null,null,3);
    }else{ if(cap)cap.textContent=T('cap.ideal');
      drawTS('cPhS',log,'ps',COL.grn,T('u.ph'),mk,0.1,null,null,null,null,3);
    }
  }
  drawCalPh();
  const n=document.getElementById('npts');if(n)n.textContent=log.length;
}
// Калібрувальний графік: pH -> мВ електрода, вимір (двосегментно) vs ідеал Нернста.
function drawCalPh(){
  const cv=document.getElementById('cCalPh');if(!cv)return;const {c,w,h}=fit(cv);
  c.clearRect(0,0,w,h);c.font='10px system-ui';const d=lastD;
  const x0=54,x1=w-14,y0=14,y1=h-26;
  if(!d||!d.pcv){c.fillStyle=COL.dim;c.textAlign='left';c.fillText(T('calph.notready'),x0+6,y0+20);return;}
  const pts=[[d.pc0,d.bv0],[d.pc1,d.bv1],[d.pc2,d.bv2]],pMin=3.5,pMax=9.5,NS=59.16;
  const idA=NS*(7-pMin),idB=NS*(7-pMax);
  let vLo=idB,vHi=idA;for(const q of pts){vLo=Math.min(vLo,q[1]);vHi=Math.max(vHi,q[1]);}
  const pad=(vHi-vLo)*0.14||10;vLo-=pad;vHi+=pad;
  const X=p=>x0+(p-pMin)/(pMax-pMin)*(x1-x0),Y=v=>y1-(v-vLo)/(vHi-vLo)*(y1-y0);
  c.textAlign='right';
  for(let i=0;i<=4;i++){const v=vLo+(vHi-vLo)*i/4,yy=Y(v);c.globalAlpha=.3;c.strokeStyle=COL.line;c.beginPath();c.moveTo(x0,yy);c.lineTo(x1,yy);c.stroke();c.globalAlpha=1;c.fillStyle=COL.dim;c.fillText(v.toFixed(0),x0-6,yy+3);}
  for(let p=4;p<=9;p++){const xx=X(p);c.globalAlpha=p===7?.7:.25;c.strokeStyle=COL.line;c.beginPath();c.moveTo(xx,y0);c.lineTo(xx,y1);c.stroke();c.globalAlpha=1;c.fillStyle=COL.dim;c.textAlign='center';c.fillText('pH '+p,xx,y1+16);}
  c.strokeStyle=COL.amb;c.lineWidth=2;c.setLineDash([7,5]);c.beginPath();c.moveTo(X(pMin),Y(idA));c.lineTo(X(pMax),Y(idB));c.stroke();c.setLineDash([]);
  const sa=(pts[1][1]-pts[0][1])/(pts[1][0]-pts[0][0]),sb=(pts[2][1]-pts[1][1])/(pts[2][0]-pts[1][0]);
  const mp=[[pMin,pts[0][1]+sa*(pMin-pts[0][0])],pts[0],pts[1],pts[2],[pMax,pts[2][1]+sb*(pMax-pts[2][0])]];
  c.strokeStyle=COL.grn;c.lineWidth=2.4;c.beginPath();mp.forEach((q,i)=>i?c.lineTo(X(q[0]),Y(q[1])):c.moveTo(X(q[0]),Y(q[1])));c.stroke();
  pts.forEach(q=>{c.fillStyle=COL.grn;c.beginPath();c.arc(X(q[0]),Y(q[1]),5,0,7);c.fill();c.strokeStyle=COL.fg;c.lineWidth=1.5;c.stroke();c.fillStyle=COL.fg;c.textAlign='center';c.fillText(q[1].toFixed(1)+' '+T('u.mv'),X(q[0]),Y(q[1])-10);});
  c.fillStyle=COL.grn;c.textAlign='left';c.fillText(T('calph.acid')+d.psa.toFixed(1)+'%',X(4.5),Y((pts[0][1]+pts[1][1])/2)-6);
  c.fillText(T('calph.alk')+d.psb.toFixed(1)+'%',X(7.3),Y((pts[1][1]+pts[2][1])/2)+14);
  c.fillStyle=COL.amb;c.fillText(T('calph.off1')+d.pof.toFixed(0)+T('calph.off2'),X(7)+4,Y(0)+16);
}

// Вимір швидкості встановлення рівноваги електрода (працює з реальним рН із S).
function eqSet(id,t){document.getElementById(id).textContent=t;}
function eqStart(){eq={run:true,t0:Date.now(),p0:null,samp:[],done:false,tset:0,t63:'—',pfin:0,tsAbs:0};
  eqSet('eqstate',T('eq.stab'));['eqt','eqdrift','eqset','eqt63'].forEach(i=>eqSet(i,'—'));eqSet('eqph','—');}
function eqStop(){eq.run=false;eqSet('eqstate', eq.done?T('eq.eq'):T('eq.stopped'));}
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
    eqSet('eqset',eq.tset.toFixed(0));eqSet('eqph',eq.pfin.toFixed(2));eqSet('eqstate',T('eq.eq'));
    const target=eq.p0+0.63*(eq.pfin-eq.p0);
    for(const s of eq.samp){if((eq.pfin>=eq.p0&&s.p>=target)||(eq.pfin<eq.p0&&s.p<=target)){eq.t63=((s.t-eq.t0)/1000).toFixed(1);break;}}
    eqSet('eqt63',eq.t63);
  }else{eqSet('eqstate',T('eq.stab'));}
}

// Ручний варіант: межі перехідного процесу вибираються кліками на графіку «рН реальний».
function selStart(){msel.mode=1;msel.a=0;msel.b=0;eq.done=false;
  eqSet('eqstate',T('eq.clickStart'));['eqt','eqdrift','eqset','eqt63'].forEach(i=>eqSet(i,'—'));eqSet('eqph','—');drawTSAll();}
function eqManual(){
  const a=Math.min(msel.a,msel.b),b=Math.max(msel.a,msel.b);
  const seg=log.filter(p=>p.ps!=null&&!isNaN(p.ps)&&p.t>=a&&p.t<=b);
  if(seg.length<2){eqSet('eqstate',T('eq.fewpts'));return;}
  const t0=seg[0].t,tEnd=seg[seg.length-1].t,p0=seg[0].ps;
  const tailFrom=tEnd-Math.min(10000,(tEnd-t0)*0.25);const tail=seg.filter(s=>s.t>=tailFrom);
  let sum=0,mn=1e9,mx=-1e9;for(const s of tail){sum+=s.ps;if(s.ps<mn)mn=s.ps;if(s.ps>mx)mx=s.ps;}
  const pfin=sum/tail.length,drift=mx-mn,tset=(tEnd-t0)/1000;
  const target=p0+0.63*(pfin-p0);let t63='—';
  for(const s of seg){if((pfin>=p0&&s.ps>=target)||(pfin<p0&&s.ps<=target)){t63=((s.t-t0)/1000).toFixed(1);break;}}
  eqSet('eqstate',T('eq.eqManual'));eqSet('eqt',tset.toFixed(0));eqSet('eqdrift',drift.toFixed(3));
  eqSet('eqset',tset.toFixed(0));eqSet('eqt63',t63);eqSet('eqph',pfin.toFixed(2));
}

// Накладка при наведенні: вертикальна лінія, точка на кривій і підпис зі значенням.
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
// Зум графіків часу. Миша: протягнути РАМКУ — ширина задає зум по X, висота — зум по Y (горизонтальний
// мазок = тільки X, як раніше); наведення без кнопки — підказка; подвійний клік — скинути (обидві осі).
// Дотик: два пальці — щипок по X; один палець — підказка; подвійний тап — скинути. X-вікно tsview[id]={a,b}
// (мс), Y-вікно yview[id]={lo,hi} (одиниці величини); обидва null = авто. Кнопка «скинути зум» — лише при зумі.
function zoomReset(id){tsview[id]=null;yview[id]=null;hover[id]=null;try{drawTSAll();}catch(e){}}
function tsPick(m,px){return m.va+(m.vb-m.va)*(Math.max(m.x0,Math.min(m.x1,px))-m.x0)/((m.x1-m.x0)||1);} // піксель → час
function yPick(m,py){return m.ymn+(m.y1-Math.max(m.y0,Math.min(m.y1,py)))*(m.ymx-m.ymn)/((m.y1-m.y0)||1);} // піксель → значення
function hookTS(id){
  const cv=document.getElementById(id);if(!cv)return;
  const xp=cx=>{const r=cv.getBoundingClientRect();return cx-r.left;};
  const yp=cy=>{const r=cv.getBoundingClientRect();return cy-r.top;};
  // --- Миша (десктоп): 2D-рамка (ширина→зум X, висота→зум Y) + наведення ---
  cv.addEventListener('mousemove',ev=>{if(Date.now()-lastTouch<600)return;const x=xp(ev.clientX),y=yp(ev.clientY);
    if(selR[id]){selR[id].bx=x;selR[id].by=y;drawTSAll();return;}
    hover[id]={x,y};drawTSAll();});
  cv.addEventListener('mouseleave',()=>{if(Date.now()-lastTouch<600)return;hover[id]=null;selR[id]=null;drawTSAll();});
  cv.addEventListener('mousedown',ev=>{if(Date.now()-lastTouch<600)return;const m=maps[id];if(!m)return;const x=xp(ev.clientX),y=yp(ev.clientY);
    if(id==='cPhS'&&msel.mode){const tt=tsPick(m,x);
      if(msel.mode===1){msel.a=tt;msel.b=0;msel.mode=2;eqSet('eqstate',T('eq.clickEnd'));}
      else{msel.b=tt;msel.mode=0;eqManual();}
      drawTSAll();return;}
    selR[id]={ax:x,ay:y,bx:x,by:y};ev.preventDefault();});
  cv.addEventListener('mouseup',ev=>{if(Date.now()-lastTouch<600)return;const s=selR[id];selR[id]=null;const m=maps[id];
    if(s&&m){const wX=Math.abs(s.bx-s.ax),wY=Math.abs(s.by-s.ay);
      if(wX>8){const a=tsPick(m,s.ax),b=tsPick(m,s.bx),lo=Math.min(a,b),hi=Math.max(a,b);
        if(hi-lo>=500)tsview[id]=(hi-lo>=(m.tFN-m.tF0)-1)?null:{a:lo,b:hi};}
      if(wY>8){const a=yPick(m,s.ay),b=yPick(m,s.by),lo=Math.min(a,b),hi=Math.max(a,b);
        if(hi-lo>1e-9)yview[id]={lo,hi};}
    }
    drawTSAll();});
  cv.addEventListener('dblclick',()=>zoomReset(id));
  // --- Дотик (мобільний): щипок двома пальцями + підказка одним + подвійний тап ---
  cv.addEventListener('touchstart',ev=>{lastTouch=Date.now();const m=maps[id];
    if(ev.touches.length===2&&m){const x0t=xp(ev.touches[0].clientX),x1t=xp(ev.touches[1].clientX),mid=(x0t+x1t)/2;
      pinchS[id]={d:Math.abs(x0t-x1t)||1,at:tsPick(m,mid),span:m.vb-m.va,pw:(m.x1-m.x0)||1,x0:m.x0,tF0:m.tF0,tFN:m.tFN};
      selR[id]=null;tapT[id]=0;ev.preventDefault();return;}
    if(ev.touches.length===1){const x=xp(ev.touches[0].clientX);
      if(id==='cPhS'&&msel.mode&&m){const tt=tsPick(m,x);
        if(msel.mode===1){msel.a=tt;msel.b=0;msel.mode=2;eqSet('eqstate',T('eq.clickEnd'));}
        else{msel.b=tt;msel.mode=0;eqManual();}
        ev.preventDefault();drawTSAll();return;}
      const now=Date.now();if(now-(tapT[id]||0)<300){zoomReset(id);tapT[id]=0;}else{tapT[id]=now;
        const r=cv.getBoundingClientRect();hover[id]={x,y:ev.touches[0].clientY-r.top};drawTSAll();}}
  },{passive:false});
  cv.addEventListener('touchmove',ev=>{lastTouch=Date.now();const p=pinchS[id];
    if(p&&ev.touches.length===2){ev.preventDefault();
      const x0t=xp(ev.touches[0].clientX),x1t=xp(ev.touches[1].clientX),mid=(x0t+x1t)/2,d=Math.abs(x0t-x1t)||1;
      const full=p.tFN-p.tF0;let span=Math.min(full,Math.max(1000,p.span*p.d/d));
      let a=p.at-(mid-p.x0)*span/p.pw,b=a+span;
      if(a<p.tF0){a=p.tF0;b=a+span;}if(b>p.tFN){b=p.tFN;a=b-span;}if(a<p.tF0)a=p.tF0;
      tsview[id]=(span>=full)?null:{a,b};drawTSAll();return;}
    if(ev.touches.length===1){const r=cv.getBoundingClientRect();hover[id]={x:xp(ev.touches[0].clientX),y:ev.touches[0].clientY-r.top};drawTSAll();}
  },{passive:false});
  cv.addEventListener('touchend',ev=>{lastTouch=Date.now();if(ev.touches.length<2)pinchS[id]=null;
    if(ev.touches.length===0){hover[id]=null;drawTSAll();}});
}
hookTS('cLeak');hookTS('cPhA');hookTS('cPhS');
hookHover('cCal',()=>{try{if(lastD)drawCal(lastD);}catch(e){}});
hookHover('cPh',()=>{try{if(lastD)drawPh(lastD);}catch(e){}});

// Експорт усієї історії у CSV-файл (завантажує браузер) і автозбереження в памʼять браузера.
function expCsv(){
  let s='t_ms,pH_real_S,S_V,S_raw_V,shunt_drop_V,I_mA\n';
  for(const p of log){s+=p.t+','+(p.ps!=null?p.ps.toFixed(3):'')+','+(p.sv!=null?p.sv.toFixed(4):'')+','+
    (p.sr!=null?p.sr.toFixed(4):'')+','+(p.sd!=null?p.sd.toFixed(4):'')+','+(p.im!=null?p.im.toFixed(2):'')+'\n';}
  const bl=new Blob([s],{type:'text/csv'});const u=URL.createObjectURL(bl);
  const a=document.createElement('a');a.href=u;a.download='calibrator_'+new Date().toISOString().replace(/[:.]/g,'-')+'.csv';
  document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(u),1000);
}
function clrLog(){log.length=0;delete tsview.cLeak;delete tsview.cPhA;delete tsview.cPhS;try{localStorage.removeItem('calLog')}catch(e){}drawTSAll();}
function saveLog(){try{const n=log.length,st=Math.max(0,n-LOG_MAX);localStorage.setItem('calLog',JSON.stringify(log.slice(st)));}catch(e){}}
function loadLog(){try{const r=localStorage.getItem('calLog');if(r){const a=JSON.parse(r);if(Array.isArray(a))for(const p of a)log.push(p);}}catch(e){}}
loadLog();setInterval(saveLog,30000);
function togBlk(id){document.getElementById(id).classList.toggle('collapsed');saveUI();}
// Стан згорток (та інші перемикачі сторінки) переживає перезавантаження — у памʼяті браузера.
function saveUI(){try{const s={};document.querySelectorAll('.block').forEach(b=>s[b.id]=b.classList.contains('collapsed'));localStorage.setItem('calUI',JSON.stringify(s));}catch(e){}}
function loadUI(){try{const r=localStorage.getItem('calUI');if(!r)return;const s=JSON.parse(r);for(const id in s){const el=document.getElementById(id);if(el)el.classList.toggle('collapsed',!!s[id]);}}catch(e){}}
loadUI();
// Згортання окремих графіків (напр. «рН опорний») — не потрібен при реальних вимірах.
function togGraph(id){const el=document.getElementById(id);if(!el)return;el.classList.toggle('collapsed');
  try{const s=JSON.parse(localStorage.getItem('calG')||'{}');s[id]=el.classList.contains('collapsed');localStorage.setItem('calG',JSON.stringify(s));}catch(e){}
  try{drawTSAll();}catch(e){}}
function loadGraphs(){try{const s=JSON.parse(localStorage.getItem('calG')||'{}');for(const id in s){const el=document.getElementById(id);if(el)el.classList.toggle('collapsed',!!s[id]);}}catch(e){}}
loadGraphs();
// Селектор графіка виміру «Real pH … measurement»: що будувати на cPhS — рН / S / мВ електрода.
// Точність показу: pH — 3 знаки, S (В) — 3 знаки, мВ — 1 знак.
let phSMode='ph'; try{phSMode=localStorage.getItem('calPhSMode')||'ph';}catch(e){}
if(!['ph','sv','mv'].includes(phSMode))phSMode='ph';
function setPhSMode(m){phSMode=m;try{localStorage.setItem('calPhSMode',m);}catch(e){}updPhSSeg();try{drawTSAll();}catch(e){}}
function updPhSSeg(){['ph','sv','mv'].forEach(m=>{const b=document.getElementById('sg_'+m);if(b)b.classList.toggle('on',phSMode===m);});}
updPhSSeg();
// Підтвердження перед скиданням параметрів (щоб випадково не стерти калібрування).
function postC(u,k){if(confirm(T(k)))post(u);}
function calUsb(){const cur=(lastD&&lastD.usb!=null)?lastD.usb.toFixed(2):'5.00';const v=prompt(T('pr.usb'),cur);if(v==null)return;const n=parseFloat((''+v).replace(',','.'));if(isFinite(n)&&n>0.5)fetch('/calusb?v='+n,{method:'POST'});}
let measT=0; try{measT=+localStorage.getItem('measT')||0;}catch(e){}  // мітка «Почати вимір»
function measStart(){measT=Date.now();try{localStorage.setItem('measT',measT);}catch(e){}drawTSAll();}

// Індикатор «плата відповідає»: зелений, поки йдуть пуші; сірий, якщо замовкли.
let onlineT=0;
function setOnline(ok){const el=document.getElementById('online');if(!el)return;
  el.classList.toggle('on',ok);el.classList.toggle('off',!ok);el.title=ok?T('on.live'):T('on.dead');}
function markOnline(){onlineT=Date.now();setOnline(true);}
setInterval(()=>{if(Date.now()-onlineT>1500)setOnline(false);},700);   // вартовий

// Пауза читання: прилад далі шле, але сторінка ігнорує пакети (графіки/покази заморожені).
let paused=false;
function updPauseLbl(){const b=document.getElementById('pauseBtn');if(b)b.textContent=paused?T('btn.resume'):T('btn.pause');}
function togPause(){paused=!paused;const b=document.getElementById('pauseBtn');if(b)b.classList.toggle('p',paused);updPauseLbl();}

// Застосувати один пакет телеметрії (прийшов по SSE або разовим /data).
function applyData(d){
    markOnline();            // прилад живий — індикатор горить навіть на паузі
    if(paused)return;        // пауза: ігноруємо пакет, графіки стоять
    lastD=d;
    const psv=d.pcv?d.pr:(d.valid?(7-d.vin/59.16):null);   // рН реальний: буферний або ідеал. Нернст
    e.textContent=fmt(d.e,1); s.textContent=fmt(d.s,3);
    sraw.textContent=fmt(d.s_raw,3); sdrop.textContent=fmt(d.shunt_drop,3);
    phA.textContent=fmt(d.ph,2); phS.textContent=(psv!=null)?psv.toFixed(2):'–';
    noise.textContent=fmt(d.noise,2);
    cur.textContent=fmt(d.i,2);
    {const u=document.getElementById('usb');if(u&&d.usb!=null)u.textContent=fmt(d.usb,3);}
    il.textContent=d.valid?fmtL(d.il):'–';
    vin.textContent=d.valid?fmt(d.vin,1):'–';
    document.getElementById('normlbl').textContent=fmtN(d.norm);
    const ov=document.getElementById('over');
    if(d.valid){
      const k=Math.abs(d.il)/((d.norm>0)?d.norm:1);
      if(k<=1){ov.textContent=T('over.ok');ov.style.color=COL.grn;}
      else{ov.textContent='×'+(k>=10?k.toFixed(0):k.toFixed(1))+T('over.over');ov.style.color=(Math.abs(d.il)>=1.0?COL.bad:COL.amb);}
    }else{ov.textContent='–';ov.style.color='';}
    p1.textContent=d.h1?('E='+fmt(d.e1,1)+' '+T('u.mv')+', S='+fmt(d.s1,3)+' '+T('u.v')):'—';
    p2.textContent=d.h2?('E='+fmt(d.e2,1)+' '+T('u.mv')+', S='+fmt(d.s2,3)+' '+T('u.v')):'—';
    gain.textContent=d.valid?fmt(d.gain,3):'—';
    zero.textContent=d.valid?fmt(d.zero,3):'—';
    gaint.textContent=d.valid?fmt(Math.abs(d.gain),2):'–';
    document.getElementById('zerot').textContent=d.valid?fmt(d.zero,2):'–';
    if(d.valid){const dl=d.vin-d.e;document.getElementById('leakoff').textContent=fmt(dl,0);document.getElementById('leakph').textContent=fmt(Math.abs(dl)/59.16,1);}
    else{document.getElementById('leakoff').textContent='—';document.getElementById('leakph').textContent='—';}
    const v=document.getElementById('valid');
    v.textContent=d.valid?T('valid.ready'):T('valid.notready'); v.className=d.valid?'ok':'bad';
    document.getElementById('leakhint').style.display=d.valid?'none':'';
    if(!rEdited) document.getElementById('rin').value=fmt(d.r,2);
    if(!nEdited) document.getElementById('nin').value=fmtN(d.norm);
    for(let i=0;i<3;i++){const h=d['bh'+i],bv=d['bv'+i];
      document.getElementById('bv'+i).textContent=h?bv.toFixed(1):'–';
      document.getElementById('bs'+i).textContent=h?'✓':'–';
      const pe=document.getElementById('pc'+i);
      if(pe)pe.textContent=(Math.abs(d.tc-25)<0.05)?'':('→ '+d['pc'+i].toFixed(2));}
    if(!tEdited)document.getElementById('btin').value=fmt(d.tc,1);
    document.getElementById('psa').textContent=d.pcv?d.psa.toFixed(1):'–';
    document.getElementById('psb').textContent=d.pcv?d.psb.toFixed(1):'–';
    document.getElementById('pof').textContent=d.pcv?d.pof.toFixed(1):'–';
    const pcs=document.getElementById('pcstate');
    if(d.pcv){const ok=d.psa>=92&&d.psa<=103&&d.psb>=92&&d.psb<=103&&Math.abs(d.pof)<30;
      pcs.textContent=ok?T('pc.ok'):T('pc.check');pcs.style.color=ok?COL.grn:COL.amb;}
    else{pcs.textContent=T('pc.notready');pcs.style.color='';}
    const now=Date.now();
    if(now-lastLogT>=LOG_EVERY){lastLogT=now;
      log.push({t:now,pa:d.ph,ps:psv,sv:d.s,sr:d.s_raw,sd:d.shunt_drop,im:d.i,vin:(d.valid?d.vin:null),il:(d.valid?d.il:null)}); if(log.length>LOG_MAX)log.shift();}
    eqTick(psv);
    drawCal(d); drawPh(d); drawTSAll();
}
// SSE: сервер пушить стан ~5×/с; EventSource сам перепідключається при обриві.
let es=null;
function startSSE(){
  try{es=new EventSource('/events');
    es.onmessage=ev=>{try{applyData(JSON.parse(ev.data));}catch(e){}};
    es.onerror=()=>setOnline(false);           // браузер перепідключиться автоматично
  }catch(e){ setInterval(pokeOnce,300); }       // фолбек: якщо EventSource недоступний — разові /data
}
async function pokeOnce(){try{const d=await (await fetch('/data')).json();applyData(d);}catch(e){}}
async function post(u){await fetch(u,{method:'POST'});setTimeout(pokeOnce,100)}
async function setR(){
  const meg=document.getElementById('rin').value;
  await fetch('/setr?meg='+encodeURIComponent(meg),{method:'POST'});
  rEdited=false; setTimeout(pokeOnce,100);
}
async function setNorm(){
  const na=document.getElementById('nin').value;
  await fetch('/setnorm?na='+encodeURIComponent(na),{method:'POST'});
  nEdited=false; setTimeout(pokeOnce,100);
}
async function setTemp(){
  const c=document.getElementById('btin').value;
  await fetch('/settemp?c='+encodeURIComponent(c),{method:'POST'});
  tEdited=false; setTimeout(pokeOnce,100);
}
window.addEventListener('resize',()=>{try{drawTSAll();if(lastD){drawCal(lastD);drawPh(lastD);}}catch(e){}});
applyLang();
startSSE(); pokeOnce();
</script>
</body></html>)HTML";

// Зібрати JSON поточного стану у buf (лише читає спільні значення). Повертає довжину.
static size_t buildDataJson(char* buf, size_t n) {
  return (size_t)snprintf(buf, n,
    "{\"e\":%.1f,\"s\":%.3f,\"ph\":%.2f,\"vin\":%.1f,\"il\":%.3f,\"noise\":%.2f,"
    "\"h1\":%d,\"e1\":%.1f,\"s1\":%.3f,\"h2\":%d,\"e2\":%.1f,\"s2\":%.3f,"
    "\"valid\":%d,\"gain\":%.3f,\"zero\":%.3f,\"r\":%.3f,\"norm\":%.3f,\"i\":%.2f,"
    "\"pr\":%.2f,\"pcv\":%d,\"psa\":%.1f,\"psb\":%.1f,\"pof\":%.1f,\"tc\":%.1f,"
    "\"usb\":%.3f,\"s_raw\":%.3f,\"shunt_drop\":%.3f,\"shunt_valid\":%d,"
    "\"pc0\":%.2f,\"pc1\":%.2f,\"pc2\":%.2f,"
    "\"bh0\":%d,\"bv0\":%.1f,\"bh1\":%d,\"bv1\":%.1f,\"bh2\":%d,\"bv2\":%.1f}",
    g_E_mV, g_S_V, g_pH, g_Vin_mV, g_Ileak_nA, g_noise_mV,
    haveP1 ? 1 : 0, E1_mV, S1_V, haveP2 ? 1 : 0, E2_mV, S2_V,
    calValid ? 1 : 0, gainS * 1000.0f, zeroS, Rseries / 1.0e6f, leakNormNA, g_I_mA,
    g_pH_real, phCalValid ? 1 : 0,
    phCalValid ? fabsf(phSlopeAcid) / PH_SLOPE * 100.0f : 0.0f,
    phCalValid ? fabsf(phSlopeBase) / PH_SLOPE * 100.0f : 0.0f, phOffset_mV, bufTempC,
    g_usb_V, g_S_raw_V, g_shuntDrop_V, g_shuntValid ? 1 : 0,
    bufPhAt(0), bufPhAt(1), bufPhAt(2),
    bufHave[0] ? 1 : 0, bufVin[0], bufHave[1] ? 1 : 0, bufVin[1], bufHave[2] ? 1 : 0, bufVin[2]);
}

// /data — одноразовий знімок (фолбек/діагностика; основний канал — SSE /events).
static void sendData(AsyncWebServerRequest* req) {
  char buf[1024];
  buildDataJson(buf, sizeof(buf));
  req->send(200, "application/json", buf);
}

// Маршрути веб-сервера (спільні для STA і запасної AP). Сервер слухає одразу.
static void startServerRoutes() {
  // HTML великий (~70 КБ): віддаємо потоком із PROGMEM (AsyncProgmemResponse, чанками),
  // БЕЗ копії у heap-String — інакше на фрагментованій купі суцільний блок не виділяється і
  // сторінка приходить порожня (Content-Length 0). send_P(PGM_P) на ESP32 саме так і робить.
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){
    r->send(200, "text/html", (const uint8_t*)INDEX_HTML, strlen_P(INDEX_HTML));
  });
  server.on("/data", HTTP_GET, sendData);
  server.on("/p1",  HTTP_POST, [](AsyncWebServerRequest* r){ req_capP1  = true; r->send(200, "text/plain", "ok"); });
  server.on("/p2",  HTTP_POST, [](AsyncWebServerRequest* r){ req_capP2  = true; r->send(200, "text/plain", "ok"); });
  server.on("/calreset", HTTP_POST, [](AsyncWebServerRequest* r){ req_calRst = true; r->send(200, "text/plain", "ok"); });
  server.on("/zeroa", HTTP_POST, [](AsyncWebServerRequest* r){ req_zeroA = true; r->send(200, "text/plain", "ok"); });
  server.on("/calusb", HTTP_POST, [](AsyncWebServerRequest* r){   // калібрувати дільник USB: v=істинна напруга
    if (r->hasParam("v")) {
      float v = r->getParam("v")->value().toFloat();
      if (v > 0.5f) { req_usbTrue = v; req_calUsb = true; }
    }
    r->send(200, "text/plain", "ok");
  });
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
  server.on("/bufcap", HTTP_POST, [](AsyncWebServerRequest* r){
    if (r->hasParam("i")) {
      int i = r->getParam("i")->value().toInt();
      if (i >= 0 && i < 3) req_bufCap = i + 1;
    }
    r->send(200, "text/plain", "ok");
  });
  server.on("/bufclr", HTTP_POST, [](AsyncWebServerRequest* r){ req_bufClr = true; r->send(200, "text/plain", "ok"); });
  server.on("/setbuf", HTTP_POST, [](AsyncWebServerRequest* r){   // відновити 3 буфери: vin у мВ (+ опц. tc)
    if (r->hasParam("v0") && r->hasParam("v1") && r->hasParam("v2")) {
      req_bufV[0] = r->getParam("v0")->value().toFloat();
      req_bufV[1] = r->getParam("v1")->value().toFloat();
      req_bufV[2] = r->getParam("v2")->value().toFloat();
      req_bufTcV  = r->hasParam("tc") ? r->getParam("tc")->value().toFloat() : 25.0f;
      req_setBuf = true;
    }
    r->send(200, "text/plain", "ok");
  });
  server.on("/settemp", HTTP_POST, [](AsyncWebServerRequest* r){
    if (r->hasParam("c")) {
      float c = r->getParam("c")->value().toFloat();
      if (c > -5.0f && c < 100.0f) { req_TempVal = c; req_setTemp = true; }
    }
    r->send(200, "text/plain", "ok");
  });
  events.onConnect([](AsyncEventSourceClient* c){   // новому клієнту — одразу поточний стан
    char buf[1024]; buildDataJson(buf, sizeof(buf)); c->send(buf, nullptr, millis());
  });
  server.addHandler(&events);
  server.onNotFound([](AsyncWebServerRequest* r){ r->redirect("/"); });   // captive portal
  server.begin();
  wifiOk = true;
}

// Запустити Wi-Fi у ФОНІ — НЕ блокує екран. Під'єднання добиває wifiTick() у loop().
static void wifiBegin() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(HOSTNAME);
  // Дозволити канали 1..13 (Livebox/EU можуть стояти на 12/13).
  wifi_country_t ctry = { {'F','R',0}, 1, 13, 20, WIFI_COUNTRY_POLICY_MANUAL };
  esp_wifi_set_country(&ctry);
  // ESP32-C3 SuperMini: знижуємо потужність передавача — інакше рве рукостискання.
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  wifiStartMs = millis();
  Serial.printf("Wi-Fi: під'єднання до \"%s\" у фоні...\n", WIFI_SSID);
  startServerRoutes();   // сервер слухає одразу, віддаватиме щойно з'явиться IP
}

// Стан Wi-Fi: викликається щоцикл у loop(), не блокує.
static void wifiTick() {
  if (apMode) return;                       // уже в запасному режимі
  if (WiFi.status() == WL_CONNECTED) {
    if (!staConnected) {
      staConnected = true;
      MDNS.begin(HOSTNAME);
      MDNS.addService("http", "tcp", 80);
      snprintf(g_ip, sizeof(g_ip), "%s", WiFi.localIP().toString().c_str());
      connMsgUntil = millis() + 4000;      // показати IP на екрані 4 с
      Serial.printf("Wi-Fi ок: http://%s/ або http://%s.local/\n", g_ip, HOSTNAME);
    }
    return;
  }
  if (millis() - wifiStartMs > 20000) {     // за 20 с не зайшло -> запасна AP + captive-DNS
    apMode = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    IPAddress ip = WiFi.softAPIP();
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, "*", ip);
    MDNS.begin(HOSTNAME);
    MDNS.addService("http", "tcp", 80);
    snprintf(g_ip, sizeof(g_ip), "%s", ip.toString().c_str());
    connMsgUntil = millis() + 6000;        // показати адресу AP на екрані 6 с
    Serial.printf("Wi-Fi не вдалось; запасна AP \"%s\" (пароль %s): http://%s/\n",
                  AP_SSID, AP_PASS, g_ip);
  }
}

// Виконати запити від веб-обробників (тільки в loop -> I2C/NVS в одному таску).
static void processWebRequests(float va, float vs) {
  if (req_zeroA)  { req_zeroA = false; offsetA += va; saveFloat("offA", offsetA);
                    Serial.printf("Автонуль A (веб): offsetA=%.5f V\n", offsetA); }
  if (req_capP1)  { req_capP1 = false; E1_mV = va * 1000.0f; S1_V = vs; haveP1 = true;
                    computeLeakCal(); saveLeak(); Serial.println(F("Точка 1 знята")); }
  if (req_capP2)  { req_capP2 = false; E2_mV = va * 1000.0f; S2_V = vs; haveP2 = true;
                    computeLeakCal(); saveLeak(); Serial.println(F("Точка 2 знята")); }
  if (req_calRst) { req_calRst = false; haveP1 = haveP2 = calValid = false;
                    E1_mV = S1_V = E2_mV = S2_V = 0; saveLeak();
                    Serial.println(F("Калібрування витоку скинуто")); }
  if (req_setR)   { req_setR = false; Rseries = req_Rval; saveLeak();
                    Serial.printf("R = %.3f МОм\n", Rseries / 1.0e6f); }
  if (req_setNorm){ req_setNorm = false; leakNormNA = req_NormVal; saveLeak();
                    Serial.printf("Норма витоку = %.3f нА\n", leakNormNA); }
  if (req_bufCap) { int i = req_bufCap - 1; req_bufCap = 0;
                    if (calValid && fabsf(gainS) > 1e-9f) {
                      bufVin[i] = (vs - zeroS) / gainS;   // мВ електрода у цьому буфері
                      bufHave[i] = true; computePhCal(); savePhCal();
                      Serial.printf("Буфер %d (pH %.2f): vin %.1f мВ\n", i + 1, BUF_PH[i], bufVin[i]);
                    } else Serial.println(F("Спершу калібрування плати (E->S)!")); }
  if (req_bufClr) { req_bufClr = false;
                    for (int i = 0; i < 3; i++) { bufHave[i] = false; bufVin[i] = 0; }
                    phCalValid = false; savePhCal(); Serial.println(F("Буферне калібр. скинуто")); }
  if (req_setBuf) { req_setBuf = false;
                    for (int i = 0; i < 3; i++) { bufVin[i] = req_bufV[i]; bufHave[i] = true; }
                    bufTempC = req_bufTcV; computePhCal(); savePhCal();
                    Serial.printf("Буфери відновлено: %.1f / %.1f / %.1f мВ @ %.1f°C\n",
                                  bufVin[0], bufVin[1], bufVin[2], bufTempC); }
  if (req_setTemp){ req_setTemp = false; bufTempC = req_TempVal; computePhCal(); savePhCal();
                    Serial.printf("T буферів = %.1f C\n", bufTempC); }
  if (req_calUsb) { req_calUsb = false;
                    if (g_usb_pin_mV > 50.0f && req_usbTrue > 0.5f) {
                      usbScale = req_usbTrue / (g_usb_pin_mV / 1000.0f); saveUsbCal();
                      Serial.printf("USB cal: %.3f В -> коеф. %.4f\n", req_usbTrue, usbScale);
                    } }
}

// ----------------------------- setup / loop ---------------------------------
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);         // не блокувати цикл до 100 мс, коли монітор закритий
  delay(250);                       // коротко: дати I2C-модулям і USB CDC устаканитись

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  loadCal();
  loadLeak();
  loadPhCal();
  loadUsbCal();

  // Вбудований АЦП для напруги USB (GPIO4): 12 біт, 11 дБ (~0..2.5 В на піні = USB/2.5)
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_USB_ADC, ADC_11db);

  // Дисплей і ADS — ПЕРШИМИ, щоб екран засвітився негайно після подачі живлення
  oled.setI2CAddress(ADDR_OLED << 1);     // u8g2 адреса 8-бітна
  for (uint8_t i = 0; i < 5 && !(oledOk && adsOk); i++) {
    if (!oledOk && i2cPresent(ADDR_OLED)) { oled.begin(); oled.setContrast(255); oledOk = true; }
    if (!adsOk)  adsOk = ads.begin(ADDR_ADS, &Wire);
    if (!(oledOk && adsOk)) delay(120);
  }
  if (adsOk) ads.setDataRate(RATE_ADS1115_250SPS);

  // ПЕРШИЙ КАДР одразу — стан мережі (екран світиться негайно після живлення)
  if (oledOk) drawStatus("WiFi...", WIFI_SSID, "connecting");
  i2cScan();
  if (!oledOk) Serial.println(F("OLED не знайдено (0x3C)"));
  if (!adsOk)  Serial.println(F("ADS1115 не знайдено (0x48)"));

  wifiBegin();   // Wi-Fi у ФОНІ — екран уже світиться, loop оновлює далі

  Serial.printf("offA=%.5f\n", offsetA);
  Serial.println(F("Готово. Команди: z=автонуль A, Z=скинути, r=показати кал."));
}

void loop() {
  static uint32_t last = 0;
  if (apMode) dnsServer.processNextRequest();   // captive-DNS лише в запасному AP-режимі
  if (millis() - last < PERIOD_MS) return;
  last = millis();

  wifiTick();   // фоновий стан Wi-Fi (під'єднання / запасна AP), не блокує

  // Поки під'єднуємось або кілька секунд після — показуємо стан мережі на OLED
  bool netScreen = (!staConnected && !apMode) || (millis() < connMsgUntil);
  if (netScreen) {
    if (!staConnected && !apMode) drawStatus("WiFi...", WIFI_SSID, "connecting");
    else drawStatus(staConnected ? "WiFi OK" : "AP mode", g_ip,
                    staConnected ? "calibrator.local" : AP_SSID);
  }

  // переініціалізація, якщо модуль ожив/під'єднали пізніше (обидва)
  if (!oledOk && i2cPresent(ADDR_OLED)) {
    oled.begin(); oled.setContrast(255); oledOk = true;
  }
  if (!adsOk) {
    adsOk = ads.begin(ADDR_ADS, &Wire);
    if (adsOk) ads.setDataRate(RATE_ADS1115_250SPS);
  }
  if (!adsOk) return;   // без ADS міряти нема чого

  float va = readA();
  float vsRaw = readS();
  // Легкий згладжувач (EMA) — прибирає миготіння молодшого розряду на екрані,
  // не сповільнюючи вимір. va/vs далі йдуть уже згладжені (екран, CSV, веб, точки).
  static float fVa = 0, fVs = 0; static bool fInit = false;
  if (!fInit) { fVa = va; fVs = vsRaw; fInit = true; }
  // помітна зміна -> одразу (без лагу); дрібне миготіння -> згладити EMA
  fVa = (fabsf(va - fVa) > SNAP_A) ? va : fVa + DISP_ALPHA * (va - fVa);
  fVs = (fabsf(vsRaw - fVs) > SNAP_S) ? vsRaw : fVs + DISP_ALPHA * (vsRaw - fVs);
  va = fVa; vsRaw = fVs;

  g_I_mA  = readI();   // струм плати — читаємо перед drawScreen, щоб на екрані було свіже
  g_usb_V = readUsb(); // напруга живлення USB (вбудований АЦП)
  float vs = vsRaw, shuntDrop = 0.0f;   // vs далі = вихід плати без падіння на шунті
  g_shuntValid = correctSForShunt(vsRaw, g_I_mA, vs, shuntDrop);

  if (!netScreen) drawScreen(va, vs);   // покази лише коли не показуємо стан мережі (власний вихід плати)

  // Оновити спільні значення для веб-інтерфейсу (читає таск AsyncTCP)
  float e_mV = va * 1000.0f;
  g_E_mV = e_mV;
  g_S_V  = vs;                    // поле s у /data — фізичний вихід плати
  g_S_raw_V = vsRaw;
  g_shuntDrop_V = shuntDrop;
  g_pH   = phFromA(va);
  if (calValid && fabsf(gainS) > 1e-9f) {
    float vin_mV = (vs - zeroS) / gainS;                          // вхід плати, мВ (шкала E)
    g_Vin_mV   = vin_mV;
    g_Ileak_nA = ((vin_mV - e_mV) * 1.0e-3f) / Rseries * 1.0e9f;  // (Vin-E)/R -> нА
    g_pH_real  = phCalValid ? phFromBuf(vin_mV)                   // буферна мапа, якщо є
                            : (PH_ISO - vin_mV / PH_SLOPE);       // інакше ідеальний Нернст
  } else {
    g_Vin_mV = 0.0f; g_Ileak_nA = 0.0f; g_pH_real = 7.0f;
  }

  // Виконати відкладені запити з веб-сторінки (I2C/NVS лише тут)
  processWebRequests(va, vs);

  // SSE: пушимо свіжий стан ~5×/с усім під'єднаним клієнтам (push, а не polling)
  static uint32_t lastEvt = 0;
  if (events.count() && millis() - lastEvt >= 200) {
    lastEvt = millis();
    char buf[1024]; buildDataJson(buf, sizeof(buf));
    events.send(buf, nullptr, millis());
  }

  // CSV у Serial: millis, A(мВ), S(В, з поправкою), pH, S сире(В), падіння шунта(В), I(мА)
  Serial.printf("%lu,%.1f,%.4f,%.2f,%.4f,%.4f,%.2f\n", (unsigned long)millis(), va * 1000.0f, vs,
                phFromA(va), vsRaw, shuntDrop, g_I_mA);

  handleSerial(va);
}
