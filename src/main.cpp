#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include "Stepper.h"
#include <esp_system.h>
#include <esp_wifi.h>
#include <mbedtls/aes.h>

// ===== Wi-Fi settings =====
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* AP_SSID = "MakerESP32-Stepper";
const char* AP_PASS = "stepper123";

// 32-byte AES-256 key for stored Wi-Fi config encryption.
// Replace this key before production use.
constexpr uint8_t AES_KEY[32] = {
  0x42, 0x75, 0x19, 0xA3, 0x5C, 0x77, 0xB1, 0xDE,
  0x88, 0x10, 0x2F, 0x61, 0x9A, 0xD4, 0x33, 0x07,
  0xC8, 0x54, 0x1D, 0xEE, 0x39, 0xAB, 0x6F, 0x92,
  0x04, 0x7B, 0xC1, 0x58, 0xE6, 0x20, 0x9D, 0x13
};

// ===== Board pin mapping =====
// Set to 1 for Stepper1 terminal block (M1+M2), or 2 for Stepper2 terminal block (M3+M4).
// If using Stepper2, keep board switch for M3/M4 in "Motor" mode.
#ifndef STEPPER_PORT
#define STEPPER_PORT 1
#endif

#if STEPPER_PORT == 1
constexpr uint8_t STEPPER_IN1 = 27;
constexpr uint8_t STEPPER_IN2 = 13;
constexpr uint8_t STEPPER_IN3 = 4;
constexpr uint8_t STEPPER_IN4 = 2;
#else
constexpr uint8_t STEPPER_IN1 = 17;
constexpr uint8_t STEPPER_IN2 = 12;
constexpr uint8_t STEPPER_IN3 = 15;
constexpr uint8_t STEPPER_IN4 = 14;
#endif

// Button inputs (choose pins suitable for your wiring)
constexpr uint8_t BTN1_PIN = 32;
constexpr uint8_t BTN2_PIN = 33;

// OLED I2C pins (Maker boards often use 4/5 for onboard OLED/I2C headers)
constexpr uint8_t I2C_SDA_PIN = 4;
constexpr uint8_t I2C_SCL_PIN = 5;
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr int OLED_W = 128;
constexpr int OLED_H = 64;
constexpr uint8_t RGB_LED_PIN = 16;
constexpr uint8_t RGB_LED_COUNT = 4;

WebServer server(80);
Preferences prefs;
TwoWire i2cBus = TwoWire(0);
Adafruit_SSD1306 display(OLED_W, OLED_H, &i2cBus, -1);
Adafruit_NeoPixel rgbLeds(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
constexpr int STEPPER_LIB_STEPS_PER_REV = 100;
Stepper stepper(STEPPER_LIB_STEPS_PER_REV, STEPPER_IN1, STEPPER_IN2, STEPPER_IN3, STEPPER_IN4);

// Runtime state
volatile bool button1Pressed = false;
volatile bool button2Pressed = false;
bool stepperEnabled = true;
bool stepperOutputsReleased = false;
long stepperPosition = 0;
long targetPosition = 0;
float speedStepsPerSec = 100.0f;
float accelStepsPerSec2 = 80.0f;
unsigned long lastDisplayMs = 0;
constexpr long STEPS_PER_INDEXER_REV = 200L * 20L * 40L;
int numberOfGears = 40;
long ticksPerGear = 4000;
constexpr uint16_t SINGLE_STEP_PULSE_US = 2500;
constexpr uint16_t SINGLE_STEP_SETTLE_MS = 25;
unsigned long lastSingleStepMs = 0;

// Debounce state
bool lastBtn1Read = HIGH;
bool lastBtn2Read = HIGH;
unsigned long lastBtn1EdgeMs = 0;
unsigned long lastBtn2EdgeMs = 0;
constexpr unsigned long DEBOUNCE_MS = 30;

String wifiMode = "STA";
IPAddress ipAddr;
IPAddress bootIpAddr;
bool bootIpCaptured = false;
bool hasStoredNetworkConfig = false;

struct NetworkConfig {
  String ssid;
  String password;
  String staticIp;
  String gateway;
  String netmask;
};

NetworkConfig savedConfig;

// Forward declarations for helper functions used before their definitions.
void hardDisableStepperPins();
void hardEnableStepperPins();
void applyStepperSpeed();

void recalcIndexerTicks() {
  if (numberOfGears < 1) {
    numberOfGears = 1;
  }
  ticksPerGear = lround((200.0f * 20.0f * 40.0f) / static_cast<float>(numberOfGears));
}

float getIndexerDegrees() {
  long modPos = stepperPosition % STEPS_PER_INDEXER_REV;
  if (modPos < 0) {
    modPos += STEPS_PER_INDEXER_REV;
  }
  return (static_cast<float>(modPos) * 360.0f) / static_cast<float>(STEPS_PER_INDEXER_REV);
}

void applyStepperSpeed() {
  // Stepper::setSpeed expects RPM for configured steps/rev.
  long rpm = lround((speedStepsPerSec * 60.0f) / static_cast<float>(STEPPER_LIB_STEPS_PER_REV));
  if (rpm < 1) {
    rpm = 1;
  }
  stepper.setSpeed(rpm);
}

void runStepperToTargetOneStep() {
  if (!stepperEnabled) {
    return;
  }
  if (stepperOutputsReleased && stepperPosition != targetPosition) {
    hardEnableStepperPins();
    applyStepperSpeed();
    Serial.println("[STEP] outputs re-enabled for queued motion");
  }
  if (stepperPosition == targetPosition) {
    return;
  }
  int dir = (targetPosition > stepperPosition) ? 1 : -1;
  stepper.step(dir);
  stepperPosition += dir;
}

const char* wifiDisconnectReasonName(uint8_t reason) {
  switch (reason) {
    case 1: return "UNSPECIFIED";
    case 2: return "AUTH_EXPIRE";
    case 3: return "AUTH_LEAVE";
    case 4: return "ASSOC_EXPIRE";
    case 5: return "ASSOC_TOOMANY";
    case 6: return "NOT_AUTHED";
    case 7: return "NOT_ASSOCED";
    case 8: return "ASSOC_LEAVE";
    case 9: return "ASSOC_NOT_AUTHED";
    case 10: return "DISASSOC_PWRCAP_BAD";
    case 11: return "DISASSOC_SUPCHAN_BAD";
    case 13: return "IE_INVALID";
    case 15: return "4WAY_HANDSHAKE_TIMEOUT";
    case 23: return "802_1X_AUTH_FAILED";
    case 200: return "BEACON_TIMEOUT";
    case 201: return "NO_AP_FOUND";
    case 202: return "AUTH_FAIL";
    case 203: return "ASSOC_FAIL";
    case 204: return "HANDSHAKE_TIMEOUT";
    default: return "UNKNOWN_REASON";
  }
}

const char* authModeName(wifi_auth_mode_t mode) {
  switch (mode) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA_PSK";
    case WIFI_AUTH_WPA2_PSK: return "WPA2_PSK";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA_WPA2_PSK";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2_ENTERPRISE";
    case WIFI_AUTH_WPA3_PSK: return "WPA3_PSK";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2_WPA3_PSK";
    case WIFI_AUTH_WAPI_PSK: return "WAPI_PSK";
    default: return "AUTH_UNKNOWN";
  }
}

const char* wifiStatusName(wl_status_t s) {
  switch (s) {
    case WL_NO_SHIELD: return "WL_NO_SHIELD";
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED: return "WL_DISCONNECTED";
    default: return "WL_UNKNOWN";
  }
}

void setFirstLedColor(uint8_t r, uint8_t g, uint8_t b) {
  rgbLeds.setPixelColor(0, rgbLeds.Color(r, g, b));
  rgbLeds.show();
}

void hardDisableStepperPins() {
  // Keep H-bridge inputs in a defined low state to prevent floating/ticking.
  pinMode(STEPPER_IN1, OUTPUT);
  pinMode(STEPPER_IN2, OUTPUT);
  pinMode(STEPPER_IN3, OUTPUT);
  pinMode(STEPPER_IN4, OUTPUT);
  digitalWrite(STEPPER_IN1, LOW);
  digitalWrite(STEPPER_IN2, LOW);
  digitalWrite(STEPPER_IN3, LOW);
  digitalWrite(STEPPER_IN4, LOW);
  stepperOutputsReleased = true;
}

void hardEnableStepperPins() {
  pinMode(STEPPER_IN1, OUTPUT);
  pinMode(STEPPER_IN2, OUTPUT);
  pinMode(STEPPER_IN3, OUTPUT);
  pinMode(STEPPER_IN4, OUTPUT);
  stepperOutputsReleased = false;
}

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
    Serial.println("[WIFI_EVT] STA_CONNECTED");
  } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    Serial.print("[WIFI_EVT] STA_GOT_IP ");
    Serial.println(IPAddress(info.got_ip.ip_info.ip.addr));
  } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    uint8_t reason = info.wifi_sta_disconnected.reason;
    Serial.print("[WIFI_EVT] STA_DISCONNECTED reason=");
    Serial.print(reason);
    Serial.print(" ");
    Serial.println(wifiDisconnectReasonName(reason));
  }
}

void debugScanForTargetSsid(const String& targetSsid) {
  Serial.print("[WIFI] scanning for SSID=");
  Serial.println(targetSsid);
  int n = WiFi.scanNetworks(false, true);
  if (n < 0) {
    Serial.println("[WIFI] scan failed");
    return;
  }
  Serial.print("[WIFI] scan count=");
  Serial.println(n);
  bool found = false;
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    if (ssid == targetSsid) {
      found = true;
      Serial.print("[WIFI] target found RSSI=");
      Serial.print(WiFi.RSSI(i));
      Serial.print("dBm channel=");
      Serial.print(WiFi.channel(i));
      Serial.print(" auth=");
      Serial.println(authModeName(static_cast<wifi_auth_mode_t>(WiFi.encryptionType(i))));
    }
  }
  if (!found) {
    Serial.println("[WIFI] target SSID not found in scan");
  }
  WiFi.scanDelete();
}

String toHex(const uint8_t* data, size_t len) {
  static const char* HEX_CHARS = "0123456789ABCDEF";
  String out;
  out.reserve(len * 2);
  for (size_t i = 0; i < len; i++) {
    out += HEX_CHARS[(data[i] >> 4) & 0x0F];
    out += HEX_CHARS[data[i] & 0x0F];
  }
  return out;
}

bool fromHexNibble(char c, uint8_t& out) {
  if (c >= '0' && c <= '9') {
    out = static_cast<uint8_t>(c - '0');
    return true;
  }
  if (c >= 'a' && c <= 'f') {
    out = static_cast<uint8_t>(10 + (c - 'a'));
    return true;
  }
  if (c >= 'A' && c <= 'F') {
    out = static_cast<uint8_t>(10 + (c - 'A'));
    return true;
  }
  return false;
}

bool fromHex(const String& hex, uint8_t* out, size_t outLen) {
  if (hex.length() != outLen * 2) {
    return false;
  }
  for (size_t i = 0; i < outLen; i++) {
    uint8_t hi = 0;
    uint8_t lo = 0;
    if (!fromHexNibble(hex[i * 2], hi) || !fromHexNibble(hex[i * 2 + 1], lo)) {
      return false;
    }
    out[i] = static_cast<uint8_t>((hi << 4) | lo);
  }
  return true;
}

String encryptAesCbc(const String& plaintext) {
  const size_t blockSize = 16;
  const size_t inLen = plaintext.length();
  const uint8_t pad = static_cast<uint8_t>(blockSize - (inLen % blockSize));
  const size_t paddedLen = inLen + pad;

  uint8_t* encBuf = static_cast<uint8_t*>(malloc(paddedLen));
  uint8_t* outBuf = static_cast<uint8_t*>(malloc(paddedLen));
  if (encBuf == nullptr || outBuf == nullptr) {
    if (encBuf != nullptr) {
      free(encBuf);
    }
    if (outBuf != nullptr) {
      free(outBuf);
    }
    return String();
  }

  memcpy(encBuf, plaintext.c_str(), inLen);
  for (size_t i = 0; i < pad; i++) {
    encBuf[inLen + i] = pad;
  }

  uint8_t iv[16];
  for (size_t i = 0; i < sizeof(iv); i += 4) {
    uint32_t r = esp_random();
    iv[i] = static_cast<uint8_t>(r & 0xFF);
    iv[i + 1] = static_cast<uint8_t>((r >> 8) & 0xFF);
    iv[i + 2] = static_cast<uint8_t>((r >> 16) & 0xFF);
    iv[i + 3] = static_cast<uint8_t>((r >> 24) & 0xFF);
  }
  uint8_t ivWork[16];
  memcpy(ivWork, iv, sizeof(iv));

  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  if (mbedtls_aes_setkey_enc(&aes, AES_KEY, 256) != 0 ||
      mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, paddedLen, ivWork, encBuf, outBuf) != 0) {
    mbedtls_aes_free(&aes);
    free(encBuf);
    free(outBuf);
    return String();
  }
  mbedtls_aes_free(&aes);
  free(encBuf);

  uint8_t* payload = static_cast<uint8_t*>(malloc(sizeof(iv) + paddedLen));
  if (payload == nullptr) {
    free(outBuf);
    return String();
  }
  memcpy(payload, iv, sizeof(iv));
  memcpy(payload + sizeof(iv), outBuf, paddedLen);
  free(outBuf);

  String hexPayload = toHex(payload, sizeof(iv) + paddedLen);
  free(payload);
  return hexPayload;
}

bool decryptAesCbc(const String& cipherHex, String& plaintextOut) {
  const size_t totalBytes = cipherHex.length() / 2;
  if (cipherHex.length() % 2 != 0 || totalBytes < 32 || ((totalBytes - 16) % 16) != 0) {
    return false;
  }

  uint8_t* raw = static_cast<uint8_t*>(malloc(totalBytes));
  if (raw == nullptr) {
    return false;
  }
  if (!fromHex(cipherHex, raw, totalBytes)) {
    free(raw);
    return false;
  }

  uint8_t iv[16];
  memcpy(iv, raw, sizeof(iv));
  const size_t cipherLen = totalBytes - sizeof(iv);
  uint8_t* plain = static_cast<uint8_t*>(malloc(cipherLen));
  if (plain == nullptr) {
    free(raw);
    return false;
  }

  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  bool ok = (mbedtls_aes_setkey_dec(&aes, AES_KEY, 256) == 0) &&
            (mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, cipherLen, iv, raw + sizeof(iv), plain) == 0);
  mbedtls_aes_free(&aes);
  free(raw);
  if (!ok) {
    free(plain);
    return false;
  }

  uint8_t pad = plain[cipherLen - 1];
  if (pad == 0 || pad > 16 || pad > cipherLen) {
    free(plain);
    return false;
  }
  for (size_t i = 0; i < pad; i++) {
    if (plain[cipherLen - 1 - i] != pad) {
      free(plain);
      return false;
    }
  }

  const size_t plainLen = cipherLen - pad;
  plaintextOut = "";
  plaintextOut.reserve(plainLen);
  for (size_t i = 0; i < plainLen; i++) {
    plaintextOut += static_cast<char>(plain[i]);
  }
  free(plain);
  return true;
}

String serializeNetworkConfig(const NetworkConfig& cfg) {
  String plain;
  plain.reserve(256);
  plain += cfg.ssid;
  plain += '\n';
  plain += cfg.password;
  plain += '\n';
  plain += cfg.staticIp;
  plain += '\n';
  plain += cfg.gateway;
  plain += '\n';
  plain += cfg.netmask;
  return plain;
}

bool parseNetworkConfig(const String& plain, NetworkConfig& cfg) {
  int p1 = plain.indexOf('\n');
  if (p1 < 0) return false;
  int p2 = plain.indexOf('\n', p1 + 1);
  if (p2 < 0) return false;
  int p3 = plain.indexOf('\n', p2 + 1);
  if (p3 < 0) return false;
  int p4 = plain.indexOf('\n', p3 + 1);
  if (p4 < 0) return false;

  cfg.ssid = plain.substring(0, p1);
  cfg.password = plain.substring(p1 + 1, p2);
  cfg.staticIp = plain.substring(p2 + 1, p3);
  cfg.gateway = plain.substring(p3 + 1, p4);
  cfg.netmask = plain.substring(p4 + 1);
  return true;
}

bool saveNetworkConfig(const NetworkConfig& cfg) {
  Serial.println("[CFG] saveNetworkConfig() called");
  Serial.print("[CFG] ssid=");
  Serial.println(cfg.ssid);
  Serial.print("[CFG] staticIp=");
  Serial.println(cfg.staticIp);
  Serial.print("[CFG] gateway=");
  Serial.println(cfg.gateway);
  Serial.print("[CFG] netmask=");
  Serial.println(cfg.netmask);
  Serial.print("[CFG] passwordLen=");
  Serial.println(cfg.password.length());

  String encrypted = encryptAesCbc(serializeNetworkConfig(cfg));
  if (encrypted.length() == 0) {
    Serial.println("[CFG] encryption failed");
    return false;
  }

  Serial.print("[CFG] encrypted payload chars=");
  Serial.println(encrypted.length());
  prefs.begin("netcfg", false);
  size_t written = prefs.putString("cfg", encrypted);
  prefs.end();
  Serial.print("[CFG] NVS putString bytes=");
  Serial.println(written);
  return written > 0;
}

bool loadNetworkConfig(NetworkConfig& cfg) {
  Serial.println("[CFG] loadNetworkConfig() called");
  prefs.begin("netcfg", true);
  String encrypted = prefs.getString("cfg", "");
  prefs.end();
  if (encrypted.length() == 0) {
    Serial.println("[CFG] no encrypted config found");
    return false;
  }

  Serial.print("[CFG] encrypted payload chars=");
  Serial.println(encrypted.length());

  String plain;
  if (!decryptAesCbc(encrypted, plain)) {
    Serial.println("[CFG] decrypt failed");
    return false;
  }
  bool ok = parseNetworkConfig(plain, cfg);
  if (!ok) {
    Serial.println("[CFG] parse plaintext config failed");
    return false;
  }

  Serial.println("[CFG] config loaded");
  Serial.print("[CFG] ssid=");
  Serial.println(cfg.ssid);
  Serial.print("[CFG] staticIp=");
  Serial.println(cfg.staticIp);
  Serial.print("[CFG] gateway=");
  Serial.println(cfg.gateway);
  Serial.print("[CFG] netmask=");
  Serial.println(cfg.netmask);
  Serial.print("[CFG] passwordLen=");
  Serial.println(cfg.password.length());
  return true;
}

String htmlEscape(const String& in) {
  String out;
  out.reserve(in.length() + 16);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '&') out += F("&amp;");
    else if (c == '<') out += F("&lt;");
    else if (c == '>') out += F("&gt;");
    else if (c == '"') out += F("&quot;");
    else if (c == '\'') out += F("&#39;");
    else out += c;
  }
  return out;
}

String htmlPage() {
  String h;
  h.reserve(7600);
  h += F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>");
  h += F("<title>Divider Indexer Controller</title><style>");
  h += F(":root{--bg1:#f4f7fb;--bg2:#e8eef8;--card:#ffffff;--ink:#1e2b3a;--muted:#5c6b7a;--line:#d9e3ef;--accent:#007f73;--accent2:#0a4f8a;}");
  h += F("*{box-sizing:border-box}body{margin:0;font-family:'Trebuchet MS','Segoe UI',sans-serif;background:linear-gradient(180deg,var(--bg1),var(--bg2));color:var(--ink)}");
  h += F(".shell{max-width:980px;margin:0 auto;padding:14px}.title{margin:6px 0 14px;font-size:clamp(1.2rem,4.8vw,2rem);letter-spacing:.02em}");
  h += F(".grid{display:grid;grid-template-columns:1.1fr 1fr;gap:12px}.card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:12px;box-shadow:0 4px 14px rgba(20,40,60,.08)}");
  h += F(".k{font-size:.82rem;color:var(--muted)}.v{font-size:1.05rem;font-weight:700}.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:8px 0}");
  h += F("button,input{font-size:15px;border-radius:10px;border:1px solid #c8d6e8;padding:10px 12px}button{background:#fff;cursor:pointer}button.primary{background:var(--accent);color:#fff;border-color:var(--accent)}");
  h += F("button.secondary{background:var(--accent2);color:#fff;border-color:var(--accent2)}button.state-on{background:#1f9d55;color:#fff;border-color:#1f9d55}button.state-off{background:#b45309;color:#fff;border-color:#b45309}");
  h += F("input{min-width:120px}.status{white-space:pre-line;font-family:ui-monospace,Consolas,monospace;font-size:.84rem}");
  h += F(".dialWrap{display:flex;flex-direction:column;align-items:center;gap:6px}.dialDeg{font-size:1.3rem;font-weight:700}.tiny{font-size:.8rem;color:var(--muted)}");
  h += F(".net{margin-top:8px;padding:10px;border:1px dashed #9ab2cc;border-radius:12px;background:#f8fbff}");
  h += F("@media (max-width:800px){.grid{grid-template-columns:1fr}.shell{padding:10px}button,input{flex:1}}");
  h += F("</style></head><body><div class='shell'><h1 class='title'>Divider Indexer Controller</h1><div class='grid'>");
  h += F("<div class='card'><div class='dialWrap'>");
  h += F("<svg id='dialSvg' width='250' height='250' viewBox='0 0 250 250' aria-label='Indexer dial'>");
  h += F("<defs><linearGradient id='g' x1='0' y1='0' x2='1' y2='1'><stop offset='0%' stop-color='#f8fcff'/><stop offset='100%' stop-color='#d9e8f7'/></linearGradient></defs>");
  h += F("<circle cx='125' cy='125' r='110' fill='url(#g)' stroke='#8ba7c4' stroke-width='2'/>");
  h += F("<circle cx='125' cy='125' r='85' fill='none' stroke='#b8cadf' stroke-width='1.5'/>");
  h += F("<line x1='125' y1='18' x2='125' y2='36' stroke='#445d78' stroke-width='3'/>");
  h += F("<text x='125' y='52' text-anchor='middle' font-size='14' fill='#2a425c'>0</text>");
  h += F("<text x='197' y='130' text-anchor='middle' font-size='14' fill='#2a425c'>90</text>");
  h += F("<text x='125' y='213' text-anchor='middle' font-size='14' fill='#2a425c'>180</text>");
  h += F("<text x='53' y='130' text-anchor='middle' font-size='14' fill='#2a425c'>270</text>");
  h += F("<line id='needle' x1='125' y1='125' x2='125' y2='34' stroke='#c62828' stroke-width='4' stroke-linecap='round'/>");
  h += F("<circle cx='125' cy='125' r='7' fill='#17324f'/></svg>");
  h += F("<div class='dialDeg'><span id='deg'>0.000</span>&deg;</div><div class='tiny'>Indexer Angle</div></div>");
  h += F("</div>");
  h += F("<div class='card'><div class='row'><div><div class='k'>IP</div><div class='v' id='ip'>-</div></div><div><div class='k'>Mode</div><div class='v' id='mode'>-</div></div></div>");
  h += F("<div class='status' id='status'>Loading...</div>");
  h += F("<div class='row'><button id='enableBtn' onclick='cmd(\"/stepper/enable\")'>Enable</button><button id='disableBtn' onclick='cmd(\"/stepper/disable\")'>Disable</button><button onclick='cmd(\"/stepper/stop\")'>Stop</button></div>");
  h += F("<div class='row'><button onclick='singleStep(1)'>+1 Step</button><button onclick='singleStep(-1)'>-1 Step</button></div>");
  h += F("<div class='row'><input id='speed' type='number' value='100' min='5' max='1500'><button class='secondary' onclick='setSpeed()'>Set Speed</button></div>");
  h += F("<div class='row'><input id='accel' type='number' value='80' min='5' max='3000'><button class='secondary' onclick='setAccel()'>Set Accel</button></div>");
  h += F("<div class='row'><input id='gears' type='number' value='40' min='1'><button class='secondary' onclick='setGears()'>Set Gears</button></div>");
  h += F("<div class='row'><button class='primary' onclick='indexStep(1)'>+1 Gear</button><button class='primary' onclick='indexStep(-1)'>-1 Gear</button></div>");
  h += F("<div class='row'><input id='steps' type='number' value='200'><button onclick='move(1)'>Move +Steps</button><button onclick='move(-1)'>Move -Steps</button></div>");
  h += F("<div class='tiny'>ticks_per_gear = round((200 * 20 * 40) / gears)</div>");
  h += F("</div>");
  if (wifiMode == "AP") {
    h += F("<div class='card net' style='grid-column:1 / -1'><h3>STA Network Setup (Saved Encrypted)</h3>");
    h += F("<div class='row'><input id='ssid' placeholder='SSID' value='");
    h += htmlEscape(savedConfig.ssid);
    h += F("'><input id='password' type='password' placeholder='Password' value='");
    h += htmlEscape(savedConfig.password);
    h += F("'></div>");
    h += F("<div class='row'><input id='ip' placeholder='Static IP' value='");
    h += htmlEscape(savedConfig.staticIp);
    h += F("'><input id='gateway' placeholder='Gateway' value='");
    h += htmlEscape(savedConfig.gateway);
    h += F("'><input id='netmask' placeholder='Netmask' value='");
    h += htmlEscape(savedConfig.netmask);
    h += F("'></div>");
    h += F("<div class='row'><button class='secondary' onclick='saveNetwork()'>Save + Reboot</button></div></div>");
  }
  h += F("</div>");
  h += F("<script>");
  h += F("async function cmd(u){await fetch(u,{method:'POST'});refresh();}");
  h += F("async function singleStep(dir){const r=await fetch('/stepper/single?dir='+dir,{method:'POST'});if(!r.ok){alert(await r.text());}refresh();}");
  h += F("async function move(dir){const s=document.getElementById('steps').value||200;await fetch('/stepper/move?steps='+s+'&dir='+dir,{method:'POST'});refresh();}");
  h += F("async function setSpeed(){const s=document.getElementById('speed').value||100;await fetch('/stepper/speed?value='+s,{method:'POST'});refresh();}");
  h += F("async function setAccel(){const a=document.getElementById('accel').value||80;await fetch('/stepper/accel?value='+a,{method:'POST'});refresh();}");
  h += F("async function indexStep(dir){await fetch('/indexer/step?dir='+dir,{method:'POST'});refresh();}");
  h += F("async function setGears(){const g=document.getElementById('gears').value||40;await fetch('/indexer/set_gears?value='+g,{method:'POST'});refresh();}");
  h += F("async function saveNetwork(){const p=new URLSearchParams({ssid:document.getElementById('ssid').value,password:document.getElementById('password').value,ip:document.getElementById('ip').value,gateway:document.getElementById('gateway').value,netmask:document.getElementById('netmask').value});");
  h += F("const r=await fetch('/config/network',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p});alert(await r.text());}");
  h += F("function renderDial(deg){const needle=document.getElementById('needle');needle.setAttribute('transform',`rotate(${deg} 125 125)`);document.getElementById('deg').innerText=Number(deg).toFixed(3);}");
  h += F("function setIfIdle(id,val){const el=document.getElementById(id);if(document.activeElement!==el){el.value=val;}}");
  h += F("async function refresh(){const r=await fetch('/status');const j=await r.json();");
  h += F("document.getElementById('ip').innerText=j.ip;document.getElementById('mode').innerText=j.wifiMode;setIfIdle('gears',j.gears);setIfIdle('speed',j.speed);setIfIdle('accel',j.accel);");
  h += F("const en=document.getElementById('enableBtn');const dis=document.getElementById('disableBtn');if(j.enabled){en.className='state-on';dis.className='';}else{en.className='';dis.className='state-off';}");
  h += F("document.getElementById('status').innerText=`Boot IP ${j.bootIp||'-'}\\nPos ${j.position} -> ${j.target} | Enabled ${j.enabled}\\nSpeed ${j.speed} st/s | Accel ${j.accel}\\nGears ${j.gears} | Ticks/Gear ${j.ticksPerGear}\\nB1 ${j.b1} B2 ${j.b2}`;");
  h += F("renderDial(j.indexerDeg);}");
  h += F("setInterval(refresh,1000);refresh();");
  h += F("</script></body></html>");
  return h;
}

void sendJsonStatus() {
  String json = "{";
  json += "\"ip\":\"" + ipAddr.toString() + "\",";
  json += "\"bootIp\":\"" + (bootIpCaptured ? bootIpAddr.toString() : String("")) + "\",";
  json += "\"wifiMode\":\"" + wifiMode + "\",";
  json += "\"enabled\":" + String(stepperEnabled ? "true" : "false") + ",";
  json += "\"position\":" + String(stepperPosition) + ",";
  json += "\"target\":" + String(targetPosition) + ",";
  json += "\"indexerDeg\":" + String(getIndexerDegrees(), 3) + ",";
  json += "\"gears\":" + String(numberOfGears) + ",";
  json += "\"ticksPerGear\":" + String(ticksPerGear) + ",";
  json += "\"speed\":" + String(speedStepsPerSec, 1) + ",";
  json += "\"accel\":" + String(accelStepsPerSec2, 1) + ",";
  json += "\"b1\":" + String(button1Pressed ? "true" : "false") + ",";
  json += "\"b2\":" + String(button2Pressed ? "true" : "false") + ",";
  json += "\"hasCfg\":" + String(hasStoredNetworkConfig ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleRoot() { server.send(200, "text/html", htmlPage()); }
void handleStatus() { sendJsonStatus(); }

void handleStepperEnable() {
  Serial.println("[HTTP] /stepper/enable");
  hardEnableStepperPins();
  stepperEnabled = true;
  applyStepperSpeed();
  Serial.println("[STEP] enabled=true");
  server.send(200, "text/plain", "OK");
}

void handleStepperDisable() {
  Serial.println("[HTTP] /stepper/disable");
  stepperEnabled = false;
  targetPosition = stepperPosition;
  hardDisableStepperPins();
  Serial.println("[STEP] enabled=false; stop requested");
  server.send(200, "text/plain", "OK");
}

void handleStepperStop() {
  targetPosition = stepperPosition;
  server.send(200, "text/plain", "OK");
}

void handleStepperMove() {
  Serial.println("[HTTP] /stepper/move");
  if (!stepperEnabled) {
    Serial.println("[STEP] move rejected: disabled");
    server.send(409, "text/plain", "Stepper is disabled");
    return;
  }
  if (!server.hasArg("steps")) {
    server.send(400, "text/plain", "Missing steps");
    return;
  }
  long steps = server.arg("steps").toInt();
  int dir = server.hasArg("dir") ? server.arg("dir").toInt() : 1;
  if (steps < 0) {
    steps = -steps;
  }
  if (dir < 0) {
    steps = -steps;
  }

  if (stepperOutputsReleased) {
    hardEnableStepperPins();
    applyStepperSpeed();
  }
  targetPosition = stepperPosition + steps;
  Serial.print("[STEP] move target=");
  Serial.println(targetPosition);
  server.send(200, "text/plain", "OK");
}

void handleStepperSingleStep() {
  Serial.println("[HTTP] /stepper/single");
  if (!stepperEnabled) {
    Serial.println("[STEP] single step rejected: disabled");
    server.send(409, "text/plain", "Stepper is disabled");
    return;
  }
  unsigned long now = millis();
  if (now - lastSingleStepMs < SINGLE_STEP_SETTLE_MS) {
    Serial.println("[STEP] single step rejected: still settling");
    server.send(429, "text/plain", "Motor settling, try again");
    return;
  }
  int dir = server.hasArg("dir") ? server.arg("dir").toInt() : 1;
  int delta = (dir < 0) ? -1 : 1;
  targetPosition = stepperPosition;
  hardEnableStepperPins();
  stepper.step(delta);
  stepperPosition += delta;
  targetPosition = stepperPosition;
  delayMicroseconds(SINGLE_STEP_PULSE_US);
  delay(SINGLE_STEP_SETTLE_MS);
  hardDisableStepperPins();
  lastSingleStepMs = millis();
  Serial.print("[STEP] single step dir=");
  Serial.print(dir);
  Serial.print(" pos=");
  Serial.print(stepperPosition);
  Serial.print(" pulse_us=");
  Serial.print(SINGLE_STEP_PULSE_US);
  Serial.print(" settle_ms=");
  Serial.println(SINGLE_STEP_SETTLE_MS);
  server.send(200, "text/plain", "OK");
}

void handleStepperSpeed() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }
  float v = server.arg("value").toFloat();
  if (v < 5.0f) {
    v = 5.0f;
  }
  if (v > 1500.0f) {
    v = 1500.0f;
  }
  speedStepsPerSec = v;
  applyStepperSpeed();
  Serial.print("[STEP] speed=");
  Serial.println(speedStepsPerSec);
  server.send(200, "text/plain", "OK");
}

void handleStepperAccel() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }
  float a = server.arg("value").toFloat();
  if (a < 5.0f) {
    a = 5.0f;
  }
  if (a > 3000.0f) {
    a = 3000.0f;
  }
  accelStepsPerSec2 = a;
  Serial.print("[STEP] accel=");
  Serial.print(accelStepsPerSec2);
  Serial.println(" (Stepper library has no acceleration ramp)");
  server.send(200, "text/plain", "OK");
}

void handleIndexerStep() {
  Serial.println("[HTTP] /indexer/step");
  if (!stepperEnabled) {
    Serial.println("[STEP] index step rejected: disabled");
    server.send(409, "text/plain", "Stepper is disabled");
    return;
  }
  int dir = server.hasArg("dir") ? server.arg("dir").toInt() : 1;
  long delta = (dir < 0) ? -ticksPerGear : ticksPerGear;
  if (stepperOutputsReleased) {
    hardEnableStepperPins();
    applyStepperSpeed();
  }
  targetPosition = stepperPosition + delta;
  Serial.print("[STEP] index dir=");
  Serial.print(dir);
  Serial.print(" ticksPerGear=");
  Serial.print(ticksPerGear);
  Serial.print(" target=");
  Serial.println(targetPosition);
  server.send(200, "text/plain", "OK");
}

void handleIndexerSetGears() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }
  int nextGears = server.arg("value").toInt();
  if (nextGears < 1) {
    server.send(400, "text/plain", "Gears must be >= 1");
    return;
  }
  numberOfGears = nextGears;
  recalcIndexerTicks();
  server.send(200, "text/plain", "OK");
}

bool parseIpArg(const String& s, IPAddress& out) {
  return out.fromString(s);
}

void handleSaveNetworkConfig() {
  Serial.println("[HTTP] /config/network POST");
  if (wifiMode != "AP") {
    Serial.println("[HTTP] rejected: not AP mode");
    server.send(403, "text/plain", "Only available in AP mode");
    return;
  }
  if (!server.hasArg("ssid") || !server.hasArg("password") || !server.hasArg("ip") ||
      !server.hasArg("gateway") || !server.hasArg("netmask")) {
    Serial.println("[HTTP] rejected: missing fields");
    server.send(400, "text/plain", "Missing fields");
    return;
  }

  NetworkConfig nextCfg;
  nextCfg.ssid = server.arg("ssid");
  nextCfg.password = server.arg("password");
  nextCfg.staticIp = server.arg("ip");
  nextCfg.gateway = server.arg("gateway");
  nextCfg.netmask = server.arg("netmask");

  Serial.print("[HTTP] ssid=");
  Serial.println(nextCfg.ssid);
  Serial.print("[HTTP] passwordLen=");
  Serial.println(nextCfg.password.length());
  Serial.print("[HTTP] ip=");
  Serial.println(nextCfg.staticIp);
  Serial.print("[HTTP] gw=");
  Serial.println(nextCfg.gateway);
  Serial.print("[HTTP] mask=");
  Serial.println(nextCfg.netmask);

  IPAddress ip;
  IPAddress gw;
  IPAddress mask;
  if (nextCfg.ssid.length() == 0 || !parseIpArg(nextCfg.staticIp, ip) ||
      !parseIpArg(nextCfg.gateway, gw) || !parseIpArg(nextCfg.netmask, mask)) {
    Serial.println("[HTTP] rejected: invalid SSID/IP settings");
    server.send(400, "text/plain", "Invalid SSID or IP settings");
    return;
  }

  if (!saveNetworkConfig(nextCfg)) {
    Serial.println("[HTTP] saveNetworkConfig failed");
    server.send(500, "text/plain", "Failed to save encrypted config");
    return;
  }

  savedConfig = nextCfg;
  hasStoredNetworkConfig = true;
  Serial.println("[HTTP] config saved; rebooting");
  server.send(200, "text/plain", "Saved encrypted config. Rebooting...");
  delay(350);
  ESP.restart();
}

void setupWeb() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/stepper/enable", HTTP_POST, handleStepperEnable);
  server.on("/stepper/disable", HTTP_POST, handleStepperDisable);
  server.on("/stepper/stop", HTTP_POST, handleStepperStop);
  server.on("/stepper/move", HTTP_POST, handleStepperMove);
  server.on("/stepper/single", HTTP_POST, handleStepperSingleStep);
  server.on("/stepper/speed", HTTP_POST, handleStepperSpeed);
  server.on("/stepper/accel", HTTP_POST, handleStepperAccel);
  server.on("/indexer/step", HTTP_POST, handleIndexerStep);
  server.on("/indexer/set_gears", HTTP_POST, handleIndexerSetGears);
  server.on("/config/network", HTTP_POST, handleSaveNetworkConfig);
  server.begin();
}

void setupWifi() {
  Serial.println("[WIFI] setupWifi() start");
  String useSsid = WIFI_SSID;
  String usePass = WIFI_PASS;
  bool usingStoredCreds = false;

  IPAddress staticIp;
  IPAddress staticGw;
  IPAddress staticMask;
  bool useStatic = false;

  if (hasStoredNetworkConfig) {
    if (savedConfig.ssid.length() > 0) {
      useSsid = savedConfig.ssid;
      usePass = savedConfig.password;
      usingStoredCreds = true;
      useStatic = parseIpArg(savedConfig.staticIp, staticIp) &&
                  parseIpArg(savedConfig.gateway, staticGw) &&
                  parseIpArg(savedConfig.netmask, staticMask);
    }
  }

  Serial.print("[WIFI] usingStoredCreds=");
  Serial.println(usingStoredCreds ? "true" : "false");
  Serial.print("[WIFI] target SSID=");
  Serial.println(useSsid);
  Serial.print("[WIFI] passwordLen=");
  Serial.println(usePass.length());
  Serial.print("[WIFI] useStatic=");
  Serial.println(useStatic ? "true" : "false");
  if (useStatic) {
    Serial.print("[WIFI] staticIp=");
    Serial.println(staticIp);
    Serial.print("[WIFI] gateway=");
    Serial.println(staticGw);
    Serial.print("[WIFI] netmask=");
    Serial.println(staticMask);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  delay(100);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  debugScanForTargetSsid(useSsid);
  setFirstLedColor(255, 0, 0);  // Red: trying to connect to configured AP
  if (useStatic) {
    bool cfgOk = WiFi.config(staticIp, staticGw, staticMask, staticGw, staticGw);
    Serial.print("[WIFI] WiFi.config result=");
    Serial.println(cfgOk ? "true" : "false");
  }
  WiFi.begin(useSsid.c_str(), usePass.c_str());
  Serial.println("[WIFI] WiFi.begin() called");

  unsigned long start = millis();
  unsigned long lastLog = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - start < 12000) {
    if (millis() - lastLog >= 1000) {
      wl_status_t s = WiFi.status();
      Serial.print("[WIFI] waiting... status=");
      Serial.print(static_cast<int>(s));
      Serial.print(" ");
      Serial.println(wifiStatusName(s));
      lastLog = millis();
    }
    delay(200);
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiMode = "STA";
    ipAddr = WiFi.localIP();
    bootIpAddr = ipAddr;
    bootIpCaptured = true;
    setFirstLedColor(0, 255, 0);  // Green: connected to AP
    Serial.print("STA connected ");
    Serial.print(usingStoredCreds ? "(saved creds)" : "(compiled creds)");
    Serial.print(" SSID=");
    Serial.print(useSsid);
    Serial.print(" IP=");
    Serial.println(ipAddr);
    return;
  }

  wl_status_t endStatus = WiFi.status();
  Serial.print("[WIFI] STA failed status=");
  Serial.print(static_cast<int>(endStatus));
  Serial.print(" ");
  Serial.println(wifiStatusName(endStatus));

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  wifiMode = "AP";
  ipAddr = WiFi.softAPIP();
  bootIpAddr = ipAddr;
  bootIpCaptured = true;
  setFirstLedColor(0, 0, 255);  // Blue: AP mode
  Serial.print("AP mode SSID=");
  Serial.print(AP_SSID);
  Serial.print(" IP=");
  Serial.println(ipAddr);
}

void readButtons() {
  bool b1 = digitalRead(BTN1_PIN);
  bool b2 = digitalRead(BTN2_PIN);
  unsigned long now = millis();

  if (b1 != lastBtn1Read) {
    lastBtn1EdgeMs = now;
    lastBtn1Read = b1;
  }
  if (b2 != lastBtn2Read) {
    lastBtn2EdgeMs = now;
    lastBtn2Read = b2;
  }

  if (now - lastBtn1EdgeMs > DEBOUNCE_MS) {
    bool newState = (b1 == LOW);
    if (newState != button1Pressed) {
      button1Pressed = newState;
      Serial.print("[BTN] B1 ");
      Serial.println(button1Pressed ? "PRESSED" : "RELEASED");
    }
  }
  if (now - lastBtn2EdgeMs > DEBOUNCE_MS) {
    bool newState = (b2 == LOW);
    if (newState != button2Pressed) {
      button2Pressed = newState;
      Serial.print("[BTN] B2 ");
      Serial.println(button2Pressed ? "PRESSED" : "RELEASED");
    }
  }
}

void handleButtonActions() {
  if (!stepperEnabled) {
    return;
  }

  static bool lastActionB1 = false;
  static bool lastActionB2 = false;

  if (button1Pressed && !lastActionB1) {
    if (stepperOutputsReleased) {
      hardEnableStepperPins();
      applyStepperSpeed();
    }
    targetPosition = stepperPosition + ticksPerGear;
    Serial.print("[BTN] B1 action: +1 gear, target=");
    Serial.println(targetPosition);
  }
  if (button2Pressed && !lastActionB2) {
    if (stepperOutputsReleased) {
      hardEnableStepperPins();
      applyStepperSpeed();
    }
    targetPosition = stepperPosition - ticksPerGear;
    Serial.print("[BTN] B2 action: -1 gear, target=");
    Serial.println(targetPosition);
  }

  lastActionB1 = button1Pressed;
  lastActionB2 = button2Pressed;
}

void updateDisplay() {
  unsigned long now = millis();
  if (now - lastDisplayMs < 200) {
    return;
  }
  lastDisplayMs = now;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("IP:");
  display.println(ipAddr);
  display.print("WiFi:");
  display.println(wifiMode);
  display.print("Pos:");
  display.print(stepperPosition);
  display.print(" T:");
  display.println(targetPosition);
  display.print("Deg:");
  display.println(getIndexerDegrees(), 2);
  display.print("Spd:");
  display.println(speedStepsPerSec, 0);
  display.print("B1:");
  display.print(button1Pressed ? "P" : "-");
  display.print(" B2:");
  display.println(button2Pressed ? "P" : "-");
  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("[BOOT] device startup");
  WiFi.onEvent(onWiFiEvent);
  Serial.print("[BOOT] STEPPER_PORT=");
  Serial.println(STEPPER_PORT);
  Serial.print("[BOOT] stepper pins IN1/IN2/IN3/IN4 = ");
  Serial.print(STEPPER_IN1);
  Serial.print("/");
  Serial.print(STEPPER_IN2);
  Serial.print("/");
  Serial.print(STEPPER_IN3);
  Serial.print("/");
  Serial.println(STEPPER_IN4);
  Serial.println("[BOOT] stepper mode=FULL4WIRE");
  Serial.print("[BOOT] default speed=");
  Serial.println(speedStepsPerSec);
  Serial.print("[BOOT] default accel=");
  Serial.println(accelStepsPerSec2);
  Serial.print("[BOOT] single_step pulse_us=");
  Serial.print(SINGLE_STEP_PULSE_US);
  Serial.print(" settle_ms=");
  Serial.println(SINGLE_STEP_SETTLE_MS);

  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);

  i2cBus.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Booting...");
    display.display();
  }

  applyStepperSpeed();
  stepperPosition = 0;
  targetPosition = 0;
  hardEnableStepperPins();
  recalcIndexerTicks();

  rgbLeds.begin();
  rgbLeds.clear();
  rgbLeds.setBrightness(40);
  rgbLeds.show();

  hasStoredNetworkConfig = loadNetworkConfig(savedConfig);
  Serial.print("[BOOT] hasStoredNetworkConfig=");
  Serial.println(hasStoredNetworkConfig ? "true" : "false");
  setupWifi();
  setupWeb();

  Serial.print("HTTP server on ");
  Serial.println(ipAddr);
}

void loop() {
  server.handleClient();
  readButtons();
  handleButtonActions();

  if (stepperEnabled) {
    runStepperToTargetOneStep();
  } else {
    // Ensure released state remains stable without repeatedly toggling pin modes.
    if (!stepperOutputsReleased) {
      hardDisableStepperPins();
    }
  }

  updateDisplay();
}
