#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Adafruit_NeoPixel.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp32-hal-timer.h>
#include <rom/ets_sys.h>
#include <soc/gpio_struct.h>
#include <mbedtls/aes.h>
#include <mbedtls/md.h>
#include <cmath>

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
constexpr uint8_t HMAC_KEY[32] = {
  0x9E, 0x12, 0x4D, 0xB6, 0x7F, 0x21, 0x8A, 0x55,
  0xC3, 0x0B, 0x6E, 0x94, 0x2D, 0xF8, 0x31, 0xA7,
  0x45, 0xD2, 0x19, 0xEE, 0x63, 0x87, 0x3A, 0xBC,
  0x08, 0x71, 0x5F, 0xC9, 0x26, 0xD4, 0x90, 0x1B
};

// ===== Board pin mapping =====
// Set to 1 for Stepper1 terminal block (M1+M2), or 2 for Stepper2 terminal block (M3+M4).
// If using Stepper2, keep board switch for M3/M4 in "Motor" mode.
#ifndef STEPPER_PORT
#define STEPPER_PORT 2
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

// OLED I2C pins
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
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

// Runtime state
volatile bool button1Pressed = false;
volatile bool button2Pressed = false;
volatile bool stepperEnabled = true;
bool stepperOutputsReleased = false;
volatile long stepperPosition = 0;
volatile long targetPosition = 0;
float speedStepsPerSec = 5500.0f;
float accelStepsPerSec2 = 7000.0f;
float currentSpeedStepsPerSec = 5.0f;
unsigned long lastDisplayMs = 0;
constexpr long MOTOR_FULL_STEPS_PER_REV = 200L;
constexpr long COMMUTATION_STATES_PER_FULL_STEP = 2L;  // half-step mode
constexpr long EFFECTIVE_STEPS_PER_REV = MOTOR_FULL_STEPS_PER_REV * COMMUTATION_STATES_PER_FULL_STEP;
constexpr long STEPS_PER_INDEXER_REV = EFFECTIVE_STEPS_PER_REV * 20L * 40L;
int numberOfGears = 10;
long ticksPerGear = 4000;
volatile int phaseIndex = 0;
bool tbInStandby = true;
unsigned long lastRampUs = 0;
uint32_t stepIntervalUs = 10000;
constexpr uint16_t TB_STANDBY_WAKE_US = 30;
constexpr uint16_t REVERSAL_DWELL_US = 1;
constexpr float START_SPEED_STEPS_PER_SEC = 5.0f;
constexpr float MAX_RAMP_DT_SEC = 0.02f;
constexpr uint32_t MIN_STEP_INTERVAL_US = 50;  // 20,000 steps/s theoretical limit
constexpr uint32_t STEPPER_TIMER_IDLE_US = 1000;
constexpr uint16_t STEPPER_TIMER_DIVIDER = 40;  // 80MHz/40 = 2MHz timer clock
constexpr uint32_t STEPPER_TIMER_TICKS_PER_US = 2;
hw_timer_t* stepperTimer = nullptr;
volatile bool timerMotionActive = false;
volatile uint32_t timerStepIntervalUs = 10000;
volatile uint32_t timerStepIntervalRequestUs = 10000;
volatile bool timerStepIntervalDirty = false;
volatile int8_t lastStepDir = 0;
constexpr int8_t OUTPUT_CMD_NONE = 0;
constexpr int8_t OUTPUT_CMD_STOP = 1;
constexpr int8_t OUTPUT_CMD_HOLD_PHASE = 2;
volatile int8_t outputCommand = OUTPUT_CMD_NONE;

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
bool oledReady = false;

struct NetworkConfig {
  String ssid;
  String password;
  String staticIp;
  String gateway;
  String netmask;
};

NetworkConfig savedConfig;

enum class MoveUnit : uint8_t { Gears = 0, Degrees = 1 };
MoveUnit uiMoveUnit = MoveUnit::Gears;
float uiMoveAmount = 1.0f;
double commandedStepsFromZero = 0.0;

// Forward declarations for helper functions used before their definitions.
void hardDisableStepperPins();
void hardEnableStepperPins();
void applyStepperSpeed();
void setStepperPhase(int phase);
void showMovingScreen();
void IRAM_ATTR onStepperTimerISR();
void IRAM_ATTR writeStepperOutputs(bool in1, bool in2, bool in3, bool in4);
bool initDisplayWithI2cPins(uint8_t sdaPin, uint8_t sclPin);
long getStepperPositionAtomic();
long getTargetPositionAtomic();
void setTargetAndCommandedAtomic(long value);

long computeIndexedAbsoluteTargetFromCurrent(long currentPos, int dir, MoveUnit unit, float amount) {
  if (dir == 0) {
    return currentPos;
  }
  if (amount < 0.0f) {
    amount = -amount;
  }
  if (amount < 0.000001f) {
    return currentPos;
  }

  double stepSpan = 0.0;
  if (unit == MoveUnit::Degrees) {
    stepSpan = (static_cast<double>(STEPS_PER_INDEXER_REV) * static_cast<double>(amount)) / 360.0;
    if (stepSpan < 0.000001) {
      return currentPos;
    }
    long delta = lround(stepSpan);
    if (delta < 1) {
      delta = 1;
    }
    return currentPos + ((dir > 0) ? delta : -delta);
  } else {
    if (numberOfGears < 1) {
      return currentPos;
    }
    // Gear mode is always one gear per click (+1 Gear / -1 Gear).
    stepSpan = static_cast<double>(STEPS_PER_INDEXER_REV) / static_cast<double>(numberOfGears);
  }
  if (stepSpan < 0.000001) {
    return currentPos;
  }

  const double pos = static_cast<double>(currentPos);
  const double q = pos / stepSpan;
  const double eps = 1e-9;
  double index = 0.0;
  if (dir > 0) {
    index = std::floor(q + eps) + 1.0;
  } else {
    index = std::ceil(q - eps) - 1.0;
  }
  long absTarget = lround(index * stepSpan);
  if (absTarget == currentPos) {
    long minDelta = lround(stepSpan);
    if (minDelta < 1) {
      minDelta = 1;
    }
    absTarget = currentPos + ((dir > 0) ? minDelta : -minDelta);
  }
  return absTarget;
}

inline uint64_t timerTicksFromUs(uint32_t us) {
  return static_cast<uint64_t>(us) * STEPPER_TIMER_TICKS_PER_US;
}

void recalcIndexerTicks() {
  if (numberOfGears < 1) {
    numberOfGears = 1;
  }
  ticksPerGear = lround((static_cast<float>(EFFECTIVE_STEPS_PER_REV) * 20.0f * 40.0f) /
                        static_cast<float>(numberOfGears));
}

long getStepperPositionAtomic() {
  noInterrupts();
  long pos = stepperPosition;
  interrupts();
  return pos;
}

long getTargetPositionAtomic() {
  noInterrupts();
  long tgt = targetPosition;
  interrupts();
  return tgt;
}

void setTargetAndCommandedAtomic(long value) {
  noInterrupts();
  targetPosition = value;
  commandedStepsFromZero = static_cast<double>(value);
  interrupts();
}

float getIndexerDegrees() {
  long modPos = stepperPosition % STEPS_PER_INDEXER_REV;
  if (modPos < 0) {
    modPos += STEPS_PER_INDEXER_REV;
  }
  return (static_cast<float>(modPos) * 360.0f) / static_cast<float>(STEPS_PER_INDEXER_REV);
}

int getCurrentGearFromPosition(long pos) {
  long modPos = pos % STEPS_PER_INDEXER_REV;
  if (modPos < 0) {
    modPos += STEPS_PER_INDEXER_REV;
  }
  int currentGear = 1;
  if (ticksPerGear > 0 && numberOfGears > 0) {
    currentGear = static_cast<int>(modPos / ticksPerGear) + 1;
    if (currentGear < 1) {
      currentGear = 1;
    } else if (currentGear > numberOfGears) {
      currentGear = numberOfGears;
    }
  }
  return currentGear;
}

void renderOledStatus() {
  if (!oledReady) {
    return;
  }

  const long pos = getStepperPositionAtomic();
  const float curDeg = getIndexerDegrees();
  String lineCur;

  if (uiMoveUnit == MoveUnit::Degrees) {
    lineCur = "Cur " + String(curDeg, 2) + " deg";
  } else {
    const int curGear = getCurrentGearFromPosition(pos);
    lineCur = "Cur G " + String(curGear) + "/" + String(numberOfGears);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  // Use a larger GFX font (~1.5x vs default) for IP + current status.
  display.setFont(&FreeSans9pt7b);
  display.setCursor(0, 14);
  display.print(ipAddr.toString());
  display.setCursor(0, 32);
  display.print(lineCur);
  display.setFont(nullptr);
  display.display();
}

void applyStepperSpeed() {
  if (speedStepsPerSec < START_SPEED_STEPS_PER_SEC) {
    speedStepsPerSec = START_SPEED_STEPS_PER_SEC;
  }
  stepIntervalUs = static_cast<uint32_t>(1000000.0f / speedStepsPerSec);
  if (stepIntervalUs < MIN_STEP_INTERVAL_US) {
    stepIntervalUs = MIN_STEP_INTERVAL_US;
  }
}

void runStepperToTargetOneStep() {
  if (!stepperEnabled) {
    timerMotionActive = false;
    currentSpeedStepsPerSec = START_SPEED_STEPS_PER_SEC;
    if (timerStepIntervalUs != STEPPER_TIMER_IDLE_US) {
      timerStepIntervalRequestUs = STEPPER_TIMER_IDLE_US;
      timerStepIntervalDirty = true;
    }
    return;
  }
  if (stepperPosition == targetPosition) {
    currentSpeedStepsPerSec = START_SPEED_STEPS_PER_SEC;
    lastRampUs = micros();
    timerMotionActive = false;
    if (timerStepIntervalUs != STEPPER_TIMER_IDLE_US) {
      timerStepIntervalRequestUs = STEPPER_TIMER_IDLE_US;
      timerStepIntervalDirty = true;
    }
    if (!tbInStandby) {
      hardDisableStepperPins();
      Serial.println("[THERM] outputs disabled (idle)");
    }
    return;
  }

  if (stepperOutputsReleased || tbInStandby) {
    hardEnableStepperPins();
    Serial.println("[STEP] outputs re-enabled for motion");
  }

  unsigned long nowUs = micros();
  if (lastRampUs == 0) {
    lastRampUs = nowUs;
  }
  float dt = static_cast<float>(nowUs - lastRampUs) / 1000000.0f;
  lastRampUs = nowUs;
  if (dt < 0.0f) {
    dt = 0.0f;
  }
  if (dt > MAX_RAMP_DT_SEC) {
    dt = MAX_RAMP_DT_SEC;
  }

  float desiredSpeed = speedStepsPerSec;
  if (desiredSpeed < START_SPEED_STEPS_PER_SEC) {
    desiredSpeed = START_SPEED_STEPS_PER_SEC;
  }

  float dv = accelStepsPerSec2 * dt;
  if (currentSpeedStepsPerSec < desiredSpeed) {
    currentSpeedStepsPerSec += dv;
    if (currentSpeedStepsPerSec > desiredSpeed) {
      currentSpeedStepsPerSec = desiredSpeed;
    }
  }
  if (currentSpeedStepsPerSec < START_SPEED_STEPS_PER_SEC) {
    currentSpeedStepsPerSec = START_SPEED_STEPS_PER_SEC;
  }

  uint32_t dynIntervalUs = static_cast<uint32_t>(1000000.0f / currentSpeedStepsPerSec);
  if (dynIntervalUs < MIN_STEP_INTERVAL_US) {
    dynIntervalUs = MIN_STEP_INTERVAL_US;
  }
  if (dynIntervalUs != timerStepIntervalUs) {
    timerStepIntervalRequestUs = dynIntervalUs;
    timerStepIntervalDirty = true;
  }

  timerMotionActive = true;
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

void IRAM_ATTR writeStepperOutputs(bool in1, bool in2, bool in3, bool in4) {
  const uint32_t pin1Mask = (1UL << STEPPER_IN1);
  const uint32_t pin2Mask = (1UL << STEPPER_IN2);
  const uint32_t pin3Mask = (1UL << STEPPER_IN3);
  const uint32_t pin4Mask = (1UL << STEPPER_IN4);
  const uint32_t allMask = pin1Mask | pin2Mask | pin3Mask | pin4Mask;
  uint32_t setMask = 0;
  if (in1) setMask |= pin1Mask;
  if (in2) setMask |= pin2Mask;
  if (in3) setMask |= pin3Mask;
  if (in4) setMask |= pin4Mask;
  uint32_t clearMask = allMask & (~setMask);
  GPIO.out_w1tc = clearMask;
  GPIO.out_w1ts = setMask;
}

void hardDisableStepperPins() {
  noInterrupts();
  // Only ISR writes stepper GPIO states.
  outputCommand = OUTPUT_CMD_STOP;
  timerMotionActive = false;
  lastStepDir = 0;
  timerStepIntervalRequestUs = STEPPER_TIMER_IDLE_US;
  timerStepIntervalDirty = true;
  interrupts();
  tbInStandby = true;
  stepperOutputsReleased = true;
}

void hardEnableStepperPins() {
  noInterrupts();
  // Only ISR writes stepper GPIO states.
  outputCommand = OUTPUT_CMD_HOLD_PHASE;
  interrupts();
  delayMicroseconds(TB_STANDBY_WAKE_US);
  tbInStandby = false;
  stepperOutputsReleased = false;
}

void setStepperPhase(int phase) {
  // 8-state half-step sequence for smoother commutation.
  // Note: OFF coil state uses STOP (00) on that H-bridge.
  switch ((phase % 8 + 8) % 8) {
    case 0:
      writeStepperOutputs(true, false, false, false);  // A+, Boff
      break;
    case 1:
      writeStepperOutputs(true, false, true, false);   // A+, B+
      break;
    case 2:
      writeStepperOutputs(false, false, true, false);  // Aoff, B+
      break;
    case 3:
      writeStepperOutputs(false, true, true, false);   // A-, B+
      break;
    case 4:
      writeStepperOutputs(false, true, false, false);  // A-, Boff
      break;
    case 5:
      writeStepperOutputs(false, true, false, true);   // A-, B-
      break;
    case 6:
      writeStepperOutputs(false, false, false, true);  // Aoff, B-
      break;
    default:
      writeStepperOutputs(true, false, false, true);   // A+, B-
      break;
  }
}

void IRAM_ATTR onStepperTimerISR() {
  if (timerStepIntervalDirty && stepperTimer != nullptr) {
    timerStepIntervalUs = timerStepIntervalRequestUs;
    timerAlarmWrite(stepperTimer, timerTicksFromUs(timerStepIntervalUs), true);
    timerStepIntervalDirty = false;
  }

  int8_t cmd = outputCommand;
  if (cmd == OUTPUT_CMD_STOP) {
    writeStepperOutputs(false, false, false, false);
    lastStepDir = 0;
    outputCommand = OUTPUT_CMD_NONE;
  } else if (cmd == OUTPUT_CMD_HOLD_PHASE) {
    setStepperPhase(phaseIndex);
    outputCommand = OUTPUT_CMD_NONE;
  }

  if (!timerMotionActive || !stepperEnabled) {
    return;
  }
  long pos = stepperPosition;
  long tgt = targetPosition;
  if (pos == tgt) {
    timerMotionActive = false;
    return;
  }

  int dir = (tgt > pos) ? 1 : -1;
  if (lastStepDir != 0 && dir != lastStepDir && REVERSAL_DWELL_US > 0) {
    // Apply brief blanking only on direction changes.
    writeStepperOutputs(false, false, false, false);
    ets_delay_us(REVERSAL_DWELL_US);
  }
  phaseIndex = (phaseIndex + dir + 8) % 8;
  setStepperPhase(phaseIndex);
  stepperPosition = pos + dir;
  lastStepDir = dir;
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

bool hmacSha256(const uint8_t* data, size_t len, uint8_t out[32]) {
  const mbedtls_md_info_t* mdInfo = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (mdInfo == nullptr) {
    return false;
  }
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  bool ok = (mbedtls_md_setup(&ctx, mdInfo, 1) == 0) &&
            (mbedtls_md_hmac_starts(&ctx, HMAC_KEY, sizeof(HMAC_KEY)) == 0) &&
            (mbedtls_md_hmac_update(&ctx, data, len) == 0) &&
            (mbedtls_md_hmac_finish(&ctx, out) == 0);
  mbedtls_md_free(&ctx);
  return ok;
}

bool constantTimeEqual(const uint8_t* a, const uint8_t* b, size_t len) {
  uint8_t diff = 0;
  for (size_t i = 0; i < len; i++) {
    diff |= static_cast<uint8_t>(a[i] ^ b[i]);
  }
  return diff == 0;
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

  const size_t cipherPayloadLen = sizeof(iv) + paddedLen;
  uint8_t* payload = static_cast<uint8_t*>(malloc(cipherPayloadLen));
  if (payload == nullptr) {
    free(outBuf);
    return String();
  }
  memcpy(payload, iv, sizeof(iv));
  memcpy(payload + sizeof(iv), outBuf, paddedLen);
  free(outBuf);

  uint8_t tag[32];
  if (!hmacSha256(payload, cipherPayloadLen, tag)) {
    free(payload);
    return String();
  }
  uint8_t* finalPayload = static_cast<uint8_t*>(malloc(cipherPayloadLen + sizeof(tag)));
  if (finalPayload == nullptr) {
    free(payload);
    return String();
  }
  memcpy(finalPayload, payload, cipherPayloadLen);
  memcpy(finalPayload + cipherPayloadLen, tag, sizeof(tag));
  free(payload);

  String hexPayload = toHex(finalPayload, cipherPayloadLen + sizeof(tag));
  free(finalPayload);
  return hexPayload;
}

bool decryptAesCbc(const String& cipherHex, String& plaintextOut) {
  const size_t totalBytes = cipherHex.length() / 2;
  if (cipherHex.length() % 2 != 0) {
    return false;
  }
  const bool looksLikeAuthFormat = (totalBytes >= 64) && (((totalBytes - 16 - 32) % 16) == 0);
  const bool looksLikeLegacyFormat = (totalBytes >= 32) && (((totalBytes - 16) % 16) == 0);
  if (!looksLikeAuthFormat && !looksLikeLegacyFormat) {
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

  size_t cipherPayloadLen = totalBytes;
  if (looksLikeAuthFormat) {
    const size_t authTagLen = 32;
    cipherPayloadLen = totalBytes - authTagLen;
    uint8_t expectedTag[32];
    if (!hmacSha256(raw, cipherPayloadLen, expectedTag)) {
      free(raw);
      return false;
    }
    const uint8_t* providedTag = raw + cipherPayloadLen;
    if (!constantTimeEqual(expectedTag, providedTag, authTagLen)) {
      free(raw);
      return false;
    }
  } else {
    Serial.println("[CFG] decrypt using legacy unauthenticated payload");
  }

  uint8_t iv[16];
  memcpy(iv, raw, sizeof(iv));
  const size_t cipherLen = cipherPayloadLen - sizeof(iv);
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
  h += F(".topbar{display:flex;justify-content:flex-end;align-items:center;margin:0 0 10px}.settings{display:none;margin-bottom:12px}");
  h += F(".grid{display:grid;grid-template-columns:1.1fr 1fr;gap:12px}.card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:12px;box-shadow:0 4px 14px rgba(20,40,60,.08)}");
  h += F(".k{font-size:.82rem;color:var(--muted)}.v{font-size:1.05rem;font-weight:700}.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:8px 0}");
  h += F("button,input{font-size:15px;border-radius:10px;border:1px solid #c8d6e8;padding:10px 12px}button{background:#fff;cursor:pointer}button.primary{background:var(--accent);color:#fff;border-color:var(--accent)}");
  h += F("button.secondary{background:var(--accent2);color:#fff;border-color:var(--accent2)}button.state-on{background:#1f9d55;color:#fff;border-color:#1f9d55}button.state-off{background:#b45309;color:#fff;border-color:#b45309}");
  h += F("input{min-width:120px}.status{white-space:pre-line;font-family:ui-monospace,Consolas,monospace;font-size:.84rem}");
  h += F(".dialWrap{display:flex;flex-direction:column;align-items:center;gap:6px}.dialDeg{font-size:1.3rem;font-weight:700}.tiny{font-size:.8rem;color:var(--muted)}");
  h += F(".net{margin-top:8px;padding:10px;border:1px dashed #9ab2cc;border-radius:12px;background:#f8fbff}");
  h += F("@media (max-width:800px){.grid{grid-template-columns:1fr}.shell{padding:10px}button,input{flex:1}}");
  h += F("</style></head><body><div class='shell'><h1 class='title'>Divider Indexer Controller</h1>");
  h += F("<div class='topbar'><button class='secondary' onclick='toggleSettings()'>Settings</button></div>");
  h += F("<div id='settingsPanel' class='card settings'><div class='row'><input id='speed' type='number' value='5500' min='5' max='10000' oninput='markDirty(\"speed\")'><button class='secondary' onclick='setSpeed()'>Set Speed</button></div>");
  h += F("<div class='row'><input id='accel' type='number' value='7000' min='5' max='10000' oninput='markDirty(\"accel\")'><button class='secondary' onclick='setAccel()'>Set Accel</button></div>");
  h += F("<div class='row'><input id='setPosDeg' type='number' value='0' min='0' max='360' step='0.001'><button class='secondary' onclick='setPositionDeg()'>Set Position (deg)</button></div>");
  h += F("<div class='row'><button class='secondary' onclick='zeroPosition()'>Zero Position</button></div></div>");
  h += F("<div class='grid'>");
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
  h += F("<line id='prevNeedle' x1='125' y1='125' x2='125' y2='56' stroke='#2f9e44' stroke-width='3' stroke-linecap='round'/>");
  h += F("<line id='nextNeedle' x1='125' y1='125' x2='125' y2='56' stroke='#2f9e44' stroke-width='3' stroke-linecap='round'/>");
  h += F("<line id='needle' x1='125' y1='125' x2='125' y2='34' stroke='#c62828' stroke-width='4' stroke-linecap='round'/>");
  h += F("<circle cx='125' cy='125' r='7' fill='#17324f'/></svg>");
  h += F("<div class='dialDeg'><span id='deg'>0.000</span>&deg;</div><div class='tiny'>Indexer Angle</div>");
  h += F("<div class='tiny' id='dialCtx'>Prev: -<br>Cur: -<br>Next: -</div></div>");
  h += F("</div>");
  h += F("<div class='card'><div class='row'><div><div class='k'>Mode</div><div class='v' id='mode'>-</div></div></div>");
  h += F("<div class='status' id='status'>Loading...</div>");
  h += F("<div class='row'><button onclick='cmd(\"/stepper/stop\")'>Stop</button></div>");
  h += F("<div class='row'><select id='moveUnit' onchange='onMoveUnitChanged()'>");
  if (uiMoveUnit == MoveUnit::Degrees) {
    h += F("<option value='gears'>Gears</option><option value='degrees' selected>Degrees</option>");
  } else {
    h += F("<option value='gears' selected>Gears</option><option value='degrees'>Degrees</option>");
  }
  h += F("</select><span id='moveLabel' class='tiny'>");
  if (uiMoveUnit == MoveUnit::Degrees) {
    h += F("Step Degrees");
  } else {
    h += F("Total Gears");
  }
  h += F("</span><input id='moveAmount' type='number' value='");
  if (uiMoveUnit == MoveUnit::Degrees) {
    h += String(uiMoveAmount, 3);
  } else {
    h += String(numberOfGears);
  }
  h += F("' min='0.001' step='0.001' oninput='markDirty(\"moveAmount\")'><button class='secondary' onclick='setMoveConfig()'>Apply Mode/Value</button></div>");
  h += F("<div class='row'><button id='indexMinusBtn' class='primary' onclick='indexStep(-1)'>");
  if (uiMoveUnit == MoveUnit::Degrees) {
    h += F("-Degree");
  } else {
    h += F("-1 Gear");
  }
  h += F("</button><button id='indexPlusBtn' class='primary' onclick='indexStep(1)'>");
  if (uiMoveUnit == MoveUnit::Degrees) {
    h += F("+Degree");
  } else {
    h += F("+1 Gear");
  }
  h += F("</button></div>");
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
  h += F("function toggleSettings(){const p=document.getElementById('settingsPanel');if(!p)return;p.style.display=(p.style.display==='block')?'none':'block';}");
  h += F("const dirtyFields=new Set();");
  h += F("function markDirty(id){dirtyFields.add(id);}function clearDirty(id){dirtyFields.delete(id);}");
  h += F("async function setSpeed(){const s=document.getElementById('speed').value||5500;const r=await fetch('/stepper/speed?value='+s,{method:'POST'});if(r.ok)clearDirty('speed');refresh();}");
  h += F("async function setAccel(){const a=document.getElementById('accel').value||7000;const r=await fetch('/stepper/accel?value='+a,{method:'POST'});if(r.ok)clearDirty('accel');refresh();}");
  h += F("async function setPositionDeg(){const d=document.getElementById('setPosDeg').value||0;const r=await fetch('/indexer/set_position_deg?value='+encodeURIComponent(d),{method:'POST'});if(!r.ok){alert(await r.text());}refresh();}");
  h += F("async function zeroPosition(){const r=await fetch('/indexer/zero',{method:'POST'});if(!r.ok){alert(await r.text());}refresh();}");
  h += F("async function indexStep(dir){await fetch('/indexer/step?dir='+dir,{method:'POST'});refresh();}");
  h += F("function onMoveUnitChanged(){const u=document.getElementById('moveUnit').value;const lbl=document.getElementById('moveLabel');const m=document.getElementById('moveAmount');");
  h += F("if(lbl){lbl.innerText=(u==='degrees')?'Step Degrees':'Total Gears';}if(m){m.min=(u==='degrees')?'0.001':'1';m.step=(u==='degrees')?'0.001':'1';}}");
  h += F("async function setMoveConfig(){const p=new URLSearchParams({unit:document.getElementById('moveUnit').value,amount:document.getElementById('moveAmount').value||'1'});");
  h += F("const r=await fetch('/move/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p});if(!r.ok){alert(await r.text());}else{clearDirty('moveAmount');}refresh();}");
  h += F("async function saveNetwork(){const p=new URLSearchParams({ssid:document.getElementById('ssid').value,password:document.getElementById('password').value,ip:document.getElementById('ip').value,gateway:document.getElementById('gateway').value,netmask:document.getElementById('netmask').value});");
  h += F("const r=await fetch('/config/network',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p});alert(await r.text());}");
  h += F("function renderDial(deg){const needle=document.getElementById('needle');needle.setAttribute('transform',`rotate(${deg} 125 125)`);document.getElementById('deg').innerText=Number(deg).toFixed(3);}");
  h += F("function wrapDeg(v){let d=v%360;if(d<0)d+=360;return d;}");
  h += F("function setDialMarker(id,deg){const el=document.getElementById(id);if(el){el.setAttribute('transform',`rotate(${deg} 125 125)`);}}");
  h += F("function updateDialContext(j){let prev='-',cur='-',next='-';");
  h += F("let prevDeg=0,curDeg=0,nextDeg=0;");
  h += F("if(j.moveUnit==='degrees'){const step=Number(j.moveAmount)||1;curDeg=Number(j.indexerDeg)||0;prevDeg=wrapDeg(curDeg-step);nextDeg=wrapDeg(curDeg+step);");
  h += F("prev=`${prevDeg.toFixed(3)} deg`;cur=`${curDeg.toFixed(3)} deg`;next=`${nextDeg.toFixed(3)} deg`;}");
  h += F("else{const gears=Math.max(1,parseInt(j.gears)||1);const stepDeg=360/gears;const eps=1e-6;curDeg=wrapDeg(Number(j.indexerDeg)||0);");
  h += F("const idx=Math.floor(curDeg/stepDeg);const lineDeg=idx*stepDeg;const onLine=Math.abs(curDeg-lineDeg)<eps||Math.abs(curDeg)<eps;");
  h += F("let prevGear=1,nextGear=1;");
  h += F("if(onLine){const curGear=(idx===0)?gears:idx;prevGear=curGear-1;if(prevGear<1)prevGear=gears;nextGear=curGear+1;if(nextGear>gears)nextGear=1;}");
  h += F("else{prevGear=(idx===0)?gears:idx;nextGear=prevGear+1;if(nextGear>gears)nextGear=1;}");
  h += F("prevDeg=(prevGear===gears)?360:(prevGear*stepDeg);nextDeg=nextGear*stepDeg;");
  h += F("prev=`G${prevGear}/${gears} (~${prevDeg.toFixed(1)} deg)`;cur=`${curDeg.toFixed(3)} deg`;next=`G${nextGear}/${gears} (~${nextDeg.toFixed(1)} deg)`;}");
  h += F("setDialMarker('prevNeedle',prevDeg);setDialMarker('nextNeedle',nextDeg);");
  h += F("const el=document.getElementById('dialCtx');if(el){el.innerHTML=`Prev: ${prev}<br>Cur: ${cur}<br>Next: ${next}`;}}");
  h += F("function setIfIdle(id,val){const el=document.getElementById(id);if(!el)return;if(document.activeElement===el||dirtyFields.has(id))return;el.value=val;}");
  h += F("function updateMoveUi(j){const lbl=document.getElementById('moveLabel');if(lbl){lbl.innerText=(j.moveUnit==='degrees')?'Step Degrees':'Total Gears';}");
  h += F("const m=document.getElementById('moveAmount');if(!m)return;m.min=(j.moveUnit==='degrees')?'0.001':'1';m.step=(j.moveUnit==='degrees')?'0.001':'1';");
  h += F("if(document.activeElement!==m&&!dirtyFields.has('moveAmount')){m.value=(j.moveUnit==='degrees')?j.moveAmount:j.gears;}}");
  h += F("async function refresh(){const r=await fetch('/status');const j=await r.json();");
  h += F("document.getElementById('mode').innerText=j.wifiMode;setIfIdle('speed',j.speed);setIfIdle('accel',j.accel);updateMoveUi(j);");
  h += F("const unitSel=document.getElementById('moveUnit');if(document.activeElement!==unitSel){unitSel.value=j.moveUnit;}");
  h += F("document.getElementById('indexPlusBtn').innerText=(j.moveUnit==='degrees')?'+Degree':'+1 Gear';");
  h += F("document.getElementById('indexMinusBtn').innerText=(j.moveUnit==='degrees')?'-Degree':'-1 Gear';");
  h += F("document.getElementById('status').innerText=`Pos ${j.position} -> ${j.target}\\nAngle ${Number(j.indexerDeg).toFixed(3)} deg\\nMode ${j.moveUnit==='degrees'?(j.moveAmount+' deg'):(''+j.gears+' gears')}`;");
  h += F("renderDial(j.indexerDeg);updateDialContext(j);}");
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
  json += "\"moveUnit\":\"" + String(uiMoveUnit == MoveUnit::Degrees ? "degrees" : "gears") + "\",";
  json += "\"moveAmount\":" + String(uiMoveAmount, 3) + ",";
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

void showMovingScreen() {
  renderOledStatus();
}

void handleStepperStop() {
  noInterrupts();
  long pos = stepperPosition;
  targetPosition = pos;
  commandedStepsFromZero = static_cast<double>(pos);
  interrupts();
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

  long currentPos = getStepperPositionAtomic();
  if (stepperOutputsReleased) {
    hardEnableStepperPins();
    applyStepperSpeed();
  }
  long nextTarget = currentPos + steps;
  if (nextTarget != currentPos) {
    showMovingScreen();
  }
  setTargetAndCommandedAtomic(nextTarget);
  Serial.print("[STEP] move target=");
  Serial.println(nextTarget);
  server.send(200, "text/plain", "OK");
}

void handleStepperSingleStep() {
  Serial.println("[HTTP] /stepper/single");
  if (!stepperEnabled) {
    Serial.println("[STEP] single step rejected: disabled");
    server.send(409, "text/plain", "Stepper is disabled");
    return;
  }
  int dir = server.hasArg("dir") ? server.arg("dir").toInt() : 1;
  long currentPos = getStepperPositionAtomic();
  long nextTarget = currentPos + ((dir < 0) ? -1L : 1L);
  if (nextTarget != currentPos) {
    showMovingScreen();
  }
  setTargetAndCommandedAtomic(nextTarget);
  Serial.print("[STEP] single queued dir=");
  Serial.print(dir);
  Serial.print(" target=");
  Serial.println(nextTarget);
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
  if (v > 10000.0f) {
    v = 10000.0f;
  }
  speedStepsPerSec = v;
  applyStepperSpeed();
  // Retune ISR cadence immediately to the selected fixed speed.
  noInterrupts();
  timerStepIntervalRequestUs = stepIntervalUs;
  timerStepIntervalDirty = true;
  interrupts();
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
  if (a > 10000.0f) {
    a = 10000.0f;
  }
  accelStepsPerSec2 = a;
  Serial.print("[STEP] accel=");
  Serial.println(accelStepsPerSec2);
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
  long currentPos = getStepperPositionAtomic();
  long nextTarget = computeIndexedAbsoluteTargetFromCurrent(currentPos, dir, uiMoveUnit, uiMoveAmount);
  if (stepperOutputsReleased) {
    hardEnableStepperPins();
    applyStepperSpeed();
  }
  if (nextTarget != currentPos) {
    showMovingScreen();
  }
  setTargetAndCommandedAtomic(nextTarget);
  Serial.print("[STEP] index dir=");
  Serial.print(dir);
  Serial.print(" unit=");
  Serial.print(uiMoveUnit == MoveUnit::Degrees ? "degrees" : "gears");
  Serial.print(" amount=");
  Serial.print(uiMoveAmount, 3);
  Serial.print(" target=");
  Serial.println(nextTarget);
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

void handleMoveConfig() {
  if (!server.hasArg("unit") || !server.hasArg("amount")) {
    server.send(400, "text/plain", "Missing unit or amount");
    return;
  }
  String unit = server.arg("unit");
  float amount = server.arg("amount").toFloat();
  if (amount <= 0.0f) {
    server.send(400, "text/plain", "Amount must be > 0");
    return;
  }
  if (unit == "degrees") {
    uiMoveUnit = MoveUnit::Degrees;
    uiMoveAmount = amount;
  } else {
    uiMoveUnit = MoveUnit::Gears;
    int nextGears = static_cast<int>(lround(amount));
    if (nextGears < 1) {
      server.send(400, "text/plain", "Gears must be >= 1");
      return;
    }
    numberOfGears = nextGears;
    recalcIndexerTicks();
    uiMoveAmount = 1.0f;
  }
  Serial.print("[MOVE] unit=");
  Serial.print(uiMoveUnit == MoveUnit::Degrees ? "degrees" : "gears");
  Serial.print(" amount=");
  Serial.println(uiMoveAmount, 3);
  server.send(200, "text/plain", "OK");
}

void handleIndexerZero() {
  noInterrupts();
  stepperPosition = 0;
  targetPosition = 0;
  commandedStepsFromZero = 0.0;
  timerMotionActive = false;
  interrupts();
  Serial.println("[INDEX] zeroed current position reference");
  server.send(200, "text/plain", "OK");
}

void handleIndexerSetPositionDeg() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }
  float deg = server.arg("value").toFloat();
  if (std::isnan(deg) || std::isinf(deg)) {
    server.send(400, "text/plain", "Invalid degree value");
    return;
  }
  while (deg < 0.0f) {
    deg += 360.0f;
  }
  while (deg >= 360.0f) {
    deg -= 360.0f;
  }

  long pos = lround((static_cast<double>(deg) * static_cast<double>(STEPS_PER_INDEXER_REV)) / 360.0);
  noInterrupts();
  stepperPosition = pos;
  targetPosition = pos;
  commandedStepsFromZero = static_cast<double>(pos);
  timerMotionActive = false;
  interrupts();
  Serial.print("[INDEX] set position deg=");
  Serial.print(deg, 3);
  Serial.print(" steps=");
  Serial.println(pos);
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
  server.on("/stepper/stop", HTTP_POST, handleStepperStop);
  server.on("/stepper/move", HTTP_POST, handleStepperMove);
  server.on("/stepper/single", HTTP_POST, handleStepperSingleStep);
  server.on("/stepper/speed", HTTP_POST, handleStepperSpeed);
  server.on("/stepper/accel", HTTP_POST, handleStepperAccel);
  server.on("/indexer/step", HTTP_POST, handleIndexerStep);
  server.on("/indexer/set_gears", HTTP_POST, handleIndexerSetGears);
  server.on("/indexer/zero", HTTP_POST, handleIndexerZero);
  server.on("/indexer/set_position_deg", HTTP_POST, handleIndexerSetPositionDeg);
  server.on("/move/config", HTTP_POST, handleMoveConfig);
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
    long currentPos = getStepperPositionAtomic();
    if (stepperOutputsReleased) {
      hardEnableStepperPins();
      applyStepperSpeed();
    }
    long nextTarget = computeIndexedAbsoluteTargetFromCurrent(currentPos, -1, uiMoveUnit, uiMoveAmount);  // B1 rewinds
    if (nextTarget != currentPos) {
      showMovingScreen();
    }
    setTargetAndCommandedAtomic(nextTarget);
    Serial.print("[BTN] B1 action: rewind ");
    Serial.print(uiMoveAmount, 3);
    Serial.print(uiMoveUnit == MoveUnit::Degrees ? " deg, target=" : " gear, target=");
    Serial.println(targetPosition);
  }
  if (button2Pressed && !lastActionB2) {
    long currentPos = getStepperPositionAtomic();
    if (stepperOutputsReleased) {
      hardEnableStepperPins();
      applyStepperSpeed();
    }
    long nextTarget = computeIndexedAbsoluteTargetFromCurrent(currentPos, 1, uiMoveUnit, uiMoveAmount);  // B2 advances
    if (nextTarget != currentPos) {
      showMovingScreen();
    }
    setTargetAndCommandedAtomic(nextTarget);
    Serial.print("[BTN] B2 action: advance ");
    Serial.print(uiMoveAmount, 3);
    Serial.print(uiMoveUnit == MoveUnit::Degrees ? " deg, target=" : " gear, target=");
    Serial.println(targetPosition);
  }

  lastActionB1 = button1Pressed;
  lastActionB2 = button2Pressed;
}

void updateDisplay() {
  if (!oledReady) {
    return;
  }

  unsigned long now = millis();
  bool moving = (stepperPosition != targetPosition);
  if (moving) {
    return;
  }
  unsigned long refreshMs = 200;
  if (now - lastDisplayMs < refreshMs) {
    return;
  }
  lastDisplayMs = now;

  renderOledStatus();
}

bool initDisplayWithI2cPins(uint8_t sdaPin, uint8_t sclPin) {
  Serial.print("[OLED] trying I2C SDA/SCL=");
  Serial.print(sdaPin);
  Serial.print("/");
  Serial.println(sclPin);

  i2cBus.begin(sdaPin, sclPin, 100000);
  i2cBus.beginTransmission(OLED_ADDR);
  uint8_t err = i2cBus.endTransmission();
  if (err != 0) {
    Serial.print("[OLED] address 0x3C not found, I2C err=");
    Serial.println(err);
    return false;
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] display.begin failed at 0x3C");
    return false;
  }

  Serial.println("[OLED] initialized at 0x3C");
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setFont(&FreeSans9pt7b);
  display.setCursor(0, 28);
  display.println("Booting...");
  display.setFont(nullptr);
  display.display();
  oledReady = true;
  return true;
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
  Serial.println("[BOOT] stepper mode=TB67H450 phase-drive");
  Serial.print("[BOOT] default speed=");
  Serial.println(speedStepsPerSec);
  Serial.print("[BOOT] default accel=");
  Serial.println(accelStepsPerSec2);
  Serial.print("[BOOT] standby wake us=");
  Serial.println(TB_STANDBY_WAKE_US);

  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(STEPPER_IN1, OUTPUT);
  pinMode(STEPPER_IN2, OUTPUT);
  pinMode(STEPPER_IN3, OUTPUT);
  pinMode(STEPPER_IN4, OUTPUT);

  bool oledOk = initDisplayWithI2cPins(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!oledOk) {
    Serial.println("[OLED] primary I2C bus init failed");
  }
  if (!oledOk) {
    Serial.println("[OLED] init failed on configured I2C pins");
    oledReady = false;
  }

  applyStepperSpeed();
  currentSpeedStepsPerSec = START_SPEED_STEPS_PER_SEC;
  lastRampUs = micros();
  stepperPosition = 0;
  targetPosition = 0;
  commandedStepsFromZero = 0.0;
  timerMotionActive = false;
  timerStepIntervalUs = stepIntervalUs;
  timerStepIntervalRequestUs = stepIntervalUs;
  timerStepIntervalDirty = false;
  hardDisableStepperPins();
  recalcIndexerTicks();

  stepperTimer = timerBegin(0, STEPPER_TIMER_DIVIDER, true);
  if (stepperTimer != nullptr) {
    timerAttachInterrupt(stepperTimer, &onStepperTimerISR, false);
    timerAlarmWrite(stepperTimer, timerTicksFromUs(STEPPER_TIMER_IDLE_US), true);
    timerAlarmEnable(stepperTimer);
    Serial.println("[STEP] hardware timer ISR started");
  } else {
    Serial.println("[STEP] failed to create hardware timer");
  }

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
  static unsigned long lastBackgroundUs = 0;
  static unsigned long lastMotionLogMs = 0;
  bool moving = stepperEnabled && (stepperPosition != targetPosition);

  if (moving && millis() - lastMotionLogMs >= 1000) {
    uint32_t activeIntervalUs = 0;
    uint32_t requestedIntervalUs = 0;
    noInterrupts();
    activeIntervalUs = timerStepIntervalUs;
    requestedIntervalUs = timerStepIntervalRequestUs;
    interrupts();
    float activeStepsPerSec = (activeIntervalUs > 0) ? (1000000.0f / static_cast<float>(activeIntervalUs)) : 0.0f;
    float activeRpm = (activeStepsPerSec * 60.0f) / static_cast<float>(EFFECTIVE_STEPS_PER_REV);
    Serial.print("[MOTION] cmdSpeed=");
    Serial.print(speedStepsPerSec, 1);
    Serial.print(" curSpeed=");
    Serial.print(currentSpeedStepsPerSec, 1);
    Serial.print(" reqIntUs=");
    Serial.print(requestedIntervalUs);
    Serial.print(" actIntUs=");
    Serial.print(activeIntervalUs);
    Serial.print(" actHz=");
    Serial.print(activeStepsPerSec, 1);
    Serial.print(" actRPM=");
    Serial.println(activeRpm, 1);
    lastMotionLogMs = millis();
  }

  if (stepperEnabled) {
    runStepperToTargetOneStep();
  } else {
    // Ensure released state remains stable without repeatedly toggling pin modes.
    if (!stepperOutputsReleased) {
      hardDisableStepperPins();
    }
  }

  // While moving, prioritize stepping and run background tasks at a lower rate.
  if (moving) {
    unsigned long nowUs = micros();
    if (nowUs - lastBackgroundUs >= 20000) {  // 20 ms background service budget
      server.handleClient();
      readButtons();
      handleButtonActions();
      lastBackgroundUs = nowUs;
    }
  } else {
    server.handleClient();
    readButtons();
    handleButtonActions();
  }

  updateDisplay();
}
