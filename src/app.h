#pragma once

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

extern const char* WIFI_SSID;
extern const char* WIFI_PASS;
extern const char* AP_SSID;
extern const char* AP_PASS;

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

#ifndef STEPPER_PORT
#define STEPPER_PORT 2
#endif

constexpr uint8_t STEP1_IN1 = 27;
constexpr uint8_t STEP1_IN2 = 13;
constexpr uint8_t STEP1_IN3 = 4;
constexpr uint8_t STEP1_IN4 = 2;
constexpr uint8_t STEP2_IN1 = 17;
constexpr uint8_t STEP2_IN2 = 12;
constexpr uint8_t STEP2_IN3 = 15;
constexpr uint8_t STEP2_IN4 = 14;

constexpr uint8_t BTN1_PIN = 26;
constexpr uint8_t BTN2_PIN = 25;
constexpr uint8_t BTN3_PIN = 33;
constexpr uint8_t BTN4_PIN = 32;

constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr int OLED_W = 128;
constexpr int OLED_H = 64;
constexpr uint8_t RGB_LED_PIN = 16;
constexpr uint8_t RGB_LED_COUNT = 4;
constexpr bool USE_STATUS_LED = false;
constexpr long MOTOR_FULL_STEPS_PER_REV = 200L;
constexpr long COMMUTATION_STATES_PER_FULL_STEP = 1L;
constexpr long EFFECTIVE_STEPS_PER_REV = MOTOR_FULL_STEPS_PER_REV * COMMUTATION_STATES_PER_FULL_STEP;
constexpr long STEPS_PER_INDEXER_REV = EFFECTIVE_STEPS_PER_REV * 20L * 40L;
constexpr uint16_t TB_STANDBY_WAKE_US = 30;
constexpr uint16_t REVERSAL_DWELL_US = 1;
constexpr float START_SPEED_STEPS_PER_SEC = 5.0f;
constexpr float MAX_RAMP_DT_SEC = 0.02f;
constexpr bool HBRIDGE_DC_TEST_MODE = false;
constexpr bool USE_HIGH_TORQUE_MODE = false;
constexpr float HIGH_TORQUE_SPEED_THRESHOLD_STEPS_PER_SEC = 2500.0f;
constexpr uint32_t MIN_STEP_INTERVAL_US = 50;
constexpr uint32_t STEPPER_TIMER_IDLE_US = 1000;
constexpr uint16_t STEPPER_TIMER_DIVIDER = 40;
constexpr uint32_t STEPPER_TIMER_TICKS_PER_US = 2;
constexpr int8_t OUTPUT_CMD_NONE = 0;
constexpr int8_t OUTPUT_CMD_STOP = 1;
constexpr int8_t OUTPUT_CMD_HOLD_PHASE = 2;
constexpr unsigned long DEBOUNCE_MS = 30;

extern WebServer server;
extern Preferences prefs;
extern TwoWire i2cBus;
extern Adafruit_SSD1306 display;
extern Adafruit_NeoPixel rgbLeds;

extern volatile bool button1Pressed;
extern volatile bool button2Pressed;
extern volatile bool button3Pressed;
extern volatile bool button4Pressed;
extern volatile bool stepperEnabled;
extern bool stepperOutputsReleased;
extern uint8_t stepperPort;
extern uint8_t stepperIn1;
extern uint8_t stepperIn2;
extern uint8_t stepperIn3;
extern uint8_t stepperIn4;
extern volatile long stepperPosition;
extern volatile long targetPosition;
extern float speedStepsPerSec;
extern float accelStepsPerSec2;
extern float currentSpeedStepsPerSec;
extern unsigned long lastDisplayMs;
extern int numberOfGears;
extern long ticksPerGear;
extern volatile int phaseIndex;
extern bool tbInStandby;
extern unsigned long lastRampUs;
extern uint32_t stepIntervalUs;
extern hw_timer_t* stepperTimer;
extern int stepperTimerIndex;
extern volatile bool timerMotionActive;
extern volatile uint32_t timerStepIntervalUs;
extern volatile uint32_t timerStepIntervalRequestUs;
extern volatile bool timerStepIntervalDirty;
extern volatile int8_t lastStepDir;
extern volatile int8_t outputCommand;
extern volatile bool highTorqueModeActive;
extern volatile bool halfStepInProgress;

extern bool lastBtn1Read;
extern bool lastBtn2Read;
extern bool lastBtn3Read;
extern bool lastBtn4Read;
extern unsigned long lastBtn1EdgeMs;
extern unsigned long lastBtn2EdgeMs;
extern unsigned long lastBtn3EdgeMs;
extern unsigned long lastBtn4EdgeMs;

extern String wifiMode;
extern IPAddress ipAddr;
extern IPAddress bootIpAddr;
extern bool bootIpCaptured;
extern bool hasStoredNetworkConfig;
extern bool oledReady;

struct NetworkConfig {
  String ssid;
  String password;
  String staticIp;
  String gateway;
  String netmask;
};

extern NetworkConfig savedConfig;

enum class MoveUnit : uint8_t { Gears = 0, Degrees = 1 };
extern MoveUnit uiMoveUnit;
extern float uiMoveAmount;
extern float degreeStepSetting;
extern float gearModule;
extern float gearPressureAngleDeg;
extern double commandedStepsFromZero;
extern double degreeIdealPosSteps;
extern bool degreeIdealSynced;
extern long backlashSteps;
extern long slopSteps;
extern long indexedLogicalPosition;
extern int lastCommandDir;
extern int logicalGearIndex;
extern String lastFault;
extern volatile uint32_t isrTickCounter;
extern volatile uint32_t isrStepCounter;
extern volatile uint64_t totalInterruptStepsTaken;
extern uint32_t diagIsrTicksPerSec;
extern uint32_t diagStepRatePerSec;
extern long missedStepEstimate;
extern bool diagBacklashTestActive;
extern int diagBacklashTestDir;
extern int diagBacklashTestRemainingSegments;
extern unsigned long diagBacklashPauseUntilMs;

enum class OledPage : uint8_t { Status = 0, Motion = 1, Diag = 2, Setup = 3 };
extern OledPage oledPage;

enum class DiagBridgeMode : uint8_t { Off = 0, M1On = 1, M2On = 2, M3On = 3, M4On = 4 };
extern DiagBridgeMode diagBridgeMode;

enum class SetupStage : uint8_t { Zero = 0, Mode = 1, Value = 2 };
extern SetupStage setupStage;
extern MoveUnit setupMoveUnit;
extern int setupGearsValue;
extern float setupDegreeValue;

struct MotionPreset {
  String name;
  int gears;
  float degreeStep;
  float speed;
  float accel;
  float gearModule;
  float gearPressureAngleDeg;
};

extern MotionPreset presets[3];

long modPositive(long value, long mod);
double wrapStepsToRevolution(double steps);
void syncDegreeIdealToPosition(long pos);
long computeDegreeModeTarget(long currentPos, int dir, float amount);
long computeIndexedAbsoluteTargetFromCurrent(long currentPos, int dir, MoveUnit unit, float amount);
int normalizeGearIndex(int index);
void syncLogicalGearIndexToPosition(long pos);
void syncIndexedLogicalPosition(long pos);
inline uint64_t timerTicksFromUs(uint32_t us) {
  return static_cast<uint64_t>(us) * STEPPER_TIMER_TICKS_PER_US;
}
void recalcIndexerTicks();
long getStepperPositionAtomic();
long getTargetPositionAtomic();
uint64_t getTotalInterruptStepsAtomic();
String formatUint64(uint64_t value);
void setTargetAndCommandedAtomic(long value);
void applyStepperPortSelection(uint8_t port);
long applyBacklashCompensation(long currentPos, long nextTarget);
long computeIndexedPhysicalTarget(long actualCurrentPos, long logicalCurrentPos, long logicalTargetPos);
float getIndexerDegrees();
int getCurrentGearFromPosition(long pos);
void beginOledSetupWizard();
void handleSetupWizardButtons(bool b1Edge, bool b3Edge, bool b4Edge);
void renderOledStatus();
void showMovingScreen();
void updateDisplay();
bool initDisplayWithI2cPins(uint8_t sdaPin, uint8_t sclPin);
void applyStepperSpeed();
uint32_t computeTimerIntervalUsForSpeed(float speed, bool highTorqueMode);
void runStepperToTargetOneStep();
void setFirstLedColor(uint8_t r, uint8_t g, uint8_t b);
void IRAM_ATTR writeStepperOutputs(bool in1, bool in2, bool in3, bool in4);
void hardDisableStepperPins();
void hardEnableStepperPins();
void forceBothHBridgesOn();
void applyDiagBridgeModeOutput();
void setStepperPhase(int phase);
void IRAM_ATTR onStepperTimerISR();

const char* wifiDisconnectReasonName(uint8_t reason);
const char* authModeName(wifi_auth_mode_t mode);
const char* wifiStatusName(wl_status_t s);
void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
void debugScanForTargetSsid(const String& targetSsid);

String toHex(const uint8_t* data, size_t len);
bool hmacSha256(const uint8_t* data, size_t len, uint8_t out[32]);
bool constantTimeEqual(const uint8_t* a, const uint8_t* b, size_t len);
bool fromHexNibble(char c, uint8_t& out);
bool fromHex(const String& hex, uint8_t* out, size_t outLen);
String encryptAesCbc(const String& plaintext);
bool decryptAesCbc(const String& cipherHex, String& plaintextOut);
String serializeNetworkConfig(const NetworkConfig& cfg);
bool parseNetworkConfig(const String& plain, NetworkConfig& cfg);
bool saveNetworkConfig(const NetworkConfig& cfg);
bool loadNetworkConfig(NetworkConfig& cfg);
void loadControlSettings();
void saveControlSettings();
void loadPresets();
void savePresetSlot(int slot);
bool parseIpArg(const String& s, IPAddress& out);

String htmlEscape(const String& in);
String jsonEscape(const String& in);
String htmlPage();
void sendJsonStatus();
void handleRoot();
void handleStatus();
void handleStepperStop();
void handleStepperMove();
void handleStepperSingleStep();
void handleStepperSpeed();
void handleStepperAccel();
void handleIndexerStep();
void handleIndexerSetGears();
void handleMoveConfig();
void handleIndexerZero();
void handleIndexerSetPositionDeg();
void handleIndexerSetPositionGear();
void handleSetBacklash();
void handleSetSlop();
void handleSetGearGeometry();
void handleSetStepperPort();
void handlePresetSave();
void handlePresetLoad();
void handleSaveNetworkConfig();
void handleDiagResetIsd();
void handleDiagResetIsdPort();
void handleDiagBridgeMode();
void handleDiagTestBacklash();
void handleDiagSingleStep();
void handleDiagResetStepTotal();
void processDiagBacklashTest();
void setupWeb();

void setupWifi();
void readButtons();
void handleButtonActions();
