#include "app.h"

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* AP_SSID = "MakerESP32-Stepper";
const char* AP_PASS = "stepper123";
constexpr bool ENABLE_UART_DEBUG = false;

WebServer server(80);
Preferences prefs;
TwoWire i2cBus = TwoWire(0);
Adafruit_SSD1306 display(OLED_W, OLED_H, &i2cBus, -1);
Adafruit_NeoPixel rgbLeds(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

volatile bool button1Pressed = false;
volatile bool button2Pressed = false;
volatile bool button3Pressed = false;
volatile bool button4Pressed = false;
volatile bool stepperEnabled = true;
bool stepperOutputsReleased = false;
uint8_t stepperPort = STEPPER_PORT;
uint8_t stepperIn1 = (STEPPER_PORT == 1) ? STEP1_IN1 : STEP2_IN1;
uint8_t stepperIn2 = (STEPPER_PORT == 1) ? STEP1_IN2 : STEP2_IN2;
uint8_t stepperIn3 = (STEPPER_PORT == 1) ? STEP1_IN3 : STEP2_IN3;
uint8_t stepperIn4 = (STEPPER_PORT == 1) ? STEP1_IN4 : STEP2_IN4;
volatile long stepperPosition = 0;
volatile long targetPosition = 0;
float speedStepsPerSec = 4000.0f;
float accelStepsPerSec2 = 3000.0f;
float currentSpeedStepsPerSec = START_SPEED_STEPS_PER_SEC;
unsigned long lastDisplayMs = 0;
int numberOfGears = 10;
long ticksPerGear = 4000;
volatile int phaseIndex = 0;
bool tbInStandby = true;
unsigned long lastRampUs = 0;
uint32_t stepIntervalUs = 10000;
hw_timer_t* stepperTimer = nullptr;
int stepperTimerIndex = -1;
volatile bool timerMotionActive = false;
volatile uint32_t timerStepIntervalUs = 10000;
volatile uint32_t timerStepIntervalRequestUs = 10000;
volatile bool timerStepIntervalDirty = false;
volatile int8_t lastStepDir = 0;
volatile int8_t outputCommand = OUTPUT_CMD_NONE;
volatile bool highTorqueModeActive = false;
volatile bool halfStepInProgress = false;

bool lastBtn1Read = HIGH;
bool lastBtn2Read = HIGH;
bool lastBtn3Read = HIGH;
bool lastBtn4Read = HIGH;
unsigned long lastBtn1EdgeMs = 0;
unsigned long lastBtn2EdgeMs = 0;
unsigned long lastBtn3EdgeMs = 0;
unsigned long lastBtn4EdgeMs = 0;

String wifiMode = "STA";
IPAddress ipAddr;
IPAddress bootIpAddr;
bool bootIpCaptured = false;
bool hasStoredNetworkConfig = false;
bool oledReady = false;
NetworkConfig savedConfig;

MoveUnit uiMoveUnit = MoveUnit::Gears;
float uiMoveAmount = 1.0f;
float degreeStepSetting = 10.0f;
float gearModule = 1.0f;
float gearPressureAngleDeg = 20.0f;
double commandedStepsFromZero = 0.0;
double degreeIdealPosSteps = 0.0;
bool degreeIdealSynced = false;
long backlashSteps = 0;
long slopSteps = 0;
long indexedLogicalPosition = 0;
int lastCommandDir = 0;
int logicalGearIndex = 0;
String lastFault = "NONE";
volatile uint32_t isrTickCounter = 0;
volatile uint32_t isrStepCounter = 0;
volatile uint64_t totalInterruptStepsTaken = 0;
uint32_t diagIsrTicksPerSec = 0;
uint32_t diagStepRatePerSec = 0;
long missedStepEstimate = 0;
bool diagBacklashTestActive = false;
int diagBacklashTestDir = 1;
int diagBacklashTestRemainingSegments = 0;
unsigned long diagBacklashPauseUntilMs = 0;

OledPage oledPage = OledPage::Status;
DiagBridgeMode diagBridgeMode = DiagBridgeMode::Off;
SetupStage setupStage = SetupStage::Zero;
MoveUnit setupMoveUnit = MoveUnit::Gears;
int setupGearsValue = 10;
float setupDegreeValue = 10.0f;
MotionPreset presets[3];

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

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
    Serial.println("[WIFI_EVT] STA_CONNECTED");
  } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    Serial.print("[WIFI_EVT] STA_GOT_IP ");
    Serial.println(IPAddress(info.got_ip.ip_info.ip.addr));
  } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    uint8_t reason = info.wifi_sta_disconnected.reason;
    lastFault = "WIFI_DISCONNECT_" + String(reason);
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

void setupWifi() {
  Serial.println("[WIFI] setupWifi() start");
  String useSsid = WIFI_SSID;
  String usePass = WIFI_PASS;
  bool usingStoredCreds = false;

  IPAddress staticIp;
  IPAddress staticGw;
  IPAddress staticMask;
  bool useStatic = false;

  if (hasStoredNetworkConfig && savedConfig.ssid.length() > 0) {
    useSsid = savedConfig.ssid;
    usePass = savedConfig.password;
    usingStoredCreds = true;
    useStatic = parseIpArg(savedConfig.staticIp, staticIp) &&
                parseIpArg(savedConfig.gateway, staticGw) &&
                parseIpArg(savedConfig.netmask, staticMask);
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
  setFirstLedColor(255, 0, 0);
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
    setFirstLedColor(0, 255, 0);
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
  setFirstLedColor(0, 0, 255);
  Serial.print("AP mode SSID=");
  Serial.print(AP_SSID);
  Serial.print(" IP=");
  Serial.println(ipAddr);
}

void readButtons() {
  bool b1 = digitalRead(BTN1_PIN);
  bool b2 = digitalRead(BTN2_PIN);
  bool b3 = digitalRead(BTN3_PIN);
  bool b4 = digitalRead(BTN4_PIN);
  unsigned long now = millis();

  if (b1 != lastBtn1Read) {
    lastBtn1EdgeMs = now;
    lastBtn1Read = b1;
  }
  if (b2 != lastBtn2Read) {
    lastBtn2EdgeMs = now;
    lastBtn2Read = b2;
  }
  if (b3 != lastBtn3Read) {
    lastBtn3EdgeMs = now;
    lastBtn3Read = b3;
  }
  if (b4 != lastBtn4Read) {
    lastBtn4EdgeMs = now;
    lastBtn4Read = b4;
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
  if (now - lastBtn3EdgeMs > DEBOUNCE_MS) {
    bool newState = (b3 == LOW);
    if (newState != button3Pressed) {
      button3Pressed = newState;
      Serial.print("[BTN] B3 ");
      Serial.println(button3Pressed ? "PRESSED" : "RELEASED");
    }
  }
  if (now - lastBtn4EdgeMs > DEBOUNCE_MS) {
    bool newState = (b4 == LOW);
    if (newState != button4Pressed) {
      button4Pressed = newState;
      Serial.print("[BTN] B4 ");
      Serial.println(button4Pressed ? "PRESSED" : "RELEASED");
    }
  }
}

void handleButtonActions() {
  static bool lastActionB1 = false;
  static bool lastActionB2 = false;
  static bool lastActionB3 = false;
  static bool lastActionB4 = false;
  bool b1Edge = button1Pressed && !lastActionB1;
  bool b2Edge = button2Pressed && !lastActionB2;
  bool b3Edge = button3Pressed && !lastActionB3;
  bool b4Edge = button4Pressed && !lastActionB4;

  if (b3Edge) {
    if (oledPage == OledPage::Status) {
      oledPage = OledPage::Motion;
    } else if (oledPage == OledPage::Motion) {
      oledPage = OledPage::Diag;
    } else if (oledPage == OledPage::Diag) {
      oledPage = OledPage::Setup;
      beginOledSetupWizard();
    } else {
      oledPage = OledPage::Status;
    }
    Serial.print("[BTN] B3 display page=");
    Serial.println(static_cast<int>(oledPage));
    renderOledStatus();
  }

  if (oledPage == OledPage::Setup) {
    handleSetupWizardButtons(b4Edge, b2Edge, b1Edge);
    lastActionB1 = button1Pressed;
    lastActionB2 = button2Pressed;
    lastActionB3 = button3Pressed;
    lastActionB4 = button4Pressed;
    return;
  }

  if (b2Edge) {
    noInterrupts();
    long pos = stepperPosition;
    targetPosition = pos;
    commandedStepsFromZero = static_cast<double>(pos);
    timerMotionActive = false;
    lastCommandDir = 0;
    halfStepInProgress = false;
    interrupts();
    syncIndexedLogicalPosition(pos);
    Serial.println("[BTN] B2 stop");
    renderOledStatus();
  }

  if (!stepperEnabled) {
    lastActionB1 = button1Pressed;
    lastActionB2 = button2Pressed;
    lastActionB3 = button3Pressed;
    lastActionB4 = button4Pressed;
    return;
  }

  if (b4Edge) {
    long currentPos = getStepperPositionAtomic();
    if (stepperOutputsReleased) {
      hardEnableStepperPins();
      applyStepperSpeed();
    }
    long logicalCurrentPos = indexedLogicalPosition;
    long logicalTarget = computeIndexedAbsoluteTargetFromCurrent(logicalCurrentPos, 1, uiMoveUnit, uiMoveAmount);
    long nextTarget = computeIndexedPhysicalTarget(currentPos, logicalCurrentPos, logicalTarget);
    if (uiMoveUnit == MoveUnit::Gears) {
      long gearMoves = lround(uiMoveAmount);
      if (gearMoves < 1) {
        gearMoves = 1;
      }
      logicalGearIndex = normalizeGearIndex(logicalGearIndex + gearMoves);
    }
    nextTarget = applyBacklashCompensation(currentPos, nextTarget);
    if (nextTarget != currentPos) {
      showMovingScreen();
    }
    setTargetAndCommandedAtomic(nextTarget);
    indexedLogicalPosition = logicalTarget;
    commandedStepsFromZero = static_cast<double>(logicalTarget);
    Serial.print("[BTN] B4 action: next ");
    Serial.print(uiMoveAmount, 3);
    Serial.print(uiMoveUnit == MoveUnit::Degrees ? " deg, target=" : " gear, target=");
    Serial.println(modPositive(getTargetPositionAtomic(), STEPS_PER_INDEXER_REV));
  }

  if (b1Edge) {
    long currentPos = getStepperPositionAtomic();
    if (stepperOutputsReleased) {
      hardEnableStepperPins();
      applyStepperSpeed();
    }
    long logicalCurrentPos = indexedLogicalPosition;
    long logicalTarget = computeIndexedAbsoluteTargetFromCurrent(logicalCurrentPos, -1, uiMoveUnit, uiMoveAmount);
    long nextTarget = computeIndexedPhysicalTarget(currentPos, logicalCurrentPos, logicalTarget);
    if (uiMoveUnit == MoveUnit::Gears) {
      long gearMoves = lround(uiMoveAmount);
      if (gearMoves < 1) {
        gearMoves = 1;
      }
      logicalGearIndex = normalizeGearIndex(logicalGearIndex - gearMoves);
    }
    nextTarget = applyBacklashCompensation(currentPos, nextTarget);
    if (nextTarget != currentPos) {
      showMovingScreen();
    }
    setTargetAndCommandedAtomic(nextTarget);
    indexedLogicalPosition = logicalTarget;
    commandedStepsFromZero = static_cast<double>(logicalTarget);
    Serial.print("[BTN] B1 action: previous ");
    Serial.print(uiMoveAmount, 3);
    Serial.print(uiMoveUnit == MoveUnit::Degrees ? " deg, target=" : " gear, target=");
    Serial.println(modPositive(getTargetPositionAtomic(), STEPS_PER_INDEXER_REV));
  }

  lastActionB1 = button1Pressed;
  lastActionB2 = button2Pressed;
  lastActionB3 = button3Pressed;
  lastActionB4 = button4Pressed;
}

void setup() {
  if (ENABLE_UART_DEBUG) {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("[BOOT] device startup");
  }
  WiFi.onEvent(onWiFiEvent);
  Serial.println("[BOOT] stepper mode=TB67H450 phase-drive");
  Serial.print("[BOOT] default speed=");
  Serial.println(speedStepsPerSec);
  Serial.print("[BOOT] default accel=");
  Serial.println(accelStepsPerSec2);
  Serial.print("[BOOT] standby wake us=");
  Serial.println(TB_STANDBY_WAKE_US);
  Serial.print("[BOOT] buttons B1/B2/B3/B4 = ");
  Serial.print(BTN1_PIN);
  Serial.print("/");
  Serial.print(BTN2_PIN);
  Serial.print("/");
  Serial.print(BTN3_PIN);
  Serial.print("/");
  Serial.println(BTN4_PIN);

  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);
  pinMode(BTN4_PIN, INPUT_PULLUP);
  pinMode(STEP1_IN1, OUTPUT);
  pinMode(STEP1_IN2, OUTPUT);
  pinMode(STEP1_IN3, OUTPUT);
  pinMode(STEP1_IN4, OUTPUT);
  pinMode(STEP2_IN1, OUTPUT);
  pinMode(STEP2_IN2, OUTPUT);
  pinMode(STEP2_IN3, OUTPUT);
  pinMode(STEP2_IN4, OUTPUT);
  digitalWrite(STEP1_IN1, LOW);
  digitalWrite(STEP1_IN2, LOW);
  digitalWrite(STEP1_IN3, LOW);
  digitalWrite(STEP1_IN4, LOW);
  digitalWrite(STEP2_IN1, LOW);
  digitalWrite(STEP2_IN2, LOW);
  digitalWrite(STEP2_IN3, LOW);
  digitalWrite(STEP2_IN4, LOW);

  bool oledOk = initDisplayWithI2cPins(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!oledOk) {
    Serial.println("[OLED] primary I2C bus init failed");
    Serial.println("[OLED] init failed on configured I2C pins");
    oledReady = false;
  }

  loadControlSettings();
  Serial.print("[BOOT] stepper_port=");
  Serial.println(stepperPort);
  Serial.print("[BOOT] stepper pins IN1/IN2/IN3/IN4 = ");
  Serial.print(stepperIn1);
  Serial.print("/");
  Serial.print(stepperIn2);
  Serial.print("/");
  Serial.print(stepperIn3);
  Serial.print("/");
  Serial.println(stepperIn4);
  loadPresets();
  if (uiMoveUnit == MoveUnit::Degrees) {
    uiMoveAmount = degreeStepSetting;
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
  recalcIndexerTicks();

  if (HBRIDGE_DC_TEST_MODE) {
    noInterrupts();
    timerMotionActive = false;
    outputCommand = OUTPUT_CMD_NONE;
    interrupts();
    forceBothHBridgesOn();
    Serial.println("[HBRIDGE_TEST] ACTIVE: both bridges forced ON (A+, B+)");
  } else {
    hardDisableStepperPins();
    stepperTimer = nullptr;
    stepperTimerIndex = -1;
    for (int t = 0; t < 4 && stepperTimer == nullptr; t++) {
      stepperTimer = timerBegin(t, STEPPER_TIMER_DIVIDER, true);
      if (stepperTimer != nullptr) {
        stepperTimerIndex = t;
        break;
      }
    }
    if (stepperTimer != nullptr) {
      timerAttachInterrupt(stepperTimer, &onStepperTimerISR, false);
      timerAlarmWrite(stepperTimer, timerTicksFromUs(STEPPER_TIMER_IDLE_US), true);
      timerAlarmEnable(stepperTimer);
      Serial.print("[STEP] hardware timer ISR started on timer ");
      Serial.println(stepperTimerIndex);
    } else {
      lastFault = "TIMER_INIT_FAIL";
      Serial.println("[STEP] failed to create any hardware timer (0..3)");
    }
  }

  rgbLeds.begin();
  rgbLeds.clear();
  rgbLeds.setBrightness(USE_STATUS_LED ? 40 : 0);
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
  static uint32_t prevIsrTicks = 0;
  static uint32_t prevIsrSteps = 0;
  static unsigned long lastDiagMs = 0;
  static bool wasMoving = false;

  if (HBRIDGE_DC_TEST_MODE || diagBridgeMode != DiagBridgeMode::Off) {
    noInterrupts();
    timerMotionActive = false;
    outputCommand = OUTPUT_CMD_NONE;
    interrupts();
    if (diagBridgeMode != DiagBridgeMode::Off) {
      applyDiagBridgeModeOutput();
    }
    server.handleClient();
    readButtons();
    updateDisplay();
    return;
  }

  bool moving = stepperEnabled && (stepperPosition != targetPosition);

  if (millis() - lastDiagMs >= 1000) {
    uint32_t ticksNow = 0;
    uint32_t stepsNow = 0;
    noInterrupts();
    ticksNow = isrTickCounter;
    stepsNow = isrStepCounter;
    interrupts();
    diagIsrTicksPerSec = ticksNow - prevIsrTicks;
    diagStepRatePerSec = stepsNow - prevIsrSteps;
    prevIsrTicks = ticksNow;
    prevIsrSteps = stepsNow;
    if (diagStepRatePerSec == 0 && moving) {
      missedStepEstimate++;
      lastFault = "STEP_RATE_ZERO";
    }
    lastDiagMs = millis();
  }

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
    Serial.print(activeRpm, 1);
    Serial.print(" torqueMode=");
    Serial.println(highTorqueModeActive ? "HIGH" : "NORMAL");
    lastMotionLogMs = millis();
  }

  if (stepperEnabled) {
    runStepperToTargetOneStep();
  } else if (!stepperOutputsReleased) {
    hardDisableStepperPins();
  }

  processDiagBacklashTest();

  bool movingNow = stepperEnabled && (stepperPosition != targetPosition);
  if (wasMoving && !movingNow) {
    Serial.print("[STEP] stop total_isr_steps=");
    Serial.println(formatUint64(getTotalInterruptStepsAtomic()));
  }
  wasMoving = movingNow;

  if (movingNow) {
    unsigned long nowUs = micros();
    if (nowUs - lastBackgroundUs >= 20000) {
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
