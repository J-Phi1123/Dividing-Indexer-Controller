#include "app.h"

void beginOledSetupWizard() {
  setupStage = SetupStage::Zero;
  setupMoveUnit = uiMoveUnit;
  setupGearsValue = numberOfGears;
  if (setupGearsValue < 1) {
    setupGearsValue = 1;
  }
  setupDegreeValue = degreeStepSetting;
  if (setupDegreeValue <= 0.0f) {
    setupDegreeValue = 10.0f;
  }
  renderOledStatus();
}

void handleSetupWizardButtons(bool b1Edge, bool b3Edge, bool b4Edge) {
  if (setupStage == SetupStage::Zero) {
    if (b1Edge) {
      handleIndexerZero();
    }
    if (b4Edge) {
      oledPage = OledPage::Status;
      renderOledStatus();
      return;
    }
    if (b3Edge) {
      setupStage = SetupStage::Mode;
      renderOledStatus();
    }
    return;
  }

  if (setupStage == SetupStage::Mode) {
    if (b1Edge) {
      setupMoveUnit = (setupMoveUnit == MoveUnit::Gears) ? MoveUnit::Degrees : MoveUnit::Gears;
      renderOledStatus();
    }
    if (b4Edge) {
      setupStage = SetupStage::Zero;
      renderOledStatus();
    }
    if (b3Edge) {
      setupStage = SetupStage::Value;
      renderOledStatus();
    }
    return;
  }

  if (setupMoveUnit == MoveUnit::Gears) {
    if (b1Edge) {
      setupGearsValue--;
      if (setupGearsValue < 1) {
        setupGearsValue = 1;
      }
      renderOledStatus();
    }
    if (b4Edge) {
      setupGearsValue++;
      renderOledStatus();
    }
    if (b3Edge) {
      numberOfGears = setupGearsValue;
      recalcIndexerTicks();
      uiMoveUnit = MoveUnit::Gears;
      uiMoveAmount = 1.0f;
      degreeIdealSynced = false;
      saveControlSettings();
      oledPage = OledPage::Status;
      renderOledStatus();
    }
  } else {
    if (b1Edge) {
      setupDegreeValue -= 1.0f;
      if (setupDegreeValue < 0.001f) {
        setupDegreeValue = 0.001f;
      }
      renderOledStatus();
    }
    if (b4Edge) {
      setupDegreeValue += 1.0f;
      renderOledStatus();
    }
    if (b3Edge) {
      degreeStepSetting = setupDegreeValue;
      uiMoveUnit = MoveUnit::Degrees;
      uiMoveAmount = degreeStepSetting;
      syncDegreeIdealToPosition(getStepperPositionAtomic());
      saveControlSettings();
      oledPage = OledPage::Status;
      renderOledStatus();
    }
  }
}

void renderOledStatus() {
  if (!oledReady) {
    return;
  }

  auto drawCenteredLarge = [](int baselineY, const String& text) {
    int16_t x1 = 0;
    int16_t y1 = 0;
    uint16_t w = 0;
    uint16_t h = 0;
    display.setFont(&FreeSans9pt7b);
    display.getTextBounds(text, 0, baselineY, &x1, &y1, &w, &h);
    int16_t x = (OLED_W - static_cast<int16_t>(w)) / 2;
    if (x < 0) {
      x = 0;
    }
    display.setCursor(x, baselineY);
    display.print(text);
  };

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setFont(nullptr);
  display.setCursor(0, 0);

  if (oledPage == OledPage::Status) {
    String line1 = ipAddr.toString();
    String line2;
    if (uiMoveUnit == MoveUnit::Degrees) {
      line2 = String(getIndexerDegrees(), 1) + " deg";
    } else {
      line2 = "G" + String(getCurrentGearFromPosition(indexedLogicalPosition)) + "/" + String(numberOfGears);
    }

    drawCenteredLarge(14, line1);
    drawCenteredLarge(38, line2);

    display.setFont(nullptr);
    display.setCursor(0, 56);
    display.print("Mode: ");
    display.println(wifiMode);
  } else if (oledPage == OledPage::Motion) {
    display.print("Speed ");
    display.println(speedStepsPerSec, 0);
    display.print("Accel ");
    display.println(accelStepsPerSec2, 0);
    display.print("Backlash ");
    display.println(backlashSteps);
    display.print("Slop ");
    display.println(slopSteps);
  } else if (oledPage == OledPage::Diag) {
    display.print("ISR ");
    display.println(diagIsrTicksPerSec);
    display.print("StepHz ");
    display.println(diagStepRatePerSec);
    display.print("Missed ");
    display.println(missedStepEstimate);
    display.print("Total ");
    display.println(formatUint64(getTotalInterruptStepsAtomic()));
  } else {
    display.println("Setup Wizard");
    if (setupStage == SetupStage::Zero) {
      display.println("B4: Exit");
      display.println("B1: Zero Position");
      display.println("B2: Next");
    } else if (setupStage == SetupStage::Mode) {
      display.print("Mode: ");
      display.println(setupMoveUnit == MoveUnit::Degrees ? "Degrees" : "Gears");
      display.println("B4: Back");
      display.println("B1: Toggle");
      display.println("B2: Next");
    } else if (setupMoveUnit == MoveUnit::Gears) {
      display.print("Gears: ");
      display.println(setupGearsValue);
      display.println("B4:+  B1:-");
      display.println("B2: Apply");
    } else {
      display.print("Deg Step: ");
      display.println(setupDegreeValue, 1);
      display.println("B4:+  B1:-");
      display.println("B2: Apply");
    }
  }
  display.display();
}

void showMovingScreen() {
  renderOledStatus();
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
  if (now - lastDisplayMs < 200) {
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
