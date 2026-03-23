#include "app.h"

namespace {
long alignToFullStepBoundary(long value) {
  return value - modPositive(value, COMMUTATION_STATES_PER_FULL_STEP);
}
}

long modPositive(long value, long mod) {
  long out = value % mod;
  if (out < 0) {
    out += mod;
  }
  return out;
}

double wrapStepsToRevolution(double steps) {
  const double rev = static_cast<double>(STEPS_PER_INDEXER_REV);
  double wrapped = std::fmod(steps, rev);
  if (wrapped < 0.0) {
    wrapped += rev;
  }
  return wrapped;
}

void syncDegreeIdealToPosition(long pos) {
  degreeIdealPosSteps = static_cast<double>(modPositive(pos, STEPS_PER_INDEXER_REV));
  degreeIdealSynced = true;
}

long computeDegreeModeTarget(long currentPos, int dir, float amount) {
  if (!degreeIdealSynced) {
    syncDegreeIdealToPosition(currentPos);
  }

  const double stepSpan = (static_cast<double>(STEPS_PER_INDEXER_REV) * static_cast<double>(amount)) / 360.0;
  if (stepSpan < 0.000001) {
    return currentPos;
  }

  degreeIdealPosSteps += (dir > 0) ? stepSpan : -stepSpan;
  degreeIdealPosSteps = wrapStepsToRevolution(degreeIdealPosSteps);

  long idealStep = lround(degreeIdealPosSteps);
  if (idealStep >= STEPS_PER_INDEXER_REV) {
    idealStep = 0;
  } else if (idealStep < 0) {
    idealStep = 0;
  }

  long currentMod = modPositive(currentPos, STEPS_PER_INDEXER_REV);
  long delta = idealStep - currentMod;
  if (dir > 0 && delta <= 0) {
    delta += STEPS_PER_INDEXER_REV;
  } else if (dir < 0 && delta >= 0) {
    delta -= STEPS_PER_INDEXER_REV;
  }
  return alignToFullStepBoundary(currentPos + delta);
}

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

  if (unit == MoveUnit::Degrees) {
    return computeDegreeModeTarget(currentPos, dir, amount);
  }

  degreeIdealSynced = false;
  if (numberOfGears < 1) {
    return currentPos;
  }

  long gearMoves = lround(amount);
  if (gearMoves < 1) {
    gearMoves = 1;
  }

  long targetGearIndex = normalizeGearIndex(logicalGearIndex + ((dir > 0) ? gearMoves : -gearMoves));
  long targetMod = lround((static_cast<double>(targetGearIndex) * static_cast<double>(STEPS_PER_INDEXER_REV)) /
                          static_cast<double>(numberOfGears));
  const long currentMod = modPositive(currentPos, STEPS_PER_INDEXER_REV);
  targetMod = modPositive(targetMod, STEPS_PER_INDEXER_REV);
  long delta = targetMod - currentMod;
  if (dir > 0 && delta <= 0) {
    delta += STEPS_PER_INDEXER_REV;
  } else if (dir < 0 && delta >= 0) {
    delta -= STEPS_PER_INDEXER_REV;
  }
  return alignToFullStepBoundary(currentPos + delta);
}

int normalizeGearIndex(int index) {
  if (numberOfGears < 1) {
    return 0;
  }
  index %= numberOfGears;
  if (index < 0) {
    index += numberOfGears;
  }
  return index;
}

void syncLogicalGearIndexToPosition(long pos) {
  if (numberOfGears < 1) {
    logicalGearIndex = 0;
    return;
  }
  long modPos = modPositive(pos, STEPS_PER_INDEXER_REV);
  long nearestGearIndex = lround((static_cast<double>(modPos) * static_cast<double>(numberOfGears)) /
                                 static_cast<double>(STEPS_PER_INDEXER_REV));
  logicalGearIndex = normalizeGearIndex(static_cast<int>(nearestGearIndex));
}

void syncIndexedLogicalPosition(long pos) {
  indexedLogicalPosition = alignToFullStepBoundary(pos);
  syncLogicalGearIndexToPosition(indexedLogicalPosition);
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

uint64_t getTotalInterruptStepsAtomic() {
  noInterrupts();
  uint64_t total = totalInterruptStepsTaken;
  interrupts();
  return total;
}

String formatUint64(uint64_t value) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(value));
  return String(buf);
}

void setTargetAndCommandedAtomic(long value) {
  value = alignToFullStepBoundary(value);
  noInterrupts();
  targetPosition = value;
  commandedStepsFromZero = static_cast<double>(value);
  interrupts();
}

void applyStepperPortSelection(uint8_t port) {
  if (port != 1 && port != 2) {
    port = 2;
  }
  stepperPort = port;
  if (stepperPort == 1) {
    stepperIn1 = STEP1_IN1;
    stepperIn2 = STEP1_IN2;
    stepperIn3 = STEP1_IN3;
    stepperIn4 = STEP1_IN4;
  } else {
    stepperIn1 = STEP2_IN1;
    stepperIn2 = STEP2_IN2;
    stepperIn3 = STEP2_IN3;
    stepperIn4 = STEP2_IN4;
  }
}

long applyBacklashCompensation(long currentPos, long nextTarget) {
  int dir = 0;
  if (nextTarget > currentPos) dir = 1;
  if (nextTarget < currentPos) dir = -1;
  if (dir != 0 && lastCommandDir != 0 && dir != lastCommandDir && backlashSteps > 0) {
    nextTarget += static_cast<long>(dir) * backlashSteps;
  }
  if (dir != 0) {
    lastCommandDir = dir;
  }
  return nextTarget;
}

long computeIndexedPhysicalTarget(long actualCurrentPos, long logicalCurrentPos, long logicalTargetPos) {
  long physicalDelta = logicalTargetPos - logicalCurrentPos;
  if (physicalDelta > 0) {
    physicalDelta -= slopSteps;
    if (physicalDelta < 0) {
      physicalDelta = 0;
    }
  } else if (physicalDelta < 0) {
    physicalDelta += slopSteps;
    if (physicalDelta > 0) {
      physicalDelta = 0;
    }
  }
  return actualCurrentPos + physicalDelta;
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
  if (numberOfGears > 0) {
    long nearestGearIndex = lround((static_cast<double>(modPos) * static_cast<double>(numberOfGears)) /
                                   static_cast<double>(STEPS_PER_INDEXER_REV));
    if (nearestGearIndex >= numberOfGears) {
      nearestGearIndex = 0;
    } else if (nearestGearIndex < 0) {
      nearestGearIndex = 0;
    }
    currentGear = static_cast<int>(nearestGearIndex) + 1;
    if (currentGear < 1) {
      currentGear = 1;
    } else if (currentGear > numberOfGears) {
      currentGear = numberOfGears;
    }
  }
  return currentGear;
}

void applyStepperSpeed() {
  if (speedStepsPerSec < START_SPEED_STEPS_PER_SEC) {
    speedStepsPerSec = START_SPEED_STEPS_PER_SEC;
  }
  stepIntervalUs = computeTimerIntervalUsForSpeed(speedStepsPerSec, false);
}

uint32_t computeTimerIntervalUsForSpeed(float speed, bool highTorqueMode) {
  if (speed < START_SPEED_STEPS_PER_SEC) {
    speed = START_SPEED_STEPS_PER_SEC;
  }
  float commutationsPerSec = speed * 2.0f;
  if (highTorqueMode) {
    commutationsPerSec *= 0.5f;
  }
  uint32_t intervalUs = static_cast<uint32_t>(1000000.0f / commutationsPerSec);
  if (intervalUs < MIN_STEP_INTERVAL_US) {
    intervalUs = MIN_STEP_INTERVAL_US;
  }
  return intervalUs;
}

void runStepperToTargetOneStep() {
  static unsigned long settleStartMs = 0;

  if (!stepperEnabled) {
    timerMotionActive = false;
    currentSpeedStepsPerSec = START_SPEED_STEPS_PER_SEC;
    highTorqueModeActive = false;
    halfStepInProgress = false;
    settleStartMs = 0;
    if (timerStepIntervalUs != STEPPER_TIMER_IDLE_US) {
      timerStepIntervalRequestUs = STEPPER_TIMER_IDLE_US;
      timerStepIntervalDirty = true;
    }
    return;
  }
  if (stepperPosition == targetPosition) {
    if (halfStepInProgress) {
      timerMotionActive = true;
      return;
    }
    if (settleStartMs == 0) {
      settleStartMs = millis();
      timerMotionActive = false;
      return;
    }
    if (millis() - settleStartMs < 100) {
      timerMotionActive = false;
      return;
    }
    settleStartMs = 0;
    currentSpeedStepsPerSec = START_SPEED_STEPS_PER_SEC;
    highTorqueModeActive = false;
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
  settleStartMs = 0;

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
  long remainingSteps = labs(targetPosition - stepperPosition);
  if (remainingSteps < 1) {
    remainingSteps = 1;
  }
  float brakingSpeed = sqrtf(2.0f * accelStepsPerSec2 * static_cast<float>(remainingSteps));
  if (brakingSpeed < desiredSpeed) {
    desiredSpeed = brakingSpeed;
  }
  if (desiredSpeed < START_SPEED_STEPS_PER_SEC) {
    desiredSpeed = START_SPEED_STEPS_PER_SEC;
  }
  float dv = accelStepsPerSec2 * dt;
  if (currentSpeedStepsPerSec < desiredSpeed) {
    currentSpeedStepsPerSec += dv;
    if (currentSpeedStepsPerSec > desiredSpeed) {
      currentSpeedStepsPerSec = desiredSpeed;
    }
  } else if (currentSpeedStepsPerSec > desiredSpeed) {
    currentSpeedStepsPerSec -= dv;
    if (currentSpeedStepsPerSec < desiredSpeed) {
      currentSpeedStepsPerSec = desiredSpeed;
    }
  }
  if (currentSpeedStepsPerSec < START_SPEED_STEPS_PER_SEC) {
    currentSpeedStepsPerSec = START_SPEED_STEPS_PER_SEC;
  }
  bool enableHighTorque =
      USE_HIGH_TORQUE_MODE && (currentSpeedStepsPerSec >= HIGH_TORQUE_SPEED_THRESHOLD_STEPS_PER_SEC);
  highTorqueModeActive = enableHighTorque;

  uint32_t dynIntervalUs = computeTimerIntervalUsForSpeed(currentSpeedStepsPerSec, enableHighTorque);
  if (dynIntervalUs != timerStepIntervalUs) {
    timerStepIntervalRequestUs = dynIntervalUs;
    timerStepIntervalDirty = true;
  }

  timerMotionActive = true;
}

void setFirstLedColor(uint8_t r, uint8_t g, uint8_t b) {
  if (!USE_STATUS_LED) {
    return;
  }
  rgbLeds.setPixelColor(0, rgbLeds.Color(r, g, b));
  rgbLeds.show();
}

void IRAM_ATTR writeStepperOutputs(bool in1, bool in2, bool in3, bool in4) {
  const uint32_t pin1Mask = (1UL << stepperIn1);
  const uint32_t pin2Mask = (1UL << stepperIn2);
  const uint32_t pin3Mask = (1UL << stepperIn3);
  const uint32_t pin4Mask = (1UL << stepperIn4);
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
  outputCommand = OUTPUT_CMD_STOP;
  timerMotionActive = false;
  lastStepDir = 0;
  halfStepInProgress = false;
  timerStepIntervalRequestUs = STEPPER_TIMER_IDLE_US;
  timerStepIntervalDirty = true;
  interrupts();
  tbInStandby = true;
  stepperOutputsReleased = true;
}

void hardEnableStepperPins() {
  noInterrupts();
  outputCommand = OUTPUT_CMD_HOLD_PHASE;
  interrupts();
  delayMicroseconds(TB_STANDBY_WAKE_US);
  tbInStandby = false;
  stepperOutputsReleased = false;
}

void forceBothHBridgesOn() {
  writeStepperOutputs(true, false, true, false);
  tbInStandby = false;
  stepperOutputsReleased = false;
}

void applyDiagBridgeModeOutput() {
  if (diagBridgeMode == DiagBridgeMode::Off) {
    return;
  }
  digitalWrite(STEP1_IN1, LOW);
  digitalWrite(STEP1_IN2, LOW);
  digitalWrite(STEP1_IN3, LOW);
  digitalWrite(STEP1_IN4, LOW);
  digitalWrite(STEP2_IN1, LOW);
  digitalWrite(STEP2_IN2, LOW);
  digitalWrite(STEP2_IN3, LOW);
  digitalWrite(STEP2_IN4, LOW);

  if (diagBridgeMode == DiagBridgeMode::M1On) {
    digitalWrite(STEP1_IN1, HIGH);
  } else if (diagBridgeMode == DiagBridgeMode::M2On) {
    digitalWrite(STEP1_IN3, HIGH);
  } else if (diagBridgeMode == DiagBridgeMode::M3On) {
    digitalWrite(STEP2_IN1, HIGH);
  } else if (diagBridgeMode == DiagBridgeMode::M4On) {
    digitalWrite(STEP2_IN3, HIGH);
  }
  tbInStandby = false;
  stepperOutputsReleased = false;
}

void setStepperPhase(int phase) {
  switch ((phase % 8 + 8) % 8) {
    case 0:
      writeStepperOutputs(true, false, false, false);
      break;
    case 1:
      writeStepperOutputs(true, false, true, false);
      break;
    case 2:
      writeStepperOutputs(false, false, true, false);
      break;
    case 3:
      writeStepperOutputs(false, true, true, false);
      break;
    case 4:
      writeStepperOutputs(false, true, false, false);
      break;
    case 5:
      writeStepperOutputs(false, true, false, true);
      break;
    case 6:
      writeStepperOutputs(false, false, false, true);
      break;
    default:
      writeStepperOutputs(true, false, false, true);
      break;
  }
}

void IRAM_ATTR onStepperTimerISR() {
  isrTickCounter++;
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
  if (pos == tgt && !halfStepInProgress) {
    timerMotionActive = false;
    return;
  }

  int dir = lastStepDir;
  if (!halfStepInProgress) {
    dir = (tgt > pos) ? 1 : -1;
  } else if (dir == 0) {
    dir = (tgt >= pos) ? 1 : -1;
  }
  if (!halfStepInProgress && lastStepDir != 0 && dir != lastStepDir && REVERSAL_DWELL_US > 0) {
    writeStepperOutputs(false, false, false, false);
    ets_delay_us(REVERSAL_DWELL_US);
  }
  phaseIndex = (phaseIndex + dir + 8) % 8;
  setStepperPhase(phaseIndex);
  if (halfStepInProgress) {
    stepperPosition = pos + dir;
    isrStepCounter++;
    halfStepInProgress = false;
  } else {
    halfStepInProgress = true;
  }
  totalInterruptStepsTaken++;
  lastStepDir = dir;
}
