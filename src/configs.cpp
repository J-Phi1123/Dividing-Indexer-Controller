#include "app.h"

void loadControlSettings() {
  prefs.begin("ctrlcfg", true);
  backlashSteps = prefs.getLong("backlash", 0);
  slopSteps = prefs.getLong("slop", 0);
  degreeStepSetting = prefs.getFloat("deg_step", 10.0f);
  gearModule = prefs.getFloat("gear_module", gearModule);
  gearPressureAngleDeg = prefs.getFloat("gear_pa", gearPressureAngleDeg);
  speedStepsPerSec = prefs.getFloat("speed", speedStepsPerSec);
  accelStepsPerSec2 = prefs.getFloat("accel", accelStepsPerSec2);
  numberOfGears = prefs.getInt("gears", numberOfGears);
  uiMoveUnit = prefs.getUChar("move_unit", static_cast<uint8_t>(uiMoveUnit)) == static_cast<uint8_t>(MoveUnit::Degrees)
                   ? MoveUnit::Degrees
                   : MoveUnit::Gears;
  long savedPos = prefs.getLong("index_pos", 0);
  long savedLogicalPos = prefs.getLong("index_logical", savedPos);
  int savedPort = prefs.getInt("stepper_port", STEPPER_PORT);
  prefs.end();
  if (backlashSteps < 0) backlashSteps = 0;
  if (degreeStepSetting <= 0.0f) degreeStepSetting = 10.0f;
  if (gearModule <= 0.0f) gearModule = 1.0f;
  if (gearPressureAngleDeg <= 0.0f) gearPressureAngleDeg = 20.0f;
  if (gearPressureAngleDeg > 45.0f) gearPressureAngleDeg = 45.0f;
  if (speedStepsPerSec < 5.0f) speedStepsPerSec = 5.0f;
  if (speedStepsPerSec > 10000.0f) speedStepsPerSec = 10000.0f;
  if (accelStepsPerSec2 < 5.0f) accelStepsPerSec2 = 5.0f;
  if (accelStepsPerSec2 > 10000.0f) accelStepsPerSec2 = 10000.0f;
  if (numberOfGears < 1) numberOfGears = 1;
  recalcIndexerTicks();
  long wrappedPos = modPositive(savedPos, STEPS_PER_INDEXER_REV);
  wrappedPos -= modPositive(wrappedPos, COMMUTATION_STATES_PER_FULL_STEP);
  noInterrupts();
  stepperPosition = wrappedPos;
  targetPosition = wrappedPos;
  commandedStepsFromZero = static_cast<double>(wrappedPos);
  timerMotionActive = false;
  lastCommandDir = 0;
  halfStepInProgress = false;
  interrupts();
  syncDegreeIdealToPosition(wrappedPos);
  syncIndexedLogicalPosition(savedLogicalPos);
  applyStepperPortSelection(static_cast<uint8_t>(savedPort));
  uiMoveAmount = (uiMoveUnit == MoveUnit::Degrees) ? degreeStepSetting : 1.0f;
}

void saveControlSettings() {
  prefs.begin("ctrlcfg", false);
  prefs.putLong("backlash", backlashSteps);
  prefs.putLong("slop", slopSteps);
  prefs.putFloat("deg_step", degreeStepSetting);
  prefs.putFloat("gear_module", gearModule);
  prefs.putFloat("gear_pa", gearPressureAngleDeg);
  prefs.putFloat("speed", speedStepsPerSec);
  prefs.putFloat("accel", accelStepsPerSec2);
  prefs.putInt("gears", numberOfGears);
  prefs.putUChar("move_unit", static_cast<uint8_t>(uiMoveUnit));
  prefs.putLong("index_pos", getStepperPositionAtomic());
  prefs.putLong("index_logical", indexedLogicalPosition);
  prefs.putInt("stepper_port", stepperPort);
  prefs.end();
}

void loadPresets() {
  for (int i = 0; i < 3; i++) {
    presets[i].name = "Preset " + String(i + 1);
    presets[i].gears = numberOfGears;
    presets[i].degreeStep = degreeStepSetting;
    presets[i].speed = speedStepsPerSec;
    presets[i].accel = accelStepsPerSec2;
    presets[i].gearModule = gearModule;
    presets[i].gearPressureAngleDeg = gearPressureAngleDeg;
  }
  prefs.begin("presets", true);
  for (int i = 0; i < 3; i++) {
    String key = "p" + String(i + 1);
    String raw = prefs.getString(key.c_str(), "");
    if (raw.length() == 0) {
      continue;
    }
    int a = raw.indexOf('|');
    int b = raw.indexOf('|', a + 1);
    int c = raw.indexOf('|', b + 1);
    int d = raw.indexOf('|', c + 1);
    if (a < 0 || b < 0 || c < 0 || d < 0) {
      continue;
    }
    presets[i].name = raw.substring(0, a);
    presets[i].gears = raw.substring(a + 1, b).toInt();
    presets[i].degreeStep = raw.substring(b + 1, c).toFloat();
    presets[i].speed = raw.substring(c + 1, d).toFloat();
    int e = raw.indexOf('|', d + 1);
    int f = (e >= 0) ? raw.indexOf('|', e + 1) : -1;
    if (e >= 0) {
      presets[i].accel = raw.substring(d + 1, e).toFloat();
      if (f >= 0) {
        presets[i].gearModule = raw.substring(e + 1, f).toFloat();
        presets[i].gearPressureAngleDeg = raw.substring(f + 1).toFloat();
      } else {
        presets[i].gearModule = gearModule;
        presets[i].gearPressureAngleDeg = gearPressureAngleDeg;
      }
    } else {
      presets[i].accel = raw.substring(d + 1).toFloat();
      presets[i].gearModule = gearModule;
      presets[i].gearPressureAngleDeg = gearPressureAngleDeg;
    }
    if (presets[i].gears < 1) presets[i].gears = 1;
    if (presets[i].degreeStep <= 0.0f) presets[i].degreeStep = 10.0f;
    if (presets[i].speed < 5.0f) presets[i].speed = 5.0f;
    if (presets[i].accel < 5.0f) presets[i].accel = 5.0f;
    if (presets[i].gearModule <= 0.0f) presets[i].gearModule = 1.0f;
    if (presets[i].gearPressureAngleDeg <= 0.0f) presets[i].gearPressureAngleDeg = 20.0f;
    if (presets[i].gearPressureAngleDeg > 45.0f) presets[i].gearPressureAngleDeg = 45.0f;
  }
  prefs.end();
}

void savePresetSlot(int slot) {
  if (slot < 0 || slot >= 3) {
    return;
  }
  String raw = presets[slot].name + "|" + String(presets[slot].gears) + "|" +
               String(presets[slot].degreeStep, 3) + "|" + String(presets[slot].speed, 1) +
               "|" + String(presets[slot].accel, 1) + "|" + String(presets[slot].gearModule, 3) +
               "|" + String(presets[slot].gearPressureAngleDeg, 1);
  prefs.begin("presets", false);
  String key = "p" + String(slot + 1);
  prefs.putString(key.c_str(), raw);
  prefs.end();
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
    if (encBuf != nullptr) free(encBuf);
    if (outBuf != nullptr) free(outBuf);
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

bool parseIpArg(const String& s, IPAddress& out) {
  return out.fromString(s);
}
