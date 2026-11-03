// Copyright © 2026 Isaiah Stone. All Rights Reserved.
// Unauthorized copying or distribution of this file is strictly prohibited.
#include "pins.h"
#include <Servo.h>
#include <Wire.h>
#include <NewPing.h>
#include <avr/interrupt.h>
#include <math.h>

// Servo on an Uno uses Timer1; NewPing::ping_timer uses Timer2. No analogWrite.
Servo drive[DRIVE_COUNT], sawEsc, armServo, forkServo, ballastServo;
NewPing sonar[SONAR_COUNT] = {
  NewPing(SONAR_PINS[SONAR_REAR], SONAR_PINS[SONAR_REAR], SONAR_MAX_CM),
  NewPing(SONAR_PINS[SONAR_FRONT], SONAR_PINS[SONAR_FRONT], SONAR_MAX_CM),
  NewPing(SONAR_PINS[SONAR_LEFT], SONAR_PINS[SONAR_LEFT], SONAR_MAX_CM),
  NewPing(SONAR_PINS[SONAR_RIGHT], SONAR_PINS[SONAR_RIGHT], SONAR_MAX_CM)
};

// Rising-to-rising PPM intervals: a >3 ms sync gap terminates the previous
// frame and starts the next. Publish only six fully in-range channel widths.
volatile uint16_t ppmBuilding[CHANNEL_COUNT], ppmPublished[CHANNEL_COUNT];
volatile uint8_t ppmCount = 0, ppmSeq = 0, ppmBadSeq = 0;
volatile bool ppmBad = false;
volatile uint32_t ppmLastRiseUs = 0, ppmPublishedAtUs = 0;
ISR(PCINT0_vect) {
  if (!(PINB & _BV(PB0))) return;  // D8 falling edge: ignore.
  const uint32_t now = micros();
  if (!ppmLastRiseUs) { ppmLastRiseUs = now; return; }
  const uint32_t width = now - ppmLastRiseUs;
  ppmLastRiseUs = now;
  if (width > PPM_GAP_US) {
    if (ppmCount == CHANNEL_COUNT && !ppmBad) {
      for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) ppmPublished[i] = ppmBuilding[i];
      ppmPublishedAtUs = now;
      ++ppmSeq;  // 8-bit atomic publication marker; copied with interrupts off.
    } else if (ppmCount) {
      ++ppmBadSeq;  // Invalid/short frame resets the continuous re-arm window.
    }
    ppmCount = 0;
    ppmBad = false;
  } else if (ppmCount < CHANNEL_COUNT) {
    if (width < PPM_MIN_US || width > PPM_MAX_US) ppmBad = true;
    ppmBuilding[ppmCount++] = (uint16_t)width;
  } else {
    ppmBad = true;  // More than six channels before sync: reject this frame.
  }
}

uint16_t ch[CHANNEL_COUNT] = {1500, 1500, 1000, 1500, 1000, 1000};
uint8_t seenSeq = 0, seenBadSeq = 0;
uint32_t lastValidUs = 0, rearmSinceMs = 0;
bool haveFrame = false, failsafe = true;
uint16_t driveUs[DRIVE_COUNT] = {1500, 1500, 1500, 1500};
uint16_t sawUs = SAW_OFF_US, forkUs = FORK_DOWN_US, ballastUs = BALLAST_CENTER_US;
uint8_t ballastState = BALLAST_STATE_CENTER;
bool armOut = false;
uint32_t armOutAtMs = 0, switchInAtMs = 0;

// Timer2 callback must only copy a result. All filtering happens in loop().
volatile bool pingReady = false;
volatile uint16_t pingEchoUs = 0;
uint8_t activeSonar = SONAR_COUNT - 1, sonarPos[SONAR_COUNT] = {0};
bool pingPending = false;
uint32_t pingStartedMs = 0, lastPingMs = 0;
uint16_t sonarSamples[SONAR_COUNT][SONAR_HISTORY], sonarMm[SONAR_COUNT];
void sonarCheck() {
  if (sonar[activeSonar].check_timer()) {
    pingEchoUs = (uint16_t)sonar[activeSonar].ping_result;
    pingReady = true;
  }
}

float batteryV = 0, pitchDeg = 0, rollDeg = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
uint32_t lastBatteryMs = 0, lastImuUs = 0, lastControlUs = 0;
uint32_t lastHitMs = 0, lastTipMs = 0, levelSinceMs = 0;
bool hitPlayed = false, tipPlayed = false, imuHealthy = false;
uint8_t audioBit = 0xFF;
uint32_t audioStartedMs = 0;
uint16_t lastLoopUs = 0;
#if DEBUG
uint32_t lastDebugMs = 0;
#endif

bool writeMpu(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0 && !Wire.getWireTimeoutFlag();
}

bool readImuRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, MPU_BURST_BYTES, (uint8_t)true) != MPU_BURST_BYTES ||
      Wire.getWireTimeoutFlag()) return false;
  ax = (int16_t)((uint16_t)Wire.read() << 8 | Wire.read());
  ay = (int16_t)((uint16_t)Wire.read() << 8 | Wire.read());
  az = (int16_t)((uint16_t)Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read(); // temperature register, unused
  gx = (int16_t)((uint16_t)Wire.read() << 8 | Wire.read());
  gy = (int16_t)((uint16_t)Wire.read() << 8 | Wire.read());
  gz = (int16_t)((uint16_t)Wire.read() << 8 | Wire.read());
  return true;
}

bool initImu() {
  return writeMpu(MPU_PWR_MGMT_1, MPU_WAKE_VALUE) &&
         writeMpu(MPU_ACCEL_CONFIG, MPU_ACCEL_8G) &&
         writeMpu(MPU_GYRO_CONFIG, MPU_GYRO_500DPS);
}
bool calibrateGyro() {
  int32_t sx = 0, sy = 0, sz = 0;
  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  // Keep the robot stationary for these 200 samples. setup() may delay;
  // loop() never does. A failed I2C calibration must not arm the saw.
  for (uint16_t i = 0; i < GYRO_CAL_SAMPLES; ++i) {
    if (!readImuRaw(ax, ay, az, gx, gy, gz)) return false;
    sx += gx; sy += gy; sz += gz;
    delay(5);
  }
  gyroBiasX = (float)sx / GYRO_CAL_SAMPLES;
  gyroBiasY = (float)sy / GYRO_CAL_SAMPLES;
  gyroBiasZ = (float)sz / GYRO_CAL_SAMPLES;
  pitchDeg = atan2f(-(float)ax, sqrtf((float)ay * ay + (float)az * az)) * RAD_TO_DEG_F;
    rollDeg = atan2f((float)ay, (float)az) * RAD_TO_DEG_F;
  lastImuUs = micros();
  return true;
}

bool writePcf(uint8_t value) {
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}
void soundRequest(uint8_t bit) {
  // LOW plays; HIGH/0xFF is idle. Releasing the previous bit lets a new
  // request replace it, without blocking the control or failsafe paths.
  if (bit > AUDIO_START) return;
  writePcf(0xFF);
  audioBit = 0xFF;
  if (writePcf((uint8_t)(0xFFu & ~(1u << bit)))) {
    audioBit = bit;
    audioStartedMs = millis();
  }
}
void soundService(uint32_t nowMs) {
  if (audioBit != 0xFF && nowMs - audioStartedMs >= SOUND_PULSE_MS) {
    writePcf(0xFF);
    audioBit = 0xFF;
  }
}

void forceSafeOutputs() {
  // Radio loss / CH3 failsafe: bypass ALL slew and arm movement delays.
  for (uint8_t i = 0; i < DRIVE_COUNT; ++i) {
    driveUs[i] = DRIVE_NEUTRAL_US;
    drive[i].writeMicroseconds(DRIVE_NEUTRAL_US);
  }
  sawUs = SAW_OFF_US; sawEsc.writeMicroseconds(SAW_OFF_US);
  armServo.writeMicroseconds(ARM_IN_US); armOut = false;
  armOutAtMs = 0; switchInAtMs = 0;
  forkUs = FORK_DOWN_US; forkServo.writeMicroseconds(forkUs);
  ballastUs = BALLAST_CENTER_US; ballastState = BALLAST_STATE_CENTER;
    ballastServo.writeMicroseconds(ballastUs);
  levelSinceMs = 0;
}
void enterFailsafe() {
  failsafe = true;
  rearmSinceMs = 0;
  forceSafeOutputs();
}

void readRadioAndSafety(uint32_t nowMs) {
  uint8_t s, b;
  uint32_t stamp;
  bool newFrame = false;
  noInterrupts();
  s = ppmSeq; b = ppmBadSeq; stamp = ppmPublishedAtUs;
  if (s != seenSeq) {
    for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) ch[i] = ppmPublished[i];
    newFrame = true;
  }
  interrupts();
  if (b != seenBadSeq) { seenBadSeq = b; rearmSinceMs = 0; }
  if (newFrame) {
    seenSeq = s;
    lastValidUs = stamp;
    haveFrame = true;
  }
  // FS-R6B keeps outputting CH1/2/4/5/6 after RF loss. Configure CH3's
  // receiver failsafe pulse <1050 us, BELOW normal throttle (~1100 us).
  const bool stale = !haveFrame || (uint32_t)(micros() - lastValidUs) > FAILSAFE_MS * 1000UL;
  const bool throttleLost = haveFrame && ch[CH_SAW] < CH3_FAILSAFE_US;
  if (stale || throttleLost) {
    if (!failsafe) enterFailsafe();
    else rearmSinceMs = 0;
    return;
  }
  if (failsafe) {
    // Continuous valid radio for 500 ms, arm switch IN, throttle near low.
    // A hazardous stick or arm state resets the window, not just its timer.
    const bool safeToRearm = ch[CH_SAW] > CH3_FAILSAFE_US + CH3_REARM_MARGIN_US &&
      ch[CH_SAW] <= CH3_REARM_MAX_US && ch[CH_ARM] <= ARM_SWITCH_IN_MAX_US;
    if (!safeToRearm) rearmSinceMs = 0;
    else if (newFrame && rearmSinceMs == 0) rearmSinceMs = nowMs;
    else if (rearmSinceMs != 0 && nowMs - rearmSinceMs >= REARM_MS) {
      failsafe = false;
      rearmSinceMs = 0;
    }
  } else if (ch[CH_ARM] < ARM_SWITCH_OUT_MIN_US && sawUs != SAW_OFF_US) {
    // Saw OFF immediately when SW.A goes IN (or becomes ambiguous).
    sawUs = SAW_OFF_US;
    sawEsc.writeMicroseconds(SAW_OFF_US);
  }
}

int16_t slew(int16_t from, int16_t to, uint16_t maxStep) {
  if (to > from + (int16_t)maxStep) return from + maxStep;
  if (to < from - (int16_t)maxStep) return from - maxStep;
  return to;
}
float axis(uint16_t pulse) {
  int16_t diff = (int16_t)pulse - (int16_t)STICK_CENTER_US;
  if (abs(diff) <= (int)STICK_DEADBAND_US) return 0.0f;
  return constrain((float)diff / STICK_SPAN_US, -1.0f, 1.0f);
}
void updateDrive() {
  const float y = axis(ch[CH_FORWARD]), x = axis(ch[CH_STRAFE]);
  const float r = axis(ch[CH_ROTATE]);
  float v[DRIVE_COUNT] = {y+x+r, y-x-r, y-x+r, y+x-r};
  float biggest = 1.0f;
  for (uint8_t i = 0; i < DRIVE_COUNT; ++i)
    if (fabsf(v[i]) > biggest) biggest = fabsf(v[i]);
  for (uint8_t i = 0; i < DRIVE_COUNT; ++i) {
    const int16_t target = constrain((int16_t)(DRIVE_NEUTRAL_US +
      DRIVE_MIX_SPAN_US * (v[i] / biggest) * MOTOR_DIR[i]), SERVO_MIN_US, SERVO_MAX_US);
    driveUs[i] = slew(driveUs[i], target, DRIVE_SLEW_US);
    drive[i].writeMicroseconds(driveUs[i]);
  }
}

void updateSawAndArm(uint32_t nowMs) {
  if (ch[CH_ARM] >= ARM_SWITCH_OUT_MIN_US) {
    switchInAtMs = 0;
    if (!armOut) {
      armServo.writeMicroseconds(ARM_OUT_US);
      armOut = true;
      armOutAtMs = nowMs;
    }
  } else {
    // Always stop the saw BEFORE starting the 150 ms arm-IN delay.
    if (sawUs != SAW_OFF_US) {
      sawUs = SAW_OFF_US;
      sawEsc.writeMicroseconds(SAW_OFF_US);
    }
    if (armOut) {
      if (switchInAtMs == 0) switchInAtMs = nowMs;
      if (nowMs - switchInAtMs >= ARM_IN_WAIT_MS) {
        armServo.writeMicroseconds(ARM_IN_US);
        armOut = false;
        armOutAtMs = 0;
      }
    }
  }
  uint16_t target = SAW_OFF_US;
  if (armOut && ch[CH_ARM] >= ARM_SWITCH_OUT_MIN_US &&
      nowMs - armOutAtMs >= SAW_ARM_WAIT_MS) {
    target = (uint16_t)map(constrain(ch[CH_SAW], CH3_NORMAL_MIN_US, SERVO_MAX_US),
      CH3_NORMAL_MIN_US, SERVO_MAX_US, SAW_OFF_US, SERVO_MAX_US);
    if (batteryV < BATTERY_SAW_CAP_V) target = min(target, SAW_LOW_BATTERY_MAX_US);
  }
  if (target > SAW_OFF_US && sawUs == SAW_OFF_US) soundRequest(AUDIO_SAW);
  sawUs = target;
  sawEsc.writeMicroseconds(sawUs);
}

void updateFork() {
  const uint16_t target = (uint16_t)map(constrain(ch[CH_FORK], PPM_MIN_US, PPM_MAX_US),
    PPM_MIN_US, PPM_MAX_US, FORK_DOWN_US, FORK_SELF_RIGHT_US);
  const uint16_t previous = forkUs;
  forkUs = slew(forkUs, target, FORK_SLEW_US);
  forkServo.writeMicroseconds(forkUs);
  if (previous < FORK_SOUND_US && forkUs >= FORK_SOUND_US) soundRequest(AUDIO_FORK);
}

void updateImuAndHit(uint32_t nowMs) {
  int16_t ax, ay, az, gx, gy, gz;
  if (!readImuRaw(ax, ay, az, gx, gy, gz)) {
    // An untrusted tilt estimate must not be used for automatic ballast.
    imuHealthy = false; pitchDeg = 0; rollDeg = 0;
    lastImuUs = micros();
    return;
  }
  const uint32_t t = micros();
  const float dt = (float)(t - lastImuUs) * 0.000001f;
  lastImuUs = t;
  const float ap = atan2f(-(float)ax, sqrtf((float)ay * ay + (float)az * az)) * RAD_TO_DEG_F;
    const float ar = atan2f((float)ay, (float)az) * RAD_TO_DEG_F;
    if (!imuHealthy || dt > MAX_IMU_DT_S) { pitchDeg = ap; rollDeg = ar; }
  else {
    pitchDeg = FILTER_GYRO * (pitchDeg + ((float)gy - gyroBiasY) /
      MPU_GYRO_LSB_PER_DPS * dt) + FILTER_ACCEL * ap;
    rollDeg = FILTER_GYRO * (rollDeg + ((float)gx - gyroBiasX) /
      MPU_GYRO_LSB_PER_DPS * dt) + FILTER_ACCEL * ar;
  }
  imuHealthy = true;
  const float a = (float)ax / MPU_ACCEL_LSB_PER_G;
  const float b = (float)ay / MPU_ACCEL_LSB_PER_G;
  const float c = (float)az / MPU_ACCEL_LSB_PER_G;
  const float hitThreshold = HIT_G + ((sawUs > SAW_OFF_US) ? HIT_SAW_EXTRA_G : 0.0f);
  if (a*a + b*b + c*c > hitThreshold*hitThreshold &&
      (!hitPlayed || nowMs - lastHitMs >= HIT_LOCKOUT_MS)) {
    hitPlayed = true; lastHitMs = nowMs; soundRequest(AUDIO_HIT);
  }
}

uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t=a; a=b; b=t; }
  if (b > c) { uint16_t t=b; b=c; c=t; }
  if (a > b) b=a;
  return b;
}
void sonarService(uint32_t nowMs) {
  if (pingPending) {
    bool done; uint16_t echo;
    noInterrupts(); done = pingReady; echo = pingEchoUs; pingReady = false; interrupts();
    if (done || nowMs - pingStartedMs >= SONAR_TIMEOUT_MS) {
      if (!done) NewPing::timer_stop(); // Never leave Timer2 running on no echo.
      uint16_t mm = echo ? min((uint16_t)((echo * 10UL + SONAR_ROUNDTRIP_US_PER_CM / 2) /
                                   SONAR_ROUNDTRIP_US_PER_CM), US_NO_ECHO_MM)
                         : US_NO_ECHO_MM;
      if (!done) mm = US_NO_ECHO_MM;
      sonarSamples[activeSonar][sonarPos[activeSonar]] = mm;
      sonarPos[activeSonar] = (sonarPos[activeSonar] + 1) % SONAR_HISTORY;
      sonarMm[activeSonar] = median3(sonarSamples[activeSonar][0],
        sonarSamples[activeSonar][1], sonarSamples[activeSonar][2]);
      pingPending = false;
    }
  }
  if (!pingPending && nowMs - lastPingMs >= PING_PERIOD_MS) {
    activeSonar = (activeSonar + 1) % SONAR_COUNT;
    pingReady = false; pingEchoUs = 0;
    pingPending = true;
    pingStartedMs = lastPingMs = nowMs;
    sonar[activeSonar].ping_timer(sonarCheck); // same TRIG/ECHO pin, Timer2 callback
  }
}

void tipSound(uint32_t nowMs) {
  if (!tipPlayed || nowMs - lastTipMs >= TIP_SOUND_LOCKOUT_MS) {
    tipPlayed = true; lastTipMs = nowMs;
    soundRequest(AUDIO_TIP);
  }
}
void updateBallast(uint32_t nowMs) {
  // Priority is intentional: rear, front, side hold, fork lift, settled center.
  if (sonarMm[SONAR_REAR] > US_LIFT_MM || (imuHealthy && pitchDeg < -TIP_DEG)) {
      ballastUs = BALLAST_REAR_US; ballastState = BALLAST_STATE_REAR;
      levelSinceMs = 0; tipSound(nowMs);
    } else if (sonarMm[SONAR_FRONT] > US_LIFT_MM || (imuHealthy && pitchDeg > TIP_DEG)) {
      ballastUs = BALLAST_FRONT_US; ballastState = BALLAST_STATE_FRONT;
      levelSinceMs = 0; tipSound(nowMs);
    } else if (sonarMm[SONAR_LEFT] > US_LIFT_MM || sonarMm[SONAR_RIGHT] > US_LIFT_MM ||
               (imuHealthy && fabsf(rollDeg) > TIP_DEG)) {
      ballastState = BALLAST_STATE_HOLD;
      levelSinceMs = 0; tipSound(nowMs); // Side tilt: hold the current position.
    } else if (forkUs > FORK_UP_US) {
      ballastUs = BALLAST_REAR_US; ballastState = BALLAST_STATE_REAR;
      levelSinceMs = 0;
    } else {
      if (levelSinceMs == 0) levelSinceMs = nowMs;
      if (nowMs - levelSinceMs >= LEVEL_SETTLE_MS) {
        ballastUs = BALLAST_CENTER_US; ballastState = BALLAST_STATE_CENTER;
      } else ballastState = BALLAST_STATE_HOLD;
    }
  ballastServo.writeMicroseconds(ballastUs);
}

void updateBattery(uint32_t nowMs) {
  if (nowMs - lastBatteryMs < BATTERY_PERIOD_MS) return;
  lastBatteryMs = nowMs;
  uint16_t sum = 0;
  for (uint8_t i = 0; i < BATTERY_SAMPLES; ++i) sum += analogRead(BATTERY_PIN);
  batteryV = ((float)sum / BATTERY_SAMPLES) * BATTERY_VOLTS_PER_COUNT;
}
#if DEBUG
void debugStatus(uint32_t nowMs) {
  if (nowMs - lastDebugMs < DEBUG_PERIOD_MS) return;
  lastDebugMs = nowMs;
  Serial.print(F("CH:"));
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) { Serial.print(' '); Serial.print(ch[i]); }
  Serial.print(F(" M:"));
  for (uint8_t i = 0; i < DRIVE_COUNT; ++i) { Serial.print(' '); Serial.print(driveUs[i]); }
  Serial.print(F(" saw:")); Serial.print(sawUs);
  Serial.print(F(" PR:")); Serial.print(pitchDeg, 1); Serial.print(','); Serial.print(rollDeg, 1);
  Serial.print(F(" sonar:"));
  for (uint8_t i = 0; i < SONAR_COUNT; ++i) { Serial.print(' '); Serial.print(sonarMm[i]); }
  Serial.print(F(" bal:"));
    switch (ballastState) {
      case BALLAST_STATE_REAR: Serial.print(F("REAR")); break;
      case BALLAST_STATE_FRONT: Serial.print(F("FRONT")); break;
      case BALLAST_STATE_HOLD: Serial.print(F("HOLD")); break;
      default: Serial.print(F("CENTER")); break;
    }
    Serial.print('/'); Serial.print(ballastUs);
  Serial.print(F(" bat:")); Serial.print(batteryV, 2);
  if (batteryV < BATTERY_WARNING_V) Serial.print(F(" LOW-BAT"));
  if (!imuHealthy) Serial.print(F(" IMU-ERR"));
  Serial.print(F(" FS:")); Serial.print(failsafe ? F("yes") : F("no"));
  Serial.print(F(" loop-us:")); Serial.println(lastLoopUs);
}
#endif

void setup() {
  // Attach every output and command neutral/OFF before any sensor/radio work.
  for (uint8_t i = 0; i < DRIVE_COUNT; ++i) drive[i].attach(DRIVE_PINS[i]);
  sawEsc.attach(SAW_PIN); armServo.attach(ARM_PIN);
  forkServo.attach(FORK_PIN); ballastServo.attach(BALLAST_PIN);
  forceSafeOutputs();
  delay(ESC_ARM_MS); // Setup-only ESC arming; no delay() anywhere in loop().

#if DEBUG
  Serial.begin(115200); // TX is D1; never read from D0.
#endif
  Wire.begin();
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
  if (!initImu() || !writePcf(0xFF) || !calibrateGyro()) {
    // Missing sensor/I2C at boot: leave all actuators safe until corrected and reset.
    forceSafeOutputs();
    for (;;) { delay(100); }
  }
  for (uint8_t i = 0; i < SONAR_COUNT; ++i) {
    sonarPos[i] = 0; sonarMm[i] = US_NO_ECHO_MM;
    for (uint8_t j = 0; j < SONAR_HISTORY; ++j) sonarSamples[i][j] = US_NO_ECHO_MM;
  }
  updateBattery(millis());
  soundRequest(AUDIO_START); // Taunt/startup only, never a radio channel.
  pinMode(PPM_PIN, INPUT);
  PCIFR = _BV(PCIF0); PCMSK0 |= _BV(PCINT0); PCICR |= _BV(PCIE0);
  // No ballast-mode channel: AUTO throughout. Startup remains in FAILSAFE
  // until 500 ms of valid frames with throttle low and arm switch IN.
  lastPingMs = millis();
  lastControlUs = micros();
}

void loop() {
  const uint32_t startUs = micros();
  const uint32_t nowMs = millis();
  readRadioAndSafety(nowMs); // Safety first, not gated by the 5 ms step.
  soundService(nowMs);
  sonarService(nowMs);
  if ((uint32_t)(micros() - lastControlUs) >= CONTROL_STEP_US) {
    lastControlUs = micros();
    updateBattery(nowMs);
    if (!failsafe) {
      updateDrive();
      updateSawAndArm(nowMs);
      updateFork();
    }
    updateImuAndHit(nowMs);  // IMU read each 5 ms step, including FAILSAFE.
    if (!failsafe) updateBallast(nowMs);
    lastLoopUs = (uint16_t)(micros() - startUs);
#if DEBUG
    debugStatus(nowMs);
#endif
  }
}
