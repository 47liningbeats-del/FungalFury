// Copyright © 2026 Isaiah Stone. All Rights Reserved.
// Unauthorized copying or distribution of this file is strictly prohibited.
#include <Servo.h>
#include <NewPing.h>
#include <DFRobotDFPlayerMini.h>

/*============================== TUNABLE CONSTANTS ==============================
  Bench-calibrate every item marked CALIBRATE before fitting/arming the robot.
  Pin assignments are fixed to the user's wiring and must not be changed.
*/
// Pin assignments (fixed wiring)
const uint8_t PIN_PPM = 2;                 // PPM interrupt input
const uint8_t PIN_MOTOR_L_IN1 = 3;          // DRV8833 AIN1
const uint8_t PIN_MOTOR_L_IN2 = 11;         // DRV8833 AIN2
const uint8_t PIN_MOTOR_R_IN1 = 5;          // DRV8833 BIN1
const uint8_t PIN_MOTOR_R_IN2 = 6;          // DRV8833 BIN2
const uint8_t PIN_DRIVER_SLEEP = A2;        // DRV8833 SLP
const uint8_t PIN_ESC = 10;                 // saw ESC signal
const uint8_t PIN_ARM_SERVO = 9;            // saw-arm servo
const uint8_t PIN_FORK_SERVO = 7;           // fork servo
const uint8_t PIN_BALLAST_SERVO = 12;       // ballast servo
const uint8_t PIN_US_FRONT = 4;
const uint8_t PIN_US_REAR = 8;
const uint8_t PIN_US_LEFT = 13;
const uint8_t PIN_US_RIGHT = A1;
const uint8_t PIN_VBAT_DRIVE = A0;
const uint8_t PIN_VBAT_SERVO = A3;

// Radio / failsafe settings
const uint8_t RADIO_CHANNEL_COUNT = 6;
const uint16_t PPM_SYNC_GAP_US = 3000;      // PPM frame separator threshold
const uint16_t PPM_VALID_MIN_US = 900;      // accepted channel minimum
const uint16_t PPM_VALID_MAX_US = 2100;     // accepted channel maximum
const uint16_t RADIO_FAILSAFE_MS = 100;     // lose-valid-frame timeout
const uint16_t RADIO_CENTER_US = 1500;      // centered stick value
const uint16_t RADIO_DEADBAND_US = 40;      // drive stick deadband
const uint16_t SAW_STICK_SAFE_US = 1100;    // must observe CH3 below this to arm
const uint16_t ARM_SWITCH_OUT_US = 1500;    // CH5 above this means arm OUT
const uint16_t FORKS_DOWN_THRESHOLD_US = 1100; // front range sensor enabled below this
const uint16_t FORK_HALF_TRAVEL_US = 1225;  // CALIBRATE for fork sound crossing point

// Drive settings
const int8_t LEFT_DIR = 1;                  // CALIBRATE: set -1 if left motor direction reversed
const int8_t RIGHT_DIR = 1;                 // CALIBRATE: set -1 if right motor direction reversed
const uint8_t DRIVE_MAX_PWM = 180;          // requested approximately 70% motor drive
const uint8_t DRIVE_RAMP_STEP = 20;          // maximum duty change per ramp tick
const uint16_t DRIVE_RAMP_INTERVAL_MS = 10;  // motor ramp interval

// Saw / ESC settings
const uint16_t ESC_STOP_US = 1000;           // ESC low/off command
const uint16_t ESC_FULL_US = 2000;           // maximum commanded pulse
const uint16_t ESC_ARMING_MS = 3000;         // boot ESC arming interval
const uint16_t ARM_SETTLE_MS = 600;          // wait after arm reaches OUT
const uint16_t SPIN_DOWN_MS = 1500;          // wait after saw stop before retract
const uint16_t SAW_STICK_MIN_US = 1100;      // CH3 active range lower endpoint
const uint16_t SAW_STICK_MAX_US = 2000;      // CH3 active range upper endpoint
const uint16_t SAW_LIMITED_MAX_US = 1500;    // low-drive-battery cap

// Servo positions and slew settings (CALIBRATE all position values on bench)
const uint16_t ARM_OUT_US = 600;             // requested placeholder: extended, about -10 degrees
const uint16_t ARM_IN_US = 2350;             // requested placeholder: folded, about 172 degrees
const uint16_t ARM_SERVO_STEP_US = 20;        // arm movement step
const uint16_t ARM_SERVO_INTERVAL_MS = 20;   // arm movement interval
const uint16_t FORK_DOWN_US = 1000;           // CALIBRATE: fork fully down
const uint16_t FORK_UP_US = 1450;             // CALIBRATE: fork fully up
const uint16_t BALLAST_REAR_US = 1250;        // CALIBRATE: -16 mm
const uint16_t BALLAST_CENTER_US = 1500;      // CALIBRATE: center
const uint16_t BALLAST_FRONT_US = 1750;       // CALIBRATE: +16 mm
const int16_t BALLAST_TRIM_LIMIT_US = 100;    // CH4 user trim limit in pulse microseconds
const uint8_t BALLAST_AUTO_OFFSET_US = 250;   // CALIBRATE: offset corresponding to 16 mm
const uint8_t BALLAST_RAMP_STEP_US = 10;      // maximum ballast movement per tick
const uint16_t SERVO_RAMP_INTERVAL_MS = 20;   // servo movement interval

// Ultrasonic sensing / tip balance
const uint16_t SENSOR_PING_INTERVAL_MS = 30;  // one ping cadence
const uint8_t SENSOR_MAX_DISTANCE_CM = 50;    // NewPing maximum distance
const uint16_t LIFT_MM = 80;                 // side/front/rear lift threshold
const uint16_t SENSOR_NO_ECHO_MM = 500;      // zero reading is treated as far

// Battery measurement and warning thresholds
const uint16_t BATTERY_SAMPLE_INTERVAL_MS = 200; // ADC sampling interval
const uint8_t BATTERY_AVERAGE_COUNT = 8;      // number of ADC samples averaged
const float ADC_REFERENCE_VOLTS = 5.0f;       // CALIBRATE to actual MCU 5V rail
const float BATTERY_DIVIDER_MULTIPLIER = 2.0f; // equal 100k/100k divider
const float VBAT_WARN = 7.0f;                 // low-battery sound threshold
const float VBAT_LOW = 6.6f;                  // saw power limiting threshold
const uint16_t LOW_BATTERY_REPEAT_MS = 30000; // warning repeat interval

// Sound / DFPlayer settings
const uint8_t DFPLAYER_VOLUME = 25;            // requested volume
const uint16_t SOUND_COOLDOWN_MS = 1000;      // per-track minimum repeat interval
const uint16_t TRACK_STARTUP = 1;             // /mp3/0001.mp3
const uint16_t TRACK_SAW_SPINUP = 2;          // /mp3/0002.mp3
const uint16_t TRACK_FORK_LIFT = 3;            // /mp3/0003.mp3
const uint16_t TRACK_TIP_ALARM = 4;            // /mp3/0004.mp3
const uint16_t TRACK_LOW_BATTERY = 5;          // /mp3/0005.mp3
const uint16_t TRACK_ARM_OUT = 6;              // /mp3/0006.mp3
const uint16_t SAW_SOUND_THRESHOLD_US = 1200;  // sound when saw first rises above this

// Timing / numerical safeguards
const uint8_t MOTOR_COUNT = 2;
const uint8_t BATTERY_COUNT = 2;
const uint8_t SENSOR_COUNT = 4;
const uint8_t SOUND_TRACK_COUNT = 7;           // track IDs 1..6, index zero unused
const uint16_t ADC_MAX_COUNT = 1023;
const uint16_t FRAME_MIN_CHANNELS = 6;
const uint8_t PPM_BUFFER_SIZE = 6;
const uint16_t UINT16_MAX_VALUE = 65535;

/*================================= OBJECTS ====================================*/
Servo sawEsc;
Servo sawArmServo;
Servo forkServo;
Servo ballastServo;
NewPing sonarFront(PIN_US_FRONT, PIN_US_FRONT, SENSOR_MAX_DISTANCE_CM);
NewPing sonarRear(PIN_US_REAR, PIN_US_REAR, SENSOR_MAX_DISTANCE_CM);
NewPing sonarLeft(PIN_US_LEFT, PIN_US_LEFT, SENSOR_MAX_DISTANCE_CM);
NewPing sonarRight(PIN_US_RIGHT, PIN_US_RIGHT, SENSOR_MAX_DISTANCE_CM);
DFRobotDFPlayerMini dfPlayer;

/*============================== RADIO ISR DATA ================================*/
volatile uint16_t ppmWorking[RADIO_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0};
volatile uint16_t ppmPublished[RADIO_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0};
volatile uint8_t ppmWorkingCount = 0;
volatile uint8_t ppmPublishedCount = 0;
volatile uint32_t ppmLastEdgeUs = 0;
volatile uint32_t ppmLastValidFrameUs = 0;
volatile uint8_t ppmFrameGeneration = 0;
uint16_t radioUs[RADIO_CHANNEL_COUNT] = {1500, 1500, 1000, 1500, 1000, 1500};
uint8_t consumedFrameGeneration = 0;
bool radioValid = false;
uint32_t lastValidRadioMs = 0;

void ppmEdgeISR() {
  const uint32_t nowUs = micros();
  const uint32_t pulseUs = nowUs - ppmLastEdgeUs;
  ppmLastEdgeUs = nowUs;

  if (pulseUs > PPM_SYNC_GAP_US) {
    // Publish only a complete, range-valid six-channel frame.
    if (ppmWorkingCount == RADIO_CHANNEL_COUNT) {
      bool valid = true;
      for (uint8_t i = 0; i < RADIO_CHANNEL_COUNT; ++i) {
        if (ppmWorking[i] < PPM_VALID_MIN_US || ppmWorking[i] > PPM_VALID_MAX_US) {
          valid = false;
          break;
        }
      }
      if (valid) {
        for (uint8_t i = 0; i < RADIO_CHANNEL_COUNT; ++i) ppmPublished[i] = ppmWorking[i];
        ppmPublishedCount = RADIO_CHANNEL_COUNT;
        ppmLastValidFrameUs = nowUs;
        ++ppmFrameGeneration;
      }
    }
    ppmWorkingCount = 0;
    return;
  }

  if (ppmWorkingCount < RADIO_CHANNEL_COUNT) {
    ppmWorking[ppmWorkingCount] = (pulseUs <= UINT16_MAX_VALUE) ? (uint16_t)pulseUs : 0;
    ++ppmWorkingCount;
  } else {
    // More than six pulses before sync: discard this malformed frame.
    ppmWorkingCount = 0;
  }
}

/*=============================== RUNTIME STATE ================================*/
uint32_t bootStartedMs = 0;
uint32_t driveRampAtMs = 0;
uint32_t servoRampAtMs = 0;
uint32_t sensorPingAtMs = 0;
uint32_t batterySampleAtMs = 0;
uint32_t lowBatterySoundAtMs = 0;
uint32_t armOutSinceMs = 0;
uint32_t sawStopSinceMs = 0;
bool sawArmed = false;
bool sawSawStickSeenLow = false;
bool armOutCommanded = false;
bool armRetractionPending = false;
bool armHasReachedOut = false;
bool startupSoundQueued = false;
bool radioWasValid = false;
bool forksWereAboveHalf = false;
uint16_t escPulseUs = ESC_STOP_US;
uint16_t armCurrentUs = ARM_IN_US;
uint16_t forkCurrentUs = FORK_DOWN_US;
uint16_t ballastCurrentUs = BALLAST_CENTER_US;
int16_t motorDutyCurrent[MOTOR_COUNT] = {0, 0};
int16_t motorDutyTarget[MOTOR_COUNT] = {0, 0};
uint16_t sonarMm[SENSOR_COUNT] = {0, 0, 0, 0};
uint8_t nextSensor = 0;
int16_t ballastAutoOffsetUs = 0;
uint16_t batteryAdcSamples[BATTERY_COUNT][BATTERY_AVERAGE_COUNT] = {{0}};
uint8_t batterySampleIndex = 0;
uint8_t batterySamplesFilled = 0;
float batteryVolts[BATTERY_COUNT] = {0.0f, 0.0f};
uint32_t lastSoundAt[SOUND_TRACK_COUNT] = {0, 0, 0, 0, 0, 0, 0};
uint16_t pendingSoundTrack = 0;

/*================================= HELPERS ====================================*/
uint16_t clampPulse(int32_t value, uint16_t low, uint16_t high) {
  if (value < low) return low;
  if (value > high) return high;
  return (uint16_t)value;
}

int16_t channelToSigned(uint16_t pulseUs) {
  int16_t delta = (int16_t)pulseUs - (int16_t)RADIO_CENTER_US;
  if (delta >= -(int16_t)RADIO_DEADBAND_US && delta <= (int16_t)RADIO_DEADBAND_US) return 0;
  if (delta > 0) delta -= (int16_t)RADIO_DEADBAND_US;
  else delta += (int16_t)RADIO_DEADBAND_US;
  return delta;
}

uint16_t readChannel(uint8_t channelIndex) {
  return radioUs[channelIndex];
}

void requestSound(uint16_t track) {
  if (track > 0 && track < SOUND_TRACK_COUNT) pendingSoundTrack = track;
}

/*================================= FUNCTIONS ==================================*/
void readRadio() {
  uint8_t generation;
  uint16_t localFrame[RADIO_CHANNEL_COUNT];
  noInterrupts();
  generation = ppmFrameGeneration;
  if (generation != consumedFrameGeneration && ppmPublishedCount == RADIO_CHANNEL_COUNT) {
    for (uint8_t i = 0; i < RADIO_CHANNEL_COUNT; ++i) localFrame[i] = ppmPublished[i];
  }
  interrupts();

  if (generation != consumedFrameGeneration && ppmPublishedCount == RADIO_CHANNEL_COUNT) {
    bool valid = true;
    for (uint8_t i = 0; i < RADIO_CHANNEL_COUNT; ++i) {
      if (localFrame[i] < PPM_VALID_MIN_US || localFrame[i] > PPM_VALID_MAX_US) valid = false;
    }
    if (valid) {
      for (uint8_t i = 0; i < RADIO_CHANNEL_COUNT; ++i) radioUs[i] = localFrame[i];
      lastValidRadioMs = millis();
      consumedFrameGeneration = generation;
      radioValid = true;
    }
  }

  if (radioValid && (uint32_t)(millis() - lastValidRadioMs) > RADIO_FAILSAFE_MS) radioValid = false;

  if (radioValid != radioWasValid) {
    if (!radioValid) {
      sawArmed = false;                 // every failsafe requires a fresh low-stick re-arm
      sawSawStickSeenLow = false;
      armRetractionPending = false;    // failsafe holds all servo positions
      motorDutyTarget[0] = 0;
      motorDutyTarget[1] = 0;
      motorDutyCurrent[0] = 0;
      motorDutyCurrent[1] = 0;
      analogWrite(PIN_MOTOR_L_IN1, 0);
      analogWrite(PIN_MOTOR_L_IN2, 0);
      analogWrite(PIN_MOTOR_R_IN1, 0);
      analogWrite(PIN_MOTOR_R_IN2, 0);
      digitalWrite(PIN_DRIVER_SLEEP, LOW);
      escPulseUs = ESC_STOP_US;
      sawEsc.writeMicroseconds(ESC_STOP_US);
    }
    radioWasValid = radioValid;
  }

  if (radioValid && !sawArmed && readChannel(2) < SAW_STICK_SAFE_US) {
    sawSawStickSeenLow = true;
  }
  // Armed latch remains set until failsafe; stick-down observation is required first.
  if (radioValid && !sawArmed && sawSawStickSeenLow && readChannel(2) >= SAW_STICK_SAFE_US) {
    sawArmed = true;
  }
}

void writeMotor(uint8_t side, int16_t signedDuty) {
  const uint8_t in1 = (side == 0) ? PIN_MOTOR_L_IN1 : PIN_MOTOR_R_IN1;
  const uint8_t in2 = (side == 0) ? PIN_MOTOR_L_IN2 : PIN_MOTOR_R_IN2;
  const int8_t direction = (side == 0) ? LEFT_DIR : RIGHT_DIR;
  int16_t adjusted = signedDuty * direction;
  uint8_t duty = (uint8_t)constrain(abs(adjusted), 0, DRIVE_MAX_PWM);
  if (adjusted > 0) {
    analogWrite(in1, duty);
    analogWrite(in2, 0);
  } else if (adjusted < 0) {
    analogWrite(in1, 0);
    analogWrite(in2, duty);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

void updateDrive() {
  if (!radioValid) {
    digitalWrite(PIN_DRIVER_SLEEP, LOW);
    return;
  }
  digitalWrite(PIN_DRIVER_SLEEP, HIGH);

  const int16_t steer = channelToSigned(readChannel(0));
  const int16_t throttle = channelToSigned(readChannel(1));
  float left = (float)(throttle + steer) / 460.0f;
  float right = (float)(throttle - steer) / 460.0f;
  // Scale combined stick values to -1..1 while preserving their ratio.
  const float largest = max(fabs(left), fabs(right));
  if (largest > 1.0f) {
    left /= largest;
    right /= largest;
  }
  motorDutyTarget[0] = (int16_t)(left * DRIVE_MAX_PWM);
  motorDutyTarget[1] = (int16_t)(right * DRIVE_MAX_PWM);

  const uint32_t now = millis();
  if ((uint32_t)(now - driveRampAtMs) >= DRIVE_RAMP_INTERVAL_MS) {
    driveRampAtMs = now;
    for (uint8_t side = 0; side < MOTOR_COUNT; ++side) {
      int16_t current = motorDutyCurrent[side];
      const int16_t target = motorDutyTarget[side];
      if (target > current) current = min((int16_t)(current + DRIVE_RAMP_STEP), target);
      else if (target < current) current = max((int16_t)(current - DRIVE_RAMP_STEP), target);
      motorDutyCurrent[side] = current;
      writeMotor(side, current);
    }
  }
}

void updateSaw() {
  const uint32_t now = millis();
  if ((uint32_t)(now - bootStartedMs) < ESC_ARMING_MS) {
    escPulseUs = ESC_STOP_US;
    sawEsc.writeMicroseconds(ESC_STOP_US);
    return;
  }
  if (!radioValid || !sawArmed) {
    escPulseUs = ESC_STOP_US;
    sawEsc.writeMicroseconds(ESC_STOP_US);
    return;
  }

  const bool wantsArmOut = readChannel(4) > ARM_SWITCH_OUT_US;
  if (wantsArmOut) {
    armRetractionPending = false;
    if (!armOutCommanded) {
      armOutCommanded = true;
      armHasReachedOut = false;
      requestSound(TRACK_ARM_OUT);
    }
    if (armCurrentUs == ARM_OUT_US && !armHasReachedOut) {
      armHasReachedOut = true;
      armOutSinceMs = now;
    }
  } else {
    if (armOutCommanded && !armRetractionPending) {
      escPulseUs = ESC_STOP_US;
      sawEsc.writeMicroseconds(ESC_STOP_US); // immediate stop on switch IN
      sawStopSinceMs = now;
      armRetractionPending = true;
    }
    if (!armOutCommanded && armCurrentUs != ARM_IN_US) {
      // Do not force motion during normal operation unless the switch requests IN.
      armRetractionPending = true;
      sawStopSinceMs = now;
    }
  }

  if (!wantsArmOut && armRetractionPending && (uint32_t)(now - sawStopSinceMs) >= SPIN_DOWN_MS) {
    armOutCommanded = false;
    armHasReachedOut = false;
  }

  uint16_t requested = ESC_STOP_US;
  const bool settled = armOutCommanded && armHasReachedOut &&
      ((uint32_t)(now - armOutSinceMs) >= ARM_SETTLE_MS);
  if (wantsArmOut && settled) {
    uint16_t ch3 = readChannel(2);
    if (ch3 > SAW_STICK_MIN_US) {
      uint16_t bounded = clampPulse(ch3, SAW_STICK_MIN_US, SAW_STICK_MAX_US);
      requested = (uint16_t)map(bounded, SAW_STICK_MIN_US, SAW_STICK_MAX_US, ESC_STOP_US, ESC_FULL_US);
      if (batteryVolts[0] > 0.0f && batteryVolts[0] < VBAT_LOW && requested > SAW_LIMITED_MAX_US) {
        requested = SAW_LIMITED_MAX_US;
      }
    }
  }
  if (requested == ESC_STOP_US) {
    if (escPulseUs != ESC_STOP_US) sawStopSinceMs = now;
    escPulseUs = ESC_STOP_US;
  } else {
    if (escPulseUs <= SAW_SOUND_THRESHOLD_US && requested > SAW_SOUND_THRESHOLD_US) requestSound(TRACK_SAW_SPINUP);
    escPulseUs = requested;
  }
  sawEsc.writeMicroseconds(escPulseUs);
}

void updateServos() {
  if (!radioValid) return; // servos hold their last command during failsafe
  const uint32_t now = millis();
  if ((uint32_t)(now - servoRampAtMs) < SERVO_RAMP_INTERVAL_MS) return;
  servoRampAtMs = now;

  const bool wantsOut = readChannel(4) > ARM_SWITCH_OUT_US;
  uint16_t armTarget = ARM_IN_US;
  if (wantsOut) armTarget = ARM_OUT_US;
  else if (armOutCommanded || armRetractionPending) armTarget = ARM_IN_US;
  if (armCurrentUs < armTarget) armCurrentUs = min((uint16_t)(armCurrentUs + ARM_SERVO_STEP_US), armTarget);
  else if (armCurrentUs > armTarget) armCurrentUs = max((uint16_t)(armCurrentUs - ARM_SERVO_STEP_US), armTarget);
  sawArmServo.writeMicroseconds(armCurrentUs);

  const uint16_t ch6 = readChannel(5);
  const uint16_t forkTarget = (uint16_t)map(ch6, PPM_VALID_MIN_US, PPM_VALID_MAX_US, FORK_DOWN_US, FORK_UP_US);
  if (forkCurrentUs < forkTarget) forkCurrentUs = min((uint16_t)(forkCurrentUs + ARM_SERVO_STEP_US), forkTarget);
  else if (forkCurrentUs > forkTarget) forkCurrentUs = max((uint16_t)(forkCurrentUs - ARM_SERVO_STEP_US), forkTarget);
  forkServo.writeMicroseconds(forkCurrentUs);

  const bool forksAboveHalf = forkCurrentUs > FORK_HALF_TRAVEL_US;
  if (forksAboveHalf && !forksWereAboveHalf) requestSound(TRACK_FORK_LIFT);
  forksWereAboveHalf = forksAboveHalf;

  int16_t trim = (int16_t)map(readChannel(3), PPM_VALID_MIN_US, PPM_VALID_MAX_US,
                              -BALLAST_TRIM_LIMIT_US, BALLAST_TRIM_LIMIT_US);
  int32_t ballastTarget = (int32_t)BALLAST_CENTER_US + trim + ballastAutoOffsetUs;
  const uint16_t minBallast = min(BALLAST_REAR_US, BALLAST_FRONT_US);
  const uint16_t maxBallast = max(BALLAST_REAR_US, BALLAST_FRONT_US);
  ballastTarget = constrain(ballastTarget, minBallast, maxBallast);
  if (ballastCurrentUs < ballastTarget) ballastCurrentUs = min((uint16_t)(ballastCurrentUs + BALLAST_RAMP_STEP_US), (uint16_t)ballastTarget);
  else if (ballastCurrentUs > ballastTarget) ballastCurrentUs = max((uint16_t)(ballastCurrentUs - BALLAST_RAMP_STEP_US), (uint16_t)ballastTarget);
  ballastServo.writeMicroseconds(ballastCurrentUs);
}

void updateSensors() {
  const uint32_t now = millis();
  if ((uint32_t)(now - sensorPingAtMs) < SENSOR_PING_INTERVAL_MS) return;
  sensorPingAtMs = now;

  NewPing* sensors[SENSOR_COUNT] = {&sonarFront, &sonarRear, &sonarLeft, &sonarRight};
  const unsigned long cm = sensors[nextSensor]->ping_cm(); // requested blocking one-pin NewPing ping
  sonarMm[nextSensor] = (cm == 0) ? SENSOR_NO_ECHO_MM : (uint16_t)(cm * 10UL);
  nextSensor = (uint8_t)((nextSensor + 1) % SENSOR_COUNT);

  int8_t directionVotes = 0;
  uint8_t liftCount = 0;
  const uint16_t frontMm = sonarMm[0];
  const uint16_t rearMm = sonarMm[1];
  const uint16_t leftMm = sonarMm[2];
  const uint16_t rightMm = sonarMm[3];
  const bool forksDown = radioValid && readChannel(5) < FORKS_DOWN_THRESHOLD_US;
  if (forksDown && frontMm > LIFT_MM) { ++directionVotes; ++liftCount; }
  if (rearMm > LIFT_MM) { --directionVotes; ++liftCount; }
  if (leftMm > LIFT_MM) { --directionVotes; ++liftCount; }
  if (rightMm > LIFT_MM) { ++directionVotes; ++liftCount; }

  if (liftCount > 0) {
    requestSound(TRACK_TIP_ALARM);
    if (directionVotes > 0) ballastAutoOffsetUs = BALLAST_AUTO_OFFSET_US;
    else if (directionVotes < 0) ballastAutoOffsetUs = -(int16_t)BALLAST_AUTO_OFFSET_US;
    else ballastAutoOffsetUs = 0; // opposing lifting detections cancel
  } else {
    ballastAutoOffsetUs = 0;
  }
}

void updateBattery() {
  const uint32_t now = millis();
  if ((uint32_t)(now - batterySampleAtMs) < BATTERY_SAMPLE_INTERVAL_MS) return;
  batterySampleAtMs = now;
  batteryAdcSamples[0][batterySampleIndex] = analogRead(PIN_VBAT_DRIVE);
  batteryAdcSamples[1][batterySampleIndex] = analogRead(PIN_VBAT_SERVO);
  batterySampleIndex = (uint8_t)((batterySampleIndex + 1) % BATTERY_AVERAGE_COUNT);
  if (batterySamplesFilled < BATTERY_AVERAGE_COUNT) ++batterySamplesFilled;

  for (uint8_t battery = 0; battery < BATTERY_COUNT; ++battery) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < batterySamplesFilled; ++i) sum += batteryAdcSamples[battery][i];
    const float average = (float)sum / batterySamplesFilled;
    batteryVolts[battery] = average * ADC_REFERENCE_VOLTS / ADC_MAX_COUNT * BATTERY_DIVIDER_MULTIPLIER;
  }

  const bool low = (batteryVolts[0] > 0.0f && batteryVolts[0] < VBAT_WARN) ||
                   (batteryVolts[1] > 0.0f && batteryVolts[1] < VBAT_WARN);
  if (low && (lowBatterySoundAtMs == 0 || (uint32_t)(now - lowBatterySoundAtMs) >= LOW_BATTERY_REPEAT_MS)) {
    requestSound(TRACK_LOW_BATTERY);
    lowBatterySoundAtMs = now;
  }
}

void updateSound() {
  if (pendingSoundTrack == 0) return;
  const uint32_t now = millis();
  const uint16_t track = pendingSoundTrack;
  if (lastSoundAt[track] != 0 && (uint32_t)(now - lastSoundAt[track]) < SOUND_COOLDOWN_MS) return;
  dfPlayer.playMp3Folder(track);
  lastSoundAt[track] = now;
  pendingSoundTrack = 0;
}

void setup() {
  pinMode(PIN_PPM, INPUT);
  pinMode(PIN_MOTOR_L_IN1, OUTPUT);
  pinMode(PIN_MOTOR_L_IN2, OUTPUT);
  pinMode(PIN_MOTOR_R_IN1, OUTPUT);
  pinMode(PIN_MOTOR_R_IN2, OUTPUT);
  pinMode(PIN_DRIVER_SLEEP, OUTPUT);
  digitalWrite(PIN_DRIVER_SLEEP, LOW);
  analogWrite(PIN_MOTOR_L_IN1, 0);
  analogWrite(PIN_MOTOR_L_IN2, 0);
  analogWrite(PIN_MOTOR_R_IN1, 0);
  analogWrite(PIN_MOTOR_R_IN2, 0);

  sawEsc.attach(PIN_ESC);
  sawArmServo.attach(PIN_ARM_SERVO);
  forkServo.attach(PIN_FORK_SERVO);
  ballastServo.attach(PIN_BALLAST_SERVO);
  sawEsc.writeMicroseconds(ESC_STOP_US);
  sawArmServo.writeMicroseconds(ARM_IN_US);
  forkServo.writeMicroseconds(FORK_DOWN_US);
  ballastServo.writeMicroseconds(BALLAST_CENTER_US);

  bootStartedMs = millis();
  driveRampAtMs = bootStartedMs;
  servoRampAtMs = bootStartedMs;
  sensorPingAtMs = bootStartedMs;
  batterySampleAtMs = bootStartedMs;
  ppmLastEdgeUs = micros();
  attachInterrupt(digitalPinToInterrupt(PIN_PPM), ppmEdgeISR, RISING);

  Serial.begin(9600); // Hardware UART TX D1 to DFPlayer RX; D0 remains unused.
  const bool playerReady = dfPlayer.begin(Serial, false, true); // no ACK because DFPlayer TX is not connected
  if (playerReady) {
    dfPlayer.volume(DFPLAYER_VOLUME);
    requestSound(TRACK_STARTUP);
  }
}

void loop() {
  readRadio();
  updateDrive();
  updateSaw();
  updateSensors();
  updateBattery();
  updateServos();
  updateSound();
}
