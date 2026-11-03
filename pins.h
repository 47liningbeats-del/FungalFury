#ifndef ROBOT_PINS_H
#define ROBOT_PINS_H
#include <Arduino.h>

#ifndef DEBUG
#define DEBUG 0  // Change to 1 for 115200-baud diagnostics on D1/TX; D0/RX stays unused.
#endif

// Wiring: do not feed the FS-R6B from the 2S battery; its BAT + is on the BEC 5 V rail.
constexpr uint8_t PPM_PIN = 8;          // Encoder PPM OUT, ATmega328P PB0 / PCINT0
constexpr uint8_t DEBUG_TX_PIN = 1;
constexpr uint8_t UNUSED_RX_PIN = 0;
constexpr uint8_t DRIVE_COUNT = 4;
constexpr uint8_t DRIVE_FL = 0, DRIVE_FR = 1, DRIVE_BL = 2, DRIVE_BR = 3;
constexpr uint8_t DRIVE_PINS[DRIVE_COUNT] = {5, 6, 3, 7};
constexpr int8_t MOTOR_DIR[DRIVE_COUNT] = {+1, +1, +1, +1};
constexpr uint8_t SAW_PIN = 9, ARM_PIN = 10, FORK_PIN = 11, BALLAST_PIN = 12;
constexpr uint8_t SONAR_COUNT = 4;
constexpr uint8_t SONAR_REAR = 0, SONAR_FRONT = 1, SONAR_LEFT = 2, SONAR_RIGHT = 3;
constexpr uint8_t SONAR_PINS[SONAR_COUNT] = {2, 4, 13, A1}; // TRIG direct; ECHO via 1k
constexpr uint8_t BATTERY_PIN = A0;
constexpr uint8_t MPU_ADDR = 0x68, PCF_ADDR = 0x20;
constexpr uint8_t MPU_PWR_MGMT_1 = 0x6B, MPU_GYRO_CONFIG = 0x1B;
constexpr uint8_t MPU_ACCEL_CONFIG = 0x1C, MPU_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t AUDIO_SAW = 0, AUDIO_FORK = 1, AUDIO_HIT = 2;
constexpr uint8_t AUDIO_TIP = 3, AUDIO_START = 4;

constexpr uint8_t CHANNEL_COUNT = 6;
constexpr uint8_t CH_STRAFE = 0, CH_FORWARD = 1, CH_SAW = 2;
constexpr uint8_t CH_ROTATE = 3, CH_ARM = 4, CH_FORK = 5;
constexpr uint16_t PPM_GAP_US = 3000, PPM_MIN_US = 900, PPM_MAX_US = 2100;
constexpr uint16_t STICK_CENTER_US = 1500, STICK_DEADBAND_US = 30;
constexpr float STICK_SPAN_US = 500.0f, DRIVE_MIX_SPAN_US = 500.0f;
constexpr uint16_t SERVO_MIN_US = 1000, SERVO_MAX_US = 2000;
constexpr uint16_t DRIVE_NEUTRAL_US = 1500, SAW_OFF_US = 1000;
constexpr uint16_t DRIVE_SLEW_US = 40, FORK_SLEW_US = 2;
constexpr uint16_t ARM_IN_US = 1000, ARM_OUT_US = 2000; // Calibrate physical travel on bench.
constexpr uint16_t ARM_SWITCH_IN_MAX_US = 1450, ARM_SWITCH_OUT_MIN_US = 1550;
constexpr uint16_t FORK_DOWN_US = 1000, FORK_UP_US = 1139; // ~25 degrees above DOWN.
constexpr uint16_t FORK_SELF_RIGHT_US = 2000;
constexpr uint16_t FORK_SOUND_US = FORK_DOWN_US + (FORK_SELF_RIGHT_US - FORK_DOWN_US) / 10;
constexpr uint16_t BALLAST_REAR_US = 1000, BALLAST_CENTER_US = 1500;
constexpr uint16_t BALLAST_FRONT_US = 2000;

// FS-R6B failsafe changes CH3 alone. Configure its lost-link CH3 BELOW the
// normal throttle minimum (~1100 us), or this safety test cannot detect loss.
constexpr uint16_t CH3_FAILSAFE_US = 1050, CH3_NORMAL_MIN_US = 1100;
constexpr uint16_t CH3_REARM_MAX_US = 1150;
constexpr uint16_t CH3_REARM_MARGIN_US = 50;
constexpr uint32_t FAILSAFE_MS = 100, REARM_MS = 500;
constexpr uint32_t ESC_ARM_MS = 3000, SAW_ARM_WAIT_MS = 250, ARM_IN_WAIT_MS = 150;
constexpr uint16_t SAW_LOW_BATTERY_MAX_US = 1500;
constexpr float BATTERY_WARNING_V = 7.0f, BATTERY_SAW_CAP_V = 6.6f;
constexpr float BATTERY_VOLTS_PER_COUNT = (5.0f / 1023.0f) * 2.0f;
constexpr uint8_t BATTERY_SAMPLES = 8;
constexpr uint32_t BATTERY_PERIOD_MS = 100, DEBUG_PERIOD_MS = 200;

constexpr uint8_t MPU_GYRO_500DPS = 0x08, MPU_ACCEL_8G = 0x10;
constexpr uint8_t MPU_WAKE_VALUE = 0x00, MPU_BURST_BYTES = 14;
constexpr float MPU_GYRO_LSB_PER_DPS = 65.5f, MPU_ACCEL_LSB_PER_G = 4096.0f;
constexpr float FILTER_GYRO = 0.98f, FILTER_ACCEL = 0.02f;
constexpr float RAD_TO_DEG_F = 57.29578f, MAX_IMU_DT_S = 0.05f;
constexpr uint16_t GYRO_CAL_SAMPLES = 200;
constexpr uint32_t I2C_TIMEOUT_US = 3000;
constexpr float HIT_G = 3.0f, HIT_SAW_EXTRA_G = 1.5f, TIP_DEG = 15.0f;
constexpr uint32_t HIT_LOCKOUT_MS = 400, TIP_SOUND_LOCKOUT_MS = 2000;

constexpr uint8_t SONAR_MAX_CM = 50, SONAR_HISTORY = 3;
constexpr uint8_t BALLAST_STATE_CENTER = 0, BALLAST_STATE_REAR = 1;
constexpr uint8_t BALLAST_STATE_FRONT = 2, BALLAST_STATE_HOLD = 3;
constexpr uint16_t US_NO_ECHO_MM = 500, US_LEVEL_MM = 55, US_LIFT_MM = 80;
constexpr uint8_t SONAR_ROUNDTRIP_US_PER_CM = 58;
constexpr uint32_t PING_PERIOD_MS = 30, SONAR_TIMEOUT_MS = 9;
constexpr uint32_t LEVEL_SETTLE_MS = 300, SOUND_PULSE_MS = 120;
constexpr uint32_t CONTROL_STEP_US = 5000;

#endif
