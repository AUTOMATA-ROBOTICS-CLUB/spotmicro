#pragma once

// ---- Wiring (ESP32-S3 WROOM camera board) ------------------------------------------------
#ifndef SDA_PIN
#define SDA_PIN 21
#endif
#ifndef SCL_PIN
#define SCL_PIN 47
#endif

// ---- PCA9685 servo driver ------------------------------------------------------------------
constexpr uint8_t PCA9685_ADDR = 0x40;            // default address (A0-A5 pads not bridged)
constexpr uint32_t PCA9685_OSC_HZ = 27000000;     // typical real oscillator of cheap boards
constexpr float SERVO_FREQ_HZ = 50.0f;            // 50 Hz = 20 ms frame, standard for MG996R
constexpr int SERVO_COUNT = 12;      // robot servos on channels 0-11
constexpr int PCA_CHANNELS = 16;     // 12-15 are spare (testing, or re-mapping a bad channel)

// Pulse values are in PCA9685 ticks: 4096 per 20 ms frame, so 1 tick = 4.88 us.
constexpr uint16_t PWM_MIN = 125;                  // ~0.61 ms, safe lower limit
constexpr uint16_t PWM_MAX = 600;                  // ~2.93 ms, safe upper limit
constexpr uint16_t PWM_CENTER = 306;               // ~1.49 ms, nominal servo centre

inline float ticksToMicros(uint16_t ticks) { return ticks * (1000000.0f / SERVO_FREQ_HZ) / 4096.0f; }

// Channel -> joint (PCA9685 outputs 0-11): each leg is shoulder (hip), upper leg (femur), lower leg (knee)
constexpr const char *SERVO_NAMES[SERVO_COUNT] = {
    "FL hip", "FL femur", "FL knee",  //
    "FR hip", "FR femur", "FR knee",  //
    "RL hip", "RL femur", "RL knee",  //
    "RR hip", "RR femur", "RR knee",
};

// ---- Leg geometry (measured on the printed KDY0523 parts, metres) ---------------------------
// Used from step 4 (leg IK) on. See hardware/README.md for how these were measured.
constexpr float COXA = 0.055f;         // hip axis -> leg plane, sideways
constexpr float COXA_OFFSET = 0.01075f;  // hip axis -> femur axis, down
constexpr float FEMUR = 0.1083f;       // femur axis -> knee
constexpr float TIBIA = 0.1354f;       // knee -> ground contact, leg straight
constexpr float BODY_L = 0.1855f;      // front <-> rear femur axes
constexpr float BODY_W = 0.078f;       // left <-> right hip axes
