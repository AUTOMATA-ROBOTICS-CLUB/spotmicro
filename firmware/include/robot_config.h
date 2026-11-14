#pragma once

// ---- Wiring (ESP32-S3 WROOM camera board) -----------------------------------
#ifndef SDA_PIN
#define SDA_PIN 21
#endif
#ifndef SCL_PIN
#define SCL_PIN 47
#endif

// ---- PCA9685 servo driver ---------------------------------------------------
// Default address (A0-A5 pads not bridged).
constexpr uint8_t PCA9685_ADDR = 0x40;
// Typical real oscillator of cheap boards.
constexpr uint32_t PCA9685_OSC_HZ = 27000000;
// 50 Hz = 20 ms frame, standard for MG996R.
constexpr float SERVO_FREQ_HZ = 50.0f;
// Robot servos (joints), normally on channels 0-11.
constexpr int SERVO_COUNT = 12;
// 12-15 are spare (testing, or re-mapping a bad channel).
constexpr int PCA_CHANNELS = 16;

// Pulse values are in PCA9685 ticks: 4096 per 20 ms frame, so 1 tick = 4.88 us.
// Our MG996Rs run quietly down to PWM_MIN.
constexpr uint16_t PWM_MIN = 100;    // ~0.49 ms
constexpr uint16_t PWM_MAX = 600;    // ~2.93 ms
constexpr uint16_t PWM_CENTER = 306; // ~1.49 ms, nominal servo centre

inline float ticksToMicros(uint16_t ticks) {
  return ticks * (1000000.0f / SERVO_FREQ_HZ) / 4096.0f;
}

// Joint order (index 0-11): each leg is shoulder (hip), upper leg (femur),
// lower leg (knee). Joint i is on PCA9685 channel i; to use a spare channel,
// change it in DEFAULT_CAL below.
constexpr const char *SERVO_NAMES[SERVO_COUNT] = {
    "FL hip", "FL femur", "FL knee", //
    "FR hip", "FR femur", "FR knee", //
    "RL hip", "RL femur", "RL knee", //
    "RR hip", "RR femur", "RR knee",
};

// ---- Servo calibration (step 3) ---------------------------------------------
// Reference pose = standing pose: the leg IK at the default body height
// (148.6 mm hip -> foot centre) gives hips level with the leg straight down,
// upper legs 58 deg back, knees bent 107 deg. A joint's centre pulse puts it in
// this pose. Joint angles use the IK convention: hip 0 = leg straight down,
// femur 0 = vertical (negative = back), knee 0 = straight (positive = bent).
constexpr float REF_ANGLE[3] = {0.0f, -58.0f, 107.0f}; // hip, femur, knee (deg)
// Rest pose: body lowered to the minimum height (99 mm hip -> foot centre, as
// Leika's rest state).
constexpr float REST_ANGLE[3] = {0.0f, -76.0f, 136.0f};
// Joint speed for pose changes, so the body doesn't drop.
constexpr float MOVE_DEG_PER_S = 60.0f;

struct ServoCal {
  uint8_t channel;   // PCA9685 output
  uint16_t center;   // pulse (ticks) at the reference (standing) pose
  int8_t dir;        // +1: a bigger angle needs a bigger pulse, -1: smaller
  float ticksPerDeg; // ~2.2 for MG996R at 50 Hz
};

// Standing pose measured by hand with servo_tool, then directions and scales
// checked on the robot with the calibrate step (2026-11-13). A table saved to
// flash overrides this one.
constexpr ServoCal DEFAULT_CAL[SERVO_COUNT] = {
    {0, 306, 1, 2.2f},  {1, 180, 1, 2.2f},   {2, 420, 1, 2.2f},   // FL
    {3, 470, -1, 2.2f}, {4, 435, -1, 2.2f},  {5, 163, -1, 2.2f},  // FR
    {6, 295, -1, 2.2f}, {7, 190, 1, 2.2f},   {8, 426, 1, 2.2f},   // RL
    {9, 310, 1, 2.2f},  {10, 490, -1, 2.2f}, {11, 179, -1, 2.2f}, // RR
};

// ---- Leg geometry (measured on the printed KDY0523 parts, metres) -----------
// Used from step 4 (leg IK) on. See hardware/README.md for how these were
// measured.
constexpr float COXA = 0.055f;          // hip axis -> leg plane, sideways
constexpr float COXA_OFFSET = 0.01075f; // hip axis -> femur axis, down
constexpr float FEMUR = 0.1083f;        // femur axis -> knee
constexpr float TIBIA = 0.1354f;        // knee -> ground contact, leg straight
constexpr float BODY_L = 0.1855f;       // front <-> rear femur axes
constexpr float BODY_W = 0.078f;        // left <-> right hip axes
