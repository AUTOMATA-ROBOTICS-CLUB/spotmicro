#pragma once

// Servo calibration table: saved in flash (Preferences) by the calibrate step, read by later steps.

#include <Preferences.h>

#include "robot_config.h"

constexpr uint32_t CAL_VERSION = 1;  // bump when ServoCal changes, so old flash data is ignored

// Loads the saved table into cal; falls back to DEFAULT_CAL. Returns true if flash had one.
inline bool loadCal(ServoCal cal[SERVO_COUNT]) {
    Preferences prefs;
    prefs.begin("servocal", true);
    bool ok = prefs.getUInt("version", 0) == CAL_VERSION &&
              prefs.getBytesLength("table") == sizeof(ServoCal) * SERVO_COUNT;
    if (ok) prefs.getBytes("table", cal, sizeof(ServoCal) * SERVO_COUNT);
    prefs.end();
    if (!ok) memcpy(cal, DEFAULT_CAL, sizeof(DEFAULT_CAL));
    return ok;
}

inline void saveCal(const ServoCal cal[SERVO_COUNT]) {
    Preferences prefs;
    prefs.begin("servocal", false);
    prefs.putBytes("table", cal, sizeof(ServoCal) * SERVO_COUNT);
    prefs.putUInt("version", CAL_VERSION);
    prefs.end();
}

inline float refAngle(int joint) { return REF_ANGLE[joint % 3]; }

// Joint angle (degrees, IK convention) -> pulse in ticks, before clamping to PWM_MIN..PWM_MAX.
inline float angleToTicks(const ServoCal &c, int joint, float deg) {
    return c.center + c.dir * c.ticksPerDeg * (deg - refAngle(joint));
}

inline float ticksToAngle(const ServoCal &c, int joint, float ticks) {
    return refAngle(joint) + (ticks - c.center) / (c.dir * c.ticksPerDeg);
}
