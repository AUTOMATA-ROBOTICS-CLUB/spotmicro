// Step 3: calibration table. Each joint gets a PCA9685 channel, a centre pulse (the pulse that puts
// it in the reference pose), a direction and ticks per degree, saved in flash. From here on joints
// are driven by angle instead of raw pulses.
//
// Reference pose (every joint at its centre) = standing: hips level with the leg straight down,
// upper legs 58 deg back, knees bent 107 deg. Angles: hip 0 = straight down, femur 0 = vertical
// (negative = back), knee 0 = straight (positive = bent).
//
// Commands:
//   j <0-11>   select a joint                  a        select all joints
//   h          selection to standing           z        selection to rest (body lowered)
//   g <deg>    selection to a joint angle (h, z and g move slowly; the first one after power-up
//              switches the servos on one by one)
//   + / -      nudge the centre by 1 tick      ++ / --  by 10
//   k <deg>    mark "this joint is really at <deg> now". Two marks at least 30 deg apart set the
//              direction, ticks per degree and centre from the pulses: g near an angle you can
//              check (straight, square), + / - until it's exact, k; then the same at a second angle
//   d          flip direction                  t <x>    set ticks per degree
//   m <0-15>   move the joint to another PCA9685 channel
//   l          list the table and the angles each joint can reach
//   w          write to flash   r  reload from flash   x  reset to the defaults in robot_config.h
//   o          outputs off (servos go limp)    ?        help
//
// Nothing moves until you send a command, so the robot doesn't jump at power-up.

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

#include "robot_config.h"
#include "servo_cal.h"

Adafruit_PWMServoDriver pca(PCA9685_ADDR, Wire);

ServoCal cal[SERVO_COUNT];
float angle[SERVO_COUNT];   // joint angle being sent now (degrees)
float target[SERVO_COUNT];  // where the joint is heading
bool on[SERVO_COUNT] = {};
uint16_t sentPwm[SERVO_COUNT];  // pulse on the wire now
struct Mark {
    bool set;
    float deg;
    uint16_t pwm;
} mark[SERVO_COUNT] = {};       // last 'k' per joint
uint32_t lastMoveMs = 0;
int selected = 0;           // joint, or -1 for all
bool dirty = false;         // table changed since the last write/load

void drive(int j) {
    float ticks = angleToTicks(cal[j], j, angle[j]);
    uint16_t pwm = constrain(lroundf(ticks), PWM_MIN, PWM_MAX);
    if (pwm != lroundf(ticks) && angle[j] == target[j]) Serial.printf("%s clamped to %u (safe range %u-%u)\n", SERVO_NAMES[j], pwm, PWM_MIN, PWM_MAX);
    pca.setPWM(cal[j].channel, 0, pwm);
    sentPwm[j] = pwm;
    on[j] = true;
}

void outputsOff() {
    for (int ch = 0; ch < PCA_CHANNELS; ch++) pca.setPWM(ch, 0, 4096);  // full-off bit: no pulse
    for (int j = 0; j < SERVO_COUNT; j++) on[j] = false;
}

// Moves a joint slowly to deg. A joint that is off has no known position, so it is switched on
// straight at deg; one at a time, so 12 servos starting together don't trip the power supply.
void moveTo(int j, float deg) {
    target[j] = deg;
    if (on[j]) return;
    angle[j] = deg;
    drive(j);
    delay(40);
}

// Runs fn on the selected joint, or on every joint when all are selected.
template <typename Fn>
void forSelection(Fn fn) {
    for (int j = 0; j < SERVO_COUNT; j++)
        if (selected < 0 || selected == j) fn(j);
}

void printJoint(int j) {
    float a1 = ticksToAngle(cal[j], j, PWM_MIN), a2 = ticksToAngle(cal[j], j, PWM_MAX);
    Serial.printf("  %2d %-9s ch %2d  centre %3u  dir %+d  %.2f t/deg  reach %4.0f..%-4.0f  %s\n", j,
                  SERVO_NAMES[j], cal[j].channel, cal[j].center, cal[j].dir, cal[j].ticksPerDeg, min(a1, a2),
                  max(a1, a2), on[j] ? String(target[j], 1).c_str() : "off");
}

void list() {
    Serial.println("  joint          channel centre dir  ticks/deg  reachable deg  angle");
    for (int j = 0; j < SERVO_COUNT; j++) printJoint(j);
    if (dirty) Serial.println("  (changed - 'w' to save)");
}

void printState() {
    if (selected < 0) return list();
    printJoint(selected);
}

void help() {
    Serial.println("\nj <0-11> select | a all | h stand | z rest | g <deg> go | + - ++ -- nudge centre");
    Serial.println("k <deg> mark real angle (2 marks = auto dir, ticks/deg, centre) | d flip dir | t <x> ticks/deg");
    Serial.println("m <0-15> channel | l list | w save | r reload | x defaults | o off");
    Serial.println("angles: hip 0 = straight down, femur 0 = vertical (- = back), knee 0 = straight");
    list();
}

// 'k': the selected joint is at deg with the pulse being sent now. With an earlier mark far enough
// away, the two (angle, pulse) points give the line pulse = centre + dir * ticksPerDeg * (angle - ref).
void markAngle(float deg) {
    int j = selected;
    if (!on[j] || angle[j] != target[j]) return (void)Serial.println("move the joint there first (g <deg>, + / -)");
    Mark &m = mark[j];
    if (m.set && fabsf(deg - m.deg) >= 30.0f) {
        float slope = (float(sentPwm[j]) - m.pwm) / (deg - m.deg);  // ticks per degree, signed
        if (fabsf(slope) < 1.0f || fabsf(slope) > 5.0f) {
            Serial.printf("%.2f ticks/deg is not plausible for an MG996R - check both angles, mark again\n", fabsf(slope));
            m = {true, deg, sentPwm[j]};
            return;
        }
        ServoCal &c = cal[j];
        c.dir = slope > 0 ? 1 : -1;
        c.ticksPerDeg = fabsf(slope);
        c.center = constrain(lroundf(sentPwm[j] + slope * (refAngle(j) - deg)), PWM_MIN, PWM_MAX);
        angle[j] = target[j] = deg;  // the new line passes through the current pulse: no jump
        dirty = true;
        Serial.printf("%s: from %.0f and %.0f deg -> dir %+d, %.2f ticks/deg, centre %u ('w' to save)\n",
                      SERVO_NAMES[j], m.deg, deg, c.dir, c.ticksPerDeg, c.center);
    } else {
        Serial.printf("marked %.0f deg at pulse %u - now mark a second angle at least 30 deg away\n", deg, sentPwm[j]);
    }
    m = {true, deg, sentPwm[j]};
}

bool needOne(const char *what) {
    if (selected >= 0) return true;
    Serial.printf("select one joint first (j <0-11>) to %s\n", what);
    return false;
}

void handle(String line) {
    line.trim();
    if (line.isEmpty()) return;
    char cmd = line[0];
    String arg = line.substring(1);
    arg.trim();

    if (line == "?") return help();
    if (line == "l") return list();
    if (cmd == 'j') {
        long j = arg.toInt();
        if (arg.isEmpty() || j < 0 || j >= SERVO_COUNT) return (void)Serial.println("joint must be 0-11");
        selected = j;
    } else if (line == "a") {
        selected = -1;
    } else if (line == "h") {
        forSelection([](int j) { moveTo(j, refAngle(j)); });
    } else if (line == "z") {
        forSelection([](int j) { moveTo(j, REST_ANGLE[j % 3]); });
    } else if (cmd == 'g') {
        if (arg.isEmpty()) return (void)Serial.println("usage: g <degrees>");
        float deg = arg.toFloat();
        forSelection([deg](int j) { moveTo(j, deg); });
    } else if (line == "+" || line == "-" || line == "++" || line == "--") {
        if (!needOne("nudge its centre")) return;
        int step = line.length() == 2 ? 10 : 1;
        ServoCal &c = cal[selected];
        c.center = constrain(c.center + (cmd == '+' ? step : -step), PWM_MIN, PWM_MAX);
        if (!on[selected]) angle[selected] = target[selected] = refAngle(selected);
        drive(selected);  // the joint moves with its centre, so you can line it up by eye
        dirty = true;
    } else if (cmd == 'k') {
        if (!needOne("mark its angle")) return;
        if (arg.isEmpty()) return (void)Serial.println("usage: k <degrees>");
        markAngle(arg.toFloat());
    } else if (line == "d") {
        if (!needOne("flip its direction")) return;
        cal[selected].dir = -cal[selected].dir;
        if (on[selected]) drive(selected);
        dirty = true;
    } else if (cmd == 't') {
        float tpd = arg.toFloat();
        if (tpd < 1.0f || tpd > 5.0f) return (void)Serial.println("ticks per degree must be 1-5 (MG996R ~2.2)");
        forSelection([tpd](int j) { cal[j].ticksPerDeg = tpd; if (on[j]) drive(j); });
        dirty = true;
    } else if (cmd == 'm') {
        if (!needOne("move it to another channel")) return;
        long ch = arg.toInt();
        if (arg.isEmpty() || ch < 0 || ch >= PCA_CHANNELS) return (void)Serial.println("channel must be 0-15");
        for (int j = 0; j < SERVO_COUNT; j++)
            if (j != selected && cal[j].channel == ch) Serial.printf("warning: %s is also on ch %ld\n", SERVO_NAMES[j], ch);
        pca.setPWM(cal[selected].channel, 0, 4096);
        cal[selected].channel = ch;
        if (on[selected]) drive(selected);
        dirty = true;
    } else if (line == "w") {
        saveCal(cal);
        dirty = false;
        Serial.println("saved to flash");
        return;
    } else if (line == "r" || line == "x") {
        outputsOff();
        if (line == "x") memcpy(cal, DEFAULT_CAL, sizeof(DEFAULT_CAL));
        else Serial.println(loadCal(cal) ? "loaded from flash" : "nothing saved - using defaults");
        dirty = line == "x";
        return list();
    } else if (line == "o") {
        outputsOff();
    } else {
        Serial.println("unknown command (? for help)");
        return;
    }
    printState();
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!pca.begin()) Serial.println("PCA9685 not responding - run the i2c_scan step first");
    pca.setOscillatorFrequency(PCA9685_OSC_HZ);
    pca.setPWMFreq(SERVO_FREQ_HZ);
    outputsOff();
    bool saved = loadCal(cal);
    for (int j = 0; j < SERVO_COUNT; j++) angle[j] = target[j] = refAngle(j);
    Serial.printf("\ncalibrate ready (%s) - servos are off until you send a command\n",
                  saved ? "table loaded from flash" : "defaults from robot_config.h");
    help();
}

void loop() {
    static String line;
    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\n' || ch == '\r') {
            handle(line);
            line = "";
        } else {
            line += ch;
        }
    }
    if (millis() - lastMoveMs >= 20) {
        lastMoveMs = millis();
        const float step = MOVE_DEG_PER_S * 0.02f;
        for (int j = 0; j < SERVO_COUNT; j++) {
            if (!on[j] || angle[j] == target[j]) continue;
            float diff = target[j] - angle[j];
            angle[j] = fabsf(diff) <= step ? target[j] : angle[j] + copysignf(step, diff);
            drive(j);
        }
    }
}
