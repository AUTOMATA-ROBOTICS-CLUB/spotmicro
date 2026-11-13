// Step 2: drive servos by hand from the serial monitor (115200 baud, send lines ending in newline).
// Doubles as a calibration tool that needs no WiFi: find each servo's centre/limits and write them down.
//
// Commands:
//   c <0-15>   select a channel (12-15 spare)  a        select all 12 robot channels
//   p <pwm>    set pulse (125-600 ticks)       + / -    nudge by 1      ++ / --   nudge by 10
//   s          sweep the selection slowly between the limits (any command stops it)
//   o          outputs off (servos go limp)    ?        show this help and the current state
//
// Nothing moves until you send a command, so the robot doesn't jump at power-up.

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

#include "robot_config.h"

Adafruit_PWMServoDriver pca(PCA9685_ADDR, Wire);

int selected = 0;      // channel, or -1 for all
uint16_t pwm = PWM_CENTER;
bool outputsOn = false;
bool sweeping = false;
int sweepDir = 1;
uint32_t lastSweepMs = 0;

void apply() {
    if (selected < 0) {
        for (int ch = 0; ch < SERVO_COUNT; ch++) pca.setPWM(ch, 0, pwm);
    } else {
        pca.setPWM(selected, 0, pwm);
    }
    outputsOn = true;
}

void outputsOff() {
    for (int ch = 0; ch < 16; ch++) pca.setPWM(ch, 0, 4096);  // full-off bit: no pulse
    outputsOn = false;
}

const char *channelName(int ch) {
    static char spare[24];
    if (ch < 0) return "all servos";
    if (ch < SERVO_COUNT) return SERVO_NAMES[ch];
    snprintf(spare, sizeof(spare), "ch %d (spare)", ch);
    return spare;
}

void printState() {
    Serial.printf("[%s] pwm %u (%.0f us)%s%s\n", channelName(selected), pwm,
                  ticksToMicros(pwm), outputsOn ? "" : "  (outputs off)", sweeping ? "  sweeping" : "");
}

void help() {
    Serial.println("\nc <0-15> select | a all | p <pwm> set | + - ++ -- nudge | s sweep | o off | ? help");
    for (int i = 0; i < SERVO_COUNT; i++) Serial.printf("  %2d %s\n", i, SERVO_NAMES[i]);
    Serial.println("  12-15 spare");
    printState();
}

void setPwm(long value) {
    pwm = constrain(value, PWM_MIN, PWM_MAX);
    if (value != pwm) Serial.printf("clamped to %u (safe range %u-%u)\n", pwm, PWM_MIN, PWM_MAX);
    apply();
}

void handle(String line) {
    line.trim();
    if (line.isEmpty()) return;
    sweeping = false;
    if (line == "?") return help();
    if (line == "a") {
        selected = -1;
    } else if (line == "o") {
        outputsOff();
    } else if (line == "s") {
        sweeping = true;
    } else if (line == "+" || line == "-" || line == "++" || line == "--") {
        int step = line.length() == 2 ? 10 : 1;
        setPwm(pwm + (line[0] == '+' ? step : -step));
    } else if (line[0] == 'c') {
        long ch = line.substring(1).toInt();
        if (ch < 0 || ch >= PCA_CHANNELS) return (void)Serial.println("channel must be 0-15");
        selected = ch;
    } else if (line[0] == 'p') {
        setPwm(line.substring(1).toInt());
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
    Serial.println("\nservo_tool ready - servos are off until you send a command");
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
    if (sweeping && millis() - lastSweepMs >= 20) {  // ~1 tick per 20 ms: a full sweep takes ~10 s
        lastSweepMs = millis();
        if (pwm >= PWM_MAX) sweepDir = -1;
        if (pwm <= PWM_MIN) sweepDir = 1;
        pwm += sweepDir;
        apply();
    }
}
