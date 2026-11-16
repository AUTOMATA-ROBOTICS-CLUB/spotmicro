#pragma once
// UDP controller input for the calibration sketch.
// Receives the 4-byte big-endian evdev key codes sent by the Linux client and runs the same
// commands as the serial console by calling handle().
//
// Include this AFTER handle() is defined and BEFORE setup() in the sketch.

#include <WiFi.h>
#include <WiFiUdp.h>

static const char *AP_SSID = "RobotController";
static const char *AP_PASS = "Eojd39tf!";
static const uint16_t UDP_PORT = 4210;

static WiFiUDP udp;

// Select the previous/next joint (wraps around).
static void stepJoint(int delta) {
    int j;
    if (selected < 0) j = delta > 0 ? 0 : SERVO_COUNT - 1;
    else j = (selected + delta + SERVO_COUNT) % SERVO_COUNT;
    handle("j " + String(j));
}

// DualSense evdev codes (EV_KEY). The d-pad and sticks are EV_ABS, so they are not sent.
static void onKey(int32_t code) {
    switch (code) {
        case 16: handle("h"); break;        // Cross     -> stand
        case 17: handle("z"); break;        // Circle    -> rest
        case 19: handle("a"); break;        // Triangle  -> select all joints
        case 18: handle("o"); break;        // Square    -> outputs off
        case 310: stepJoint(-1); break;      // L1        -> previous joint
        case 311: stepJoint(+1); break;      // R1        -> next joint
        case 312: handle("-"); break;        // L2        -> nudge centre -1
        case 313: handle("+"); break;        // R2        -> nudge centre +1
        case 314: handle("d"); break;        // Create    -> flip direction
        case 315: handle("w"); break;        // Options   -> save to flash
        default: Serial.printf("unmapped key %ld\n", (long)code); break;
    }
}

void udpBegin() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    udp.begin(UDP_PORT);
    Serial.printf("WiFi AP '%s' up, UDP on %s:%u\n", AP_SSID, WiFi.softAPIP().toString().c_str(), UDP_PORT);
}

void udpLoop() {
    while (int size = udp.parsePacket()) {
        if (size != 4) {  // ignore scans and other noise
            udp.flush();
            continue;
        }
        uint8_t b[4];
        if (udp.read(b, 4) != 4) continue;
        int32_t code = ((int32_t)b[0] << 24) | ((int32_t)b[1] << 16) | ((int32_t)b[2] << 8) | b[3];
        onKey(code);
    }
}
