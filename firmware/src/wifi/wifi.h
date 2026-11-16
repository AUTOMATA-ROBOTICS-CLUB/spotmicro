#pragma once
// UDP controller input for the calibration sketch.
// Receives 4-byte big-endian evdev key codes from the Linux client
// (src/wifi/main.cpp) and runs the same commands as the serial console by
// calling handle().
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
  if (selected < 0)
    j = delta > 0 ? 0 : SERVO_COUNT - 1;
  else
    j = (selected + delta + SERVO_COUNT) % SERVO_COUNT;
  handle("j " + String(j));
}

// DualSense evdev codes (EV_KEY). The d-pad and sticks are EV_ABS, so they are
// not sent.
static void onKey(int32_t code) {
  switch (code) {
  // Cross -> stand
  case 16:
    handle("h");
    break;
  // Circle -> rest
  case 17:
    handle("z");
    break;
  // Triangle -> select all joints
  case 19:
    handle("a");
    break;
  // Square -> outputs off
  case 18:
    handle("o");
    break;
  // L1 -> previous joint
  case 310:
    stepJoint(-1);
    break;
  // R1 -> next joint
  case 311:
    stepJoint(+1);
    break;
  // L2 -> nudge centre -1
  case 312:
    handle("-");
    break;
  // R2 -> nudge centre +1
  case 313:
    handle("+");
    break;
  // Create -> flip direction
  case 314:
    handle("d");
    break;
  // Options -> save to flash
  case 315:
    handle("w");
    break;
  default:
    Serial.printf("unmapped key %ld\n", (long)code);
    break;
  }
}

static void udpBegin() {
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_SSID, AP_PASS)) {
    Serial.println("WiFi AP failed to start");
    return;
  }
  udp.begin(UDP_PORT);
  Serial.printf("WiFi AP '%s' up, UDP on %s:%u\n", AP_SSID,
                WiFi.softAPIP().toString().c_str(), UDP_PORT);
}

static void udpLoop() {
  while (int size = udp.parsePacket()) {
    // Ignore anything that is not one key code (scans and other noise).
    if (size != 4) {
      udp.clear();
      continue;
    }
    uint8_t b[4];
    if (udp.read(b, 4) != 4)
      continue;
    int32_t code = ((int32_t)b[0] << 24) | ((int32_t)b[1] << 16) |
                   ((int32_t)b[2] << 8) | b[3];
    onKey(code);
  }
}
