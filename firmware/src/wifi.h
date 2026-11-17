#pragma once
// UDP controller input for the calibration sketch.
// Receives 4-byte big-endian evdev key codes from the Linux client
// (src/pc_controller.cpp) and runs the same commands as the serial console by
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

// Linux evdev key codes (EV_KEY). Keyboard letters run the calibrate command
// of the same letter; the DualSense sends its shoulder and menu buttons. The
// d-pad and sticks are EV_ABS, so they are not sent.
static void onKey(int32_t code) {
  switch (code) {
  // ---- Keyboard ----
  // H -> stand
  case 35:
    handle("h");
    break;
  // Z -> rest
  case 44:
    handle("z");
    break;
  // A -> select all joints
  case 30:
    handle("a");
    break;
  // Left -> previous joint
  case 105:
    stepJoint(-1);
    break;
  // Right -> next joint
  case 106:
    stepJoint(+1);
    break;
  // + (= key or keypad +) or Up -> nudge centre +1
  case 13:
  case 78:
  case 103:
    handle("+");
    break;
  // - (- key or keypad -) or Down -> nudge centre -1
  case 12:
  case 74:
  case 108:
    handle("-");
    break;
  // Page Up -> nudge centre +10
  case 104:
    handle("++");
    break;
  // Page Down -> nudge centre -10
  case 109:
    handle("--");
    break;
  // D -> flip direction
  case 32:
    handle("d");
    break;
  // L -> list the table
  case 38:
    handle("l");
    break;
  // W -> write to flash
  case 17:
    handle("w");
    break;
  // R -> reload from flash
  case 19:
    handle("r");
    break;
  // X -> reset to DEFAULT_CAL in robot_config.h
  case 45:
    handle("x");
    break;
  // O -> outputs off
  case 24:
    handle("o");
    break;
  // / (the ? key) -> help
  case 53:
    handle("?");
    break;

  // ---- DualSense ----
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
