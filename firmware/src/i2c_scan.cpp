// Step 1: check the I2C wiring. Lists every device on the bus and names the ones this robot uses.
// Expect at least the PCA9685 at 0x40. Open the serial monitor at 115200 baud.

#include <Arduino.h>
#include <Wire.h>

#include "robot_config.h"

struct KnownDevice {
    uint8_t addr;
    const char *name;
};

const KnownDevice KNOWN[] = {
    {0x40, "PCA9685 servo driver"},     {0x70, "PCA9685 all-call address"},
    {0x68, "MPU6050 IMU (GY-87)"},      {0x1E, "HMC5883L magnetometer (GY-87)"},
    {0x0D, "QMC5883L magnetometer"},    {0x77, "BMP180 barometer (GY-87)"},
    {0x28, "BNO055 IMU"},               {0x29, "BNO055 IMU (alt)"},
    {0x3C, "SSD1306 OLED"},             {0x73, "PAJ7620U2 gesture sensor"},
};

const char *deviceName(uint8_t addr) {
    for (const auto &d : KNOWN)
        if (d.addr == addr) return d.name;
    return "unknown";
}

void scan() {
    Serial.printf("\nScanning I2C (SDA=%d, SCL=%d)...\n", SDA_PIN, SCL_PIN);
    int found = 0;
    bool pca = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  0x%02X  %s\n", addr, deviceName(addr));
            found++;
            pca |= addr == PCA9685_ADDR;
        }
    }
    if (found == 0) {
        Serial.println("  nothing found: check SDA/SCL (not swapped?), 3V3 and GND to the PCA9685");
    } else if (!pca) {
        Serial.println("  PCA9685 not at 0x40: check its address pads (A0-A5) and wiring");
    } else {
        Serial.println("  PCA9685 OK - go on to step 2 (servo_tool)");
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);  // give the serial monitor time to connect
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
}

void loop() {
    scan();
    delay(3000);
}
