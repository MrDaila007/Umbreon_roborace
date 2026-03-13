/**
 * I2C Scanner for Umbreon (RP2350 / Pico 2)
 *
 * Scans the I2C bus on GP0 (SDA) / GP1 (SCL) at 400 kHz.
 * Expected device: MPU-6050 at 0x68.
 *
 * Open Serial Monitor at 115200 baud.
 */

#include <Wire.h>

#define I2C_SDA  0   // GP0 — same as main firmware
#define I2C_SCL  1   // GP1

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();
    Wire.setClock(400000);

    Serial.println("\nUmbreon I2C Scanner");
    Serial.print("SDA=GP"); Serial.print(I2C_SDA);
    Serial.print("  SCL=GP"); Serial.println(I2C_SCL);
}

void loop() {
    Serial.println("\nScanning...");

    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();

        if (err == 0) {
            Serial.print("  0x");
            if (addr < 16) Serial.print("0");
            Serial.print(addr, HEX);

            if (addr == 0x68) Serial.print("  MPU-6050 (gyro)");
            if (addr == 0x69) Serial.print("  MPU-6050 (AD0=HIGH)");

            Serial.println();
            found++;
        } else if (err == 4) {
            Serial.print("  0x");
            if (addr < 16) Serial.print("0");
            Serial.print(addr, HEX);
            Serial.println("  ERROR");
        }
    }

    if (found == 0)
        Serial.println("No I2C devices found!");
    else {
        Serial.print(found);
        Serial.println(" device(s) found.");
    }

    delay(5000);
}
