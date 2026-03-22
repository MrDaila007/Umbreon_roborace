// VL53L0X Sensor + OLED standalone test for Raspberry Pi Pico 2
//
// OLED SSD1306 128x64:  I2C0 — GP0 (SDA), GP1 (SCL), addr 0x3C
// VL53L0X x6:           I2C1 — GP20 (SDA), GP21 (SCL)
// XSHUT pins:           GP6, GP7, GP8, GP9, GP14, GP15
//
// Shows live distances on OLED + Serial, diagnoses init/read problems

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>

// ─── OLED ────────────────────────────────────────────────────────────────────
#define OLED_SDA    0
#define OLED_SCL    1
#define OLED_ADDR   0x3C
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oled_ok = false;

// ─── VL53L0X ─────────────────────────────────────────────────────────────────
#define VL53_SDA    2
#define VL53_SCL    3
#define VL53_COUNT  6
#define VL53_BASE   0x30

static const uint8_t xshut_pins[VL53_COUNT] = {6, 7, 8, 9, 14, 15};
static const char*   sensor_names[VL53_COUNT] = {"HR", "FR", "R ", "L ", "FL", "HL"};

Adafruit_VL53L0X sensors[VL53_COUNT];
bool     sensor_ok[VL53_COUNT];
uint16_t range_mm[VL53_COUNT];
uint32_t read_count[VL53_COUNT];
uint32_t fail_count[VL53_COUNT];

// ─── I2C1 bus scan ───────────────────────────────────────────────────────────
void i2c1_scan() {
    Serial.println("\n--- I2C1 Scan (VL53L0X bus) ---");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire1.beginTransmission(addr);
        if (Wire1.endTransmission() == 0) {
            Serial.print("  0x");
            if (addr < 16) Serial.print("0");
            Serial.print(addr, HEX);
            if (addr == 0x29) Serial.print(" <- VL53L0X default");
            if (addr >= VL53_BASE && addr < VL53_BASE + VL53_COUNT) {
                Serial.print(" <- VL53 #");
                Serial.print(addr - VL53_BASE);
            }
            Serial.println();
            found++;
        }
    }
    Serial.print("  Found: "); Serial.println(found);
}

// ─── I2C1 bus recovery ───────────────────────────────────────────────────────
bool i2c1_recover() {
    pinMode(VL53_SDA, INPUT_PULLUP);
    pinMode(VL53_SCL, OUTPUT);
    for (int j = 0; j < 16; j++) {
        digitalWrite(VL53_SCL, LOW);
        delayMicroseconds(5);
        digitalWrite(VL53_SCL, HIGH);
        delayMicroseconds(5);
    }
    pinMode(VL53_SDA, OUTPUT);
    digitalWrite(VL53_SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(VL53_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(VL53_SDA, HIGH);
    delayMicroseconds(5);

    pinMode(VL53_SDA, INPUT_PULLUP);
    pinMode(VL53_SCL, INPUT_PULLUP);
    delay(1);
    bool sda = digitalRead(VL53_SDA);
    bool scl = digitalRead(VL53_SCL);
    Serial.print("I2C1 bus: SDA="); Serial.print(sda);
    Serial.print(" SCL="); Serial.println(scl);
    return sda && scl;
}

// ─── Init sensors one by one ─────────────────────────────────────────────────
int init_sensors() {
    // Hold all XSHUT low
    for (int i = 0; i < VL53_COUNT; i++) {
        pinMode(xshut_pins[i], OUTPUT);
        digitalWrite(xshut_pins[i], LOW);
        sensor_ok[i] = false;
        range_mm[i] = 0;
        read_count[i] = 0;
        fail_count[i] = 0;
    }
    delay(10);

    int ok = 0;
    for (int i = 0; i < VL53_COUNT; i++) {
        Serial.print("  #"); Serial.print(i);
        Serial.print(" ("); Serial.print(sensor_names[i]);
        Serial.print(") XSHUT=GP"); Serial.print(xshut_pins[i]);
        Serial.flush();

        // Bring up this sensor (others still held in reset)
        digitalWrite(xshut_pins[i], HIGH);
        delay(50);

        // Init + assign address (like Adafruit example)
        uint8_t addr = VL53_BASE + i;
        Serial.print(" -> 0x"); Serial.print(addr, HEX);
        Serial.print("...");
        Serial.flush();

        if (sensors[i].begin(addr, false, &Wire1)) {
            sensors[i].startRangeContinuous(33);
            sensor_ok[i] = true;
            ok++;
            Serial.println(" OK");
        } else {
            Serial.println(" FAIL");
            // Don't shut down — leave up, maybe address already changed
        }
    }
    return ok;
}

// ─── Draw OLED ───────────────────────────────────────────────────────────────
void draw_oled(int online) {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);

    // Header
    oled.setCursor(0, 0);
    oled.print("VL53L0X ");
    oled.print(online);
    oled.print("/");
    oled.print(VL53_COUNT);
    oled.print(" online");
    oled.drawLine(0, 9, 127, 9, SSD1306_WHITE);

    // Sensor bars + values
    for (int i = 0; i < VL53_COUNT; i++) {
        int y = 12 + i * 9;
        oled.setCursor(0, y);
        oled.print(sensor_names[i]);

        if (!sensor_ok[i]) {
            oled.setCursor(18, y);
            oled.print("--- offline ---");
            continue;
        }

        // Distance value
        oled.setCursor(18, y);
        if (range_mm[i] == 0 || range_mm[i] >= 8190) {
            oled.print("OOR");
        } else {
            oled.print(range_mm[i]);
            oled.print("mm");
        }

        // Bar graph (max 1200mm = full bar)
        if (range_mm[i] > 0 && range_mm[i] < 8190) {
            int bar_w = constrain(range_mm[i], 0, 1200);
            bar_w = map(bar_w, 0, 1200, 0, 56);
            oled.fillRect(72, y, bar_w, 7, SSD1306_WHITE);
        }
    }

    oled.display();
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(1000);

    Serial.println("=== VL53L0X + OLED Sensor Test ===");

    // ─── OLED init ───
    Wire.setSDA(OLED_SDA);
    Wire.setSCL(OLED_SCL);
    Wire.begin();
    Wire.setClock(400000);

    Serial.print("OLED init... ");
    if (oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        oled_ok = true;
        Serial.println("OK");
        oled.clearDisplay();
        oled.setTextSize(2);
        oled.setTextColor(SSD1306_WHITE);
        oled.setCursor(4, 4);
        oled.print("Sensor");
        oled.setCursor(4, 24);
        oled.print("Test");
        oled.setTextSize(1);
        oled.setCursor(4, 48);
        oled.print("VL53L0X x6 + OLED");
        oled.display();
        delay(1000);
    } else {
        Serial.println("FAIL (continuing without display)");
    }

    // ─── I2C1 bus recovery + init ───
    Serial.println("\nI2C1 bus recovery...");
    if (!i2c1_recover()) {
        Serial.println("ERROR: I2C1 stuck! Check wiring GP20/GP21");
        if (oled_ok) {
            oled.clearDisplay();
            oled.setTextSize(1);
            oled.setTextColor(SSD1306_WHITE);
            oled.setCursor(0, 0);
            oled.print("I2C1 BUS STUCK!");
            oled.setCursor(0, 16);
            oled.print("Check GP20 (SDA)");
            oled.setCursor(0, 26);
            oled.print("Check GP21 (SCL)");
            oled.setCursor(0, 42);
            oled.print("Power cycle sensors");
            oled.display();
        }
        Serial.println("Halted. Fix wiring and reset.");
        while (true) delay(1000);
    }

    Serial.print("Wire1.setSDA/SCL...");
    Serial.flush();
    Wire1.setSDA(VL53_SDA);
    Wire1.setSCL(VL53_SCL);
    Serial.println(" OK");

    Serial.print("Wire1.begin()...");
    Serial.flush();
    Wire1.begin();
    Serial.println(" OK");

    Serial.print("Wire1.setClock...");
    Serial.flush();
    Wire1.setClock(100000);
    Wire1.setTimeout(50);
    Serial.println(" OK");

    Serial.println("\nScanning I2C1 before init...");
    i2c1_scan();

    Serial.println("\nInitializing sensors...");
    int online = init_sensors();
    Serial.print("\nResult: "); Serial.print(online);
    Serial.print("/"); Serial.print(VL53_COUNT); Serial.println(" online");

    Serial.println("\nScanning I2C1 after init...");
    i2c1_scan();

    if (online == 0) {
        Serial.println("\nNo sensors! Check:");
        Serial.println("  - VCC -> 3.3V");
        Serial.println("  - GND -> GND");
        Serial.print("  - SDA -> GP"); Serial.println(VL53_SDA);
        Serial.print("  - SCL -> GP"); Serial.println(VL53_SCL);
        Serial.println("  - XSHUT wiring (GP6,7,8,9,14,15)");
    }

    Serial.println("\n--- Live readings (every 100ms) ---");
    Serial.print("     ");
    for (int i = 0; i < VL53_COUNT; i++) {
        Serial.print(sensor_names[i]); Serial.print("\t");
    }
    Serial.println();
}

void loop() {
    // Read all sensors
    int online = 0;
    for (int i = 0; i < VL53_COUNT; i++) {
        if (!sensor_ok[i]) continue;
        online++;

        if (sensors[i].isRangeComplete()) {
            uint16_t mm = sensors[i].readRangeResult();
            read_count[i]++;
            if (mm < 8190) {
                range_mm[i] = mm;
            } else {
                fail_count[i]++;
            }
        }
    }

    // Serial output
    static unsigned long last_serial = 0;
    if (millis() - last_serial >= 200) {
        last_serial = millis();
        Serial.print("["); Serial.print(millis() / 1000); Serial.print("s] ");
        for (int i = 0; i < VL53_COUNT; i++) {
            if (!sensor_ok[i]) {
                Serial.print("---\t");
            } else if (range_mm[i] == 0 || range_mm[i] >= 8190) {
                Serial.print("OOR\t");
            } else {
                Serial.print(range_mm[i]); Serial.print("mm\t");
            }
        }
        // Stats for first online sensor
        for (int i = 0; i < VL53_COUNT; i++) {
            if (sensor_ok[i]) {
                Serial.print(" (rd="); Serial.print(read_count[i]);
                Serial.print(" err="); Serial.print(fail_count[i]);
                Serial.print(")");
                break;
            }
        }
        Serial.println();
    }

    // OLED update
    static unsigned long last_oled = 0;
    if (oled_ok && millis() - last_oled >= 120) {
        last_oled = millis();
        draw_oled(online);
    }
}
