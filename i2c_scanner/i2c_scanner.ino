// I2C Scanner for Raspberry Pi Pico 2
// Scans I2C0 and I2C1 with pin-level diagnostics
//
// I2C0: GP0 (SDA), GP1 (SCL) — OLED + MPU-6050
// I2C1: GP20 (SDA), GP21 (SCL) — VL53L0X sensors
//
// Uses second core as watchdog to recover from hung digitalRead/Wire

#include <Wire.h>

// Watchdog on core1 — resets if core0 gets stuck
volatile bool core0_alive = true;
volatile bool core0_stuck = false;

void setup1() {}
void loop1() {
    delay(5000);
    if (!core0_alive) {
        core0_stuck = true;
        Serial.println("\n  *** TIMEOUT — core0 stuck! Rebooting... ***");
        Serial.flush();
        delay(100);
        rp2040.reboot();
    }
    core0_alive = false;
}

// ─── Safe pin read with GPIO override ────────────────────────────────────────
// Force pin to SIO (GPIO) mode, clearing any I2C/PIO alt function
int safe_digital_read(uint8_t pin) {
    gpio_init(pin);                 // reset to SIO function
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
    busy_wait_us(100);
    return gpio_get(pin) ? 1 : 0;
}

// ─── Scan ────────────────────────────────────────────────────────────────────
void scan_bus(TwoWire& bus, const char* name) {
    Serial.print("  Scanning... ");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        bus.beginTransmission(addr);
        uint8_t err = bus.endTransmission();
        if (err == 0) {
            if (found == 0) Serial.println();
            Serial.print("    0x");
            if (addr < 16) Serial.print("0");
            Serial.print(addr, HEX);

            if (addr == 0x3C || addr == 0x3D) Serial.print("  <- SSD1306 OLED");
            if (addr == 0x68)                 Serial.print("  <- MPU-6050 (AD0=LOW)");
            if (addr == 0x69)                 Serial.print("  <- MPU-6050 (AD0=HIGH)");
            if (addr == 0x29)                 Serial.print("  <- VL53L0X (default)");
            if (addr >= 0x30 && addr <= 0x35) { Serial.print("  <- VL53L0X #"); Serial.print(addr - 0x30); }

            Serial.println();
            found++;
        }
    }
    if (found == 0) Serial.println("no devices");
    else { Serial.print("  Total: "); Serial.println(found); }
}

// ─── Check + recover bus lines ───────────────────────────────────────────────
bool check_bus(uint8_t sda, uint8_t scl) {
    core0_alive = true;

    Serial.print("  SDA(GP"); Serial.print(sda); Serial.print(")=");
    Serial.flush();
    int sda_v = safe_digital_read(sda);
    Serial.print(sda_v);

    Serial.print("  SCL(GP"); Serial.print(scl); Serial.print(")=");
    Serial.flush();
    int scl_v = safe_digital_read(scl);
    Serial.println(scl_v);

    core0_alive = true;

    if (!sda_v || !scl_v) {
        Serial.print("  STUCK:");
        if (!sda_v) Serial.print(" SDA=LOW");
        if (!scl_v) Serial.print(" SCL=LOW");
        Serial.println();

        // Recovery: clock SCL + STOP condition
        Serial.print("  Recovery...");
        gpio_init(sda); gpio_set_dir(sda, GPIO_IN); gpio_pull_up(sda);
        gpio_init(scl); gpio_set_dir(scl, GPIO_OUT);
        for (int j = 0; j < 16; j++) {
            gpio_put(scl, 0); busy_wait_us(5);
            gpio_put(scl, 1); busy_wait_us(5);
        }
        gpio_set_dir(sda, GPIO_OUT);
        gpio_put(sda, 0); busy_wait_us(5);
        gpio_put(scl, 1); busy_wait_us(5);
        gpio_put(sda, 1); busy_wait_us(5);
        gpio_set_dir(sda, GPIO_IN); gpio_pull_up(sda);
        gpio_set_dir(scl, GPIO_IN); gpio_pull_up(scl);
        busy_wait_us(1000);

        sda_v = gpio_get(sda);
        scl_v = gpio_get(scl);
        Serial.print(" SDA="); Serial.print(sda_v);
        Serial.print(" SCL="); Serial.println(scl_v);
        if (!sda_v || !scl_v) {
            Serial.println("  FAILED — skip");
            return false;
        }
    }
    return true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(1000);
    Serial.println("=== I2C Scanner — Pico 2 ===");
    Serial.println("Core1 watchdog: 5s timeout\n");
}

void loop() {
    static unsigned long n = 0;
    n++;
    Serial.print("--- Scan #"); Serial.print(n); Serial.println(" ---");
    core0_alive = true;

    // ─── I2C0 ───
    Serial.println("\n[I2C0 — OLED+IMU]  GP0/GP1");
    if (check_bus(0, 1)) {
        Wire.setSDA(0);
        Wire.setSCL(1);
        Wire.begin();
        Wire.setClock(100000);
        scan_bus(Wire, "I2C0");
        Wire.end();
    }
    core0_alive = true;

    // ─── I2C1 — GP20/GP21 (current) ───
    Serial.println("\n[I2C1]  GP20/GP21");
    if (check_bus(20, 21)) {
        Wire1.setSDA(20);
        Wire1.setSCL(21);
        Wire1.begin();
        Wire1.setClock(100000);
        scan_bus(Wire1, "I2C1 GP20/21");
        Wire1.end();
    }
    core0_alive = true;

    // ─── I2C1 — GP2/GP3 (alt) ───
    Serial.println("\n[I2C1 alt]  GP2/GP3");
    if (check_bus(2, 3)) {
        Wire1.setSDA(2);
        Wire1.setSCL(3);
        Wire1.begin();
        Wire1.setClock(100000);
        scan_bus(Wire1, "I2C1 GP2/3");
        Wire1.end();
    }
    core0_alive = true;

    // ─── I2C1 — GP4/GP5 (alt) ───
    Serial.println("\n[I2C1 alt]  GP4/GP5");
    if (check_bus(4, 5)) {
        Wire1.setSDA(4);
        Wire1.setSCL(5);
        Wire1.begin();
        Wire1.setClock(100000);
        scan_bus(Wire1, "I2C1 GP4/5");
        Wire1.end();
    }
    core0_alive = true;

    // ─── I2C1 — GP26/GP27 (alt) ───
    Serial.println("\n[I2C1 alt]  GP26/GP27");
    if (check_bus(26, 27)) {
        Wire1.setSDA(26);
        Wire1.setSCL(27);
        Wire1.begin();
        Wire1.setClock(100000);
        scan_bus(Wire1, "I2C1 GP26/27");
        Wire1.end();
    }
    core0_alive = true;

    Serial.println();
    delay(3000);
    core0_alive = true;
}
