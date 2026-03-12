/**
 * taho_calibrate — Tachometer pulse counter for Umbreon
 * RP2350 (Pico 2)
 *
 * HOW TO USE:
 *   1. Upload this sketch.
 *   2. Open Serial Monitor at 115200 baud.
 *   3. Lift the car so the wheel spins freely.
 *   4. Press ENTER (or any key) in Serial Monitor → resets counter to 0.
 *   5. Rotate the wheel EXACTLY one full turn by hand (slowly is fine).
 *   6. Read the "Total pulses" number — that is your pulses-per-revolution.
 *
 * The sketch also shows:
 *   - Running pulse total (updates in place)
 *   - Time between last two pulses (µs) — useful for spotting missing holes
 *   - Estimated pulses-per-revolution (auto, based on full-stop detection)
 */

#define TAHO_PIN   13
#define DEBOUNCE   500UL   // µs — ignore bounces; optical encoder is clean

// ─── ISR state ────────────────────────────────────────────────────────────────
volatile unsigned long pulse_count     = 0;
volatile unsigned long last_pulse_us   = 0;
volatile unsigned long last_interval   = 0;

void taho_isr() {
    unsigned long now = micros();
    unsigned long dt  = now - last_pulse_us;
    if (dt < DEBOUNCE) return;          // debounce
    pulse_count++;
    last_interval  = dt;
    last_pulse_us  = now;
}

// ─── Stopped detection ────────────────────────────────────────────────────────
// Returns true if no pulse arrived for >500 ms (wheel stopped)
bool wheel_stopped() {
    return (micros() - last_pulse_us) > 500000UL;
}

// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    pinMode(TAHO_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(TAHO_PIN), taho_isr, RISING);

    Serial.println("=== Tachometer Calibration ===");
    Serial.println("Lift car, then spin wheel EXACTLY one turn.");
    Serial.println("Send any character (Enter) to reset counter.\n");
}

// ─── Loop ────────────────────────────────────────────────────────────────────
unsigned long last_print = 0;
unsigned long snapshot_count = 0;
bool was_moving = false;

void loop() {
    // Reset on any Serial input
    if (Serial.available()) {
        while (Serial.available()) Serial.read();
        noInterrupts();
        pulse_count   = 0;
        last_pulse_us = micros();
        last_interval = 0;
        interrupts();
        Serial.println("\n--- Counter reset to 0 ---\n");
        was_moving = false;
        return;
    }

    bool moving = !wheel_stopped();

    // Print every 150 ms while moving, or once when stopped
    unsigned long now = millis();
    if (moving || (was_moving && !moving)) {
        if (moving && (now - last_print < 150)) return;
        last_print = now;

        noInterrupts();
        unsigned long cnt = pulse_count;
        unsigned long iv  = last_interval;
        interrupts();

        if (moving) {
            Serial.print("Pulses: ");
            Serial.print(cnt);
            Serial.print("   |   interval: ");
            Serial.print(iv);
            Serial.println(" us");
        } else {
            // Wheel just stopped — print summary
            Serial.println();
            Serial.println("=============================");
            Serial.print("  WHEEL STOPPED. Total pulses: ");
            Serial.println(cnt);
            Serial.println("  (This is your pulses-per-revolution");
            Serial.println("   if you turned exactly one full rotation)");
            Serial.println("=============================\n");
            Serial.println("Press Enter to reset and try again.");
        }
    }

    was_moving = moving;
}
