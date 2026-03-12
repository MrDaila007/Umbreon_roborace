/**
 * hardware_test — Full hardware check for Umbreon
 * RP2350 (Pico 2)
 *
 * Open Serial Monitor at 115200 baud.
 * The sketch runs each test automatically in sequence.
 * Send a command letter at any time to jump to a specific test:
 *
 *   l  — LiDAR stream (continuous, all 4 sensors)
 *   s  — Servo sweep
 *   t  — Tachometer live (spin wheel by hand)
 *   e  — ESC arm + minimal forward pulse (5 s, then stop)
 *   p  — Speed hold test (PID by tachometer, 62 pulses/rev)
 *   u  — PID auto-tune (relay method, ~30s)
 *   r  — Reactive steering (LiDAR → servo, live obstacle avoidance)
 *   a  — Run all tests in sequence
 *   ?  — Print this help
 */

#include <Servo.h>
#include <SerialPIO.h>

// ─── Pins ─────────────────────────────────────────────────────────────────────
#define SERVO_PIN   10
#define MOTOR_PIN   11
#define TAHO_PIN    13

static const uint8_t  LIDAR_RX[4] = {2, 3, 4, 5};
static const uint32_t LIDAR_BAUD  = 115200;
static const char*    LIDAR_NAME[4] = {"Left    ", "FrontL  ", "FrontR  ", "Right   "};

// ─── ESC / Servo limits ───────────────────────────────────────────────────────
#define SERVO_NEUTRAL   90
#define SERVO_LEFT      40
#define SERVO_RIGHT     140
#define ESC_NEUTRAL     90
#define ESC_FWD_MIN     96    // minimal forward — just enough to confirm arming
#define ESC_FWD_MAX     120   // safety cap for speed-hold test

// ─── Tachometer / wheel geometry ─────────────────────────────────────────────
#define PULSES_PER_REV  62
#define WHEEL_DIAM_M    0.063f
#define WHEEL_CIRC_M    (3.14159f * WHEEL_DIAM_M)

// ─── LiDAR parser ─────────────────────────────────────────────────────────────
struct LidarState { uint8_t buf[9]; uint8_t idx; bool hdr; uint16_t cm; bool ok; };

SerialPIO ls0(NOPIN, LIDAR_RX[0]);
SerialPIO ls1(NOPIN, LIDAR_RX[1]);
SerialPIO ls2(NOPIN, LIDAR_RX[2]);
SerialPIO ls3(NOPIN, LIDAR_RX[3]);
SerialPIO* ls[4] = {&ls0, &ls1, &ls2, &ls3};

LidarState ld[4] = {};

void lidar_feed(int id, uint8_t b) {
    LidarState& s = ld[id];
    if (!s.hdr) {
        if      (s.idx == 0 && b == 0x59) s.buf[s.idx++] = b;
        else if (s.idx == 1 && b == 0x59) { s.buf[s.idx++] = b; s.hdr = true; }
        else                               s.idx = 0;
        return;
    }
    s.buf[s.idx++] = b;
    if (s.idx == 9) {
        uint8_t sum = 0;
        for (int j = 0; j < 8; j++) sum += s.buf[j];
        if (sum == s.buf[8]) {
            s.cm = s.buf[2] | (s.buf[3] << 8);
            s.ok = true;
        }
        s.idx = 0; s.hdr = false;
    }
}

void poll_all_lidars() {
    for (int i = 0; i < 4; i++) {
        while (ls[i]->available() > 0) {
            int c = ls[i]->read();
            if (c >= 0) lidar_feed(i, (uint8_t)c);
        }
    }
}

// ─── Tachometer ───────────────────────────────────────────────────────────────
volatile unsigned long taho_count = 0;
volatile unsigned long taho_last  = 0;
volatile unsigned long taho_iv    = 0;

void taho_isr() {
    unsigned long now = micros();
    unsigned long dt  = now - taho_last;
    if (dt < 500) return;
    taho_count++;
    taho_iv   = dt;
    taho_last = now;
}

// ─── Actuators ────────────────────────────────────────────────────────────────
Servo steer;
Servo esc;

// ─── Print helpers ────────────────────────────────────────────────────────────
void print_help() {
    Serial.println("\nCommands:");
    Serial.println("  l  LiDAR stream");
    Serial.println("  s  Servo sweep");
    Serial.println("  t  Tachometer live");
    Serial.println("  e  ESC arm test  (car will move briefly!)");
    Serial.println("  p  Speed hold (PID, +/- to change target)");
    Serial.println("  u  PID auto-tune (relay method, ~30s)");
    Serial.println("  r  Reactive steering (LiDAR → servo)");
    Serial.println("  a  All tests in sequence");
    Serial.println("  ?  This help\n");
}

void flush_serial() { while (Serial.available()) Serial.read(); }

char wait_key() {
    while (!Serial.available()) { poll_all_lidars(); delay(1); }
    char c = Serial.read();
    flush_serial();
    return c;
}

// ─── Test: LiDAR ─────────────────────────────────────────────────────────────
void test_lidar(unsigned long duration_ms = 0) {
    Serial.println("\n=== LiDAR Test ===");
    Serial.println("Press any key to stop.\n");
    Serial.println("Sensor    | Distance  | Status");
    Serial.println("----------|-----------|--------");

    unsigned long start = millis();
    while (!Serial.available()) {
        if (duration_ms && (millis() - start) > duration_ms) break;
        poll_all_lidars();

        // Move cursor up 4 lines (VT100) — works in most Serial monitors
        Serial.print("\x1b[4A");
        for (int i = 0; i < 4; i++) {
            Serial.print(LIDAR_NAME[i]);
            Serial.print(" | ");
            if (ld[i].ok) {
                Serial.print(ld[i].cm);
                Serial.print(" cm      | OK    ");
            } else {
                Serial.print("--         | no data");
            }
            Serial.println();
        }
        delay(100);
    }
    flush_serial();
    Serial.println("\nLiDAR test done.");
}

// ─── Test: Servo ─────────────────────────────────────────────────────────────
void test_servo() {
    Serial.println("\n=== Servo Test ===");
    Serial.println("Sweeping LEFT..."); steer.write(SERVO_LEFT);  delay(800);
    Serial.println("Sweeping RIGHT..."); steer.write(SERVO_RIGHT); delay(800);
    Serial.println("Centre."); steer.write(SERVO_NEUTRAL);        delay(400);

    Serial.println("Slow sweep left→right→centre:");
    for (int a = SERVO_LEFT; a <= SERVO_RIGHT; a++) { steer.write(a); delay(8); }
    for (int a = SERVO_RIGHT; a >= SERVO_LEFT; a--) { steer.write(a); delay(8); }
    steer.write(SERVO_NEUTRAL);
    Serial.println("Servo test done.");
}

// ─── Test: Tachometer ────────────────────────────────────────────────────────
void test_taho(unsigned long duration_ms = 0) {
    Serial.println("\n=== Tachometer Test ===");
    Serial.println("Spin the wheel by hand. Press any key to stop.\n");

    noInterrupts(); taho_count = 0; taho_last = micros(); taho_iv = 0; interrupts();

    unsigned long start = millis();
    while (!Serial.available()) {
        if (duration_ms && (millis() - start) > duration_ms) break;

        noInterrupts();
        unsigned long cnt = taho_count;
        unsigned long iv  = taho_iv;
        interrupts();

        bool stopped = (micros() - taho_last) > 500000UL;
        float speed_ms = 0;
        if (!stopped && iv > 0) {
            speed_ms = (3.14159f * 0.063f) / (62.0f * (iv / 1e6f));
        }

        Serial.print("Pulses: "); Serial.print(cnt);
        Serial.print("   interval: "); Serial.print(iv); Serial.print(" us");
        Serial.print("   speed: "); Serial.print(speed_ms, 2); Serial.print(" m/s");
        Serial.print(stopped ? "   [STOPPED]" : "   [spinning]");
        Serial.println();
        delay(150);
    }
    flush_serial();
    Serial.println("Tachometer test done.");
}

// ─── Test: ESC ───────────────────────────────────────────────────────────────
void test_esc() {
    Serial.println("\n=== ESC Test ===");
    Serial.println("!! WHEEL WILL SPIN FOR ~2 SECONDS !!");
    Serial.println("Confirm by sending 'y', anything else cancels.");

    char c = wait_key();
    if (c != 'y' && c != 'Y') { Serial.println("Cancelled."); return; }

    Serial.println("Arming ESC (neutral 2 s)...");
    esc.write(ESC_NEUTRAL); delay(2000);

    // reset tachometer before motor spin
    noInterrupts();
    taho_count = 0;
    taho_last  = micros();
    taho_iv    = 0;
    interrupts();

    Serial.println("Minimal forward pulse (2 s)...");
    esc.write(ESC_FWD_MIN);

    unsigned long esc_start = millis();
    while (millis() - esc_start < 2000) {
        poll_all_lidars();

        noInterrupts();
        unsigned long cnt = taho_count;
        unsigned long iv  = taho_iv;
        unsigned long last = taho_last;
        interrupts();

        bool stopped = (micros() - last) > 500000UL;
        float speed_ms = 0;
        if (!stopped && iv > 0) {
            speed_ms = WHEEL_CIRC_M / (PULSES_PER_REV * (iv / 1e6f));
        }
        float revs = (float)cnt / PULSES_PER_REV;

        Serial.print("\x1b[1A");
        Serial.print("Pulses: "); Serial.print(cnt);
        Serial.print("  revs: "); Serial.print(revs, 1);
        Serial.print("  speed: "); Serial.print(speed_ms, 2); Serial.print(" m/s");
        Serial.print(stopped ? "  [STOPPED]  " : "  [spinning] ");
        Serial.println();
        delay(100);
    }

    Serial.println("Stopping.");
    esc.write(ESC_NEUTRAL); delay(500);

    noInterrupts();
    unsigned long final_cnt = taho_count;
    interrupts();

    Serial.print("Total pulses: "); Serial.print(final_cnt);
    Serial.print("  ("); Serial.print((float)final_cnt / PULSES_PER_REV, 1);
    Serial.println(" revolutions)");
    Serial.println("ESC test done.");
}

// ─── Test: Speed Hold (PID) ──────────────────────────────────────────────────
void test_speed_hold() {
    Serial.println("\n=== Speed Hold Test ===");
    Serial.println("!! WHEEL WILL SPIN !!");
    Serial.println("Confirm by sending 'y', anything else cancels.");

    char c = wait_key();
    if (c != 'y' && c != 'Y') { Serial.println("Cancelled."); return; }

    float target_speed = 1.5f;  // m/s — starting target
    float kP = 4.18f;   // auto-tuned (Tyreus-Luyben)
    float kI = 2.93f;
    float kD = 0.43f;
    float integral   = 0;
    float prev_error = 0;
    float filtered_speed = 0;  // exponential moving average
    int   esc_val    = ESC_NEUTRAL;
    unsigned long prev_cnt = 0; // for count-based speed

    Serial.println("Arming ESC (neutral 2 s)...");
    esc.write(ESC_NEUTRAL);
    delay(2000);

    Serial.println("Starting PID loop.  +/- change target,  any other key = stop.\n");
    Serial.println("CSV:ms,target,raw,filtered,esc,error,integral,deriv,pulses");

    noInterrupts();
    taho_count = 0;
    taho_last  = micros();
    taho_iv    = 0;
    interrupts();

    unsigned long prev_ms = millis();

    while (true) {
        poll_all_lidars();

        // handle user input
        if (Serial.available()) {
            char ch = Serial.read();
            if (ch == '+' || ch == '=') {
                target_speed += 0.1f;
                Serial.print("\n>> Target → "); Serial.print(target_speed, 1); Serial.println(" m/s");
            } else if (ch == '-' || ch == '_') {
                target_speed -= 0.1f;
                if (target_speed < 0) target_speed = 0;
                Serial.print("\n>> Target → "); Serial.print(target_speed, 1); Serial.println(" m/s");
            } else {
                break;  // any other key = stop
            }
            flush_serial();
        }

        unsigned long now_ms = millis();
        if (now_ms - prev_ms < 80) continue;  // 12.5 Hz — more pulses per tick
        float dt = (now_ms - prev_ms) / 1000.0f;
        prev_ms = now_ms;

        // read tachometer — count-based speed (pulses per period)
        noInterrupts();
        unsigned long cnt = taho_count;
        unsigned long last = taho_last;
        interrupts();

        unsigned long delta_cnt = cnt - prev_cnt;
        prev_cnt = cnt;

        // speed from pulse count over dt
        float raw_speed = (delta_cnt / (float)PULSES_PER_REV) * WHEEL_CIRC_M / dt;

        // light filter
        const float alpha = 0.5f;
        filtered_speed = alpha * raw_speed + (1.0f - alpha) * filtered_speed;

        // detect full stop
        bool stopped = (micros() - last) > 500000UL;
        if (stopped) { filtered_speed = 0; }

        // PID
        float error = target_speed - filtered_speed;
        integral += error * dt;
        integral = constrain(integral, -5.0f, 5.0f);  // anti-windup
        float derivative = (error - prev_error) / dt;
        prev_error = error;

        // feedforward: motor doesn't spin below ESC_FWD_MIN
        float ff = (target_speed > 0.01f) ? (float)(ESC_FWD_MIN - ESC_NEUTRAL) : 0;
        float output = ff + kP * error + kI * integral + kD * derivative;
        esc_val = ESC_NEUTRAL + (int)output;
        esc_val = constrain(esc_val, ESC_NEUTRAL, ESC_FWD_MAX);
        esc.write(esc_val);

        // CSV telemetry
        Serial.print("D,");
        Serial.print(now_ms); Serial.print(",");
        Serial.print(target_speed, 2); Serial.print(",");
        Serial.print(raw_speed, 2); Serial.print(",");
        Serial.print(filtered_speed, 2); Serial.print(",");
        Serial.print(esc_val); Serial.print(",");
        Serial.print(error, 3); Serial.print(",");
        Serial.print(integral, 3); Serial.print(",");
        Serial.print(derivative, 3); Serial.print(",");
        Serial.println(cnt);
    }

    esc.write(ESC_NEUTRAL);
    flush_serial();

    noInterrupts();
    unsigned long final_cnt = taho_count;
    interrupts();

    Serial.print("\nStopped. Total pulses: "); Serial.print(final_cnt);
    Serial.print("  ("); Serial.print((float)final_cnt / PULSES_PER_REV, 1);
    Serial.println(" revolutions)");
    Serial.println("Speed hold test done.");
}

// ─── Test: PID Auto-Tune (Relay / Åström-Hägglund) ──────────────────────────
void test_autotune() {
    Serial.println("\n=== PID Auto-Tuner (Relay Method) ===");
    Serial.println("!! WHEEL WILL SPIN for ~30s !!");
    Serial.println("Confirm 'y', anything else cancels.");

    char c = wait_key();
    if (c != 'y' && c != 'Y') { Serial.println("Cancelled."); return; }

    // ── Tuning parameters ──
    const float TARGET     = 1.5f;
    const int   RELAY_D    = 2;       // relay half-amplitude (ESC units)
    const float HYST       = 0.10f;   // hysteresis band (m/s)
    const int   BASE_ESC   = 98;      // empirical: ~1.5 m/s operating point
    const int   SKIP_HALF  = 4;       // skip transient half-cycles
    const int   NEED_HALF  = 12;      // measure these half-cycles
    const unsigned long TIMEOUT = 40000;

    // ── Arm ESC ──
    Serial.println("Arming ESC (2s)...");
    esc.write(ESC_NEUTRAL);
    delay(2000);

    noInterrupts();
    taho_count = 0; taho_last = micros(); taho_iv = 0;
    interrupts();

    Serial.println("Phase 1: Relay oscillation...");
    Serial.println("CSV:ms,target,raw,filtered,esc,relay");

    // ── State ──
    bool relay_high = true;
    float filtered = 0;
    float speed_peak = 0, speed_trough = 999.0f;
    unsigned long prev_cnt = 0, prev_ms = millis(), start_ms = prev_ms;
    int half_cycle = 0;

    // ── Measurement ──
    const int MAXM = 16;
    float meas_peaks[MAXM], meas_troughs[MAXM];
    unsigned long sw_times[MAXM * 2];
    int np = 0, nt = 0, nsw = 0;

    int esc_val = constrain(BASE_ESC + RELAY_D, ESC_NEUTRAL, ESC_FWD_MAX);
    esc.write(esc_val);

    while (millis() - start_ms < TIMEOUT) {
        poll_all_lidars();
        if (Serial.available()) { flush_serial(); break; }

        unsigned long now = millis();
        if (now - prev_ms < 80) continue;   // 12.5 Hz — more pulses per tick
        float dt = (now - prev_ms) / 1000.0f;
        prev_ms = now;

        // ── Speed (count-based) ──
        noInterrupts();
        unsigned long cnt = taho_count;
        unsigned long last = taho_last;
        interrupts();

        unsigned long dc = cnt - prev_cnt;
        prev_cnt = cnt;
        float raw = (dc / (float)PULSES_PER_REV) * WHEEL_CIRC_M / dt;
        filtered = 0.5f * raw + 0.5f * filtered;
        if ((micros() - last) > 500000UL) filtered = 0;

        // ── Track extremes ──
        if (filtered > speed_peak)   speed_peak   = filtered;
        if (filtered < speed_trough) speed_trough = filtered;

        // ── Relay switching ──
        float spd_err = filtered - TARGET;

        if (relay_high && spd_err > HYST) {
            // H→L: ending HIGH phase, trough was in this phase
            relay_high = false;
            if (half_cycle >= SKIP_HALF && nt < MAXM)
                meas_troughs[nt++] = speed_trough;
            if (half_cycle >= SKIP_HALF && nsw < MAXM * 2)
                sw_times[nsw++] = now;
            half_cycle++;
            speed_peak = filtered;       // reset for LOW phase
            speed_trough = 999.0f;
        }
        else if (!relay_high && spd_err < -HYST) {
            // L→H: ending LOW phase, peak was in this phase
            relay_high = true;
            if (half_cycle >= SKIP_HALF && np < MAXM)
                meas_peaks[np++] = speed_peak;
            if (half_cycle >= SKIP_HALF && nsw < MAXM * 2)
                sw_times[nsw++] = now;
            half_cycle++;
            speed_trough = filtered;     // reset for HIGH phase
            speed_peak = 0;
        }

        esc_val = relay_high ? (BASE_ESC + RELAY_D) : (BASE_ESC - RELAY_D);
        esc_val = constrain(esc_val, ESC_NEUTRAL, ESC_FWD_MAX);
        esc.write(esc_val);

        // CSV
        Serial.print("D,"); Serial.print(now);
        Serial.print(","); Serial.print(TARGET, 2);
        Serial.print(","); Serial.print(raw, 2);
        Serial.print(","); Serial.print(filtered, 2);
        Serial.print(","); Serial.print(esc_val);
        Serial.print(","); Serial.println(relay_high ? 1 : 0);

        if (half_cycle >= SKIP_HALF + NEED_HALF) break;
    }

    esc.write(ESC_NEUTRAL);
    delay(500);

    // ── Calculate ──
    if (np < 2 || nt < 2 || nsw < 4) {
        Serial.println("\nERR: Not enough oscillation data!");
        return;
    }

    float avg_peak = 0, avg_trough = 0;
    for (int i = 0; i < np; i++) avg_peak   += meas_peaks[i];
    for (int i = 0; i < nt; i++) avg_trough += meas_troughs[i];
    avg_peak   /= np;
    avg_trough /= nt;

    float amplitude = (avg_peak - avg_trough) / 2.0f;

    // Period: sw_times[i] and sw_times[i+2] are same-type switches
    float sum_period = 0;
    int n_periods = 0;
    for (int i = 0; i + 2 < nsw; i++) {
        sum_period += (sw_times[i + 2] - sw_times[i]) / 1000.0f;
        n_periods++;
    }
    float Tu = (n_periods > 0) ? sum_period / n_periods : 1.0f;
    float Ku = 4.0f * RELAY_D / (3.14159f * amplitude);

    // ── Ziegler-Nichols PID ──
    float zn_kP = 0.6f * Ku;
    float zn_kI = zn_kP / (0.5f * Tu);
    float zn_kD = zn_kP * Tu / 8.0f;

    // ── Tyreus-Luyben (conservative, good for noisy) ──
    float tl_kP = Ku / 2.2f;
    float tl_kI = tl_kP / (2.2f * Tu);
    float tl_kD = tl_kP * Tu / 6.3f;

    // ── PI only (no derivative — safest for noisy encoder) ──
    float pi_kP = 0.45f * Ku;
    float pi_kI = pi_kP / (0.83f * Tu);

    // ── Print results ──
    Serial.println("\n======== AUTO-TUNE RESULTS ========");
    Serial.print("Peaks(n="); Serial.print(np);
    Serial.print(") avg="); Serial.println(avg_peak, 3);
    Serial.print("Troughs(n="); Serial.print(nt);
    Serial.print(") avg="); Serial.println(avg_trough, 3);
    Serial.print("Amplitude: "); Serial.print(amplitude, 3); Serial.println(" m/s");
    Serial.print("Period Tu: "); Serial.print(Tu, 3); Serial.println(" s");
    Serial.print("Ult. gain Ku: "); Serial.println(Ku, 2);
    Serial.print("Relay d: "); Serial.println(RELAY_D);

    Serial.println("\nR,method,kP,kI,kD");
    Serial.print("R,ZN,");   Serial.print(zn_kP,2); Serial.print(","); Serial.print(zn_kI,2); Serial.print(","); Serial.println(zn_kD,3);
    Serial.print("R,TL,");   Serial.print(tl_kP,2); Serial.print(","); Serial.print(tl_kI,2); Serial.print(","); Serial.println(tl_kD,3);
    Serial.print("R,PI,");   Serial.print(pi_kP,2); Serial.print(","); Serial.print(pi_kI,2); Serial.println(",0.000");

    Serial.println("===================================");

    // ── Phase 2: Verify with Tyreus-Luyben gains (10s) ──
    Serial.println("\nPhase 2: Verifying with Tyreus-Luyben gains (10s)...");
    Serial.println("CSV:ms,target,raw,filtered,esc,error,integral,deriv,pulses");

    float kP = tl_kP, kI = tl_kI, kD = tl_kD;
    float integral = 0, prev_error = 0;
    filtered = 0;
    prev_cnt = 0;

    esc.write(ESC_NEUTRAL);
    delay(2000);

    noInterrupts();
    taho_count = 0; taho_last = micros(); taho_iv = 0;
    interrupts();

    prev_ms = millis();
    unsigned long verify_start = millis();

    while (millis() - verify_start < 10000) {
        poll_all_lidars();
        if (Serial.available()) { flush_serial(); break; }

        unsigned long now = millis();
        if (now - prev_ms < 80) continue;   // same rate as main loop
        float vdt = (now - prev_ms) / 1000.0f;
        prev_ms = now;

        noInterrupts();
        unsigned long vcnt = taho_count;
        unsigned long vlast = taho_last;
        interrupts();

        unsigned long vdc = vcnt - prev_cnt;
        prev_cnt = vcnt;
        float vraw = (vdc / (float)PULSES_PER_REV) * WHEEL_CIRC_M / vdt;
        filtered = 0.5f * vraw + 0.5f * filtered;
        if ((micros() - vlast) > 500000UL) filtered = 0;

        float error = TARGET - filtered;
        integral += error * vdt;
        integral = constrain(integral, -5.0f, 5.0f);
        float deriv = (error - prev_error) / vdt;
        prev_error = error;

        float ff = (TARGET > 0.01f) ? (float)(ESC_FWD_MIN - ESC_NEUTRAL) : 0;
        float out = ff + kP * error + kI * integral + kD * deriv;
        esc_val = ESC_NEUTRAL + (int)out;
        esc_val = constrain(esc_val, ESC_NEUTRAL, ESC_FWD_MAX);
        esc.write(esc_val);

        Serial.print("D,"); Serial.print(now);
        Serial.print(","); Serial.print(TARGET, 2);
        Serial.print(","); Serial.print(vraw, 2);
        Serial.print(","); Serial.print(filtered, 2);
        Serial.print(","); Serial.print(esc_val);
        Serial.print(","); Serial.print(error, 3);
        Serial.print(","); Serial.print(integral, 3);
        Serial.print(","); Serial.print(deriv, 3);
        Serial.print(","); Serial.println(vcnt);
    }

    esc.write(ESC_NEUTRAL);

    noInterrupts();
    unsigned long final_cnt = taho_count;
    interrupts();

    Serial.print("\nVerify done. Pulses: "); Serial.print(final_cnt);
    Serial.print("  ("); Serial.print((float)final_cnt / PULSES_PER_REV, 1);
    Serial.println(" revs)");
    Serial.println("Auto-tune complete.");
}

// ─── Test: Reactive Steering (LiDAR → Servo) ────────────────────────────────
void test_reactive() {
    Serial.println("\n=== Reactive Steering Test ===");
    Serial.println("LiDAR controls servo in real-time. Press any key to stop.\n");
    Serial.println("  Left(L)  FrontL(FL)  FrontR(FR)  Right(R)  → Steer");
    Serial.println("  ─────────────────────────────────────────────────────");

    const int CLOSE_DIST = 120;   // cm — start reacting
    const int FAR_DIST   = 300;   // cm — fully open

    while (!Serial.available()) {
        poll_all_lidars();

        int L  = ld[0].ok ? ld[0].cm : 999;
        int FL = ld[1].ok ? ld[1].cm : 999;
        int FR = ld[2].ok ? ld[2].cm : 999;
        int R  = ld[3].ok ? ld[3].cm : 999;

        // Steering logic: balance left vs right
        // Negative diff → steer left, positive → steer right
        float diff = 0;

        // Side sensors: push away from close wall
        diff += (R - L);

        // Front sensors: steer away from blocked side
        if (FL < CLOSE_DIST) diff += (CLOSE_DIST - FL);    // FL blocked → steer right (+)
        if (FR < CLOSE_DIST) diff -= (CLOSE_DIST - FR);    // FR blocked → steer left  (-)

        // Normalize to servo range
        float steer_f = diff / (float)FAR_DIST;
        steer_f = constrain(steer_f, -1.0f, 1.0f);

        int servo_val;
        if (steer_f < 0)
            servo_val = map((int)(steer_f * 1000), -1000, 0, SERVO_LEFT, SERVO_NEUTRAL);
        else
            servo_val = map((int)(steer_f * 1000), 0, 1000, SERVO_NEUTRAL, SERVO_RIGHT);

        steer.write(servo_val);

        // Display
        Serial.print("\x1b[1A");
        Serial.print("  L="); Serial.print(L);
        Serial.print("  FL="); Serial.print(FL);
        Serial.print("  FR="); Serial.print(FR);
        Serial.print("  R="); Serial.print(R);
        Serial.print("  → servo="); Serial.print(servo_val);
        if (steer_f < -0.2f)      Serial.print("  ←LEFT   ");
        else if (steer_f > 0.2f)  Serial.print("  RIGHT→  ");
        else                       Serial.print("  CENTER  ");
        Serial.println();

        delay(50);
    }

    steer.write(SERVO_NEUTRAL);
    flush_serial();
    Serial.println("\nReactive steering test done.");
}

// ─── All tests ────────────────────────────────────────────────────────────────
void run_all() {
    test_lidar(4000);
    test_servo();
    test_taho(5000);
    // ESC skipped in auto-run — requires manual confirmation
    Serial.println("\nAll automatic tests done. Run 'e' for ESC test.");
}

// ─── Setup / loop ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    steer.attach(SERVO_PIN);
    esc.attach(MOTOR_PIN);
    steer.write(SERVO_NEUTRAL);
    esc.write(ESC_NEUTRAL);

    for (int i = 0; i < 4; i++) ls[i]->begin(LIDAR_BAUD);

    pinMode(TAHO_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(TAHO_PIN), taho_isr, RISING);

    Serial.println("\n============================");
    Serial.println("  Umbreon — Hardware Test");
    Serial.println("============================");
    print_help();
}

void loop() {
    poll_all_lidars();

    if (!Serial.available()) return;
    char cmd = Serial.read();
    flush_serial();

    switch (cmd) {
        case 'l': test_lidar();  break;
        case 's': test_servo();  break;
        case 't': test_taho();   break;
        case 'e': test_esc();    break;
        case 'p': test_speed_hold(); break;
        case 'u': test_autotune();   break;
        case 'r': test_reactive();   break;
        case 'a': run_all();     break;
        case '?': print_help();  break;
        default:
            Serial.print("Unknown command '");
            Serial.print(cmd);
            Serial.println("'. Send ? for help.");
    }
}
