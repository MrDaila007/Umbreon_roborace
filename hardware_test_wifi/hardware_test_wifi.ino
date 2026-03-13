/**
 * hardware_test_wifi — Full hardware check for Umbreon
 * RP2350 (Pico 2) + Wemos D1 Mini WiFi + MPU-6050 IMU
 *
 * Output goes to BOTH USB Serial Monitor AND WiFi (Serial1 → ESP → TCP/WS).
 * Input accepted from either USB or WiFi (PuTTY / netcat to 192.168.4.1:23).
 *
 * Open Serial Monitor at 115200 baud, or connect to the Umbreon WiFi AP
 * and open a raw TCP session to 192.168.4.1:23.
 *
 * Commands:
 *   l  — LiDAR stream (continuous, all 4 sensors)
 *   s  — Servo sweep
 *   t  — Tachometer live (spin wheel by hand)
 *   e  — ESC arm + minimal forward pulse (2 s, then stop)
 *   p  — Speed hold test (PID by tachometer)
 *   u  — PID auto-tune (relay method, ~30 s)
 *   r  — Reactive steering (LiDAR → servo)
 *   i  — IMU test (gyro Z bias + live yaw/heading)
 *   a  — Run all tests in sequence
 *   ?  — Print this help
 */

#pragma GCC optimize("Ofast")

#include <Servo.h>
#include <SerialPIO.h>

// ─── Feature flags ──────────────────────────────────────────────────────────
#define USE_IMU         1       // 1 = enable MPU-6050 gyro test
#define USE_WIFI_DEBUG  1       // 1 = enable output over WiFi (Serial1 → ESP)

#if USE_IMU
#include <Wire.h>
#define IMU_SDA_PIN    0
#define IMU_SCL_PIN    1
#define MPU6050_ADDR   0x68
#define GYRO_FS_500    0x08     // ±500°/s full-scale
#define GYRO_SENS      65.5f   // LSB/(°/s) at ±500°/s
#endif

#if USE_WIFI_DEBUG
#define WIFI_TX_PIN    16       // GP16 = UART0 TX → D1 Mini RX
#define WIFI_RX_PIN    17       // GP17 = UART0 RX ← D1 Mini TX
#endif

// ─── Pins ─────────────────────────────────────────────────────────────────────
#define SERVO_PIN   10
#define MOTOR_PIN   11
#define TAHO_PIN    13

static const uint8_t  LIDAR_RX[4] = {2, 3, 4, 5};
static const uint32_t LIDAR_BAUD  = 115200;
static const char*    LIDAR_NAME[4] = {"Left    ", "FrontL  ", "FrontR  ", "Right   "};

// ─── ESC / Servo limits ─────────────────────────────────────────────────────
#define SERVO_NEUTRAL   90
#define SERVO_LEFT      40
#define SERVO_RIGHT     140
#define ESC_NEUTRAL     90
#define ESC_FWD_MIN     96
#define ESC_FWD_MAX     120

// ─── Tachometer / wheel geometry ────────────────────────────────────────────
#define PULSES_PER_REV  62
#define WHEEL_DIAM_M    0.063f
#define WHEEL_CIRC_M    (3.14159f * WHEEL_DIAM_M)

// ─── Dual output (USB Serial + WiFi Serial1) ────────────────────────────────
class DualPrint : public Print {
public:
    size_t write(uint8_t c) override {
        Serial.write(c);
#if USE_WIFI_DEBUG
        Serial1.write(c);
#endif
        return 1;
    }
    size_t write(const uint8_t* buf, size_t size) override {
        Serial.write(buf, size);
#if USE_WIFI_DEBUG
        Serial1.write(buf, size);
#endif
        return size;
    }
};

DualPrint out;

// ─── Input from either USB or WiFi ──────────────────────────────────────────
bool input_available() {
    if (Serial.available()) return true;
#if USE_WIFI_DEBUG
    if (Serial1.available()) return true;
#endif
    return false;
}

char read_input() {
    if (Serial.available()) return (char)Serial.read();
#if USE_WIFI_DEBUG
    if (Serial1.available()) return (char)Serial1.read();
#endif
    return 0;
}

void flush_input() {
    while (Serial.available()) Serial.read();
#if USE_WIFI_DEBUG
    while (Serial1.available()) Serial1.read();
#endif
}

char wait_key() {
    while (!input_available()) { poll_all_lidars(); delay(1); }
    char c = read_input();
    flush_input();
    return c;
}

// ─── LiDAR parser ────────────────────────────────────────────────────────────
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

// ─── Tachometer ──────────────────────────────────────────────────────────────
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

float get_speed() {
    unsigned long elapsed = (unsigned long)(micros() - taho_last);
    elapsed = max(elapsed, taho_iv);
    if (taho_iv == 0 || elapsed > 500000UL) return 0.0f;
    return WHEEL_CIRC_M / (PULSES_PER_REV * (elapsed / 1e6f));
}

// ─── Actuators ───────────────────────────────────────────────────────────────
Servo steer;
Servo esc;

// ─── Print helpers ───────────────────────────────────────────────────────────
void print_help() {
    out.println("\nCommands:");
    out.println("  l  LiDAR stream");
    out.println("  s  Servo sweep");
    out.println("  t  Tachometer live");
    out.println("  e  ESC arm test  (car will move briefly!)");
    out.println("  p  Speed hold (PID, +/- to change target)");
    out.println("  u  PID auto-tune (relay method, ~30s)");
    out.println("  r  Reactive steering (LiDAR -> servo)");
#if USE_IMU
    out.println("  i  IMU test (gyro Z bias + live yaw/heading)");
#endif
    out.println("  a  All tests in sequence");
    out.println("  ?  This help\n");
}

// ─── Test: LiDAR ─────────────────────────────────────────────────────────────
void test_lidar(unsigned long duration_ms = 0) {
    out.println("\n=== LiDAR Test ===");
    out.println("Press any key to stop.\n");
    out.println("Sensor    | Distance  | Status");
    out.println("----------|-----------|--------");

    unsigned long start = millis();
    while (!input_available()) {
        if (duration_ms && (millis() - start) > duration_ms) break;
        poll_all_lidars();

        Serial.print("\x1b[4A");   // VT100 cursor up (USB only)
        for (int i = 0; i < 4; i++) {
            out.print(LIDAR_NAME[i]);
            out.print(" | ");
            if (ld[i].ok) {
                out.print(ld[i].cm);
                out.print(" cm      | OK    ");
            } else {
                out.print("--         | no data");
            }
            out.println();
        }
        delay(100);
    }
    flush_input();
    out.println("\nLiDAR test done.");
}

// ─── Test: Servo ─────────────────────────────────────────────────────────────
void test_servo() {
    out.println("\n=== Servo Test ===");
    out.println("Sweeping LEFT...");  steer.write(SERVO_LEFT);    delay(800);
    out.println("Sweeping RIGHT..."); steer.write(SERVO_RIGHT);   delay(800);
    out.println("Centre.");           steer.write(SERVO_NEUTRAL); delay(400);

    out.println("Slow sweep left->right->centre:");
    for (int a = SERVO_LEFT; a <= SERVO_RIGHT; a++) { steer.write(a); delay(8); }
    for (int a = SERVO_RIGHT; a >= SERVO_LEFT; a--) { steer.write(a); delay(8); }
    steer.write(SERVO_NEUTRAL);
    out.println("Servo test done.");
}

// ─── Test: Tachometer ────────────────────────────────────────────────────────
void test_taho(unsigned long duration_ms = 0) {
    out.println("\n=== Tachometer Test ===");
    out.println("Spin the wheel by hand. Press any key to stop.\n");

    noInterrupts(); taho_count = 0; taho_last = micros(); taho_iv = 0; interrupts();

    unsigned long start = millis();
    while (!input_available()) {
        if (duration_ms && (millis() - start) > duration_ms) break;

        noInterrupts();
        unsigned long cnt = taho_count;
        unsigned long iv  = taho_iv;
        interrupts();

        bool stopped = (micros() - taho_last) > 500000UL;
        float speed_ms = 0;
        if (!stopped && iv > 0) {
            speed_ms = WHEEL_CIRC_M / (PULSES_PER_REV * (iv / 1e6f));
        }

        out.print("Pulses: "); out.print(cnt);
        out.print("   interval: "); out.print(iv); out.print(" us");
        out.print("   speed: "); out.print(speed_ms, 2); out.print(" m/s");
        out.print(stopped ? "   [STOPPED]" : "   [spinning]");
        out.println();
        delay(150);
    }
    flush_input();
    out.println("Tachometer test done.");
}

// ─── Test: ESC ───────────────────────────────────────────────────────────────
void test_esc() {
    out.println("\n=== ESC Test ===");
    out.println("!! WHEEL WILL SPIN FOR ~2 SECONDS !!");
    out.println("Confirm by sending 'y', anything else cancels.");

    char c = wait_key();
    if (c != 'y' && c != 'Y') { out.println("Cancelled."); return; }

    out.println("Arming ESC (neutral 2 s)...");
    esc.write(ESC_NEUTRAL); delay(2000);

    noInterrupts();
    taho_count = 0; taho_last = micros(); taho_iv = 0;
    interrupts();

    out.println("Minimal forward pulse (2 s)...");
    esc.write(ESC_FWD_MIN);

    unsigned long esc_start = millis();
    while (millis() - esc_start < 2000) {
        poll_all_lidars();

        noInterrupts();
        unsigned long cnt  = taho_count;
        unsigned long iv   = taho_iv;
        unsigned long last = taho_last;
        interrupts();

        bool stopped = (micros() - last) > 500000UL;
        float speed_ms = 0;
        if (!stopped && iv > 0) {
            speed_ms = WHEEL_CIRC_M / (PULSES_PER_REV * (iv / 1e6f));
        }
        float revs = (float)cnt / PULSES_PER_REV;

        out.print("Pulses: "); out.print(cnt);
        out.print("  revs: "); out.print(revs, 1);
        out.print("  speed: "); out.print(speed_ms, 2); out.print(" m/s");
        out.print(stopped ? "  [STOPPED]  " : "  [spinning] ");
        out.println();
        delay(100);
    }

    out.println("Stopping.");
    esc.write(ESC_NEUTRAL); delay(500);

    noInterrupts();
    unsigned long final_cnt = taho_count;
    interrupts();

    out.print("Total pulses: "); out.print(final_cnt);
    out.print("  ("); out.print((float)final_cnt / PULSES_PER_REV, 1);
    out.println(" revolutions)");
    out.println("ESC test done.");
}

// ─── Test: Speed Hold (PID) ──────────────────────────────────────────────────
void test_speed_hold() {
    out.println("\n=== Speed Hold Test ===");
    out.println("!! WHEEL WILL SPIN !!");
    out.println("Confirm by sending 'y', anything else cancels.");

    char c = wait_key();
    if (c != 'y' && c != 'Y') { out.println("Cancelled."); return; }

    float target_speed = 1.5f;
    float kP = 4.18f, kI = 2.93f, kD = 0.43f;
    float integral = 0, prev_error = 0, filtered_speed = 0;
    int   esc_val = ESC_NEUTRAL;
    unsigned long prev_cnt = 0;

    out.println("Arming ESC (neutral 2 s)...");
    esc.write(ESC_NEUTRAL); delay(2000);

    out.println("Starting PID loop.  +/- change target,  any other key = stop.\n");
    out.println("CSV:ms,target,raw,filtered,esc,error,integral,deriv,pulses");

    noInterrupts();
    taho_count = 0; taho_last = micros(); taho_iv = 0;
    interrupts();

    unsigned long prev_ms = millis();

    while (true) {
        poll_all_lidars();

        if (input_available()) {
            char ch = read_input();
            if (ch == '+' || ch == '=') {
                target_speed += 0.1f;
                out.print("\n>> Target -> "); out.print(target_speed, 1); out.println(" m/s");
            } else if (ch == '-' || ch == '_') {
                target_speed -= 0.1f;
                if (target_speed < 0) target_speed = 0;
                out.print("\n>> Target -> "); out.print(target_speed, 1); out.println(" m/s");
            } else {
                break;
            }
            flush_input();
        }

        unsigned long now_ms = millis();
        if (now_ms - prev_ms < 80) continue;
        float dt = (now_ms - prev_ms) / 1000.0f;
        prev_ms = now_ms;

        noInterrupts();
        unsigned long cnt  = taho_count;
        unsigned long last = taho_last;
        interrupts();

        unsigned long delta_cnt = cnt - prev_cnt;
        prev_cnt = cnt;

        float raw_speed = (delta_cnt / (float)PULSES_PER_REV) * WHEEL_CIRC_M / dt;
        filtered_speed = 0.5f * raw_speed + 0.5f * filtered_speed;

        bool stopped = (micros() - last) > 500000UL;
        if (stopped) filtered_speed = 0;

        float error = target_speed - filtered_speed;
        integral += error * dt;
        integral = constrain(integral, -5.0f, 5.0f);
        float derivative = (error - prev_error) / dt;
        prev_error = error;

        float ff = (target_speed > 0.01f) ? (float)(ESC_FWD_MIN - ESC_NEUTRAL) : 0;
        float output = ff + kP * error + kI * integral + kD * derivative;
        esc_val = ESC_NEUTRAL + (int)output;
        esc_val = constrain(esc_val, ESC_NEUTRAL, ESC_FWD_MAX);
        esc.write(esc_val);

        out.print("D,");
        out.print(now_ms);          out.print(",");
        out.print(target_speed, 2); out.print(",");
        out.print(raw_speed, 2);    out.print(",");
        out.print(filtered_speed, 2); out.print(",");
        out.print(esc_val);         out.print(",");
        out.print(error, 3);        out.print(",");
        out.print(integral, 3);     out.print(",");
        out.print(derivative, 3);   out.print(",");
        out.println(cnt);
    }

    esc.write(ESC_NEUTRAL);
    flush_input();

    noInterrupts();
    unsigned long final_cnt = taho_count;
    interrupts();

    out.print("\nStopped. Total pulses: "); out.print(final_cnt);
    out.print("  ("); out.print((float)final_cnt / PULSES_PER_REV, 1);
    out.println(" revolutions)");
    out.println("Speed hold test done.");
}

// ─── Test: PID Auto-Tune (Relay) ─────────────────────────────────────────────
void test_autotune() {
    out.println("\n=== PID Auto-Tuner (Relay Method) ===");
    out.println("!! WHEEL WILL SPIN for ~30s !!");
    out.println("Confirm 'y', anything else cancels.");

    char c = wait_key();
    if (c != 'y' && c != 'Y') { out.println("Cancelled."); return; }

    const float TARGET     = 1.5f;
    const int   RELAY_D    = 2;
    const float HYST       = 0.10f;
    const int   BASE_ESC   = 98;
    const int   SKIP_HALF  = 4;
    const int   NEED_HALF  = 12;
    const unsigned long TIMEOUT = 40000;

    out.println("Arming ESC (2s)...");
    esc.write(ESC_NEUTRAL); delay(2000);

    noInterrupts();
    taho_count = 0; taho_last = micros(); taho_iv = 0;
    interrupts();

    out.println("Phase 1: Relay oscillation...");
    out.println("CSV:ms,target,raw,filtered,esc,relay");

    bool relay_high = true;
    float filtered = 0;
    float speed_peak = 0, speed_trough = 999.0f;
    unsigned long prev_cnt = 0, prev_ms = millis(), start_ms = prev_ms;
    int half_cycle = 0;

    const int MAXM = 16;
    float meas_peaks[MAXM], meas_troughs[MAXM];
    unsigned long sw_times[MAXM * 2];
    int np = 0, nt = 0, nsw = 0;

    int esc_val = constrain(BASE_ESC + RELAY_D, ESC_NEUTRAL, ESC_FWD_MAX);
    esc.write(esc_val);

    while (millis() - start_ms < TIMEOUT) {
        poll_all_lidars();
        if (input_available()) { flush_input(); break; }

        unsigned long now = millis();
        if (now - prev_ms < 80) continue;
        float dt = (now - prev_ms) / 1000.0f;
        prev_ms = now;

        noInterrupts();
        unsigned long cnt  = taho_count;
        unsigned long last = taho_last;
        interrupts();

        unsigned long dc = cnt - prev_cnt;
        prev_cnt = cnt;
        float raw = (dc / (float)PULSES_PER_REV) * WHEEL_CIRC_M / dt;
        filtered = 0.5f * raw + 0.5f * filtered;
        if ((micros() - last) > 500000UL) filtered = 0;

        if (filtered > speed_peak)   speed_peak   = filtered;
        if (filtered < speed_trough) speed_trough = filtered;

        float spd_err = filtered - TARGET;

        if (relay_high && spd_err > HYST) {
            relay_high = false;
            if (half_cycle >= SKIP_HALF && nt < MAXM)
                meas_troughs[nt++] = speed_trough;
            if (half_cycle >= SKIP_HALF && nsw < MAXM * 2)
                sw_times[nsw++] = now;
            half_cycle++;
            speed_peak = filtered;
            speed_trough = 999.0f;
        }
        else if (!relay_high && spd_err < -HYST) {
            relay_high = true;
            if (half_cycle >= SKIP_HALF && np < MAXM)
                meas_peaks[np++] = speed_peak;
            if (half_cycle >= SKIP_HALF && nsw < MAXM * 2)
                sw_times[nsw++] = now;
            half_cycle++;
            speed_trough = filtered;
            speed_peak = 0;
        }

        esc_val = relay_high ? (BASE_ESC + RELAY_D) : (BASE_ESC - RELAY_D);
        esc_val = constrain(esc_val, ESC_NEUTRAL, ESC_FWD_MAX);
        esc.write(esc_val);

        out.print("D,"); out.print(now);
        out.print(",");  out.print(TARGET, 2);
        out.print(",");  out.print(raw, 2);
        out.print(",");  out.print(filtered, 2);
        out.print(",");  out.print(esc_val);
        out.print(",");  out.println(relay_high ? 1 : 0);

        if (half_cycle >= SKIP_HALF + NEED_HALF) break;
    }

    esc.write(ESC_NEUTRAL);
    delay(500);

    if (np < 2 || nt < 2 || nsw < 4) {
        out.println("\nERR: Not enough oscillation data!");
        return;
    }

    float avg_peak = 0, avg_trough = 0;
    for (int i = 0; i < np; i++) avg_peak   += meas_peaks[i];
    for (int i = 0; i < nt; i++) avg_trough += meas_troughs[i];
    avg_peak   /= np;
    avg_trough /= nt;

    float amplitude = (avg_peak - avg_trough) / 2.0f;

    float sum_period = 0;
    int n_periods = 0;
    for (int i = 0; i + 2 < nsw; i++) {
        sum_period += (sw_times[i + 2] - sw_times[i]) / 1000.0f;
        n_periods++;
    }
    float Tu = (n_periods > 0) ? sum_period / n_periods : 1.0f;
    float Ku = 4.0f * RELAY_D / (3.14159f * amplitude);

    float zn_kP = 0.6f * Ku;
    float zn_kI = zn_kP / (0.5f * Tu);
    float zn_kD = zn_kP * Tu / 8.0f;

    float tl_kP = Ku / 2.2f;
    float tl_kI = tl_kP / (2.2f * Tu);
    float tl_kD = tl_kP * Tu / 6.3f;

    float pi_kP = 0.45f * Ku;
    float pi_kI = pi_kP / (0.83f * Tu);

    out.println("\n======== AUTO-TUNE RESULTS ========");
    out.print("Peaks(n="); out.print(np);
    out.print(") avg="); out.println(avg_peak, 3);
    out.print("Troughs(n="); out.print(nt);
    out.print(") avg="); out.println(avg_trough, 3);
    out.print("Amplitude: "); out.print(amplitude, 3); out.println(" m/s");
    out.print("Period Tu: "); out.print(Tu, 3); out.println(" s");
    out.print("Ult. gain Ku: "); out.println(Ku, 2);
    out.print("Relay d: "); out.println(RELAY_D);

    out.println("\nR,method,kP,kI,kD");
    out.print("R,ZN,");  out.print(zn_kP, 2); out.print(","); out.print(zn_kI, 2); out.print(","); out.println(zn_kD, 3);
    out.print("R,TL,");  out.print(tl_kP, 2); out.print(","); out.print(tl_kI, 2); out.print(","); out.println(tl_kD, 3);
    out.print("R,PI,");  out.print(pi_kP, 2); out.print(","); out.print(pi_kI, 2); out.println(",0.000");

    out.println("===================================");

    // Phase 2: Verify with Tyreus-Luyben gains
    out.println("\nPhase 2: Verifying with Tyreus-Luyben gains (10s)...");
    out.println("CSV:ms,target,raw,filtered,esc,error,integral,deriv,pulses");

    float kP = tl_kP, kI = tl_kI, kD = tl_kD;
    float integral = 0, prev_error = 0;
    filtered = 0;
    prev_cnt = 0;

    esc.write(ESC_NEUTRAL); delay(2000);

    noInterrupts();
    taho_count = 0; taho_last = micros(); taho_iv = 0;
    interrupts();

    prev_ms = millis();
    unsigned long verify_start = millis();

    while (millis() - verify_start < 10000) {
        poll_all_lidars();
        if (input_available()) { flush_input(); break; }

        unsigned long now = millis();
        if (now - prev_ms < 80) continue;
        float vdt = (now - prev_ms) / 1000.0f;
        prev_ms = now;

        noInterrupts();
        unsigned long vcnt  = taho_count;
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
        float vout = ff + kP * error + kI * integral + kD * deriv;
        esc_val = ESC_NEUTRAL + (int)vout;
        esc_val = constrain(esc_val, ESC_NEUTRAL, ESC_FWD_MAX);
        esc.write(esc_val);

        out.print("D,"); out.print(now);
        out.print(",");  out.print(TARGET, 2);
        out.print(",");  out.print(vraw, 2);
        out.print(",");  out.print(filtered, 2);
        out.print(",");  out.print(esc_val);
        out.print(",");  out.print(error, 3);
        out.print(",");  out.print(integral, 3);
        out.print(",");  out.print(deriv, 3);
        out.print(",");  out.println(vcnt);
    }

    esc.write(ESC_NEUTRAL);

    noInterrupts();
    unsigned long final_cnt = taho_count;
    interrupts();

    out.print("\nVerify done. Pulses: "); out.print(final_cnt);
    out.print("  ("); out.print((float)final_cnt / PULSES_PER_REV, 1);
    out.println(" revs)");
    out.println("Auto-tune complete.");
}

// ─── Test: Reactive Steering (LiDAR → Servo) ─────────────────────────────────
void test_reactive() {
    out.println("\n=== Reactive Steering Test ===");
    out.println("LiDAR controls servo in real-time. Press any key to stop.\n");
    out.println("  Left(L)  FrontL(FL)  FrontR(FR)  Right(R)  -> Steer");
    out.println("  -------------------------------------------------------");

    const int CLOSE_DIST = 120;
    const int FAR_DIST   = 300;

    while (!input_available()) {
        poll_all_lidars();

        int L  = ld[0].ok ? ld[0].cm : 999;
        int FL = ld[1].ok ? ld[1].cm : 999;
        int FR = ld[2].ok ? ld[2].cm : 999;
        int R  = ld[3].ok ? ld[3].cm : 999;

        float diff = 0;
        diff += (R - L);
        if (FL < CLOSE_DIST) diff += (CLOSE_DIST - FL);
        if (FR < CLOSE_DIST) diff -= (CLOSE_DIST - FR);

        float steer_f = diff / (float)FAR_DIST;
        steer_f = constrain(steer_f, -1.0f, 1.0f);

        int servo_val;
        if (steer_f < 0)
            servo_val = map((int)(steer_f * 1000), -1000, 0, SERVO_LEFT, SERVO_NEUTRAL);
        else
            servo_val = map((int)(steer_f * 1000), 0, 1000, SERVO_NEUTRAL, SERVO_RIGHT);

        steer.write(servo_val);

        out.print("  L="); out.print(L);
        out.print("  FL="); out.print(FL);
        out.print("  FR="); out.print(FR);
        out.print("  R="); out.print(R);
        out.print("  -> servo="); out.print(servo_val);
        if (steer_f < -0.2f)       out.print("  <-LEFT   ");
        else if (steer_f > 0.2f)   out.print("  RIGHT->  ");
        else                        out.print("  CENTER   ");
        out.println();

        delay(50);
    }

    steer.write(SERVO_NEUTRAL);
    flush_input();
    out.println("\nReactive steering test done.");
}

// ─── Test: IMU (MPU-6050 Gyro Z) ─────────────────────────────────────────────
#if USE_IMU
void test_imu(unsigned long duration_ms = 0) {
    out.println("\n=== IMU Test (MPU-6050 Gyro Z) ===");

    // Init I2C
    Wire.setSDA(IMU_SDA_PIN);
    Wire.setSCL(IMU_SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);

    // Check WHO_AM_I (reg 0x75, expect 0x68)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x75);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    if (Wire.available() < 1) {
        out.println("ERROR: MPU-6050 not found on I2C bus!");
        out.print("  SDA=GP"); out.print(IMU_SDA_PIN);
        out.print("  SCL=GP"); out.println(IMU_SCL_PIN);
        return;
    }
    uint8_t who = Wire.read();
    out.print("WHO_AM_I: 0x"); out.print(who, HEX);
    if      (who == 0x68) out.println("  (MPU-6050)");
    else if (who == 0x70) out.println("  (MPU-6500)");
    else if (who == 0x71) out.println("  (MPU-9250)");
    else if (who == 0x73) out.println("  (MPU-9255)");
    else if (who == 0xE5) { out.println("  ADXL345 — not a gyro!"); return; }
    else { out.print("  (unknown 0x"); out.print(who, HEX); out.println(")"); }

    // Wake from sleep
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) {
        out.println("ERROR: Failed to wake MPU-6050!");
        return;
    }

    // Gyro full-scale ±500°/s
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);  // GYRO_CONFIG
    Wire.write(GYRO_FS_500);
    Wire.endTransmission();

    // DLPF ~42 Hz
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1A);  // CONFIG
    Wire.write(0x03);
    Wire.endTransmission();

    delay(100);  // let DLPF settle

    // Bias calibration — average gyro Z at rest for 2 seconds
    out.println("Calibrating gyro bias (2 s, keep car still)...");
    float bias_sum = 0;
    int bias_n = 0;
    unsigned long cal_start = millis();

    while (millis() - cal_start < 2000) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x47);  // GYRO_ZOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
        if (Wire.available() >= 2) {
            int16_t raw_z = ((int16_t)Wire.read() << 8) | Wire.read();
            bias_sum += raw_z / GYRO_SENS;
            bias_n++;
        }
        delay(10);
    }

    float bias = (bias_n > 0) ? bias_sum / bias_n : 0.0f;
    out.print("Gyro Z bias: "); out.print(bias, 3); out.print(" deg/s");
    out.print("  ("); out.print(bias_n); out.println(" samples)");

    // Live stream
    out.println("\nStreaming gyro Z. Press any key to stop.\n");
    out.println("  Raw      | Yaw Rate  | Heading   | Status");
    out.println("  ---------|-----------|-----------|--------");

    float heading = 0.0f;
    unsigned long prev_us = micros();

    unsigned long start = millis();
    while (!input_available()) {
        if (duration_ms && (millis() - start) > duration_ms) break;

        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x47);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
        if (Wire.available() < 2) { delay(10); continue; }

        int16_t raw_z = ((int16_t)Wire.read() << 8) | Wire.read();
        float yaw_rate = raw_z / GYRO_SENS - bias;

        unsigned long now_us = micros();
        float dt = (now_us - prev_us) / 1e6f;
        prev_us = now_us;
        if (dt > 0.0f && dt < 0.5f)
            heading += yaw_rate * dt;

        bool moving = (yaw_rate > 1.0f || yaw_rate < -1.0f);

        out.print("  ");
        out.print(raw_z);
        out.print("\t| ");
        out.print(yaw_rate, 2);
        out.print(" d/s\t| ");
        out.print(heading, 1);
        out.print(" deg\t| ");
        out.println(moving ? "turning" : "still  ");

        delay(50);
    }

    flush_input();
    out.print("\nFinal heading: "); out.print(heading, 1); out.println(" deg");
    out.println("IMU test done.");
}
#endif

// ─── All tests ───────────────────────────────────────────────────────────────
void run_all() {
    test_lidar(4000);
    test_servo();
    test_taho(5000);
#if USE_IMU
    test_imu(5000);
#endif
    // ESC skipped in auto-run — requires manual confirmation
    out.println("\nAll automatic tests done. Run 'e' for ESC test.");
}

// ─── WiFi protocol command handling ──────────────────────────────────────────
#if USE_WIFI_DEBUG

// Manual drive state
static bool drv_enabled = false;
static int drv_steer = 0;
static float drv_speed = 0.0f;
static unsigned long drv_last_ms = 0;

// Telemetry timing
static unsigned long telem_next_ms = 0;
#define TELEM_INTERVAL_MS  40   // 25 Hz

// IMU state for telemetry
#if USE_IMU
static bool imu_ok = false;
static float imu_bias = 0.0f;
static float imu_yaw_rate = 0.0f;
static float imu_heading = 0.0f;
static unsigned long imu_prev_us = 0;
#endif

// WiFi command line buffer (separate from interactive serial)
static char wifi_cmd[64];
static int wifi_cmd_len = 0;

static void wifi_send_telemetry() {
    int s0 = ld[0].ok ? ld[0].cm * 10 : 0;
    int s1 = ld[1].ok ? ld[1].cm * 10 : 0;
    int s2 = ld[2].ok ? ld[2].cm * 10 : 0;
    int s3 = ld[3].ok ? ld[3].cm * 10 : 0;

    Serial1.print(millis());       Serial1.print(',');
    Serial1.print(s0);            Serial1.print(',');
    Serial1.print(s1);            Serial1.print(',');
    Serial1.print(s2);            Serial1.print(',');
    Serial1.print(s3);            Serial1.print(',');
    Serial1.print(drv_steer);     Serial1.print(',');
    Serial1.print(get_speed(), 2); Serial1.print(',');
    Serial1.print(drv_speed, 1);
#if USE_IMU
    if (imu_ok) {
        Serial1.print(',');
        Serial1.print(imu_yaw_rate, 1); Serial1.print(',');
        Serial1.print(imu_heading, 1);
    }
#endif
    Serial1.println();
}

#if USE_IMU
static void imu_quick_init() {
    Wire.setSDA(IMU_SDA_PIN);
    Wire.setSCL(IMU_SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);

    // Check WHO_AM_I
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x75);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    if (Wire.available() < 1) { imu_ok = false; return; }
    uint8_t who = Wire.read();
    if (who == 0xE5) { imu_ok = false; return; }   // ADXL345 — no gyro

    // Wake + configure gyro ±500°/s + DLPF 42 Hz
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1B); Wire.write(GYRO_FS_500); Wire.endTransmission();
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();
    delay(100);

    // Quick bias calibration (1 s)
    float bias_sum = 0;
    int n = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 1000) {
        Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x47); Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
        if (Wire.available() >= 2) {
            int16_t raw = ((int16_t)Wire.read() << 8) | Wire.read();
            bias_sum += raw / GYRO_SENS;
            n++;
        }
        delay(5);
    }
    imu_bias = (n > 0) ? bias_sum / n : 0.0f;
    imu_prev_us = micros();
    imu_ok = true;
}

static void imu_update() {
    if (!imu_ok) return;
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x47); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return;
    int16_t raw = ((int16_t)Wire.read() << 8) | Wire.read();
    imu_yaw_rate = raw / GYRO_SENS - imu_bias;
    unsigned long now_us = micros();
    float dt = (now_us - imu_prev_us) / 1e6f;
    imu_prev_us = now_us;
    if (dt > 0.0f && dt < 0.5f) imu_heading += imu_yaw_rate * dt;
}
#endif

static void wifi_handle_drv(const char* args) {
    char buf[32];
    strncpy(buf, args, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char* comma = strchr(buf, ',');
    if (!comma) return;
    *comma = '\0';
    drv_steer = atoi(buf);
    drv_speed = atof(comma + 1);
    drv_last_ms = millis();

    if (!drv_enabled) return;   // ignore if not enabled

    // Map steer -1000..+1000 → servo angle
    int angle = SERVO_NEUTRAL + (int)((long)drv_steer * (SERVO_RIGHT - SERVO_LEFT) / 2000);
    angle = constrain(angle, SERVO_LEFT, SERVO_RIGHT);
    steer.write(angle);

    // Map speed (m/s) → ESC angle
    if (drv_speed < 0.05f) {
        esc.write(ESC_NEUTRAL);
    } else {
        int esc_val = ESC_FWD_MIN + (int)((drv_speed / 3.0f) * (ESC_FWD_MAX - ESC_FWD_MIN));
        esc_val = constrain(esc_val, ESC_FWD_MIN, ESC_FWD_MAX);
        esc.write(esc_val);
    }
}

static void wifi_process_line(const char* line) {
    if (strcmp(line, "$PING")   == 0) { Serial1.println("$PONG"); }
    else if (strcmp(line, "$STATUS") == 0) { Serial1.println("$STS:STOP"); }
    else if (strcmp(line, "$STOP")  == 0) {
        drv_enabled = false;
        drv_steer = 0;
        drv_speed = 0.0f;
        steer.write(SERVO_NEUTRAL);
        esc.write(ESC_NEUTRAL);
        Serial1.println("$ACK");
        Serial1.println("$STS:STOP");
    }
    else if (strcmp(line, "$START") == 0) {
        Serial1.println("$ACK");
        Serial1.println("$STS:RUN");
    }
    else if (strcmp(line, "$DRVEN")  == 0) {
        drv_enabled = true;
        Serial1.println("$ACK");
    }
    else if (strcmp(line, "$DRVOFF") == 0) {
        drv_enabled = false;
        drv_steer = 0;
        drv_speed = 0.0f;
        steer.write(SERVO_NEUTRAL);
        esc.write(ESC_NEUTRAL);
        Serial1.println("$ACK");
    }
    else if (strncmp(line, "$DRV:", 5) == 0) { wifi_handle_drv(line + 5); }
    else if (strncmp(line, "$TEST:", 6) == 0) {
        // Forward test name as single-char command to interactive handler
        const char* name = line + 6;
        if      (strcmp(name, "lidar")    == 0) test_lidar(5000);
        else if (strcmp(name, "servo")    == 0) test_servo();
        else if (strcmp(name, "taho")     == 0) test_taho(5000);
        else if (strcmp(name, "esc")      == 0) test_esc();
        else if (strcmp(name, "speed")    == 0) test_speed_hold(10000);
        else if (strcmp(name, "autotune") == 0) test_autotune();
        else if (strcmp(name, "reactive") == 0) test_reactive(10000);
        else { Serial1.print("$NAK:unknown_test:"); Serial1.println(name); }
    }
}

static void wifi_process_commands() {
    while (Serial1.available() > 0) {
        char c = (char)Serial1.read();
        if (c == '\n' || c == '\r') {
            if (wifi_cmd_len > 0) {
                wifi_cmd[wifi_cmd_len] = '\0';
                wifi_process_line(wifi_cmd);
                wifi_cmd_len = 0;
            }
        } else if (wifi_cmd_len < (int)sizeof(wifi_cmd) - 1) {
            wifi_cmd[wifi_cmd_len++] = c;
        }
    }
}

#endif  // USE_WIFI_DEBUG

// ─── Setup / loop ─────────────────────────────────────────────────────────────
void setup() {
    // USB CDC needs time to enumerate before other peripherals init
    Serial.begin(115200);
    delay(2000);

#if USE_WIFI_DEBUG
    Serial1.setTX(WIFI_TX_PIN);
    Serial1.setRX(WIFI_RX_PIN);
    Serial1.begin(115200);
    delay(500);
    flush_input();
#endif

    steer.attach(SERVO_PIN);
    esc.attach(MOTOR_PIN);
    steer.write(SERVO_NEUTRAL);
    esc.write(ESC_NEUTRAL);

    for (int i = 0; i < 4; i++) ls[i]->begin(LIDAR_BAUD);

    pinMode(TAHO_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(TAHO_PIN), taho_isr, RISING);

#if USE_WIFI_DEBUG && USE_IMU
    imu_quick_init();
#endif

    out.println("\n================================");
    out.println("  Umbreon -- Hardware Test");
    out.println("  Output: USB");
#if USE_WIFI_DEBUG
    out.print("        + WiFi (GP");
    out.print(WIFI_TX_PIN);
    out.print("/GP");
    out.print(WIFI_RX_PIN);
    out.println(")");
#endif
#if USE_IMU
    out.println("  IMU:   MPU-6050 enabled");
#else
    out.println("  IMU:   disabled");
#endif
    out.println("================================");
    print_help();
}

void loop() {
    poll_all_lidars();

#if USE_WIFI_DEBUG
    // Process WiFi protocol commands ($PING, $DRV, $TEST, etc.)
    wifi_process_commands();

    // Manual drive timeout — stop motors if no $DRV for 500 ms
    if (drv_enabled && drv_speed > 0.01f && millis() - drv_last_ms > 500) {
        drv_speed = 0.0f;
        drv_steer = 0;
        esc.write(ESC_NEUTRAL);
        steer.write(SERVO_NEUTRAL);
    }

#if USE_IMU
    imu_update();
#endif

    // Send telemetry to web dashboard at 25 Hz
    if (millis() >= telem_next_ms) {
        telem_next_ms = millis() + TELEM_INTERVAL_MS;
        wifi_send_telemetry();
    }
#endif

    // Interactive serial commands (USB or WiFi single-char)
    if (Serial.available()) {
        char cmd = (char)Serial.read();
        while (Serial.available()) Serial.read();

        switch (cmd) {
            case 'l': test_lidar();       break;
            case 's': test_servo();       break;
            case 't': test_taho();        break;
            case 'e': test_esc();         break;
            case 'p': test_speed_hold();  break;
            case 'u': test_autotune();    break;
            case 'r': test_reactive();    break;
#if USE_IMU
            case 'i': test_imu();        break;
#endif
            case 'a': run_all();          break;
            case '?': print_help();       break;
            default:
                out.print("Unknown command '");
                out.print(cmd);
                out.println("'. Send ? for help.");
        }
    }
}
