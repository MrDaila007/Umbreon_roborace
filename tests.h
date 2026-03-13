#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// tests.h — diagnostic and calibration routines for Umbreon
// Include this file and call functions from setup() or loop() while testing.
// Comment out the #include in the main sketch for competition builds.
//
// Quick-start: add to setup():   test_menu_init();
//              add to loop():    test_menu_tick(car);
// Then open Serial Monitor at 115200 and send command letters.
// ─────────────────────────────────────────────────────────────────────────────

// ─── Helpers ────────────────────────────────────────────────────────────────
static void _flush_serial() { while (Serial.available()) Serial.read(); }

static char _wait_key(Car& car) {
    while (!Serial.available()) { car.poll_lidars(); delay(1); }
    char c = Serial.read();
    _flush_serial();
    return c;
}

// ─── LiDAR diagnostics ─────────────────────────────────────────────────────

// Print all 4 LiDAR distances to Serial (cm×10 units).
void print_sensors(Car& car) {
    car.poll_lidars();
    int* s = car.read_sensors();
    const char* labels[4] = {"Left    ", "FrontL  ", "FrontR  ", "Right   "};
    for (int i = 0; i < 4; i++) {
        Serial.print(labels[i]);
        Serial.print(": ");
        if (s[i] == 9999) {
            Serial.println("  --  (no reading)");
        } else {
            Serial.print(s[i] / 10);
            Serial.println(" cm");
        }
    }
    Serial.println("--------------------");
}

// Stream all 4 distances as a tab-separated line (for Serial Plotter).
void plot_sensors(Car& car) {
    car.poll_lidars();
    int* s = car.read_sensors();
    for (int i = 0; i < 4; i++) {
        Serial.print(s[i] == 9999 ? 0 : s[i] / 10);
        if (i < 3) Serial.print('\t');
    }
    Serial.println();
}

// Continuous LiDAR stream with VT100 display. Press any key to stop.
void test_lidar(Car& car, unsigned long duration_ms = 0) {
    Serial.println("\n=== LiDAR Test ===");
    Serial.println("Press any key to stop.\n");
    Serial.println("Sensor    | Distance  | Status");
    Serial.println("----------|-----------|--------");

    const char* names[4] = {"Left    ", "FrontL  ", "FrontR  ", "Right   "};
    unsigned long start = millis();

    while (!Serial.available()) {
        if (duration_ms && (millis() - start) > duration_ms) break;
        car.poll_lidars();
        int* s = car.read_sensors();

        Serial.print("\x1b[4A");
        for (int i = 0; i < 4; i++) {
            Serial.print(names[i]);
            Serial.print(" | ");
            if (s[i] != 9999) {
                Serial.print(s[i] / 10);
                Serial.print(" cm      | OK    ");
            } else {
                Serial.print("--         | no data");
            }
            Serial.println();
        }
        delay(100);
    }
    _flush_serial();
    Serial.println("\nLiDAR test done.");
}

// ─── Steering tests ─────────────────────────────────────────────────────────

// Sweep steering left → right → centre.
void wiggle(Car& car) {
    for (int i = -100; i <= 100; i++) {
        car.write_steer(i * 10);
        delay(5);
    }
    for (int i = 100; i >= -100; i--) {
        car.write_steer(i * 10);
        delay(5);
    }
    car.write_steer(0);
}

void turn_left(Car& car)    { car.write_steer(-1000); }
void turn_right(Car& car)   { car.write_steer( 1000); }
void turn_centre(Car& car)  { car.write_steer(    0); }

// Full servo test: snap left/right, then slow sweep.
void test_servo(Car& car) {
    Serial.println("\n=== Servo Test ===");
    Serial.println("Sweeping LEFT...");  car.write_steer(-1000); delay(800);
    Serial.println("Sweeping RIGHT..."); car.write_steer( 1000); delay(800);
    Serial.println("Centre.");           car.write_steer(    0); delay(400);

    Serial.println("Slow sweep left->right->centre:");
    for (int i = -100; i <= 100; i++) { car.write_steer(i * 10); delay(8); }
    for (int i = 100; i >= -100; i--) { car.write_steer(i * 10); delay(8); }
    car.write_steer(0);
    Serial.println("Servo test done.");
}

// ─── Tachometer tests ───────────────────────────────────────────────────────

// Stream real-time speed (m/s) to Serial Plotter.
void plot_speed() {
    Serial.println(get_speed(), 3);
}

// Print speed and raw encoder period to Serial.
void print_speed() {
    Serial.print("Speed: ");
    Serial.print(get_speed(), 3);
    Serial.print(" m/s  |  pulse period: ");
    Serial.print(_taho_iv);
    Serial.println(" us");
}

// Live tachometer monitor. Spin wheel by hand. Press any key to stop.
void test_taho(Car& car, unsigned long duration_ms = 0) {
    Serial.println("\n=== Tachometer Test ===");
    Serial.println("Spin the wheel by hand. Press any key to stop.\n");

    noInterrupts();
    _taho_count = 0;
    _taho_last  = micros();
    _taho_iv    = 0;
    interrupts();

    unsigned long start = millis();
    while (!Serial.available()) {
        if (duration_ms && (millis() - start) > duration_ms) break;
        car.poll_lidars();

        noInterrupts();
        unsigned long cnt = _taho_count;
        unsigned long iv  = _taho_iv;
        interrupts();

        bool stopped = (micros() - _taho_last) > 500000UL;
        float speed_ms = stopped ? 0.0f : get_speed();

        Serial.print("Pulses: "); Serial.print(cnt);
        Serial.print("   interval: "); Serial.print(iv); Serial.print(" us");
        Serial.print("   speed: "); Serial.print(speed_ms, 2); Serial.print(" m/s");
        Serial.print(stopped ? "   [STOPPED]" : "   [spinning]");
        Serial.println();
        delay(150);
    }
    _flush_serial();
    Serial.println("Tachometer test done.");
}

// ─── Speed / ESC tests ──────────────────────────────────────────────────────

// Ramp ESC from 0 to full forward and back, then full reverse and back.
// WARNING: car will move — have it on a stand or in a clear area.
void max_speed_test(Car& car) {
    delay(1500);
    for (int i = 0; i <= 100; i++) { car.write_speed(i * 10);  delay(20); }
    delay(300);
    for (int i = 100; i >= 0; i--) { car.write_speed(i * 10);  delay(10); }
    delay(1500);
    for (int i = 0; i <= 100; i++) { car.write_speed(-i * 10); delay(10); }
    delay(300);
    for (int i = 100; i >= 0; i--) { car.write_speed(-i * 10); delay(20); }
    car.write_speed(0);
}

// Gentle ramp for ESC range calibration (30 % of max_speed_test).
void small_speed_test(Car& car) {
    delay(1500);
    for (int i = 0; i <= 100; i++) { car.write_speed(i * 3);  delay(20); }
    delay(300);
    for (int i = 100; i >= 0; i--) { car.write_speed(i * 3);  delay(20); }
    delay(1500);
    for (int i = 0; i <= 100; i++) { car.write_speed(-i * 3); delay(20); }
    delay(300);
    for (int i = 100; i >= 0; i--) { car.write_speed(-i * 3); delay(20); }
    car.write_speed(0);
}

// ESC arm test — brief forward spin with tachometer readout.
void test_esc(Car& car) {
    Serial.println("\n=== ESC Test ===");
    Serial.println("!! WHEEL WILL SPIN FOR ~2 SECONDS !!");
    Serial.println("Confirm by sending 'y', anything else cancels.");

    char c = _wait_key(car);
    if (c != 'y' && c != 'Y') { Serial.println("Cancelled."); return; }

    Serial.println("Arming ESC (neutral 2 s)...");
    car.write_speed(0);
    delay(2000);

    noInterrupts();
    _taho_count = 0;
    _taho_last  = micros();
    _taho_iv    = 0;
    interrupts();

    Serial.println("Minimal forward pulse (2 s)...");
    // cfg_min_speed is the lowest forward ESC value — write it directly
    car.motor_esc.write(cfg_min_speed);

    unsigned long esc_start = millis();
    while (millis() - esc_start < 2000) {
        car.poll_lidars();

        noInterrupts();
        unsigned long cnt  = _taho_count;
        interrupts();

        bool stopped = (micros() - _taho_last) > 500000UL;
        float speed_ms = stopped ? 0.0f : get_speed();
        float revs = (float)cnt / (float)cfg_encoder_holes;

        Serial.print("\x1b[1A");
        Serial.print("Pulses: "); Serial.print(cnt);
        Serial.print("  revs: "); Serial.print(revs, 1);
        Serial.print("  speed: "); Serial.print(speed_ms, 2); Serial.print(" m/s");
        Serial.print(stopped ? "  [STOPPED]  " : "  [spinning] ");
        Serial.println();
        delay(100);
    }

    Serial.println("Stopping.");
    car.write_speed(0);
    delay(500);

    noInterrupts();
    unsigned long final_cnt = _taho_count;
    interrupts();

    Serial.print("Total pulses: "); Serial.print(final_cnt);
    Serial.print("  ("); Serial.print((float)final_cnt / cfg_encoder_holes, 1);
    Serial.println(" revolutions)");
    Serial.println("ESC test done.");
}

// Drive at a fixed PID target and log actual speed every 100 ms.
// Runs for `duration_ms` milliseconds then stops.
void speed_step_test(Car& car, float target_ms, unsigned long duration_ms) {
    car.write_steer(0);
    car.write_speed_ms(target_ms);
    unsigned long end = millis() + duration_ms;
    while (millis() < end) {
        car.poll_lidars();
        car.pid_control_motor();
        Serial.print("tgt:");  Serial.print(target_ms, 2);
        Serial.print("\tact:"); Serial.println(get_speed(), 3);
        delay(100);
    }
    car.write_speed(0);
    car.write_speed_ms(0);
}

// Interactive speed hold — +/- to change target, any other key stops.
void test_speed_hold(Car& car) {
    Serial.println("\n=== Speed Hold Test ===");
    Serial.println("!! WHEEL WILL SPIN !!");
    Serial.println("Confirm by sending 'y', anything else cancels.");

    char c = _wait_key(car);
    if (c != 'y' && c != 'Y') { Serial.println("Cancelled."); return; }

    float target = 1.5f;

    Serial.println("Arming ESC (neutral 2 s)...");
    car.write_speed(0);
    delay(2000);

    Serial.println("Starting PID loop.  +/- change target,  any other key = stop.\n");
    Serial.println("CSV:ms,target,raw,filtered,esc,error,integral,deriv,pulses");

    // Reset PID state
    car.pid_integral   = 0;
    car.pid_prev_error = 0;
    car.pid_filtered   = 0;
    car.pid_prev_cnt   = 0;
    car.pid_prev_ms    = 0;

    noInterrupts();
    _taho_count = 0;
    _taho_last  = micros();
    _taho_iv    = 0;
    interrupts();

    car.write_speed_ms(target);
    unsigned long prev_ms = millis();

    while (true) {
        car.poll_lidars();

        if (Serial.available()) {
            char ch = Serial.read();
            if (ch == '+' || ch == '=') {
                target += 0.1f;
                car.write_speed_ms(target);
                Serial.print("\n>> Target -> "); Serial.print(target, 1); Serial.println(" m/s");
            } else if (ch == '-' || ch == '_') {
                target -= 0.1f;
                if (target < 0) target = 0;
                car.write_speed_ms(target);
                Serial.print("\n>> Target -> "); Serial.print(target, 1); Serial.println(" m/s");
            } else {
                break;
            }
            _flush_serial();
        }

        unsigned long now_ms = millis();
        if (now_ms - prev_ms < 80) continue;
        prev_ms = now_ms;

        car.pid_control_motor();

        // CSV telemetry
        noInterrupts();
        unsigned long cnt = _taho_count;
        interrupts();

        Serial.print("D,");
        Serial.print(now_ms); Serial.print(",");
        Serial.print(target, 2); Serial.print(",");
        Serial.print(get_speed(), 2); Serial.print(",");
        Serial.print(car.pid_filtered, 2); Serial.print(",");
        Serial.print(car.pid_integral, 3); Serial.print(",");
        Serial.print(car.pid_prev_error, 3); Serial.print(",");
        Serial.println(cnt);
    }

    car.write_speed(0);
    car.write_speed_ms(0);
    _flush_serial();

    noInterrupts();
    unsigned long final_cnt = _taho_count;
    interrupts();

    Serial.print("\nStopped. Total pulses: "); Serial.print(final_cnt);
    Serial.print("  ("); Serial.print((float)final_cnt / cfg_encoder_holes, 1);
    Serial.println(" revolutions)");
    Serial.println("Speed hold test done.");
}

// ─── PID Auto-Tune (Relay / Astrom-Hagglund) ────────────────────────────────
void test_autotune(Car& car) {
    Serial.println("\n=== PID Auto-Tuner (Relay Method) ===");
    Serial.println("!! WHEEL WILL SPIN for ~30s !!");
    Serial.println("Confirm 'y', anything else cancels.");

    char c = _wait_key(car);
    if (c != 'y' && c != 'Y') { Serial.println("Cancelled."); return; }

    const float TARGET     = 1.5f;
    const int   RELAY_D    = 2;
    const float HYST       = 0.10f;
    const int   BASE_ESC   = cfg_min_speed + 2;
    const int   SKIP_HALF  = 4;
    const int   NEED_HALF  = 12;
    const unsigned long TIMEOUT = 40000;

    Serial.println("Arming ESC (2s)...");
    car.write_speed(0);
    delay(2000);

    noInterrupts();
    _taho_count = 0; _taho_last = micros(); _taho_iv = 0;
    interrupts();

    Serial.println("Phase 1: Relay oscillation...");
    Serial.println("CSV:ms,target,raw,filtered,esc,relay");

    bool relay_high = true;
    float filtered = 0;
    float speed_peak = 0, speed_trough = 999.0f;
    unsigned long prev_cnt = 0, prev_ms = millis(), start_ms = prev_ms;
    int half_cycle = 0;

    const int MAXM = 16;
    float meas_peaks[MAXM], meas_troughs[MAXM];
    unsigned long sw_times[MAXM * 2];
    int np = 0, nt = 0, nsw = 0;

    int esc_val = constrain(BASE_ESC + RELAY_D, NEUTRAL_SPEED, cfg_max_speed);
    car.motor_esc.write(esc_val);

    while (millis() - start_ms < TIMEOUT) {
        car.poll_lidars();
        if (Serial.available()) { _flush_serial(); break; }

        unsigned long now = millis();
        if (now - prev_ms < 80) continue;
        float dt = (now - prev_ms) / 1000.0f;
        prev_ms = now;

        noInterrupts();
        unsigned long cnt  = _taho_count;
        unsigned long last = _taho_last;
        interrupts();

        unsigned long dc = cnt - prev_cnt;
        prev_cnt = cnt;
        float raw = (dc / (float)cfg_encoder_holes) *
                    (3.14159265f * cfg_wheel_diam_m) / dt;
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
        esc_val = constrain(esc_val, NEUTRAL_SPEED, cfg_max_speed);
        car.motor_esc.write(esc_val);

        Serial.print("D,"); Serial.print(now);
        Serial.print(","); Serial.print(TARGET, 2);
        Serial.print(","); Serial.print(raw, 2);
        Serial.print(","); Serial.print(filtered, 2);
        Serial.print(","); Serial.print(esc_val);
        Serial.print(","); Serial.println(relay_high ? 1 : 0);

        if (half_cycle >= SKIP_HALF + NEED_HALF) break;
    }

    car.write_speed(0);
    delay(500);

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

    float sum_period = 0;
    int n_periods = 0;
    for (int i = 0; i + 2 < nsw; i++) {
        sum_period += (sw_times[i + 2] - sw_times[i]) / 1000.0f;
        n_periods++;
    }
    float Tu = (n_periods > 0) ? sum_period / n_periods : 1.0f;
    float Ku = 4.0f * RELAY_D / (3.14159f * amplitude);

    // Ziegler-Nichols
    float zn_kP = 0.6f * Ku;
    float zn_kI = zn_kP / (0.5f * Tu);
    float zn_kD = zn_kP * Tu / 8.0f;

    // Tyreus-Luyben (conservative)
    float tl_kP = Ku / 2.2f;
    float tl_kI = tl_kP / (2.2f * Tu);
    float tl_kD = tl_kP * Tu / 6.3f;

    // PI only (safest for noisy encoder)
    float pi_kP = 0.45f * Ku;
    float pi_kI = pi_kP / (0.83f * Tu);

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
    Serial.print("R,ZN,");  Serial.print(zn_kP,2); Serial.print(","); Serial.print(zn_kI,2); Serial.print(","); Serial.println(zn_kD,3);
    Serial.print("R,TL,");  Serial.print(tl_kP,2); Serial.print(","); Serial.print(tl_kI,2); Serial.print(","); Serial.println(tl_kD,3);
    Serial.print("R,PI,");  Serial.print(pi_kP,2); Serial.print(","); Serial.print(pi_kI,2); Serial.println(",0.000");

    Serial.println("===================================");

    // Phase 2: Verify with Tyreus-Luyben gains (10s)
    Serial.println("\nPhase 2: Verifying with Tyreus-Luyben gains (10s)...");
    Serial.println("CSV:ms,target,filtered,esc,pulses");

    // Temporarily apply auto-tuned gains
    float saved_kp = cfg_pid_kp, saved_ki = cfg_pid_ki, saved_kd = cfg_pid_kd;
    cfg_pid_kp = tl_kP;
    cfg_pid_ki = tl_kI;
    cfg_pid_kd = tl_kD;

    car.pid_integral   = 0;
    car.pid_prev_error = 0;
    car.pid_filtered   = 0;
    car.pid_prev_cnt   = 0;
    car.pid_prev_ms    = 0;

    car.write_speed(0);
    delay(2000);

    noInterrupts();
    _taho_count = 0; _taho_last = micros(); _taho_iv = 0;
    interrupts();

    car.write_speed_ms(TARGET);
    prev_ms = millis();
    unsigned long verify_start = millis();

    while (millis() - verify_start < 10000) {
        car.poll_lidars();
        if (Serial.available()) { _flush_serial(); break; }

        unsigned long now = millis();
        if (now - prev_ms < 80) continue;
        prev_ms = now;

        car.pid_control_motor();

        noInterrupts();
        unsigned long vcnt = _taho_count;
        interrupts();

        Serial.print("D,"); Serial.print(now);
        Serial.print(","); Serial.print(TARGET, 2);
        Serial.print(","); Serial.print(car.pid_filtered, 2);
        Serial.print(","); Serial.println(vcnt);
    }

    car.write_speed(0);
    car.write_speed_ms(0);

    // Restore original gains
    cfg_pid_kp = saved_kp;
    cfg_pid_ki = saved_ki;
    cfg_pid_kd = saved_kd;

    noInterrupts();
    unsigned long final_cnt = _taho_count;
    interrupts();

    Serial.print("\nVerify done. Pulses: "); Serial.print(final_cnt);
    Serial.print("  ("); Serial.print((float)final_cnt / cfg_encoder_holes, 1);
    Serial.println(" revs)");
    Serial.println("Auto-tune complete.");
}

// ─── Reactive Steering (LiDAR → Servo) ──────────────────────────────────────
void test_reactive(Car& car) {
    Serial.println("\n=== Reactive Steering Test ===");
    Serial.println("LiDAR controls servo in real-time. Press any key to stop.\n");
    Serial.println("  Left(L)  FrontL(FL)  FrontR(FR)  Right(R)  -> Steer");
    Serial.println("  -------------------------------------------------------");

    const int CLOSE_DIST = 1200;   // cm×10
    const int FAR_DIST   = 3000;   // cm×10

    while (!Serial.available()) {
        car.poll_lidars();
        int* s = car.read_sensors();

        int L  = (s[0] != 9999) ? s[0] : 9999;
        int FL = (s[1] != 9999) ? s[1] : 9999;
        int FR = (s[2] != 9999) ? s[2] : 9999;
        int R  = (s[3] != 9999) ? s[3] : 9999;

        // Balance left vs right
        float diff = (float)(R - L);

        // Front sensors: steer away from blocked side
        if (FL < CLOSE_DIST) diff += (float)(CLOSE_DIST - FL);
        if (FR < CLOSE_DIST) diff -= (float)(CLOSE_DIST - FR);

        // Normalize to -1000..+1000 for Car::write_steer
        float steer_f = diff / (float)FAR_DIST;
        steer_f = constrain(steer_f, -1.0f, 1.0f);
        int steer_val = (int)(steer_f * 1000.0f);
        car.write_steer(steer_val);

        Serial.print("\x1b[1A");
        Serial.print("  L="); Serial.print(L / 10);
        Serial.print("  FL="); Serial.print(FL / 10);
        Serial.print("  FR="); Serial.print(FR / 10);
        Serial.print("  R="); Serial.print(R / 10);
        Serial.print("  -> steer="); Serial.print(steer_val);
        if (steer_f < -0.2f)      Serial.print("  <-LEFT   ");
        else if (steer_f > 0.2f)  Serial.print("  RIGHT->  ");
        else                       Serial.print("  CENTER   ");
        Serial.println();

        delay(50);
    }

    car.write_steer(0);
    _flush_serial();
    Serial.println("\nReactive steering test done.");
}

// ─── Run all non-destructive tests ──────────────────────────────────────────
void test_run_all(Car& car) {
    test_lidar(car, 4000);
    test_servo(car);
    test_taho(car, 5000);
    Serial.println("\nAll automatic tests done. Run 'e' for ESC test.");
}

// ─── Interactive test menu ──────────────────────────────────────────────────
void test_menu_help() {
    Serial.println("\nCommands:");
    Serial.println("  l  LiDAR stream");
    Serial.println("  s  Servo sweep");
    Serial.println("  t  Tachometer live");
    Serial.println("  e  ESC arm test  (car will move briefly!)");
    Serial.println("  p  Speed hold (PID, +/- to change target)");
    Serial.println("  u  PID auto-tune (relay method, ~30s)");
    Serial.println("  r  Reactive steering (LiDAR -> servo)");
    Serial.println("  a  Run all tests in sequence");
    Serial.println("  ?  This help\n");
}

void test_menu_init() {
    Serial.println("\n============================");
    Serial.println("  Umbreon — Hardware Test");
    Serial.println("============================");
    test_menu_help();
}

void test_menu_tick(Car& car) {
    car.poll_lidars();
    if (!Serial.available()) return;

    char cmd = Serial.read();
    _flush_serial();

    switch (cmd) {
        case 'l': test_lidar(car);       break;
        case 's': test_servo(car);       break;
        case 't': test_taho(car);        break;
        case 'e': test_esc(car);         break;
        case 'p': test_speed_hold(car);  break;
        case 'u': test_autotune(car);    break;
        case 'r': test_reactive(car);    break;
        case 'a': test_run_all(car);     break;
        case '?': test_menu_help();      break;
        default:
            Serial.print("Unknown command '");
            Serial.print(cmd);
            Serial.println("'. Send ? for help.");
    }
}
