#pragma once
// ═════════════════════════════════════════════════════════════════════════════
// wifi_tests.h — WiFi-accessible hardware test routines
//
// Each test streams $T: progress lines, optional $TR: results, and ends with
// $TDONE:<name>.  Tests auto-stop the car.  Send $STOP to abort a running test.
//
// Depends on: car (Car instance), telem, _taho_count/_taho_iv/_taho_last,
//             get_speed(), cfg_* globals, HAS_TELEM
// ═════════════════════════════════════════════════════════════════════════════

#if HAS_TELEM

static bool wifi_check_abort() {
    if (telem.available() > 0 && telem.peek() == '$')
        return true;
    return false;
}

// ─── LiDAR test ──────────────────────────────────────────────────────────────
static void wifi_test_lidar() {
    unsigned long start = millis();
    while ((millis() - start) < 5000) {
        if (wifi_check_abort()) break;
        car.poll_lidars();
        int* s = car.read_sensors();
        telem.print("$T:LIDAR,L="); telem.print(s[0]);
        telem.print(",FL=");        telem.print(s[1]);
        telem.print(",FR=");        telem.print(s[2]);
        telem.print(",R=");         telem.print(s[3]);
#if PLATFORM_ESP32S3
        telem.print(",F=");         telem.print(s[4]);
        telem.print(",RE=");        telem.print(s[5]);
#endif
        telem.println();
        delay(100);
    }
    telem.println("$TDONE:lidar");
}

// ─── Servo test ──────────────────────────────────────────────────────────────
static void wifi_test_servo() {
    telem.println("$T:SERVO,phase=left");
    car.write_steer(-1000); delay(800);
    if (wifi_check_abort()) { car.write_steer(0); telem.println("$TDONE:servo"); return; }

    telem.println("$T:SERVO,phase=right");
    car.write_steer(1000); delay(800);
    if (wifi_check_abort()) { car.write_steer(0); telem.println("$TDONE:servo"); return; }

    telem.println("$T:SERVO,phase=center");
    car.write_steer(0); delay(400);

    telem.println("$T:SERVO,phase=sweep");
    for (int i = -100; i <= 100; i++) {
        car.write_steer(i * 10);
        delay(8);
        if (wifi_check_abort()) break;
    }
    for (int i = 100; i >= -100; i--) {
        car.write_steer(i * 10);
        delay(8);
        if (wifi_check_abort()) break;
    }
    car.write_steer(0);
    telem.println("$TDONE:servo");
}

// ─── Tachometer test ─────────────────────────────────────────────────────────
static void wifi_test_taho() {
    noInterrupts();
    _taho_count = 0;
    _taho_last  = micros();
    _taho_iv    = 0;
    interrupts();

    unsigned long start = millis();
    while ((millis() - start) < 5000) {
        if (wifi_check_abort()) break;
        car.poll_lidars();

        noInterrupts();
        unsigned long cnt = _taho_count;
        unsigned long iv  = _taho_iv;
        interrupts();

        bool stopped = (micros() - _taho_last) > 500000UL;
        float speed_ms = stopped ? 0.0f : get_speed();

        telem.print("$T:TAHO,pulses=");   telem.print(cnt);
        telem.print(",interval=");         telem.print(iv);
        telem.print(",speed=");            telem.print(speed_ms, 2);
        telem.print(",state=");            telem.print(stopped ? "stopped" : "spinning");
        telem.println();
        delay(150);
    }
    telem.println("$TDONE:taho");
}

// ─── ESC test ────────────────────────────────────────────────────────────────
static void wifi_test_esc() {
    telem.println("$T:ESC,phase=arm");
    car.write_speed(0);
    delay(2000);

    noInterrupts();
    _taho_count = 0;
    _taho_last  = micros();
    _taho_iv    = 0;
    interrupts();

    telem.println("$T:ESC,phase=run");
    car.motor_esc.writeMicroseconds(cfg_min_speed);

    unsigned long esc_start = millis();
    while ((millis() - esc_start) < 2000) {
        if (wifi_check_abort()) break;
        car.poll_lidars();

        noInterrupts();
        unsigned long cnt = _taho_count;
        interrupts();

        bool stopped = (micros() - _taho_last) > 500000UL;
        float speed_ms = stopped ? 0.0f : get_speed();
        float revs = (float)cnt / (float)cfg_encoder_holes;

        telem.print("$T:ESC,pulses=");  telem.print(cnt);
        telem.print(",revs=");           telem.print(revs, 1);
        telem.print(",speed=");          telem.print(speed_ms, 2);
        telem.println();
        delay(100);
    }

    car.write_speed(0);
    delay(500);

    noInterrupts();
    unsigned long final_cnt = _taho_count;
    interrupts();

    telem.print("$T:ESC,phase=done,total_pulses=");  telem.print(final_cnt);
    telem.print(",total_revs=");                      telem.print((float)final_cnt / cfg_encoder_holes, 1);
    telem.println();
    telem.println("$TDONE:esc");
}

// ─── Speed PID test ──────────────────────────────────────────────────────────
static void wifi_test_speed() {
    float target = 1.5f;

    telem.println("$T:SPEED,phase=arm");
    car.write_speed(0);
    delay(2000);

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

    telem.println("$T:SPEED,phase=run");
    car.write_speed_ms(target);
    unsigned long prev_ms = millis();
    unsigned long start   = millis();

    while ((millis() - start) < 10000) {
        if (wifi_check_abort()) break;
        car.poll_lidars();

        unsigned long now_ms = millis();
        if (now_ms - prev_ms < 80) continue;
        prev_ms = now_ms;

        car.pid_control_motor();

        telem.print("$T:SPEED,target=");   telem.print(target, 2);
        telem.print(",actual=");            telem.print(get_speed(), 2);
        telem.print(",filtered=");          telem.print(car.pid_filtered, 2);
        telem.println();
    }

    car.write_speed(0);
    car.write_speed_ms(0);
    telem.println("$TDONE:speed");
}

// ─── PID autotune (relay method) ─────────────────────────────────────────────
static void wifi_test_autotune() {
    const float TARGET     = 1.5f;
    const int   RELAY_D    = 20;
    const float HYST       = 0.10f;
    const int   BASE_ESC   = cfg_min_speed + 20;
    const int   SKIP_HALF  = 4;
    const int   NEED_HALF  = 12;
    const unsigned long TIMEOUT = 40000;

    telem.println("$T:TUNE,phase=arm");
    car.write_speed(0);
    delay(2000);

    noInterrupts();
    _taho_count = 0; _taho_last = micros(); _taho_iv = 0;
    interrupts();

    telem.println("$T:TUNE,phase=relay");

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
    car.motor_esc.writeMicroseconds(esc_val);

    while (millis() - start_ms < TIMEOUT) {
        car.poll_lidars();
        if (wifi_check_abort()) break;

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
        car.motor_esc.writeMicroseconds(esc_val);

        telem.print("$T:TUNE,speed="); telem.print(filtered, 2);
        telem.print(",relay=");         telem.print(relay_high ? 1 : 0);
        telem.print(",half=");          telem.print(half_cycle);
        telem.println();

        if (half_cycle >= SKIP_HALF + NEED_HALF) break;
    }

    car.write_speed(0);
    delay(500);

    if (np < 2 || nt < 2 || nsw < 4) {
        telem.println("$T:TUNE,phase=error,msg=not_enough_data");
        telem.println("$TDONE:autotune");
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

    // Raw tune parameters
    telem.print("$TR:TUNE,Ku="); telem.print(Ku, 2);
    telem.print(",Tu=");          telem.print(Tu, 3);
    telem.print(",amp=");         telem.print(amplitude, 3);
    telem.println();

    // Ziegler-Nichols
    float zn_kP = 0.6f * Ku;
    float zn_kI = zn_kP / (0.5f * Tu);
    float zn_kD = zn_kP * Tu / 8.0f;
    telem.print("$TR:ZN,KP="); telem.print(zn_kP, 2);
    telem.print(",KI=");       telem.print(zn_kI, 2);
    telem.print(",KD=");       telem.print(zn_kD, 3);
    telem.println();

    // Tyreus-Luyben
    float tl_kP = Ku / 2.2f;
    float tl_kI = tl_kP / (2.2f * Tu);
    float tl_kD = tl_kP * Tu / 6.3f;
    telem.print("$TR:TL,KP="); telem.print(tl_kP, 2);
    telem.print(",KI=");       telem.print(tl_kI, 2);
    telem.print(",KD=");       telem.print(tl_kD, 3);
    telem.println();

    // PI only
    float pi_kP = 0.45f * Ku;
    float pi_kI = pi_kP / (0.83f * Tu);
    telem.print("$TR:PI,KP="); telem.print(pi_kP, 2);
    telem.print(",KI=");       telem.print(pi_kI, 2);
    telem.println(",KD=0.000");

    telem.println("$TDONE:autotune");
}

// ─── Reactive steering test ──────────────────────────────────────────────────
static void wifi_test_reactive() {
    const int CLOSE_DIST = 1200;
    const int FAR_DIST   = 3000;

    unsigned long start = millis();
    while ((millis() - start) < 30000) {
        if (wifi_check_abort()) break;
        car.poll_lidars();
        int* s = car.read_sensors();

        int L = s[0], FL = s[1], FR = s[2], R = s[3];
        float diff = (float)(R - L);
        if (FL < CLOSE_DIST) diff += (float)(CLOSE_DIST - FL);
        if (FR < CLOSE_DIST) diff -= (float)(CLOSE_DIST - FR);
#if PLATFORM_ESP32S3
        int F = s[4];
        if (F < CLOSE_DIST) {
            // Front blocked — bias toward the more open side
            diff += (float)(FR - FL) * 0.5f;
        }
#endif

        float steer_f = constrain(diff / (float)FAR_DIST, -1.0f, 1.0f);
        int steer_val = (int)(steer_f * 1000.0f);
        car.write_steer(steer_val);

        telem.print("$T:REACT,L="); telem.print(L);
        telem.print(",FL=");         telem.print(FL);
        telem.print(",FR=");         telem.print(FR);
        telem.print(",R=");          telem.print(R);
#if PLATFORM_ESP32S3
        telem.print(",F=");          telem.print(F);
        telem.print(",RE=");         telem.print(s[5]);
#endif
        telem.print(",steer=");      telem.print(steer_val);
        telem.println();
        delay(50);
    }

    car.write_steer(0);
    telem.println("$TDONE:reactive");
}

#endif  // HAS_TELEM
