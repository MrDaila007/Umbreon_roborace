#pragma once
// ═════════════════════════════════════════════════════════════════════════════
// control.h — Autonomous driving logic
//
// Contains the main control loop (work()), stuck recovery, and wrong-direction
// detection.  Called every cfg_loop_ms from loop() when car_running is true.
//
// Depends on: config.h, car (Car instance), telem, get_speed()
// ═════════════════════════════════════════════════════════════════════════════

// ─── Stuck / reverse helpers ─────────────────────────────────────────────────

void go_back() {
    car.write_speed(0);
    while (get_speed() > 0.1f) {}   // wait until stopped
    car.write_speed(-150);
    delay(200);
    car.write_speed(0);
    delay(80);
    car.write_speed(-150);
    delay(700);
    car.write_speed(0);
}

void go_back_long() {
    car.write_speed(0);
    while (get_speed() > 0.1f) {}
    car.write_speed(-150);
    delay(1000);
    car.write_speed(0);
    delay(80);
    car.write_speed(-150);
    delay(1800);
    car.write_speed(0);
}

// ─── Main control logic ─────────────────────────────────────────────────────

void work() {
    // Bring sensor data up to date before reading
    car.poll_lidars();
#if USE_IMU
    car.imu_update();
#endif
    int* s = car.read_sensors();

    // ── Steering ─────────────────────────────────────────────────────────────
    int diff;
    bool f_l = s[1] < cfg_front_obstacle_dist;  // front-left blocked
    bool f_r = s[2] < cfg_front_obstacle_dist;  // front-right blocked
#if PLATFORM_ESP32S3
    bool f_c = s[4] < cfg_front_obstacle_dist;  // front-center blocked
#endif

    if (s[0] > cfg_side_open_dist && s[3] > cfg_side_open_dist) {
        // Both sides open — keep to right wall
        diff = 800;
    } else {
        // Balance between walls
        diff = s[3] - s[0];  // positive → steer right (away from right wall)
    }

    // All sensors close: hard turn to escape
    if (s[0] < cfg_all_close_dist && s[1] < cfg_all_close_dist &&
        s[2] < cfg_all_close_dist && s[3] < cfg_all_close_dist) {
        diff = 800;
    }

    // ── Speed ────────────────────────────────────────────────────────────────
#if PLATFORM_ESP32S3
    int how_clear = (int)f_l + (int)f_r + (int)f_c;  // 0 = path clear, 1-3 = blocked
#else
    int how_clear = (int)f_l + (int)f_r;  // 0 = path clear, 1-2 = blocked
#endif
    float coef, spd;

    switch (how_clear) {
        case 0:
            coef = cfg_coe_clear;   spd = cfg_spd_clear;   break;
        default:
            coef = cfg_coe_blocked; spd = cfg_spd_blocked;  break;
    }

    car.write_steer((int)(diff * coef));
    car.write_speed_ms(spd);
    car.pid_control_motor();

    // ── Telemetry ────────────────────────────────────────────────────────────
#if HAS_TELEM
    telem.print(millis());              telem.print(',');
    telem.print(s[0]);                  telem.print(',');
    telem.print(s[1]);                  telem.print(',');
    telem.print(s[2]);                  telem.print(',');
    telem.print(s[3]);                  telem.print(',');
#if PLATFORM_ESP32S3
    telem.print(s[4]);                  telem.print(',');
    telem.print(s[5]);                  telem.print(',');
#endif
    telem.print((int)(diff * coef));    telem.print(',');
    telem.print(get_speed(), 2);        telem.print(',');
    telem.print(spd, 1);
#if USE_IMU
    telem.print(',');
    telem.print(car.yaw_rate, 1);       telem.print(',');
    telem.print(car.heading, 1);
#endif
    telem.println();
#endif

    // ── Stuck detection ──────────────────────────────────────────────────────
    bool c_fl = s[1] < cfg_close_front_dist;
    bool c_fr = s[2] < cfg_close_front_dist;
    bool low_speed = get_speed() < 0.1f;

    static int stuck_time = 0;
    if (c_fl || c_fr || low_speed) {
        stuck_time++;
    } else {
        stuck_time = 0;
    }

    if (stuck_time > cfg_stuck_thresh) {
        car.write_steer(0);
        go_back();
        car.write_speed_ms(2.0f);
        stuck_time = 0;
#if USE_IMU
        car.reset_heading();
#endif
    }

    // ── Wrong-direction / dead-end detection ─────────────────────────────────
    static float turns = 0.0f;

#if USE_IMU
    // Real heading from gyro Z (°): positive = CCW, negative = CW
    turns += car.yaw_rate * (cfg_loop_ms / 1000.0f);
    // Decay correct-direction accumulation so normal laps don't build up
    if ( cfg_race_cw && turns < 0.0f) turns *= 0.97f;
    if (!cfg_race_cw && turns > 0.0f) turns *= 0.97f;
    turns = constrain(turns, -200.0f, 200.0f);
    bool wrong_way = cfg_race_cw ? (turns > cfg_wrong_dir_deg)
                                  : (turns < -cfg_wrong_dir_deg);
#else
    // Heuristic: accumulate diff × speed as proxy for heading change
    turns += diff * get_speed() / -1000.0f;
    turns = constrain(turns, -1500.0f, 50.0f);
    bool wrong_way = turns < -18.0f;
#endif

    if (wrong_way) {
        car.write_speed(0);
        delay(100);
#if USE_IMU
        car.write_steer(cfg_race_cw ? 1000 : -1000);
#else
        car.write_steer(1000);
#endif
        delay(20);
        go_back_long();
#if USE_IMU
        car.write_steer(cfg_race_cw ? -700 : 700);
#else
        car.write_steer(-700);
#endif
        car.write_speed_ms(2.0f);
        unsigned long strt = millis();
        while ((millis() - strt) < 900) {
            car.poll_lidars();
#if USE_IMU
            car.imu_update();
#endif
            car.pid_control_motor();
        }
        turns = 0.0f;
#if USE_IMU
        car.reset_heading();
#endif
    }
}
