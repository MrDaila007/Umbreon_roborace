#pragma once
// ═════════════════════════════════════════════════════════════════════════════
// commands.h — Command protocol, idle telemetry, ESC calibration
//
// ASCII protocol over TCP/WebSocket.  Commands start with '$', responses are
// $ACK/$NAK/$CFG/etc.  See CLAUDE.md for the full protocol reference.
//
// Depends on: config.h, eeprom_settings.h, wifi_tests.h, car, telem,
//             get_speed(), HAS_TELEM
// ═════════════════════════════════════════════════════════════════════════════

// ─── ESC calibration ─────────────────────────────────────────────────────────
// Standard ESC calibration: max throttle → ESC learns high endpoint,
// min throttle → ESC learns low endpoint, then neutral.
void run_calibration() {
#if HAS_TELEM
    telem.println("$T:CAL,phase=esc_max");
#endif
    car.motor_esc.writeMicroseconds(2000);  // ESC max signal
    delay(3000);                            // wait for ESC to register max (beeps)

#if HAS_TELEM
    telem.println("$T:CAL,phase=esc_min");
#endif
    car.motor_esc.writeMicroseconds(1000);  // ESC min signal
    delay(3000);                            // wait for ESC to register min (beeps)

#if HAS_TELEM
    telem.println("$T:CAL,phase=esc_neutral");
#endif
    car.motor_esc.writeMicroseconds(NEUTRAL_SPEED);  // neutral (1500µs)
    delay(1000);

    // Mark as calibrated and persist
    cfg_calibrated = true;
    save_settings();

#if HAS_TELEM
    telem.println("$T:CAL,phase=done");
    telem.println("$TDONE:cal");
#endif
}

// ─── Command handlers ────────────────────────────────────────────────────────
#if HAS_TELEM
static char cmd_buf[512];
static int  cmd_len = 0;

static void cmd_ping() {
    telem.println("$PONG");
}

static void cmd_get() {
    telem.print("$CFG:");
    telem.print("FOD=");  telem.print(cfg_front_obstacle_dist);
    telem.print(",SOD="); telem.print(cfg_side_open_dist);
    telem.print(",ACD="); telem.print(cfg_all_close_dist);
    telem.print(",CFD="); telem.print(cfg_close_front_dist);
    telem.print(",KP=");  telem.print(cfg_pid_kp, 2);
    telem.print(",KI=");  telem.print(cfg_pid_ki, 2);
    telem.print(",KD=");  telem.print(cfg_pid_kd, 2);
    telem.print(",MSP="); telem.print(cfg_min_speed);
    telem.print(",XSP="); telem.print(cfg_max_speed);
    telem.print(",BSP="); telem.print(cfg_min_bspeed);
    telem.print(",MNP="); telem.print(cfg_min_point);
    telem.print(",XNP="); telem.print(cfg_max_point);
    telem.print(",NTP="); telem.print(cfg_neutral_point);
    telem.print(",ENH="); telem.print(cfg_encoder_holes);
    telem.print(",WDM="); telem.print(cfg_wheel_diam_m, 3);
    telem.print(",LMS="); telem.print(cfg_loop_ms);
    telem.print(",SPD1="); telem.print(cfg_spd_clear, 1);
    telem.print(",SPD2="); telem.print(cfg_spd_blocked, 1);
    telem.print(",COE1="); telem.print(cfg_coe_clear, 2);
    telem.print(",COE2="); telem.print(cfg_coe_blocked, 2);
    telem.print(",WDD="); telem.print(cfg_wrong_dir_deg, 1);
    telem.print(",RCW="); telem.print(cfg_race_cw ? 1 : 0);
    telem.print(",STK="); telem.print(cfg_stuck_thresh);
    telem.print(",IMR="); telem.print(cfg_imu_rotate ? 1 : 0);
    telem.print(",SVR="); telem.print(cfg_servo_reverse ? 1 : 0);
    telem.print(",CAL="); telem.print(cfg_calibrated ? 1 : 0);
    telem.print(",BEN="); telem.print(cfg_bat_enabled ? 1 : 0);
    telem.print(",BML="); telem.print(cfg_bat_multiplier, 2);
    telem.print(",BLV="); telem.print(cfg_bat_low, 1);
    telem.print(",IMU="); telem.print(USE_IMU);
    telem.print(",DBG="); telem.print(HAS_TELEM);
    telem.print(",SNS="); telem.print(car.sensor_amount);
    telem.println();
}

static void cmd_sens() {
    telem.print("$SENS:");
    for (int i = 0; i < car.sensor_amount; i++) {
        if (i > 0) telem.print(',');
        telem.print(car.sensor_ok(i) ? 1 : 0);
    }
    telem.println();
}

static bool parse_set_pair(const char* pair) {
    const char* eq = strchr(pair, '=');
    if (!eq) return false;

    char key[8];
    int klen = eq - pair;
    if (klen <= 0 || klen >= (int)sizeof(key)) return false;
    memcpy(key, pair, klen);
    key[klen] = '\0';

    const char* val = eq + 1;

    if      (strcmp(key, "FOD")  == 0) cfg_front_obstacle_dist = atoi(val);
    else if (strcmp(key, "SOD")  == 0) cfg_side_open_dist      = atoi(val);
    else if (strcmp(key, "ACD")  == 0) cfg_all_close_dist      = atoi(val);
    else if (strcmp(key, "CFD")  == 0) cfg_close_front_dist    = atoi(val);
    else if (strcmp(key, "KP")   == 0) cfg_pid_kp              = atof(val);
    else if (strcmp(key, "KI")   == 0) cfg_pid_ki              = atof(val);
    else if (strcmp(key, "KD")   == 0) cfg_pid_kd              = atof(val);
    else if (strcmp(key, "MSP")  == 0) cfg_min_speed           = atoi(val);
    else if (strcmp(key, "XSP")  == 0) cfg_max_speed           = atoi(val);
    else if (strcmp(key, "BSP")  == 0) cfg_min_bspeed          = atoi(val);
    else if (strcmp(key, "MNP")  == 0) cfg_min_point           = atoi(val);
    else if (strcmp(key, "XNP")  == 0) cfg_max_point           = atoi(val);
    else if (strcmp(key, "NTP")  == 0) cfg_neutral_point       = atoi(val);
    else if (strcmp(key, "ENH")  == 0) cfg_encoder_holes       = atoi(val);
    else if (strcmp(key, "WDM")  == 0) cfg_wheel_diam_m        = atof(val);
    else if (strcmp(key, "LMS")  == 0) cfg_loop_ms             = atoi(val);
    else if (strcmp(key, "SPD1") == 0) cfg_spd_clear           = atof(val);
    else if (strcmp(key, "SPD2") == 0) cfg_spd_blocked         = atof(val);
    else if (strcmp(key, "COE1") == 0) cfg_coe_clear           = atof(val);
    else if (strcmp(key, "COE2") == 0) cfg_coe_blocked         = atof(val);
    else if (strcmp(key, "WDD")  == 0) cfg_wrong_dir_deg       = atof(val);
    else if (strcmp(key, "RCW")  == 0) cfg_race_cw             = atoi(val) != 0;
    else if (strcmp(key, "STK")  == 0) cfg_stuck_thresh        = atoi(val);
    else if (strcmp(key, "IMR")  == 0) cfg_imu_rotate          = atoi(val) != 0;
    else if (strcmp(key, "SVR")  == 0) cfg_servo_reverse       = atoi(val) != 0;
    else if (strcmp(key, "CAL")  == 0) cfg_calibrated          = atoi(val) != 0;
    else if (strcmp(key, "BEN")  == 0) cfg_bat_enabled          = atoi(val) != 0;
    else if (strcmp(key, "BML")  == 0) cfg_bat_multiplier      = atof(val);
    else if (strcmp(key, "BLV")  == 0) cfg_bat_low             = atof(val);
    // IMU, DBG are read-only — silently ignore
    else return false;

    return true;
}

static void cmd_set(const char* args) {
    char buf[512];
    strncpy(buf, args, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char* token = strtok(buf, ",");
    while (token) {
        if (!parse_set_pair(token)) {
            telem.print("$NAK:");
            telem.println(token);
            return;
        }
        token = strtok(NULL, ",");
    }
    telem.println("$ACK");
}

static void cmd_save() {
    save_settings();
    telem.println("$ACK");
}

static void cmd_load() {
    if (load_settings()) {
        telem.println("$ACK");
    } else {
        telem.println("$NAK:no_saved_config");
    }
}

static void cmd_rst() {
    cfg_front_obstacle_dist = 1200;
    cfg_side_open_dist      = 1000;
    cfg_all_close_dist      =  800;
    cfg_close_front_dist    =  201;
    cfg_pid_kp   = 60.0f;
    cfg_pid_ki   = 40.0f;
    cfg_pid_kd   = 6.0f;
    cfg_min_speed   = 1540;
    cfg_max_speed   = 1700;
    cfg_min_bspeed  = 1460;
    cfg_min_point     = 40;
    cfg_max_point     = 140;
    cfg_neutral_point = 90;
    cfg_encoder_holes = 62;
    cfg_wheel_diam_m  = 0.060f;
    cfg_loop_ms       = 40;
    cfg_spd_clear     = 2.7f;
    cfg_spd_blocked   = 0.8f;
    cfg_coe_clear     = 0.3f;
    cfg_coe_blocked   = 0.7f;
    cfg_wrong_dir_deg = 120.0f;
    cfg_race_cw       = true;
    cfg_stuck_thresh  = 25;
    cfg_imu_rotate    = true;
    cfg_servo_reverse = true;
    cfg_calibrated    = false;
    cfg_bat_enabled    = false;
    cfg_bat_multiplier = 2.8f;
    cfg_bat_low        = 6.0f;
    telem.println("$ACK");
}

// ─── Manual drive command ────────────────────────────────────────────────────
static void cmd_drv(const char* args) {
    char buf[32];
    strncpy(buf, args, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char* comma = strchr(buf, ',');
    if (!comma) return;
    *comma = '\0';
    manual_steer = atoi(buf);
    manual_speed = atof(comma + 1);
    manual_mode = true;
    last_drv_ms = millis();
}

// ─── Idle telemetry (sent when car is stopped) ──────────────────────────────
static void send_idle_telemetry() {
    car.poll_lidars();
#if USE_IMU
    car.imu_update();
#endif
    int* s = car.read_sensors();
    telem.print(millis());              telem.print(',');
    telem.print(s[0]);                  telem.print(',');
    telem.print(s[1]);                  telem.print(',');
    telem.print(s[2]);                  telem.print(',');
    telem.print(s[3]);                  telem.print(',');
#if PLATFORM_ESP32S3
    telem.print(s[4]);                  telem.print(',');
    telem.print(s[5]);                  telem.print(',');
#endif
    telem.print(0);                     telem.print(',');
    telem.print(get_speed(), 2);        telem.print(',');
    telem.print(0.0, 1);
#if USE_IMU
    telem.print(',');
    telem.print(car.yaw_rate, 1);       telem.print(',');
    telem.print(car.heading, 1);
#endif
    telem.println();
}

// ─── Start / Stop / Status / Test ────────────────────────────────────────────
static void cmd_start() {
    car.pid_integral   = 0;
    car.pid_prev_error = 0;
    car.pid_filtered   = 0;
    car.pid_prev_ms    = 0;
#if USE_IMU
    car.reset_heading();
#endif
    telem.println("$ACK");

    // 5-second countdown — idle telemetry flows, $STOP aborts
    unsigned long start_at = millis() + 5000;
    while (millis() < start_at) {
        car.poll_lidars();
        if (wifi_check_abort()) return;
        send_idle_telemetry();
        delay(cfg_loop_ms);
    }
    car_running = true;
    telem.println("$STS:RUN");
}

static void cmd_stop() {
    car_running = false;
    drv_enabled = false;
    manual_mode = false;
    manual_steer = 0;
    manual_speed = 0.0f;
    car.write_speed(0);
    car.write_steer(0);
    telem.println("$ACK");
    telem.println("$STS:STOP");
}

static void cmd_status() {
    telem.println(car_running ? "$STS:RUN" : "$STS:STOP");
}

static void cmd_test(const char* name) {
    // Auto-stop car before running any test
    if (car_running) {
        car_running = false;
        car.write_speed(0);
        car.write_steer(0);
    }

    if      (strcmp(name, "lidar")    == 0) wifi_test_lidar();
    else if (strcmp(name, "servo")    == 0) wifi_test_servo();
    else if (strcmp(name, "taho")     == 0) wifi_test_taho();
    else if (strcmp(name, "esc")      == 0) wifi_test_esc();
    else if (strcmp(name, "speed")    == 0) wifi_test_speed();
    else if (strcmp(name, "autotune") == 0) wifi_test_autotune();
    else if (strcmp(name, "reactive") == 0) wifi_test_reactive();
    else if (strcmp(name, "cal")      == 0) { cfg_calibrated = false; run_calibration(); }
    else {
        telem.print("$NAK:unknown_test:");
        telem.println(name);
    }
}

// ─── Command dispatcher ─────────────────────────────────────────────────────
static void dispatch_command(const char* line) {
    if      (strcmp(line, "$PING")   == 0) cmd_ping();
    else if (strcmp(line, "$GET")    == 0) cmd_get();
    else if (strncmp(line, "$SET:", 5) == 0) cmd_set(line + 5);
    else if (strcmp(line, "$SAVE")   == 0) cmd_save();
    else if (strcmp(line, "$LOAD")   == 0) cmd_load();
    else if (strcmp(line, "$RST")    == 0) cmd_rst();
    else if (strcmp(line, "$START")  == 0) cmd_start();
    else if (strcmp(line, "$STOP")   == 0) cmd_stop();
    else if (strcmp(line, "$STATUS") == 0) cmd_status();
    else if (strcmp(line, "$BAT")    == 0) {
        telem.print("$BAT:"); telem.println(car.bat_voltage, 2);
    }
    else if (strcmp(line, "$SENS")   == 0) cmd_sens();
    else if (strncmp(line, "$TEST:", 6) == 0) cmd_test(line + 6);
    else if (strncmp(line, "$DRV:", 5) == 0) cmd_drv(line + 5);
    else if (strncmp(line, "$SRV:", 5) == 0) {
        int angle = constrain(atoi(line + 5), 0, 180);
        car.steer_servo.write(angle);
    }
    else if (strncmp(line, "$ESC:", 5) == 0) {
        int val = constrain(atoi(line + 5), 1000, 2000);
        car.motor_esc.writeMicroseconds(val);
    }
    else if (strcmp(line, "$DRVEN")  == 0) { drv_enabled = true;  telem.println("$ACK"); }
    else if (strcmp(line, "$DRVOFF") == 0) {
        drv_enabled = false; manual_mode = false;
        manual_steer = 0; manual_speed = 0.0f;
        car.write_steer(0); car.write_speed(0);
        telem.println("$ACK");
    }
    // Unknown commands silently ignored
}

static void process_commands() {
    while (telem.available() > 0) {
        char c = (char)telem.read();
        if (c == '\n' || c == '\r') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                if (cmd_buf[0] == '$') {
                    dispatch_command(cmd_buf);
                }
                cmd_len = 0;
            }
        } else if (cmd_len < (int)sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_len++] = c;
        }
    }
}

#endif  // HAS_TELEM
