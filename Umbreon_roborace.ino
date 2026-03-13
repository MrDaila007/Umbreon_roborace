/**
 * ______      _
 * | ___ \    | |
 * | |_/ /___ | |__   ___  _ __ __ _  ___ ___
 * |    // _ \| '_ \ / _ \| '__/ _` |/ __/ _ \
 * | |\ \ (_) | |_) | (_) | | | (_| | (_|  __/
 * \_| \_\___/|_.__/ \___/|_|  \__,_|\___\___|
 *
 * Umbreon — RP2350 (Pico 2) + 4× TF-Luna LiDAR
 *
 * Sensor layout (SerialPIO RX pins):
 *   s[0] Left        GP2
 *   s[1] Front-Left  GP3
 *   s[2] Front-Right GP4
 *   s[3] Right       GP5
 *
 * Actuators:
 *   Steering servo   GP10
 *   Motor ESC        GP11
 *   Tachometer       GP13 (RISING interrupt)
 *
 * Distance units: cm×10  (e.g. 1200 = 120 cm, 200 = 20 cm)
 *   TF-Luna range: ~20 cm … 800 cm  →  200 … 8000 in these units
 */

#pragma GCC optimize("Ofast")

// ─── Feature flags ──────────────────────────────────────────────────────────
#define USE_IMU         1       // 1 = enable MPU-6050 gyro, 0 = disable
#define USE_WIFI_DEBUG  1       // 1 = enable Wemos D1 Mini WiFi telemetry, 0 = disable

#if USE_WIFI_DEBUG
#define DEBUG_TX_PIN  16         // GP16 = UART0 TX → D1 Mini RX
#define DEBUG_RX_PIN  17         // GP17 = UART0 RX ← D1 Mini TX
#endif

// ─── Runtime-configurable globals (defaults match original #defines) ────────
// Obstacle thresholds (cm×10)
int   cfg_front_obstacle_dist = 1200;   // 120 cm — start steering around
int   cfg_side_open_dist      = 1000;   // 100 cm — side is "open"
int   cfg_all_close_dist      =  800;   //  80 cm — surrounded, force turn
int   cfg_close_front_dist    =  201;   //  20 cm — emergency reverse

// PID coefficients (Tyreus-Luyben auto-tuned: Ku=9.20, Tu=0.648s)
float cfg_pid_kp   = 4.18f;
float cfg_pid_ki   = 2.93f;
float cfg_pid_kd   = 0.43f;

// ESC limits
int   cfg_min_speed   = 96;    // slowest forward
int   cfg_max_speed   = 110;   // fastest forward
int   cfg_min_bspeed  = 85;    // slowest reverse

// Steering limits
int   cfg_min_point     = 40;
int   cfg_max_point     = 140;
int   cfg_neutral_point = 90;

// Tachometer
int   cfg_encoder_holes = 62;
float cfg_wheel_diam_m  = 0.060f;

// Control loop
int   cfg_loop_ms = 40;

// Speed/steer coefficients
float cfg_spd_clear   = 2.7f;   // speed when path clear (m/s)
float cfg_spd_blocked = 0.8f;   // speed when blocked
float cfg_coe_clear   = 0.3f;   // steer coefficient when clear
float cfg_coe_blocked = 0.7f;   // steer coefficient when blocked

// Navigation
float cfg_wrong_dir_deg = 120.0f;
bool  cfg_race_cw       = true;
int   cfg_stuck_thresh  = 25;

// ─── Start/Stop control ────────────────────────────────────────────────────
// #define COMPETITION_MODE     // uncomment to start driving immediately
#ifdef COMPETITION_MODE
bool car_running = true;
#else
bool car_running = false;
#endif

// ─── Manual drive mode ($DRV command) ─────────────────────────────────────
bool manual_mode = false;
unsigned long last_drv_ms = 0;
int manual_steer = 0;
float manual_speed = 0.0f;

#include "luna_car.h"
#include "tests.h"      // hardware tests + WiFi remote tests

#include <EEPROM.h>

Car car;

// ─── EEPROM settings ────────────────────────────────────────────────────────
#define SETTINGS_MAGIC   0x554D4252   // "UMBR"
#define SETTINGS_VERSION 1
#define SETTINGS_ADDR    0

struct __attribute__((packed)) CarSettings {
    uint32_t magic;
    uint8_t  version;
    // Obstacle thresholds
    int16_t  front_obstacle_dist;
    int16_t  side_open_dist;
    int16_t  all_close_dist;
    int16_t  close_front_dist;
    // PID
    float    pid_kp;
    float    pid_ki;
    float    pid_kd;
    // ESC
    int8_t   min_speed;
    int8_t   max_speed;
    int8_t   min_bspeed;
    // Steering
    int8_t   min_point;
    int8_t   max_point;
    int8_t   neutral_point;
    // Tachometer
    int8_t   encoder_holes;
    float    wheel_diam_m;
    // Control
    int8_t   loop_ms;
    float    spd_clear;
    float    spd_blocked;
    float    coe_clear;
    float    coe_blocked;
    // Navigation
    float    wrong_dir_deg;
    uint8_t  race_cw;
    int8_t   stuck_thresh;
    // Checksum (sum of all preceding bytes)
    uint8_t  checksum;
};

static uint8_t compute_checksum(const CarSettings& s) {
    uint8_t sum = 0;
    const uint8_t* p = (const uint8_t*)&s;
    size_t len = sizeof(CarSettings) - 1;  // exclude checksum byte
    for (size_t i = 0; i < len; i++) sum += p[i];
    return sum;
}

static void populate_struct(CarSettings& s) {
    s.magic   = SETTINGS_MAGIC;
    s.version = SETTINGS_VERSION;
    s.front_obstacle_dist = (int16_t)cfg_front_obstacle_dist;
    s.side_open_dist      = (int16_t)cfg_side_open_dist;
    s.all_close_dist      = (int16_t)cfg_all_close_dist;
    s.close_front_dist    = (int16_t)cfg_close_front_dist;
    s.pid_kp       = cfg_pid_kp;
    s.pid_ki       = cfg_pid_ki;
    s.pid_kd       = cfg_pid_kd;
    s.min_speed    = (int8_t)cfg_min_speed;
    s.max_speed    = (int8_t)cfg_max_speed;
    s.min_bspeed   = (int8_t)cfg_min_bspeed;
    s.min_point    = (int8_t)cfg_min_point;
    s.max_point    = (int8_t)cfg_max_point;
    s.neutral_point = (int8_t)cfg_neutral_point;
    s.encoder_holes = (int8_t)cfg_encoder_holes;
    s.wheel_diam_m  = cfg_wheel_diam_m;
    s.loop_ms       = (int8_t)cfg_loop_ms;
    s.spd_clear     = cfg_spd_clear;
    s.spd_blocked   = cfg_spd_blocked;
    s.coe_clear     = cfg_coe_clear;
    s.coe_blocked   = cfg_coe_blocked;
    s.wrong_dir_deg = cfg_wrong_dir_deg;
    s.race_cw       = cfg_race_cw ? 1 : 0;
    s.stuck_thresh  = (int8_t)cfg_stuck_thresh;
    s.checksum      = compute_checksum(s);
}

static void apply_struct(const CarSettings& s) {
    cfg_front_obstacle_dist = s.front_obstacle_dist;
    cfg_side_open_dist      = s.side_open_dist;
    cfg_all_close_dist      = s.all_close_dist;
    cfg_close_front_dist    = s.close_front_dist;
    cfg_pid_kp       = s.pid_kp;
    cfg_pid_ki       = s.pid_ki;
    cfg_pid_kd       = s.pid_kd;
    cfg_min_speed    = s.min_speed;
    cfg_max_speed    = s.max_speed;
    cfg_min_bspeed   = s.min_bspeed;
    cfg_min_point    = s.min_point;
    cfg_max_point    = s.max_point;
    cfg_neutral_point = s.neutral_point;
    cfg_encoder_holes = s.encoder_holes;
    cfg_wheel_diam_m  = s.wheel_diam_m;
    cfg_loop_ms       = s.loop_ms;
    cfg_spd_clear     = s.spd_clear;
    cfg_spd_blocked   = s.spd_blocked;
    cfg_coe_clear     = s.coe_clear;
    cfg_coe_blocked   = s.coe_blocked;
    cfg_wrong_dir_deg = s.wrong_dir_deg;
    cfg_race_cw       = s.race_cw != 0;
    cfg_stuck_thresh  = s.stuck_thresh;
}

bool load_settings() {
    CarSettings s;
    EEPROM.get(SETTINGS_ADDR, s);
    if (s.magic != SETTINGS_MAGIC || s.version != SETTINGS_VERSION)
        return false;
    if (compute_checksum(s) != s.checksum)
        return false;
    apply_struct(s);
    return true;
}

bool save_settings() {
    CarSettings s;
    populate_struct(s);
    EEPROM.put(SETTINGS_ADDR, s);
    EEPROM.commit();
    return true;
}

// ─── Command protocol ──────────────────────────────────────────────────────
#if USE_WIFI_DEBUG
static char cmd_buf[256];
static int  cmd_len = 0;

static void cmd_ping() {
    Serial1.println("$PONG");
}

static void cmd_get() {
    Serial1.print("$CFG:");
    Serial1.print("FOD=");  Serial1.print(cfg_front_obstacle_dist);
    Serial1.print(",SOD="); Serial1.print(cfg_side_open_dist);
    Serial1.print(",ACD="); Serial1.print(cfg_all_close_dist);
    Serial1.print(",CFD="); Serial1.print(cfg_close_front_dist);
    Serial1.print(",KP=");  Serial1.print(cfg_pid_kp, 2);
    Serial1.print(",KI=");  Serial1.print(cfg_pid_ki, 2);
    Serial1.print(",KD=");  Serial1.print(cfg_pid_kd, 2);
    Serial1.print(",MSP="); Serial1.print(cfg_min_speed);
    Serial1.print(",XSP="); Serial1.print(cfg_max_speed);
    Serial1.print(",BSP="); Serial1.print(cfg_min_bspeed);
    Serial1.print(",MNP="); Serial1.print(cfg_min_point);
    Serial1.print(",XNP="); Serial1.print(cfg_max_point);
    Serial1.print(",NTP="); Serial1.print(cfg_neutral_point);
    Serial1.print(",ENH="); Serial1.print(cfg_encoder_holes);
    Serial1.print(",WDM="); Serial1.print(cfg_wheel_diam_m, 3);
    Serial1.print(",LMS="); Serial1.print(cfg_loop_ms);
    Serial1.print(",SPD1="); Serial1.print(cfg_spd_clear, 1);
    Serial1.print(",SPD2="); Serial1.print(cfg_spd_blocked, 1);
    Serial1.print(",COE1="); Serial1.print(cfg_coe_clear, 2);
    Serial1.print(",COE2="); Serial1.print(cfg_coe_blocked, 2);
    Serial1.print(",WDD="); Serial1.print(cfg_wrong_dir_deg, 1);
    Serial1.print(",RCW="); Serial1.print(cfg_race_cw ? 1 : 0);
    Serial1.print(",STK="); Serial1.print(cfg_stuck_thresh);
    Serial1.print(",IMU="); Serial1.print(USE_IMU);
    Serial1.print(",DBG="); Serial1.print(USE_WIFI_DEBUG);
    Serial1.println();
}

static bool parse_set_pair(const char* pair) {
    // Parse "KEY=VALUE" and apply to the matching cfg_* global
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
    // IMU, DBG are read-only — silently ignore
    else return false;

    return true;
}

static void cmd_set(const char* args) {
    // args = "KP=5.0,KI=3.0,..."
    char buf[200];
    strncpy(buf, args, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char* token = strtok(buf, ",");
    while (token) {
        if (!parse_set_pair(token)) {
            Serial1.print("$NAK:");
            Serial1.println(token);
            return;
        }
        token = strtok(NULL, ",");
    }
    Serial1.println("$ACK");
}

static void cmd_save() {
    save_settings();
    Serial1.println("$ACK");
}

static void cmd_load() {
    if (load_settings()) {
        Serial1.println("$ACK");
    } else {
        Serial1.println("$NAK:no_saved_config");
    }
}

static void cmd_rst() {
    // Reset to compile-time defaults
    cfg_front_obstacle_dist = 1200;
    cfg_side_open_dist      = 1000;
    cfg_all_close_dist      =  800;
    cfg_close_front_dist    =  201;
    cfg_pid_kp   = 4.18f;
    cfg_pid_ki   = 2.93f;
    cfg_pid_kd   = 0.43f;
    cfg_min_speed   = 96;
    cfg_max_speed   = 110;
    cfg_min_bspeed  = 85;
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
    Serial1.println("$ACK");
}

// ─── WiFi remote tests ──────────────────────────────────────────────────

static bool wifi_check_abort() {
    if (Serial1.available() > 0 && Serial1.peek() == '$')
        return true;
    return false;
}

static void wifi_test_lidar() {
    unsigned long start = millis();
    while ((millis() - start) < 5000) {
        if (wifi_check_abort()) break;
        car.poll_lidars();
        int* s = car.read_sensors();
        Serial1.print("$T:LIDAR,L="); Serial1.print(s[0]);
        Serial1.print(",FL=");        Serial1.print(s[1]);
        Serial1.print(",FR=");        Serial1.print(s[2]);
        Serial1.print(",R=");         Serial1.print(s[3]);
        Serial1.println();
        delay(100);
    }
    Serial1.println("$TDONE:lidar");
}

static void wifi_test_servo() {
    Serial1.println("$T:SERVO,phase=left");
    car.write_steer(-1000); delay(800);
    if (wifi_check_abort()) { car.write_steer(0); Serial1.println("$TDONE:servo"); return; }

    Serial1.println("$T:SERVO,phase=right");
    car.write_steer(1000); delay(800);
    if (wifi_check_abort()) { car.write_steer(0); Serial1.println("$TDONE:servo"); return; }

    Serial1.println("$T:SERVO,phase=center");
    car.write_steer(0); delay(400);

    Serial1.println("$T:SERVO,phase=sweep");
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
    Serial1.println("$TDONE:servo");
}

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

        Serial1.print("$T:TAHO,pulses=");   Serial1.print(cnt);
        Serial1.print(",interval=");         Serial1.print(iv);
        Serial1.print(",speed=");            Serial1.print(speed_ms, 2);
        Serial1.print(",state=");            Serial1.print(stopped ? "stopped" : "spinning");
        Serial1.println();
        delay(150);
    }
    Serial1.println("$TDONE:taho");
}

static void wifi_test_esc() {
    Serial1.println("$T:ESC,phase=arm");
    car.write_speed(0);
    delay(2000);

    noInterrupts();
    _taho_count = 0;
    _taho_last  = micros();
    _taho_iv    = 0;
    interrupts();

    Serial1.println("$T:ESC,phase=run");
    car.motor_esc.write(cfg_min_speed);

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

        Serial1.print("$T:ESC,pulses=");  Serial1.print(cnt);
        Serial1.print(",revs=");           Serial1.print(revs, 1);
        Serial1.print(",speed=");          Serial1.print(speed_ms, 2);
        Serial1.println();
        delay(100);
    }

    car.write_speed(0);
    delay(500);

    noInterrupts();
    unsigned long final_cnt = _taho_count;
    interrupts();

    Serial1.print("$T:ESC,phase=done,total_pulses=");  Serial1.print(final_cnt);
    Serial1.print(",total_revs=");                      Serial1.print((float)final_cnt / cfg_encoder_holes, 1);
    Serial1.println();
    Serial1.println("$TDONE:esc");
}

static void wifi_test_speed() {
    float target = 1.5f;

    Serial1.println("$T:SPEED,phase=arm");
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

    Serial1.println("$T:SPEED,phase=run");
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

        Serial1.print("$T:SPEED,target=");   Serial1.print(target, 2);
        Serial1.print(",actual=");            Serial1.print(get_speed(), 2);
        Serial1.print(",filtered=");          Serial1.print(car.pid_filtered, 2);
        Serial1.println();
    }

    car.write_speed(0);
    car.write_speed_ms(0);
    Serial1.println("$TDONE:speed");
}

static void wifi_test_autotune() {
    const float TARGET     = 1.5f;
    const int   RELAY_D    = 2;
    const float HYST       = 0.10f;
    const int   BASE_ESC   = cfg_min_speed + 2;
    const int   SKIP_HALF  = 4;
    const int   NEED_HALF  = 12;
    const unsigned long TIMEOUT = 40000;

    Serial1.println("$T:TUNE,phase=arm");
    car.write_speed(0);
    delay(2000);

    noInterrupts();
    _taho_count = 0; _taho_last = micros(); _taho_iv = 0;
    interrupts();

    Serial1.println("$T:TUNE,phase=relay");

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
        car.motor_esc.write(esc_val);

        Serial1.print("$T:TUNE,speed="); Serial1.print(filtered, 2);
        Serial1.print(",relay=");         Serial1.print(relay_high ? 1 : 0);
        Serial1.print(",half=");          Serial1.print(half_cycle);
        Serial1.println();

        if (half_cycle >= SKIP_HALF + NEED_HALF) break;
    }

    car.write_speed(0);
    delay(500);

    if (np < 2 || nt < 2 || nsw < 4) {
        Serial1.println("$T:TUNE,phase=error,msg=not_enough_data");
        Serial1.println("$TDONE:autotune");
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
    Serial1.print("$TR:TUNE,Ku="); Serial1.print(Ku, 2);
    Serial1.print(",Tu=");          Serial1.print(Tu, 3);
    Serial1.print(",amp=");         Serial1.print(amplitude, 3);
    Serial1.println();

    // Ziegler-Nichols
    float zn_kP = 0.6f * Ku;
    float zn_kI = zn_kP / (0.5f * Tu);
    float zn_kD = zn_kP * Tu / 8.0f;
    Serial1.print("$TR:ZN,KP="); Serial1.print(zn_kP, 2);
    Serial1.print(",KI=");       Serial1.print(zn_kI, 2);
    Serial1.print(",KD=");       Serial1.print(zn_kD, 3);
    Serial1.println();

    // Tyreus-Luyben
    float tl_kP = Ku / 2.2f;
    float tl_kI = tl_kP / (2.2f * Tu);
    float tl_kD = tl_kP * Tu / 6.3f;
    Serial1.print("$TR:TL,KP="); Serial1.print(tl_kP, 2);
    Serial1.print(",KI=");       Serial1.print(tl_kI, 2);
    Serial1.print(",KD=");       Serial1.print(tl_kD, 3);
    Serial1.println();

    // PI only
    float pi_kP = 0.45f * Ku;
    float pi_kI = pi_kP / (0.83f * Tu);
    Serial1.print("$TR:PI,KP="); Serial1.print(pi_kP, 2);
    Serial1.print(",KI=");       Serial1.print(pi_kI, 2);
    Serial1.println(",KD=0.000");

    Serial1.println("$TDONE:autotune");
}

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

        float steer_f = constrain(diff / (float)FAR_DIST, -1.0f, 1.0f);
        int steer_val = (int)(steer_f * 1000.0f);
        car.write_steer(steer_val);

        Serial1.print("$T:REACT,L="); Serial1.print(L);
        Serial1.print(",FL=");         Serial1.print(FL);
        Serial1.print(",FR=");         Serial1.print(FR);
        Serial1.print(",R=");          Serial1.print(R);
        Serial1.print(",steer=");      Serial1.print(steer_val);
        Serial1.println();
        delay(50);
    }

    car.write_steer(0);
    Serial1.println("$TDONE:reactive");
}

// ─── Manual drive command ─────────────────────────────────────────────────

static void cmd_drv(const char* args) {
    // Parse "$DRV:<steer>,<speed>" — fire-and-forget (no response)
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

// ─── Start / Stop / Status / Test commands ────────────────────────────────

static void cmd_start() {
    car.pid_integral   = 0;
    car.pid_prev_error = 0;
    car.pid_filtered   = 0;
    car.pid_prev_ms    = 0;
#if USE_IMU
    car.reset_heading();
#endif
    Serial1.println("$ACK");

    // 5-second countdown — idle telemetry flows, $STOP aborts
    unsigned long start_at = millis() + 5000;
    while (millis() < start_at) {
        car.poll_lidars();
        if (wifi_check_abort()) return;
        send_idle_telemetry();
        delay(cfg_loop_ms);
    }
    car_running = true;
    Serial1.println("$STS:RUN");
}

static void cmd_stop() {
    car_running = false;
    car.write_speed(0);
    car.write_steer(0);
    Serial1.println("$ACK");
    Serial1.println("$STS:STOP");
}

static void cmd_status() {
    Serial1.println(car_running ? "$STS:RUN" : "$STS:STOP");
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
    else {
        Serial1.print("$NAK:unknown_test:");
        Serial1.println(name);
    }
}

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
    else if (strncmp(line, "$TEST:", 6) == 0) cmd_test(line + 6);
    else if (strncmp(line, "$DRV:", 5) == 0) cmd_drv(line + 5);
    // Unknown commands silently ignored
}

static void process_commands() {
    while (Serial1.available() > 0) {
        char c = (char)Serial1.read();
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
static void send_idle_telemetry() {
    car.poll_lidars();
#if USE_IMU
    car.imu_update();
#endif
    int* s = car.read_sensors();
    Serial1.print(millis());              Serial1.print(',');
    Serial1.print(s[0]);                  Serial1.print(',');
    Serial1.print(s[1]);                  Serial1.print(',');
    Serial1.print(s[2]);                  Serial1.print(',');
    Serial1.print(s[3]);                  Serial1.print(',');
    Serial1.print(0);                     Serial1.print(',');
    Serial1.print(get_speed(), 2);        Serial1.print(',');
    Serial1.print(0.0, 1);
#if USE_IMU
    Serial1.print(',');
    Serial1.print(car.yaw_rate, 1);       Serial1.print(',');
    Serial1.print(car.heading, 1);
#endif
    Serial1.println();
}

#endif  // USE_WIFI_DEBUG

// ─── Stuck / reverse helpers ──────────────────────────────────────────────────
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

// ─── Main control logic ───────────────────────────────────────────────────────
void work() {
    // Bring LiDAR data up to date before reading
    car.poll_lidars();
#if USE_IMU
    car.imu_update();
#endif
    int* s = car.read_sensors();

    // ── Steering ──────────────────────────────────────────────────────────────
    int diff;
    bool f_l = s[1] < cfg_front_obstacle_dist;  // front-left blocked
    bool f_r = s[2] < cfg_front_obstacle_dist;  // front-right blocked

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

    // ── Speed ─────────────────────────────────────────────────────────────────
    int how_clear = (int)f_l + (int)f_r;  // 0 = path clear, 1-2 = blocked
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
#if USE_WIFI_DEBUG
    Serial1.print(millis());              Serial1.print(',');
    Serial1.print(s[0]);                  Serial1.print(',');
    Serial1.print(s[1]);                  Serial1.print(',');
    Serial1.print(s[2]);                  Serial1.print(',');
    Serial1.print(s[3]);                  Serial1.print(',');
    Serial1.print((int)(diff * coef));    Serial1.print(',');
    Serial1.print(get_speed(), 2);        Serial1.print(',');
    Serial1.print(spd, 1);
#if USE_IMU
    Serial1.print(',');
    Serial1.print(car.yaw_rate, 1);       Serial1.print(',');
    Serial1.print(car.heading, 1);
#endif
    Serial1.println();
#endif

    // ── Stuck detection ───────────────────────────────────────────────────────
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

    // ── Wrong-direction / dead-end detection ───────────────────────────────
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

// ─── Setup / loop ─────────────────────────────────────────────────────────────
unsigned long next_loop = 0;

void setup() {
    Serial.begin(115200);

    // Try loading saved settings from EEPROM (falls back to compile-time defaults)
    EEPROM.begin(256);
    load_settings();

    car.init();
#if USE_IMU
    car.imu_init();
#endif
#if USE_WIFI_DEBUG
    Serial1.setTX(DEBUG_TX_PIN);
    Serial1.setRX(DEBUG_RX_PIN);
    Serial1.begin(115200);
    Serial1.println("#ms,s0,s1,s2,s3,steer,speed,target"
#if USE_IMU
                    ",yaw,heading"
#endif
                    );
#endif
    delay(3700);   // allow ESC to arm and LiDARs to start streaming
}

void loop() {
    // Drain LiDAR bytes even between control ticks
    car.poll_lidars();

#if USE_WIFI_DEBUG
    // Process incoming dashboard commands
    process_commands();
#endif

    unsigned long now = millis();
    if (now >= next_loop) {
        next_loop = max(now, next_loop + (unsigned long)cfg_loop_ms);
        if (car_running) {
            if (manual_mode && (millis() - last_drv_ms < 500)) {
                // Manual drive mode — apply $DRV commands directly
                car.poll_lidars();
#if USE_IMU
                car.imu_update();
#endif
                int* s = car.read_sensors();
                car.write_steer(manual_steer);
                car.write_speed_ms(manual_speed);
                car.pid_control_motor();
#if USE_WIFI_DEBUG
                Serial1.print(millis());              Serial1.print(',');
                Serial1.print(s[0]);                  Serial1.print(',');
                Serial1.print(s[1]);                  Serial1.print(',');
                Serial1.print(s[2]);                  Serial1.print(',');
                Serial1.print(s[3]);                  Serial1.print(',');
                Serial1.print(manual_steer);          Serial1.print(',');
                Serial1.print(get_speed(), 2);        Serial1.print(',');
                Serial1.print(manual_speed, 1);
#if USE_IMU
                Serial1.print(',');
                Serial1.print(car.yaw_rate, 1);       Serial1.print(',');
                Serial1.print(car.heading, 1);
#endif
                Serial1.println();
#endif
            } else {
                manual_mode = false;
                work();
            }
        }
#if USE_WIFI_DEBUG
        else {
            send_idle_telemetry();
        }
#endif
    }
}
