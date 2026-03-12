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

// ─── IMU configuration ─────────────────────────────────────────────────────
#define USE_IMU         1       // 1 = enable MPU-6050 gyro, 0 = disable
#define RACE_CW         true    // true = clockwise, false = counter-clockwise
#define WRONG_DIR_DEG   120.0f  // degrees in wrong direction before recovery

#include "luna_car.h"
// #include "tests.h"   // uncomment for bench testing; remove for competition

#define LOOP_MS  40   // control loop period in milliseconds

Car car;

// ─── Obstacle thresholds ─────────────────────────────────────────────────────
// All values in cm×10
#define FRONT_OBSTACLE_DIST  1200  // 120 cm — start steering around obstacle
#define SIDE_OPEN_DIST       1000  // 100 cm — side is considered "open"
#define ALL_CLOSE_DIST        800  //  80 cm — surrounded, force turn
#define CLOSE_FRONT_DIST      201  //  20 cm — emergency reverse

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
    bool f_l = s[1] < FRONT_OBSTACLE_DIST;  // front-left blocked
    bool f_r = s[2] < FRONT_OBSTACLE_DIST;  // front-right blocked

    if (s[0] > SIDE_OPEN_DIST && s[3] > SIDE_OPEN_DIST) {
        // Both sides open — keep to right wall
        diff = 800;
    } else {
        // Balance between walls
        diff = s[3] - s[0];  // positive → steer right (away from right wall)
    }

    // All sensors close: hard turn to escape
    if (s[0] < ALL_CLOSE_DIST && s[1] < ALL_CLOSE_DIST &&
        s[2] < ALL_CLOSE_DIST && s[3] < ALL_CLOSE_DIST) {
        diff = 800;
    }

    // ── Speed ─────────────────────────────────────────────────────────────────
    int how_clear = (int)f_l + (int)f_r;  // 0 = path clear, 1-2 = blocked
    float coef, spd;

    switch (how_clear) {
        case 0:
            coef = 0.3f; spd = 2.7f;   // clear ahead — drive fast, gentle steering
            break;
        default:
            coef = 0.7f; spd = 0.8f;   // obstacle — slow down, steer harder
            break;
    }

    car.write_steer((int)(diff * coef));
    car.write_speed_ms(spd);
    car.pid_control_motor();

    // ── Stuck detection ───────────────────────────────────────────────────────
    bool c_fl = s[1] < CLOSE_FRONT_DIST;
    bool c_fr = s[2] < CLOSE_FRONT_DIST;
    bool low_speed = get_speed() < 0.1f;

    static int stuck_time = 0;
    if (c_fl || c_fr || low_speed) {
        stuck_time++;
    } else {
        stuck_time = 0;
    }

    if (stuck_time > 25) {
        car.write_steer(0);
        go_back();
        car.write_speed_ms(2.0f);
        stuck_time = 0;
#if USE_IMU
        car.reset_heading();
#endif
    }

    // ── U-turn / dead-end detection ───────────────────────────────────────────
    // Accumulates signed turning work; a large negative value means the car
    // has been steering hard in one direction for too long → likely a dead end.
    static float turns = 0.0f;
    turns += diff * get_speed() / -1000.0f;
    turns = constrain(turns, -1500.0f, 50.0f);

    if (turns < -18.0f) {
        car.write_speed(0);
        delay(100);
        car.write_steer(1000);
        delay(20);
        go_back_long();
        car.write_steer(-700);
        car.write_speed_ms(2.0f);
        unsigned long strt = millis();
        while ((millis() - strt) < 900) {
            car.poll_lidars();
            car.pid_control_motor();
        }
        turns = 0.0f;
#if USE_IMU
        car.reset_heading();
#endif
    }

    // ── Wrong-direction detection (IMU gyro) ────────────────────────────────
#if USE_IMU
    {
        static float wd = 0.0f;
        wd += car.yaw_rate * (LOOP_MS / 1000.0f);

        // Decay correct-direction accumulation so normal laps don't build up
        if ( RACE_CW && wd < 0.0f) wd *= 0.97f;
        if (!RACE_CW && wd > 0.0f) wd *= 0.97f;
        wd = constrain(wd, -200.0f, 200.0f);

        bool wrong = RACE_CW ? (wd > WRONG_DIR_DEG)
                              : (wd < -WRONG_DIR_DEG);
        if (wrong) {
            car.write_speed(0);
            delay(100);
            car.write_steer(RACE_CW ? 1000 : -1000);
            delay(20);
            go_back_long();
            car.write_steer(RACE_CW ? -700 : 700);
            car.write_speed_ms(2.0f);
            unsigned long t0 = millis();
            while ((millis() - t0) < 900) {
                car.poll_lidars();
                car.imu_update();
                car.pid_control_motor();
            }
            wd = 0.0f;
            car.reset_heading();
        }
    }
#endif
}

// ─── Setup / loop ─────────────────────────────────────────────────────────────
unsigned long next_loop = 0;

void setup() {
    Serial.begin(115200);
    car.init();
#if USE_IMU
    car.imu_init();
#endif
    delay(3700);   // allow ESC to arm and LiDARs to start streaming
}

void loop() {
    // Drain LiDAR bytes even between control ticks
    car.poll_lidars();

    unsigned long now = millis();
    if (now >= next_loop) {
        next_loop = max(now, next_loop + LOOP_MS);
        work();
    }
}
