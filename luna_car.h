#pragma once

#include <Servo.h>
#include <SerialPIO.h>

// ─── Pin assignments (RP2350 / Pico 2) ───────────────────────────────────────
// LiDAR RX-only UART (TX=NOPIN):
//   s[0] = Left       → GP2
//   s[1] = Front-Left → GP3
//   s[2] = Front-Right→ GP4
//   s[3] = Right      → GP5
static const uint8_t  LIDAR_RX_PINS[4] = {2, 3, 4, 5};
static const uint32_t LIDAR_BAUD       = 115200;

#define SERVO_PIN   10   // Steering servo (PWM)
#define MOTOR_PIN   11   // Motor ESC (PWM)
#define TAHO_PIN    13   // Optical encoder on central rod (interrupt, RISING)

// ─── Steering limits ──────────────────────────────────────────────────────────
#define NEUTRAL_POINT  90
#define MIN_POINT      40
#define MAX_POINT      140

// ─── ESC limits ───────────────────────────────────────────────────────────────
#define NEUTRAL_SPEED  90
#define MIN_SPEED      96   // slowest forward
#define MAX_SPEED      110  // fastest forward
#define MIN_BSPEED     85   // slowest reverse

// ─── Speed PID (auto-tuned: Tyreus-Luyben, Ku=9.20, Tu=0.648s) ──────────────
#define PID_KP   4.18f
#define PID_KI   2.93f
#define PID_KD   0.43f

// ─── Tachometer / speed ───────────────────────────────────────────────────────
// Optical encoder: disc with 62 holes on the central rod (measured by taho_calibrate).
// Wheel diameter: 63 mm  →  circumference = π × 0.063 m
// Each pulse = 1 hole = 1/62 revolution
// speed (m/s) = (π × d) / (ENCODER_HOLES × pulse_period_s)

#define ENCODER_HOLES  62
#define WHEEL_DIAM_M   0.063f   // 63 mm

volatile unsigned long _taho_count = 0;
volatile unsigned long _taho_last  = 0;
volatile unsigned long _taho_iv    = 0;

void taho_interrupt() {
    unsigned long now   = micros();
    unsigned long delta = now - _taho_last;
    if (delta < 500UL) return;   // debounce 500µs
    _taho_count++;
    _taho_iv   = delta;
    _taho_last = now;
}

// Interval-based speed (for go_back wait loops & stuck detection)
float get_speed() {
    unsigned long elapsed = (unsigned long)(micros() - _taho_last);
    elapsed = max(elapsed, _taho_iv);
    if (_taho_iv == 0 || elapsed > 500000UL) return 0.0f;
    return (3.14159265f * WHEEL_DIAM_M) /
           ((float)ENCODER_HOLES * ((float)elapsed / 1e6f));
}

// ─── TF-Luna packet state ─────────────────────────────────────────────────────
struct LidarState {
    uint8_t  buf[9];
    uint8_t  idx;
    bool     gotHeader;
    uint16_t dist_cm;    // raw distance in centimetres
    bool     hasReading;
};

// Global SerialPIO objects (RX-only, one per LiDAR)
SerialPIO _lidar_serial0(NOPIN, LIDAR_RX_PINS[0]);
SerialPIO _lidar_serial1(NOPIN, LIDAR_RX_PINS[1]);
SerialPIO _lidar_serial2(NOPIN, LIDAR_RX_PINS[2]);
SerialPIO _lidar_serial3(NOPIN, LIDAR_RX_PINS[3]);

// ─── Car class ────────────────────────────────────────────────────────────────
class Car {
public:
    Servo steer_servo;
    Servo motor_esc;
    float target_speed   = 0.0f;
    float pid_integral   = 0.0f;
    float pid_prev_error = 0.0f;
    float pid_filtered   = 0.0f;
    unsigned long pid_prev_cnt = 0;
    unsigned long pid_prev_ms  = 0;

    const int sensor_amount = 4;

    void init();

    // Drive & steering (same interface as original big_car.h)
    void write_speed(int s);          // raw ESC value  -1000…1000
    void write_speed_ms(float s);     // set target m/s for PID
    void pid_control_motor();
    void write_steer(int s);          // -1000 (left) … +1000 (right)

    // Sensor access
    // Returns pointer to static array [4] of distances in cm×10
    // (same units as original IR-sensor code so thresholds stay identical)
    int* read_sensors();

    // Feed incoming LiDAR bytes — call every loop iteration
    void poll_lidars();

private:
    SerialPIO* _serials[4];
    LidarState _lidars[4];

    void _process_byte(int id, uint8_t b);
};

// ─── Car::init ────────────────────────────────────────────────────────────────
void Car::init() {
    steer_servo.attach(SERVO_PIN);
    motor_esc.attach(MOTOR_PIN);
    write_steer(0);
    write_speed(0);

    pinMode(TAHO_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(TAHO_PIN), taho_interrupt, RISING);

    _serials[0] = &_lidar_serial0;
    _serials[1] = &_lidar_serial1;
    _serials[2] = &_lidar_serial2;
    _serials[3] = &_lidar_serial3;

    for (int i = 0; i < 4; i++) {
        _serials[i]->begin(LIDAR_BAUD);
        _lidars[i] = {};
    }
}

// ─── TF-Luna packet parser ────────────────────────────────────────────────────
// Frame: 0x59 0x59 DistL DistH StrL StrH RawTemp RawTempH Checksum
void Car::_process_byte(int id, uint8_t b) {
    LidarState& st = _lidars[id];

    if (!st.gotHeader) {
        if      (st.idx == 0 && b == 0x59) { st.buf[st.idx++] = b; }
        else if (st.idx == 1 && b == 0x59) { st.buf[st.idx++] = b; st.gotHeader = true; }
        else                               { st.idx = 0; }
        return;
    }

    st.buf[st.idx++] = b;
    if (st.idx == 9) {
        uint8_t sum = 0;
        for (int j = 0; j < 8; j++) sum += st.buf[j];
        if (sum == st.buf[8]) {
            st.dist_cm   = (uint16_t)st.buf[2] | ((uint16_t)st.buf[3] << 8);
            st.hasReading = true;
        }
        st.idx = 0;
        st.gotHeader = false;
    }
}

void Car::poll_lidars() {
    for (int i = 0; i < 4; i++) {
        while (_serials[i]->available() > 0) {
            int c = _serials[i]->read();
            if (c >= 0) _process_byte(i, (uint8_t)c);
        }
    }
}

// Returns distances as cm×10 so existing roborace thresholds work unchanged.
// If a sensor has no valid reading yet, returns 9999 (very far / unknown).
int* Car::read_sensors() {
    static int values[4];
    for (int i = 0; i < 4; i++) {
        values[i] = _lidars[i].hasReading ? (int)_lidars[i].dist_cm * 10 : 9999;
    }
    return values;
}

// ─── Motor control (count-based PID with feedforward) ────────────────────────
void Car::pid_control_motor() {
    unsigned long now_ms = millis();
    if (pid_prev_ms == 0) { pid_prev_ms = now_ms; return; }  // first call — init

    float dt = (now_ms - pid_prev_ms) / 1000.0f;
    if (dt < 0.01f) return;   // too soon
    pid_prev_ms = now_ms;

    // count-based speed measurement
    noInterrupts();
    unsigned long cnt  = _taho_count;
    unsigned long last = _taho_last;
    interrupts();

    unsigned long delta_cnt = cnt - pid_prev_cnt;
    pid_prev_cnt = cnt;

    float raw_speed = (delta_cnt / (float)ENCODER_HOLES) *
                      (3.14159265f * WHEEL_DIAM_M) / dt;

    // EMA filter
    pid_filtered = 0.5f * raw_speed + 0.5f * pid_filtered;
    if ((micros() - last) > 500000UL) pid_filtered = 0;

    // PID
    float error = target_speed - pid_filtered;
    pid_integral += error * dt;
    pid_integral = constrain(pid_integral, -5.0f, 5.0f);
    float deriv = (error - pid_prev_error) / dt;
    pid_prev_error = error;

    // feedforward: motor dead zone below MIN_SPEED
    float ff = (target_speed > 0.01f) ? (float)(MIN_SPEED - NEUTRAL_SPEED) : 0;
    float output = ff + PID_KP * error + PID_KI * pid_integral + PID_KD * deriv;

    int esc_val = NEUTRAL_SPEED + (int)output;
    esc_val = constrain(esc_val, NEUTRAL_SPEED, MAX_SPEED);
    motor_esc.write(esc_val);
}

void Car::write_speed_ms(float s) {
    target_speed = s;
}

void Car::write_speed(int s) {
    // Reset PID state — direct control bypasses PID
    pid_integral = 0; pid_prev_error = 0; pid_filtered = 0; pid_prev_ms = 0;

    s = constrain(s, -1000, 1000);
    if      (s > 0) s = map(s,     1, 1000, MIN_SPEED,  MAX_SPEED);
    else if (s < 0) s = map(s, -1000,   -1, 0,          MIN_BSPEED);
    else            s = NEUTRAL_SPEED;
    motor_esc.write(s);
}

// ─── Steering ─────────────────────────────────────────────────────────────────
void Car::write_steer(int s) {
    s = -s;   // invert so positive = right (match original convention)
    s = constrain(s, -1000, 1000);
    if (s < 0) s = map(s, -1000, 0,    MIN_POINT,    NEUTRAL_POINT);
    else       s = map(s,     0, 1000, NEUTRAL_POINT, MAX_POINT);
    steer_servo.write(s);
}
