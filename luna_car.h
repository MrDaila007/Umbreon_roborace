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
#define TAHO_PIN    13   // Hall-effect tachometer (interrupt, RISING)

// ─── Steering limits ──────────────────────────────────────────────────────────
#define NEUTRAL_POINT  90
#define MIN_POINT      40
#define MAX_POINT      140

// ─── ESC limits ───────────────────────────────────────────────────────────────
#define NEUTRAL_SPEED  90
#define MIN_SPEED      96   // slowest forward
#define MAX_SPEED      110  // fastest forward
#define MIN_BSPEED     85   // slowest reverse

// ─── Speed PID ────────────────────────────────────────────────────────────────
#define MOTOR_P  200
#define MOTOR_D  110

// ─── Tachometer / speed ───────────────────────────────────────────────────────
volatile unsigned long _last_turnover = 0;
volatile unsigned long _turnover_time = 0;

void taho_interrupt() {
    unsigned long now   = micros();
    unsigned long delta = now - _last_turnover;
    if (delta > 20000UL) {   // debounce: ignore pulses < 20 ms apart
        _turnover_time  = delta;
        _last_turnover  = now;
    }
}

// Returns wheel speed in m/s (wheel radius = 27 mm)
float get_speed() {
    unsigned long elapsed = (unsigned long)(micros() - _last_turnover);
    elapsed = max(elapsed, _turnover_time);
    if (elapsed > _turnover_time * 3UL) return 0.0f;  // stopped
    const float WHEEL_RADIUS_M = 0.027f;
    return 2.0f * 3.14159265f / ((float)elapsed / 1e6f) * WHEEL_RADIUS_M / 10.0f;
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
    float target_speed = 0.0f;
    float last_error   = 0.0f;

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

// ─── Motor control ────────────────────────────────────────────────────────────
void Car::pid_control_motor() {
    float spd   = get_speed();
    float error = target_speed - spd;
    int   ctrl  = (int)roundf(error * MOTOR_P + (error - last_error) * MOTOR_D);
    write_speed(constrain(ctrl, 0, 100));
    last_error = error;
}

void Car::write_speed_ms(float s) {
    target_speed = s;
}

void Car::write_speed(int s) {
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
