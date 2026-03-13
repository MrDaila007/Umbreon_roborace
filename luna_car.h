#pragma once

#include <Servo.h>
#include <SerialPIO.h>

#if USE_IMU
#include <Wire.h>
#define IMU_SDA_PIN    0
#define IMU_SCL_PIN    1
#define MPU6050_ADDR   0x68
#define GYRO_FS_500    0x08       // ±500°/s full-scale
#define GYRO_SENS      65.5f     // LSB/(°/s) at ±500°/s
#endif

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

// ─── Steering limits (runtime-configurable) ─────────────────────────────────
extern int   cfg_neutral_point;
extern int   cfg_min_point;
extern int   cfg_max_point;

// ─── ESC limits (runtime-configurable) ──────────────────────────────────────
#define NEUTRAL_SPEED  90       // never changes
extern int   cfg_min_speed;     // slowest forward
extern int   cfg_max_speed;     // fastest forward
extern int   cfg_min_bspeed;    // slowest reverse

// ─── Speed PID (runtime-configurable) ───────────────────────────────────────
extern float cfg_pid_kp;
extern float cfg_pid_ki;
extern float cfg_pid_kd;

// ─── Tachometer / speed (runtime-configurable) ──────────────────────────────
// Optical encoder: disc with N holes on the central rod.
// speed (m/s) = (π × d) / (encoder_holes × pulse_period_s)
extern int   cfg_encoder_holes;
extern float cfg_wheel_diam_m;

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
    return (3.14159265f * cfg_wheel_diam_m) /
           ((float)cfg_encoder_holes * ((float)elapsed / 1e6f));
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

#if USE_IMU
    bool          imu_ok       = false;
    float         yaw_rate     = 0.0f;   // current gyro Z rate (°/s)
    float         heading      = 0.0f;   // accumulated heading change (°)
    unsigned long imu_prev_us  = 0;
#endif

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

#if USE_IMU
    bool imu_init();
    void imu_update();
    void reset_heading() { heading = 0.0f; }
#endif

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

    float raw_speed = (delta_cnt / (float)cfg_encoder_holes) *
                      (3.14159265f * cfg_wheel_diam_m) / dt;

    // EMA filter
    pid_filtered = 0.5f * raw_speed + 0.5f * pid_filtered;
    if ((micros() - last) > 500000UL) pid_filtered = 0;

    // PID
    float error = target_speed - pid_filtered;
    pid_integral += error * dt;
    pid_integral = constrain(pid_integral, -5.0f, 5.0f);
    float deriv = (error - pid_prev_error) / dt;
    pid_prev_error = error;

    // feedforward: motor dead zone below cfg_min_speed
    float ff = (target_speed > 0.01f) ? (float)(cfg_min_speed - NEUTRAL_SPEED) : 0;
    float output = ff + cfg_pid_kp * error + cfg_pid_ki * pid_integral + cfg_pid_kd * deriv;

    int esc_val = NEUTRAL_SPEED + (int)output;
    esc_val = constrain(esc_val, NEUTRAL_SPEED, cfg_max_speed);
    motor_esc.write(esc_val);
}

void Car::write_speed_ms(float s) {
    target_speed = s;
}

void Car::write_speed(int s) {
    // Reset PID state — direct control bypasses PID
    pid_integral = 0; pid_prev_error = 0; pid_filtered = 0; pid_prev_ms = 0;

    s = constrain(s, -1000, 1000);
    if      (s > 0) s = map(s,     1, 1000, cfg_min_speed,  cfg_max_speed);
    else if (s < 0) s = map(s, -1000,   -1, 0,              cfg_min_bspeed);
    else            s = NEUTRAL_SPEED;
    motor_esc.write(s);
}

// ─── Steering ─────────────────────────────────────────────────────────────────
void Car::write_steer(int s) {
    s = -s;   // invert so positive = right (match original convention)
    s = constrain(s, -1000, 1000);
    if (s < 0) s = map(s, -1000, 0,    cfg_min_point,     cfg_neutral_point);
    else       s = map(s,     0, 1000, cfg_neutral_point,  cfg_max_point);
    steer_servo.write(s);
}

// ─── IMU (MPU-6050 gyro Z) ──────────────────────────────────────────────────
#if USE_IMU
bool Car::imu_init() {
    Wire.setSDA(IMU_SDA_PIN);
    Wire.setSCL(IMU_SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);

    // Wake from sleep
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);            // PWR_MGMT_1
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) { imu_ok = false; return false; }

    // Gyro full-scale ±500°/s
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);            // GYRO_CONFIG
    Wire.write(GYRO_FS_500);
    Wire.endTransmission();

    // DLPF ~42 Hz bandwidth
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1A);            // CONFIG
    Wire.write(0x03);            // DLPF_CFG = 3
    Wire.endTransmission();

    imu_ok      = true;
    imu_prev_us = micros();
    return true;
}

void Car::imu_update() {
    if (!imu_ok) return;

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x47);            // GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return;

    int16_t raw_z = ((int16_t)Wire.read() << 8) | Wire.read();
    yaw_rate = raw_z / GYRO_SENS;

    unsigned long now = micros();
    float dt = (now - imu_prev_us) / 1e6f;
    imu_prev_us = now;

    if (dt > 0.0f && dt < 0.5f)
        heading += yaw_rate * dt;
}
#endif
