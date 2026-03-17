#pragma once
// ═════════════════════════════════════════════════════════════════════════════
// hw_esp32s3.h — ESP32-S3 single-chip hardware layer for Umbreon
//
// Replaces both luna_car.h (RP2350 hardware) and wifi_debug.ino (ESP8266 WiFi
// bridge) when building for ESP32-S3.  One chip does everything:
//   - 6× VL53L0X ToF sensors via I2C (XSHUT-based address assignment)
//   - MPU-6050 IMU via I2C (same bus as sensors, address 0x68)
//   - Servo steering + ESC motor via PWM (LEDC)
//   - Optical encoder via GPIO interrupt
//   - WiFi AP/STA with HTTP + WebSocket + TCP servers (built-in)
//
// The Car class has the same public interface as the RP2350 version so that
// Umbreon_roborace.ino and tests.h work unchanged (s[0]–s[3] keep their
// original roles; s[4]=Front and s[5]=Rear are additive).
//
// ── Prerequisites ──────────────────────────────────────────────────────────
// Board package: arduino-esp32 (ESP32-S3 Dev Module)
// Libraries:     ESP32Servo, WebSockets (by Markus Sattler), VL53L0X (Pololu)
//
// ── VL53L0X wiring ─────────────────────────────────────────────────────────
// All 6 sensors share one I2C bus (SDA/SCL).  Each sensor has its own XSHUT
// pin.  At boot all sensors start at the default address 0x29.  The firmware
// holds all XSHUT lines LOW, then releases them one at a time and assigns
// unique addresses.  No manual pre-configuration needed.
//
// All 6 sensors are mounted on the front bumper:
//
// Sensor layout (index → position → angle → XSHUT GPIO → I2C address):
//   [0] Left         −90°  GPIO 4   0x30
//   [1] Front-Left   −45°  GPIO 5   0x31
//   [2] Front-L        0°  GPIO 6   0x32
//   [3] Front-R        0°  GPIO 7   0x33
//   [4] Front-Right  +45°  GPIO 12  0x34
//   [5] Right        +90°  GPIO 14  0x35
// ═════════════════════════════════════════════════════════════════════════════

#if PLATFORM_ESP32S3

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <EEPROM.h>
#include <VL53L0X.h>

// Web UI (shared with ESP8266 version — PROGMEM is memory-mapped on ESP32)
#include "wifi_debug/web_ui.h"

// ─── Pin assignments (ESP32-S3-DevKitC-1) ──────────────────────────────────
//
// I2C bus — shared between 6× VL53L0X + MPU-6050 IMU:
//   SDA  GPIO 8
//   SCL  GPIO 9
//
// VL53L0X XSHUT pins (active-LOW shutdown, all on front bumper):
//   [0] Left  (−90°)       GPIO 4
//   [1] Front-Left (−45°)  GPIO 5
//   [2] Front-L (0°)       GPIO 6
//   [3] Front-R (0°)       GPIO 7
//   [4] Front-Right (+45°) GPIO 12
//   [5] Right (+90°)       GPIO 14
//
// Actuators:
//   Steering servo   GPIO 10  (PWM via LEDC)
//   Motor ESC        GPIO 11  (PWM via LEDC)
//   Tachometer       GPIO 13  (RISING interrupt)
//
// Status LED (WS2812 RGB NeoPixel on ESP32-S3-DevKitC-1):
//   GPIO 48  — use neopixelWrite() (built into ESP32 Arduino core)

#define I2C_SDA_PIN      8
#define I2C_SCL_PIN      9
#define SERVO_PIN       10
#define MOTOR_PIN       11
#define TAHO_PIN        13
#define RGB_LED_PIN     48

#define NUM_TOF_SENSORS  6

// ─── VL53L0X XSHUT pins and I2C addresses ──────────────────────────────────
static const uint8_t XSHUT_PINS[NUM_TOF_SENSORS]   = {4, 5, 6, 7, 12, 14};
static const uint8_t TOF_I2C_ADDR[NUM_TOF_SENSORS]  = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35};
static VL53L0X      tof_sensor[NUM_TOF_SENSORS];

// ─── IMU ───────────────────────────────────────────────────────────────────
#if USE_IMU
#define MPU6050_ADDR   0x68
#define GYRO_FS_500    0x08       // ±500°/s full-scale
#define GYRO_SENS      65.5f     // LSB/(°/s) at ±500°/s
#endif

// ─── WiFi configuration ───────────────────────────────────────────────────
// Copy wifi_config_example.h → wifi_config.h and edit your credentials.
// wifi_config.h is git-ignored so secrets stay local.
// If the file doesn't exist, safe defaults below are used (AP mode).
#if __has_include("wifi_config.h")
  #include "wifi_config.h"
#endif

#ifndef WIFI_MODE_SETTING
  #define WIFI_MODE_SETTING  WIFI_AP
#endif
#ifndef AP_SSID
  #define AP_SSID            "Umbreon"
#endif
#ifndef AP_PASS
  #define AP_PASS            "12345678"
#endif
#ifndef STA_SSID
  #define STA_SSID           "YourWiFi"
#endif
#ifndef STA_PASS
  #define STA_PASS           "YourPassword"
#endif
#ifndef STA_TIMEOUT_S
  #define STA_TIMEOUT_S      15
#endif

// Server ports (same as ESP8266 version)
#define HTTP_PORT    80
#define WS_PORT      81
#define TCP_PORT     23

// ─── Steering / ESC externs (defined in .ino) ──────────────────────────────
extern int   cfg_neutral_point;
extern int   cfg_min_point;
extern int   cfg_max_point;

#define NEUTRAL_SPEED  90
extern int   cfg_min_speed;
extern int   cfg_max_speed;
extern int   cfg_min_bspeed;

extern float cfg_pid_kp;
extern float cfg_pid_ki;
extern float cfg_pid_kd;

extern int   cfg_encoder_holes;
extern float cfg_wheel_diam_m;

extern float cfg_bat_multiplier;  // (R1+R2)/R2 — depends on your divider

// ─── Battery ADC ─────────────────────────────────────────────────────────
#define BAT_PIN     26   // GPIO 26 = ADC
#define BAT_EMA     0.05f // slow EMA for stable reading

// ─── Tachometer / speed ───────────────────────────────────────────────────
// Identical logic to RP2350 — encoder ISR + interval-based speed calc.
volatile unsigned long _taho_count = 0;
volatile unsigned long _taho_last  = 0;
volatile unsigned long _taho_iv    = 0;

void IRAM_ATTR taho_interrupt() {
    unsigned long now   = micros();
    unsigned long delta = now - _taho_last;
    if (delta < 500UL) return;   // debounce 500µs
    _taho_count++;
    _taho_iv   = delta;
    _taho_last = now;
}

float get_speed() {
    unsigned long elapsed = (unsigned long)(micros() - _taho_last);
    elapsed = max(elapsed, (unsigned long)_taho_iv);
    if (_taho_iv == 0 || elapsed > 500000UL) return 0.0f;
    return ((float)M_PI * cfg_wheel_diam_m) /
           ((float)cfg_encoder_holes * ((float)elapsed / 1e6f));
}

// ─── WiFi server instances ────────────────────────────────────────────────
static WebServer         httpServer(HTTP_PORT);
static WebSocketsServer  wsServer = WebSocketsServer(WS_PORT);
static WiFiServer        tcpServer(TCP_PORT);

#define MAX_TCP_CLIENTS 4
static WiFiClient tcpClients[MAX_TCP_CLIENTS];

// RGB LED state
static unsigned long led_prev_ms = 0;
static bool          led_state   = false;

// RGB LED helper — low brightness to avoid blinding
static void rgb_led(uint8_t r, uint8_t g, uint8_t b) {
    neopixelWrite(RGB_LED_PIN, r, g, b);
}

// ─── TelemetryStream ──────────────────────────────────────────────────────
// Stream subclass that replaces Serial1 (UART to ESP8266) on the RP2350.
// Output: line-buffered, broadcasts complete lines to all TCP + WS clients.
// Input:  ring buffer fed by WiFi event handlers (commands from dashboard).
//
// Usage: `telem.print(...)` / `telem.available()` / `telem.read()` — same
// API as Serial1, so the main .ino code works unchanged.
class TelemetryStream : public Stream {
private:
    // ── Output (line-buffered broadcast) ──
    char outBuf[300];
    int  outPos = 0;

    // ── Input (command ring buffer) ──
    char inBuf[512];
    volatile int inHead = 0;
    int inTail = 0;

    void broadcastLine() {
        // TCP clients
        for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
            if (tcpClients[i] && tcpClients[i].connected()) {
                tcpClients[i].write((const uint8_t*)outBuf, outPos);
            }
        }
        // WebSocket clients
        wsServer.broadcastTXT(outBuf, outPos);
    }

public:
    // Inject received data from WiFi event handlers
    void feedInput(const char* data, size_t len) {
        for (size_t i = 0; i < len; i++) {
            int next = (inHead + 1) % (int)sizeof(inBuf);
            if (next != inTail) {
                inBuf[inHead] = data[i];
                inHead = next;
            }
        }
    }

    // ── Stream input interface ──
    int available() override {
        return (inHead - inTail + (int)sizeof(inBuf)) % (int)sizeof(inBuf);
    }

    int read() override {
        if (inHead == inTail) return -1;
        char c = inBuf[inTail];
        inTail = (inTail + 1) % (int)sizeof(inBuf);
        return c;
    }

    int peek() override {
        if (inHead == inTail) return -1;
        return inBuf[inTail];
    }

    // ── Print output interface ──
    size_t write(uint8_t c) override {
        if (outPos < (int)sizeof(outBuf) - 1) {
            outBuf[outPos++] = c;
        }
        if (c == '\n') {
            outBuf[outPos] = '\0';
            broadcastLine();
            outPos = 0;
        }
        return 1;
    }

    size_t write(const uint8_t* buf, size_t size) override {
        for (size_t i = 0; i < size; i++) write(buf[i]);
        return size;
    }
};

TelemetryStream _telem_stream;

// ─── WiFi event handlers ──────────────────────────────────────────────────
static void ws_event(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
    switch (type) {
        case WStype_TEXT:
            // Dashboard command → feed into command ring buffer
            _telem_stream.feedInput((const char*)payload, len);
            _telem_stream.feedInput("\n", 1);
            break;
        default:
            break;
    }
}

// ─── WiFi setup ───────────────────────────────────────────────────────────
static void start_ap() {
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("AP started: ");
    Serial.println(WiFi.softAPIP());
}

void wifi_setup() {
    rgb_led(0, 0, 10);  // dim blue — booting

    if (WIFI_MODE_SETTING == WIFI_STA) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(STA_SSID, STA_PASS);
        Serial.print("Connecting to ");
        Serial.print(STA_SSID);
        unsigned long start = millis();
        bool blink = false;
        while (WiFi.status() != WL_CONNECTED) {
            if (millis() - start > STA_TIMEOUT_S * 1000UL) {
                Serial.println("\nSTA failed, falling back to AP");
                WiFi.disconnect();
                WiFi.mode(WIFI_AP);
                start_ap();
                break;
            }
            blink = !blink;
            rgb_led(blink ? 10 : 0, 0, blink ? 0 : 10);  // red/blue blink
            delay(100);
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.print("\nConnected: ");
            Serial.println(WiFi.localIP());
        }
    } else {
        WiFi.mode(WIFI_AP);
        start_ap();
    }

    // HTTP — serve web dashboard
    httpServer.on("/", []() {
        httpServer.send(200, "text/html", PAGE_HTML);
    });
    httpServer.begin();

    // WebSocket — real-time bidirectional
    wsServer.begin();
    wsServer.onEvent(ws_event);

    // TCP — backward compat with Python dashboard / ROS2
    tcpServer.begin();
    tcpServer.setNoDelay(true);

    Serial.println("Servers started (HTTP:80, WS:81, TCP:23)");
}

// ─── WiFi loop (call every iteration) ─────────────────────────────────────
void wifi_loop() {
    httpServer.handleClient();
    wsServer.loop();

    // Accept new TCP clients
    if (tcpServer.hasClient()) {
        int slot = -1;
        for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
            if (!tcpClients[i] || !tcpClients[i].connected()) {
                slot = i;
                break;
            }
        }
        if (slot >= 0) {
            tcpClients[slot] = tcpServer.accept();
            tcpClients[slot].setNoDelay(true);
            IPAddress ip = (WiFi.getMode() & WIFI_AP)
                         ? WiFi.softAPIP() : WiFi.localIP();
            tcpClients[slot].printf("# Umbreon ESP32-S3  IP %s\r\n",
                                    ip.toString().c_str());
        } else {
            tcpServer.accept().stop();   // reject — no free slot
        }
    }

    // Read TCP client data into command buffer
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (tcpClients[i] && tcpClients[i].connected()) {
            while (tcpClients[i].available()) {
                char c = tcpClients[i].read();
                _telem_stream.feedInput(&c, 1);
            }
        }
    }

    // RGB LED status
    int clients = wsServer.connectedClients();
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (tcpClients[i] && tcpClients[i].connected()) clients++;
    }
    unsigned long now = millis();
    if (clients == 0) {
        // No clients — slow blue blink
        if (now - led_prev_ms > 1000) {
            led_prev_ms = now;
            led_state = !led_state;
            rgb_led(0, 0, led_state ? 10 : 0);
        }
    } else {
        // Client connected — steady green
        rgb_led(0, 10, 0);
    }

    // STA auto-reconnect
    if (WIFI_MODE_SETTING == WIFI_STA && WiFi.status() != WL_CONNECTED) {
        static unsigned long last_retry = 0;
        if (now - last_retry > 5000) {
            last_retry = now;
            WiFi.reconnect();
        }
    }
}

// ─── VL53L0X boot-time address assignment ────────────────────────────────
// All sensors start at 0x29 (factory default).  We hold all XSHUT LOW to
// put every sensor in hardware standby, then release them one at a time,
// assign a unique address, and start continuous ranging.
static void init_tof_sensors(bool* ok_out) {
    // 1. Shut down all sensors
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        pinMode(XSHUT_PINS[i], OUTPUT);
        digitalWrite(XSHUT_PINS[i], LOW);
    }
    delay(10);

    // 2. Enable one at a time, assign address, start ranging
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        digitalWrite(XSHUT_PINS[i], HIGH);
        delay(10);   // VL53L0X boot time

        tof_sensor[i].setTimeout(100);  // short timeout — don't block WiFi loop
        if (!tof_sensor[i].init()) {
            ok_out[i] = false;
            Serial.print("VL53L0X ");
            Serial.print(i);
            Serial.println(" init failed");
            continue;
        }

        tof_sensor[i].setAddress(TOF_I2C_ADDR[i]);
        tof_sensor[i].setMeasurementTimingBudget(33000);  // 33 ms — fast + accurate
        tof_sensor[i].startContinuous();
        ok_out[i] = true;
    }
}

// ─── Car class ───────────────────────────────────────────────────────────
// Same public interface as the RP2350 version (luna_car.h).
// Internals differ: VL53L0X via I2C, ESP32Servo for PWM, shared I2C bus.
// Sensor indices: [0]=Left(−90°), [1]=FL(−45°), [2]=Front-L(0°), [3]=Front-R(0°), [4]=FR(+45°), [5]=Right(+90°)
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
    float         yaw_rate     = 0.0f;
    float         heading      = 0.0f;
    unsigned long imu_prev_us  = 0;
#endif

    float bat_voltage = 0.0f;
    unsigned long bat_prev_ms  = 0;

    const int sensor_amount = NUM_TOF_SENSORS;

    void init();
    void write_speed(int s);
    void write_speed_ms(float s);
    void pid_control_motor();
    void write_steer(int s);
    int* read_sensors();
    void poll_lidars();
    void bat_update();
    bool sensor_ok(int i) const { return (i >= 0 && i < NUM_TOF_SENSORS) ? _tof_ok[i] : false; }

#if USE_IMU
    bool imu_init();
    void imu_calibrate();
    void imu_update();
    void reset_heading() { heading = 0.0f; }
    float gyro_bias = 0.0f;
#endif

private:
    uint16_t _tof_dist[NUM_TOF_SENSORS]     = {};
    bool     _tof_ok[NUM_TOF_SENSORS]       = {};
    uint8_t  _tof_fail_cnt[NUM_TOF_SENSORS] = {};  // consecutive timeout counter
};

// ─── Car::init ───────────────────────────────────────────────────────────
void Car::init() {
    // I2C bus — shared between 6× VL53L0X + MPU-6050
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    // VL53L0X sensors — XSHUT address assignment + continuous ranging
    init_tof_sensors(_tof_ok);

    // Report sensor init status
    int ok_count = 0;
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        if (_tof_ok[i]) ok_count++;
    }
    Serial.printf("VL53L0X: %d/%d sensors OK\n", ok_count, NUM_TOF_SENSORS);
    if (ok_count < NUM_TOF_SENSORS) {
        Serial.print("FAILED sensors:");
        for (int i = 0; i < NUM_TOF_SENSORS; i++) {
            if (!_tof_ok[i]) { Serial.print(' '); Serial.print(i); }
        }
        Serial.println();
    }

    // Servo + ESC
    steer_servo.attach(SERVO_PIN);
    motor_esc.attach(MOTOR_PIN);
    write_steer(0);
    write_speed(0);

    // Tachometer
    pinMode(TAHO_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(TAHO_PIN), taho_interrupt, RISING);

    // Battery ADC
    pinMode(BAT_PIN, INPUT);
}

// ─── VL53L0X polling (non-blocking) ──────────────────────────────────────
// Checks RESULT_INTERRUPT_STATUS before reading — skips sensors whose data
// isn't ready yet, so the loop never blocks on I2C.  If a sensor times out
// 10 consecutive polls it is marked as failed and excluded from future reads.
void Car::poll_lidars() {
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        if (!_tof_ok[i]) continue;

        // Non-blocking: check if measurement result is ready
        if ((tof_sensor[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
            _tof_fail_cnt[i]++;
            if (_tof_fail_cnt[i] > 10) {
                _tof_ok[i] = false;  // disable after 10 consecutive misses
                Serial.print("VL53L0X "); Serial.print(i); Serial.println(" disabled (timeout)");
            }
            continue;  // data not ready — skip, don't block
        }

        // Data ready — read range and clear interrupt
        uint16_t d = tof_sensor[i].readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
        tof_sensor[i].writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);

        if (d < 8190) {
            _tof_dist[i] = d;   // mm = cm×10
            _tof_fail_cnt[i] = 0;  // reset error counter on success
        }
        yield();   // let WiFi stack service connections between reads
    }
}

// Returns distances as cm×10 (same units as RP2350 version).
// VL53L0X reports millimeters; 1 mm == 1 in cm×10 (120 cm = 1200 mm = 1200 cm×10).
int* Car::read_sensors() {
    static int values[NUM_TOF_SENSORS];
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        values[i] = _tof_ok[i] ? (int)_tof_dist[i] : 9999;
    }
    return values;
}

// ─── Motor control (identical to RP2350) ─────────────────────────────────
void Car::pid_control_motor() {
    unsigned long now_ms = millis();
    if (pid_prev_ms == 0) { pid_prev_ms = now_ms; return; }

    float dt = (now_ms - pid_prev_ms) / 1000.0f;
    if (dt < 0.01f) return;
    pid_prev_ms = now_ms;

    noInterrupts();
    unsigned long cnt  = _taho_count;
    unsigned long last = _taho_last;
    interrupts();

    unsigned long delta_cnt = cnt - pid_prev_cnt;
    pid_prev_cnt = cnt;

    float raw_speed = (delta_cnt / (float)cfg_encoder_holes) *
                      ((float)M_PI * cfg_wheel_diam_m) / dt;

    pid_filtered = 0.5f * raw_speed + 0.5f * pid_filtered;
    if ((micros() - last) > 500000UL) pid_filtered = 0;

    float error = target_speed - pid_filtered;
    pid_integral += error * dt;
    pid_integral = constrain(pid_integral, -5.0f, 5.0f);
    float deriv = (error - pid_prev_error) / dt;
    pid_prev_error = error;

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
    pid_integral = 0; pid_prev_error = 0; pid_filtered = 0; pid_prev_ms = 0;
    s = constrain(s, -1000, 1000);
    if      (s > 0) s = map(s,     1, 1000, cfg_min_speed,  cfg_max_speed);
    else if (s < 0) s = map(s, -1000,   -1, 0,              cfg_min_bspeed);
    else            s = NEUTRAL_SPEED;
    motor_esc.write(s);
}

// ─── Steering (identical to RP2350) ──────────────────────────────────────
void Car::write_steer(int s) {
    extern bool cfg_servo_reverse;
    if (cfg_servo_reverse) s = -s;   // invert so positive = right (depends on servo mounting)
    s = constrain(s, -1000, 1000);
    if (s < 0) s = map(s, -1000, 0,    cfg_min_point,     cfg_neutral_point);
    else       s = map(s,     0, 1000, cfg_neutral_point,  cfg_max_point);
    steer_servo.write(s);
}

// ─── Battery voltage (ADC + resistor divider) ────────────────────────────
void Car::bat_update() {
    unsigned long now = millis();
    if (now - bat_prev_ms < 500) return;
    bat_prev_ms = now;

    int raw = analogRead(BAT_PIN);
    float v_adc = raw * (3.3f / 4095.0f);
    float v_bat = v_adc * cfg_bat_multiplier;

    if (bat_voltage < 0.1f) bat_voltage = v_bat;
    else bat_voltage = BAT_EMA * v_bat + (1.0f - BAT_EMA) * bat_voltage;
}

// ─── IMU (same MPU-6050, shared I2C bus) ─────────────────────────────────
#if USE_IMU
bool Car::imu_init() {
    // Wire.begin() already called in Car::init() — don't call again
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);            // PWR_MGMT_1
    Wire.write(0x00);            // wake from sleep
    if (Wire.endTransmission() != 0) { imu_ok = false; return false; }

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);            // GYRO_CONFIG
    Wire.write(GYRO_FS_500);     // ±500°/s
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1A);            // CONFIG — DLPF ~42 Hz
    Wire.write(0x03);
    Wire.endTransmission();

    imu_ok      = true;
    imu_prev_us = micros();
    return true;
}

void Car::imu_calibrate() {
    if (!imu_ok) return;
    float sum = 0.0f;
    const int N = 200;
    for (int i = 0; i < N; i++) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x47);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
        if (Wire.available() >= 2) {
            int16_t raw = ((int16_t)Wire.read() << 8) | Wire.read();
            sum += raw / GYRO_SENS;
        }
        delay(5);
    }
    gyro_bias = sum / (float)N;
}


#define IMU_EMA_ALPHA  0.3f    // EMA smoothing (0.0=max smooth, 1.0=no filter)
#define IMU_DEADZONE   0.4f    // ignore rates below this (°/s) — kills drift when stationary

void Car::imu_update() {
    if (!imu_ok) return;

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x47);            // GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return;

    extern bool cfg_imu_rotate;
    int16_t raw_z = ((int16_t)Wire.read() << 8) | Wire.read();
    float raw_rate = (raw_z / GYRO_SENS) - gyro_bias;
    if (cfg_imu_rotate) raw_rate = -raw_rate;

    // Dead zone — zero out noise when nearly stationary
    if (fabsf(raw_rate) < IMU_DEADZONE) raw_rate = 0.0f;

    // EMA low-pass filter
    yaw_rate = IMU_EMA_ALPHA * raw_rate + (1.0f - IMU_EMA_ALPHA) * yaw_rate;

    unsigned long now = micros();
    float dt = (now - imu_prev_us) / 1e6f;
    imu_prev_us = now;

    if (dt > 0.0f && dt < 0.5f)
        heading += yaw_rate * dt;
}
#endif

#endif // PLATFORM_ESP32S3
