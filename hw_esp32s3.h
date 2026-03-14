#pragma once
// ═════════════════════════════════════════════════════════════════════════════
// hw_esp32s3.h — ESP32-S3 single-chip hardware layer for Umbreon
//
// Replaces both luna_car.h (RP2350 hardware) and wifi_debug.ino (ESP8266 WiFi
// bridge) when building for ESP32-S3.  One chip does everything:
//   - 4× TF-Luna LiDAR via I2C (not UART — each sensor needs a unique address)
//   - MPU-6050 IMU via I2C (same bus as LiDARs, address 0x68)
//   - Servo steering + ESC motor via PWM (LEDC)
//   - Optical encoder via GPIO interrupt
//   - WiFi AP/STA with HTTP + WebSocket + TCP servers (built-in)
//
// The Car class has the same public interface as the RP2350 version so that
// Umbreon_roborace.ino and tests.h work unchanged.
//
// ── Prerequisites ──────────────────────────────────────────────────────────
// Board package: arduino-esp32 (ESP32-S3 Dev Module)
// Libraries:     ESP32Servo, WebSockets (by Markus Sattler)
//
// ── TF-Luna I2C setup (one-time, per sensor) ──────────────────────────────
// Factory default: UART mode, I2C address 0x10.
// Before using this firmware, switch each sensor to I2C mode and assign
// a unique address.  Two approaches:
//
//   Option A — via UART (before switching mode):
//     1. Connect sensor to any serial port
//     2. Send: 5A 05 0A 01 6A  (set communication mode to I2C)
//     3. Send: 5A 05 0B <addr> <checksum>  (set I2C address)
//     4. Power-cycle the sensor
//
//   Option B — via I2C (if already in I2C mode):
//     Write <new_addr> to register 0x22, then write 0x01 to register 0x20
//     (save settings).  Power-cycle.
//
// Recommended addresses:
//   Left=0x10, Front-Left=0x11, Front-Right=0x12, Right=0x13
// ═════════════════════════════════════════════════════════════════════════════

#if PLATFORM_ESP32S3

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <EEPROM.h>

// Web UI (shared with ESP8266 version — PROGMEM is memory-mapped on ESP32)
#include "wifi_debug/web_ui.h"

// ─── Pin assignments (ESP32-S3-DevKitC-1) ──────────────────────────────────
//
// I2C bus — shared between 4× TF-Luna LiDAR + MPU-6050 IMU:
//   SDA  GPIO 8
//   SCL  GPIO 9
//
// Actuators:
//   Steering servo   GPIO 10  (PWM via LEDC)
//   Motor ESC        GPIO 11  (PWM via LEDC)
//   Tachometer       GPIO 13  (RISING interrupt)
//
// Status LED:
//   GPIO 2  (built-in LED — varies by board, change if needed)

#define I2C_SDA_PIN      8
#define I2C_SCL_PIN      9
#define SERVO_PIN       10
#define MOTOR_PIN       11
#define TAHO_PIN        13
#define STATUS_LED_PIN   2

// ─── TF-Luna I2C addresses ────────────────────────────────────────────────
// Sensor layout (same as RP2350 version):
//   [0] = Left       0x10
//   [1] = Front-Left 0x11
//   [2] = Front-Right 0x12
//   [3] = Right       0x13
static const uint8_t LIDAR_I2C_ADDR[4] = {0x10, 0x11, 0x12, 0x13};

// ─── IMU ───────────────────────────────────────────────────────────────────
#if USE_IMU
#define MPU6050_ADDR   0x68
#define GYRO_FS_500    0x08       // ±500°/s full-scale
#define GYRO_SENS      65.5f     // LSB/(°/s) at ±500°/s
#endif

// ─── WiFi configuration ───────────────────────────────────────────────────
// Change WIFI_MODE_SETTING to WIFI_STA to join an existing network.
// If STA connection fails, falls back to AP mode automatically.
#define WIFI_MODE_SETTING  WIFI_AP      // WIFI_AP or WIFI_STA
#define AP_SSID            "Umbreon"
#define AP_PASS            "12345678"
#define STA_SSID           "YourWiFi"
#define STA_PASS           "YourPassword"
#define STA_TIMEOUT_S      15

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
    elapsed = max(elapsed, _taho_iv);
    if (_taho_iv == 0 || elapsed > 500000UL) return 0.0f;
    return (3.14159265f * cfg_wheel_diam_m) /
           ((float)cfg_encoder_holes * ((float)elapsed / 1e6f));
}

// ─── WiFi server instances ────────────────────────────────────────────────
static WebServer         httpServer(HTTP_PORT);
static WebSocketsServer  wsServer = WebSocketsServer(WS_PORT);
static WiFiServer        tcpServer(TCP_PORT);

#define MAX_TCP_CLIENTS 4
static WiFiClient tcpClients[MAX_TCP_CLIENTS];

// LED blink state
static unsigned long led_prev_ms = 0;
static bool          led_state   = false;

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
    pinMode(STATUS_LED_PIN, OUTPUT);

    if (WIFI_MODE_SETTING == WIFI_STA) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(STA_SSID, STA_PASS);
        Serial.print("Connecting to ");
        Serial.print(STA_SSID);
        unsigned long start = millis();
        while (WiFi.status() != WL_CONNECTED) {
            if (millis() - start > STA_TIMEOUT_S * 1000UL) {
                Serial.println("\nSTA failed, falling back to AP");
                WiFi.disconnect();
                WiFi.mode(WIFI_AP);
                start_ap();
                break;
            }
            digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
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

    // LED status
    int clients = wsServer.connectedClients();
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (tcpClients[i] && tcpClients[i].connected()) clients++;
    }
    unsigned long now = millis();
    if (clients == 0) {
        if (now - led_prev_ms > 1000) {
            led_prev_ms = now;
            led_state = !led_state;
            digitalWrite(STATUS_LED_PIN, led_state);
        }
    } else {
        digitalWrite(STATUS_LED_PIN, HIGH);
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

// ─── TF-Luna I2C read ────────────────────────────────────────────────────
// In I2C mode, distance register is at 0x00–0x01 (little-endian, in cm).
// Much simpler than the UART 9-byte packet parser used on RP2350.
static uint16_t read_lidar_i2c(uint8_t addr) {
    Wire.beginTransmission(addr);
    Wire.write(0x00);               // DIST_LO register
    if (Wire.endTransmission(false) != 0) return 0;
    if (Wire.requestFrom(addr, (uint8_t)2) != 2) return 0;
    uint16_t dist = Wire.read();
    dist |= (uint16_t)Wire.read() << 8;
    return dist;                     // cm
}

// ─── Car class ───────────────────────────────────────────────────────────
// Same public interface as the RP2350 version (luna_car.h).
// Internals differ: I2C for LiDAR, ESP32Servo for PWM, shared I2C bus.
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

    const int sensor_amount = 4;

    void init();
    void write_speed(int s);
    void write_speed_ms(float s);
    void pid_control_motor();
    void write_steer(int s);
    int* read_sensors();
    void poll_lidars();

#if USE_IMU
    bool imu_init();
    void imu_update();
    void reset_heading() { heading = 0.0f; }
#endif

private:
    uint16_t _lidar_dist[4] = {0, 0, 0, 0};
    bool     _lidar_ok[4]   = {false, false, false, false};
};

// ─── Car::init ───────────────────────────────────────────────────────────
void Car::init() {
    // I2C bus — shared between 4× TF-Luna + MPU-6050
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    // Servo + ESC
    steer_servo.attach(SERVO_PIN);
    motor_esc.attach(MOTOR_PIN);
    write_steer(0);
    write_speed(0);

    // Tachometer
    pinMode(TAHO_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(TAHO_PIN), taho_interrupt, RISING);

    // Probe each LiDAR sensor on I2C
    for (int i = 0; i < 4; i++) {
        Wire.beginTransmission(LIDAR_I2C_ADDR[i]);
        _lidar_ok[i] = (Wire.endTransmission() == 0);
        if (!_lidar_ok[i]) {
            Serial.print("LiDAR ");
            Serial.print(i);
            Serial.print(" not found at 0x");
            Serial.println(LIDAR_I2C_ADDR[i], HEX);
        }
    }
}

// ─── LiDAR polling (I2C) ────────────────────────────────────────────────
void Car::poll_lidars() {
    for (int i = 0; i < 4; i++) {
        if (!_lidar_ok[i]) continue;
        uint16_t d = read_lidar_i2c(LIDAR_I2C_ADDR[i]);
        if (d > 0) _lidar_dist[i] = d;
    }
}

// Returns distances as cm×10 (same units as RP2350 version).
int* Car::read_sensors() {
    static int values[4];
    for (int i = 0; i < 4; i++) {
        values[i] = _lidar_ok[i] ? (int)_lidar_dist[i] * 10 : 9999;
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
                      (3.14159265f * cfg_wheel_diam_m) / dt;

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
    s = -s;   // invert so positive = right
    s = constrain(s, -1000, 1000);
    if (s < 0) s = map(s, -1000, 0,    cfg_min_point,     cfg_neutral_point);
    else       s = map(s,     0, 1000, cfg_neutral_point,  cfg_max_point);
    steer_servo.write(s);
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

#endif // PLATFORM_ESP32S3
