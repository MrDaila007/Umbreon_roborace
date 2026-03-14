/**
 * hardware_test_esp32s3 — Full hardware check for Umbreon (ESP32-S3)
 * ESP32-S3 single-chip: WiFi + I2C LiDAR + IMU + Servo/ESC + Tachometer
 *
 * Output goes to BOTH USB Serial AND WiFi (HTTP:80 + WebSocket:81 + TCP:23).
 * Input accepted from USB, TCP, or WebSocket.
 *
 * Open Serial Monitor at 115200, or connect to Umbreon WiFi AP
 * and open http://192.168.4.1 (web dashboard) or TCP 192.168.4.1:23.
 *
 * Commands:
 *   l  — LiDAR stream (continuous, all 4 sensors via I2C)
 *   s  — Servo sweep
 *   t  — Tachometer live (spin wheel by hand)
 *   e  — ESC arm + minimal forward pulse (2 s, then stop)
 *   p  — Speed hold test (PID by tachometer)
 *   u  — PID auto-tune (relay method, ~30 s)
 *   r  — Reactive steering (LiDAR → servo)
 *   i  — IMU test (gyro Z bias + live yaw/heading)
 *   c  — I2C bus scan (find LiDAR addresses + IMU)
 *   a  — Run all tests in sequence
 *   ?  — Print this help
 *
 * TF-Luna I2C setup:
 *   Each sensor must be pre-configured for I2C mode with a unique address.
 *   Default addresses: Left=0x10, Front-Left=0x11, Front-Right=0x12, Right=0x13
 *   Run the I2C scan test (c) to verify sensor addresses.
 */

#pragma GCC optimize("Ofast")

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>
#include <Wire.h>

// Web UI (shared with ESP8266 version)
#include "../wifi_debug/web_ui.h"

// ─── Feature flags ──────────────────────────────────────────────────────────
#define USE_IMU  1    // 1 = enable MPU-6050 gyro test

// ─── Pin assignments (ESP32-S3-DevKitC-1) ──────────────────────────────────
#define I2C_SDA_PIN      8
#define I2C_SCL_PIN      9
#define SERVO_PIN       10
#define MOTOR_PIN       11
#define TAHO_PIN        13
#define STATUS_LED_PIN   2

// ─── TF-Luna I2C addresses ────────────────────────────────────────────────
// Each sensor must have a unique address. Use test 'c' (I2C scan) to verify.
static const uint8_t LIDAR_ADDR[4] = {0x10, 0x11, 0x12, 0x13};
static const char*   LIDAR_NAME[4] = {"Left    ", "FrontL  ", "FrontR  ", "Right   "};
static bool          lidar_ok[4]   = {false, false, false, false};
static uint16_t      lidar_cm[4]   = {0, 0, 0, 0};

// ─── IMU ───────────────────────────────────────────────────────────────────
#if USE_IMU
#define MPU6050_ADDR   0x68
#define GYRO_FS_500    0x08
#define GYRO_SENS      65.5f
#endif

// ─── WiFi configuration ───────────────────────────────────────────────────
#define WIFI_MODE_SETTING  WIFI_AP
#define AP_SSID            "Umbreon"
#define AP_PASS            "12345678"
#define STA_SSID           "YourWiFi"
#define STA_PASS           "YourPassword"
#define STA_TIMEOUT_S      15

#define HTTP_PORT    80
#define WS_PORT      81
#define TCP_PORT     23

// ─── ESC / Servo limits ─────────────────────────────────────────────────────
#define SERVO_NEUTRAL   90
#define SERVO_LEFT      40
#define SERVO_RIGHT     140
#define ESC_NEUTRAL     90
#define ESC_FWD_MIN     96
#define ESC_FWD_MAX     120

// ─── Tachometer / wheel geometry ────────────────────────────────────────────
#define PULSES_PER_REV  62
#define WHEEL_DIAM_M    0.063f
#define WHEEL_CIRC_M    (3.14159f * WHEEL_DIAM_M)

// ─── WiFi server instances ────────────────────────────────────────────────
static WebServer         httpServer(HTTP_PORT);
static WebSocketsServer  wsServer = WebSocketsServer(WS_PORT);
static WiFiServer        tcpServer(TCP_PORT);

#define MAX_TCP_CLIENTS 4
static WiFiClient tcpClients[MAX_TCP_CLIENTS];

// ─── WiFi command ring buffer ─────────────────────────────────────────────
static char wifi_ring[512];
static volatile int wifi_ring_head = 0;
static int wifi_ring_tail = 0;

static void wifi_ring_push(const char* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        int next = (wifi_ring_head + 1) % (int)sizeof(wifi_ring);
        if (next != wifi_ring_tail) {
            wifi_ring[wifi_ring_head] = data[i];
            wifi_ring_head = next;
        }
    }
}

static int wifi_ring_available() {
    return (wifi_ring_head - wifi_ring_tail + (int)sizeof(wifi_ring)) % (int)sizeof(wifi_ring);
}

static int wifi_ring_read() {
    if (wifi_ring_head == wifi_ring_tail) return -1;
    char c = wifi_ring[wifi_ring_tail];
    wifi_ring_tail = (wifi_ring_tail + 1) % (int)sizeof(wifi_ring);
    return c;
}

static int wifi_ring_peek() {
    if (wifi_ring_head == wifi_ring_tail) return -1;
    return wifi_ring[wifi_ring_tail];
}

// ─── WiFi broadcast (send line to all TCP + WS clients) ──────────────────
static void wifi_broadcast(const char* line, int len) {
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (tcpClients[i] && tcpClients[i].connected()) {
            tcpClients[i].write((const uint8_t*)line, len);
        }
    }
    wsServer.broadcastTXT(line, len);
}

// ─── DualPrint (USB Serial + WiFi broadcast) ──────────────────────────────
// Line-buffered: accumulates output, broadcasts complete lines to WiFi.
class DualPrint : public Print {
private:
    char buf[300];
    int pos = 0;
public:
    size_t write(uint8_t c) override {
        Serial.write(c);
        if (pos < (int)sizeof(buf) - 1) buf[pos++] = c;
        if (c == '\n') {
            buf[pos] = '\0';
            wifi_broadcast(buf, pos);
            pos = 0;
        }
        return 1;
    }
    size_t write(const uint8_t* data, size_t size) override {
        for (size_t i = 0; i < size; i++) write(data[i]);
        return size;
    }
};

DualPrint out;

// ─── Input from USB or WiFi ──────────────────────────────────────────────
bool input_available() {
    if (Serial.available()) return true;
    if (wifi_ring_available()) return true;
    return false;
}

char read_input() {
    if (Serial.available()) return (char)Serial.read();
    if (wifi_ring_available()) return (char)wifi_ring_read();
    return 0;
}

void flush_input() {
    while (Serial.available()) Serial.read();
    while (wifi_ring_available()) wifi_ring_read();
}

char wait_key() {
    while (!input_available()) { poll_all_lidars(); wifi_loop_tick(); delay(1); }
    char c = read_input();
    flush_input();
    return c;
}

// ─── WiFi event handlers ──────────────────────────────────────────────────
static void ws_event(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
    if (type == WStype_TEXT) {
        wifi_ring_push((const char*)payload, len);
        wifi_ring_push("\n", 1);
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

    httpServer.on("/", []() {
        httpServer.send(200, "text/html", PAGE_HTML);
    });
    httpServer.begin();

    wsServer.begin();
    wsServer.onEvent(ws_event);

    tcpServer.begin();
    tcpServer.setNoDelay(true);

    Serial.println("Servers started (HTTP:80, WS:81, TCP:23)");
}

// ─── WiFi loop tick ───────────────────────────────────────────────────────
void wifi_loop_tick() {
    httpServer.handleClient();
    wsServer.loop();

    // Accept new TCP clients
    if (tcpServer.hasClient()) {
        int slot = -1;
        for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
            if (!tcpClients[i] || !tcpClients[i].connected()) { slot = i; break; }
        }
        if (slot >= 0) {
            tcpClients[slot] = tcpServer.accept();
            tcpClients[slot].setNoDelay(true);
            IPAddress ip = (WiFi.getMode() & WIFI_AP)
                         ? WiFi.softAPIP() : WiFi.localIP();
            tcpClients[slot].printf("# Umbreon ESP32-S3 HW Test  IP %s\r\n",
                                    ip.toString().c_str());
        } else {
            tcpServer.accept().stop();
        }
    }

    // Read TCP client data into ring buffer
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (tcpClients[i] && tcpClients[i].connected()) {
            while (tcpClients[i].available()) {
                char c = tcpClients[i].read();
                wifi_ring_push(&c, 1);
            }
        }
    }

    // LED status
    static unsigned long led_ms = 0;
    static bool led_st = false;
    int clients = wsServer.connectedClients();
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (tcpClients[i] && tcpClients[i].connected()) clients++;
    }
    if (clients == 0) {
        if (millis() - led_ms > 1000) { led_ms = millis(); led_st = !led_st; digitalWrite(STATUS_LED_PIN, led_st); }
    } else {
        digitalWrite(STATUS_LED_PIN, HIGH);
    }

    // STA reconnect
    if (WIFI_MODE_SETTING == WIFI_STA && WiFi.status() != WL_CONNECTED) {
        static unsigned long retry_ms = 0;
        if (millis() - retry_ms > 5000) { retry_ms = millis(); WiFi.reconnect(); }
    }
}

// ─── WiFi telemetry output (direct to clients, bypassing DualPrint) ──────
// Used for 25 Hz telemetry — avoids USB Serial flood during normal operation.
static char telem_line[200];

static void wifi_send_line(const char* line) {
    int len = strlen(line);
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (tcpClients[i] && tcpClients[i].connected()) {
            tcpClients[i].write((const uint8_t*)line, len);
        }
    }
    wsServer.broadcastTXT(line, len);
}

// ─── TF-Luna I2C read ────────────────────────────────────────────────────
static uint16_t read_lidar_i2c(uint8_t addr) {
    Wire.beginTransmission(addr);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0) return 0;
    if (Wire.requestFrom(addr, (uint8_t)2) != 2) return 0;
    uint16_t dist = Wire.read();
    dist |= (uint16_t)Wire.read() << 8;
    return dist;   // cm
}

void poll_all_lidars() {
    for (int i = 0; i < 4; i++) {
        if (!lidar_ok[i]) continue;
        uint16_t d = read_lidar_i2c(LIDAR_ADDR[i]);
        if (d > 0) lidar_cm[i] = d;
    }
}

// ─── Tachometer ──────────────────────────────────────────────────────────
volatile unsigned long taho_count = 0;
volatile unsigned long taho_last  = 0;
volatile unsigned long taho_iv    = 0;

void IRAM_ATTR taho_isr() {
    unsigned long now = micros();
    unsigned long dt  = now - taho_last;
    if (dt < 500) return;
    taho_count++;
    taho_iv   = dt;
    taho_last = now;
}

float get_speed() {
    unsigned long elapsed = (unsigned long)(micros() - taho_last);
    elapsed = max(elapsed, (unsigned long)taho_iv);
    if (taho_iv == 0 || elapsed > 500000UL) return 0.0f;
    return WHEEL_CIRC_M / (PULSES_PER_REV * (elapsed / 1e6f));
}

// ─── Actuators ───────────────────────────────────────────────────────────
Servo steer;
Servo esc;

// ─── Print helpers ───────────────────────────────────────────────────────
void print_help() {
    out.println("\nCommands:");
    out.println("  l  LiDAR stream (I2C)");
    out.println("  s  Servo sweep");
    out.println("  t  Tachometer live");
    out.println("  e  ESC arm test  (car will move briefly!)");
    out.println("  p  Speed hold (PID, +/- to change target)");
    out.println("  u  PID auto-tune (relay method, ~30s)");
    out.println("  r  Reactive steering (LiDAR -> servo)");
#if USE_IMU
    out.println("  i  IMU test (gyro Z bias + live yaw/heading)");
#endif
    out.println("  c  I2C bus scan (find LiDAR + IMU addresses)");
    out.println("  a  All tests in sequence");
    out.println("  ?  This help\n");
}

// ─── Test: I2C Scan ─────────────────────────────────────────────────────
void test_i2c_scan() {
    out.println("\n=== I2C Bus Scan ===");
    out.print("SDA=GPIO"); out.print(I2C_SDA_PIN);
    out.print("  SCL=GPIO"); out.println(I2C_SCL_PIN);
    out.println();

    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            out.print("  0x");
            if (addr < 16) out.print("0");
            out.print(addr, HEX);
            out.print("  ");

            // Identify known devices
            if (addr >= 0x10 && addr <= 0x13) {
                int idx = addr - 0x10;
                out.print("TF-Luna LiDAR [");
                out.print(idx);
                out.print("] ");
                out.print(LIDAR_NAME[idx]);
            }
            else if (addr == 0x68 || addr == 0x69) out.print("MPU-6050 IMU");
            else if (addr == 0x53) out.print("ADXL345 Accelerometer");
            else if (addr == 0x76 || addr == 0x77) out.print("BMP280/BME280");
            else if (addr >= 0x20 && addr <= 0x27) out.print("PCF8574 I/O Expander");
            else if (addr >= 0x70 && addr <= 0x77) out.print("TCA9548A I2C Mux");
            else out.print("(unknown)");

            out.println();
            found++;
        }
    }

    if (found == 0) {
        out.println("  No devices found!");
        out.println("  Check wiring: SDA/SCL connections, pull-up resistors.");
    } else {
        out.print("\n");
        out.print(found);
        out.println(" device(s) found.");
    }

    // Check expected LiDARs
    out.println("\nExpected LiDAR addresses:");
    for (int i = 0; i < 4; i++) {
        out.print("  ");
        out.print(LIDAR_NAME[i]);
        out.print(" 0x");
        if (LIDAR_ADDR[i] < 16) out.print("0");
        out.print(LIDAR_ADDR[i], HEX);
        out.print("  ");
        out.println(lidar_ok[i] ? "OK" : "NOT FOUND");
    }
    out.println("\nI2C scan done.");
}

// ─── Test: LiDAR ─────────────────────────────────────────────────────────
void test_lidar(unsigned long duration_ms = 0) {
    out.println("\n=== LiDAR Test (I2C) ===");
    out.println("Press any key to stop.\n");
    out.println("Sensor    | Distance  | Status");
    out.println("----------|-----------|--------");

    unsigned long start = millis();
    while (!input_available()) {
        if (duration_ms && (millis() - start) > duration_ms) break;
        poll_all_lidars();
        wifi_loop_tick();

        for (int i = 0; i < 4; i++) {
            out.print(LIDAR_NAME[i]);
            out.print(" | ");
            if (lidar_ok[i] && lidar_cm[i] > 0) {
                out.print(lidar_cm[i]);
                out.print(" cm      | OK    ");
            } else if (!lidar_ok[i]) {
                out.print("--         | no I2C");
            } else {
                out.print("0          | error ");
            }
            out.println();
        }
        delay(100);
    }
    flush_input();
    out.println("\nLiDAR test done.");
}

// ─── Test: Servo ─────────────────────────────────────────────────────────
void test_servo() {
    out.println("\n=== Servo Test ===");
    out.println("Sweeping LEFT...");  steer.write(SERVO_LEFT);    delay(800);
    out.println("Sweeping RIGHT..."); steer.write(SERVO_RIGHT);   delay(800);
    out.println("Centre.");           steer.write(SERVO_NEUTRAL); delay(400);

    out.println("Slow sweep left->right->centre:");
    for (int a = SERVO_LEFT; a <= SERVO_RIGHT; a++) { steer.write(a); delay(8); }
    for (int a = SERVO_RIGHT; a >= SERVO_LEFT; a--) { steer.write(a); delay(8); }
    steer.write(SERVO_NEUTRAL);
    out.println("Servo test done.");
}

// ─── Test: Tachometer ────────────────────────────────────────────────────
void test_taho(unsigned long duration_ms = 0) {
    out.println("\n=== Tachometer Test ===");
    out.println("Spin the wheel by hand. Press any key to stop.\n");

    noInterrupts(); taho_count = 0; taho_last = micros(); taho_iv = 0; interrupts();

    unsigned long start = millis();
    while (!input_available()) {
        if (duration_ms && (millis() - start) > duration_ms) break;
        wifi_loop_tick();

        noInterrupts();
        unsigned long cnt = taho_count;
        unsigned long iv  = taho_iv;
        interrupts();

        bool stopped = (micros() - taho_last) > 500000UL;
        float speed_ms = (!stopped && iv > 0)
            ? WHEEL_CIRC_M / (PULSES_PER_REV * (iv / 1e6f)) : 0.0f;

        out.print("Pulses: "); out.print(cnt);
        out.print("   interval: "); out.print(iv); out.print(" us");
        out.print("   speed: "); out.print(speed_ms, 2); out.print(" m/s");
        out.print(stopped ? "   [STOPPED]" : "   [spinning]");
        out.println();
        delay(150);
    }
    flush_input();
    out.println("Tachometer test done.");
}

// ─── Test: ESC ───────────────────────────────────────────────────────────
void test_esc() {
    out.println("\n=== ESC Test ===");
    out.println("!! WHEEL WILL SPIN FOR ~2 SECONDS !!");
    out.println("Confirm by sending 'y', anything else cancels.");

    char c = wait_key();
    if (c != 'y' && c != 'Y') { out.println("Cancelled."); return; }

    out.println("Arming ESC (neutral 2 s)...");
    esc.write(ESC_NEUTRAL); delay(2000);

    noInterrupts();
    taho_count = 0; taho_last = micros(); taho_iv = 0;
    interrupts();

    out.println("Minimal forward pulse (2 s)...");
    esc.write(ESC_FWD_MIN);

    unsigned long esc_start = millis();
    while (millis() - esc_start < 2000) {
        poll_all_lidars();
        wifi_loop_tick();

        noInterrupts();
        unsigned long cnt  = taho_count;
        unsigned long iv   = taho_iv;
        unsigned long last = taho_last;
        interrupts();

        bool stopped = (micros() - last) > 500000UL;
        float speed_ms = (!stopped && iv > 0)
            ? WHEEL_CIRC_M / (PULSES_PER_REV * (iv / 1e6f)) : 0.0f;
        float revs = (float)cnt / PULSES_PER_REV;

        out.print("Pulses: "); out.print(cnt);
        out.print("  revs: "); out.print(revs, 1);
        out.print("  speed: "); out.print(speed_ms, 2); out.print(" m/s");
        out.print(stopped ? "  [STOPPED]  " : "  [spinning] ");
        out.println();
        delay(100);
    }

    out.println("Stopping.");
    esc.write(ESC_NEUTRAL); delay(500);

    noInterrupts();
    unsigned long final_cnt = taho_count;
    interrupts();

    out.print("Total pulses: "); out.print(final_cnt);
    out.print("  ("); out.print((float)final_cnt / PULSES_PER_REV, 1);
    out.println(" revolutions)");
    out.println("ESC test done.");
}

// ─── Test: Speed Hold (PID) ──────────────────────────────────────────────
void test_speed_hold() {
    out.println("\n=== Speed Hold Test ===");
    out.println("!! WHEEL WILL SPIN !!");
    out.println("Confirm by sending 'y', anything else cancels.");

    char c = wait_key();
    if (c != 'y' && c != 'Y') { out.println("Cancelled."); return; }

    float target_speed = 1.5f;
    float kP = 4.18f, kI = 2.93f, kD = 0.43f;
    float integral = 0, prev_error = 0, filtered_speed = 0;
    int   esc_val = ESC_NEUTRAL;
    unsigned long prev_cnt = 0;

    out.println("Arming ESC (neutral 2 s)...");
    esc.write(ESC_NEUTRAL); delay(2000);

    out.println("Starting PID loop.  +/- change target,  any other key = stop.\n");
    out.println("CSV:ms,target,raw,filtered,esc,error,integral,deriv,pulses");

    noInterrupts();
    taho_count = 0; taho_last = micros(); taho_iv = 0;
    interrupts();

    unsigned long prev_ms = millis();

    while (true) {
        poll_all_lidars();
        wifi_loop_tick();

        if (input_available()) {
            char ch = read_input();
            if (ch == '+' || ch == '=') {
                target_speed += 0.1f;
                out.print("\n>> Target -> "); out.print(target_speed, 1); out.println(" m/s");
            } else if (ch == '-' || ch == '_') {
                target_speed -= 0.1f;
                if (target_speed < 0) target_speed = 0;
                out.print("\n>> Target -> "); out.print(target_speed, 1); out.println(" m/s");
            } else {
                break;
            }
            flush_input();
        }

        unsigned long now_ms = millis();
        if (now_ms - prev_ms < 80) continue;
        float dt = (now_ms - prev_ms) / 1000.0f;
        prev_ms = now_ms;

        noInterrupts();
        unsigned long cnt  = taho_count;
        unsigned long last = taho_last;
        interrupts();

        unsigned long delta_cnt = cnt - prev_cnt;
        prev_cnt = cnt;

        float raw_speed = (delta_cnt / (float)PULSES_PER_REV) * WHEEL_CIRC_M / dt;
        filtered_speed = 0.5f * raw_speed + 0.5f * filtered_speed;

        bool stopped = (micros() - last) > 500000UL;
        if (stopped) filtered_speed = 0;

        float error = target_speed - filtered_speed;
        integral += error * dt;
        integral = constrain(integral, -5.0f, 5.0f);
        float derivative = (error - prev_error) / dt;
        prev_error = error;

        float ff = (target_speed > 0.01f) ? (float)(ESC_FWD_MIN - ESC_NEUTRAL) : 0;
        float output = ff + kP * error + kI * integral + kD * derivative;
        esc_val = ESC_NEUTRAL + (int)output;
        esc_val = constrain(esc_val, ESC_NEUTRAL, ESC_FWD_MAX);
        esc.write(esc_val);

        out.print("D,");
        out.print(now_ms);           out.print(",");
        out.print(target_speed, 2);  out.print(",");
        out.print(raw_speed, 2);     out.print(",");
        out.print(filtered_speed, 2); out.print(",");
        out.print(esc_val);          out.print(",");
        out.print(error, 3);         out.print(",");
        out.print(integral, 3);      out.print(",");
        out.print(derivative, 3);    out.print(",");
        out.println(cnt);
    }

    esc.write(ESC_NEUTRAL);
    flush_input();

    noInterrupts();
    unsigned long final_cnt = taho_count;
    interrupts();

    out.print("\nStopped. Total pulses: "); out.print(final_cnt);
    out.print("  ("); out.print((float)final_cnt / PULSES_PER_REV, 1);
    out.println(" revolutions)");
    out.println("Speed hold test done.");
}

// ─── Test: PID Auto-Tune (Relay) ─────────────────────────────────────────
void test_autotune() {
    out.println("\n=== PID Auto-Tuner (Relay Method) ===");
    out.println("!! WHEEL WILL SPIN for ~30s !!");
    out.println("Confirm 'y', anything else cancels.");

    char c = wait_key();
    if (c != 'y' && c != 'Y') { out.println("Cancelled."); return; }

    const float TARGET     = 1.5f;
    const int   RELAY_D    = 2;
    const float HYST       = 0.10f;
    const int   BASE_ESC   = 98;
    const int   SKIP_HALF  = 4;
    const int   NEED_HALF  = 12;
    const unsigned long TIMEOUT = 40000;

    out.println("Arming ESC (2s)...");
    esc.write(ESC_NEUTRAL); delay(2000);

    noInterrupts();
    taho_count = 0; taho_last = micros(); taho_iv = 0;
    interrupts();

    out.println("Phase 1: Relay oscillation...");
    out.println("CSV:ms,target,raw,filtered,esc,relay");

    bool relay_high = true;
    float filtered = 0;
    float speed_peak = 0, speed_trough = 999.0f;
    unsigned long prev_cnt = 0, prev_ms = millis(), start_ms = prev_ms;
    int half_cycle = 0;

    const int MAXM = 16;
    float meas_peaks[MAXM], meas_troughs[MAXM];
    unsigned long sw_times[MAXM * 2];
    int np = 0, nt = 0, nsw = 0;

    int esc_val = constrain(BASE_ESC + RELAY_D, ESC_NEUTRAL, ESC_FWD_MAX);
    esc.write(esc_val);

    while (millis() - start_ms < TIMEOUT) {
        poll_all_lidars();
        wifi_loop_tick();
        if (input_available()) { flush_input(); break; }

        unsigned long now = millis();
        if (now - prev_ms < 80) continue;
        float dt = (now - prev_ms) / 1000.0f;
        prev_ms = now;

        noInterrupts();
        unsigned long cnt  = taho_count;
        unsigned long last = taho_last;
        interrupts();

        unsigned long dc = cnt - prev_cnt;
        prev_cnt = cnt;
        float raw = (dc / (float)PULSES_PER_REV) * WHEEL_CIRC_M / dt;
        filtered = 0.5f * raw + 0.5f * filtered;
        if ((micros() - last) > 500000UL) filtered = 0;

        if (filtered > speed_peak)   speed_peak   = filtered;
        if (filtered < speed_trough) speed_trough = filtered;

        float spd_err = filtered - TARGET;

        if (relay_high && spd_err > HYST) {
            relay_high = false;
            if (half_cycle >= SKIP_HALF && nt < MAXM)  meas_troughs[nt++] = speed_trough;
            if (half_cycle >= SKIP_HALF && nsw < MAXM*2) sw_times[nsw++] = now;
            half_cycle++;
            speed_peak = filtered; speed_trough = 999.0f;
        }
        else if (!relay_high && spd_err < -HYST) {
            relay_high = true;
            if (half_cycle >= SKIP_HALF && np < MAXM)  meas_peaks[np++] = speed_peak;
            if (half_cycle >= SKIP_HALF && nsw < MAXM*2) sw_times[nsw++] = now;
            half_cycle++;
            speed_trough = filtered; speed_peak = 0;
        }

        esc_val = relay_high ? (BASE_ESC + RELAY_D) : (BASE_ESC - RELAY_D);
        esc_val = constrain(esc_val, ESC_NEUTRAL, ESC_FWD_MAX);
        esc.write(esc_val);

        out.print("D,"); out.print(now);
        out.print(",");  out.print(TARGET, 2);
        out.print(",");  out.print(raw, 2);
        out.print(",");  out.print(filtered, 2);
        out.print(",");  out.print(esc_val);
        out.print(",");  out.println(relay_high ? 1 : 0);

        if (half_cycle >= SKIP_HALF + NEED_HALF) break;
    }

    esc.write(ESC_NEUTRAL);
    delay(500);

    if (np < 2 || nt < 2 || nsw < 4) {
        out.println("\nERR: Not enough oscillation data!");
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

    float zn_kP = 0.6f * Ku, zn_kI = zn_kP / (0.5f * Tu), zn_kD = zn_kP * Tu / 8.0f;
    float tl_kP = Ku / 2.2f, tl_kI = tl_kP / (2.2f * Tu), tl_kD = tl_kP * Tu / 6.3f;
    float pi_kP = 0.45f * Ku, pi_kI = pi_kP / (0.83f * Tu);

    out.println("\n======== AUTO-TUNE RESULTS ========");
    out.print("Amplitude: "); out.print(amplitude, 3); out.println(" m/s");
    out.print("Period Tu: "); out.print(Tu, 3); out.println(" s");
    out.print("Ult. gain Ku: "); out.println(Ku, 2);

    out.println("\nR,method,kP,kI,kD");
    out.print("R,ZN,");  out.print(zn_kP, 2); out.print(","); out.print(zn_kI, 2); out.print(","); out.println(zn_kD, 3);
    out.print("R,TL,");  out.print(tl_kP, 2); out.print(","); out.print(tl_kI, 2); out.print(","); out.println(tl_kD, 3);
    out.print("R,PI,");  out.print(pi_kP, 2); out.print(","); out.print(pi_kI, 2); out.println(",0.000");
    out.println("===================================");
    out.println("Auto-tune complete.");
}

// ─── Test: Reactive Steering (LiDAR → Servo) ────────────────────────────
void test_reactive(unsigned long duration_ms = 0) {
    out.println("\n=== Reactive Steering Test ===");
    out.println("LiDAR controls servo in real-time. Press any key to stop.\n");

    const int CLOSE_DIST = 120;   // cm
    const int FAR_DIST   = 300;

    unsigned long start = millis();
    while (!input_available()) {
        if (duration_ms && (millis() - start) > duration_ms) break;
        poll_all_lidars();
        wifi_loop_tick();

        int L  = (lidar_ok[0] && lidar_cm[0] > 0) ? lidar_cm[0] : 999;
        int FL = (lidar_ok[1] && lidar_cm[1] > 0) ? lidar_cm[1] : 999;
        int FR = (lidar_ok[2] && lidar_cm[2] > 0) ? lidar_cm[2] : 999;
        int R  = (lidar_ok[3] && lidar_cm[3] > 0) ? lidar_cm[3] : 999;

        float diff = (float)(R - L);
        if (FL < CLOSE_DIST) diff += (float)(CLOSE_DIST - FL);
        if (FR < CLOSE_DIST) diff -= (float)(CLOSE_DIST - FR);

        float steer_f = constrain(diff / (float)FAR_DIST, -1.0f, 1.0f);

        int servo_val;
        if (steer_f < 0)
            servo_val = map((int)(steer_f * 1000), -1000, 0, SERVO_LEFT, SERVO_NEUTRAL);
        else
            servo_val = map((int)(steer_f * 1000), 0, 1000, SERVO_NEUTRAL, SERVO_RIGHT);
        steer.write(servo_val);

        out.print("  L="); out.print(L);
        out.print("  FL="); out.print(FL);
        out.print("  FR="); out.print(FR);
        out.print("  R="); out.print(R);
        out.print("  -> servo="); out.print(servo_val);
        if (steer_f < -0.2f)       out.print("  <-LEFT   ");
        else if (steer_f > 0.2f)   out.print("  RIGHT->  ");
        else                        out.print("  CENTER   ");
        out.println();
        delay(50);
    }

    steer.write(SERVO_NEUTRAL);
    flush_input();
    out.println("\nReactive steering test done.");
}

// ─── Test: IMU (MPU-6050 Gyro Z) ────────────────────────────────────────
#if USE_IMU
void test_imu(unsigned long duration_ms = 0) {
    out.println("\n=== IMU Test (MPU-6050 Gyro Z) ===");

    // I2C already initialized — check WHO_AM_I
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x75);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    if (Wire.available() < 1) {
        out.println("ERROR: MPU-6050 not found on I2C bus!");
        out.print("  SDA=GPIO"); out.print(I2C_SDA_PIN);
        out.print("  SCL=GPIO"); out.println(I2C_SCL_PIN);
        return;
    }
    uint8_t who = Wire.read();
    out.print("WHO_AM_I: 0x"); out.print(who, HEX);
    if      (who == 0x68) out.println("  (MPU-6050)");
    else if (who == 0x70) out.println("  (MPU-6500)");
    else if (who == 0x71) out.println("  (MPU-9250)");
    else { out.print("  (0x"); out.print(who, HEX); out.println(")"); }

    // Wake + configure
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x6B); Wire.write(0x00);
    if (Wire.endTransmission() != 0) {
        out.println("ERROR: Failed to wake MPU-6050!");
        return;
    }
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1B); Wire.write(GYRO_FS_500); Wire.endTransmission();
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();
    delay(100);

    // Bias calibration (2 s)
    out.println("Calibrating gyro bias (2 s, keep car still)...");
    float bias_sum = 0;
    int bias_n = 0;
    unsigned long cal_start = millis();
    while (millis() - cal_start < 2000) {
        wifi_loop_tick();
        Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x47); Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
        if (Wire.available() >= 2) {
            int16_t raw_z = ((int16_t)Wire.read() << 8) | Wire.read();
            bias_sum += raw_z / GYRO_SENS;
            bias_n++;
        }
        delay(10);
    }

    float bias = (bias_n > 0) ? bias_sum / bias_n : 0.0f;
    out.print("Gyro Z bias: "); out.print(bias, 3); out.print(" deg/s");
    out.print("  ("); out.print(bias_n); out.println(" samples)");

    // Live stream
    out.println("\nStreaming gyro Z. Press any key to stop.\n");

    float heading = 0.0f;
    unsigned long prev_us = micros();

    unsigned long start = millis();
    while (!input_available()) {
        if (duration_ms && (millis() - start) > duration_ms) break;
        wifi_loop_tick();

        Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x47); Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
        if (Wire.available() < 2) { delay(10); continue; }

        int16_t raw_z = ((int16_t)Wire.read() << 8) | Wire.read();
        float yaw_rate = raw_z / GYRO_SENS - bias;

        unsigned long now_us = micros();
        float dt = (now_us - prev_us) / 1e6f;
        prev_us = now_us;
        if (dt > 0.0f && dt < 0.5f) heading += yaw_rate * dt;

        out.print("  yaw="); out.print(yaw_rate, 2);
        out.print(" d/s  heading="); out.print(heading, 1);
        out.print(" deg  "); out.println((yaw_rate > 1.0f || yaw_rate < -1.0f) ? "turning" : "still  ");
        delay(50);
    }

    flush_input();
    out.print("\nFinal heading: "); out.print(heading, 1); out.println(" deg");
    out.println("IMU test done.");
}
#endif

// ─── All tests ───────────────────────────────────────────────────────────
void run_all() {
    test_i2c_scan();
    test_lidar(4000);
    test_servo();
    test_taho(5000);
#if USE_IMU
    test_imu(5000);
#endif
    out.println("\nAll automatic tests done. Run 'e' for ESC test.");
}

// ─── WiFi protocol command handling ──────────────────────────────────────
// Manual drive state
static bool drv_enabled = false;
static int drv_steer = 0;
static float drv_speed = 0.0f;
static unsigned long drv_last_ms = 0;

// Telemetry timing
static unsigned long telem_next_ms = 0;
#define TELEM_INTERVAL_MS  40   // 25 Hz

// IMU state for telemetry
#if USE_IMU
static bool imu_telem_ok = false;
static float imu_bias = 0.0f;
static float imu_yaw_rate = 0.0f;
static float imu_heading = 0.0f;
static unsigned long imu_prev_us = 0;

static void imu_quick_init() {
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x75); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    if (Wire.available() < 1) { imu_telem_ok = false; return; }
    Wire.read();  // discard WHO_AM_I

    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1B); Wire.write(GYRO_FS_500); Wire.endTransmission();
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();
    delay(100);

    // Quick bias (1 s)
    float bias_sum = 0;
    int n = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 1000) {
        Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x47); Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
        if (Wire.available() >= 2) {
            int16_t raw = ((int16_t)Wire.read() << 8) | Wire.read();
            bias_sum += raw / GYRO_SENS;
            n++;
        }
        delay(5);
    }
    imu_bias = (n > 0) ? bias_sum / n : 0.0f;
    imu_prev_us = micros();
    imu_telem_ok = true;
}

static void imu_telem_update() {
    if (!imu_telem_ok) return;
    Wire.beginTransmission(MPU6050_ADDR); Wire.write(0x47); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return;
    int16_t raw = ((int16_t)Wire.read() << 8) | Wire.read();
    imu_yaw_rate = raw / GYRO_SENS - imu_bias;
    unsigned long now_us = micros();
    float dt = (now_us - imu_prev_us) / 1e6f;
    imu_prev_us = now_us;
    if (dt > 0.0f && dt < 0.5f) imu_heading += imu_yaw_rate * dt;
}
#endif

// WiFi command line buffer
static char proto_cmd[64];
static int proto_cmd_len = 0;

static void wifi_send_telemetry() {
    int s0 = (lidar_ok[0] && lidar_cm[0] > 0) ? lidar_cm[0] * 10 : 0;
    int s1 = (lidar_ok[1] && lidar_cm[1] > 0) ? lidar_cm[1] * 10 : 0;
    int s2 = (lidar_ok[2] && lidar_cm[2] > 0) ? lidar_cm[2] * 10 : 0;
    int s3 = (lidar_ok[3] && lidar_cm[3] > 0) ? lidar_cm[3] * 10 : 0;

    int len = snprintf(telem_line, sizeof(telem_line),
        "%lu,%d,%d,%d,%d,%d,%.2f,%.1f",
        millis(), s0, s1, s2, s3, drv_steer, get_speed(), drv_speed);
#if USE_IMU
    if (imu_telem_ok) {
        len += snprintf(telem_line + len, sizeof(telem_line) - len,
            ",%.1f,%.1f", imu_yaw_rate, imu_heading);
    }
#endif
    telem_line[len++] = '\n';
    telem_line[len] = '\0';
    wifi_send_line(telem_line);
}

static void wifi_handle_drv(const char* args) {
    char buf[32];
    strncpy(buf, args, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char* comma = strchr(buf, ',');
    if (!comma) return;
    *comma = '\0';
    drv_steer = atoi(buf);
    drv_speed = atof(comma + 1);
    drv_last_ms = millis();

    if (!drv_enabled) return;

    int angle = SERVO_NEUTRAL + (int)((long)drv_steer * (SERVO_RIGHT - SERVO_LEFT) / 2000);
    angle = constrain(angle, SERVO_LEFT, SERVO_RIGHT);
    steer.write(angle);

    if (drv_speed < 0.05f) {
        esc.write(ESC_NEUTRAL);
    } else {
        int esc_val = ESC_FWD_MIN + (int)((drv_speed / 3.0f) * (ESC_FWD_MAX - ESC_FWD_MIN));
        esc_val = constrain(esc_val, ESC_FWD_MIN, ESC_FWD_MAX);
        esc.write(esc_val);
    }
}

static void proto_respond(const char* msg) {
    // Send response to WiFi clients only
    char line[80];
    int len = snprintf(line, sizeof(line), "%s\n", msg);
    wifi_send_line(line);
}

static void wifi_process_line(const char* line) {
    if      (strcmp(line, "$PING")   == 0) proto_respond("$PONG");
    else if (strcmp(line, "$STATUS") == 0) proto_respond("$STS:STOP");
    else if (strcmp(line, "$STOP")  == 0) {
        drv_enabled = false; drv_steer = 0; drv_speed = 0.0f;
        steer.write(SERVO_NEUTRAL); esc.write(ESC_NEUTRAL);
        proto_respond("$ACK"); proto_respond("$STS:STOP");
    }
    else if (strcmp(line, "$START") == 0) { proto_respond("$ACK"); proto_respond("$STS:RUN"); }
    else if (strcmp(line, "$DRVEN")  == 0) { drv_enabled = true;  proto_respond("$ACK"); }
    else if (strcmp(line, "$DRVOFF") == 0) {
        drv_enabled = false; drv_steer = 0; drv_speed = 0.0f;
        steer.write(SERVO_NEUTRAL); esc.write(ESC_NEUTRAL);
        proto_respond("$ACK");
    }
    else if (strncmp(line, "$DRV:", 5) == 0) wifi_handle_drv(line + 5);
    else if (strncmp(line, "$TEST:", 6) == 0) {
        const char* name = line + 6;
        if      (strcmp(name, "lidar")    == 0) test_lidar(5000);
        else if (strcmp(name, "servo")    == 0) test_servo();
        else if (strcmp(name, "taho")     == 0) test_taho(5000);
        else if (strcmp(name, "esc")      == 0) test_esc();
        else if (strcmp(name, "speed")    == 0) test_speed_hold();
        else if (strcmp(name, "autotune") == 0) test_autotune();
        else if (strcmp(name, "reactive") == 0) test_reactive(10000);
        else { char resp[40]; snprintf(resp, sizeof(resp), "$NAK:unknown_test:%s", name); proto_respond(resp); }
    }
}

static void wifi_process_commands() {
    while (wifi_ring_available()) {
        char c = (char)wifi_ring_read();
        if (c == '\n' || c == '\r') {
            if (proto_cmd_len > 0) {
                proto_cmd[proto_cmd_len] = '\0';
                if (proto_cmd[0] == '$') wifi_process_line(proto_cmd);
                proto_cmd_len = 0;
            }
        } else if (proto_cmd_len < (int)sizeof(proto_cmd) - 1) {
            proto_cmd[proto_cmd_len++] = c;
        }
    }
}

// ─── Setup / Loop ────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);   // USB CDC enumerate

    // I2C bus (shared: LiDARs + IMU)
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    // WiFi + servers
    wifi_setup();

    // Servo + ESC
    steer.attach(SERVO_PIN);
    esc.attach(MOTOR_PIN);
    steer.write(SERVO_NEUTRAL);
    esc.write(ESC_NEUTRAL);

    // Tachometer
    pinMode(TAHO_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(TAHO_PIN), taho_isr, RISING);

    // Probe LiDAR sensors
    for (int i = 0; i < 4; i++) {
        Wire.beginTransmission(LIDAR_ADDR[i]);
        lidar_ok[i] = (Wire.endTransmission() == 0);
    }

    // IMU for telemetry
#if USE_IMU
    imu_quick_init();
#endif

    out.println("\n================================");
    out.println("  Umbreon -- ESP32-S3 HW Test");
    out.println("  WiFi:  built-in");
    out.println("  LiDAR: I2C");

    // Report LiDAR status
    int lidar_found = 0;
    for (int i = 0; i < 4; i++) if (lidar_ok[i]) lidar_found++;
    out.print("  LiDAR: "); out.print(lidar_found); out.println("/4 found");
#if USE_IMU
    out.print("  IMU:   ");
    out.println(imu_telem_ok ? "MPU-6050 OK" : "not found");
#endif
    out.println("================================");
    print_help();
}

void loop() {
    poll_all_lidars();
    wifi_loop_tick();

    // Protocol commands from WiFi ($PING, $DRV, $TEST, etc.)
    wifi_process_commands();

    // Manual drive timeout
    if (drv_enabled && drv_speed > 0.01f && millis() - drv_last_ms > 500) {
        drv_speed = 0.0f;
        drv_steer = 0;
        esc.write(ESC_NEUTRAL);
        steer.write(SERVO_NEUTRAL);
    }

#if USE_IMU
    imu_telem_update();
#endif

    // Telemetry to web dashboard at 25 Hz
    if (millis() >= telem_next_ms) {
        telem_next_ms = millis() + TELEM_INTERVAL_MS;
        wifi_send_telemetry();
    }

    // Interactive serial commands (USB)
    if (Serial.available()) {
        char cmd = (char)Serial.read();
        while (Serial.available()) Serial.read();

        switch (cmd) {
            case 'l': test_lidar();       break;
            case 's': test_servo();       break;
            case 't': test_taho();        break;
            case 'e': test_esc();         break;
            case 'p': test_speed_hold();  break;
            case 'u': test_autotune();    break;
            case 'r': test_reactive();    break;
#if USE_IMU
            case 'i': test_imu();         break;
#endif
            case 'c': test_i2c_scan();    break;
            case 'a': run_all();          break;
            case '?': print_help();       break;
            default:
                out.print("Unknown command '");
                out.print(cmd);
                out.println("'. Send ? for help.");
        }
    }
}
