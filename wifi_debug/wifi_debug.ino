/**
 * Wemos D1 Mini WiFi Bridge for Umbreon
 *
 * Protocol-aware bidirectional UART ↔ TCP bridge.
 * Line-buffered relay ensures complete protocol messages are forwarded intact.
 *
 * LED status (built-in LED, GPIO2, active LOW):
 *   Slow blink (1 Hz)  — AP ready, no TCP client
 *   Fast blink (3 Hz)  — TCP client connected, no car data
 *   Solid ON           — client connected, car data flowing
 *
 * ─── Flashing ───────────────────────────────────────────────────────────────
 *   Arduino IDE / arduino-cli:
 *     Board:  "LOLIN(WEMOS) D1 mini"  (ESP8266 core)
 *     Flash:  4 MB (FS:2MB OTA:~1019KB)
 *     Upload: 921600 baud (via built-in USB)
 *   arduino-cli FQBN: esp8266:esp8266:d1_mini
 *   Connect via USB — no manual download-mode wiring needed.
 *
 * ─── Wiring to Pico 2 ──────────────────────────────────────────────────────
 *   D1 Mini TX  → Pico GP7  (UART1 RX)
 *   D1 Mini RX  → Pico GP6  (UART1 TX)
 *   D1 Mini 3V3 → 3.3 V  (or power via USB)
 *   D1 Mini GND → GND
 *
 * ─── Usage ──────────────────────────────────────────────────────────────────
 *   1. Power up — module creates AP "Umbreon" (password "12345678")
 *   2. Connect laptop/phone to the AP
 *   3. Open TCP connection to 192.168.4.1:23
 *      e.g.  putty → Raw → 192.168.4.1 : 23
 *            or    nc 192.168.4.1 23
 *   4. Live CSV telemetry streams in
 */

#include <ESP8266WiFi.h>

// ─── Configuration ──────────────────────────────────────────────────────────
#define AP_SSID   "Umbreon"
#define AP_PASS   "12345678"   // min 8 chars for WPA2
#define TCP_PORT  23
#define BAUD      115200
#define LED       LED_BUILTIN  // GPIO2, active LOW

WiFiServer server(TCP_PORT);
WiFiClient client;

// Line buffers for protocol-aware relay
static char uart_line[512];   // UART → TCP (telemetry + responses)
static int  uart_pos = 0;
static char tcp_line[256];    // TCP → UART (commands from dashboard)
static int  tcp_pos  = 0;

// Status tracking
static unsigned long last_uart_rx = 0;

// ─── LED helpers ────────────────────────────────────────────────────────────
static void led_on()  { digitalWrite(LED, LOW);  }
static void led_off() { digitalWrite(LED, HIGH); }

static void update_led() {
    unsigned long now = millis();
    bool has_client  = client && client.connected();
    bool data_active = has_client && (now - last_uart_rx < 500);

    if (!has_client) {
        // Slow blink — waiting for TCP client
        ((now / 500) % 2) ? led_off() : led_on();
    } else if (data_active) {
        // Solid ON — data flowing
        led_on();
    } else {
        // Fast blink — client connected but no car data
        ((now / 150) % 2) ? led_off() : led_on();
    }
}

// ─── Setup ──────────────────────────────────────────────────────────────────
void setup() {
    pinMode(LED, OUTPUT);
    led_on();

    Serial.begin(BAUD);

    // Boot hello over UART (# prefix — car ignores non-$ lines)
    Serial.println();
    Serial.println("# ── Umbreon WiFi Bridge ──");
    Serial.println("# Board: Wemos D1 Mini");
    Serial.print("# SSID:  "); Serial.println(AP_SSID);

    WiFi.softAP(AP_SSID, AP_PASS);

    Serial.print("# IP:    "); Serial.println(WiFi.softAPIP());
    Serial.print("# Port:  "); Serial.println(TCP_PORT);
    Serial.println("# Status: ready");

    server.begin();
    server.setNoDelay(true);

    led_off();
}

// ─── Loop ───────────────────────────────────────────────────────────────────
void loop() {
    // Accept new TCP client (one at a time)
    if (!client || !client.connected()) {
        WiFiClient nc = server.accept();
        if (nc) {
            if (client) client.stop();
            client = nc;
            client.setNoDelay(true);
            // Hello to dashboard client
            client.println("# Umbreon WiFi Bridge — connected");
            client.print("# IP: ");      client.println(WiFi.softAPIP());
            client.print("# Clients: "); client.println(WiFi.softAPgetStationNum());
            // Notify car side
            Serial.println("# TCP client connected");
        }
    }

    bool live = client && client.connected();

    // UART → TCP  (line-buffered: forward complete lines only)
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        last_uart_rx = millis();

        if (c == '\n') {
            if (uart_pos > 0) {
                uart_line[uart_pos] = '\0';
                if (live) client.println(uart_line);
                uart_pos = 0;
            }
        } else if (c != '\r' && uart_pos < (int)sizeof(uart_line) - 1) {
            uart_line[uart_pos++] = c;
        }
    }

    // TCP → UART  (line-buffered: forward complete commands only)
    if (live) {
        while (client.available() > 0) {
            char c = (char)client.read();

            if (c == '\n') {
                if (tcp_pos > 0) {
                    tcp_line[tcp_pos] = '\0';
                    Serial.println(tcp_line);
                    tcp_pos = 0;
                }
            } else if (c != '\r' && tcp_pos < (int)sizeof(tcp_line) - 1) {
                tcp_line[tcp_pos++] = c;
            }
        }
    }

    update_led();
}
