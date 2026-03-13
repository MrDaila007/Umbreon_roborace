/**
 * Wemos D1 Mini WiFi Bridge for Umbreon
 *
 * Protocol-aware bidirectional UART ↔ TCP bridge with built-in web dashboard.
 * Line-buffered relay ensures complete protocol messages are forwarded intact.
 *
 * Servers:
 *   Port 80  — HTTP   (serves web dashboard)
 *   Port 81  — WebSocket (real-time bidirectional relay for web UI)
 *   Port 23  — Raw TCP (backward compat with Python dashboard / ROS2 bridge)
 *
 * LED status (built-in LED, GPIO2, active LOW):
 *   Slow blink (1 Hz)  — AP ready, no clients
 *   Fast blink (3 Hz)  — client connected, no car data
 *   Solid ON           — client connected, car data flowing
 *
 * ─── Dependencies ────────────────────────────────────────────────────────────
 *   "WebSockets" by Markus Sattler — install via Arduino Library Manager
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
 *   D1 Mini TX  → Pico GP16  (UART1 RX)
 *   D1 Mini RX  → Pico GP17  (UART1 TX)
 *   D1 Mini 3V3 → 3.3 V  (or power via USB)
 *   D1 Mini GND → GND
 *
 * ─── Usage ──────────────────────────────────────────────────────────────────
 *   1. Power up — module creates AP "Umbreon" (password "12345678")
 *   2. Connect laptop/phone to the AP
 *   3. Open http://192.168.4.1 in a browser for the web dashboard
 *      — or connect via raw TCP to 192.168.4.1:23 for the Python dashboard
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include "web_ui.h"

// ─── Configuration ──────────────────────────────────────────────────────────
#define AP_SSID   "Umbreon"
#define AP_PASS   "12345678"   // min 8 chars for WPA2
#define TCP_PORT  23
#define HTTP_PORT 80
#define WS_PORT   81
#define BAUD      115200
#define LED       LED_BUILTIN  // GPIO2, active LOW

// ─── Servers ────────────────────────────────────────────────────────────────
WiFiServer      tcpServer(TCP_PORT);
WiFiClient      tcpClient;
ESP8266WebServer httpServer(HTTP_PORT);
WebSocketsServer wsServer(WS_PORT);

// ─── Line buffers ───────────────────────────────────────────────────────────
static char uart_line[512];   // UART → TCP + WS (telemetry + responses)
static int  uart_pos = 0;
static char tcp_line[256];    // TCP → UART (commands from Python dashboard)
static int  tcp_pos  = 0;

// ─── Status tracking ───────────────────────────────────────────────────────
static unsigned long last_uart_rx = 0;

// ─── LED helpers ────────────────────────────────────────────────────────────
static void led_on()  { digitalWrite(LED, LOW);  }
static void led_off() { digitalWrite(LED, HIGH); }

static void update_led() {
    unsigned long now = millis();
    bool has_tcp = tcpClient && tcpClient.connected();
    bool has_ws  = wsServer.connectedClients() > 0;
    bool has_client  = has_tcp || has_ws;
    bool data_active = has_client && (now - last_uart_rx < 500);

    if (!has_client) {
        // Slow blink — waiting for any client
        ((now / 500) % 2) ? led_off() : led_on();
    } else if (data_active) {
        // Solid ON — data flowing
        led_on();
    } else {
        // Fast blink — client connected but no car data
        ((now / 150) % 2) ? led_off() : led_on();
    }
}

// ─── WebSocket event handler ────────────────────────────────────────────────
static void wsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED:
            wsServer.sendTXT(num, "# Umbreon WiFi Bridge \xe2\x80\x94 connected");
            Serial.println("# WS client connected");
            break;

        case WStype_DISCONNECTED:
            break;

        case WStype_TEXT:
            // Forward command to car via UART
            if (length > 0) {
                Serial.println((char*)payload);
            }
            break;

        default:
            break;
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
    Serial.print("# TCP:   "); Serial.println(TCP_PORT);
    Serial.print("# HTTP:  "); Serial.println(HTTP_PORT);
    Serial.print("# WS:    "); Serial.println(WS_PORT);

    // Raw TCP server (backward compat)
    tcpServer.begin();
    tcpServer.setNoDelay(true);

    // HTTP server (web dashboard)
    httpServer.on("/", []() {
        httpServer.send_P(200, "text/html", PAGE_HTML);
    });
    httpServer.begin();

    // WebSocket server (real-time bridge for web UI)
    wsServer.begin();
    wsServer.onEvent(wsEvent);

    Serial.println("# Status: ready");
    led_off();
}

// ─── Loop ───────────────────────────────────────────────────────────────────
void loop() {
    // Service HTTP and WebSocket
    httpServer.handleClient();
    wsServer.loop();

    // ── Accept raw TCP client (one at a time) ───────────────────────────────
    if (!tcpClient || !tcpClient.connected()) {
        WiFiClient nc = tcpServer.accept();
        if (nc) {
            if (tcpClient) tcpClient.stop();
            tcpClient = nc;
            tcpClient.setNoDelay(true);
            tcpClient.println("# Umbreon WiFi Bridge \xe2\x80\x94 connected");
            tcpClient.print("# IP: ");      tcpClient.println(WiFi.softAPIP());
            tcpClient.print("# Clients: "); tcpClient.println(WiFi.softAPgetStationNum());
            Serial.println("# TCP client connected");
        }
    }

    bool tcp_live = tcpClient && tcpClient.connected();

    // ── UART → TCP + WebSocket (line-buffered) ─────────────────────────────
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        last_uart_rx = millis();

        if (c == '\n') {
            if (uart_pos > 0) {
                uart_line[uart_pos] = '\0';
                if (tcp_live) tcpClient.println(uart_line);
                wsServer.broadcastTXT(uart_line);
                uart_pos = 0;
            }
        } else if (c != '\r' && uart_pos < (int)sizeof(uart_line) - 1) {
            uart_line[uart_pos++] = c;
        }
    }

    // ── TCP → UART (line-buffered commands from Python dashboard) ───────────
    if (tcp_live) {
        while (tcpClient.available() > 0) {
            char c = (char)tcpClient.read();

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
