/**
 * DT-06 WiFi Debug Bridge
 *
 * Transparent bidirectional UART ↔ TCP bridge.
 * Creates a WiFi AP; connect with any TCP client (PuTTY, telnet, netcat)
 * to stream live telemetry from Umbreon.
 *
 * ─── Flashing ───────────────────────────────────────────────────────────────
 *   Arduino IDE:
 *     Board:  "Generic ESP8285 Module"  (ESP8266 core)
 *     Flash:  1 MB (match your module)
 *     Upload: 115200 baud
 *   Put module in download mode: hold GPIO0 LOW while resetting.
 *
 * ─── Wiring to Pico 2 ──────────────────────────────────────────────────────
 *   DT-06 RX  → Pico GP6  (UART1 TX)
 *   DT-06 TX  → Pico GP7  (UART1 RX)   [optional — for sending commands]
 *   DT-06 VCC → 3.3 V
 *   DT-06 GND → GND
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

WiFiServer server(TCP_PORT);
WiFiClient client;

void setup() {
    Serial.begin(BAUD);

    WiFi.softAP(AP_SSID, AP_PASS);
    server.begin();
    server.setNoDelay(true);
}

void loop() {
    // Accept new client (one at a time)
    if (!client || !client.connected()) {
        WiFiClient nc = server.accept();
        if (nc) {
            if (client) client.stop();
            client = nc;
            client.setNoDelay(true);
            client.println("# Umbreon WiFi Debug — connected");
        }
    }

    bool live = client && client.connected();

    // UART → TCP
    int n = Serial.available();
    if (n > 0) {
        uint8_t buf[256];
        n = min(n, (int)sizeof(buf));
        for (int i = 0; i < n; i++) buf[i] = (uint8_t)Serial.read();
        if (live) client.write(buf, n);
    }

    // TCP → UART (remote commands)
    if (live) {
        n = client.available();
        if (n > 0) {
            uint8_t buf[256];
            n = min(n, (int)sizeof(buf));
            for (int i = 0; i < n; i++) buf[i] = (uint8_t)client.read();
            Serial.write(buf, n);
        }
    }
}
