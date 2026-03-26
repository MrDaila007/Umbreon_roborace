// OLED + Encoder standalone test for Raspberry Pi Pico 2
//
// OLED SSD1306 128x64:  I2C0 — GP0 (SDA), GP1 (SCL), addr 0x3C
// Rotary encoder:       GP12 (CLK/S1), GP22 (DT/S2), GP19 (KEY)
//
// Encoder module: 20 detents/rev, pull-ups + RC debounce on board
// VCC = 3.3V from Pico, button pressed = LOW

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EncButton.h>

#define SDA_PIN     0
#define SCL_PIN     1
#define OLED_ADDR   0x3C

#define ENC_CLK     12   // S1
#define ENC_DT      22   // S2
#define ENC_SW      19   // KEY (active LOW, pull-up on module)

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
EncButton enc(ENC_CLK, ENC_DT, ENC_SW);

bool oled_ok = false;
int  counter = 0;
int  clicks  = 0;
bool held    = false;

// Test menu
const char* menu_items[] = {"Brightness", "Contrast", "Invert", "Flip", "Reset"};
const int MENU_COUNT = 5;
int  menu_sel = 0;
bool in_menu  = false;

void i2c_scan() {
    Serial.println("\n--- I2C0 Scan ---");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("  0x");
            if (addr < 16) Serial.print("0");
            Serial.print(addr, HEX);
            if (addr == 0x3C || addr == 0x3D) Serial.print(" <- SSD1306");
            if (addr == 0x68) Serial.print(" <- MPU-6050");
            Serial.println();
            found++;
        }
    }
    if (found == 0) Serial.println("  No devices!");
    Serial.print("  Total: "); Serial.println(found);
}

void draw_main() {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);

    oled.setTextSize(2);
    oled.setCursor(16, 0);
    oled.print("Umbreon");

    oled.setTextSize(1);
    oled.setCursor(0, 24);
    oled.print("Encoder: ");
    oled.print(counter);

    oled.setCursor(0, 36);
    oled.print("Clicks:  ");
    oled.print(clicks);

    oled.setCursor(0, 48);
    oled.print(held ? "BTN: HELD" : "BTN: ---");

    // Raw pin states for debug
    oled.setCursor(0, 56);
    oled.print("S1="); oled.print(digitalRead(ENC_CLK));
    oled.print(" S2="); oled.print(digitalRead(ENC_DT));
    oled.print(" KEY="); oled.print(digitalRead(ENC_SW));

    oled.display();
}

void draw_menu() {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);

    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print("> Test Menu <");
    oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);

    for (int i = 0; i < MENU_COUNT; i++) {
        oled.setCursor(8, 14 + i * 10);
        if (i == menu_sel) {
            oled.fillRect(0, 13 + i * 10, 128, 10, SSD1306_WHITE);
            oled.setTextColor(SSD1306_BLACK);
        } else {
            oled.setTextColor(SSD1306_WHITE);
        }
        oled.print(menu_items[i]);
    }
    oled.setTextColor(SSD1306_WHITE);

    oled.setCursor(0, 56);
    oled.print("Click=select Hold=back");
    oled.display();
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(1000);

    Serial.println("=== OLED + Encoder Test ===");

    // Read encoder pins before init — verify pull-ups
    pinMode(ENC_CLK, INPUT);
    pinMode(ENC_DT,  INPUT);
    pinMode(ENC_SW,  INPUT);
    Serial.print("Pin state (idle): S1="); Serial.print(digitalRead(ENC_CLK));
    Serial.print(" S2="); Serial.print(digitalRead(ENC_DT));
    Serial.print(" KEY="); Serial.println(digitalRead(ENC_SW));
    Serial.println("Expected: S1=1 S2=1 KEY=1 (all HIGH via pull-ups)");

    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);

    i2c_scan();

    Serial.print("Init OLED 0x");
    Serial.print(OLED_ADDR, HEX);
    Serial.print("... ");
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("FAILED!");
        return;
    }
    oled_ok = true;
    Serial.println("OK");

    // Encoder: 20 detents, pull-ups to VCC on module, idle = HIGH
    enc.setEncType(EB_STEP4_LOW);

    // Splash
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(4, 8);
    oled.print("OLED+ENC");
    oled.setTextSize(1);
    oled.setCursor(8, 32);
    oled.print("Turn / Click / Hold");
    oled.setCursor(12, 48);
    oled.print("20 det, pull-up HI");
    oled.display();
    delay(2000);

    Serial.println("\nReady! Turn/click/hold encoder.");
    draw_main();
}

void loop() {
    if (!oled_ok) return;

    enc.tick();
    bool redraw = false;

    if (enc.turn()) {
        int dir = enc.dir();
        if (in_menu) {
            menu_sel = constrain(menu_sel + dir, 0, MENU_COUNT - 1);
        } else {
            counter += dir;
        }
        Serial.print("Turn "); Serial.print(dir > 0 ? "CW" : "CCW");
        Serial.print(" val="); Serial.println(in_menu ? menu_sel : counter);
        redraw = true;
    }

    if (enc.click()) {
        clicks++;
        if (!in_menu) {
            in_menu = true;
            menu_sel = 0;
            Serial.println("-> Menu");
        } else {
            Serial.print("Select: "); Serial.println(menu_items[menu_sel]);
        }
        redraw = true;
    }

    if (enc.hold()) {
        held = true;
        if (in_menu) {
            in_menu = false;
            Serial.println("<- Back");
        }
        redraw = true;
    }

    if (enc.release()) {
        held = false;
        redraw = true;
    }

    // Refresh pin states on screen every 200ms
    static unsigned long last_refresh = 0;
    if (millis() - last_refresh >= 200) {
        last_refresh = millis();
        if (!in_menu) redraw = true;
    }

    if (redraw) {
        if (in_menu) draw_menu();
        else draw_main();
    }
}
