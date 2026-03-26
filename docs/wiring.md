# Wiring

Pin assignments for the Raspberry Pi Pico 2 (RP2350). Select sensor configuration at compile time via `SENSOR_CONFIG` in `Umbreon_roborace.ino`.

---

## Config 1 — 4x TF-Luna (`SENSOR_4X_LUNA`)

| Pico 2 Pin | Function | Connected to |
|---|---|---|
| GP0 | I2C0 SDA | MPU-6050 SDA (optional, `USE_IMU=1`) |
| GP1 | I2C0 SCL | MPU-6050 SCL (optional, `USE_IMU=1`) |
| GP2 | SerialPIO RX | TF-Luna s[0] Left — TX |
| GP3 | SerialPIO RX | TF-Luna s[1] Front-Left — TX |
| GP4 | SerialPIO RX | TF-Luna s[2] Front-Right — TX |
| GP5 | SerialPIO RX | TF-Luna s[3] Right — TX |
| GP10 | PWM | Steering servo signal |
| GP11 | PWM | Motor ESC signal |
| GP13 | Digital input (IRQ) | Optical encoder (62 holes, RISING) |
| GP16 | UART1 TX | Wemos D1 Mini RX |
| GP17 | UART1 RX | Wemos D1 Mini TX |
| GP26 | ADC0 | Battery divider (optional, `BEN=1`) |
| 3V3 | Power | MPU-6050 VCC, TF-Luna VCC |
| 5V / VBUS | Power | TF-Luna VCC (5V required) |
| GND | Ground | Common ground |

**TF-Luna wiring (per sensor):**

| TF-Luna pin | Connect to |
|---|---|
| TX (green) | Pico GP2/GP3/GP4/GP5 (RX only, 115200 baud) |
| RX | Not connected |
| 5V (red) | 5V supply |
| GND (black) | GND |

---

## Config 2 — 6x VL53L0X (`SENSOR_6X_VL53L0X`)

| Pico 2 Pin | Function | Connected to |
|---|---|---|
| GP0 | I2C0 SDA | MPU-6050 SDA (optional, `USE_IMU=1`) |
| GP1 | I2C0 SCL | MPU-6050 SCL (optional, `USE_IMU=1`) |
| GP6 | Digital output | VL53L0X s[0] Hard-Right — XSHUT |
| GP7 | Digital output | VL53L0X s[1] Front-Right — XSHUT |
| GP8 | Digital output | VL53L0X s[2] Right — XSHUT |
| GP9 | Digital output | VL53L0X s[3] Left — XSHUT |
| GP10 | PWM | Steering servo signal |
| GP11 | PWM | Motor ESC signal |
| GP13 | Digital input (IRQ) | Optical encoder (62 holes, RISING) |
| GP14 | Digital output | VL53L0X s[4] Front-Left — XSHUT |
| GP15 | Digital output | VL53L0X s[5] Hard-Left — XSHUT |
| GP16 | UART1 TX | Wemos D1 Mini RX |
| GP17 | UART1 RX | Wemos D1 Mini TX |
| GP20 | I2C1 SDA | VL53L0X SDA (all 6 on shared bus) |
| GP21 | I2C1 SCL | VL53L0X SCL (all 6 on shared bus) |
| GP26 | ADC0 | Battery divider (optional, `BEN=1`) |
| 3V3 | Power | MPU-6050 VCC, VL53L0X VCC |
| GND | Ground | Common ground |

**VL53L0X wiring (6 sensors on shared I2C1 bus):**

| VL53L0X pin | Connect to |
|---|---|
| SDA | GP20 (shared bus, Wire1) |
| SCL | GP21 (shared bus, Wire1) |
| XSHUT | Individual pin per sensor (GP6-GP9, GP14-GP15) |
| VCC | 3.3V |
| GND | GND |

I2C addresses are assigned at boot (0x30-0x35) via XSHUT pin sequencing — all sensors start in reset, then are enabled one at a time (50 ms delay per sensor). I2C1 runs at 100 kHz for reliable communication.

---

## OLED Display + Rotary Encoder (optional, `USE_OLED_MENU=1`)

Same wiring for both sensor configs.

| Pico 2 Pin | Function | Connected to |
|---|---|---|
| GP0 | I2C0 SDA | SSD1306 SDA (shared bus with MPU-6050) |
| GP1 | I2C0 SCL | SSD1306 SCL (shared bus with MPU-6050) |
| GP22 | Digital input | Rotary encoder CLK |
| GP12 | Digital input | Rotary encoder DT |
| GP19 | Digital input | Rotary encoder SW (button) |
| 3V3 | Power | SSD1306 VCC, encoder VCC |
| GND | Ground | SSD1306 GND, encoder GND |

OLED I2C address `0x3C` — no conflict with MPU-6050 at `0x68`. If `USE_IMU=0`, the menu initialises Wire itself.

---

## WiFi Bridge — Wemos D1 Mini (ESP8266)

Same wiring for both sensor configs.

| D1 Mini pin | Pico 2 pin |
|---|---|
| RX | GP16 (UART1 TX) |
| TX | GP17 (UART1 RX) |
| 5V | Separate USB (recommended) or shared 5V |
| GND | GND |

---

## Battery Monitoring (optional)

Disabled by default (`BEN=0`). Enable via `$SET:BEN=1`.

```
Battery+ ──[18k R1]──┬──[10k R2]── GND
                      └── GP26 (ADC0)
```

Multiplier: `BML = (R1 + R2) / R2 = 2.8` (default). Max safe input: ~9.4V with these resistors (3.3V ADC limit).

---

## Pin Map Summary

```
         ┌─────────────────────┐
         │    Pico 2 (RP2350)  │
         │                     │
  IMU ── │ GP0  SDA      GP26 │ ── Battery ADC
 OLED    │ (shared I2C0)      │
  IMU ── │ GP1  SCL      GP22 │ ── Encoder DT
 OLED    │ (shared I2C0)      │
         │ GP2  Luna[0]  GP21 │ ── VL53 SCL
         │ GP3  Luna[1]  GP20 │ ── VL53 SDA
         │ GP4  Luna[2]  GP19 │ ── Encoder SW
         │ GP5  Luna[3]  GP18 │
  VL53 ──│ GP6  XSHUT[0] GP17 │ ── D1 Mini TX
  VL53 ──│ GP7  XSHUT[1] GP16 │ ── D1 Mini RX
  VL53 ──│ GP8  XSHUT[2] GP15 │ ── VL53 XSHUT[5]
  VL53 ──│ GP9  XSHUT[3] GP14 │ ── VL53 XSHUT[4]
 Servo ──│ GP10 PWM       GP13 │ ── Tachometer
   ESC ──│ GP11 PWM       GP12 │ ── Encoder DT
         │                     │
         └─────────────────────┘

  4x TF-Luna uses:  GP2-GP5 (UART RX)
  6x VL53L0X uses:  GP6-GP9, GP14-GP15 (XSHUT) + GP20-GP21 (I2C1)
  OLED menu uses:    GP0-GP1 (shared I2C0), GP22 (CLK), GP19 (SW), GP12 (DT)
  Both configs use:  GP0-GP1 (IMU/OLED), GP10-GP11 (servo/ESC), GP13 (tacho),
                     GP16-GP17 (WiFi UART), GP26 (battery)
```
