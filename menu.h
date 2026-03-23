#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// menu.h — OLED SSD1306 128x64 menu system with rotary encoder
//
// Gated by USE_OLED_MENU. When disabled (0), all functions become no-ops.
//
// Hardware:
//   OLED SSD1306 128x64:  I2C0 — GP0 (SDA), GP1 (SCL), addr 0x3C
//   Rotary encoder:       GP12 (CLK), GP22 (DT), GP19 (SW/button)
//
// The OLED shares I2C0 with the MPU-6050 IMU (addr 0x68) — no conflict.
// ─────────────────────────────────────────────────────────────────────────────

#if USE_OLED_MENU

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EncButton.h>

// ─── Pin definitions ─────────────────────────────────────────────────────────
#define ENC_CLK_PIN   22
#define ENC_DT_PIN    12
#define ENC_SW_PIN    19

// ─── Display ─────────────────────────────────────────────────────────────────
#define SCREEN_W      128
#define SCREEN_H       64
#define OLED_ADDR    0x3C

static Adafruit_SSD1306 _oled(SCREEN_W, SCREEN_H, &Wire, -1);
static EncButton _enc(ENC_CLK_PIN, ENC_DT_PIN, ENC_SW_PIN);

// ─── Timing ──────────────────────────────────────────────────────────────────
static unsigned long _menu_last_draw = 0;
static const unsigned long MENU_DRAW_INTERVAL = 120;  // ~8 FPS

// ─── Screen state machine ────────────────────────────────────────────────────
enum MenuScreen : uint8_t {
    SCR_DASHBOARD,
    SCR_MAIN_MENU,
    SCR_SETTINGS_GROUPS,
    SCR_SETTINGS_LIST,
    SCR_SETTINGS_EDIT,
    SCR_TESTS,
    SCR_TEST_RUNNING,
    SCR_ACTIONS,
    SCR_CONFIRM,
    SCR_INFO,
};

static MenuScreen _scr         = SCR_DASHBOARD;
static MenuScreen _scr_prev    = SCR_DASHBOARD;  // for back navigation
static int        _sel         = 0;               // cursor position in lists
static int        _scroll      = 0;               // scroll offset for lists

// ─── Parameter descriptor ────────────────────────────────────────────────────
enum ParamType : uint8_t { PT_INT, PT_FLOAT, PT_BOOL };

struct ParamDesc {
    const char* key;       // short name (matches $GET key)
    const char* label;     // display name (max ~12 chars)
    ParamType   type;
    float       step;      // normal encoder step
    float       step_fast; // fast rotation step
    float       lo;        // min value
    float       hi;        // max value
    void*       ptr;       // pointer to cfg_* global
};

// Forward-declare cfg_* globals (defined in .ino)
extern int   cfg_front_obstacle_dist;
extern int   cfg_side_open_dist;
extern int   cfg_all_close_dist;
extern int   cfg_close_front_dist;
extern float cfg_pid_kp;
extern float cfg_pid_ki;
extern float cfg_pid_kd;
extern int   cfg_min_speed;
extern int   cfg_max_speed;
extern int   cfg_min_bspeed;
extern int   cfg_min_point;
extern int   cfg_max_point;
extern int   cfg_neutral_point;
extern int   cfg_encoder_holes;
extern float cfg_wheel_diam_m;
extern int   cfg_loop_ms;
extern float cfg_spd_clear;
extern float cfg_spd_blocked;
extern float cfg_coe_clear;
extern float cfg_coe_blocked;
extern float cfg_wrong_dir_deg;
extern bool  cfg_race_cw;
extern int   cfg_stuck_thresh;
extern bool  cfg_imu_rotate;
extern bool  cfg_servo_reverse;
extern bool  cfg_calibrated;
extern bool  cfg_bat_enabled;
extern float cfg_bat_multiplier;
extern float cfg_bat_low;

// ─── Parameter table (27 writable params) ────────────────────────────────────
static const ParamDesc _params[] = {
    // Obstacles (cm×10)
    {"FOD", "Front Obs",  PT_INT,   10,  100,   50, 9000, &cfg_front_obstacle_dist},
    {"SOD", "Side Open",  PT_INT,   10,  100,   50, 9000, &cfg_side_open_dist},
    {"ACD", "All Close",  PT_INT,   10,  100,   50, 9000, &cfg_all_close_dist},
    {"CFD", "Close Frt",  PT_INT,   10,  100,   50, 9000, &cfg_close_front_dist},
    // PID
    {"KP",  "PID Kp",     PT_FLOAT, 0.1f, 1.0f, 0, 200,  &cfg_pid_kp},
    {"KI",  "PID Ki",     PT_FLOAT, 0.1f, 1.0f, 0, 200,  &cfg_pid_ki},
    {"KD",  "PID Kd",     PT_FLOAT, 0.1f, 1.0f, 0, 200,  &cfg_pid_kd},
    // Speed / ESC (µs)
    {"MSP", "Min Spd",    PT_INT,   5,   20,  1500, 2000, &cfg_min_speed},
    {"XSP", "Max Spd",    PT_INT,   5,   20,  1500, 2000, &cfg_max_speed},
    {"BSP", "Min Bck",    PT_INT,   5,   20,  1000, 1500, &cfg_min_bspeed},
    // Steering (degrees)
    {"MNP", "Srv Min",    PT_INT,   1,    5,    0, 90,    &cfg_min_point},
    {"XNP", "Srv Max",    PT_INT,   1,    5,   90, 180,   &cfg_max_point},
    {"NTP", "Srv Neu",    PT_INT,   1,    5,    0, 180,   &cfg_neutral_point},
    // Tachometer
    {"ENH", "Enc Holes",  PT_INT,   1,   10,    1, 200,   &cfg_encoder_holes},
    {"WDM", "Wheel D",    PT_FLOAT, 0.001f, 0.01f, 0.01f, 0.5f, &cfg_wheel_diam_m},
    // Control
    {"LMS", "Loop ms",    PT_INT,   5,   10,   10, 200,   &cfg_loop_ms},
    {"SPD1","Spd Clr",    PT_FLOAT, 0.1f, 0.5f, 0, 5.0f,  &cfg_spd_clear},
    {"SPD2","Spd Blk",    PT_FLOAT, 0.1f, 0.5f, 0, 5.0f,  &cfg_spd_blocked},
    {"COE1","Coe Clr",    PT_FLOAT, 0.05f, 0.1f, 0, 2.0f, &cfg_coe_clear},
    {"COE2","Coe Blk",    PT_FLOAT, 0.05f, 0.1f, 0, 2.0f, &cfg_coe_blocked},
    // Navigation
    {"WDD", "Wrong Dir",  PT_FLOAT, 5.0f, 20.0f, 10, 360, &cfg_wrong_dir_deg},
    {"RCW", "Race CW",    PT_BOOL,  1,    1,     0, 1,    &cfg_race_cw},
    {"STK", "Stuck Thr",  PT_INT,   1,    5,     1, 100,  &cfg_stuck_thresh},
    // Hardware flags
    {"IMR", "IMU Rot",    PT_BOOL,  1,    1,     0, 1,    &cfg_imu_rotate},
    {"SVR", "Srv Rev",    PT_BOOL,  1,    1,     0, 1,    &cfg_servo_reverse},
    {"BEN", "Bat En",     PT_BOOL,  1,    1,     0, 1,    &cfg_bat_enabled},
    {"BML", "Bat Mult",   PT_FLOAT, 0.1f, 0.5f, 1.0f, 10.0f, &cfg_bat_multiplier},
    {"BLV", "Bat Low V",  PT_FLOAT, 0.1f, 0.5f, 3.0f, 12.0f, &cfg_bat_low},
};
static const int PARAM_COUNT = sizeof(_params) / sizeof(_params[0]);

// ─── Parameter groups ────────────────────────────────────────────────────────
struct ParamGroup {
    const char* name;
    uint8_t     start;  // index into _params[]
    uint8_t     count;
};

static const ParamGroup _groups[] = {
    {"Obstacles",   0,  4},
    {"PID",         4,  3},
    {"Speed/ESC",   7,  3},
    {"Steering",   10,  3},
    {"Tachometer", 13,  2},
    {"Control",    15,  5},
    {"Navigation", 20,  3},
    {"Hardware",   23,  5},
};
static const int GROUP_COUNT = sizeof(_groups) / sizeof(_groups[0]);

// ─── Test names ──────────────────────────────────────────────────────────────
struct TestItem {
    const char* name;     // command name (matches $TEST:<name>)
    const char* label;    // display label
    bool        motor;    // true = needs confirm (motor tests)
};

static const TestItem _tests[] = {
    {"lidar",    "Lidar Scan",  false},
    {"servo",    "Servo Sweep", false},
    {"taho",     "Tachometer",  false},
    {"esc",      "ESC Test",    true},
    {"speed",    "Speed Hold",  true},
    {"autotune", "PID Autotune",true},
    {"reactive", "Reactive",    false},
    {"cal",      "ESC Calibr.", true},
};
static const int TEST_COUNT = sizeof(_tests) / sizeof(_tests[0]);

// ─── Action items ────────────────────────────────────────────────────────────
enum ActionId : uint8_t {
    ACT_START, ACT_STOP, ACT_SAVE, ACT_LOAD, ACT_RESET
};

struct ActionItem {
    const char* label;
    ActionId    id;
    bool        confirm;  // show confirm dialog first
};

static const ActionItem _actions[] = {
    {"Start Car",    ACT_START, true},
    {"Stop Car",     ACT_STOP,  false},
    {"Save EEPROM",  ACT_SAVE,  true},
    {"Load EEPROM",  ACT_LOAD,  true},
    {"Reset Defaults",ACT_RESET,true},
};
static const int ACTION_COUNT = sizeof(_actions) / sizeof(_actions[0]);

// ─── State for sub-screens ───────────────────────────────────────────────────
static int   _grp_sel       = 0;    // selected group in SETTINGS_GROUPS
static int   _param_idx     = 0;    // index into _params[] for edit screen
static float _edit_val      = 0;    // value being edited
static int   _test_idx      = 0;    // which test is selected/running
static int   _action_idx    = 0;    // which action is pending confirm
static bool  _confirm_yes   = false;// cursor in confirm dialog

// ─── Helpers ─────────────────────────────────────────────────────────────────

static float _param_get(const ParamDesc& p) {
    switch (p.type) {
        case PT_INT:   return (float)(*(int*)p.ptr);
        case PT_FLOAT: return *(float*)p.ptr;
        case PT_BOOL:  return *(bool*)p.ptr ? 1.0f : 0.0f;
    }
    return 0;
}

static void _param_set(const ParamDesc& p, float v) {
    v = constrain(v, p.lo, p.hi);
    switch (p.type) {
        case PT_INT:   *(int*)p.ptr   = (int)v;     break;
        case PT_FLOAT: *(float*)p.ptr = v;           break;
        case PT_BOOL:  *(bool*)p.ptr  = (v > 0.5f); break;
    }
}

// Print a param value to a buffer (returns chars written)
static int _param_fmt(char* buf, int sz, const ParamDesc& p, float v) {
    switch (p.type) {
        case PT_INT:   return snprintf(buf, sz, "%d", (int)v);
        case PT_FLOAT: return snprintf(buf, sz, "%.2f", (double)v);
        case PT_BOOL:  return snprintf(buf, sz, "%s", v > 0.5f ? "ON" : "OFF");
    }
    return 0;
}

// Clamp selection and compute scroll offset for a list
// visible_lines = how many items fit on screen, total = list length
static void _clamp_scroll(int& sel, int& scroll, int total, int visible) {
    if (sel < 0) sel = 0;
    if (sel >= total) sel = total - 1;
    if (sel < scroll) scroll = sel;
    if (sel >= scroll + visible) scroll = sel - visible + 1;
}

// ─── Screen drawing ──────────────────────────────────────────────────────────

// Forward declarations
extern Car   car;
extern bool  car_running;

static void _draw_dashboard() {
    // Line 0: status bar
    _oled.setCursor(0, 0);
    _oled.print(car_running ? "RUN " : "STOP");

    // Battery voltage (if enabled)
    extern bool cfg_bat_enabled;
    if (cfg_bat_enabled) {
        char vbuf[8];
        snprintf(vbuf, sizeof(vbuf), " %4.1fV", (double)car.bat_voltage);
        _oled.print(vbuf);
    }

    // Speed
    float spd = get_speed();
    char sbuf[12];
    snprintf(sbuf, sizeof(sbuf), " %4.2fm/s", (double)spd);
    // Right-align speed
    int sw = strlen(sbuf) * 6;
    _oled.setCursor(SCREEN_W - sw, 0);
    _oled.print(sbuf);

    // Separator line
    _oled.drawFastHLine(0, 10, SCREEN_W, SSD1306_WHITE);

    // Sensor bars (visual representation)
    int* s = car.read_sensors();
    const int bar_y = 14;
    const int bar_h = 5;
    const int bar_spacing = 7;
    const int bar_max_w = 76;   // leave room for value text on the right
    const int val_x = 102;      // value text starts here
    const int max_range = MAX_SENSOR_RANGE;

    for (int i = 0; i < SENSOR_COUNT && i < 6; i++) {
        int y = bar_y + i * bar_spacing;
        // Label
        _oled.setCursor(0, y);
        _oled.print(_sensor_short[i]);

        // Bar
        int val = (s[i] == 9999) ? 0 : s[i];
        int bw = (int)((long)val * bar_max_w / max_range);
        bw = constrain(bw, 0, bar_max_w);
        _oled.drawRect(20, y, bar_max_w + 2, bar_h, SSD1306_WHITE);
        if (bw > 0) {
            _oled.fillRect(21, y + 1, bw, bar_h - 2, SSD1306_WHITE);
        }

        // Value text (cm)
        char vb[6];
        if (s[i] == 9999) snprintf(vb, sizeof(vb), "--");
        else               snprintf(vb, sizeof(vb), "%d", s[i] / 10);
        _oled.setCursor(val_x, y);
        _oled.print(vb);
    }

#if USE_IMU
    // IMU heading at bottom
    _oled.setCursor(0, 56);
    char ibuf[22];
    snprintf(ibuf, sizeof(ibuf), "Yaw:%5.1f Hdg:%5.1f", (double)car.yaw_rate, (double)car.heading);
    _oled.print(ibuf);
#endif
}

// Generic list drawer: title on line 0, items below
// Returns number of visible lines used (for scrolling calc)
static const int LIST_VISIBLE = 6;  // 64px / 10px per line = 6 item lines (line 0 is title)

static void _draw_list(const char* title, int count,
                        void (*item_fn)(int idx, char* buf, int sz),
                        int sel, int scroll) {
    // Title (inverted)
    _oled.fillRect(0, 0, SCREEN_W, 10, SSD1306_WHITE);
    _oled.setTextColor(SSD1306_BLACK);
    _oled.setCursor(2, 1);
    _oled.print(title);
    _oled.setTextColor(SSD1306_WHITE);

    // Items
    for (int i = 0; i < LIST_VISIBLE && (scroll + i) < count; i++) {
        int idx = scroll + i;
        int y = 12 + i * 9;

        if (idx == sel) {
            _oled.fillRect(0, y - 1, SCREEN_W, 9, SSD1306_WHITE);
            _oled.setTextColor(SSD1306_BLACK);
        }

        _oled.setCursor(2, y);
        char buf[22];
        item_fn(idx, buf, sizeof(buf));
        _oled.print(buf);

        if (idx == sel) {
            _oled.setTextColor(SSD1306_WHITE);
        }
    }

    // Scroll indicator
    if (count > LIST_VISIBLE) {
        int bar_h = max(4, (int)(SCREEN_H * LIST_VISIBLE / count));
        int bar_y = (int)((long)scroll * (SCREEN_H - bar_h) / max(1, count - LIST_VISIBLE));
        _oled.fillRect(SCREEN_W - 2, bar_y, 2, bar_h, SSD1306_WHITE);
    }
}

// ─── Main menu items ─────────────────────────────────────────────────────────
static const char* _main_items[] = {"Settings", "Tests", "Actions", "Info"};
static const int MAIN_COUNT = 4;

static void _main_item_fn(int idx, char* buf, int sz) {
    snprintf(buf, sz, "%s", _main_items[idx]);
}

// ─── Settings groups item renderer ───────────────────────────────────────────
static void _group_item_fn(int idx, char* buf, int sz) {
    snprintf(buf, sz, "%s (%d)", _groups[idx].name, _groups[idx].count);
}

// ─── Settings list (params in a group) ───────────────────────────────────────
static void _param_item_fn(int idx, char* buf, int sz) {
    int pi = _groups[_grp_sel].start + idx;
    const ParamDesc& p = _params[pi];
    char vbuf[10];
    _param_fmt(vbuf, sizeof(vbuf), p, _param_get(p));
    snprintf(buf, sz, "%-8s %s", p.label, vbuf);
}

// ─── Tests item renderer ────────────────────────────────────────────────────
static void _test_item_fn(int idx, char* buf, int sz) {
    snprintf(buf, sz, "%s%s", _tests[idx].label, _tests[idx].motor ? " [!]" : "");
}

// ─── Actions item renderer ──────────────────────────────────────────────────
static void _action_item_fn(int idx, char* buf, int sz) {
    snprintf(buf, sz, "%s", _actions[idx].label);
}

// ─── Settings edit screen ────────────────────────────────────────────────────
static void _draw_edit() {
    const ParamDesc& p = _params[_param_idx];

    // Title
    _oled.fillRect(0, 0, SCREEN_W, 10, SSD1306_WHITE);
    _oled.setTextColor(SSD1306_BLACK);
    _oled.setCursor(2, 1);
    _oled.print(p.label);
    _oled.setTextColor(SSD1306_WHITE);

    // Key
    _oled.setCursor(2, 14);
    _oled.print("Key: ");
    _oled.print(p.key);

    // Value (2x size)
    _oled.setTextSize(2);
    char vbuf[12];
    _param_fmt(vbuf, sizeof(vbuf), p, _edit_val);
    // Center value
    int tw = strlen(vbuf) * 12;  // 6px * 2 scale
    _oled.setCursor((SCREEN_W - tw) / 2, 28);
    _oled.print(vbuf);
    _oled.setTextSize(1);

    // Range
    char rbuf[22];
    if (p.type == PT_BOOL) {
        snprintf(rbuf, sizeof(rbuf), "Turn to toggle");
    } else if (p.type == PT_INT) {
        snprintf(rbuf, sizeof(rbuf), "Range: %d..%d", (int)p.lo, (int)p.hi);
    } else {
        snprintf(rbuf, sizeof(rbuf), "Range: %.1f..%.1f", (double)p.lo, (double)p.hi);
    }
    _oled.setCursor(2, 48);
    _oled.print(rbuf);

    // Hint
    _oled.setCursor(2, 56);
    _oled.print("Click=OK  Hold=Cancel");
}

// ─── Confirm dialog ──────────────────────────────────────────────────────────
static const char* _confirm_msg = "";

static void _draw_confirm() {
    _oled.setCursor(2, 8);
    _oled.print(_confirm_msg);

    _oled.setCursor(2, 24);
    _oled.print("Are you sure?");

    // Yes / No buttons
    int y = 42;
    // Yes
    if (_confirm_yes) {
        _oled.fillRect(10, y - 2, 44, 14, SSD1306_WHITE);
        _oled.setTextColor(SSD1306_BLACK);
    }
    _oled.setCursor(16, y);
    _oled.print("[ Yes ]");
    if (_confirm_yes) _oled.setTextColor(SSD1306_WHITE);

    // No
    if (!_confirm_yes) {
        _oled.fillRect(70, y - 2, 44, 14, SSD1306_WHITE);
        _oled.setTextColor(SSD1306_BLACK);
    }
    _oled.setCursor(76, y);
    _oled.print("[ No ]");
    if (!_confirm_yes) _oled.setTextColor(SSD1306_WHITE);
}

// ─── Test running screen ─────────────────────────────────────────────────────
static void _draw_test_running() {
    _oled.setCursor(2, 10);
    _oled.setTextSize(2);
    _oled.print("Running");
    _oled.setTextSize(1);

    _oled.setCursor(2, 34);
    _oled.print("Test: ");
    _oled.print(_tests[_test_idx].label);

    // Animated dots
    static uint8_t dots = 0;
    dots = (dots + 1) % 4;
    _oled.setCursor(2, 48);
    for (uint8_t i = 0; i < dots; i++) _oled.print('.');

    _oled.setCursor(2, 56);
    _oled.print("Click = Abort");
}

// ─── Info screen ─────────────────────────────────────────────────────────────
// Info screen: scrollable list of status lines
static const int INFO_VISIBLE = 6;  // visible lines after title bar
static int _info_scroll = 0;

// Build info lines into a buffer, return count
static int _info_build(char lines[][22]) {
    int n = 0;

    snprintf(lines[n++], 22, "FW: v%s", FW_VERSION);

#if SENSOR_CONFIG == SENSOR_4X_LUNA
    snprintf(lines[n++], 22, "Sensors: %d (Luna)", SENSOR_COUNT);
#elif SENSOR_CONFIG == SENSOR_6X_VL53L0X
    snprintf(lines[n++], 22, "Sensors: %d (VL53)", SENSOR_COUNT);
#else
    snprintf(lines[n++], 22, "Sensors: %d", SENSOR_COUNT);
#endif

#if USE_IMU
    snprintf(lines[n++], 22, "IMU: %s", car.imu_ok ? "OK" : "FAIL");
#else
    snprintf(lines[n++], 22, "IMU: Disabled");
#endif

    if (cfg_bat_enabled) {
        snprintf(lines[n++], 22, "Bat: %.1fV", (double)car.bat_voltage);
    } else {
        snprintf(lines[n++], 22, "Bat: Disabled");
    }

    snprintf(lines[n++], 22, "Loop: %dms  Enc: %d", cfg_loop_ms, cfg_encoder_holes);

    // WiFi section
#if USE_WIFI_DEBUG
    extern bool esp_wifi_ready;
    extern bool esp_wifi_is_ap;
    extern char esp_wifi_ssid[];
    extern char esp_wifi_ip[];

    if (!esp_wifi_ready) {
        snprintf(lines[n++], 22, "WiFi: waiting...");
    } else {
        snprintf(lines[n++], 22, "WiFi: %s", esp_wifi_is_ap ? "AP" : "STA");
    }
    if (esp_wifi_ssid[0]) {
        snprintf(lines[n++], 22, "Net: %s", esp_wifi_ssid);
    }
    if (esp_wifi_ip[0]) {
        snprintf(lines[n++], 22, "IP: %s", esp_wifi_ip);
    }
#else
    snprintf(lines[n++], 22, "WiFi: Disabled");
#endif

    return n;
}

static void _draw_info() {
    // Title bar
    _oled.fillRect(0, 0, SCREEN_W, 10, SSD1306_WHITE);
    _oled.setTextColor(SSD1306_BLACK);
    _oled.setCursor(2, 1);
    _oled.print("Info");
    _oled.setTextColor(SSD1306_WHITE);

    char lines[12][22];
    int count = _info_build(lines);

    // Clamp scroll
    if (_info_scroll > count - INFO_VISIBLE) _info_scroll = count - INFO_VISIBLE;
    if (_info_scroll < 0) _info_scroll = 0;

    // Draw visible lines
    for (int i = 0; i < INFO_VISIBLE && (_info_scroll + i) < count; i++) {
        _oled.setCursor(2, 12 + i * 9);
        _oled.print(lines[_info_scroll + i]);
    }

    // Scroll indicator
    if (count > INFO_VISIBLE) {
        int bar_h = max(4, (int)(SCREEN_H * INFO_VISIBLE / count));
        int bar_y = (int)((long)_info_scroll * (SCREEN_H - bar_h) / max(1, count - INFO_VISIBLE));
        _oled.fillRect(SCREEN_W - 2, bar_y, 2, bar_h, SSD1306_WHITE);
    }
}

// ─── Navigation logic ────────────────────────────────────────────────────────

static void _go_screen(MenuScreen scr) {
    _scr_prev = _scr;
    _scr = scr;
    _sel = 0;
    _scroll = 0;
}

static void _go_back() {
    switch (_scr) {
        case SCR_MAIN_MENU:       _scr = SCR_DASHBOARD;       break;
        case SCR_SETTINGS_GROUPS: _scr = SCR_MAIN_MENU;       break;
        case SCR_SETTINGS_LIST:   _scr = SCR_SETTINGS_GROUPS; break;
        case SCR_SETTINGS_EDIT:   _scr = SCR_SETTINGS_LIST;   break;
        case SCR_TESTS:           _scr = SCR_MAIN_MENU;       break;
        case SCR_ACTIONS:         _scr = SCR_MAIN_MENU;       break;
        case SCR_CONFIRM:         _scr = _scr_prev;           break;
        case SCR_INFO:            _scr = SCR_MAIN_MENU;       break;
        case SCR_TEST_RUNNING:    break;  // can't go back, click = abort
        default:                  _scr = SCR_DASHBOARD;       break;
    }
    _sel = 0;
    _scroll = 0;
}

// Execute a confirmed action
extern bool save_settings();
extern bool load_settings();

static void _exec_action(ActionId id) {
    switch (id) {
        case ACT_START:
            car_running = true;
            break;
        case ACT_STOP:
            car_running = false;
            car.write_speed(0);
            car.write_steer(0);
            break;
        case ACT_SAVE:
            save_settings();
            break;
        case ACT_LOAD:
            load_settings();
            break;
        case ACT_RESET: {
            // Reuse the cmd_rst logic — reset all cfg_* to defaults
            cfg_front_obstacle_dist = DEFAULT_FOD;
            cfg_side_open_dist      = DEFAULT_SOD;
            cfg_all_close_dist      = DEFAULT_ACD;
            cfg_close_front_dist    = DEFAULT_CFD;
            cfg_pid_kp   = 60.0f;
            cfg_pid_ki   = 40.0f;
            cfg_pid_kd   = 6.0f;
            cfg_min_speed   = 1540;
            cfg_max_speed   = 1700;
            cfg_min_bspeed  = 1460;
            cfg_min_point     = 40;
            cfg_max_point     = 140;
            cfg_neutral_point = 90;
            cfg_encoder_holes = 62;
            cfg_wheel_diam_m  = 0.060f;
            cfg_loop_ms       = 40;
            cfg_spd_clear     = 2.7f;
            cfg_spd_blocked   = 0.8f;
            cfg_coe_clear     = 0.3f;
            cfg_coe_blocked   = 0.7f;
            cfg_wrong_dir_deg = 120.0f;
            cfg_race_cw       = true;
            cfg_stuck_thresh  = 25;
            cfg_imu_rotate    = true;
            cfg_servo_reverse = false;
            cfg_calibrated    = false;
            cfg_bat_enabled    = false;
            cfg_bat_multiplier = 2.8f;
            cfg_bat_low        = 6.0f;
            break;
        }
    }
}

// Run a test via WiFi test functions (defined in .ino) or Serial test functions (tests.h)
extern void run_calibration();

#if USE_WIFI_DEBUG
extern void wifi_test_lidar();
extern void wifi_test_servo();
extern void wifi_test_taho();
extern void wifi_test_esc();
extern void wifi_test_speed();
extern void wifi_test_autotune();
extern void wifi_test_reactive();
#endif

static void _run_test(int idx) {
    _test_idx = idx;
    _go_screen(SCR_TEST_RUNNING);

    // Auto-stop car before test
    if (car_running) {
        car_running = false;
        car.write_speed(0);
        car.write_steer(0);
    }

#if USE_WIFI_DEBUG
    switch (idx) {
        case 0: wifi_test_lidar();    break;
        case 1: wifi_test_servo();    break;
        case 2: wifi_test_taho();     break;
        case 3: wifi_test_esc();      break;
        case 4: wifi_test_speed();    break;
        case 5: wifi_test_autotune(); break;
        case 6: wifi_test_reactive(); break;
        case 7: cfg_calibrated = false; run_calibration(); break;
    }
#else
    // Without WiFi, use Serial-based tests from tests.h
    switch (idx) {
        case 0: test_lidar(car, 5000);   break;
        case 1: test_servo(car);          break;
        case 2: test_taho(car, 5000);     break;
        case 3: test_esc(car);            break;
        case 4: test_speed_hold(car);     break;
        case 5: test_autotune(car);       break;
        case 6: test_reactive(car);       break;
        case 7: cfg_calibrated = false; run_calibration(); break;
    }
#endif

    // Test finished — go back to tests list
    _scr = SCR_TESTS;
    _sel = idx;
    _scroll = 0;
}

// ─── Input handling ──────────────────────────────────────────────────────────

static void _handle_input() {
    _enc.tick();

    int dir = 0;
    if (_enc.left())  dir = -1;
    if (_enc.right()) dir = 1;
    bool fast = _enc.fast();
    bool click = _enc.click();
    bool held = _enc.hold();

    // Long-press from anywhere → dashboard
    if (held) {
        if (_scr == SCR_SETTINGS_EDIT) {
            // Cancel edit — don't apply value
            _scr = SCR_SETTINGS_LIST;
        } else {
            _scr = SCR_DASHBOARD;
        }
        _sel = 0;
        _scroll = 0;
        return;
    }

    switch (_scr) {
        case SCR_DASHBOARD:
            if (click) _go_screen(SCR_MAIN_MENU);
            break;

        case SCR_MAIN_MENU:
            _sel += dir;
            _clamp_scroll(_sel, _scroll, MAIN_COUNT, LIST_VISIBLE);
            if (click) {
                switch (_sel) {
                    case 0: _go_screen(SCR_SETTINGS_GROUPS); break;
                    case 1: _go_screen(SCR_TESTS);           break;
                    case 2: _go_screen(SCR_ACTIONS);          break;
                    case 3: _go_screen(SCR_INFO);             break;
                }
            }
            break;

        case SCR_SETTINGS_GROUPS:
            _sel += dir;
            _clamp_scroll(_sel, _scroll, GROUP_COUNT, LIST_VISIBLE);
            if (click) {
                _grp_sel = _sel;
                _go_screen(SCR_SETTINGS_LIST);
            }
            break;

        case SCR_SETTINGS_LIST: {
            int cnt = _groups[_grp_sel].count;
            _sel += dir;
            _clamp_scroll(_sel, _scroll, cnt, LIST_VISIBLE);
            if (click) {
                _param_idx = _groups[_grp_sel].start + _sel;
                const ParamDesc& p = _params[_param_idx];
                _edit_val = _param_get(p);
                _go_screen(SCR_SETTINGS_EDIT);
            }
            break;
        }

        case SCR_SETTINGS_EDIT: {
            const ParamDesc& p = _params[_param_idx];
            if (dir != 0) {
                if (p.type == PT_BOOL) {
                    _edit_val = (_edit_val > 0.5f) ? 0.0f : 1.0f;
                } else {
                    float s = fast ? p.step_fast : p.step;
                    _edit_val += dir * s;
                    _edit_val = constrain(_edit_val, p.lo, p.hi);
                }
            }
            if (click) {
                // Apply value
                _param_set(p, _edit_val);
                _scr = SCR_SETTINGS_LIST;
                _sel = _param_idx - _groups[_grp_sel].start;
                _scroll = 0;
                _clamp_scroll(_sel, _scroll, _groups[_grp_sel].count, LIST_VISIBLE);
            }
            break;
        }

        case SCR_TESTS:
            _sel += dir;
            _clamp_scroll(_sel, _scroll, TEST_COUNT, LIST_VISIBLE);
            if (click) {
                if (_tests[_sel].motor) {
                    // Show confirm dialog
                    _test_idx = _sel;
                    _confirm_msg = _tests[_sel].label;
                    _confirm_yes = false;
                    _scr_prev = SCR_TESTS;
                    _go_screen(SCR_CONFIRM);
                } else {
                    _run_test(_sel);
                }
            }
            break;

        case SCR_TEST_RUNNING:
            // Click aborts (the test functions check Serial, not encoder,
            // but we go back to menu on click)
            if (click) {
                car.write_speed(0);
                car.write_steer(0);
                _scr = SCR_TESTS;
            }
            break;

        case SCR_ACTIONS:
            _sel += dir;
            _clamp_scroll(_sel, _scroll, ACTION_COUNT, LIST_VISIBLE);
            if (click) {
                _action_idx = _sel;
                if (_actions[_sel].confirm) {
                    _confirm_msg = _actions[_sel].label;
                    _confirm_yes = false;
                    _scr_prev = SCR_ACTIONS;
                    _go_screen(SCR_CONFIRM);
                } else {
                    _exec_action(_actions[_sel].id);
                    _go_screen(SCR_DASHBOARD);
                }
            }
            break;

        case SCR_CONFIRM:
            if (dir != 0) _confirm_yes = !_confirm_yes;
            if (click) {
                if (_confirm_yes) {
                    // Execute the confirmed action
                    if (_scr_prev == SCR_TESTS) {
                        _run_test(_test_idx);
                    } else if (_scr_prev == SCR_ACTIONS) {
                        _exec_action(_actions[_action_idx].id);
                        _go_screen(SCR_DASHBOARD);
                    }
                } else {
                    _go_back();
                }
            }
            break;

        case SCR_INFO:
            if (dir != 0) _info_scroll += dir;
            if (click) { _info_scroll = 0; _go_back(); }
            break;
    }
}

// ─── Redraw ──────────────────────────────────────────────────────────────────

static void _draw_screen() {
    _oled.clearDisplay();
    _oled.setTextSize(1);
    _oled.setTextColor(SSD1306_WHITE);

    switch (_scr) {
        case SCR_DASHBOARD:
            _draw_dashboard();
            break;
        case SCR_MAIN_MENU:
            _draw_list("Menu", MAIN_COUNT, _main_item_fn, _sel, _scroll);
            break;
        case SCR_SETTINGS_GROUPS:
            _draw_list("Settings", GROUP_COUNT, _group_item_fn, _sel, _scroll);
            break;
        case SCR_SETTINGS_LIST:
            _draw_list(_groups[_grp_sel].name, _groups[_grp_sel].count,
                       _param_item_fn, _sel, _scroll);
            break;
        case SCR_SETTINGS_EDIT:
            _draw_edit();
            break;
        case SCR_TESTS:
            _draw_list("Tests", TEST_COUNT, _test_item_fn, _sel, _scroll);
            break;
        case SCR_TEST_RUNNING:
            _draw_test_running();
            break;
        case SCR_ACTIONS:
            _draw_list("Actions", ACTION_COUNT, _action_item_fn, _sel, _scroll);
            break;
        case SCR_CONFIRM:
            _draw_confirm();
            break;
        case SCR_INFO:
            _draw_info();
            break;
    }

    _oled.display();
}

// ─── Public API ──────────────────────────────────────────────────────────────

void menu_init() {
#if !USE_IMU
    // If IMU is disabled, we need to init Wire ourselves
    Wire.setSDA(0);
    Wire.setSCL(1);
    Wire.begin();
    Wire.setClock(400000);
#endif

    if (!_oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("OLED: SSD1306 not found");
        return;
    }

    // Boot splash
    _oled.clearDisplay();
    _oled.setTextSize(2);
    _oled.setTextColor(SSD1306_WHITE);
    _oled.setCursor(16, 8);
    _oled.print("Umbreon");
    _oled.setTextSize(1);
    _oled.setCursor(32, 32);
    _oled.print("Roborace");
    _oled.setCursor(40, 48);
    _oled.print("v");
    _oled.print(FW_VERSION);
    _oled.display();

    _enc.setEncType(EB_STEP4_LOW);  // standard encoder with 4 steps per detent
    Serial.println("OLED: menu ready");
}

void menu_tick() {
    _handle_input();

    // Throttle redraws
    unsigned long now = millis();
    if (now - _menu_last_draw >= MENU_DRAW_INTERVAL) {
        _menu_last_draw = now;
        _draw_screen();
    }
}

#else  // USE_OLED_MENU == 0

// No-op stubs when menu is disabled
inline void menu_init() {}
inline void menu_tick() {}

#endif  // USE_OLED_MENU
