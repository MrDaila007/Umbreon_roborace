#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// track_learn.h — Track profile learning & race mode for RP2350
//
// Qualification lap: records distance-indexed speed/steering profile.
// Race mode: uses learned profile for anticipatory speed control.
//
// Commands (via $TRK:):
//   LEARN  — start recording track profile (car must be running)
//   STOP   — stop learning or racing (back to IDLE)
//   SAVE   — persist track to EEPROM (manual only)
//   LOAD   — load track from EEPROM
//   CLR    — clear track data from RAM
//   RACE   — enable race mode (uses learned profile)
//   STATUS — report mode, point count, distances
//   GET    — dump full track profile as CSV
//
// Memory budget:
//   RAM:    TRK_MAX_POINTS × 4 bytes = 3600 bytes (900 pts)
//   EEPROM: offset 128 + 12 header + 3600 data = 3740 bytes (fits in 4096)
// ─────────────────────────────────────────────────────────────────────────────

#include <EEPROM.h>

// ─── Constants ───────────────────────────────────────────────────────────────
#define TRK_MAGIC           0x4B435254  // "TRCK"
#define TRK_EEPROM_OFFSET   128         // after CarSettings (66 bytes + room)
#define TRK_MAX_POINTS      900         // max track points (~90m at 10cm step)
#define TRK_SAMPLE_DIST_CM  10          // record one point every 10 cm

// ─── Data structures ─────────────────────────────────────────────────────────
struct __attribute__((packed)) TrackPoint {
    uint16_t dist_cm;     // cumulative distance from start (cm), max 655 m
    int8_t   steer;       // steering_cmd / 8, compressed to ±125
    uint8_t  speed_x10;   // target speed × 10 (0–25.5 m/s)
};  // 4 bytes

struct __attribute__((packed)) TrackHeader {
    uint32_t magic;        // TRK_MAGIC
    uint16_t count;        // number of recorded points
    uint16_t lap_dist_cm;  // total lap distance (cm)
    uint8_t  checksum;
    uint8_t  _pad[3];      // pad to 12 bytes
};  // 12 bytes

enum TrackMode : uint8_t {
    TRK_IDLE  = 0,
    TRK_LEARN = 1,
    TRK_RACE  = 2,
};

// ─── State ───────────────────────────────────────────────────────────────────
static TrackPoint  trk_points[TRK_MAX_POINTS];
static TrackHeader trk_hdr = {};
static TrackMode   trk_mode = TRK_IDLE;

static float         trk_odo_m = 0.0f;          // current odometer (meters)
static float         trk_last_sample_m = 0.0f;  // distance at last recorded point
static unsigned long trk_taho_start = 0;         // encoder count at learn/race start

// ─── External references (defined in luna_car.h / .ino) ─────────────────────
extern volatile unsigned long _taho_count;
extern int   cfg_encoder_holes;
extern float cfg_wheel_diam_m;

// ─── Odometry from encoder ──────────────────────────────────────────────────
static float trk_get_odo_m() {
    unsigned long counts = _taho_count - trk_taho_start;
    if (cfg_encoder_holes <= 0) return 0.0f;
    return (counts * 3.14159265f * cfg_wheel_diam_m) / (float)cfg_encoder_holes;
}

// ─── Learning ────────────────────────────────────────────────────────────────
static void trk_start_learn() {
    trk_mode = TRK_LEARN;
    trk_hdr.count = 0;
    trk_hdr.lap_dist_cm = 0;
    trk_taho_start = _taho_count;
    trk_odo_m = 0.0f;
    trk_last_sample_m = 0.0f;
}

static void trk_stop_learn() {
    if (trk_mode != TRK_LEARN) return;
    trk_hdr.magic = TRK_MAGIC;
    trk_hdr.lap_dist_cm = (uint16_t)(trk_get_odo_m() * 100);
    trk_mode = TRK_IDLE;
}

// Call from work() every control tick — records a point every TRK_SAMPLE_DIST_CM
static void trk_learn_tick(int steer_cmd, float target_speed) {
    if (trk_mode != TRK_LEARN) return;

    trk_odo_m = trk_get_odo_m();
    const float sample_m = TRK_SAMPLE_DIST_CM / 100.0f;

    if ((trk_odo_m - trk_last_sample_m) >= sample_m && trk_hdr.count < TRK_MAX_POINTS) {
        TrackPoint& pt = trk_points[trk_hdr.count];
        pt.dist_cm   = (uint16_t)(trk_odo_m * 100);
        pt.steer     = (int8_t)constrain(steer_cmd / 8, -125, 125);
        pt.speed_x10 = (uint8_t)constrain((int)(target_speed * 10), 0, 255);
        trk_hdr.count++;
        trk_last_sample_m = trk_odo_m;
    }
}

// ─── Race mode ───────────────────────────────────────────────────────────────
static void trk_start_race() {
    if (trk_hdr.count < 2) return;
    trk_mode = TRK_RACE;
    trk_taho_start = _taho_count;
    trk_odo_m = 0.0f;
}

static void trk_stop_race() {
    if (trk_mode == TRK_RACE) trk_mode = TRK_IDLE;
}

// Returns recommended speed at (current position + lookahead).
// Takes the minimum learned speed in a ±3 point window for safety.
// Returns -1.0 if no recommendation (no data or not in race mode).
static float trk_recommend_speed(float lookahead_m) {
    if (trk_mode != TRK_RACE || trk_hdr.count < 2) return -1.0f;

    trk_odo_m = trk_get_odo_m();
    float lap_m = trk_hdr.lap_dist_cm / 100.0f;
    if (lap_m <= 0.0f) return -1.0f;

    // Wrap around lap for continuous racing
    float query_m = fmodf(trk_odo_m + lookahead_m, lap_m);
    uint16_t query_cm = (uint16_t)(query_m * 100);

    // Binary search for nearest point
    int lo = 0, hi = (int)trk_hdr.count - 1;
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (trk_points[mid].dist_cm < query_cm) lo = mid + 1;
        else hi = mid;
    }

    // Minimum speed in ±3 point window (conservative — anticipates worst case)
    float min_spd = 99.0f;
    for (int i = max(0, lo - 3); i <= min((int)trk_hdr.count - 1, lo + 3); i++) {
        float s = trk_points[i].speed_x10 / 10.0f;
        if (s > 0.01f && s < min_spd) min_spd = s;
    }

    return (min_spd < 99.0f) ? min_spd : -1.0f;
}

// ─── Clear ───────────────────────────────────────────────────────────────────
static void trk_clear() {
    trk_mode = TRK_IDLE;
    memset(&trk_hdr, 0, sizeof(trk_hdr));
}

// ─── EEPROM persistence (manual save/load only) ─────────────────────────────
static uint8_t trk_checksum() {
    uint8_t sum = 0;
    const uint8_t* p = (const uint8_t*)&trk_hdr;
    // Sum first 8 bytes of header (magic + count + lap_dist)
    for (size_t i = 0; i < 8; i++) sum += p[i];
    // Sum all point data
    for (uint16_t i = 0; i < trk_hdr.count; i++) {
        const uint8_t* pp = (const uint8_t*)&trk_points[i];
        for (size_t j = 0; j < sizeof(TrackPoint); j++) sum += pp[j];
    }
    return sum;
}

static bool trk_save() {
    if (trk_hdr.count == 0) return false;
    trk_hdr.magic = TRK_MAGIC;
    trk_hdr.checksum = trk_checksum();

    int addr = TRK_EEPROM_OFFSET;
    EEPROM.put(addr, trk_hdr);
    addr += sizeof(TrackHeader);
    for (uint16_t i = 0; i < trk_hdr.count; i++) {
        EEPROM.put(addr, trk_points[i]);
        addr += sizeof(TrackPoint);
    }
    EEPROM.commit();
    return true;
}

static bool trk_load() {
    TrackHeader h;
    EEPROM.get(TRK_EEPROM_OFFSET, h);
    if (h.magic != TRK_MAGIC || h.count == 0 || h.count > TRK_MAX_POINTS)
        return false;

    // Read points into buffer
    int addr = TRK_EEPROM_OFFSET + sizeof(TrackHeader);
    for (uint16_t i = 0; i < h.count; i++) {
        EEPROM.get(addr, trk_points[i]);
        addr += sizeof(TrackPoint);
    }

    trk_hdr = h;
    // Verify checksum
    if (trk_checksum() != trk_hdr.checksum) {
        memset(&trk_hdr, 0, sizeof(trk_hdr));
        return false;
    }
    return true;
}

// ─── Serial output ──────────────────────────────────────────────────────────
static void trk_send_status(Stream& out) {
    out.print("$TRK:STS,mode=");
    out.print(trk_mode == TRK_LEARN ? "LEARN" : trk_mode == TRK_RACE ? "RACE" : "IDLE");
    out.print(",pts=");
    out.print(trk_hdr.count);
    out.print(",dist=");
    out.print(trk_hdr.lap_dist_cm);
    out.print(",max=");
    out.print((int)TRK_MAX_POINTS);
    if (trk_mode != TRK_IDLE) {
        out.print(",odo=");
        out.print((int)(trk_odo_m * 100));
    }
    out.println();
}

// Dump full track profile as CSV: $TRK:D,index,dist_cm,steer,speed_x10
static void trk_send_data(Stream& out) {
    out.print("$TRK:HDR,pts=");
    out.print(trk_hdr.count);
    out.print(",lap=");
    out.println(trk_hdr.lap_dist_cm);
    for (uint16_t i = 0; i < trk_hdr.count; i++) {
        out.print("$TRK:D,");
        out.print(i);            out.print(',');
        out.print(trk_points[i].dist_cm);  out.print(',');
        out.print(trk_points[i].steer);    out.print(',');
        out.println(trk_points[i].speed_x10);
    }
    out.println("$TRK:DONE");
}

// ─── Command dispatcher (called from main dispatch_command) ─────────────────
static void trk_dispatch(const char* sub, Stream& out) {
    if      (strcmp(sub, "LEARN")  == 0) { trk_start_learn(); out.println("$ACK"); trk_send_status(out); }
    else if (strcmp(sub, "STOP")   == 0) { trk_stop_learn(); trk_stop_race(); out.println("$ACK"); trk_send_status(out); }
    else if (strcmp(sub, "SAVE")   == 0) {
        if (trk_save()) { out.println("$ACK"); trk_send_status(out); }
        else out.println("$NAK:no_data");
    }
    else if (strcmp(sub, "LOAD")   == 0) {
        if (trk_load()) { out.println("$ACK"); trk_send_status(out); }
        else out.println("$NAK:no_track");
    }
    else if (strcmp(sub, "CLR")    == 0) { trk_clear(); out.println("$ACK"); trk_send_status(out); }
    else if (strcmp(sub, "RACE")   == 0) {
        if (trk_hdr.count >= 2) { trk_start_race(); out.println("$ACK"); trk_send_status(out); }
        else out.println("$NAK:no_track");
    }
    else if (strcmp(sub, "STATUS") == 0) trk_send_status(out);
    else if (strcmp(sub, "GET")    == 0) trk_send_data(out);
    else out.println("$NAK:unknown_trk_cmd");
}
