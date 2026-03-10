#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// tests.h — diagnostic and calibration routines for Umbreon
// Include this file and call functions from setup() or loop() while testing.
// Comment out the #include in the main sketch for competition builds.
// ─────────────────────────────────────────────────────────────────────────────

// ─── LiDAR diagnostics ───────────────────────────────────────────────────────

// Print all 4 LiDAR distances to Serial (cm×10 units).
void print_sensors(Car& car) {
    car.poll_lidars();
    int* s = car.read_sensors();
    const char* labels[4] = {"Left    ", "FrontL  ", "FrontR  ", "Right   "};
    for (int i = 0; i < 4; i++) {
        Serial.print(labels[i]);
        Serial.print(": ");
        if (s[i] == 9999) {
            Serial.println("  --  (no reading)");
        } else {
            Serial.print(s[i] / 10);
            Serial.println(" cm");
        }
    }
    Serial.println("--------------------");
}

// Stream all 4 distances as a tab-separated line (for Serial Plotter).
void plot_sensors(Car& car) {
    car.poll_lidars();
    int* s = car.read_sensors();
    for (int i = 0; i < 4; i++) {
        Serial.print(s[i] == 9999 ? 0 : s[i] / 10);
        if (i < 3) Serial.print('\t');
    }
    Serial.println();
}

// ─── Steering tests ───────────────────────────────────────────────────────────

// Sweep steering left → right → centre.
void wiggle(Car& car) {
    for (int i = -100; i <= 100; i++) {
        car.write_steer(i * 10);
        delay(5);
    }
    for (int i = 100; i >= -100; i--) {
        car.write_steer(i * 10);
        delay(5);
    }
    car.write_steer(0);
}

void turn_left(Car& car)    { car.write_steer(-1000); }
void turn_right(Car& car)   { car.write_steer( 1000); }
void turn_centre(Car& car)  { car.write_steer(    0); }

// ─── Speed / ESC tests ───────────────────────────────────────────────────────

// Ramp ESC from 0 to full forward and back, then full reverse and back.
// WARNING: car will move — have it on a stand or in a clear area.
void max_speed_test(Car& car) {
    delay(1500);
    for (int i = 0; i <= 100; i++) { car.write_speed(i * 10);  delay(20); }
    delay(300);
    for (int i = 100; i >= 0; i--) { car.write_speed(i * 10);  delay(10); }
    delay(1500);
    for (int i = 0; i <= 100; i++) { car.write_speed(-i * 10); delay(10); }
    delay(300);
    for (int i = 100; i >= 0; i--) { car.write_speed(-i * 10); delay(20); }
    car.write_speed(0);
}

// Gentle ramp for ESC range calibration (30 % of max_speed_test).
void small_speed_test(Car& car) {
    delay(1500);
    for (int i = 0; i <= 100; i++) { car.write_speed(i * 3);  delay(20); }
    delay(300);
    for (int i = 100; i >= 0; i--) { car.write_speed(i * 3);  delay(20); }
    delay(1500);
    for (int i = 0; i <= 100; i++) { car.write_speed(-i * 3); delay(20); }
    delay(300);
    for (int i = 100; i >= 0; i--) { car.write_speed(-i * 3); delay(20); }
    car.write_speed(0);
}

// ─── Tachometer / speed tests ────────────────────────────────────────────────

// Stream real-time speed (m/s) to Serial Plotter.
void plot_speed() {
    Serial.println(get_speed(), 3);
}

// Print speed and raw encoder period to Serial.
void print_speed() {
    Serial.print("Speed: ");
    Serial.print(get_speed(), 3);
    Serial.print(" m/s  |  pulse period: ");
    Serial.print(_turnover_time);
    Serial.println(" us");
}

// Drive at a fixed PID target and log actual speed every 100 ms.
// Runs for `duration_ms` milliseconds then stops.
void speed_step_test(Car& car, float target_ms, unsigned long duration_ms) {
    car.write_steer(0);
    car.write_speed_ms(target_ms);
    unsigned long end = millis() + duration_ms;
    while (millis() < end) {
        car.poll_lidars();
        car.pid_control_motor();
        Serial.print("tgt:");  Serial.print(target_ms, 2);
        Serial.print("\tact:"); Serial.println(get_speed(), 3);
        delay(100);
    }
    car.write_speed(0);
    car.write_speed_ms(0);
}
