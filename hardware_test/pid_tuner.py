"""
PID tuner — serial telemetry collector & analyzer.

Usage:
  python pid_tuner.py [COM_PORT] [DURATION_S]         — run speed-hold test
  python pid_tuner.py [COM_PORT] --autotune            — run relay auto-tuner

Examples:
  python pid_tuner.py COM15 20
  python pid_tuner.py COM15 --autotune
"""

import serial
import time
import sys
import statistics

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM15"
AUTOTUNE = "--autotune" in sys.argv
DURATION = 20
for a in sys.argv[2:]:
    if a.isdigit():
        DURATION = int(a)
BAUD = 115200


def connect(port):
    print(f"Connecting to {port} @ {BAUD}...")
    ser = serial.Serial(port, BAUD, timeout=0.5)
    time.sleep(2)
    ser.reset_input_buffer()
    return ser


def wait_for_confirm(ser):
    while True:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if line:
            print(f"<< {line}")
        if "Confirm" in line or "cancels" in line:
            break


def collect_data(ser, duration, label=""):
    rows = []
    start = time.time()
    print(f"\nCollecting {label}data for {duration}s...\n")

    while time.time() - start < duration:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode('utf-8', errors='replace').strip()
        if not line:
            continue

        if line.startswith("D,"):
            parts = line[2:].split(",")
            if len(parts) >= 6:
                try:
                    row = {
                        'ms':       int(parts[0]),
                        'target':   float(parts[1]),
                        'raw':      float(parts[2]),
                        'filtered': float(parts[3]),
                        'esc':      int(parts[4]),
                    }
                    # PID mode has more fields
                    if len(parts) >= 9:
                        row['error']    = float(parts[5])
                        row['integral'] = float(parts[6])
                        row['deriv']    = float(parts[7])
                        row['pulses']   = int(parts[8])
                    else:
                        row['relay'] = int(parts[5]) if len(parts) > 5 else 0

                    rows.append(row)
                    if len(rows) % 10 == 0:
                        r = row
                        extra = ""
                        if 'error' in r:
                            extra = f"  err={r['error']:.3f}  I={r['integral']:.2f}"
                        elif 'relay' in r:
                            extra = f"  relay={'HIGH' if r['relay'] else 'LOW'}"
                        print(f"  t={r['ms']/1000:.1f}s  "
                              f"target={r['target']:.2f}  "
                              f"actual={r['filtered']:.2f}  "
                              f"ESC={r['esc']}{extra}")
                except (ValueError, IndexError):
                    pass
        elif line.startswith("R,") and line.count(",") >= 3:
            # Auto-tune result line
            print(f"  >> {line}")
        else:
            print(f"<< {line}")

            # If we see "Phase 2" or "Verifying", return current data and let caller continue
            if "Phase 2" in line or "Verifying" in line:
                return rows, True  # signal: more data coming

            # If we see "done" or "complete", we're finished
            if "done" in line.lower() or "complete" in line.lower():
                return rows, False

    return rows, False


def save_csv(rows, path):
    if not rows:
        return
    with open(path, 'w') as f:
        keys = list(rows[0].keys())
        f.write(",".join(keys) + "\n")
        for r in rows:
            f.write(",".join(str(r.get(k, "")) for k in keys) + "\n")
    print(f"Saved {len(rows)} samples to {path}")


def analyze_pid(rows, target=None):
    if len(rows) < 10:
        print("Not enough data for analysis.")
        return

    # auto-detect target from telemetry
    if target is None:
        target = rows[0].get('target', 0.5)

    speeds = [r['filtered'] for r in rows]
    errors = [r.get('error', target - r['filtered']) for r in rows]
    esc_vals = [r['esc'] for r in rows]

    # settle time
    settle_idx = None
    for i in range(len(speeds)):
        if abs(speeds[i] - target) < target * 0.1:
            ok = True
            for j in range(i, min(i + 10, len(speeds))):
                if abs(speeds[j] - target) > target * 0.15:
                    ok = False
                    break
            if ok:
                settle_idx = i
                break

    total_time = (rows[-1]['ms'] - rows[0]['ms']) / 1000.0

    print(f"\n{'='*55}")
    print(f"  PID ANALYSIS  (target={target:.2f} m/s, {total_time:.1f}s)")
    print(f"{'='*55}")

    if settle_idx is not None:
        settle_time = (rows[settle_idx]['ms'] - rows[0]['ms']) / 1000.0
        print(f"Settle time:     {settle_time:.1f}s")
        ss = speeds[settle_idx:]
        print(f"Steady avg:      {statistics.mean(ss):.3f} m/s")
        print(f"Steady std:      {statistics.stdev(ss):.3f} m/s")
        print(f"Steady min/max:  {min(ss):.3f} / {max(ss):.3f}")
    else:
        print("!! Never settled within 10% of target")
        print(f"Avg speed:       {statistics.mean(speeds):.3f} m/s")

    print(f"Avg ESC:         {statistics.mean(esc_vals):.1f}")
    print(f"ESC range:       {min(esc_vals)} — {max(esc_vals)}")

    # speed drops
    drops = 0
    for i, s in enumerate(speeds):
        if s < target * 0.7 and i > 10:
            drops += 1
    print(f"Samples <70%:    {drops}/{len(speeds)}")
    print(f"{'='*55}")


def run_speed_hold(ser, duration):
    print("Sending 'p' (speed hold)...")
    ser.write(b'p')
    time.sleep(0.3)
    wait_for_confirm(ser)
    print("Sending 'y'...")
    ser.write(b'y')

    rows, _ = collect_data(ser, duration, "speed-hold ")

    print("\nSending stop...")
    ser.write(b'x')
    time.sleep(1)
    while ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if line:
            print(f"<< {line}")

    save_csv(rows, "pid_log.csv")
    analyze_pid(rows)


def run_autotune(ser):
    print("Sending 'u' (auto-tune)...")
    ser.write(b'u')
    time.sleep(0.3)
    wait_for_confirm(ser)
    print("Sending 'y'...")
    ser.write(b'y')

    # Phase 1: relay oscillation (~30s)
    relay_rows, more = collect_data(ser, 45, "relay ")
    save_csv(relay_rows, "autotune_relay.csv")

    if more:
        # Phase 2: verification (~15s)
        verify_rows, _ = collect_data(ser, 20, "verify ")
        save_csv(verify_rows, "autotune_verify.csv")

        if verify_rows:
            analyze_pid(verify_rows)
    else:
        # Read remaining output
        time.sleep(1)
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if line:
                print(f"<< {line}")


def main():
    ser = connect(PORT)
    try:
        if AUTOTUNE:
            run_autotune(ser)
        else:
            run_speed_hold(ser, DURATION)
    finally:
        ser.close()
        print("\nDone.")


if __name__ == "__main__":
    main()
