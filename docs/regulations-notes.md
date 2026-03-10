# Roborace Regulations — Key Points for Umbreon

Source: `Roborace-Regulations-2025-11-12.pdf`

---

## Track Constraints

| Parameter | Value |
|---|---|
| Track width (straight) | 950 – 1300 mm |
| Track width (corner) | 950 – 1750 mm |
| Wall height | 100 – 200 mm |
| Surface | Recommended black |
| Max threshold / gap | 10 mm |
| Overpass incline | ≤ 20° |

**Implication for algorithm:** track is 95 – 175 cm wide. Our `SIDE_OPEN_DIST = 1000` (100 cm) and `FRONT_OBSTACLE_DIST = 1200` (120 cm) were chosen for this range.

---

## Robot Dimensions (PRO Mini category)

| Parameter | Limit |
|---|---|
| Width | 150 mm |
| Length | 200 mm |
| Height | ≤ 200 mm |
| Mass | ≤ 1 kg |

**Required:** soft bumper (foam/rubber, cross-section ≥ 1 cm²) along entire front.

After start, robot footprint must stay within inspection frame dimensions — no wing deployments.

---

## Admission Speed Requirement

Minimum qualifying speed: **50 cm/s = 0.5 m/s** along the centre line.

Our cornering target: 0.8 m/s → safely above limit.

---

## Race Format (ERC)

| Race | Duration |
|---|---|
| Elimination | 5 minutes |
| Final | 10 minutes |

Goal: maximum laps in time. Direction set by chief judge (recommended clockwise).

---

## Obstacles (Qualification Only)

- Size: 15 × 20 cm (± 1 cm), height ≥ 10 cm
- Penalty: +10 s per obstacle touched
- Arrangement same for all participants

**Our `FRONT_OBSTACLE_DIST = 1200` (120 cm) will trigger steering before hitting the 15×20 cm blocks.**

---

## Pit Stop Rules

- Unlimited pit stops per race
- Each stop: 1-minute mandatory pause (program may be updated)
- Re-enter within 1 m of start line, no robots within 3 m

---

## Penalties Relevant to Autonomous Robots

- Touching same 1-metre track section ≥ 3 times per lap → forced pit stop
- This means stuck-reversal must clear the section, not loop in place
- "Traffic jam" (10 s tangled with another robot) → all tangled robots pit

---

## Wall Marker Convention

| Colour | Meaning |
|---|---|
| Blue / Yellow | Upcoming turn |
| Red / Green | Direction of travel |

(Only relevant if camera-based navigation is added in future.)
