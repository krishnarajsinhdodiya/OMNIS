# Mecanum Kinematics — Quick Reference

**Project:** OMNIS · 4-wheel holonomic mecanum platform
**Scope:** Notation, equations, sign conventions, RC channel mapping, and
verified numbers. No derivation — see
[`mecanum-kinematics-derivation.md`](mecanum-kinematics-derivation.md) for that.

---

## 1. Notation

### Body-frame quantities

Right-handed frame, ROS REP-103 convention.

| Symbol | Quantity | Positive direction | Unit |
|---|---|---|---|
| `vx` | longitudinal velocity | **forward** | mm/s |
| `vy` | lateral velocity (strafe) | **left** | mm/s |
| `w` | yaw rate | **counter-clockwise** from above | rad/s |
| `ω_i` | wheel angular velocity | sense that drives robot **forward** | rad/s |
| `f_i` | step-pulse rate | sign follows `ω_i` | microsteps/s |

**Unit rule:** geometry is in mm, so `vx`/`vy` must be in **mm/s**. Yaw is in
**rad/s**. Mixing in m/s breaks the yaw term by 1000× while forward motion still
looks correct — normalise once at the RC input layer.

### Geometry parameters — from `params.json`

| Name | Meaning | OMNIS value |
|---|---|---|
| `wheel_radius_mm` | wheel radius | 30 |
| `wheelbase_mm` | front-to-back wheel-centre distance | 223 |
| `track_width_mm` | left-to-right wheel-centre distance | 230 |

### Fixed drivetrain constants — hard-wired, not parameters

| Name | Value | Source |
|---|---|---|
| `MOTOR_FULL_STEPS_PER_REV` | 200 | 1.8°/step NEMA17 |
| `A4988_MICROSTEPS` | 16 | MS1/MS2/MS3 tied HIGH |
| `MICROSTEPS_PER_REV` | 3200 | 200 × 16 |
| `STEPS_PER_RAD` | 509.295818 | 3200 / (2π) |

### Derived quantity

```
k = (wheelbase_mm + track_width_mm) / 2        [mm]     "yaw lever arm"
```

Depends on the **sum** of the frame dimensions, not either alone.
For OMNIS: `k = 226.5 mm`, and `k / wheel_radius_mm = 7.55`.

---

## 2. Inverse kinematics

### Mixing matrix

```
                  ┌                       ┐
 ┌      ┐         │   1     -1     -k     │   ┌      ┐
 │ ω_FL │         │                       │   │  vx  │
 │ ω_FR │    1    │   1     +1     +k     │   │  vy  │
 │ ω_RL │  = ───  │                       │ · │   w  │
 │ ω_RR │    r    │   1     +1     -k     │   └      ┘
 └      ┘         │                       │
                  │   1     -1     +k     │
                  └                       ┘

        r = wheel_radius_mm
        k = (wheelbase_mm + track_width_mm) / 2
```

### Written out

```
ω_FL = ( vx - vy - k*w ) / wheel_radius_mm
ω_FR = ( vx + vy + k*w ) / wheel_radius_mm
ω_RL = ( vx + vy - k*w ) / wheel_radius_mm
ω_RR = ( vx - vy + k*w ) / wheel_radius_mm
```

### To step-pulse rate

```
f_i = ω_i * STEPS_PER_RAD                     STEPS_PER_RAD = 3200/(2π)
```

### Fully combined

```
f_FL = ( vx - vy - k*w ) * 509.295818 / wheel_radius_mm
f_FR = ( vx + vy + k*w ) * 509.295818 / wheel_radius_mm
f_RL = ( vx + vy - k*w ) * 509.295818 / wheel_radius_mm
f_RR = ( vx - vy + k*w ) * 509.295818 / wheel_radius_mm
```

`sign(f_i)` → A4988 `DIR` pin. `|f_i|` → `STEP` pulse rate.

---

## 3. Forward kinematics — commanded-velocity estimate

Open-loop, no encoders: this is what the wheels were **asked** to do, not what
they did.

```
vx = (wheel_radius_mm / 4) * (  ω_FL + ω_FR + ω_RL + ω_RR )

vy = (wheel_radius_mm / 4) * ( -ω_FL + ω_FR + ω_RL - ω_RR )

w  = (wheel_radius_mm / (2*(wheelbase_mm + track_width_mm)))
                       * ( -ω_FL + ω_FR - ω_RL + ω_RR )
```

From step rates: `ω_i = f_i / 509.295818`

### Slip / health check

This wheel-speed combination — the **front pair opposing the rear pair** —
produces **no body motion** and should always be zero for a correctly-formed
command:

```
null = ( +ω_FL + ω_FR - ω_RL - ω_RR )
```

Non-zero means something corrupted the command downstream of the IK — a
per-wheel clamp, a saturated axis, or a wrong invert flag. The drivetrain is
fighting itself.

---

## 4. Sign convention per wheel

### Which physical corner is which — as-built

Confirmed against the assembled board, 2026-09-05. **Front is the OLED / button
end.** Everything below is stated in these labels, so getting this wrong inverts
the meaning of the whole table.

```
                     +X  FRONT
                       ^
                       |
   ┌───────────────────┼───────────────────┐
   │  FL  [motor]      |      [motor]  FR  │
   │  IMU_A 0x68       |                   │
   │              OLED + buttons           │
+Y │- - - - - - - - - -+- - - - - - - - - -│ -Y
LEFT                   |                  RIGHT
   │                 ESP32-S3              │
   │                   |       IMU_B 0x69  │
   │  RL  [motor]      |      [motor]  RR  │
   └───────────────────┼───────────────────┘
                      REAR
```

Right-handed, +Z out of the page. Identical frame to
[`../control/omnis_imu_mounting.h`](../control/omnis_imu_mounting.h), which
carries the same diagram for the estimation side.

The two MPU6050s sit on the **FL/RR diagonal** — which is also the `δ = -1`
roller pair below. Coincidence of layout; nothing depends on it.


| Wheel | Roller `δ` | `vx` | `vy` (left+) | `w` (CCW+) |
|---|---|---|---|---|
| **FL** | −1 | +1 | **−1** | **−k** |
| **FR** | +1 | +1 | **+1** | **+k** |
| **RL** | +1 | +1 | **+1** | **−k** |
| **RR** | −1 | +1 | **−1** | **+k** |

**Roller handedness pairing:**
- **FL + RR** → `δ = -1`, roller axis along the forward-right diagonal. Negative on `vy`.
- **FR + RL** → `δ = +1`, roller axis along the forward-left diagonal. Positive on `vy`.

If the physical wheels are mirrored from this, swap the two `δ` values — the
`vy` **and** `w` columns then both flip. They are not independent; do not flip
one by hand.

### Motion pattern cross-check

| Motion | Input | FL | FR | RL | RR | Split |
|---|---|---|---|---|---|---|
| Forward | `vx > 0` | + | + | + | + | none — all same |
| Backward | `vx < 0` | − | − | − | − | none — all same |
| Strafe **left** | `vy > 0` | **−** | **+** | **+** | **−** | **diagonals** |
| Strafe right | `vy < 0` | + | − | − | + | **diagonals** |
| Rotate **CCW** | `w > 0` | **−** | **+** | **−** | **+** | **sides** |
| Rotate CW | `w < 0` | + | − | + | − | **sides** |
| Diagonal fwd-left | `vx = vy > 0` | **0** | + | + | **0** | one diagonal idles |

Strafe splits along **diagonals**; rotation splits along **sides**. These match
the motion diagrams in the Wikipedia "Mecanum wheel" article and together fix
the matrix uniquely.

**Bench test:** command `vx = vy` (diagonal forward-left). FL and RR must be
completely stationary. If they creep, a handedness assignment or a `DIR` invert
flag is wrong.

---

## 5. RC channel mapping

Fixed input scheme: throttle → `vx`, yaw stick → `w`, pitch + roll → `vy` and
secondary `vx`.

```
vx = throttle_norm * VX_MAX_MMPS  +  pitch_norm * VX_SECONDARY_MMPS
vy = roll_norm     * VY_MAX_MMPS
w  = yaw_norm      * W_MAX_RADPS
```

Each `*_norm` is in the range [−1, +1].

| Channel | Drives | Positive stick | Scale constant | Unit |
|---|---|---|---|---|
| Throttle | `vx` (primary) | forward | `VX_MAX_MMPS` | mm/s |
| Pitch | `vx` (secondary) | forward | `VX_SECONDARY_MMPS` | mm/s |
| Roll | `vy` | **left** | `VY_MAX_MMPS` | mm/s |
| Yaw | `w` | **left / CCW** | `W_MAX_RADPS` | rad/s |

**Stick polarity:** yaw stick left → `w` positive (CCW). Roll stick left → `vy`
positive. If your transmitter is right-positive on either axis, negate **at this
mapping layer only**. Never negate inside the kinematics.

**Combined `vx`:** throttle and pitch both feed `vx` additively, so the sum can
exceed `VX_MAX_MMPS`. Clamp the combined `vx` before it reaches the IK, or rely
on the common-factor wheel clamp in §6 — but not both, or the response becomes
non-linear near full stick.

---

## 6. Implementation checklist

| Rule | Why |
|---|---|
| Clamp with a **common scale factor** on all four wheels | Per-wheel clipping warps the motion vector — a strafe becomes an arc |
| Clamp **before** computing FK | Feeding pre-clamp values into dead reckoning is the classic open-loop drift bug: the estimator believes a velocity the hardware never produced |
| Handle motor mirroring with a per-wheel `dir_invert` flag at the GPIO layer | Flipping a matrix sign to fix a backwards motor breaks the FK round-trip and every other motion using that wheel |
| Deadband small commands — below ≈20 steps/s, hold stopped | A4988 needs `DIR` stable ≥200 ns before a `STEP` edge; a wheel dithering across zero will chatter |
| Keep geometry in `params.json`, passed as arguments | Changing wheel size or frame dimensions must not require touching the function body |
| Unit-test the IK→FK round trip | Any sign error shows up immediately as a failed round-trip |

---

## 7. Verified numbers — OMNIS build

`wheel_radius_mm = 30`, `wheelbase_mm = 223`, `track_width_mm = 230`
→ `k = 226.5 mm`, `STEPS_PER_RAD = 509.295818`, `k/r = 7.55`

### Case A — pure forward, `vx = 200 mm/s`

| Wheel | ω (rad/s) | f (µsteps/s) |
|---|---|---|
| FL | +6.66667 | +3395.305 |
| FR | +6.66667 | +3395.305 |
| RL | +6.66667 | +3395.305 |
| RR | +6.66667 | +3395.305 |

### Case B — pure strafe left, `vy = 200 mm/s`

| Wheel | ω (rad/s) | f (µsteps/s) |
|---|---|---|
| FL | −6.66667 | −3395.305 |
| FR | +6.66667 | +3395.305 |
| RL | +6.66667 | +3395.305 |
| RR | −6.66667 | −3395.305 |

Same wheel speed as forward for the same body speed. It feels weaker because
only the roller-axis force component reaches the floor — expected, not an error.

### Case C — pure rotation CCW, `w = 1.0 rad/s`

| Wheel | ω (rad/s) | f (µsteps/s) |
|---|---|---|
| FL | −7.55000 | −3845.183 |
| FR | +7.55000 | +3845.183 |
| RL | −7.55000 | −3845.183 |
| RR | +7.55000 | +3845.183 |

### Case D — combined, `vx = 200`, `vy = 100`, `w = 0.5`

`k*w = 113.25 mm/s`

| Wheel | Arithmetic | ω (rad/s) | f (µsteps/s) | DIR |
|---|---|---|---|---|
| FL | (200 − 100 − 113.25)/30 | −0.441667 | **−224.939** | reverse |
| FR | (200 + 100 + 113.25)/30 | +13.775000 | **+7015.550** | forward |
| RL | (200 + 100 − 113.25)/30 | +6.225000 | **+3170.366** | forward |
| RR | (200 − 100 + 113.25)/30 | +7.108333 | **+3620.244** | forward |

FK round-trip → `(200.0, 100.0, 0.5)` ✓ · null-space check → `0.000` ✓

**Practical limits visible in Case D:**
- FR at 7016 µsteps/s = 2.19 rev/s ≈ **131 RPM**. The A4988 handles that pulse
  rate fine, but a NEMA17 on 12 V is well off its torque curve there. Case D is
  near the practical ceiling of this build.
- FL at −225 µsteps/s sits close to zero and will chatter across the direction
  reversal — this is what the deadband in §6 is for.
