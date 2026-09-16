# Mecanum Kinematics — Quick Reference

**Project:** OMNIS · 4-wheel holonomic mecanum platform
**Layout:** **lateral parallel rollers** — left pair alike, right pair mirrored
**Scope:** Notation, equations, sign conventions, RC channel mapping, and
verified numbers. No derivation — see
[`mecanum-kinematics-derivation.md`](mecanum-kinematics-derivation.md) for that.

> **Not a textbook X-drive.** Both left wheels carry the same roller tilt and
> both right wheels the mirror, so the standard mecanum matrix does not describe
> this robot. If you arrived here with the usual `±(L+W)/2`-on-all-four formula
> in mind, read §2 carefully — the rear rows differ.

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

### Derived quantities — **two** yaw lever arms, not one

```
k = (wheelbase_mm + track_width_mm) / 2      [mm]   FRONT pair yaw lever
m = (wheelbase_mm - track_width_mm) / 2      [mm]   REAR  pair yaw lever
```

For OMNIS: `k = 226.5 mm`, `m = -3.5 mm`, `k/r = 7.55`, `|k/m| = 64.7`.

The single `k` of the X-drive no longer exists. The front axle carries **98.5%**
of the yaw authority; the rear axle's lever is small, and its **sign flips** if
the frame is ever made longer than it is wide (`m = 0` at `L = W`).

Identities used throughout: `k + m = L`, `k - m = W`.

---

## 2. Inverse kinematics

### Mixing matrix

```
                  ┌                       ┐
 ┌      ┐         │   1     -1     -k     │   ┌      ┐
 │ ω_FL │         │                       │   │  vx  │
 │ ω_FR │    1    │   1     +1     +k     │   │  vy  │
 │ ω_RL │  = ───  │                       │ · │   w  │
 │ ω_RR │    r    │   1     -1     +m     │   └      ┘
 └      ┘         │                       │
                  │   1     +1     -m     │
                  └                       ┘
```

### Written out

```
ω_FL = ( vx - vy - k*w ) / wheel_radius_mm
ω_FR = ( vx + vy + k*w ) / wheel_radius_mm
ω_RL = ( vx - vy + m*w ) / wheel_radius_mm
ω_RR = ( vx + vy - m*w ) / wheel_radius_mm
```

The **front two rows are identical to the X-drive's**; both rear rows differ, in
the `vy` sign and in the lever. That is a fast way to spot a half-finished port.

### To step-pulse rate

```
f_i = ω_i * STEPS_PER_RAD                     STEPS_PER_RAD = 3200/(2π)
```

### Fully combined

```
f_FL = ( vx - vy - k*w ) * 509.295818 / wheel_radius_mm
f_FR = ( vx + vy + k*w ) * 509.295818 / wheel_radius_mm
f_RL = ( vx - vy + m*w ) * 509.295818 / wheel_radius_mm
f_RR = ( vx + vy - m*w ) * 509.295818 / wheel_radius_mm
```

`sign(f_i)` → A4988 `DIR` pin. `|f_i|` → `STEP` pulse rate.

---

## 3. Forward kinematics — commanded-velocity estimate

Open-loop, no encoders: this is what the wheels were **asked** to do, not what
they did.

In terms of the two **axle differentials** — where this form is cleanest:

```
Δ_F = ω_FL - ω_FR                Δ_R = ω_RL - ω_RR

vx = (wheel_radius_mm / 4) * ( ω_FL + ω_FR + ω_RL + ω_RR )

vy = (wheel_radius_mm / (4*wheelbase_mm))
            * ( (track_width_mm - wheelbase_mm) * Δ_F
              - (track_width_mm + wheelbase_mm) * Δ_R )

w  = (wheel_radius_mm / (2*wheelbase_mm)) * ( -Δ_F + Δ_R )
```

> **Yaw is the difference of the two axle differentials; strafe is their
> weighted sum.** The X-drive had it the other way round — yaw from the side
> pattern, strafe from the diagonal. Getting these two backwards is the classic
> port error, and it is silent: forward motion still works and only strafe and
> yaw come out swapped.

Note `track_width_mm` does **not** appear in the `w` row. Yaw resolution is set
by the wheelbase alone, because `det(MᵀM) = 4L²`.

From step rates: `ω_i = f_i / 509.295818`

### Slip / health check

This wheel-speed combination — the **front pair opposing the rear pair** —
produces **no body motion** and should always be zero for a correctly-formed
command:

```
null = ( +ω_FL + ω_FR - ω_RL - ω_RR )
```

**Unchanged from the X-drive.** It annihilates all three motion columns for
either roller handedness and any geometry — the one result the layout change
left completely alone.

Non-zero means something corrupted the command downstream of the IK — a
per-wheel clamp, a saturated axis, or a wrong invert flag. The drivetrain is
fighting itself.

### Precision cost

The `vy` and `w` columns are no longer orthogonal — they meet at **44.1°**
(`cos θ = W/√(L²+W²)`). Reachability is unaffected, but the least-squares
estimate amplifies per-wheel error more: **1.44× on `vy`, 2.03× on `w`** against
the X-drive, for the same steppers. Dead reckoning drifts correspondingly
faster. `vx` is unaffected.

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
   │  FL  [motor] /    |    \ [motor]  FR  │
   │  IMU_A 0x68       |                   │
   │              OLED + buttons           │
+Y │- - - - - - - - - -+- - - - - - - - - -│ -Y
LEFT                   |                  RIGHT
   │                 ESP32-S3              │
   │                   |       IMU_B 0x69  │
   │  RL  [motor] /    |    \ [motor]  RR  │
   └───────────────────┼───────────────────┘
                      REAR
```

Right-handed, +Z out of the page. Identical frame to
[`../control/omnis_imu_mounting.h`](../control/omnis_imu_mounting.h), which
carries the same diagram for the estimation side.

The `/` and `\` marks are the **roller axes** as seen from above: both left
wheels alike, both right wheels the mirror.

| Wheel | Roller `δ` | Tilt | `vx` | `vy` (left+) | `w` (CCW+) |
|---|---|---|---|---|---|
| **FL** | −1 | `/` | +1 | **−1** | **−k** |
| **FR** | +1 | `\` | +1 | **+1** | **+k** |
| **RL** | −1 | `/` | +1 | **−1** | **+m** |
| **RR** | +1 | `\` | +1 | **+1** | **−m** |

**Roller handedness pairing:**
- **FL + RL** (the left side) → `δ = -1`, axis on the forward-right diagonal,
  drawn `/`. Negative on `vy`.
- **FR + RR** (the right side) → `δ = +1`, axis on the forward-left diagonal,
  drawn `\`. Positive on `vy`.

In firmware these four signs are `MECANUM_DELTA_FL` … `MECANUM_DELTA_RR` in
[`mecanum_kinematics.h`](mecanum_kinematics.h) — the **only** place the layout
is stated. The `vy` signs and both lever arms are computed from them.

If the physical wheels are mirrored from this, negate **all four `δ` together**
and rebuild. The `vy` **and** `w` columns then both flip; they are not
independent, so do not flip one by hand, and never "fix" a wrong strafe with a
`DIR` invert flag.

### Motion pattern cross-check

| Motion | Input | FL | FR | RL | RR | Split |
|---|---|---|---|---|---|---|
| Forward | `vx > 0` | + | + | + | + | none — all same |
| Backward | `vx < 0` | − | − | − | − | none — all same |
| Strafe **left** | `vy > 0` | **−** | **+** | **−** | **+** | **sides**, equal magnitude |
| Strafe right | `vy < 0` | + | − | + | − | **sides**, equal magnitude |
| Rotate **CCW** | `w > 0` | **−** | **+** | **−** | **+** | **sides**, front 64.7× rear |
| Rotate CW | `w < 0` | + | − | + | − | **sides**, front 64.7× rear |
| Diagonal fwd-left | `vx = vy > 0` | **0** | + | **0** | + | **left side idles** |
| Diagonal fwd-right | `vx = -vy > 0` | + | **0** | + | **0** | **right side idles** |

Strafe and rotation now share the **same sign pattern** and are told apart only
by the front:rear *magnitude ratio* — 1:1 for strafe, 64.7:1 for yaw. Under an
X-drive they had visibly different patterns; they no longer do.

**Bench test — run BOTH directions.** Command `vx = vy` (diagonal forward-left):
both **left** wheels must be completely stationary. Then command `vx = -vy`:
both **right** wheels must be stationary instead.

- Correct side idles both times → handedness is right.
- The *other* side idles → handedness is mirrored; negate all four `δ`.
- A *diagonal* pair idles → the wheels are not in the parallel layout at all.
- Anything else → a `DIR` invert flag is wrong. Fix forward motion first.

---

## 5. RC channel mapping

Unaffected by the layout change. Fixed input scheme: throttle → `vx`, yaw stick
→ `w`, pitch + roll → `vy` and secondary `vx`.

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
| State the roller layout **only** in `MECANUM_DELTA_*`, and negate all four together | Both `vy` signs and both lever arms are derived from them; changing one without the others is not a valid configuration |
| Clamp with a **common scale factor** on all four wheels | Per-wheel clipping warps the motion vector — a strafe becomes an arc |
| Clamp **before** computing FK | Feeding pre-clamp values into dead reckoning is the classic open-loop drift bug: the estimator believes a velocity the hardware never produced |
| Handle motor mirroring with a per-wheel `dir_invert` flag at the GPIO layer | Flipping a matrix sign to fix a backwards motor breaks the FK round-trip and every other motion using that wheel |
| Deadband small commands — below ≈20 steps/s, hold stopped | A4988 needs `DIR` stable ≥200 ns before a `STEP` edge; a wheel dithering across zero will chatter |
| Expect the **rear pair to deadband during slow yaw** | Their lever is 1.5% of the front's, so below ≈0.34 rad/s they sit below the deadband while the front pair drives. Correct, not a dead motor |
| Keep geometry in `params.json`, passed as arguments | Changing wheel size or frame dimensions must not require touching the function body |
| Unit-test the IK→FK round trip **on each axis separately** | A rank-deficient layout fails only one axis; a combined test can mask it |

---

## 7. Verified numbers — OMNIS build

`wheel_radius_mm = 30`, `wheelbase_mm = 223`, `track_width_mm = 230`
→ `k = 226.5 mm`, `m = -3.5 mm`, `STEPS_PER_RAD = 509.295818`, `k/r = 7.55`

These are the values `test/test_kinematics.c` and the firmware's boot self-check
both assert. If a refactor disagrees with this table, this table is right.

### Case A — pure forward, `vx = 200 mm/s`

| Wheel | ω (rad/s) | f (µsteps/s) |
|---|---|---|
| FL | +6.66667 | +3395.305 |
| FR | +6.66667 | +3395.305 |
| RL | +6.66667 | +3395.305 |
| RR | +6.66667 | +3395.305 |

Unchanged from the X-drive — forward motion never involved handedness.

### Case B — pure strafe left, `vy = 200 mm/s`

| Wheel | ω (rad/s) | f (µsteps/s) |
|---|---|---|
| FL | −6.66667 | −3395.305 |
| FR | +6.66667 | +3395.305 |
| RL | −6.66667 | −3395.305 |
| RR | +6.66667 | +3395.305 |

Left side back, right side forward, all four equal magnitude. Same wheel speed
as forward for the same body speed. It feels weaker because only the roller-axis
force component reaches the floor — expected, not an error.

### Case C — pure rotation CCW, `w = 1.0 rad/s`

| Wheel | ω (rad/s) | f (µsteps/s) |
|---|---|---|
| FL | −7.55000 | −3845.183 |
| FR | +7.55000 | +3845.183 |
| RL | **−0.11667** | **−59.418** |
| RR | **+0.11667** | **+59.418** |

**The signature case.** The rear pair crawls at 1.5% of the front pair. If all
four come out at 3845, the build is still running X-drive kinematics.

Below about **0.34 rad/s** of commanded yaw the rear pair falls under a 20
steps/s deadband and is held stopped while the front pair drives.

### Case D — combined, `vx = 200`, `vy = 100`, `w = 0.5`

`k*w = +113.25 mm/s`, `m*w = -1.75 mm/s`

| Wheel | Arithmetic | ω (rad/s) | f (µsteps/s) | DIR |
|---|---|---|---|---|
| FL | (200 − 100 − 113.25)/30 | −0.441667 | **−224.939** | reverse |
| FR | (200 + 100 + 113.25)/30 | +13.775000 | **+7015.550** | forward |
| RL | (200 − 100 − 1.75)/30 | +3.275000 | **+1667.944** | forward |
| RR | (200 + 100 + 1.75)/30 | +10.058333 | **+5122.667** | forward |

FK round-trip → `(200.0, 100.0, 0.5)` ✓ · null-space check → `0.000` ✓

**Practical limits visible in Case D:**
- FR at 7016 µsteps/s = 2.19 rev/s ≈ **131 RPM**. The A4988 handles that pulse
  rate fine, but a NEMA17 on 12 V is well off its torque curve there. Case D is
  near the practical ceiling of this build.
- FL at −225 µsteps/s sits close to zero and will chatter across the direction
  reversal — this is what the deadband in §6 is for.
- The rear pair's spread (1668…5123) is far narrower than the front pair's
  (−225…7016), because only the front carries the yaw term meaningfully. **The
  front wheels reach the clamp first whenever yaw is commanded.**

### Case E — the handedness bench test

`vx = vy = 200` (diagonal forward-left) → FL and RL exactly **0**, FR and RR at
**+6790.611**.
`vx = 200, vy = -200` (forward-right) → FR and RR exactly **0**, FL and RL at
**+6790.611**.

Run both. See §4.
