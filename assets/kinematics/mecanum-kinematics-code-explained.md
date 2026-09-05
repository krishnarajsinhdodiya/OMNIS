# Mecanum Kinematics — Code Walkthrough

**Project:** OMNIS · 4-wheel holonomic mecanum platform
**Companion files:** [`mecanum_kinematics.c`](mecanum_kinematics.c) ·
[`mecanum_kinematics.h`](mecanum_kinematics.h)

This document reproduces the firmware block by block and explains what each
piece does and why it is written that way. The maths behind it is in
[`mecanum-kinematics-derivation.md`](mecanum-kinematics-derivation.md); the
equations alone are in
[`mecanum-kinematics-reference.md`](mecanum-kinematics-reference.md).

The code compiles clean under `cc -std=c11 -Wall -Wextra` and has been
runtime-verified against the numbers in §7 of the reference document.

---

## 1. Constants and types

```c
#define MOTOR_FULL_STEPS_PER_REV   200.0f   /* 1.8 deg/step NEMA17-class    */
#define A4988_MICROSTEPS            16.0f   /* MS1/MS2/MS3 = HIGH           */
#define MICROSTEPS_PER_REV   (MOTOR_FULL_STEPS_PER_REV * A4988_MICROSTEPS)

#define STEPS_PER_RAD        (MICROSTEPS_PER_REV / (2.0f * (float)M_PI))
```

**Why these are macros and the geometry is not.** These four values describe
things that are physically soldered or jumpered: the motor's step angle and the
A4988's microstep jumpers tied HIGH. They cannot change without a hardware
change, so compile-time is the right place for them. Wheel radius and frame
dimensions *can* change, so they arrive as function arguments instead.

`MICROSTEPS_PER_REV` evaluates to 3200. `STEPS_PER_RAD` evaluates to
**509.295818**, the single constant converting rad/s to microsteps/s. Writing it
as an expression rather than a literal means changing the microstep jumpers is a
one-line edit and the derived constant follows automatically.

The `(float)M_PI` cast matters: `M_PI` is a `double`. Without the cast the whole
expression promotes to double, and on an ESP32-S3 that means software
double-precision arithmetic instead of the hardware single-precision FPU.

```c
typedef struct {
    float fl;   /**< front-left   [microsteps/s] */
    float fr;   /**< front-right  [microsteps/s] */
    float rl;   /**< rear-left    [microsteps/s] */
    float rr;   /**< rear-right   [microsteps/s] */
} wheel_rates_t;

typedef struct {
    float vx;   /**< +forward             [mm/s]  */
    float vy;   /**< +left                [mm/s]  */
    float w;    /**< +counter-clockwise   [rad/s] */
} body_vel_t;
```

Two named structs rather than four loose floats or a `float[4]`. The array form
invites indexing bugs — `wheels[2]` is meaningless at a glance, `rates.rl` is
not. Both structs are 16 and 12 bytes respectively, so returning them by value
is cheap and avoids output-parameter aliasing.

**The rates are signed.** Sign carries the `DIR` pin state; magnitude carries
the `STEP` pulse rate. Keeping them in one signed float means the kinematics
never has to know about GPIO.

---

## 2. Inverse kinematics

```c
wheel_rates_t mecanum_inverse(float vx, float vy, float w,
                              float wheel_radius_mm,
                              float wheelbase_mm,
                              float track_width_mm)
{
```

The three geometry values are **parameters, not globals**. This is the whole
point of the exercise: the caller passes `g_params.wheel_radius_mm` and friends
straight from `params.json`, and resizing the frame never touches this function.

```c
    const float k = 0.5f * (wheelbase_mm + track_width_mm);
```

The **yaw lever arm**. It falls out of the derivation as the coefficient on `w`
for every wheel, differing only in sign.

Worth internalising: it depends on the **sum** of the two frame dimensions.
Neither appears alone, and their ratio is irrelevant. Making the frame 10 mm
longer and 10 mm narrower changes rotation behaviour not at all.

```c
    const float yaw_term = k * w;
```

Computed once and reused four times. Dimensionally this is `[mm] × [rad/s] =
[mm/s]`, which is why it adds directly to `vx` and `vy` on the next lines
without any further conversion. This is also the line that silently breaks if
someone passes `vx`/`vy` in m/s — `yaw_term` would still be in mm/s and the mix
would be wrong by 1000×, but *only when rotating*. Forward motion would still
look perfect, which is what makes that bug so slow to find.

```c
    const float inv_r = 1.0f / wheel_radius_mm;
```

One divide instead of four. Division is markedly slower than multiplication even
with the S3's FPU, and this function runs in the control loop.

```c
    const float w_fl = (vx - vy - yaw_term) * inv_r;   /* [rad/s] */
    const float w_fr = (vx + vy + yaw_term) * inv_r;
    const float w_rl = (vx + vy - yaw_term) * inv_r;
    const float w_rr = (vx - vy + yaw_term) * inv_r;
```

**The mixing matrix, one row per line.** Read the sign columns vertically:

| | `vx` | `vy` | `yaw_term` |
|---|---|---|---|
| FL | + | **−** | **−** |
| FR | + | **+** | **+** |
| RL | + | **+** | **−** |
| RR | + | **−** | **+** |

- The `vy` column splits along **diagonals**: FL/RR negative, FR/RL positive.
  That is the roller handedness — FL and RR share one roller tilt, FR and RL the
  mirrored one.
- The `yaw_term` column splits along **sides**: FL/RL negative, FR/RR positive.
  Left side backward, right side forward, for a counter-clockwise turn.

Those two patterns are different from each other, and that difference is what
makes the platform holonomic. If they were the same, strafe and rotation would
be indistinguishable.

**Do not edit these signs to fix a motor spinning backwards.** The `vy` and `w`
signs both derive from the same roller-handedness term in the derivation, so
they are not independent — flipping one alone produces a matrix that is not the
kinematics of any physical robot. Motor mirroring belongs in §6.

```c
    wheel_rates_t out;
    out.fl = w_fl * STEPS_PER_RAD;
    out.fr = w_fr * STEPS_PER_RAD;
    out.rl = w_rl * STEPS_PER_RAD;
    out.rr = w_rr * STEPS_PER_RAD;
    return out;
}
```

The rad/s → microsteps/s conversion, applied last. Doing it as a separate final
step (rather than folding `STEPS_PER_RAD` into `inv_r`) keeps the intermediate
`w_*` values in real physical units, which makes them meaningful in a debugger
and lets you sanity-check against the derivation directly.

---

## 3. Forward kinematics

```c
body_vel_t mecanum_forward(const wheel_rates_t *f,
                           float wheel_radius_mm,
                           float wheelbase_mm,
                           float track_width_mm)
{
    const float inv_spr = 1.0f / STEPS_PER_RAD;

    const float w_fl = f->fl * inv_spr;   /* [rad/s] */
    const float w_fr = f->fr * inv_spr;
    const float w_rl = f->rl * inv_spr;
    const float w_rr = f->rr * inv_spr;
```

Undo the step conversion first, back to rad/s, so the rest of the function is
pure kinematics in physical units. `inv_spr` is a compile-time-foldable constant
expression, so this costs nothing.

```c
    const float r_over_4 = wheel_radius_mm * 0.25f;

    body_vel_t v;
    v.vx = r_over_4 * ( w_fl + w_fr + w_rl + w_rr);
    v.vy = r_over_4 * (-w_fl + w_fr + w_rl - w_rr);
    v.w  = (wheel_radius_mm / (2.0f * (wheelbase_mm + track_width_mm)))
                   * (-w_fl + w_fr - w_rl + w_rr);
    return v;
}
```

This is the **left pseudoinverse** of the 4×3 mixing matrix — the least-squares
body motion most consistent with all four wheel speeds. It comes out in closed
form with no matrix arithmetic because the matrix columns are mutually
orthogonal, which makes `MᵀM` diagonal.

**Reading it as a check.** Each row here is a *column* of the IK matrix,
transposed:

- `vx` row: `(+,+,+,+)` — the forward column.
- `vy` row: `(−,+,+,−)` — the strafe column, diagonals.
- `w` row: `(−,+,−,+)` — the rotation column, sides.

If you ever suspect a sign error, compare these three patterns against the table
in §2. They must match exactly. That symmetry is not a coincidence; it is what
column orthogonality means.

The `w` row's scale factor is `r / (2·(L+W))` rather than `r/(4k)` — the same
value with the `k` substituted and the fraction cleared, so the function does not
depend on `k` being in scope.

**This is a command, not a measurement.** With no encoders, this reports what
the wheels were *asked* to do. Two systematic errors separate it from reality:
integer rounding at the timer peripheral, and skipped steps under load — the
latter always in the lossy direction, since a stepper can lose steps but never
gain them. Both being systematic rather than random is exactly what lets a
downstream velocity-bias loop absorb them.

---

## 4. Saturation

```c
void mecanum_clamp(wheel_rates_t *r, float max_steps_per_sec)
{
    if (r == NULL || max_steps_per_sec <= 0.0f) {
        return;
    }

    float peak = fabsf(r->fl);
    if (fabsf(r->fr) > peak) peak = fabsf(r->fr);
    if (fabsf(r->rl) > peak) peak = fabsf(r->rl);
    if (fabsf(r->rr) > peak) peak = fabsf(r->rr);

    if (peak > max_steps_per_sec) {
        const float s = max_steps_per_sec / peak;
        r->fl *= s;
        r->fr *= s;
        r->rl *= s;
        r->rr *= s;
    }
}
```

**The single most important thing in this file after the matrix itself.**

The naive implementation clips each wheel independently:

```c
/* WRONG — do not do this */
if (r->fr >  max) r->fr =  max;
if (r->fr < -max) r->fr = -max;
```

That **warps the motion vector**. If FR saturates and the other three do not,
the ratio between the four wheel speeds changes, and by the forward kinematics
that is a different body velocity — a commanded pure strafe becomes an arc, and
the robot curves away from where you pointed it.

Scaling all four by one common factor preserves every ratio, so the direction of
motion is exactly preserved and only the magnitude drops. Verified: clamping the
worked example to 4000 steps/s scales `vx`, `vy` and `w` all by the identical
factor 0.5702.

Guarding on `max_steps_per_sec <= 0.0f` prevents a division that would produce
zero or a negative scale factor; the `peak > max` test means the common case
(nothing saturated) costs four `fabsf` calls and no writes.

---

## 5. Direction-reversal deadband

```c
void mecanum_deadband(wheel_rates_t *r, float deadband_steps_per_sec)
{
    if (r == NULL || deadband_steps_per_sec <= 0.0f) {
        return;
    }

    if (fabsf(r->fl) < deadband_steps_per_sec) r->fl = 0.0f;
    if (fabsf(r->fr) < deadband_steps_per_sec) r->fr = 0.0f;
    if (fabsf(r->rl) < deadband_steps_per_sec) r->rl = 0.0f;
    if (fabsf(r->rr) < deadband_steps_per_sec) r->rr = 0.0f;
}
```

The A4988 requires `DIR` to be stable for at least 200 ns before a `STEP` edge.
A wheel whose command hovers near zero will flip `DIR` back and forth as the
command dithers, producing chatter and no useful motion.

This is not hypothetical for this drivetrain: in the worked example
(`vx=200, vy=100, w=0.5`) the FL wheel comes out at **−225 steps/s** — genuinely
close to zero while the other three run at thousands. A threshold around
20 steps/s holds it stopped instead.

Apply this **after** `mecanum_clamp`, since clamping scales rates down and can
push a wheel into the deadband that was previously above it.

---

## 6. Null-space health check

```c
float mecanum_null_space(const wheel_rates_t *f)
{
    if (f == NULL) {
        return 0.0f;
    }
    return (f->fl + f->fr - f->rl - f->rr);
}
```

The 4×3 matrix maps 3 degrees of freedom onto 4 actuators, so it has a
one-dimensional left null space. Solving `nᵀM = 0` gives `(+1, +1, −1, −1)` —
the **front pair opposing the rear pair**. Wheel speeds in that combination
produce no body motion whatsoever; the energy goes entirely into roller scrub.

Verify it against the three motion patterns: forward `(+,+,+,+)` → `1+1−1−1 = 0`;
strafe `(−,+,+,−)` → `−1+1−1+1 = 0`; rotation `(−,+,−,+)` → `−1+1+1−1 = 0`. All
three body motions are orthogonal to it, as they must be.

Because `mecanum_inverse` can only produce combinations of those three columns,
this function returns ~0 for any well-formed command. A non-zero result means
something **downstream** corrupted the command — per-wheel clipping instead of
common-factor scaling, a wrong `dir_invert` flag, or a stale wheel rate. It is a
cheap assertion to run in a debug build.

Expect small non-zero values from float rounding: the worked example returns
about `−0.0002` on rates of magnitude ~7000, which is normal. Compare against a
tolerance, not against exact zero.

---

## 7. Call site

```c
wheel_rates_t cmd = mecanum_inverse(vx, vy, w,
                                    g_params.wheel_radius_mm,
                                    g_params.wheelbase_mm,
                                    g_params.track_width_mm);

mecanum_clamp(&cmd, g_params.max_steps_per_sec);
mecanum_deadband(&cmd, g_params.deadband_steps_per_sec);

stepper_set_rate(MOTOR_FL, cmd.fl);   /* sign -> DIR, |value| -> STEP */
stepper_set_rate(MOTOR_FR, cmd.fr);
stepper_set_rate(MOTOR_RL, cmd.rl);
stepper_set_rate(MOTOR_RR, cmd.rr);

/* Dead reckoning gets the POST-clamp rates. */
body_vel_t est = mecanum_forward(&cmd,
                                 g_params.wheel_radius_mm,
                                 g_params.wheelbase_mm,
                                 g_params.track_width_mm);
```

Order matters in three places:

1. **Clamp before deadband.** Scaling can move a wheel into the deadband.
2. **Clamp before forward kinematics.** Passing the pre-clamp command into dead
   reckoning is the classic open-loop drift bug — the estimator integrates a
   velocity the hardware never produced, and the error compounds silently for as
   long as you stay saturated.
3. **Motor mirroring lives inside `stepper_set_rate`.** The left-side motors are
   physically mirrored, so realising a positive `ω` needs the opposite `DIR`
   level from the right side. Handle that with a per-wheel `dir_invert` flag in
   the GPIO layer. Putting it in the matrix instead would break the FK
   round-trip and every other motion involving that wheel.

---

## 8. Verification

Test harness output, `wheel_radius_mm=30`, `wheelbase_mm=223`,
`track_width_mm=230`:

```
A pure forward vx=200    FL=  3395.305 FR=  3395.305 RL=  3395.305 RR=  3395.305 | FK=(200.0000, 0.0000, 0.0000) | null=0.000000
B strafe left vy=200     FL= -3395.305 FR=  3395.305 RL=  3395.305 RR= -3395.305 | FK=(0.0000, 200.0000, 0.0000) | null=0.000000
C rotate CCW w=1.0       FL= -3845.183 FR=  3845.183 RL= -3845.183 RR=  3845.183 | FK=(0.0000, 0.0000, 1.0000) | null=0.000000
D combined               FL=  -224.939 FR=  7015.550 RL=  3170.367 RR=  3620.244 | FK=(200.0000, 100.0000, 0.5000) | null=-0.000244

diagonal vx=vy=150       FL=     0.000 FR=  5092.958 RL=  5092.958 RR=     0.000
clamped to 4000 steps/s  FL=  -128.252 FR=  4000.000 RL=  1807.623 RR=  2064.126 | FK=(114.032, 57.016, 0.2851)
  ratios vs unclamped: vx 0.5702  vy 0.5702  w 0.5702
```

Four properties worth keeping as unit tests:

| Test | Property |
|---|---|
| **IK → FK round trip** | Any `(vx, vy, w)` must come back unchanged. Catches every sign error. |
| **Diagonal idles a pair** | `vx = vy` must leave FL and RR at exactly 0. Catches a wrong handedness assignment or `DIR` flag. |
| **Null space is zero** | Any output of `mecanum_inverse` must give ~0. Catches command corruption. |
| **Clamp preserves direction** | `vx`, `vy` and `w` must all scale by the *same* factor. Catches per-wheel clipping creeping back in. |

The diagonal test is the most useful one to run on the physical robot: command
`vx = vy` and confirm FL and RR are genuinely stationary. It is unambiguous by
eye and fails loudly if the roller handedness is mirrored from what the matrix
assumes.

**One hardware limit visible in case D:** FR at 7016 microsteps/s is 2.19 rev/s
≈ 131 RPM. The A4988 emits that pulse rate without difficulty, but a NEMA17 on a
12 V rail is well off its torque curve there, and skipped steps become likely.
Case D sits near the practical ceiling of this build — worth setting
`max_steps_per_sec` with that in mind rather than at the driver's electrical
limit.
