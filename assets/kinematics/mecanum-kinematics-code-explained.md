# Mecanum Kinematics — Code Walkthrough

**Project:** OMNIS · 4-wheel holonomic mecanum platform
**Layout:** **lateral parallel rollers** — left pair alike, right pair mirrored
**Companion files:** [`mecanum_kinematics.c`](mecanum_kinematics.c) ·
[`mecanum_kinematics.h`](mecanum_kinematics.h)

This document reproduces the firmware block by block and explains what each
piece does and why it is written that way. The maths behind it is in
[`mecanum-kinematics-derivation.md`](mecanum-kinematics-derivation.md); the
equations alone are in
[`mecanum-kinematics-reference.md`](mecanum-kinematics-reference.md).

The code compiles clean under `cc -std=c99 -Wall -Wextra -Werror` and has been
runtime-verified against the numbers in §7 of the reference document. The
transcript in §9 is real output from the shipped source, not a hand-worked
example.

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

### The roller handedness constants — the only statement of the layout

```c
#define MECANUM_DELTA_FL   (-1)
#define MECANUM_DELTA_FR   (+1)
#define MECANUM_DELTA_RL   (-1)
#define MECANUM_DELTA_RR   (+1)

#define MECANUM_DELTA_LEFT  MECANUM_DELTA_FL
```

`δ_i` is the sign distinguishing the two mirrored roller tilts: the roller axis
of wheel *i* projected into the ground plane is `(1, δ_i)/√2` in body
coordinates. Seen from above with the robot facing up the page, `δ = -1` draws
as `/` and `δ = +1` as `\`.

Read down the list: **both left wheels −1, both right wheels +1**. That is the
lateral parallel layout, and it is stated here and nowhere else. Every `vy` sign
and both yaw lever arms in this file are *computed from* these four numbers
rather than written out independently, so the two can never disagree.

**Why integers and not floats.** The assertions below need integer constant
expressions; C only permits floating constants inside `_Static_assert` as the
immediate operand of a cast. Making them `int` and casting at the point of use
keeps the assertions readable.

```c
_Static_assert(MECANUM_DELTA_FL == MECANUM_DELTA_RL, "left pair must share a roller tilt");
_Static_assert(MECANUM_DELTA_FR == MECANUM_DELTA_RR, "right pair must share a roller tilt");
_Static_assert(MECANUM_DELTA_FR == -MECANUM_DELTA_FL, "the two sides must be mirrored");
_Static_assert(MECANUM_DELTA_FL == 1 || MECANUM_DELTA_FL == -1, "handedness is +1 or -1");
```

These are not decoration. `mecanum_inverse()` below is fully general — it would
happily compute an X-drive if you gave it X-drive deltas — but `mecanum_forward()`
uses a **closed-form pseudoinverse derived specifically for the parallel
family**. Setting the deltas back to an X-drive would leave the IK correct and
the FK quietly wrong, which is the worst possible failure: forward motion looks
fine and dead reckoning drifts. The assertions turn that into a compile error
naming the derivation section to read.

```c
typedef struct { float fl, fr, rl, rr; } wheel_rates_t;   /* [microsteps/s] */
typedef struct { float vx, vy, w;      } body_vel_t;
typedef struct { float fl, fr, rl, rr; } mecanum_levers_t; /* [mm] */
```

Named structs rather than four loose floats or a `float[4]`. The array form
invites indexing bugs — `wheels[2]` is meaningless at a glance, `rates.rl` is
not. Returning them by value is cheap and avoids output-parameter aliasing.

**The rates are signed.** Sign carries the `DIR` pin state; magnitude carries
the `STEP` pulse rate. Keeping them in one signed float means the kinematics
never has to know about GPIO.

`mecanum_levers_t` is new in this edition. Under the X-drive there was a single
lever arm `k` and a struct would have been silly; under parallel rollers the
four wheels genuinely have different levers and they are worth naming.

---

## 2. The yaw lever arms

```c
mecanum_levers_t mecanum_yaw_levers(float wheelbase_mm, float track_width_mm)
{
    const float half_l = 0.5f * wheelbase_mm;
    const float half_w = 0.5f * track_width_mm;

    mecanum_levers_t k;
    k.fl = (float)MECANUM_DELTA_FL * ( half_l) - ( half_w);
    k.fr = (float)MECANUM_DELTA_FR * ( half_l) - (-half_w);
    k.rl = (float)MECANUM_DELTA_RL * (-half_l) - ( half_w);
    k.rr = (float)MECANUM_DELTA_RR * (-half_l) - (-half_w);
    return k;
}
```

This is derivation (5.3)'s `κ_i = δ_i·x_i − y_i` transcribed literally, with the
wheel positions `x_i = ±L/2` (front positive) and `y_i = ±W/2` (left positive)
written out in place rather than tabulated. The doubled parentheses and the
visible `-(-half_w)` are deliberate: they keep each line in one-to-one
correspondence with a row of the derivation table, so the two can be checked
against each other by eye.

For the as-built geometry this evaluates to:

```
FL = -226.5    FR = +226.5    RL = -3.5    RR = +3.5   [mm]
```

**The front pair gets `±(L+W)/2` and the rear pair `±(L−W)/2`.** Under an
X-drive all four came out at `±(L+W)/2` and a single `k` sufficed. Here the
front axle's lever is **64.7×** the rear's, so the front pair supplies 98.5% of
the yaw authority and the rear wheels barely turn during a spin.

Two things to know about the rear lever:

- It is **small**, so a slow yaw command will push the rear pair below the
  deadband while the front pair still drives. Correct, not a dead motor.
- Its **sign depends on whether the frame is longer or wider**. `(L−W)/2` is
  −3.5 mm on OMNIS (wider than long) and would be positive on a longer chassis,
  zero on a square one. Nothing breaks at the crossover — §8.2 of the derivation
  shows the matrix stays full rank — but do not memorise which way the rear
  wheels turn during a spin. It is not a robust fact.

The deltas are compile-time constants, so at `-O2` each of these four lines
folds to a single add of two halved parameters. There is no runtime cost to the
generality.

---

## 3. Inverse kinematics

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
    const mecanum_levers_t k = mecanum_yaw_levers(wheelbase_mm, track_width_mm);
    const float inv_r = 1.0f / wheel_radius_mm;
```

One divide instead of four. Division is markedly slower than multiplication even
with the S3's FPU, and this function runs in the 500 Hz control loop.

```c
    const float w_fl = (vx + (float)MECANUM_DELTA_FL * vy + k.fl * w) * inv_r;
    const float w_fr = (vx + (float)MECANUM_DELTA_FR * vy + k.fr * w) * inv_r;
    const float w_rl = (vx + (float)MECANUM_DELTA_RL * vy + k.rl * w) * inv_r;
    const float w_rr = (vx + (float)MECANUM_DELTA_RR * vy + k.rr * w) * inv_r;
```

**One line per wheel, and every line is the same formula.** This is derivation
(5.3) written once:

```
r·ω_i = vx + δ_i·vy + w·κ_i
```

The previous edition wrote the signs out by hand (`vx - vy - yaw_term` and so
on). Writing them as `δ_i` and `κ_i` instead is not stylistic: it makes the four
rows *structurally* identical, so there is no row in which a sign can be wrong
independently of the others. A mirrored build is now four `#define`s, and a
half-applied mirror is not expressible.

Substituting the constants, this expands to the matrix in the reference:

| | `vx` | `vy` | `w` |
|---|---|---|---|
| FL | + | **−** | **−k** |
| FR | + | **+** | **+k** |
| RL | + | **−** | **+m** |
| RR | + | **+** | **−m** |

with `k = (L+W)/2` and `m = (L−W)/2`.

- The `vy` column splits along **sides**: left pair negative, right pair
  positive, all four equal magnitude. Under an X-drive this split along the
  diagonals.
- The `w` column **also** splits along sides — but with the front pair 64.7×
  larger than the rear.

So strafe and yaw now share a sign pattern and are distinguished only by the
front:rear magnitude ratio, 1:1 against 64.7:1. They remain independent
directions — the platform is still fully holonomic, and derivation §8.2 proves
rank 3 for any non-zero wheelbase — but they are no longer *orthogonal* wheel
patterns, and §4 is where that bill arrives.

**Do not edit these signs to fix a motor spinning backwards.** The `vy` and `w`
signs both derive from the same `δ_i`, so they are not independent — flipping
one alone produces a matrix that is not the kinematics of any physical robot.
Motor mirroring belongs in §8.

```c
    wheel_rates_t out;
    out.fl = w_fl * STEPS_PER_RAD;
    /* ... */
    return out;
}
```

The rad/s → microsteps/s conversion, applied last. Doing it as a separate final
step (rather than folding `STEPS_PER_RAD` into `inv_r`) keeps the intermediate
`w_*` values in real physical units, which makes them meaningful in a debugger
and lets you sanity-check against the derivation directly.

---

## 4. Forward kinematics

```c
body_vel_t mecanum_forward(const wheel_rates_t *f, float wheel_radius_mm,
                           float wheelbase_mm, float track_width_mm)
{
    body_vel_t v = { 0.0f, 0.0f, 0.0f };

    if (f == NULL || !(wheelbase_mm > 0.0f)) {
        return v;
    }
```

**Why the wheelbase is guarded and the previous edition's divisor was not.**
The old `w` row divided by `2·(L+W)`, which needed *both* dimensions to be zero
to blow up. The new one divides by `2·L` alone — `det(MᵀM)` works out to `4L²`
— so a single mistyped parameter is enough. `!(x > 0.0f)` rather than `x <= 0.0f`
so that NaN also takes the early return. Failing to a zero velocity estimate is
safe; returning an infinity into a dead-reckoning integrator is not.

```c
    const float inv_spr = 1.0f / STEPS_PER_RAD;
    const float w_fl = f->fl * inv_spr;   /* [rad/s] */
    /* ... */
```

Undo the step conversion first, back to rad/s, so the rest of the function is
pure kinematics in physical units. `inv_spr` is a compile-time-foldable constant
expression, so this costs nothing.

```c
    const float s  = (float)MECANUM_DELTA_LEFT;
    const float d_front = w_fl - w_fr;
    const float d_rear  = w_rl - w_rr;

    v.vx = 0.25f * wheel_radius_mm * (w_fl + w_fr + w_rl + w_rr);

    v.vy = (wheel_radius_mm * s / (4.0f * wheelbase_mm))
         * ((wheelbase_mm + s * track_width_mm) * d_front
          + (wheelbase_mm - s * track_width_mm) * d_rear);

    v.w  = (wheel_radius_mm * s / (2.0f * wheelbase_mm)) * (d_front - d_rear);
```

This is the **left pseudoinverse** `(MᵀM)⁻¹Mᵀ` of the 4×3 mixing matrix — the
least-squares body motion most consistent with all four wheel speeds.

**Why it is no longer a transpose.** Under the X-drive the three matrix columns
were mutually orthogonal, `MᵀM` was diagonal, and the pseudoinverse was the
transpose with a scale factor per row — which is why the old code could be read
as "each row here is a column of the IK matrix". That shortcut is gone. The
strafe and yaw columns now have dot product `2W`, and inverting the resulting
2×2 block mixes them. Anyone porting the old code by copying its row patterns
across will get a function that is exactly right for `vx` and wrong for the
other two.

**Why it is written in axle differentials.** Expanding the algebra gives eight
per-wheel coefficients that are hard to check by eye. Grouping into

```
Δ_F = ω_FL − ω_FR      Δ_R = ω_RL − ω_RR
```

collapses it to something you can state in one sentence:

> **Yaw is the difference of the two axle differentials. Strafe is their
> weighted sum.**

Under the X-drive it was the other way round — yaw came from the side pattern,
strafe from the diagonal. The failure mode of getting them backwards is
particularly nasty because it is *silent on the axis people test first*: forward
motion is unaffected, and only strafe and yaw come out interchanged.

Three details in the code:

- `s` is `MECANUM_DELTA_LEFT`. Carrying it symbolically rather than folding the
  `-1` in means negating the handedness constants flips this function too,
  automatically and in the right direction. It costs one multiply by a
  compile-time constant.
- `track_width_mm` appears in the `vy` row but **not** in the `w` row. Yaw
  resolution is set by the wheelbase alone. A longer robot estimates its own
  rotation better; a wider one does not.
- `(wheelbase_mm - s * track_width_mm)` is `L + W = 453` for the as-built robot
  (since `s = −1`), and `(wheelbase_mm + s * track_width_mm)` is `L − W = −7`.
  The rear differential dominates `vy` by 65:1 — the mirror image of the front
  pair dominating yaw.

**Precision cost.** Reachability is unaffected: any command the IK produced
round-trips exactly. But a least-squares solve over non-orthogonal columns
amplifies per-wheel error more than an orthogonal one would — **1.44× on `vy`
and 2.03× on `w`** against the X-drive, for the same steppers. `vx` is
unaffected. Derivation §9.3 has the propagation.

**This is a command, not a measurement.** With no encoders, this reports what
the wheels were *asked* to do. Two systematic errors separate it from reality:
integer rounding at the timer peripheral, and skipped steps under load — the
latter always in the lossy direction, since a stepper can lose steps but never
gain them. Both being systematic rather than random is exactly what lets a
downstream velocity-bias loop absorb them.

---

## 5. Saturation

```c
void mecanum_clamp(wheel_rates_t *r, float max_steps_per_sec)
{
    if (r == NULL || max_steps_per_sec <= 0.0f) return;

    float peak = fabsf(r->fl);
    if (fabsf(r->fr) > peak) peak = fabsf(r->fr);
    if (fabsf(r->rl) > peak) peak = fabsf(r->rl);
    if (fabsf(r->rr) > peak) peak = fabsf(r->rr);

    if (peak > max_steps_per_sec) {
        const float s = max_steps_per_sec / peak;
        r->fl *= s;  r->fr *= s;  r->rl *= s;  r->rr *= s;
    }
}
```

**The single most important thing in this file after the matrix itself.**
Unchanged by the layout — it is pure vector scaling and knows nothing about
rollers.

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
motion is exactly preserved and only the magnitude drops. Verified in §9:
clamping the worked example to 4000 steps/s scales `vx`, `vy` and `w` all by the
identical factor 0.5702.

**What the layout changed here: which wheel saturates.** The front pair carries
essentially the whole yaw term, so whenever yaw is commanded the front wheels
reach the limit first and the clamp is triggered by one of them. Under an
X-drive both axles saturated together. The clamp itself does not care, but it
means a mixed command spends more of its range on the front motors — worth
knowing when picking `max_step_rate`.

Guarding on `max_steps_per_sec <= 0.0f` prevents a division that would produce
zero or a negative scale factor; the `peak > max` test means the common case
(nothing saturated) costs four `fabsf` calls and no writes.

---

## 6. Direction-reversal deadband

```c
void mecanum_deadband(wheel_rates_t *r, float deadband_steps_per_sec)
{
    if (r == NULL || deadband_steps_per_sec <= 0.0f) return;

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
close to zero while the other three run in the thousands. A threshold around
20 steps/s holds it stopped instead.

**New under this layout: the rear pair deadbands during slow yaw.** Their lever
is 1.5% of the front's, so a pure yaw command below about **0.34 rad/s** puts
both rear wheels under a 20 steps/s threshold while the front pair drives
normally. That is the kinematics being obeyed, not a fault — but it looks
exactly like two dead motors to anyone who has not read this paragraph, which is
why it is also on the bench card in `TESTING.md`.

It is also the concrete reason the null-space metric is taken **before** the
deadband in `drive.c`: the deadband is a per-wheel operation and legitimately
puts energy into the null space, so measuring afterwards would make the health
metric fire on every gentle turn.

Apply this **after** `mecanum_clamp`, since clamping scales rates down and can
push a wheel into the deadband that was previously above it.

---

## 7. Null-space health check

```c
float mecanum_null_space(const wheel_rates_t *f)
{
    if (f == NULL) return 0.0f;
    return (f->fl + f->fr - f->rl - f->rr);
}
```

**This function did not change, and that is a result rather than an oversight.**

The 4×3 matrix maps 3 degrees of freedom onto 4 actuators, so it has a
one-dimensional left null space. Solving `nᵀM = 0` gives `(+1, +1, −1, −1)` —
the **front pair opposing the rear pair**. Wheel speeds in that combination
produce no body motion whatsoever; the energy goes entirely into roller scrub.

Under the X-drive the same vector spanned the null space. The derivation (§9.2)
shows why the coincidence is not one: solving `nᵀM = 0` uses only `k ≠ m`, i.e.
`W ≠ 0`, so **the same direction annihilates all three columns for either
handedness and any geometry**. Of everything in this file, the slip metric is
the one piece the layout change left entirely alone.

Verify it against the three motion patterns: forward `(+,+,+,+)` → `1+1−1−1 = 0`;
strafe `(−,+,−,+)` → `−1+1+1−1 = 0`; rotation `(−k,+k,+m,−m)` → `−k+k−m+m = 0`.
All three body motions are orthogonal to it, as they must be.

Because `mecanum_inverse` can only produce combinations of those three columns,
this function returns ~0 for any well-formed command. A non-zero result means
something **downstream** corrupted the command — per-wheel clipping instead of
common-factor scaling, a wrong `dir_invert` flag, or a stale wheel rate.

Expect small non-zero values from float rounding on large rates. Compare against
a tolerance, not against exact zero.

---

## 8. Call site

```c
wheel_rates_t cmd = mecanum_inverse(vx, vy, w,
                                    g_params.wheel_radius_mm,
                                    g_params.wheelbase_mm,
                                    g_params.track_width_mm);

mecanum_clamp(&cmd, g_params.max_steps_per_sec);
/* null-space health metric is taken HERE: post-clamp, pre-deadband */
mecanum_deadband(&cmd, g_params.deadband_steps_per_sec);

stepper_set_rate(MOTOR_FL, cmd.fl);   /* sign -> DIR, |value| -> STEP */
stepper_set_rate(MOTOR_FR, cmd.fr);
stepper_set_rate(MOTOR_RL, cmd.rl);
stepper_set_rate(MOTOR_RR, cmd.rr);

/* Dead reckoning gets the POST-clamp, POST-deadband rates. */
body_vel_t est = mecanum_forward(&cmd,
                                 g_params.wheel_radius_mm,
                                 g_params.wheelbase_mm,
                                 g_params.track_width_mm);
```

In this firmware that sequence lives in `drive_solve()` (`main/drive.c`) so no
call site has to remember it. Order matters in four places:

1. **Clamp before deadband.** Scaling can move a wheel into the deadband.
2. **Null space between the two.** The clamp preserves it (all four wheels scale
   equally); the deadband does not.
3. **Clamp before forward kinematics.** Passing the pre-clamp command into dead
   reckoning is the classic open-loop drift bug — the estimator integrates a
   velocity the hardware never produced, and the error compounds silently for as
   long as you stay saturated.
4. **Motor mirroring lives inside `stepper_set_rate`.** The left-side motors are
   physically mirrored, so realising a positive `ω` needs the opposite `DIR`
   level from the right side. Handle that with a per-wheel `dir_invert` flag in
   the GPIO layer. Putting it in the matrix instead would break the FK
   round-trip and every other motion involving that wheel.

---

## 9. Verification

Real output from the shipped source, `wheel_radius_mm=30`, `wheelbase_mm=223`,
`track_width_mm=230`:

```
A pure forward vx=200    FL=  3395.305 FR=  3395.305 RL=  3395.305 RR=  3395.305 | FK=(200.0000, -0.0000, -0.0000) | null=0.000000
B strafe left vy=200     FL= -3395.305 FR=  3395.305 RL= -3395.305 RR=  3395.305 | FK=(0.0000, 200.0000, -0.0000) | null=0.000000
C rotate CCW w=1.0       FL= -3845.183 FR=  3845.183 RL=   -59.418 RR=    59.418 | FK=(0.0000, 0.0000, 1.0000) | null=0.000000
D combined               FL=  -224.939 FR=  7015.550 RL=  1667.944 RR=  5122.667 | FK=(200.0000, 100.0000, 0.5000) | null=0.000000

diagonal vx=vy=200       FL=     0.000 FR=  6790.611 RL=     0.000 RR=  6790.611
anti-diag vx=-vy=200     FL=  6790.611 FR=     0.000 RL=  6790.611 RR=     0.000
clamped to 4000 steps/s  FL=  -128.252 FR=  4000.000 RL=   950.998 RR=  2920.750 | FK=(114.032, 57.016, 0.2851)
  ratios vs unclamped: vx 0.5702  vy 0.5702  w 0.5702   (per-wheel 0.5702 0.5702 0.5702 0.5702)

yaw levers: FL=-226.5 FR=226.5 RL=-3.5 RR=3.5 mm   front:rear = 64.71:1
```

Line C is the one to look at. Under an X-drive all four wheels read ±3845; here
the rear pair reads ±59. **If a build shows ±3845 on all four, it is still
running the old kinematics.**

Properties worth keeping as unit tests — all of these are in
`test/test_kinematics.c`, and the firmware re-runs the first three at boot:

| Test | Property |
|---|---|
| **IK → FK round trip, per axis** | `vx`, `vy` and `w` each separately must come back unchanged. Per-axis rather than combined, because a rank-deficient layout fails only one of them. |
| **Side idles, both directions** | `vx = vy` leaves FL and RL at exactly 0; `vx = -vy` leaves FR and RR at 0. Running both is what separates "handedness correct" from "handedness mirrored". |
| **Null space is zero** | Any output of `mecanum_inverse` must give ~0. Catches command corruption. |
| **Clamp preserves direction** | `vx`, `vy` and `w` must all scale by the *same* factor. Catches per-wheel clipping creeping back in. |
| **Lever ratio** | `\|κ_F/κ_R\| = 64.71` on this frame. Catches a partially-applied layout change, which would otherwise pass the round trip. |
| **Column non-orthogonality** | `c_vy · c_w = 2W = 460`. Asserts the layout is what the FK closed form assumes. |

The **side test is the most useful one to run on the physical robot**: command
`vx = vy` and confirm both left wheels are genuinely stationary, then `vx = -vy`
and confirm both right wheels are. Two adjacent wheels standing still while
their neighbours drive is unambiguous across a bench — easier to see than the
X-drive's diagonal pair was — and it fails loudly if the roller handedness is
mirrored from what the matrix assumes.

**One hardware limit visible in case D:** FR at 7016 microsteps/s is 2.19 rev/s
≈ 131 RPM. The A4988 emits that pulse rate without difficulty, but a NEMA17 on a
12 V rail is well off its torque curve there, and skipped steps become likely.
Case D sits near the practical ceiling of this build — worth setting
`max_steps_per_sec` with that in mind rather than at the driver's electrical
limit. The peak wheel is a front wheel under either layout.
