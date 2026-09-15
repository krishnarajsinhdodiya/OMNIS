# Attitude EKF — Derivation

**Project:** OMNIS · dual-MPU6050 attitude estimation
**Scope:** Why this filter has the form it does. For the equations as
implemented, see [`attitude-ekf-reference.md`](attitude-ekf-reference.md).
**Implements:** `omnis-info.md` §11

---

## 1. The problem

Two sensors, both individually useless for attitude:

- **Gyroscope** measures angular *rate*. Integrating gives angle — but any
  constant offset in the rate integrates into a linearly growing angle error.
  An MPU6050 with a 0.5 °/s bias drifts 30° in a minute. Excellent short-term,
  worthless long-term.
- **Accelerometer** measures specific force. At rest that is purely gravity, so
  it gives an absolute tilt reference with no drift — but it also picks up every
  linear acceleration and every motor vibration. Excellent long-term, terrible
  short-term.

The errors are in *complementary* frequency bands. Any sensible fusion is a
high-pass on the gyro and a low-pass on the accelerometer. A fixed-coefficient
complementary filter does exactly that; a Kalman filter does it too, but derives
the crossover from stated noise levels and — crucially — carries a **gyro bias
state** so the drift is estimated and removed rather than merely filtered.

That bias state is the entire reason to prefer a Kalman filter here.

---

## 2. Why not a full quaternion AHRS

The usual reasons to go quaternion are yaw estimation and singularity-free
attitude. Neither applies:

- §11a: the balance controller does not use yaw. Left to drift.
- The singularity (§7 below) is real but sits at pitch ±90°, where the *pitch*
  estimate — the one balance mode actually uses — remains perfectly well-defined.

A quaternion AHRS would cost a 7-state filter, quaternion normalisation every
tick, and a magnetometer the BOM does not have, to estimate a state nothing
reads. The 4-state Euler filter is the right size for the job.

---

## 3. State and process model

```
x = [ roll, pitch, bias_roll, bias_pitch ]^T
```

The gyro is treated as an *input*, not a measurement — standard for
inertial filters. The measurement is then the accelerometer alone.

```
roll_dot   = gyro_x - bias_roll
pitch_dot  = gyro_y - bias_pitch
bias_dot   = 0  + w_bias        (random walk)
```

Modelling bias as a random walk rather than a constant is what lets the filter
track thermal drift. `q_bias` sets how fast it is willing to believe the bias
changed: too small and it cannot follow warm-up drift, too large and it starts
absorbing genuine tilt into the bias state.

Discretising with a first-order hold over `dt`:

```
roll_k  = roll_{k-1} + (gyro_x - bias_roll)*dt
```

The Jacobian `F = ∂f/∂x`:

```
      ┌                    ┐
      │ 1   0  -dt   0     │
F  =  │ 0   1   0   -dt    │
      │ 0   0   1    0     │
      │ 0   0   0    1     │
      └                    ┘
```

Note this is **constant** — the process model is linear. The `-dt` entries are
the whole mechanism: they say "an error in my bias estimate becomes an error in
my angle, at a rate of `dt` per step."

---

## 4. Why 4 states decompose exactly into 2 + 2

Look at the structure. `F` is block-diagonal:

```
       ┌ A  0 ┐                    ┌ 1  -dt ┐
F  =   │      │       where   A  = │        │
       └ 0  A ┘                    └ 0   1  ┘
```

with the state ordered `[roll, bias_roll | pitch, bias_pitch]`. The process
noise `Q` is diagonal, so also block-diagonal. The measurement matrix

```
H = [ 1 0 0 0 ]      (roll from accel)
    [ 0 0 1 0 ]      (pitch from accel)
```

picks one state from each block, and `R` is diagonal.

Kalman propagation `P' = F P F^T + Q` preserves block-diagonal structure when
`F` and `Q` are block-diagonal. The update `P -= K H P` preserves it when `H`
and `R` are too. So **if `P` starts block-diagonal it stays block-diagonal
forever**, and the off-diagonal blocks are permanently zero.

Storing and multiplying those zeros is pure waste. Two 2-state filters give
bit-identical results for 4 floats of covariance each and no matrix inverse.

This is an *identity*, not an approximation. It stops holding the moment you
add a cross-axis term — full Euler rate coupling (§8) would do it — at which
point you genuinely need the 4×4.

---

## 5. The measurement model, and why it is "extended"

With the platform static, the accelerometer reads the gravity vector rotated
into the body frame. For roll `φ` then pitch `θ`:

```
a_x = -sin(θ)
a_y =  sin(φ) cos(θ)
a_z =  cos(φ) cos(θ)
```

Inverting:

```
φ = atan2( a_y, a_z )
θ = atan2( -a_x, sqrt(a_y^2 + a_z^2) )
```

Check the first: `atan2(sinφ cosθ, cosφ cosθ) = atan2(sinφ, cosφ) = φ`, valid
wherever `cosθ > 0`. The second: `sqrt(a_y²+a_z²) = |cosθ|`, so
`atan2(sinθ, |cosθ|) = θ` on `θ ∈ (−90°, 90°)`.

**Where the nonlinearity went.** Having pre-processed the accelerometer into
angles, the measurement matrix `H = [1 0 0 0]` is *linear*. So strictly this is
a linear Kalman filter operating on a nonlinearly-transformed measurement, not
an EKF in the textbook sense.

That is a deliberate and very common choice. The alternative — using the raw
3-axis accel vector as the measurement with the true nonlinear `h(x)` and its
Jacobian — is a genuine EKF, costs a 3×3 inverse per update, and buys
essentially nothing here.

The price of the `atan2` shortcut: the transformation is nonlinear, so
Gaussian noise on `a` does not stay Gaussian on `φ`. The distortion is
negligible near level and grows as `cosθ → 0`. Since balance mode sits exactly
there, this is worth knowing — but it degrades *optimality*, not correctness,
and is dominated by the fact that `R` is a conservative guess anyway (reference §5).

`omnis-info.md` §11 calls it an EKF, and that name is kept throughout for
consistency with the project documentation.

---

## 6. Adaptive R — the part that actually matters

§11b calls this "most of what separates a good tilt EKF from a bad one on a
moving robot", and that is not an overstatement.

The measurement model in §5 assumed `a = gravity`. What the accelerometer
actually measures is **specific force**:

```
a_measured = a_gravity + a_linear
```

When the robot accelerates, the derived angle is wrong by about
`atan(a_linear / g)`. At 0.3 g of forward acceleration that is 17° — far larger
than anything the filter's noise model contemplates.

Worse, on a *balancing* robot the linear acceleration is directly caused by the
lean angle. The error is not zero-mean noise; it is systematically correlated
with the state being estimated. Averaging does not help.

**The detector.** We cannot separate gravity from linear acceleration in
general — that is fundamentally impossible with one accelerometer. But we can
detect *when* it is happening, because gravity alone has magnitude exactly 1 g:

```
dev = | |a| - 1g |
```

`dev > 0` proves linear acceleration. (`dev = 0` does *not* prove its absence —
a horizontal acceleration perpendicular to gravity changes direction while
barely changing magnitude. This detector has a real blind spot; see reference §7.5.)

**The response.** Inflate `R` with `dev`:

```
R_eff = R * (1 + gain * (dev - thresh))^2      for dev > thresh
```

Three properties worth stating:

- **Continuous at the threshold.** `excess = 0` gives factor exactly 1. A hard
  switch in `R` would step the Kalman gain and put a visible kick in the
  estimate — mid-balance, that kick is a disturbance the PID reacts to.
- **Quadratic**, so trust falls off fast enough to matter: 9× less trust at
  0.4 g of deviation.
- **Hard rejection past 0.5 g**, where the accel-derived angle carries no usable
  information at all and the honest thing is to propagate on the gyro alone.
  With `τ = 0.198 s` the filter coasts through a few hundred ms of this without
  meaningful drift — which is exactly how long an aggressive manoeuvre lasts.

---

## 7. The singularity, the fold, and why balance mode needs its own frame

> **Correction (2026-09-15).** An earlier revision of this section was titled
> "why it is survivable" and argued that pitch passes cleanly through ±90°. That
> was wrong, and building on it would have made balance mode unstable. The pitch
> estimate is *continuous* at 90°, but it *folds* there — and the fold destroys
> the sign of the lean at exactly the balancing point. Found while designing the
> balance controller, before any of it reached hardware.

Euler angles have a gimbal lock. For this `roll → pitch` sequence it is at
`pitch = ±90°`, where `cos θ = 0`:

```
a_y = sin(φ) * 0 = 0
a_z = cos(φ) * 0 = 0
φ   = atan2(0, 0)          -- undefined
```

**OMNIS balance mode sits exactly at pitch ≈ ±90°.** This is not a corner case;
it is the primary operating point of the feature the whole filter exists for.

### 7.1 Roll dies — that part was always right

Two healthy IMUs at pitch ≈ 90° can report wildly different roll, so the dual-IMU
fault check must not compare Euler angles. Verified: `(roll 30°, pitch 88°)`
versus `(roll −30°, pitch 92°)` is a 60° roll difference but only **3.46°** of real
disagreement about where "down" is. The fusion layer compares gravity *unit
vectors*, which is singularity-free everywhere. See
[`sensor-fusion-reference.md`](sensor-fusion-reference.md) §3.

### 7.2 Pitch folds — the part that was missed

```
pitch = atan2( -a_x, sqrt(a_y² + a_z²) )
```

The second argument is a square root, so it is **never negative**, so `pitch` is
confined to `[−90°, +90°]`. An attitude *past* 90° cannot be represented; it
reflects back:

| Nose-down attitude | accel `(a_x, a_z)` | `pitch` reads |
|---|---|---|
| 85° — leaning 5° back from the front-pair balance point | `(−0.996, +0.087)` | **85.0°** |
| 90° — upright | `(−1.000, 0.000)` | 90.0° |
| 95° — leaning 5° forward | `(−0.996, −0.087)` | **85.0°** |

Leaning 5° forward and 5° back produce **the same number**. A balance controller
driven by it cannot tell which way the robot is falling and pushes the wrong way
on one side of upright. The EKF is damaged too: on the far side of the fold its
`pitch_dot = gyro_y` prediction and its accelerometer update disagree in sign, so
the filter fights itself precisely where it is needed. Verified in
`test_control.c` Case 13.

The earlier revision said pitch is "well-defined and continuous" at 90°. Both
words are true of the function and beside the point for the control problem.

### 7.3 The fix — change frames, not formulas

In balance mode, rotate the body-frame accelerometer and gyro a further 90° about
Y **before** they reach the EKF, so the balancing pose looks *level* to the filter.
Lean then sits near 0°, where pitch is continuous, signed and far from the fold —
and roll is no longer gimbal-locked either.

Both rotations are multiples of 90°, so each is another signed axis permutation,
applied with the same `imu_apply_mount()` used for the sensor mounting. Both
frames put `+X'` toward the chassis top face (the OLED side) and `+Z'` up:

| Pose | Frame | Map | `+pitch'` means |
|---|---|---|---|
| Front pair down (nose down 90°) | `X'=+Z, Y'=+Y, Z'=−X` | `{+3,+2,−1}` | falling toward the top face |
| Rear pair down (nose up 90°) | `X'=+Z, Y'=−Y, Z'=+X` | `{+3,−2,+1}` | falling toward the top face |

Defining both around the top face gives **one sign convention for either pair**:
positive balance-frame pitch always means "falling forward", and `gyro_y'` is
always its rate. The cost is that `Y'` flips in the rear pose, so when balancing
on the rear pair the robot's "left" wheel is RR and its "right" wheel is RL.

Constants: `OMNIS_IMU_BALANCE_FRAME_FRONT_DOWN` and
`OMNIS_IMU_BALANCE_FRAME_REAR_DOWN` in `omnis_imu_mounting.h`.

Verified (Case 13): in the front-down frame 95° reads **+5.0°** and 85° reads
**−5.0°**; the same holds for the rear pair; the gyro maps to `gyro_y' = +` for a
forward fall in both poses; and an EKF seeded at +3° follows the lean through zero
to −3° with the correct sign.

### 7.4 What the flat frame is still good for

The **sign** of the flat-frame pitch survives the fold — 85° and 95° both read
+85°, and both are front-down. So the flat frame still answers "which pair is on
the ground" (`omnis_balance_pair_from_pitch()`), which is exactly the question at
the moment of arming, before the frame switch. It must never be used as the lean.

On switching frames the EKF angle states are meaningless in the new frame, so
both filters are **re-seeded from the accelerometer**. Gyro bias is subtracted
before the frame change, so no bias state needs rotating.

---

## 8. What was deliberately left out

**Full Euler rate coupling.** The exact relation between body rates and Euler
rates is:

```
roll_dot  = g_x + g_y sin(φ) tan(θ) + g_z cos(φ) tan(θ)
pitch_dot = g_y cos(φ) - g_z sin(φ)
```

Note `tan(θ)` in the roll equation — it diverges at pitch = 90°, the same
gimbal lock from the *rate* side. The pitch equation stays finite.

§11b specifies the decoupled form `roll_dot = g_x − b`, `pitch_dot = g_y − b`,
which is the small-roll approximation. Justification for keeping it:

- The error is second order in roll.
- The accelerometer update corrects any resulting bias within τ ≈ 0.2 s.
- Adding `tan(θ)` would introduce a divergence at precisely the operating point,
  requiring its own guard — a net loss in robustness.
- It would also break the exact 2+2 decomposition in §4, forcing a real 4×4.

**Lever-arm compensation.** `a_corrected = a − ω̇×r − ω×(ω×r)`. Needs a measured
position vector `r` from the rotation centre to each IMU. §11b defers it, on the
reasoning that adaptive R already distrusts the accelerometer exactly when this
error is largest. Revisit only if balance quality in hard turns proves
inadequate.

**Scale-factor and misalignment calibration.** §12 covers bias and scale via the
six-position test. Cross-axis misalignment is not modelled — for a hobby-grade
MPU6050 it is below the noise floor established by `R`.
