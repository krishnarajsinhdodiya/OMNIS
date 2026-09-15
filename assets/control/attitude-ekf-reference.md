# Attitude EKF — Quick Reference

**Project:** OMNIS · dual-MPU6050 attitude estimation
**Scope:** State model, matrices, tuning, and verified numbers. No derivation —
see [`attitude-ekf-derivation.md`](attitude-ekf-derivation.md) for that.
**Implements:** `omnis-info.md` §11b, §11d

---

## 1. What this filter estimates

Per IMU, four states:

| Symbol | State | Unit |
|---|---|---|
| `roll` | rotation about body +X | rad |
| `pitch` | rotation about body +Y | rad |
| `bias_roll` | gyro X zero-offset | rad/s |
| `bias_pitch` | gyro Y zero-offset | rad/s |

Yaw is **not estimated**. §11a is explicit: the balance controller does not use
it, so it is left to drift rather than paying for a magnetometer and a full
quaternion AHRS.

**Unit rule:** radians and rad/s internally, seconds for `dt`, and the
accelerometer in **g — not m/s²**. The adaptive-R gate keys off `|a|` deviating
from `1.0`, so feeding m/s² silently disables it (every sample looks like 9.8 g
of deviation and every update is rejected). Convert once at the driver boundary.

---

## 2. Model

### Process

```
roll_k   = roll_{k-1}  + (gyro_x - bias_roll)  * dt
pitch_k  = pitch_{k-1} + (gyro_y - bias_pitch) * dt
bias_k   = bias_{k-1}                              (slow random walk)
```

### Measurement — accelerometer as a gravity reference

```
roll_meas  = atan2( ay, az )
pitch_meas = atan2( -ax, sqrt(ay^2 + az^2) )
```

### Matrices, per axis

```
      ┌         ┐            ┌                      ┐
F  =  │  1  -dt │      Q  =  │ q_angle*dt      0    │      H = [ 1  0 ]
      │  0   1  │            │     0      q_bias*dt │
      └         ┘            └                      ┘
```

### Covariance propagation, expanded

```
P00' = P00 - dt*(P01 + P10) + dt^2 * P11 + q_angle*dt
P01' = P01 - dt*P11
P10' = P10 - dt*P11
P11' = P11 + q_bias*dt
```

The `dt^2 * P11` term couples bias uncertainty into angle uncertainty. Several
widely-copied tilt filters drop it; doing so makes the filter over-confident and
it slowly stops trusting the accelerometer at all.

### Scalar update

```
S  = P00 + R
K  = [ P00/S , P10/S ]
y  = wrap_pi( z - angle )
x += K * y
P -= K * H * P
```

Two sequential scalar updates (roll, then pitch) are **exactly** equivalent to
one joint 2×2 update, because `R` is diagonal. No matrix inverse anywhere.

---

## 3. The 4-state filter is two 2-state filters

`roll` couples only to `bias_roll`, `pitch` only to `bias_pitch`, and `R` is
diagonal. So the 4×4 filter is block-diagonal and decomposes **exactly** into two
independent 2-state filters. This is an identity, not an approximation — see
[derivation §4](attitude-ekf-derivation.md).

Cost: 4×4 needs a 4×4 covariance (16 floats) and a 2×2 inverse.
Two 2-state filters need 2×(4 floats) and no inverse at all.

`attitude_ekf.c` stores it the second way.

---

## 4. Adaptive measurement trust

The accelerometer measures **specific force**, not gravity. Under linear
acceleration the derived angle is wrong by roughly `atan(a_linear / g)` — and on
a balancing robot that error is *correlated with the lean angle being measured*,
so it is systematic, not noise that averages out.

The cheap, reliable detector is `|a|` deviating from 1 g:

```
dev = | |a| - 1.0 |                                     [g]

dev <= 0.20          ->  R = R_accel                    (trust normally)
0.20 < dev < 0.50    ->  R = R_accel * (1 + 10*(dev-0.20))^2
dev >= 0.50          ->  skip the update entirely       (gyro-only this tick)
```

Quadratic **in the excess**, so `R` is continuous at the threshold. A step change
in `R` produces a visible kick in the estimate — exactly what you do not want
mid-balance.

### Verified inflation table

| `\|a\|` (g) | `dev` | `R_eff` | factor |
|---|---|---|---|
| 1.00 | 0.00 | 0.010000 | 1.0 |
| 1.20 | 0.20 | 0.010000 | 1.0 |
| 1.30 | 0.30 | 0.040000 | 4.0 |
| 1.40 | 0.40 | 0.090000 | 9.0 |
| 1.50 | 0.50 | — | **rejected** |

---

## 5. Tuning constants

`Q` values are **continuous-time power spectral densities**; the filter
multiplies by `dt` internally, so the tuning does not change if the loop rate
changes.

| Name | Value | Unit | Where it came from |
|---|---|---|---|
| `EKF_Q_ANGLE_PSD` | `5.0e-4` | rad²/s | tuned for a 0.8 Hz crossover (below) |
| `EKF_Q_BIAS_PSD` | `1.0e-7` | rad²/s³ | MPU6050 in-run bias drift, generously |
| `EKF_R_ACCEL` | `1.0e-2` | rad² | ≈5.7° 1σ — vibration-dominated, not datasheet |

**Why `R` is 100× the datasheet number.** MPU6050 accel noise density is
400 µg/√Hz, which over a ~100 Hz bandwidth is ~4 mg RMS ≈ 4 mrad of angle noise,
i.e. `R ≈ 1.6e-5`. That is the number on a *bench*. On a chassis with four
stepper motors it is optimistic by orders of magnitude. `1e-2` is a deliberately
conservative starting point; lower it only if bench data justifies it.

### Resulting filter behaviour at 500 Hz

| Quantity | Value |
|---|---|
| Steady-state `K` | `[0.009978, -0.000141]` |
| Steady-state `P` | `[[9.978088e-05, -1.407140e-06], [-1.407140e-06, 7.091040e-06]]` |
| σ angle | **0.5723°** |
| σ bias | 0.1526 °/s |
| Equivalent complementary time constant τ | **0.198 s** |
| Accel/gyro crossover | **0.80 Hz** |

Below 0.8 Hz the accelerometer dominates; above it, the gyro does. That is the
right neighbourhood for a balancer: fast enough to correct gyro drift before it
matters, slow enough to ignore per-step vibration.

**Sanity check if you retune:** τ ≈ `dt·(1−K₀)/K₀`. If τ lands above ~1 s the
filter will visibly lag real tilt; below ~0.05 s it will chase vibration.

---

## 6. Verified numbers

All computed independently in Python, then re-checked against the C in
`test_control.c` (`./run_host_tests.sh` — 97 assertions, 0 failures).
Conditions: `dt = 0.002`, `q_angle = 5e-4`, `q_bias = 1e-7`, `R = 1e-2`.

### Case 1 — static and level, cold start

Converges to the steady-state `P` in §5. Angle and bias both stay exactly 0.

### Case 2 — un-modelled constant gyro bias, 0.02 rad/s, held level

| t (s) | angle error (°) | bias estimate (rad/s) | % of truth |
|---|---|---|---|
| 1.0 | 0.0172 | 0.018474 | 92.4 |
| 2.0 | 0.0069 | 0.019394 | 97.0 |
| 5.0 | 0.0025 | 0.019785 | 98.9 |
| 10.0 | 0.0012 | 0.019897 | 99.5 |
| 50.0 | 0.0002 | 0.019981 | 99.9 |

Bias converges *much* more slowly than angle — over a second to reach 92%. This
is why `attitude_ekf_set_gyro_bias()` exists: preload the boot calibration
(§12.1) rather than making the filter rediscover it while the robot is trying
to balance.

### Case 3 — step tilt to 30°, gyro silent

| t (s) | angle (°) |
|---|---|
| 0.100 | 12.2134 |
| 0.200 | 19.6100 |
| 0.400 | 26.7949 |
| 1.000 | 30.6913 |
| 2.000 | 30.7802 |

63% at **0.190 s**, 95% at 0.506 s — matching the predicted τ = 0.198 s.

The small overshoot past 30° (peak 30.78°, ~2.6%) is the bias state briefly
absorbing part of the step, then releasing it. Expected, not a defect: a step in
angle with no gyro motion is physically impossible, so the filter is right to
attribute some of it to bias before the evidence accumulates.

---

## 7. Known limitations

Carried forward from §11b, plus what fell out of implementation:

1. **No lever-arm compensation.** Both IMUs sit at corners, not the CG, so yaw
   rate and hard acceleration add a centripetal/tangential term the model
   ignores. Full correction needs `a_corrected = a − ω̇×r − ω×(ω×r)` and a
   measured `r` per IMU. §11b says skip for v1, and the adaptive-R gate already
   distrusts the accelerometer exactly when this error is largest.

2. **At pitch ≈ ±90° roll is meaningless AND pitch folds.** Textbook Euler
   gimbal lock kills roll, which is why the fusion layer compares gravity
   *vectors* rather than Euler angles
   ([`sensor-fusion-reference.md`](sensor-fusion-reference.md) §3). Pitch is
   continuous there but confined to ±90°, so it reflects: see item 3. OMNIS
   balance mode lives at exactly this attitude, so **balance mode runs the EKF in
   a rotated frame where the balancing pose is level** —
   [`attitude-ekf-derivation.md`](attitude-ekf-derivation.md) §7.3.

3. **Pitch cannot exceed ±90°, and for OMNIS that is the balance point.**
   `atan2(-ax, hypot(ay,az))` has that range by construction, so an attitude past
   vertical reflects back: nose-down 95° reads 85°, the same as 85°. *An earlier
   revision called this irrelevant for a balancer ("past 90° it has fallen").
   That was wrong* — the robot balances *at* 90°, so leaning 5° forward and 5°
   back were indistinguishable. Fixed by the balance-mode frame
   (`OMNIS_IMU_BALANCE_FRAME_*` in `omnis_imu_mounting.h`); verified in
   `test_control.c` Case 13.

4. **Decoupled Euler rates.** `pitch_dot = gyro_y` is exact only at `roll = 0`;
   the full relation is `pitch_dot = gyro_y·cos(roll) − gyro_z·sin(roll)`. §11b
   specifies the simple form and it is what is implemented. The error is second
   order in roll and the accelerometer update corrects it within τ.

5. **The 1 g gate cannot see constant-magnitude errors.** A steady coordinated
   turn keeps `|a|` at 1 g while the gravity direction is wrong. Only lever-arm
   compensation or a velocity reference fixes that.
