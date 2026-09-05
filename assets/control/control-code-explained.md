# Control Stack — Code Walkthrough

**Project:** OMNIS · `attitude_ekf` · `imu_fusion` · `pid`
**Scope:** How the modules fit together, and why the non-obvious lines are the
way they are. For equations see the reference docs; for *why those equations*
see [`attitude-ekf-derivation.md`](attitude-ekf-derivation.md).

---

## 1. Files

| File | Lines | Job |
|---|---|---|
| `attitude_ekf.h/.c` | ~470 | per-IMU 4-state attitude filter (§11b) |
| `imu_fusion.h/.c` | ~460 | mounting, fusion, fault, side detection (§11c) |
| `pid.h/.c` | ~330 | inner PID + outer velocity-bias loop (§13b) |
| `test_control.c` | ~370 | 67 host-side assertions against verified values |
| `run_host_tests.sh` | — | `cc` + run. No ESP-IDF, no hardware. |

**Shared properties, by design:**

- Pure arithmetic on caller-owned structs. No `malloc`, no statics, no globals.
- No RTOS primitives, no peripheral access, no blocking, no logging.
- C99, `float` throughout, `<math.h>` the only dependency.
- Every function null-guards its pointers and returns something safe.

They are therefore trivially unit-testable on the host, which is the point:
every number in the reference docs was verified on a Mac before any of it ran on
an ESP32.

---

## 2. Integration — the whole thing in one tick

```c
/* ---- boot, once ---- */
attitude_ekf_config_t cfg;
attitude_ekf_config_defaults(&cfg);

attitude_ekf_t ekf_a, ekf_b;
attitude_ekf_init(&ekf_a, &cfg);
attitude_ekf_init(&ekf_b, &cfg);

/* Determined once on the bench — see sensor-fusion-reference.md §2. */
const imu_mount_t mount_a = IMU_MOUNT_IDENTITY;
const imu_mount_t mount_b = IMU_MOUNT_ROT_Z_180;

/* Assert these; a bad map is invisible at runtime. */
assert(imu_mount_is_valid(&mount_a) && imu_mount_is_right_handed(&mount_a));
assert(imu_mount_is_valid(&mount_b) && imu_mount_is_right_handed(&mount_b));

/* §12.1 — average the gyro at rest, then hand it to the filters so they do not
 * have to rediscover the bias while the robot is trying to balance. */
attitude_ekf_set_gyro_bias(&ekf_a, bias_ax, bias_ay);
attitude_ekf_set_gyro_bias(&ekf_b, bias_bx, bias_by);

/* Seed from one accel sample so we do not start at zero and spend a second
 * settling if the robot is not level at power-on. */
attitude_ekf_seed_from_accel(&ekf_a, a_ax, a_ay, a_az);
attitude_ekf_seed_from_accel(&ekf_b, b_ax, b_ay, b_az);

/* ---- every 500 Hz tick ---- */
float raw_a[3], raw_g[3], body_a[3], body_g[3];

mpu6050_read(&imu_a, raw_a, raw_g);          /* I2C — superloop only, NEVER an ISR */
imu_apply_mount(&mount_a, raw_a, body_a);    /* BEFORE the EKF. non-negotiable. */
imu_apply_mount(&mount_a, raw_g, body_g);
attitude_ekf_step(&ekf_a, body_g[0], body_g[1],
                  body_a[0], body_a[1], body_a[2], dt);

/* ... same for imu_b ... */

imu_fusion_result_t fused;
imu_fusion_combine(&ekf_a, &ekf_b, IMU_DISAGREE_THRESH_RAD, &fused);

if (fused.fault) {
    fault_raise(FAULT_IMU_DISAGREE);          /* -> COM_ENA off, buzzer */
    return;                                   /* before any motor command */
}

/* Balance mode: pitch is the lean angle, roll is gimbal-locked and ignored. */
const float accel_cmd = pid_update(&balance_pid,
                                   target_lean,      /* from the outer loop */
                                   fused.pitch,
                                   body_g[1],        /* RAW gyro, not differentiated */
                                   dt);

/* Outer loop runs at 20 Hz — every 25th tick. */
if ((tick % 25) == 0) {
    target_lean = vel_bias_update(&vbias, commanded_step_rate, 0.05f);
}
```

---

## 3. `attitude_ekf.c` — the lines that matter

### `ekf_wrap_pi()` — `atan2f(sinf(a), cosf(a))`

Looks like an expensive no-op. It is not. The innovation `y = z − x` is an angle
difference; without wrapping, a measurement at +179° against a state at −179°
produces an innovation of **358° instead of 2°**, and the filter slams the
estimate across the whole circle. The `atan2(sin,cos)` form is branch-free and
beats the conditional-subtract loop it replaces.

### `axis_predict()` — the `dt*dt*p11` term

```c
a->P[0][0] = p00 - dt*(p01 + p10) + dt*dt*p11 + q_angle_psd*dt;
```

This is the term several widely-copied tilt filters omit. It says: *the longer
my bias estimate stays uncertain, the faster my angle confidence decays.*
Dropping it makes `P00` shrink too fast, the Kalman gain collapses, and the
filter quietly stops trusting the accelerometer — it looks fine on a bench and
drifts in the field.

### `axis_update()` — saved `p00`, `p01`

```c
const float p00 = a->P[0][0];
const float p01 = a->P[0][1];

a->P[0][0] -= k0 * p00;
a->P[0][1] -= k0 * p01;
a->P[1][0] -= k1 * p00;     /* PRE-update p00, not the value just written */
a->P[1][1] -= k1 * p01;
```

Row 1 must use the **pre-update** row 0. Overwriting row 0 first and reading it
back is the classic silent bug here: the bias covariance shrinks too fast and
the filter stops adapting. It produces no error, no warning, and a plot that
looks almost right.

### `adaptive_r()` — quadratic *in the excess*

```c
const float excess = dev - cfg->accel_dev_thresh;
const float factor = 1.0f + cfg->adapt_r_gain * excess;
return cfg->r_accel * factor * factor;
```

`excess`, not `dev`. At exactly the threshold `excess = 0` and the factor is
exactly 1.0, so `R` is **continuous** across the switch. A discontinuity in `R`
steps the Kalman gain, which steps the estimate, which the balance PID sees as a
disturbance and reacts to — a self-inflicted kick, mid-balance.

### Rejecting `|a| < 1e-3`

A near-zero reading is a dead or unplugged sensor, not free-fall worth
modelling. Feeding `atan2(0,0)` into the filter would inject an arbitrary angle
with full confidence.

---

## 4. `imu_fusion.c` — the lines that matter

### The `acosf` clamp

```c
if (dot >  1.0f) dot =  1.0f;
if (dot < -1.0f) dot = -1.0f;
```

Both vectors are unit length, so `|dot| ≤ 1` mathematically. Float rounding
produces `1.0000001`, `acosf` returns **NaN**, and `NaN > threshold` evaluates
**false** — silently disabling the IMU fault check. A NaN that switches off a
safety interlock is exactly what this clamp prevents.

### `imu_apply_mount()` — permutation, not matrix

Both modules lie flat on the same board, so every realistic mounting differs by
a multiple of 90°: a signed axis permutation. Bit-exact, three loads instead of
nine multiply-accumulates, and no accumulated float error. See
[`sensor-fusion-reference.md`](sensor-fusion-reference.md) §2 for the bench
procedure to determine yours — and for why getting it wrong makes two healthy
sensors read 20° apart at boot.

### Out-of-range map entry emits zero, does not read out of bounds

```c
if (idx < 0 || idx > 2) { out[i] = 0.0f; continue; }
```

A zeroed axis shows up immediately as a stuck attitude. An out-of-bounds read is
undefined behaviour that may look fine on the bench and fail in flight.

### Wrapped averaging in `imu_fusion_combine()`

```c
out->roll = ra + ekf_wrap_pi(rb - ra) * (wb / wsum);
```

Averaging relative to A rather than `(a+b)/2`. Naively averaging +179° and −179°
gives 0°, which is 180° wrong.

### Fault computed *before* the fuse

The caller always has a disagreement number even when it throws the fused value
away. A fault you cannot quantify is a fault you cannot diagnose.

---

## 5. `pid.c` — the lines that matter

### `pid_config_defaults()` sets every gain to **zero**

Deliberate, per §13d. A zero-gain controller does nothing, which is visible and
safe. A guessed-gain controller looks like it works and falls over. The gains
depend on mass, CG height and wheel radius that this code does not know.

### `D = -kd * measurement_rate`

The minus sign is a dropped derivation, not an error:
`d(error)/dt = d(setpoint)/dt − d(measurement)/dt`, and the setpoint moves
slowly at 500 Hz, so the first term goes. Taking the rate from the **raw gyro**
(§13b) also avoids amplifying EKF quantisation noise by a `1/dt` = 500 factor.

### Conditional integration — four steps, in this order

```c
float cand = pid->integral + pid->cfg.ki * error * dt;   /* 1. candidate  */
const float tentative = p_term + cand + d_term;          /* 2. tentative  */
if (tentative > out_max) blocked = (cand > pid->integral);
else if (tentative < out_min) blocked = (cand < pid->integral);
if (!blocked) pid->integral = cand;                      /* 3. commit?    */
const float raw = p_term + pid->integral + d_term;       /* 4. recompute  */
```

Freezing beats clamping. A clamped integrator still winds to its cap while the
actuator is pinned, and that energy comes back out as overshoot at exactly the
moment the robot is recovering. Verified: frozen at exactly `0.0` through six
seconds of saturation, free on the first tick after the error reverses.

### `vel_bias_update()` — the sign

```c
const float target = -vb->cfg.gain * vb->integral + vb->cfg.trim_rad;
```

Forward drive effort commands a **backward** lean. Leaning back is how a
balancing robot decelerates. Wrong sign = accelerates until it falls over, and
it is the most common bug in this loop.

---

## 6. Building and testing

```bash
./run_host_tests.sh
```

Compiles with `-Wall -Wextra -Werror` and runs 67 assertions covering: adaptive-R
inflation, steady-state covariance and gain, gyro-bias recovery, step response,
inverse-covariance fusion, the disagreement metric (including the balance-mode
gimbal-lock case), mounting remap and validation, side detection with the rest
gate, PID arithmetic, anti-windup under real saturation, and the outer loop.

Every expected value was computed independently in Python first. If a refactor
breaks one, the number in the reference doc is the authority.

### Adding to an ESP-IDF component

```cmake
idf_component_register(
    SRCS "attitude_ekf.c" "imu_fusion.c" "pid.c"
    INCLUDE_DIRS "."
    REQUIRES ""            # nothing. these modules need no IDF component.
)
```

`REQUIRES` is empty and should stay that way. The moment these files need an IDF
component, they have stopped being host-testable.
