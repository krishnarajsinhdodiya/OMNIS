# Dual-IMU Sensor Fusion — Reference

**Project:** OMNIS · two MPU6050s at diagonally opposite corners
**Scope:** Mounting reconciliation, fusion, disagreement fault, side detection.
**Implements:** `omnis-info.md` §11c

---

## 1. What this layer does

Three jobs, deliberately kept separate because they fail differently:

| Job | Function | Fails how |
|---|---|---|
| **Reconcile mounting** | `imu_apply_mount()` | silently, at boot, looks like a broken sensor |
| **Fuse** | `imu_fusion_combine()` | gracefully — degrades to one IMU |
| **Fault** | disagreement metric | must be loud; it gates the motors |
| **Side detect** | `imu_detect_side()` | returns `UNKNOWN` rather than guessing |

Order matters: **mounting must be corrected before the EKFs run**, not after.

---

## 2. Mounting reconciliation — read this first

The two modules sit at opposite corners of a hand-routed board. There is no
guarantee their packages point the same way; one is very often rotated 180°
about Z relative to the other, because that is what makes the I2C run reach.

If that rotation is not undone before the EKF sees the data, the two filters
converge to attitudes differing by the mounting rotation, the disagreement
metric reads large, and **the 15° fault fires at boot, every boot, on two
perfectly healthy sensors.**

Verified: an un-corrected 180° Z mounting on a chassis at 10° of roll reads as
**20.0°** of disagreement — comfortably over the threshold.

> This is the single most likely reason a correct dual-IMU implementation
> refuses to arm. Check it before suspecting the filter.

### Encoding

Both modules lie flat on the same board, so every realistic mounting differs by
a multiple of 90°. That is exactly a **signed axis permutation** — a swap and a
sign flip. No trig, no rotation matrix, bit-exact, three loads instead of nine
multiply-accumulates.

```
map[i] selects which sensor axis feeds body axis i, 1-based so sign is usable:
  +1/-1 = sensor X    +2/-2 = sensor Y    +3/-3 = sensor Z

  body_x = sign(map[0]) * sensor[ |map[0]| - 1 ]
```

Body frame matches the kinematics convention exactly
(`mecanum-kinematics-reference.md` §1): **+X forward, +Y left, +Z up.**

### Presets

| Macro | `map` | Meaning |
|---|---|---|
| `IMU_MOUNT_IDENTITY` | `{+1,+2,+3}` | axes already match the body frame |
| `IMU_MOUNT_ROT_Z_90` | `{+2,-1,+3}` | rotated 90° CCW seen from above |
| `IMU_MOUNT_ROT_Z_180` | `{-1,-2,+3}` | **the common opposite-corner case** |
| `IMU_MOUNT_ROT_Z_270` | `{-2,+1,+3}` | rotated 270° CCW |
| `IMU_MOUNT_FLIP_X` | `{+1,-2,-3}` | mounted on the underside |
| `IMU_MOUNT_FLIP_X_ROT_Z_180` | `{-1,+2,-3}` | underside *and* rotated |

Apply to **both** the accelerometer and the gyro. Both are vectors in the sensor
frame; a frame change does not care what they measure.

### Two validity checks worth running once at boot

- `imu_mount_is_valid()` — every entry in ±1..±3 and all three axes distinct.
  A typo like `{+1,+1,+3}` collapses two body axes onto one sensor axis and
  produces an attitude that looks plausible but is wrong in a way that is very
  hard to see on a plot.
- `imu_mount_is_right_handed()` — determinant +1. A mirrored map such as
  `{+2,+1,+3}` is a valid permutation but inverts the sign of every rotation, so
  the gyro and the accel-derived angle disagree permanently and the EKF fights
  itself forever.

### OMNIS as-built — confirmed from the board

Confirmed by the builder against the assembled board, 2026-09-05. Front edge is
the OLED / button end.

| IMU | I2C | Corner | Faces | Mount constant |
|---|---|---|---|---|
| **A** | `0x68` | front-left | **forward** (+X) | `IMU_MOUNT_IDENTITY` |
| **B** | `0x69` | rear-right | **rearward** (−X) | `IMU_MOUNT_ROT_Z_180` |

The two modules are mounted **antiparallel** — exactly the case this section
warns about, now a confirmed property of the hardware rather than a suspicion.
Both are component-side up, so `+Z` is unchanged and the correction is a pure
180° rotation about Z: `{-1, -2, +3}`.

Recorded in [`omnis_imu_mounting.h`](omnis_imu_mounting.h), which also carries
the chassis diagram and the corner labelling that follows from the front edge
(FL = front-left, the same labels used in `mecanum-kinematics-reference.md` §4).

The two IMUs sit on the **FL/RR diagonal**. Under the old X-drive roller layout
that diagonal was also a roller-handedness pair; under the as-built lateral
parallel layout it is not — the `δ = -1` pair is the LEFT SIDE, FL and RL.
Nothing here ever depended on the coincidence, and it is mentioned only because
an earlier edition of this paragraph asserted it.

These constants read the marked arrow on each module as its sensor **+X** axis.
**Do not rely on that reading — derive it.** `imu_mount_resolve()` recovers the
descriptor from two poses with no interpretation required; see §2.5 below.

A wrong *global* orientation (both IMUs consistently rotated) does **not** trip
the fault, because the two still agree with each other perfectly. It silently
changes which body axis is the lean axis, and the balance controller then drives
the wrong one. **Nothing downstream can catch this class of error** — not the
disagreement fault, not the EKF, not the PID. Deriving the mounting is the only
defence, and it costs two minutes.

### 2.5 Deriving the mounting — `imu_mount_resolve()`

Two poses, per IMU, no arrow-reading and no raw-number interpretation:

| Step | Pose | What it pins down |
|---|---|---|
| 1 | Chassis **flat and level**, stationary. Average a few hundred accel samples. | body **+Z** — the accelerometer reads the *up* direction, so the axis reading ≈ +1 g is body +Z, sign included |
| 2 | Chassis tipped **nose-down** (front edge lowered) 15–75°, held still. Average again. | body **+X** — of the two axes that are not Z, the one that moved away from zero is X, and it must read *negative* |
| 3 | — | body **+Y** is *forced* by right-handedness, not measured. With X and Z fixed there is exactly one Y sign giving determinant +1 |

```c
imu_mount_t mount;
if (imu_mount_resolve(level_sample, nose_down_sample, &mount)) {
    /* authoritative — use this over any assumed constant */
}
```

**Why two poses and not one.** A single tilted sample is genuinely ambiguous. A
sensor reading of `(0.5, 0, 0.866)` is consistent *both* with "body X is sensor
X at 30° of tilt" *and* with "body X is sensor Z at 60° of tilt" — both produce
the correct sign pattern and both are right-handed. The level pose pins body +Z
independently of any tilt angle, and only then does the nose-down pose become
unambiguous.

**It fails rather than guesses.** Returns `false` and leaves the output untouched
if the "level" pose was tilted more than ~37° (so it was not level) or the
nose-down tilt was under ~12° (so X and Y are not separated). A wrong answer here
is much worse than no answer.

**Verified exhaustively:** all **24** right-handed signed-permutation mountings
synthesised and recovered exactly, plus every rejection case. Case 12 in
`test_control.c`.

### Pitch sign — counter-intuitive, so it is asserted

With the `atan2` measurement model in `attitude_ekf.c`:

| Attitude | accel (body) | pitch |
|---|---|---|
| level | `(0, 0, +1.000)` | `+0.0°` |
| nose down 30° | `(−0.500, 0, +0.866)` | **`+30.0°`** |
| nose down 60° | `(−0.866, 0, +0.500)` | **`+60.0°`** |
| nose down 90° | `(−1.000, 0, +0.000)` | **`+90.0°`** |

**Positive pitch means nose DOWN** — the opposite of the aerospace convention.
This is a property of `pitch = atan2(−ax, hypot(ay, az))`, not a bug, and it is
why `omnis_balance_pair_from_pitch()` maps `pitch ≥ 0` to the *front* pair being
grounded. Someone will eventually try to "fix" this sign; the test asserts it.

### How to determine yours

No measurement needed — a two-minute bench procedure:

1. Lie the chassis flat, powered, stationary. Print raw accel from both IMUs.
   Both should read `az ≈ +1 g`, `ax ≈ ay ≈ 0`. If one reads `az ≈ −1 g`, it is
   mounted on the underside.
2. Tip the chassis **nose-down** ~30°. Both should show `ax` going **negative**
   by roughly the same amount. If one goes positive, its X is reversed.
3. Tip **left-side-down** ~30°. Both should show `ay` going **positive**
   (+Y is left). If one goes negative, its Y is reversed.
4. If X and Y are *swapped* rather than reversed, it is a 90° or 270° case —
   step 2 will move `ay` instead of `ax`.

Pick the preset that makes both IMUs agree, then confirm: the disagreement
metric should read **< 2°** with the chassis static and level.

---

## 3. The disagreement metric — gravity vectors, not Euler angles

§11c calls for a fault when the two IMUs "disagree beyond ~15°". *What* to
compare is where this gets subtle.

**Do not compare Euler angles.** At pitch ≈ ±90° — which is exactly where
balance mode lives — roll is gimbal-locked and undefined, so two healthy sensors
can report wildly different roll values and both be right.

**Compare gravity directions instead.** Reconstruct each IMU's estimate of
"down" as a unit vector and take the angle between them:

```
u = ( -sin(pitch),  sin(roll)*cos(pitch),  cos(roll)*cos(pitch) )

disagreement = acos( clamp( u_A · u_B , -1, +1 ) )
```

This is the exact inverse of the EKF measurement model, so it is consistent by
construction, and it is singularity-free at every attitude.

> The `clamp` is not cosmetic. Both are unit vectors so `|dot| ≤ 1`
> mathematically, but float rounding produces `1.0000001`, `acosf` returns NaN,
> and `NaN > threshold` evaluates **false** — silently disabling the IMU fault
> check. A NaN that switches off a safety interlock is precisely the failure
> this clamp exists to prevent.

### Verified disagreement values

| IMU A (roll, pitch) | IMU B (roll, pitch) | disagreement | verdict |
|---|---|---|---|
| 0°, 0° | 0°, 0° | 0.0000° | ok |
| 0°, 0° | 10°, 0° | 10.0000° | ok |
| 0°, 0° | 0°, 10° | 10.0000° | ok |
| 0°, 0° | 15°, 0° | 15.0000° | at threshold |
| 0°, 0° | 20°, 0° | 20.0000° | **FAULT** |
| 0°, 0° | 10°, 10° | 14.1060° | ok |
| 0°, 88° | 0°, 92° | 4.0000° | ok |
| **30°, 88°** | **−30°, 92°** | **3.4639°** | **ok** |

That last row is the whole argument. In balance mode a **60° roll difference is
only 3.46° of real disagreement**, because roll means nothing there. A naive
`|roll_A − roll_B|` test would fault continuously on a healthy robot.

### Balance mode changes frames — the metric does not care

While balancing, the firmware rotates both IMUs' vectors into a balance frame
where the balancing pose is level
([`attitude-ekf-derivation.md`](attitude-ekf-derivation.md) §7.3), because the
flat-frame pitch folds at ±90°. The disagreement metric needs no change: both
gravity vectors pass through the *same* rotation, and a rotation preserves the
angle between two vectors. The 15° threshold means exactly the same thing in
either frame.

### Threshold

```
IMU_DISAGREE_THRESH_RAD = 0.2617993878   /* 15.0 deg, per §11c */
```

§11c is explicit that beyond this it is "a bad mount, sensor, or cable, not
something to average through" — flag the fault and fail toward the link-loss
failsafe (disable steppers) rather than balance on an untrustworthy number.

---

## 4. Inverse-covariance weighted fusion

```
x_fused = ( x_A/P_A + x_B/P_B ) / ( 1/P_A + 1/P_B )
P_fused = 1 / ( 1/P_A + 1/P_B )
```

This is the maximum-likelihood combination of two independent Gaussian
estimates: weight each by its confidence, and the result is more confident than
either.

### Verified

| A | P_A | B | P_B | fused | P_fused | σ |
|---|---|---|---|---|---|---|
| 2.00° | 1e-4 | 3.00° | 1e-4 | 2.5000° | 5.000e-5 | 0.4051° |
| 2.00° | 1e-4 | 3.00° | 4e-4 | 2.2000° | 8.000e-5 | 0.5125° |
| 2.00° | 1e-4 | 10.00° | 1e-2 | 2.0792° | 9.901e-5 | 0.5701° |

Row 1: equal confidence gives the midpoint. Row 2: B four times less certain,
result pulled 80% toward A. Row 3: B nearly worthless, fused sits essentially on
A — the useful behaviour when one IMU is being shaken and the other is not.

### Honest caveat

**The two IMUs are not independent.** They observe the same physical motion,
share a chassis, and see a correlated vibration environment. Treating them as
independent makes `P_fused` **optimistic** — the real uncertainty is larger.

This is acceptable here only because `P_fused` is *not fed back into anything*:
it is reported for diagnostics, and the fault check is a separate geometric
test that does not use it. Do not start using `P_fused` as a calibrated
uncertainty. If that is ever needed, covariance intersection gives a
conservative bound instead.

### Wrapped averaging

Roll and pitch are angles. Averaging +179° and −179° naively gives 0°, which is
180° wrong. The implementation averages *relative to A*:

```
fused = a + wrap_pi(b - a) * (w_b / (w_a + w_b))
```

### Degraded modes

| Situation | Behaviour |
|---|---|
| One filter `NULL` | return the other, `fault = false` — nothing to disagree with |
| Both `NULL` | zeroed estimate, `fault = true` |
| A variance ≤ 0 | fall back to plain average — a corrupted covariance must not divide by zero |

A single-IMU estimate beats no estimate; the caller's fault path decides whether
to keep driving.

---

## 5. Side detection

§11c needs "which side is down" for the active-wheel-pair remapping in §1.

```
imu_side_t imu_detect_side(ax, ay, az, gyro_mag)
```

### Rest gate — both conditions required

| Gate | Value | Why |
|---|---|---|
| `\|gyro\| <` | 3 °/s | rotating platform has no static gravity reference |
| `\| \|a\| − 1g \| <` | 0.10 g | accelerating platform likewise |

Steady free-fall has low gyro but no usable gravity vector; a constant-rate
rotation can hold `|a|` near 1 g while pointing nowhere useful. Both gates are
load-bearing.

### Dominance requirement

The largest-magnitude axis wins, but only if it exceeds **0.80 g** — within ~37°
of an axis. On a 45° corner two axes read ~0.71 g each and there is no correct
answer; returning `UNKNOWN` beats picking one and having the balance controller
drive the wrong wheel pair.

### Verified

| Input | Result |
|---|---|
| flat, at rest | `Z_UP` |
| inverted, at rest | `Z_DOWN` |
| nose-up balance pose (`ax = −1 g`) | `X_DOWN` |
| flat but spinning at 1 rad/s | `UNKNOWN` |
| flat but `\|a\| = 1.5 g` | `UNKNOWN` |
| 45° corner | `UNKNOWN` |

### Call it rarely

§11c: evaluate **only at mode-entry or after a detected flip, never
continuously** — a live side-detector fights the balance controller mid-balance.
The rest gate enforces this defensively (a caller that ignores the advice still
cannot get a bad answer while manoeuvring), but the gate is a backstop, not a
licence to poll it every tick.

---

## 6. Call order

```
per IMU, every tick:
    raw accel, gyro  (I2C — superloop only, never an ISR)
        |
    imu_apply_mount()          <-- BEFORE the EKF. non-negotiable.
        |
    attitude_ekf_step()
        |
    +---> imu_fusion_combine()  -> fused angle + fault
```

Side detection sits outside this path, called at mode entry only.
