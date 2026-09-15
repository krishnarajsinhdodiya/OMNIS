# OMNIS — Control & Estimation

Sensor fusion, attitude EKF, and balance PID for the OMNIS platform. Companion
to [`../kinematics/`](../kinematics/), which covers the mecanum drivetrain.

Implements `omnis-info.md` §11 (EKF and dual-IMU fusion), §12 (calibration
hooks) and §13 (balance control).

## Layout

| File | What it is |
|---|---|
| [`attitude-ekf-reference.md`](attitude-ekf-reference.md) | State model, matrices, tuning, verified numbers |
| [`attitude-ekf-derivation.md`](attitude-ekf-derivation.md) | Why the filter has this form; the singularity; what was left out |
| [`sensor-fusion-reference.md`](sensor-fusion-reference.md) | Mounting, fusion, the 15° fault, side detection |
| [`pid-reference.md`](pid-reference.md) | Cascaded architecture, anti-windup, tuning procedure |
| [`control-code-explained.md`](control-code-explained.md) | Integration example and line-by-line walkthrough |
| `attitude_ekf.h/.c` | Per-IMU 4-state attitude filter |
| `imu_fusion.h/.c` | Mounting remap, fusion, fault, side detection |
| `pid.h/.c` | Inner PID + outer velocity-bias loop |
| `omnis_imu_mounting.h` | **As-built** chassis frame, corner labelling and IMU mount constants |
| `test_control.c` | 97 host-side assertions |
| `run_host_tests.sh` | Build and run them — no ESP-IDF, no hardware |

## As-built chassis facts

Front edge is the OLED / button end. Body frame is right-handed, **+X forward,
+Y left, +Z up** — identical to the kinematics convention.

| | Corner | I2C | Faces | Mount |
|---|---|---|---|---|
| IMU A | front-left | `0x68` | forward | `IMU_MOUNT_IDENTITY` |
| IMU B | rear-right | `0x69` | rearward | `IMU_MOUNT_ROT_Z_180` |

Balance mode tips about the body **Y** axis, so the lean angle is **pitch** and
roll is gimbal-locked and unused. The grounded pair is FL+FR or RL+RR.

**Positive pitch = nose DOWN**, opposite to the aerospace convention. It falls
out of `pitch = atan2(−ax, hypot(ay,az))` and is asserted in the tests.

See [`omnis_imu_mounting.h`](omnis_imu_mounting.h) for the chassis diagram.

## Verify

```bash
./run_host_tests.sh
```

## Four things that will bite

1. **Correct the IMU mounting before the EKF, not after.** OMNIS's two modules
   *are* mounted antiparallel (confirmed 2026-09-05) — uncorrected, that makes
   two healthy sensors read 20° apart and the fault fires at boot, every boot.
   Constants in `omnis_imu_mounting.h`; derive them for real with
   `imu_mount_resolve()` rather than trusting a silkscreen arrow.
   → `sensor-fusion-reference.md` §2, §2.5
2. **Never compare Euler angles to detect IMU disagreement.** Balance mode sits
   at pitch ≈ 90° where roll is gimbal-locked; a 60° roll difference there is
   only 3.46° of real disagreement. Compare gravity vectors. → §3
3. **The outer velocity-bias loop's sign.** Forward drive effort must command a
   *backward* lean. Wrong sign accelerates until it falls over.
   → `pid-reference.md` §4
4. **Never use the flat-frame pitch as the balance lean angle.** At the balance
   point (±90°) `atan2(-ax, hypot(ay,az))` folds: nose-down 85° and 95° both read
   85°, so which way the robot is falling is lost. Balance mode rotates into a
   frame where the pose is level (`OMNIS_IMU_BALANCE_FRAME_*`).
   → `attitude-ekf-derivation.md` §7

## Units

Radians and rad/s throughout. Accelerometer in **g**, not m/s² — the adaptive-R
gate keys off `|a|` deviating from 1.0, so m/s² silently rejects every update.
Convert once at the driver boundary.

Body frame matches the kinematics convention exactly: **+X forward, +Y left,
+Z up**.

## Deliberately not here

- Gyro/accel calibration routines (§12) — these modules *consume* calibration
  via `attitude_ekf_set_gyro_bias()`; producing it is a driver concern.
- Any I2C or peripheral access. Pure arithmetic, host-testable.
- Numeric PID gains. §13d declines to guess them without mass, CG height and
  wheel radius; `pid_config_defaults()` sets them to zero on purpose.
- Lever-arm compensation. §11b defers it to v2; needs measured IMU positions.
